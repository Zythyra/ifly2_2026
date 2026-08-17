#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "line_follow/line_follow.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

class TrafficLightRecognizer
{
public:
    enum Direction
    {
        DIR_NONE = 0,
        DIR_LEFT,
        DIR_STRAIGHT,
        DIR_RIGHT,
        DIR_UNKNOWN
    };

    TrafficLightRecognizer()
        : nh_(),
          pnh_("~"),
          route_started_(false),
          stable_frames_(0),
          stable_count_(0),
          last_direction_(DIR_NONE)
    {
        // =========================
        // 摄像头
        // =========================
        pnh_.param("camera_index", camera_index_, 0);
        pnh_.param("camera_width", camera_width_, 640);
        pnh_.param("camera_height", camera_height_, 480);

        // 摄像头画面左右镜像翻转
        pnh_.param("mirror_image", mirror_image_, true);

        // =========================
        // ROS
        // =========================
        pnh_.param<std::string>(
            "cmd_vel_topic",
            cmd_vel_topic_,
            std::string("/cmd_vel"));

        pnh_.param<std::string>(
            "direction_topic",
            direction_topic_,
            std::string("/traffic_light/direction"));

        // =========================
        // 全图识别
        //
        // 默认 0,0,1,1 = 整张 640x480。
        // 不依赖红绿灯固定位置。
        // =========================
        pnh_.param("roi_x", roi_x_, 0.0);
        pnh_.param("roi_y", roi_y_, 0.0);
        pnh_.param("roi_w", roi_w_, 1.0);
        pnh_.param("roi_h", roi_h_, 1.0);

        // =========================
        // HSV 绿色
        // =========================
        pnh_.param("green_h_min", green_h_min_, 35);
        pnh_.param("green_h_max", green_h_max_, 95);
        pnh_.param("green_s_min", green_s_min_, 120);
        pnh_.param("green_v_min", green_v_min_, 120);

        // 负形箭头提取参数：
        // 实测中 LED 箭头中心因为过曝，在 green_mask 里反而是黑色，
        // 周围绿色光晕是白色。
        // 所以不再做“更严格的绿色 core”，改成：
        // 先对 green_mask 做闭运算得到绿色支撑区域，
        // 再用 support - green_mask 提取其中的黑色负形箭头。
        pnh_.param("negative_close_kernel", negative_close_kernel_, 17);

        // 对负形箭头做一次轻微开运算，去掉小碎点
        pnh_.param("negative_open_kernel", negative_open_kernel_, 3);

        // 真实箭头中心存在过曝泛白：
        // V 很高、S 较低。负形候选必须同时满足“高亮白芯”，
        // 这样可以排除绿色椅子内部的暗孔、阴影等黑色负形。
        pnh_.param("bright_core_v_min", bright_core_v_min_, 175);
        pnh_.param("bright_core_s_max", bright_core_s_max_, 150);

        // 最终 arrow_mask 中面积小于该值的白色连通域直接删除，
        // 主要清掉环境反光造成的零散白点。
        pnh_.param("negative_component_min_area",
                   negative_component_min_area_, 8);

        // 红灯仅作为“停车状态提示”检测。
        // 真正运动许可仍然只有一个条件：
        // 连续 stable_frames 帧确认 LEFT/STRAIGHT/RIGHT。
        // 这样即使场地存在红色胶带，也不会因为红色检测误差阻塞绿灯。
        pnh_.param("red_h1_min", red_h1_min_, 0);
        pnh_.param("red_h1_max", red_h1_max_, 12);
        pnh_.param("red_h2_min", red_h2_min_, 168);
        pnh_.param("red_h2_max", red_h2_max_, 179);
        pnh_.param("red_s_min", red_s_min_, 160);
        pnh_.param("red_v_min", red_v_min_, 185);
        pnh_.param("red_min_pixels", red_min_pixels_, 80);

        // =========================
        // 整块箭头候选过滤
        // =========================
        pnh_.param("min_arrow_pixels", min_arrow_pixels_, 40);
        pnh_.param("max_arrow_pixels", max_arrow_pixels_, 2200);

        pnh_.param("min_arrow_width", min_arrow_width_, 10);
        pnh_.param("max_arrow_width", max_arrow_width_, 160);
        pnh_.param("min_arrow_height", min_arrow_height_, 10);
        pnh_.param("max_arrow_height", max_arrow_height_, 160);

        pnh_.param("bbox_aspect_min", bbox_aspect_min_, 0.30);
        pnh_.param("bbox_aspect_max", bbox_aspect_max_, 3.20);

        // 原始绿色像素在候选框中的占比。
        // 箭头是稀疏点阵/发光块，不应像绿色椅子那样大面积实心。
        pnh_.param("min_arrow_fill_ratio", min_arrow_fill_ratio_, 0.04);
        pnh_.param("max_arrow_fill_ratio", max_arrow_fill_ratio_, 0.55);

        // 把 1.8m 距离下断裂的箭头小块临时连起来，仅用于“找候选框”。
        // 真正判断方向仍使用原始绿色 mask，不使用膨胀后的形状。
        pnh_.param("close_kernel", close_kernel_, 11);

        // 箭头方向
        pnh_.param("arrow_head_ratio", arrow_head_ratio_, 1.18);

        // 最佳方向必须明显优于第二名。
        // 例如 RIGHT=2.67、UP=2.50 这种非常接近的情况直接判 UNKNOWN，
        // 防止环境噪点被硬猜成某个方向。
        pnh_.param("direction_dominance_ratio",
                   direction_dominance_ratio_, 1.12);

        pnh_.param("edge_fraction", edge_fraction_, 0.40);

        // 连续确认帧数：只从 YAML 读取。
        if (!pnh_.getParam("stable_frames", stable_frames_))
        {
            ROS_FATAL("Missing required parameter: ~stable_frames");
            ros::shutdown();
            return;
        }

        if (stable_frames_ < 1)
        {
            ROS_FATAL("Invalid ~stable_frames=%d, must be >= 1", stable_frames_);
            ros::shutdown();
            return;
        }

        // 调试
        pnh_.param("show_debug", show_debug_, true);

        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

        direction_pub_ =
            nh_.advertise<std_msgs::String>(direction_topic_, 1, true);

        // =========================
        // 巡线服务客户端
        //
        // 三个巡线节点应当提前启动并等待服务调用：
        // LEFT     -> /line2_left
        // STRAIGHT -> /line_right
        // RIGHT    -> /line2_right
        //
        // 注意：这里直接调用 ROS service，不再通过 YAML 写 rosrun 命令。
        // =========================
        left_client_ =
            nh_.serviceClient<line_follow::line_follow>("/line2_left");

        straight_client_ =
            nh_.serviceClient<line_follow::line_follow>("/line_right");

        right_client_ =
            nh_.serviceClient<line_follow::line_follow>("/line2_right");

        if (!cap_.open(camera_index_, cv::CAP_V4L2))
        {
            ROS_FATAL("Cannot open /dev/video%d", camera_index_);
            ros::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        // 摄像头必须先 open()，之后才能清理采集队列。
        // 正式识别前先丢弃一批启动阶段旧帧。
        ROS_INFO("Flushing camera startup buffer...");
        flushCameraBuffer(8);

        ROS_INFO(
            "traffic_light_fullframe started: /dev/video%d, %dx%d",
            camera_index_,
            camera_width_,
            camera_height_);

        ROS_INFO("traffic_light: full-frame green-arrow detection + direct route-service dispatch enabled.");
    }

    ~TrafficLightRecognizer()
    {
        if (cap_.isOpened())
            cap_.release();

        if (show_debug_)
            cv::destroyAllWindows();
    }

    void run()
    {
        ros::Rate rate(20);

        while (ros::ok() && !route_started_)
        {
            publishStop();

            cv::Mat frame;

            // 每轮主动丢弃积压帧，只取更接近当前时刻的画面。
            if (!readLatestFrame(frame))
            {
                ROS_WARN_THROTTLE(1.0, "Camera latest-frame read failed");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // 左右镜像翻转后再进行显示和识别
            if (mirror_image_)
            {
                cv::flip(frame, frame, 1);
            }

            const cv::Rect roi_rect = makeROI(frame);
            cv::Mat roi = frame(roi_rect).clone();

            cv::Mat raw_green_mask;
            cv::Mat arrow_negative_mask;
            cv::Rect arrow_box;

            int arrow_pixels = 0;
            double fill_ratio = 0.0;
            double left_score = 0.0;
            double right_score = 0.0;
            double up_score = 0.0;

            Direction direction =
                recognizeArrow(
                    roi,
                    raw_green_mask,
                    arrow_negative_mask,
                    arrow_box,
                    arrow_pixels,
                    fill_ratio,
                    left_score,
                    right_score,
                    up_score);

            // 默认状态始终是停车。
            // 只有连续确认三种绿箭头后，正式模式才允许启动巡线。
            // 当当前帧没有有效绿箭头时，额外检测高亮红灯并在终端提示。
            if (direction == DIR_NONE ||
                direction == DIR_UNKNOWN)
            {
                if (detectRedLight(roi))
                {
                    ROS_WARN_THROTTLE(
                        0.5,
                        "RED LIGHT DETECTED -> HOLD STOP");
                }
            }

            updateStableResult(direction);

            if (show_debug_)
            {
                drawDebug(
                    roi,
                    arrow_box,
                    direction,
                    arrow_pixels,
                    fill_ratio,
                    left_score,
                    right_score,
                    up_score);

                cv::imshow("traffic_light_debug", roi);
                cv::imshow("traffic_light_green_mask", raw_green_mask);
                cv::imshow("traffic_light_arrow_mask", arrow_negative_mask);
                cv::waitKey(1);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void flushCameraBuffer(int frame_count)
    {
        if (!cap_.isOpened())
            return;

        for (int i = 0; i < frame_count; ++i)
        {
            if (!cap_.grab())
            {
                ROS_WARN("Camera buffer flush stopped at frame %d", i);
                break;
            }
        }
    }

    bool readLatestFrame(cv::Mat &frame)
    {
        if (!cap_.isOpened())
            return false;

        const int discard_count = 4;

        for (int i = 0; i < discard_count; ++i)
        {
            if (!cap_.grab())
                return false;
        }

        if (!cap_.retrieve(frame))
            return false;

        return !frame.empty();
    }

    std::string directionName(Direction d) const
    {
        switch (d)
        {
        case DIR_LEFT:
            return "LEFT";
        case DIR_STRAIGHT:
            return "STRAIGHT";
        case DIR_RIGHT:
            return "RIGHT";
        case DIR_UNKNOWN:
            return "UNKNOWN";
        default:
            return "WAIT";
        }
    }

    void publishStop()
    {
        geometry_msgs::Twist stop;
        cmd_pub_.publish(stop);
    }

    cv::Rect makeROI(const cv::Mat &frame) const
    {
        int x = static_cast<int>(roi_x_ * frame.cols);
        int y = static_cast<int>(roi_y_ * frame.rows);
        int w = static_cast<int>(roi_w_ * frame.cols);
        int h = static_cast<int>(roi_h_ * frame.rows);

        x = std::max(0, std::min(x, frame.cols - 1));
        y = std::max(0, std::min(y, frame.rows - 1));
        w = std::max(1, std::min(w, frame.cols - x));
        h = std::max(1, std::min(h, frame.rows - y));

        return cv::Rect(x, y, w, h);
    }

    double spanYInBand(
        const cv::Mat &mask,
        int x0,
        int x1) const
    {
        x0 = std::max(0, x0);
        x1 = std::min(mask.cols, x1);

        int min_y = mask.rows;
        int max_y = -1;

        for (int y = 0; y < mask.rows; ++y)
        {
            for (int x = x0; x < x1; ++x)
            {
                if (mask.at<unsigned char>(y, x) > 0)
                {
                    min_y = std::min(min_y, y);
                    max_y = std::max(max_y, y);
                }
            }
        }

        if (max_y < 0)
            return 0.0;

        return static_cast<double>(max_y - min_y + 1);
    }

    double spanXInBand(
        const cv::Mat &mask,
        int y0,
        int y1) const
    {
        y0 = std::max(0, y0);
        y1 = std::min(mask.rows, y1);

        int min_x = mask.cols;
        int max_x = -1;

        for (int y = y0; y < y1; ++y)
        {
            for (int x = 0; x < mask.cols; ++x)
            {
                if (mask.at<unsigned char>(y, x) > 0)
                {
                    min_x = std::min(min_x, x);
                    max_x = std::max(max_x, x);
                }
            }
        }

        if (max_x < 0)
            return 0.0;

        return static_cast<double>(max_x - min_x + 1);
    }

    void calculateDirectionScores(
        const cv::Mat &arrow_mask,
        double &left_score,
        double &right_score,
        double &up_score) const
    {
        const int edge_w =
            std::max(
                1,
                static_cast<int>(
                    arrow_mask.cols * edge_fraction_));

        const int edge_h =
            std::max(
                1,
                static_cast<int>(
                    arrow_mask.rows * edge_fraction_));

        const double left_span =
            spanYInBand(
                arrow_mask,
                0,
                edge_w);

        const double right_span =
            spanYInBand(
                arrow_mask,
                arrow_mask.cols - edge_w,
                arrow_mask.cols);

        const double top_span =
            spanXInBand(
                arrow_mask,
                0,
                edge_h);

        const double bottom_span =
            spanXInBand(
                arrow_mask,
                arrow_mask.rows - edge_h,
                arrow_mask.rows);

        const double eps = 1.0;

        left_score =
            left_span /
            std::max(eps, right_span);

        right_score =
            right_span /
            std::max(eps, left_span);

        up_score =
            top_span /
            std::max(eps, bottom_span);
    }

    Direction scoreToDirection(
        double left_score,
        double right_score,
        double up_score,
        double &best_score) const
    {
        double scores[3] =
        {
            left_score,
            right_score,
            up_score
        };

        Direction directions[3] =
        {
            DIR_LEFT,
            DIR_RIGHT,
            DIR_STRAIGHT
        };

        int best_index = 0;

        if (scores[1] > scores[best_index])
            best_index = 1;

        if (scores[2] > scores[best_index])
            best_index = 2;

        best_score = scores[best_index];

        if (best_score < arrow_head_ratio_)
            return DIR_UNKNOWN;

        // 找第二高分
        double second_best = 0.0;

        for (int i = 0; i < 3; ++i)
        {
            if (i == best_index)
                continue;

            second_best =
                std::max(
                    second_best,
                    scores[i]);
        }

        // 最佳方向与第二名太接近时，宁可 UNKNOWN，也不猜方向。
        const double dominance =
            best_score /
            std::max(0.01, second_best);

        if (dominance <
            direction_dominance_ratio_)
        {
            return DIR_UNKNOWN;
        }

        return directions[best_index];
    }

    bool detectRedLight(const cv::Mat &bgr) const
    {
        cv::Mat hsv;

        cv::cvtColor(
            bgr,
            hsv,
            cv::COLOR_BGR2HSV);

        cv::Mat red1;
        cv::Mat red2;
        cv::Mat red_mask;

        cv::inRange(
            hsv,
            cv::Scalar(
                red_h1_min_,
                red_s_min_,
                red_v_min_),
            cv::Scalar(
                red_h1_max_,
                255,
                255),
            red1);

        cv::inRange(
            hsv,
            cv::Scalar(
                red_h2_min_,
                red_s_min_,
                red_v_min_),
            cv::Scalar(
                red_h2_max_,
                255,
                255),
            red2);

        cv::bitwise_or(
            red1,
            red2,
            red_mask);

        cv::Mat kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3));

        cv::morphologyEx(
            red_mask,
            red_mask,
            cv::MORPH_OPEN,
            kernel);

        // 这里故意只做“强红色像素数量”提示。
        // 不拿它作为绿灯的硬否决条件，避免纸箱红胶带造成误阻塞。
        const int red_pixels =
            cv::countNonZero(red_mask);

        return red_pixels >= red_min_pixels_;
    }

    Direction recognizeArrow(
        const cv::Mat &bgr,
        cv::Mat &raw_green_mask,
        cv::Mat &arrow_negative_mask,
        cv::Rect &arrow_box,
        int &arrow_pixels,
        double &fill_ratio,
        double &left_score,
        double &right_score,
        double &up_score)
    {
        // ------------------------------------------------------------
        // 1. 全图 HSV 绿色提取
        // ------------------------------------------------------------
        cv::Mat hsv;

        cv::cvtColor(
            bgr,
            hsv,
            cv::COLOR_BGR2HSV);

        cv::inRange(
            hsv,
            cv::Scalar(
                green_h_min_,
                green_s_min_,
                green_v_min_),
            cv::Scalar(
                green_h_max_,
                255,
                255),
            raw_green_mask);

        // 去除极小噪点
        cv::Mat open_kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3));

        cv::morphologyEx(
            raw_green_mask,
            raw_green_mask,
            cv::MORPH_OPEN,
            open_kernel);

        // ------------------------------------------------------------
        // 2. 从宽松绿色 mask 中提取“黑色负形箭头”
        //
        // 真实测试中：
        //   - 箭头 LED 中心过曝/泛白 -> 不满足绿色阈值 -> 在 green_mask 中是黑色
        //   - 箭头周围绿色光晕      -> 满足绿色阈值   -> 在 green_mask 中是白色
        //
        // 因此先用较大的闭运算把绿色光晕围成“支撑区域”，
        // 再做：
        //
        //   arrow_negative_mask = green_support AND (NOT green_mask)
        //
        // 这样会把“被绿色包围或紧邻绿色的黑色箭头负形”提出来。
        // ------------------------------------------------------------
        int close_k = negative_close_kernel_;

        if (close_k < 1)
            close_k = 1;

        if (close_k % 2 == 0)
            ++close_k;

        cv::Mat support_kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(close_k, close_k));

        cv::Mat green_support;

        cv::morphologyEx(
            raw_green_mask,
            green_support,
            cv::MORPH_CLOSE,
            support_kernel);

        cv::Mat not_green;

        cv::bitwise_not(
            raw_green_mask,
            not_green);

        cv::Mat bright_core_mask;

        // 过曝 LED 中心通常表现为：
        //   V 高（很亮）
        //   S 低到中等（由于泛白，饱和度下降）
        // H 在这里不再限制。
        cv::inRange(
            hsv,
            cv::Scalar(
                0,
                0,
                bright_core_v_min_),
            cv::Scalar(
                179,
                bright_core_s_max_,
                255),
            bright_core_mask);

        cv::bitwise_and(
            green_support,
            not_green,
            arrow_negative_mask);

        // 只保留“绿色支撑附近的过曝白芯”。
        // 这是本版去除绿色椅子/绿色反光噪点的关键。
        cv::bitwise_and(
            arrow_negative_mask,
            bright_core_mask,
            arrow_negative_mask);

        // 去掉负形 mask 中极小的碎点
        int open_k = negative_open_kernel_;

        if (open_k < 1)
            open_k = 1;

        if (open_k % 2 == 0)
            ++open_k;

        cv::Mat negative_open_kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(open_k, open_k));

        cv::morphologyEx(
            arrow_negative_mask,
            arrow_negative_mask,
            cv::MORPH_OPEN,
            negative_open_kernel);

        // ------------------------------------------------------------
        // 删除 arrow_mask 中很小的孤立白点
        // ------------------------------------------------------------
        if (negative_component_min_area_ > 1)
        {
            cv::Mat labels;
            cv::Mat stats;
            cv::Mat centroids;

            const int component_count =
                cv::connectedComponentsWithStats(
                    arrow_negative_mask,
                    labels,
                    stats,
                    centroids,
                    8,
                    CV_32S);

            cv::Mat cleaned =
                cv::Mat::zeros(
                    arrow_negative_mask.size(),
                    CV_8UC1);

            for (int label = 1;
                 label < component_count;
                 ++label)
            {
                const int area =
                    stats.at<int>(
                        label,
                        cv::CC_STAT_AREA);

                if (area <
                    negative_component_min_area_)
                {
                    continue;
                }

                cleaned.setTo(
                    255,
                    labels == label);
            }

            arrow_negative_mask =
                cleaned;
        }

        // ------------------------------------------------------------
        // 3. 扫描整张图的所有候选，不依赖固定位置
        // ------------------------------------------------------------
        std::vector<std::vector<cv::Point> > contours;

        cv::findContours(
            arrow_negative_mask.clone(),
            contours,
            cv::RETR_EXTERNAL,
            cv::CHAIN_APPROX_SIMPLE);

        bool found = false;

        double best_quality = -1.0;
        Direction best_direction = DIR_NONE;

        cv::Rect best_box;
        int best_pixels = 0;
        double best_fill = 0.0;
        double best_left = 0.0;
        double best_right = 0.0;
        double best_up = 0.0;

        for (size_t i = 0; i < contours.size(); ++i)
        {
            const cv::Rect box =
                cv::boundingRect(contours[i]);

            // 尺寸门：排除很小噪点、大片椅子/地面
            if (box.width < min_arrow_width_ ||
                box.width > max_arrow_width_ ||
                box.height < min_arrow_height_ ||
                box.height > max_arrow_height_)
            {
                continue;
            }

            const double aspect =
                static_cast<double>(box.width) /
                static_cast<double>(
                    std::max(1, box.height));

            // 排除很细的横线、竖线
            if (aspect < bbox_aspect_min_ ||
                aspect > bbox_aspect_max_)
            {
                continue;
            }

            const cv::Mat candidate =
                arrow_negative_mask(box);

            const int pixels =
                cv::countNonZero(candidate);

            if (pixels < min_arrow_pixels_ ||
                pixels > max_arrow_pixels_)
            {
                continue;
            }

            const double fill =
                static_cast<double>(pixels) /
                static_cast<double>(
                    std::max(1, box.width * box.height));

            // 这里分析的是“黑色负形箭头”反相后的白色区域。
            // 过小/过满的候选都排除。
            if (fill < min_arrow_fill_ratio_ ||
                fill > max_arrow_fill_ratio_)
            {
                continue;
            }

            // 直接对负形箭头候选算 LEFT / RIGHT / UP 几何结构。
            double l = 0.0;
            double r = 0.0;
            double u = 0.0;

            calculateDirectionScores(
                candidate,
                l,
                r,
                u);

            double direction_score = 0.0;

            const Direction direction =
                scoreToDirection(
                    l,
                    r,
                    u,
                    direction_score);

            if (direction == DIR_UNKNOWN)
                continue;

            // 候选评分：
            // 方向特征越明显越好；
            // 像素足够但不是实心大块越好。
            const double sparsity_bonus =
                1.0 -
                std::min(
                    1.0,
                    std::fabs(fill - 0.22));

            const double quality =
                direction_score *
                std::sqrt(
                    static_cast<double>(pixels)) *
                sparsity_bonus;

            if (quality > best_quality)
            {
                best_quality = quality;
                best_direction = direction;
                best_box = box;
                best_pixels = pixels;
                best_fill = fill;
                best_left = l;
                best_right = r;
                best_up = u;
                found = true;
            }
        }

        if (!found)
        {
            arrow_box = cv::Rect();
            arrow_pixels = 0;
            fill_ratio = 0.0;
            left_score = 0.0;
            right_score = 0.0;
            up_score = 0.0;

            ROS_INFO_THROTTLE(
                0.5,
                "WAIT: no valid negative-arrow candidate");

            return DIR_NONE;
        }

        arrow_box = best_box;
        arrow_pixels = best_pixels;
        fill_ratio = best_fill;
        left_score = best_left;
        right_score = best_right;
        up_score = best_up;

        ROS_INFO_THROTTLE(
            0.3,
            "ARROW box=(%d,%d,%d,%d) pix=%d fill=%.2f | "
            "LEFT=%.2f RIGHT=%.2f UP=%.2f",
            best_box.x,
            best_box.y,
            best_box.width,
            best_box.height,
            best_pixels,
            best_fill,
            best_left,
            best_right,
            best_up);

        return best_direction;
    }

    void updateStableResult(Direction direction)
    {
        if (direction == DIR_NONE ||
            direction == DIR_UNKNOWN)
        {
            stable_count_ = 0;
            last_direction_ = direction;
            return;
        }

        if (direction == last_direction_)
        {
            ++stable_count_;
        }
        else
        {
            last_direction_ = direction;
            stable_count_ = 1;
        }

        ROS_INFO_THROTTLE(
            0.3,
            "candidate=%s stable=%d/%d",
            directionName(direction).c_str(),
            stable_count_,
            stable_frames_);

        if (stable_count_ < stable_frames_)
            return;

        std_msgs::String result;
        result.data = directionName(direction);

        direction_pub_.publish(result);

        ROS_WARN(
            "GREEN ARROW CONFIRMED: %s",
            result.data.c_str());

        callRouteService(direction);
    }

    void callRouteService(Direction direction)
    {
        ros::ServiceClient *client = nullptr;
        const char *service_name = nullptr;

        if (direction == DIR_LEFT)
        {
            client = &left_client_;
            service_name = "/line2_left";
        }
        else if (direction == DIR_STRAIGHT)
        {
            client = &straight_client_;
            service_name = "/line_right";
        }
        else if (direction == DIR_RIGHT)
        {
            client = &right_client_;
            service_name = "/line2_right";
        }
        else
        {
            return;
        }

        // 先确认服务存在。
        // 三个巡线服务节点应提前启动；这里只负责在绿箭头确认后调用。
        if (!client->waitForExistence(ros::Duration(1.0)))
        {
            ROS_ERROR(
                "Route service %s is not available -> HOLD STOP",
                service_name);

            // 服务不存在时不进入巡线模式，继续识别并保持停车。
            stable_count_ = 0;
            last_direction_ = DIR_NONE;
            publishStop();
            return;
        }

        // 从这一刻开始，方向已经在绿灯阶段正式确认。
        // 立即锁定巡线模式，不再继续识别后续红/黄/绿灯。
        route_started_ = true;

        // 切换控制权前最后发布一次停车，避免旧速度残留。
        publishStop();

        // traffic_light 当前占用 /dev/video0。
        // 巡线服务回调也会打开 /dev/video0，所以必须先释放摄像头。
        if (cap_.isOpened())
        {
            cap_.release();
            ROS_INFO("Released /dev/video%d before route service", camera_index_);
        }

        if (show_debug_)
        {
            cv::destroyAllWindows();
        }

        // 给 V4L2 很短的释放时间，然后立刻调用巡线服务。
        ros::Duration(0.05).sleep();

        ROS_WARN(
            "Calling route service: %s",
            service_name);

        line_follow::line_follow srv;

        // 巡线服务回调会持续执行完整巡线流程。
        // 本调用是同步调用；在服务执行期间，本节点不会再做红绿灯识别，
        // 也不会再向 /cmd_vel 发布停车指令。
        if (!client->call(srv))
        {
            ROS_ERROR(
                "Failed to call route service %s",
                service_name);

            // 此时摄像头已经释放，最安全的处理是停车并结束本节点，
            // 避免在未知状态下重新抢占摄像头。
            publishStop();
            ros::shutdown();
            return;
        }

        ROS_INFO(
            "Route service %s returned",
            service_name);

        ros::shutdown();
    }

    void drawDebug(
        cv::Mat &image,
        const cv::Rect &box,
        Direction direction,
        int pixels,
        double fill_ratio,
        double left_score,
        double right_score,
        double up_score)
    {
        if (box.area() > 0)
        {
            cv::rectangle(
                image,
                box,
                cv::Scalar(0, 255, 255),
                2);
        }

        char line1[160];
        char line2[160];

        std::snprintf(
            line1,
            sizeof(line1),
            "DIR=%s PIX=%d FILL=%.2f",
            directionName(direction).c_str(),
            pixels,
            fill_ratio);

        std::snprintf(
            line2,
            sizeof(line2),
            "L=%.2f R=%.2f UP=%.2f",
            left_score,
            right_score,
            up_score);

        cv::putText(
            image,
            line1,
            cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX,
            0.58,
            cv::Scalar(0, 255, 255),
            2);

        cv::putText(
            image,
            line2,
            cv::Point(10, 50),
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(0, 255, 255),
            2);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher cmd_pub_;
    ros::Publisher direction_pub_;

    cv::VideoCapture cap_;

    int camera_index_;
    int camera_width_;
    int camera_height_;
    bool mirror_image_;

    std::string cmd_vel_topic_;
    std::string direction_topic_;

    double roi_x_;
    double roi_y_;
    double roi_w_;
    double roi_h_;

    int green_h_min_;
    int green_h_max_;
    int green_s_min_;
    int green_v_min_;

    int negative_close_kernel_;
    int negative_open_kernel_;

    int bright_core_v_min_;
    int bright_core_s_max_;
    int negative_component_min_area_;

    int red_h1_min_;
    int red_h1_max_;
    int red_h2_min_;
    int red_h2_max_;
    int red_s_min_;
    int red_v_min_;
    int red_min_pixels_;

    int min_arrow_pixels_;
    int max_arrow_pixels_;

    int min_arrow_width_;
    int max_arrow_width_;
    int min_arrow_height_;
    int max_arrow_height_;

    double bbox_aspect_min_;
    double bbox_aspect_max_;

    double min_arrow_fill_ratio_;
    double max_arrow_fill_ratio_;

    int close_kernel_;

    double arrow_head_ratio_;
    double direction_dominance_ratio_;
    double edge_fraction_;

    int stable_frames_;
    bool show_debug_;

    ros::ServiceClient left_client_;
    ros::ServiceClient straight_client_;
    ros::ServiceClient right_client_;

    bool route_started_;
    int stable_count_;
    Direction last_direction_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_light");

    TrafficLightRecognizer node;

    if (!ros::ok())
        return 1;

    node.run();
    return 0;
}