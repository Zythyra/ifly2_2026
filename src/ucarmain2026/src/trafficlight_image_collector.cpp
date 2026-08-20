#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <cstring>
#include <clocale>

class TrafficLightImageCollector
{
public:
    TrafficLightImageCollector()
        : nh_(),
          pnh_("~"),
          current_yaw_(0.0),
          odom_received_(false),
          total_image_count_(0)
    {
        // =========================
        // 参数
        // =========================
        pnh_.param<std::string>(
            "camera_device",
            camera_device_,
            std::string("/dev/video0"));

        pnh_.param<std::string>(
            "save_dir",
            save_dir_,
            std::string("/home/ucar/ucar_ws_copy/src/ucarmain2026/trafficlight_images"));

        pnh_.param<int>("image_width", image_width_, 640);
        pnh_.param<int>("image_height", image_height_, 480);

        pnh_.param<double>("capture_interval", capture_interval_, 1.0);

        // 转向控制参数
        pnh_.param<double>("rotate_max_speed", rotate_max_speed_, 0.45);
        pnh_.param<double>("rotate_min_speed", rotate_min_speed_, 0.12);
        pnh_.param<double>("rotate_kp", rotate_kp_, 2.5);
        pnh_.param<double>("rotate_tolerance_deg", rotate_tolerance_deg_, 0.5);

        // 连续多少个控制周期满足角度误差后认为转向完成
        pnh_.param<int>("rotate_stable_frames", rotate_stable_frames_, 5);

        // 摄像头缓存刷新帧数
        pnh_.param<int>("camera_flush_frames", camera_flush_frames_, 5);

        // =========================
        // ROS
        // =========================
        cmd_vel_pub_ =
            nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        odom_sub_ =
            nh_.subscribe("/odom",
                          20,
                          &TrafficLightImageCollector::odomCallback,
                          this);

        ROS_INFO("========================================");
        ROS_INFO("交通灯图像采集程序启动");
        ROS_INFO("摄像头：%s", camera_device_.c_str());
        ROS_INFO("图像尺寸：%d x %d", image_width_, image_height_);
        ROS_INFO("保存目录：%s", save_dir_.c_str());
        ROS_INFO("采集间隔：%.2f 秒", capture_interval_);
        ROS_INFO("========================================");
    }

    ~TrafficLightImageCollector()
    {
        stopRobot();

        if (camera_.isOpened())
        {
            camera_.release();
        }
    }

    bool initialize()
    {
        // =========================
        // 检查保存目录
        // =========================
        if (!ensureDirectory(save_dir_))
        {
            ROS_ERROR("无法创建或访问图片保存目录：%s",
                      save_dir_.c_str());
            return false;
        }

        // =========================
        // 打开摄像头
        // =========================
        ROS_INFO("正在打开摄像头 %s ...",
                 camera_device_.c_str());

        if (!camera_.open(camera_device_, cv::CAP_V4L2))
        {
            ROS_ERROR("摄像头打开失败：%s",
                      camera_device_.c_str());
            return false;
        }

        // 设置摄像头参数
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

        // 尽量减小 V4L2 缓存
        camera_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        // 部分 USB 摄像头使用 MJPG 可以获得更稳定帧率
        camera_.set(
            cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // 给摄像头一点初始化时间
        ros::Duration(1.0).sleep();

        // 丢掉最开始若干帧
        cv::Mat dummy;
        for (int i = 0; i < 10; ++i)
        {
            if (!camera_.read(dummy))
            {
                ROS_WARN("摄像头初始化阶段读取第 %d 帧失败", i + 1);
            }

            ros::Duration(0.03).sleep();
        }

        ROS_INFO("摄像头打开成功");
        ROS_INFO("实际分辨率：%.0f x %.0f",
                 camera_.get(cv::CAP_PROP_FRAME_WIDTH),
                 camera_.get(cv::CAP_PROP_FRAME_HEIGHT));

        // =========================
        // 等待里程计
        // =========================
        ROS_INFO("等待 /odom ...");

        ros::Rate wait_rate(20.0);

        const ros::Time start_time = ros::Time::now();

        while (ros::ok() && !odom_received_)
        {
            ros::spinOnce();
            stopRobot();

            if ((ros::Time::now() - start_time).toSec() > 10.0)
            {
                ROS_ERROR("等待 /odom 超时，请检查底盘里程计是否正常发布");
                return false;
            }

            wait_rate.sleep();
        }

        ROS_INFO("/odom 接收正常");
        ROS_INFO("当前小车 yaw = %.2f°",
                 rad2deg(current_yaw_));

        return true;
    }

    void run()
    {
        ROS_WARN("========================================");
        ROS_WARN("开始自动图像采集");
        ROS_WARN("总共将采集 200 张图片");
        ROS_WARN("请确保小车周围有足够旋转空间");
        ROS_WARN("========================================");

        stopRobot();
        ros::Duration(0.5).sleep();

        // ============================================================
        // 第一阶段：当前方向采 100 张
        // ============================================================
        ROS_WARN("【阶段 1/5】当前方向采集 100 张");

        if (!captureImages(100, 1))
        {
            abortSequence();
            return;
        }

        // ============================================================
        // 第二阶段：左转 10°
        // ============================================================
        ROS_WARN("【阶段 2/5】开始左转 10°");

        if (!rotateRelative(deg2rad(10.0)))
        {
            abortSequence();
            return;
        }

        ROS_WARN("左转 10°完成");

        // 稳定一下再开始拍
        stopRobot();
        ros::Duration(0.8).sleep();

        // ============================================================
        // 第三阶段：采 50 张
        // ============================================================
        ROS_WARN("【阶段 3/5】左转 10°位置采集 50 张");

        if (!captureImages(50, 2))
        {
            abortSequence();
            return;
        }

        // ============================================================
        // 第四阶段：右转 20°
        // ============================================================
        ROS_WARN("【阶段 4/5】开始右转 20°");

        if (!rotateRelative(deg2rad(-20.0)))
        {
            abortSequence();
            return;
        }

        ROS_WARN("右转 20°完成");

        stopRobot();
        ros::Duration(0.8).sleep();

        // ============================================================
        // 第五阶段：采 50 张
        // ============================================================
        ROS_WARN("【阶段 5/5】右转后位置采集 50 张");

        if (!captureImages(50, 3))
        {
            abortSequence();
            return;
        }

        stopRobot();

        ROS_WARN("========================================");
        ROS_WARN("全部采集完成！");
        ROS_WARN("共保存 %d 张图片", total_image_count_);
        ROS_WARN("保存位置：%s", save_dir_.c_str());
        ROS_WARN("========================================");
    }

private:
    // ================================================================
    // ODOM 回调
    // ================================================================
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        const auto &q = msg->pose.pose.orientation;

        // 四元数 -> yaw
        const double siny_cosp =
            2.0 * (q.w * q.z + q.x * q.y);

        const double cosy_cosp =
            1.0 - 2.0 * (q.y * q.y + q.z * q.z);

        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        odom_received_ = true;
    }

    // ================================================================
    // 采集若干张图片
    // ================================================================
    bool captureImages(int count, int stage)
    {
        if (!camera_.isOpened())
        {
            ROS_ERROR("摄像头未打开");
            return false;
        }

        ROS_INFO("----------------------------------------");
        ROS_INFO("开始第 %d 阶段采图，共 %d 张", stage, count);
        ROS_INFO("----------------------------------------");

        ros::Rate capture_rate(1.0 / capture_interval_);

        for (int i = 0; i < count && ros::ok(); ++i)
        {
            ros::spinOnce();

            // 采图阶段持续确保车处于停车状态
            stopRobot();

            cv::Mat frame;

            // --------------------------------------------------------
            // 刷新缓存
            //
            // 因为这里只是每 1 秒读一次，如果不刷新缓存，
            // 某些 V4L2 摄像头可能读到缓存中的旧画面。
            // --------------------------------------------------------
            bool read_ok = false;

            for (int flush = 0;
                 flush < camera_flush_frames_;
                 ++flush)
            {
                cv::Mat temp;

                if (camera_.read(temp) && !temp.empty())
                {
                    frame = temp;
                    read_ok = true;
                }
                else
                {
                    ROS_WARN("刷新摄像头缓存时读取失败");
                }
            }

            if (!read_ok || frame.empty())
            {
                ROS_ERROR("第 %d 阶段第 %d 张图片读取失败",
                          stage,
                          i + 1);

                // 不退出整个程序，继续尝试下一次采集
                capture_rate.sleep();
                continue;
            }

            // --------------------------------------------------------
            // 生成时间戳文件名
            // --------------------------------------------------------
            ++total_image_count_;

            const std::string filename =
                generateTimestampFilename(total_image_count_);

            const std::string full_path =
                save_dir_ + "/" + filename;

            // JPEG 保存质量
            std::vector<int> jpeg_params;
            jpeg_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            jpeg_params.push_back(95);

            try
            {
                if (!cv::imwrite(full_path, frame, jpeg_params))
                {
                    ROS_ERROR("保存图片失败：%s",
                              full_path.c_str());
                }
                else
                {
                    ROS_INFO(
                        "[阶段%d] %d/%d | 总计 %d/200 | 保存：%s",
                        stage,
                        i + 1,
                        count,
                        total_image_count_,
                        filename.c_str());
                }
            }
            catch (const cv::Exception &e)
            {
                ROS_ERROR("OpenCV 保存图片异常：%s",
                          e.what());
            }

            capture_rate.sleep();
        }

        if (!ros::ok())
        {
            return false;
        }

        ROS_INFO("第 %d 阶段采图完成", stage);

        return true;
    }

    // ================================================================
    // 相对旋转
    //
    // angle > 0：左转
    // angle < 0：右转
    // ================================================================
    bool rotateRelative(double angle)
    {
        if (!odom_received_)
        {
            ROS_ERROR("没有收到 /odom，无法进行闭环旋转");
            return false;
        }

        ros::spinOnce();

        const double start_yaw = current_yaw_;
        const double target_yaw =
            normalizeAngle(start_yaw + angle);

        ROS_INFO("----------------------------------------");
        ROS_INFO("开始旋转");
        ROS_INFO("起始 yaw：%.2f°", rad2deg(start_yaw));
        ROS_INFO("相对旋转：%.2f°", rad2deg(angle));
        ROS_INFO("目标 yaw：%.2f°", rad2deg(target_yaw));
        ROS_INFO("----------------------------------------");

        const double tolerance =
            deg2rad(rotate_tolerance_deg_);

        ros::Rate control_rate(50.0);

        int stable_count = 0;

        const ros::Time rotate_start_time =
            ros::Time::now();

        while (ros::ok())
        {
            ros::spinOnce();

            const double error =
                normalizeAngle(target_yaw - current_yaw_);

            const double abs_error =
                std::fabs(error);

            // --------------------------------------------------------
            // 到达目标角度
            // --------------------------------------------------------
            if (abs_error <= tolerance)
            {
                stable_count++;

                stopRobot();

                if (stable_count >= rotate_stable_frames_)
                {
                    stopRobot();

                    ROS_INFO(
                        "旋转完成：当前 yaw=%.2f°，目标=%.2f°，误差=%.3f°",
                        rad2deg(current_yaw_),
                        rad2deg(target_yaw),
                        rad2deg(error));

                    ros::Duration(0.2).sleep();

                    return true;
                }
            }
            else
            {
                stable_count = 0;

                // ----------------------------------------------------
                // P 控制
                // ----------------------------------------------------
                double angular_speed =
                    rotate_kp_ * error;

                angular_speed =
                    std::max(-rotate_max_speed_,
                             std::min(rotate_max_speed_,
                                      angular_speed));

                // 靠近目标时仍保持最低可执行角速度
                if (std::fabs(angular_speed) <
                    rotate_min_speed_)
                {
                    angular_speed =
                        (error > 0.0)
                            ? rotate_min_speed_
                            : -rotate_min_speed_;
                }

                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = 0.0;

                cmd.angular.x = 0.0;
                cmd.angular.y = 0.0;
                cmd.angular.z = angular_speed;

                cmd_vel_pub_.publish(cmd);
            }

            // 10° / 20°正常几秒就能完成。
            // 15 秒仍未完成视作异常，避免无限旋转。
            if ((ros::Time::now() - rotate_start_time).toSec() > 15.0)
            {
                stopRobot();

                ROS_ERROR(
                    "旋转超时！当前 yaw=%.2f°，目标 yaw=%.2f°",
                    rad2deg(current_yaw_),
                    rad2deg(target_yaw));

                return false;
            }

            control_rate.sleep();
        }

        stopRobot();

        return false;
    }

    // ================================================================
    // 停车
    // ================================================================
    void stopRobot()
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_vel_pub_.publish(cmd);
    }

    // ================================================================
    // 异常终止
    // ================================================================
    void abortSequence()
    {
        stopRobot();

        ROS_ERROR("========================================");
        ROS_ERROR("图像采集流程异常终止");
        ROS_ERROR("当前已成功编号 %d 张图片", total_image_count_);
        ROS_ERROR("========================================");
    }

    // ================================================================
    // 时间戳文件名
    //
    // 示例：
    // 20260818_180721_315_0001.jpg
    // ================================================================
    std::string generateTimestampFilename(int index)
    {
        using namespace std::chrono;

        const auto now =
            system_clock::now();

        const auto milliseconds_since_epoch =
            duration_cast<milliseconds>(
                now.time_since_epoch());

        const int millisecond =
            static_cast<int>(
                milliseconds_since_epoch.count() % 1000);

        const std::time_t time_now =
            system_clock::to_time_t(now);

        std::tm local_tm;

        localtime_r(&time_now, &local_tm);

        std::ostringstream oss;

        oss << std::put_time(
                   &local_tm,
                   "%Y%m%d_%H%M%S")
            << "_"
            << std::setw(3)
            << std::setfill('0')
            << millisecond
            << "_"
            << std::setw(4)
            << std::setfill('0')
            << index
            << ".jpg";

        return oss.str();
    }

    // ================================================================
    // 确保保存目录存在
    // ================================================================
    bool ensureDirectory(const std::string &path)
    {
        struct stat info;

        if (stat(path.c_str(), &info) == 0)
        {
            if (info.st_mode & S_IFDIR)
            {
                return true;
            }

            ROS_ERROR("%s 已存在，但不是目录",
                      path.c_str());

            return false;
        }

        // 这里的父目录本身应该已经存在
        if (mkdir(path.c_str(), 0755) == 0)
        {
            ROS_INFO("已创建图片目录：%s",
                     path.c_str());

            return true;
        }

        if (errno == EEXIST)
        {
            return true;
        }

        ROS_ERROR("创建目录失败：%s，错误：%s",
                  path.c_str(),
                  std::strerror(errno));

        return false;
    }

    // ================================================================
    // 角度工具
    // ================================================================
    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

    static double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    static double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;

    cv::VideoCapture camera_;

    std::string camera_device_;
    std::string save_dir_;

    int image_width_;
    int image_height_;

    double capture_interval_;

    double rotate_max_speed_;
    double rotate_min_speed_;
    double rotate_kp_;
    double rotate_tolerance_deg_;

    int rotate_stable_frames_;
    int camera_flush_frames_;

    double current_yaw_;
    bool odom_received_;

    int total_image_count_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc,
              argv,
              "trafficlight_image_collector");

    TrafficLightImageCollector collector;

    if (!collector.initialize())
    {
        ROS_ERROR("初始化失败，程序退出");
        return 1;
    }

    collector.run();

    return 0;
}