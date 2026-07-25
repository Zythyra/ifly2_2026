#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <random>
#include <string>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>
#include "line_follow/line_follow.h"
#include "ucarmain2026/getpose_server.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// 定义MoveBase客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv;
using namespace std;

// 声明赛道结构体（关键修复：在类外部或内部提前声明）
struct RaceTrack {
    double slope = 2.0;           // 赛道斜率
    vector<Point> points;          // 赛道点集
    int direction_change = 0;      // 方向变化次数
    int slope_change_count = 0;    // 斜率变化次数
    bool right_point = false;       // 是否为右赛道标志
};

class LineFollowerNode {
private:
    // ROS核心组件
    ros::NodeHandle nh_;                  // 节点句柄
    ros::ServiceServer line_server_;      // 服务端
    ros::Publisher cmd_pub_;              // 速度发布者

    ros::ServiceClient pose_client_;      // 位姿服务客户端
    ros::ServiceClient reconfigure_client_;// 动态配置客户端
    tf::TransformListener* tf_listener_;  // TF监听器
    MoveBaseClient* ac_;                  // MoveBase客户端

    // 消息对象
    geometry_msgs::Twist twist_;

    ucarmain2026::getpose_server pose_;

    // 图像处理相关
    Mat cameraMatrix_, distCoeffs_;       // 相机内参和畸变系数
    VideoCapture cap_;                    // 相机捕获
    VideoWriter out_;                     // 视频录制
    string output_file_;                  // 视频保存路径
    int fourcc_;                          // 视频编码格式
    ostringstream displayStream_;         // 信息显示流
    Rect roi_;                            // 图像裁剪区域
    Mat map1_, map2_;                     // 去畸变映射表
    int center_distance;

    // 控制参数
    double p_, i_, d_;                    // PID参数
    double rightpoint_p_, rightpoint_I_, rightpoint_D_; // 右点控制参数
    double x_max_, integration_limit_;    // 速度和积分限制
    double out_turn_angel_;               // 右点追踪完成后的目标角度
    double start_straight_distance_;       // 从起始停止线向前行驶的距离
    double start_turn_angular_speed_;      // 原地左转和搜线时的角速度大小
    double start_rotate_angle_deg_;        // 前进完成后的原地左转角度（度）
    double integration_, pre_error_;      // 积分和前向误差
    double pointx_integration_, pointx_pre_error_; // 右点积分和前向误差

    // 启动阶段：先直行，再原地左转指定角度，随后继续原地左转搜线
    enum class StartStage {
        STRAIGHT,
        ROTATE_FIXED_ANGLE,
        SEARCH_LEFT_LINE,
        NORMAL_TRACKING
    };

    StartStage start_stage_;
    double start_x_;
    double start_y_;
    double rotate_start_yaw_;
    int start_line_stable_count_;

    // 原巡线状态变量
    bool double_line_;                    // 双边巡线标志
    bool right_point_start_;               // 右点追踪标志
    bool point_forward_;                  // 右点前进标志
    int trace_failed_count_;              // 追踪失败计数

public:
    // 构造函数：初始化所有组件
    LineFollowerNode() : 
        nh_(""), 
        output_file_("/home/ucar/ucar_ws_copy/src/line_follow/image/line2_left.avi"),
        fourcc_(VideoWriter::fourcc('X', 'V', 'I', 'D')),
        roi_(0, 210, 640, 270),
        start_stage_(StartStage::STRAIGHT),
        start_x_(0.0),
        start_y_(0.0),
        rotate_start_yaw_(0.0),
        start_line_stable_count_(0),
        double_line_(false),
        right_point_start_(false),
        point_forward_(true),
        trace_failed_count_(0),
        integration_(0), 
        pre_error_(0),
        pointx_integration_(0),
        pointx_pre_error_(0) {

        ROS_INFO("开始初始化LineFollowerNode...");

        // 1. 初始化服务端（优先初始化）
        line_server_ = nh_.advertiseService("line2_left", &LineFollowerNode::line_server_callback, this);
        ROS_INFO("line2_left服务已初始化");

        // 2. 加载参数
        loadParameters();

        // 3. 初始化ROS客户端和发布者
        initRosComponents();

        // 4. 读取相机标定文件并初始化去畸变
        if (!loadCalibrationFile()) {
            ROS_FATAL("标定文件加载失败，节点无法启动");
            ros::shutdown();
            return;
        }

        ROS_INFO("所有组件初始化完成");
    }

    // 析构函数：释放资源
    ~LineFollowerNode() {
        cap_.release();
        out_.release();
        delete tf_listener_;
        delete ac_;
        ROS_INFO("节点资源已释放");
    }

    // 运行节点主循环
    void run() {
        ros::spin();
    }

private:
    // 加载ROS参数
    void loadParameters() {
        nh_.getParam("/line2_left/left_P", p_);
        nh_.getParam("/line2_left/left_I", i_);
        nh_.getParam("/line2_left/left_D", d_);
        nh_.getParam("/line2_left/rightpoint_p", rightpoint_p_);
        nh_.getParam("/line2_left/rightpoint_I", rightpoint_I_);
        nh_.getParam("/line2_left/rightpoint_D", rightpoint_D_);
        nh_.getParam("/line2_left/x_max_", x_max_);
        nh_.getParam("/line2_left/integration_limit", integration_limit_);
        nh_.getParam("/line2_left/out_turn_angel", out_turn_angel_);
        nh_.getParam("/line2_left/center_distance", center_distance);

        // 起始动作参数：直行距离、原地左转角速度、原地左转角度。
        nh_.param<double>(
            "/line2_left/start_straight_distance",
            start_straight_distance_,
            0.20
        );
        nh_.param<double>(
            "/line2_left/start_turn_angular_speed",
            start_turn_angular_speed_,
            0.35
        );
        nh_.param<double>(
            "/line2_left/start_rotate_angle_deg",
            start_rotate_angle_deg_,
            40.0
        );

        ROS_INFO(
            "参数加载完成: center_distance=%d, 起点直行距离=%.3f m, "
            "原地左转角度=%.1f 度, 左转角速度大小=%.3f rad/s",
            center_distance,
            start_straight_distance_,
            start_rotate_angle_deg_,
            start_turn_angular_speed_
        );
    }

    // 初始化ROS组件（客户端、发布者等）
    void initRosComponents() {
        // 初始化速度发布者
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("cmd_vel发布者已初始化");

       

        // 初始化位姿服务客户端
        ROS_INFO("等待坐标获取服务中...");
        pose_client_ = nh_.serviceClient<ucarmain2026::getpose_server>("/getpose_server");
        pose_.request.getpose_start = 1;
        if (!pose_client_.waitForExistence()) {
            ROS_FATAL("超时未连接到getpose_server服务");
            ros::shutdown();
        }
        ROS_INFO("getpose_server服务已连接");

        // 初始化MoveBase客户端
        ac_ = new MoveBaseClient("move_base", true);
        ROS_INFO("等待movebase服务中...");
        if (!ac_->waitForServer()) {
            ROS_FATAL("超时未连接到move_base服务");
            ros::shutdown();
        }
        ROS_INFO("move_base action server 已连接");

        // 初始化动态配置客户端
        reconfigure_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
        if (!reconfigure_client_.waitForExistence()) {
            ROS_FATAL("超时未连接到动态配置服务");
            ros::shutdown();
        }
        ROS_INFO("动态配置服务已连接");
        configureMoveBaseParameters();

        // 初始化TF监听器
        tf_listener_ = new tf::TransformListener();
        ROS_INFO("TF变换监听器已初始化");
    }

    // 配置move_base参数
    void configureMoveBaseParameters() {
        dynamic_reconfigure::ReconfigureRequest request;
        dynamic_reconfigure::ReconfigureResponse response;
        dynamic_reconfigure::DoubleParameter planner_frequency;
        planner_frequency.name = "planner_frequency";
        planner_frequency.value = 0.0;
        request.config.doubles.push_back(planner_frequency);
        
        if (reconfigure_client_.call(request, response)) {
            ROS_INFO("参数更新成功");
            double new_value;
            if (ros::param::get("/move_base/planner_frequency", new_value)) {
                ROS_INFO("Current planner_frequency: %.2f", new_value);
            }
        } else {
            ROS_ERROR("参数更新失败");
        }
    }

    // 加载相机标定文件
    bool loadCalibrationFile() {
    std::string calibration_file;

    nh_.param<std::string>(
        "/line2_left/calibration_file",
        calibration_file,
        "/home/ucar/ucar_ws_copy/src/line_follow/camera_info/pinhole.yaml"
    );

    ROS_INFO("准备加载相机标定文件：%s", calibration_file.c_str());

    FileStorage fs(calibration_file, FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_ERROR("无法打开标定文件：%s", calibration_file.c_str());
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;

    if (cameraMatrix_.empty() || distCoeffs_.empty()) {
        ROS_ERROR("标定文件内容不完整，缺少 camera_matrix 或 distortion_coefficients");
        return false;
    }

    Mat optimalMatrix = getOptimalNewCameraMatrix(
        cameraMatrix_,
        distCoeffs_,
        Size(640, 480),
        1,
        Size(640, 480)
    );

    initUndistortRectifyMap(
        cameraMatrix_,
        distCoeffs_,
        Mat(),
        optimalMatrix,
        Size(640, 480),
        CV_16SC2,
        map1_,
        map2_
    );

    ROS_INFO("标定文件加载和去畸变初始化完成");
    return true;
}

    // 初始化相机和视频录制
    bool initCameraAndVideo() {
        // 打开相机
        cap_.open("/dev/video0", cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            ROS_ERROR("无法打开相机");
            return false;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 5);

        // 初始化视频录制
        out_.open(output_file_, fourcc_, 5, Size(640, 270));
        if (!out_.isOpened()) {
            ROS_ERROR("无法打开视频输出文件");
            return false;
        }
        ROS_INFO("相机和视频录制初始化完成");
        return true;
    }

    // 服务回调函数（核心逻辑）
    bool line_server_callback(
        line_follow::line_follow::Request& req,
        line_follow::line_follow::Response& resp
    ) {
        Mat image, brightness_threshold_image, cropped, gray_img;

        if (!initCameraAndVideo()) {
            ROS_FATAL("相机或视频初始化失败，节点无法启动");
            return false;
        }

        // 每次服务调用都从“直行离开起始停止线”阶段重新开始。
        start_stage_ = StartStage::STRAIGHT;
        start_line_stable_count_ = 0;
        double_line_ = false;
        right_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;

        // 记录启动时的位置，用实际位移控制初始直行距离。
        pose_.request.getpose_start = 1;
        if (!pose_client_.call(pose_) || pose_.response.pose_at.size() < 3) {
            ROS_ERROR("无法获取起始位姿，取消本次圆弧巡线");
            stopRobot();
            cap_.release();
            out_.release();
            return false;
        }

        start_x_ = pose_.response.pose_at[0];
        start_y_ = pose_.response.pose_at[1];

        ROS_INFO(
            "圆弧巡线启动：起点=(%.3f, %.3f)，先直行 %.3f m",
            start_x_,
            start_y_,
            start_straight_distance_
        );

        while (ros::ok()) {
            // 读取并预处理图像
            cap_.read(image);
            if (image.empty()) {
                continue;
            }

            cropped = image(roi_);
            flip(cropped, cropped, 1);

            vector<Mat> channels;
            split(cropped, channels);
            gray_img = channels[2];

            int brightness_threshold =
                brightness_threshold_calculator(gray_img, cropped);
            (void)brightness_threshold;

            threshold(
                gray_img,
                brightness_threshold_image,
                180,
                255,
                THRESH_BINARY
            );
            threshold_image(gray_img);
            cv::cvtColor(gray_img, cropped, cv::COLOR_GRAY2BGR);

            if (start_stage_ == StartStage::STRAIGHT) {
                // 阶段1：从起始停止线向前直行指定距离。
                if (!pose_client_.call(pose_) ||
                    pose_.response.pose_at.size() < 3) {
                    ROS_WARN_THROTTLE(1.0, "起始直行阶段暂时无法获取位姿");
                    twist_.linear.x = 0.0;
                    twist_.linear.y = 0.0;
                    twist_.angular.z = 0.0;
                } else {
                    const double dx =
                        pose_.response.pose_at[0] - start_x_;
                    const double dy =
                        pose_.response.pose_at[1] - start_y_;
                    const double moved_distance = std::hypot(dx, dy);

                    if (moved_distance < start_straight_distance_) {
                        twist_.linear.x = 0.18;
                        twist_.linear.y = 0.0;
                        twist_.angular.z = 0.0;

                        ROS_INFO_THROTTLE(
                            0.5,
                            "起始直行：%.3f / %.3f m",
                            moved_distance,
                            start_straight_distance_
                        );
                    } else {
                        // 以直行完成时的航向角作为原地左转的起点。
                        rotate_start_yaw_ = pose_.response.pose_at[2];
                        start_stage_ = StartStage::ROTATE_FIXED_ANGLE;

                        twist_.linear.x = 0.0;
                        twist_.linear.y = 0.0;
                        twist_.angular.z =
                            std::abs(start_turn_angular_speed_);

                        ROS_INFO(
                            "起始直行完成：%.3f m，开始原地左转 %.1f 度",
                            moved_distance,
                            start_rotate_angle_deg_
                        );
                    }
                }

                displayStream_.str("");
                displayStream_
                    << "起始直行"
                    << " v=" << twist_.linear.x
                    << " w=" << twist_.angular.z;
                putText(
                    cropped,
                    displayStream_.str(),
                    Point(50, 50),
                    FONT_HERSHEY_SIMPLEX,
                    0.5,
                    Scalar(0, 165, 255),
                    1
                );
                out_.write(cropped);

            } else if (
                start_stage_ == StartStage::ROTATE_FIXED_ANGLE
            ) {
                // 阶段2：原地左转到 YAML 指定角度，此阶段不搜索左边线。
                if (!pose_client_.call(pose_) ||
                    pose_.response.pose_at.size() < 3) {
                    ROS_WARN_THROTTLE(1.0, "固定角度左转阶段暂时无法获取航向角");
                    twist_.linear.x = 0.0;
                    twist_.linear.y = 0.0;
                    twist_.angular.z = 0.0;
                } else {
                    const double current_yaw = pose_.response.pose_at[2];

                    // 航向角差归一化到 [-pi, pi]，避免跨越正负 pi 时计算错误。
                    const double signed_yaw_change = std::atan2(
                        std::sin(current_yaw - rotate_start_yaw_),
                        std::cos(current_yaw - rotate_start_yaw_)
                    );
                    const double turned_left_rad = signed_yaw_change;
                    const double target_rad =
                        start_rotate_angle_deg_ * M_PI / 180.0;

                    if (turned_left_rad < target_rad) {
                        twist_.linear.x = 0.0;
                        twist_.linear.y = 0.0;
                        twist_.angular.z =
                            std::abs(start_turn_angular_speed_);

                        ROS_INFO_THROTTLE(
                            0.3,
                            "原地左转：%.1f / %.1f 度",
                            turned_left_rad * 180.0 / M_PI,
                            start_rotate_angle_deg_
                        );
                    } else {
                        // 转满指定角度后，才开始识别左边线。
                        start_stage_ = StartStage::SEARCH_LEFT_LINE;
                        start_line_stable_count_ = 0;

                        // 找线阶段仍然原地左转，找不到就持续左转。
                        twist_.linear.x = 0.0;
                        twist_.linear.y = 0.0;
                        twist_.angular.z =
                            std::abs(start_turn_angular_speed_);

                        ROS_INFO(
                            "原地左转完成：%.1f 度，开始原地左转寻找左边线",
                            turned_left_rad * 180.0 / M_PI
                        );
                    }
                }

                displayStream_.str("");
                displayStream_
                    << "固定角度左转"
                    << " v=" << twist_.linear.x
                    << " w=" << twist_.angular.z;
                putText(
                    cropped,
                    displayStream_.str(),
                    Point(50, 50),
                    FONT_HERSHEY_SIMPLEX,
                    0.5,
                    Scalar(0, 165, 255),
                    1
                );
                out_.write(cropped);

            } else if (
                start_stage_ == StartStage::SEARCH_LEFT_LINE
            ) {
                // 阶段3：原地持续左转寻找左边线，找不到就一直转。
                vector<Point> start_points =
                    find_track_edge(gray_img, 299, 70, cropped);
                RaceTrack candidate_track;

                const bool found_left_line =
                    trace_edge(
                        gray_img,
                        start_points,
                        candidate_track,
                        cropped
                    ) &&
                    candidate_track.points.size() >= 15;

                if (found_left_line) {
                    start_line_stable_count_++;
                } else {
                    start_line_stable_count_ = 0;
                }

                if (start_line_stable_count_ >= 3) {
                    start_stage_ = StartStage::NORMAL_TRACKING;
                    trace_failed_count_ = 0;
                    integration_ = 0.0;
                    pre_error_ = 0.0;

                    ROS_INFO(
                        "左边线连续稳定识别3帧，进入正常巡线"
                    );

                    // 找到线后由原 PID 巡线立即接管。
                    runNormalTracking(gray_img, cropped);
                } else {
                    // 没找到左边线：保持原地左转，不向前运动。
                    twist_.linear.x = 0.0;
                    twist_.linear.y = 0.0;
                    twist_.angular.z =
                        std::abs(start_turn_angular_speed_);

                    ROS_INFO_THROTTLE(
                        0.5,
                        "原地左转搜线：稳定识别 %d / 3 帧",
                        start_line_stable_count_
                    );

                    displayStream_.str("");
                    displayStream_
                        << "原地左转搜线 "
                        << start_line_stable_count_
                        << "/3"
                        << " v=" << twist_.linear.x
                        << " w=" << twist_.angular.z;
                    putText(
                        cropped,
                        displayStream_.str(),
                        Point(50, 50),
                        FONT_HERSHEY_SIMPLEX,
                        0.5,
                        Scalar(0, 165, 255),
                        1
                    );
                    out_.write(cropped);
                }

            } else {
                // 阶段4：完全恢复原来的巡线状态机。
                if (double_line_) {
                    runDoubleLineTracking(gray_img, cropped);
                } else if (right_point_start_) {
                    runRightPointTracking(gray_img, cropped);
                } else {
                    runNormalTracking(gray_img, cropped);
                }
            }

            cmd_pub_.publish(twist_);

            // 起始停止线阶段禁止停车检测；进入原巡线后才检测终点线。
            // int stop_point_count = 0;
            // if (start_stage_ == StartStage::NORMAL_TRACKING &&
            //     stop_car(gray_img, stop_point_count, cropped)) {
            //     ROS_INFO("巡线结束，触发停车");
            //     twist_.linear.x = 0.0;
            //     twist_.linear.y = 0.0;
            //     twist_.angular.z = 0.0;
            //     cmd_pub_.publish(twist_);
            //     break;
            // }
        }

        // 为下一次服务调用复位状态。
        start_stage_ = StartStage::STRAIGHT;
        start_line_stable_count_ = 0;
        double_line_ = false;
        right_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;

        cap_.release();
        out_.release();
        return true;
    }

    int brightness_threshold_calculator(Mat& gray_img, Mat& visualizeImg) {
        // 水平镜像原右巡线逻辑：从右向左寻找“暗到亮”的最大跳变。
        int max_brightness_change = 0;
        int best_binary_brightness = 180;
        Point threshold_keypoint(320, 180);

        for (int y = 269; y > 100; y--) {
            for (int x = 609; x > 1; x--) {
                int current = static_cast<int>(gray_img.at<uchar>(y, x));
                int next = static_cast<int>(gray_img.at<uchar>(y, x - 1));

                if (next >= 150 && current > 80) {
                    if (next - current >= max_brightness_change) {
                        max_brightness_change = next - current;
                        best_binary_brightness = next - 25;
                        threshold_keypoint = Point(x, y);
                    }
                }
            }
        }

        circle(
            visualizeImg,
            threshold_keypoint,
            7,
            Scalar(0, 255, 255),
            -1
        );
        return best_binary_brightness;
    }


    // 停止机器人
    void stopRobot() {
        twist_.linear.x = 0;
        twist_.linear.y = 0;
        twist_.angular.z = 0;
        cmd_pub_.publish(twist_);
        ros::Duration(0.1).sleep();
    }

    // 双边巡线逻辑
    void runDoubleLineTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        double line_error = double_find(gray_img, cropped);
        
        // PID计算
        integration_ += line_error * 0.03;
        integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
        double diff = line_error - pre_error_;
        diff = clamp(diff, -50.0, 50.0);
        
        // 速度控制
        twist_.linear.x = x_max_ / exp(abs(line_error) / 100.0);
        twist_.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
        pre_error_ = line_error;

        // 显示信息
        displayStream_ << "doubleerror: " << line_error 
                      << " P: " << line_error*p_ 
                      << " I: " << integration_*i_ 
                      << " D: " << diff*d_ 
                      << " 角速度: " << twist_.angular.z;
        putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        out_.write(cropped);
    }

    // 右点追踪逻辑
    void runRightPointTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        if (!point_forward_) {
            // 右点到达底部后，按原逻辑完成转向并切换双边巡线。
            // 这两个值固定在代码中，因此line.yaml不再需要out_forward/out_turn。
            ROS_INFO("右点追踪完成后的转向中");
            twist_.linear.x = 0.075;
            twist_.linear.y = 0.0;
            twist_.angular.z = 1.0;
            out_.write(cropped);
            
            // 旋转到位后切换模式
            pose_client_.call(pose_);
            // ROS_INFO("角度%f,位姿%f",out_turn_angel_,pose_.response.pose_at[2]);
            if (pose_.response.pose_at[2] > out_turn_angel_) {
                right_point_start_ = false;
                double_line_ = true;
                x_max_ = 0.5;
                nh_.getParam("/line2_left/double_P", p_);
                nh_.getParam("/line2_left/double_I", i_);
                nh_.getParam("/line2_left/double_D", d_);
                ROS_INFO("旋转完成，切换双边巡线 (P=%.2f)", p_);
            }
            return;
        }

        // 寻找右点并控制
        Point right_point;
        if (find_right_edge(gray_img, right_point, cropped)) {
            double error_x = right_point.x - 320;
            pointx_integration_ += error_x * 0.02;
            pointx_integration_ = clamp(pointx_integration_, -1.0, 1.0);
            
            // 右点过低时停止前进
            if (right_point.y > 240) {
                point_forward_ = false;
            }

            // PID计算
            double point_diff = error_x - pointx_pre_error_;
            twist_.linear.x = 0.23;
            twist_.angular.z = error_x*rightpoint_p_ + pointx_integration_*rightpoint_I_ + point_diff*rightpoint_D_;
            pointx_pre_error_ = error_x;

            // 显示信息
            displayStream_ << "righterror: " << error_x << " P: " << error_x*rightpoint_p_ << " I: " << pointx_integration_*rightpoint_I_;
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        }
        out_.write(cropped);
    }

    // 正常巡线逻辑
    void runNormalTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        vector<Point> start_points = find_track_edge(gray_img, 299, 70, cropped);
        RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用

        if (trace_edge(gray_img, start_points, racetrack, cropped)) {
            // 成功追踪到赛道
            trace_failed_count_ = 0;
            double line_error = error_calculater(racetrack.points, cropped);
            
            // PID计算
            integration_ += line_error * 0.03;
            integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
            double diff = line_error - pre_error_;
            diff = clamp(diff, -50.0, 50.0);
            
            // 速度控制
            twist_.linear.x = x_max_ / exp(abs(line_error) / 100.0);
            twist_.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
            pre_error_ = line_error;

            // 显示信息
            displayStream_ << "正常误差: " << line_error 
                          << " P: " << line_error*p_ 
                          << " I: " << integration_*i_ 
                          << " D: " << diff*d_ 
                          << " 角速度: " << twist_.angular.z;
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        } else {
            // 保留原代码：左边线连续丢失后，不固定左转，
            // 而是切换到右点追踪，再按原状态机进入双边巡线。
            trace_failed_count_++;
            if (trace_failed_count_ > 5) {
                right_point_start_ = true;
                ROS_INFO("连续追踪左边线失败，切换至右点追踪模式");
            }
        }
        out_.write(cropped);
    }

    // 工具函数：数值 clamping
    template <typename T>
    T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    // 图像处理：阈值化
    void threshold_image(Mat& gray) {
        int adaptive_block = 45;
        int adaptive_c = -15;
        int min_contour_area = 250;

        Mat binary;
        adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, adaptive_block, adaptive_c);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Mat denoised = Mat::zeros(binary.size(), CV_8UC1);
        for (size_t i = 0; i < contours.size(); i++) {
            if (contourArea(contours[i]) > min_contour_area) {
                drawContours(denoised, contours, i, Scalar(255), FILLED);
            }
        }
        gray = denoised.clone();
    }

    // 停车检测
    bool stop_car(Mat& gray, int& point, Mat& visual_img) {
        int white_count = 0;
        for (int y = 254; y >= 227; y--) {
            for (int x = 1; x < 639; x++) {
                if (gray.at<uchar>(y, x) == 255) {
                    white_count++;
                    circle(visual_img, Point(x, y), 2, Scalar(0, 0, 0), -1);
                }
            }
        }
        point = white_count;
        return white_count > 2058;
    }

    // 寻找赛道边缘起点
    vector<Point> find_track_edge(
        Mat& gray_img,
        int bottom_trace_end,
        int left_trace_end,
        Mat& visual_img
    ) {
        bool is_now_white = false;
        vector<Point> maybe_start_point;

        // 底部寻找：从左向右扫描图像左半区，记录白线的右边缘。
        for (int i = 0; i < bottom_trace_end; i++) {
            if (!is_now_white && gray_img.at<uchar>(269, i) == 255) {
                is_now_white = true;
            }
            if (is_now_white && gray_img.at<uchar>(269, i) == 0) {
                maybe_start_point.emplace_back(i - 1, 269);
                circle(
                    visual_img,
                    Point(i - 1, 269),
                    5,
                    Scalar(0, 0, 255),
                    -1
                );
                is_now_white = false;
            }
        }

        // 左部寻找：与原右侧边界搜索完全对称。
        is_now_white = true;
        for (int i = 269; i > left_trace_end; i--) {
            if (is_now_white && gray_img.at<uchar>(i, 0) == 0) {
                is_now_white = false;
            }
            if (!is_now_white && gray_img.at<uchar>(i, 0) == 255) {
                maybe_start_point.emplace_back(0, i);
                circle(
                    visual_img,
                    Point(0, i),
                    5,
                    Scalar(0, 0, 255),
                    -1
                );
                is_now_white = true;
            }
        }

        return maybe_start_point;
    }


    // 追踪赛道边缘（修正参数：将int& racetrack改为RaceTrack& racetrack）
    bool trace_edge(
        Mat& gray_img,
        vector<Point> start_points,
        RaceTrack& racetrack,
        Mat& visual_img
    ) {
        int point_number = static_cast<int>(start_points.size());
        vector<RaceTrack> racetracks(point_number);
        int width = gray_img.cols;
        int search_range = 40;

        for (int idx = 0; idx < point_number; idx++) {
            bool last_right = true;
            bool last_left = false;
            int fail_count = 0;
            Point start = start_points[idx];
            int center_x = start.x;
            int center_y = start.y - 1;

            while (center_y > start.y - 100) {
                bool right_found = false;
                bool left_found = false;

                for (int dx = 0; dx <= search_range / 2; dx++) {
                    int cand_right = center_x + dx;
                    int cand_left = center_x - dx;
                    bool right_check = (cand_right < width - 1);
                    bool left_check = (cand_left > 0);

                    // 在当前位置右侧找到白到黑跳变：白线右边缘。
                    if (right_check &&
                        gray_img.at<uchar>(center_y, cand_right) == 255 &&
                        gray_img.at<uchar>(center_y, cand_right + 1) == 0) {
                        racetracks[idx].points.emplace_back(
                            cand_right,
                            center_y
                        );
                        right_found = true;
                        left_found = false;
                        center_x = cand_right;
                        break;
                    }

                    // 从当前位置左侧反向找到同一个白到黑边缘。
                    if (left_check &&
                        gray_img.at<uchar>(center_y, cand_left) == 0 &&
                        gray_img.at<uchar>(center_y, cand_left - 1) == 255) {
                        right_found = false;
                        left_found = true;
                        center_x = cand_left - 1;
                        break;
                    }
                }

                if (last_right && left_found) {
                    racetracks[idx].direction_change++;
                    last_right = false;
                    last_left = true;
                }
                if (last_left && right_found) {
                    racetracks[idx].direction_change++;
                    last_right = true;
                    last_left = false;
                }

                if (right_found || left_found) {
                    fail_count = 0;
                    center_y--;
                } else {
                    fail_count++;
                    center_y--;
                    if (fail_count >= 4) {
                        break;
                    }
                }

                if (center_y <= 0 || racetracks[idx].points.size() > 60) {
                    break;
                }
            }

            if (racetracks[idx].points.size() > 15) {
                Vec4f line_params;
                fitLine(
                    racetracks[idx].points,
                    line_params,
                    DIST_L2,
                    0,
                    0.01,
                    0.01
                );
                racetracks[idx].slope = line_params[1] / line_params[0];
            } else {
                racetracks[idx].slope = 2.0;
            }
        }

        // 原右巡线斜率判据做水平镜像：slope -> -slope。
        int best_idx = -1;
        float min_dangerous = 2.1f;
        for (int i = 0; i < point_number; i++) {
            if (racetracks[i].points.empty()) {
                continue;
            }

            if (!(racetracks[i].slope > -0.05 &&
                  racetracks[i].slope < 10.0)) {
                float ratio =
                    racetracks[i].direction_change /
                    static_cast<float>(racetracks[i].points.size());

                if (ratio < min_dangerous) {
                    min_dangerous = ratio;
                    best_idx = i;
                }
            }
        }

        if (best_idx == -1) {
            return false;
        }

        racetrack = racetracks[best_idx];
        for (const auto& point : racetrack.points) {
            circle(visual_img, point, 2, Scalar(0, 255, 0), -1);
        }

        ostringstream oss;
        oss << "斜率: " << racetrack.slope
            << " 方向变化: " << racetrack.direction_change;
        putText(
            visual_img,
            oss.str(),
            Point(50, 100),
            FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar(255, 255, 0),
            1
        );
        return true;
    }


    // 寻找右边缘
    bool find_right_edge(
        Mat gray_img,
        Point& right_point,
        Mat& visualizeImg
    ) {
        bool is_now_white = false;
        vector<Point> maybe_right_point;

        // 在图像右侧寻找起点；x=634 是原 x=5 的水平镜像位置。
        for (int i = 269; i > 2; i--) {
            if (is_now_white && gray_img.at<uchar>(i, 634) == 0) {
                is_now_white = false;
            }
            if (!is_now_white && gray_img.at<uchar>(i, 634) == 255) {
                maybe_right_point.emplace_back(634, i);
                circle(
                    visualizeImg,
                    Point(634, i),
                    9,
                    Scalar(255, 0, 0),
                    -1
                );
                is_now_white = true;
            }
        }

        int point_number = static_cast<int>(maybe_right_point.size());
        vector<RaceTrack> racetracks(point_number);
        int search_range = 40;

        // 从右向左追踪右点边缘。
        for (int idx = 0; idx < point_number; idx++) {
            bool last_up = false;
            bool last_down = false;
            int fail_count = 0;
            Point start = maybe_right_point[idx];
            int center_x = start.x - 1;
            int center_y = start.y;

            while (center_x > 19) {
                bool found = false;

                for (int dy = 0; dy <= search_range / 2; dy++) {
                    bool up_check = (center_y - dy > 2);
                    bool down_check = (center_y + dy < 268);

                    if (down_check &&
                        gray_img.at<uchar>(center_y + dy, center_x) == 255 &&
                        gray_img.at<uchar>(center_y + dy + 1, center_x) == 0) {
                        racetracks[idx].points.emplace_back(
                            center_x,
                            center_y + dy
                        );
                        found = true;
                        center_y += dy;
                        if (last_up) {
                            racetracks[idx].direction_change++;
                        }
                        last_down = true;
                        last_up = false;
                        break;
                    }

                    if (!found && up_check &&
                        gray_img.at<uchar>(center_y - dy, center_x) == 255 &&
                        gray_img.at<uchar>(center_y - dy + 1, center_x) == 0) {
                        racetracks[idx].points.emplace_back(
                            center_x - 1,
                            center_y - dy
                        );
                        found = true;
                        center_y -= dy;
                        if (last_down) {
                            racetracks[idx].direction_change++;
                        }
                        last_down = false;
                        last_up = true;
                        break;
                    }
                }

                if (found) {
                    fail_count = 0;
                } else {
                    fail_count++;
                    if (fail_count >= 10) {
                        break;
                    }
                }
                center_x--;
            }

            if (racetracks[idx].points.size() > 120) {
                racetracks[idx].right_point = true;
            }
        }

        int best_idx = -1;
        int lowest_y = 0;
        for (int i = 0; i < point_number; i++) {
            if (racetracks[i].right_point &&
                !racetracks[i].points.empty() &&
                racetracks[i].points[0].y > lowest_y) {
                lowest_y = racetracks[i].points[0].y;
                best_idx = i;
            }
        }

        if (best_idx == -1) {
            return false;
        }

        RaceTrack racetrack = racetracks[best_idx];
        Point best_point(639, 0);
        for (size_t i = 0; i < racetrack.points.size(); i += 3) {
            if (racetrack.points[i].y > best_point.y) {
                best_point = racetrack.points[i];
            }
            circle(
                visualizeImg,
                racetrack.points[i],
                2,
                Scalar(255, 0, 0),
                -1
            );
        }

        circle(
            visualizeImg,
            best_point,
            9,
            Scalar(0, 0, 255),
            -1
        );
        right_point = best_point;

        ostringstream oss;
        oss << "右点: (" << best_point.x << "," << best_point.y << ")";
        putText(
            visualizeImg,
            oss.str(),
            Point(50, 100),
            FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar(255, 255, 0),
            1
        );
        return true;
    }


    // 双边巡线误差计算
    double double_find(Mat gray_img, Mat& visual_img) {
        vector<int> left_total;
        vector<int> right_total;
        double error = 0.0;
        vector<Point> mid_points;

        bool found_once = false;
        int failed = 0;

        // 提取左边界。
        for (int y = 269; y >= 50; y--) {
            int left = 0;
            bool found = false;
            for (int x = 319; x > 1; x--) {
                if (gray_img.at<uchar>(y, x) == 255) {
                    left = x;
                    found = true;
                    found_once = true;
                    failed = 0;
                    break;
                }
            }
            if (found_once && !found && ++failed > 10) {
                break;
            }
            left_total.push_back(left);
        }

        found_once = false;
        failed = 0;

        // 提取右边界。
        for (int y = 269; y >= 50; y--) {
            int right = 639;
            bool found = false;
            for (int x = 319; x < 639; x++) {
                if (gray_img.at<uchar>(y, x) == 255) {
                    right = x;
                    found = true;
                    found_once = true;
                    failed = 0;
                    break;
                }
            }
            if (found_once && !found && ++failed > 10) {
                break;
            }
            right_total.push_back(right);
        }

        float row = static_cast<float>(
            min(left_total.size(), right_total.size())
        );
        if (row <= 0.0f) {
            return 0.0;
        }

        for (int i = 0; i < static_cast<int>(row); i++) {
            error +=
                (640 - (left_total[i] + right_total[i])) *
                (1.0 - static_cast<double>(i) / row);
        }

        int min_size = static_cast<int>(
            min(left_total.size(), right_total.size())
        );
        for (int i = 0; i < min_size; i++) {
            int mid_x = (left_total[i] + right_total[i]) / 2;
            int y = 269 - i;
            mid_points.emplace_back(mid_x, y);
            circle(
                visual_img,
                Point(mid_x, y),
                1,
                Scalar(0, 255, 255),
                -1
            );
        }

        return error / row;
    }


    // 误差计算
    double error_calculater(vector<Point>& traced_points, Mat& visualizeImg) {
        if (traced_points.empty()) {
            return 100.0;
        }

        double total_error = 0.0;

        for (size_t i = 0; i < traced_points.size(); i++) {
            double y = static_cast<double>(traced_points[i].y);

            // 今年二代车相机标定结果
            double center_offset =
                center_distance + (y - 140.0) * 1.40;

            // 根据左侧边线推算赛道中线
            double estimated_center_x =
                traced_points[i].x + center_offset;

            double weight;
            if (i <= 30) {
                weight = 1.0 - static_cast<double>(i) / 100.0;
            } else {
                weight = 0.7 *
                    exp(-0.064 * (static_cast<double>(i) - 30.0));
            }

            total_error +=
                (estimated_center_x - 320.0) * weight;
        }

        // 绘制推算出的中线
        for (const auto& point : traced_points) {
            double y = static_cast<double>(point.y);
            double center_offset =
                center_distance + (y - 140.0) * 1.40;

            Point center_point(
                cvRound(point.x + center_offset),
                point.y
            );

            circle(
                visualizeImg,
                center_point,
                3,
                Scalar(0, 255, 0),
                -1
            );
        }

        return -total_error / traced_points.size();
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "line2_left");
    
    // 创建节点对象（构造函数中完成所有初始化）
    LineFollowerNode node;
    
    // 运行节点
    node.run();
    
    return 0;
}