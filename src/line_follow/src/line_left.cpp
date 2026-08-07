// 版本：寻左线丢线左转 + AMCL定位触发 + 纯PP全向位姿停靠版 V9（2026-08-06）
// 唯一校验标识：LINE_LEFT_TRUE_MIRROR_AMCL_PP_V9
// 本文件关键标识：x > 4.25，且距终点(4.75, 0.25) < 0.75m；最终方向0°。
// 巡线基线：用户此前提供并实测的原始line_left（寻左线、丢线左转）。
// 保留初始直行找线，并直接沿用原line_left的左线搜索、误差计算和丢线左转逻辑；
// 已彻底移除原视觉终止判定和旧停车控制模块。
// 每次巡线服务启动前先向 /initialpose 发布指定AMCL初始位姿；
// 满足定位触发条件后不发布零速度，直接无缝切入终点纯PP位姿停靠。

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <random>
#include <string>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <clocale>
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
    double slope;                  // 赛道斜率
    vector<Point> points;          // 赛道点集
    int direction_change;          // 方向变化次数
    int slope_change_count;        // 斜率变化次数
    bool right_point;              // 是否为右赛道标志
};

class LineFollowerNode {
private:
    // ROS核心组件
    ros::NodeHandle nh_;                  // 节点句柄
    ros::ServiceServer line_server_;      // 服务端
    ros::Publisher cmd_pub_;              // 速度发布者
    ros::Publisher initial_pose_pub_;     // AMCL初始位姿发布者

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

    // 图像局部自适应二值化参数（由 line_left.yaml 配置）
    int adaptive_block_;                  // 自适应阈值邻域大小，必须为大于1的奇数
    int adaptive_c_;                      // 自适应阈值常数 C
    int min_contour_area_;                // 二值化后保留轮廓的最小面积

    // 控制参数
    double p_, i_, d_;                    // PID参数
    double leftpoint_p_, leftpoint_I_, leftpoint_D_; // 右点控制参数（沿用原YAML键名）
    double x_max_, integration_limit_;    // 速度和积分限制
    double out_turn_, out_forward_,out_turn_angel_;       // 旋转和前进参数
    double integration_, pre_error_;      // 积分和前向误差
    double pointx_integration_, pointx_pre_error_; // 右点积分和前向误差

    // AMCL初始位姿参数
    string map_frame_;
    string base_frame_;
    double initial_pose_x_;
    double initial_pose_y_;
    double initial_pose_yaw_deg_;
    double initial_pose_covariance_xy_;
    double initial_pose_covariance_yaw_;
    int initial_pose_publish_count_;
    double initial_pose_publish_interval_;

    // 定位触发与终点纯PP位姿停靠参数
    double docking_trigger_min_x_;
    double docking_trigger_distance_;
    double docking_goal_x_;
    double docking_goal_y_;
    double docking_goal_yaw_deg_;
    double docking_control_rate_;
    double docking_position_tolerance_;
    double docking_yaw_tolerance_;
    double docking_linear_x_gain_;
    double docking_linear_y_gain_;
    double docking_angular_gain_;
    double docking_min_linear_speed_;
    double docking_min_angular_speed_;
    double docking_max_vel_x_;
    double docking_max_vel_y_;
    double docking_max_vel_theta_;
    double docking_acc_lim_x_;
    double docking_acc_lim_y_;
    double docking_acc_lim_theta_;

    // 状态变量

    bool double_line_;                    // 双边巡线标志
    bool right_point_start_;              // 右点追踪标志
    bool point_forward_;                  // 右点前进标志
    int trace_failed_count_;              // 追踪失败计数

public:
    // 构造函数：初始化所有组件
    LineFollowerNode() : 
        nh_(""),
        tf_listener_(nullptr),
        ac_(nullptr),
        output_file_("/home/ucar/ucar_ws_copy/src/line_follow/image/line_left.avi"),
        fourcc_(VideoWriter::fourcc('X', 'V', 'I', 'D')),
        roi_(0, 210, 640, 270),

        double_line_(false),
        right_point_start_(false),
        point_forward_(true),
        trace_failed_count_(0),
        integration_(0), 
        pre_error_(0),
        pointx_integration_(0),
        pointx_pre_error_(0) {

        ROS_INFO("启动 line_left V9（寻左线、丢线左转、AMCL定位触发、纯PP全向位姿停靠）");

        // 1. 初始化服务端（优先初始化）
        line_server_ = nh_.advertiseService("line_left", &LineFollowerNode::line_server_callback, this);
        ROS_INFO("line_left服务已初始化");

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
        // 服务回调会长时间执行巡线与停靠控制，使用异步Spinner保持ROS通信。
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    }

private:
    // 加载ROS参数
    void loadParameters() {
        nh_.getParam("/line_left/right_P", p_);
        nh_.getParam("/line_left/right_I", i_);
        nh_.getParam("/line_left/right_D", d_);
        nh_.getParam("/line_left/leftpoint_p", leftpoint_p_);
        nh_.getParam("/line_left/leftpoint_I", leftpoint_I_);
        nh_.getParam("/line_left/leftpoint_D", leftpoint_D_);
        nh_.getParam("/line_left/x_max_", x_max_);
        nh_.getParam("/line_left/integration_limit", integration_limit_);
        nh_.getParam("/line_left/out_forward", out_forward_);
        nh_.getParam("/line_left/out_turn", out_turn_);
        nh_.getParam("/line_left/out_turn_angel", out_turn_angel_);
        nh_.getParam("/line_left/center_distance", center_distance);

        // 图像局部自适应二值化参数
        // YAML 中未填写时保持原代码默认值：45、-15、250。
        nh_.param("/line_left/adaptive_block", adaptive_block_, 45);
        nh_.param("/line_left/adaptive_c", adaptive_c_, -15);
        nh_.param("/line_left/min_contour_area", min_contour_area_, 250);

        // AMCL初始位姿。每次服务真正启动巡线前都会重新读取并发布。
        nh_.param<string>("/line_left/map_frame", map_frame_, "map");
        nh_.param<string>("/line_left/base_frame", base_frame_, "base_link");
        nh_.param("/line_left/initial_pose_x", initial_pose_x_, 2.50);
        nh_.param("/line_left/initial_pose_y", initial_pose_y_, 2.60);
        nh_.param("/line_left/initial_pose_yaw_deg", initial_pose_yaw_deg_, -90.0);
        nh_.param("/line_left/initial_pose_covariance_xy",
                  initial_pose_covariance_xy_, 0.01);
        nh_.param("/line_left/initial_pose_covariance_yaw",
                  initial_pose_covariance_yaw_, 0.01);
        nh_.param("/line_left/initial_pose_publish_count",
                  initial_pose_publish_count_, 3);
        nh_.param("/line_left/initial_pose_publish_interval",
                  initial_pose_publish_interval_, 0.10);

        // 巡线终止触发条件与固定终点。
        nh_.param("/line_left/docking_trigger_min_x",
                  docking_trigger_min_x_, 4.25);
        nh_.param("/line_left/docking_trigger_distance",
                  docking_trigger_distance_, 0.75);
        nh_.param("/line_left/docking_goal_x", docking_goal_x_, 4.75);
        nh_.param("/line_left/docking_goal_y", docking_goal_y_, 0.25);
        nh_.param("/line_left/docking_goal_yaw_deg",
                  docking_goal_yaw_deg_, 0.0);

        // 终点纯PP位姿控制，参数与已调好的line2_right保持一致。
        nh_.param("/line_left/docking_control_rate",
                  docking_control_rate_, 30.0);
        nh_.param("/line_left/docking_position_tolerance",
                  docking_position_tolerance_, 0.025);
        nh_.param("/line_left/docking_yaw_tolerance",
                  docking_yaw_tolerance_, 0.05);
        nh_.param("/line_left/docking_linear_x_gain",
                  docking_linear_x_gain_, 2.50);
        nh_.param("/line_left/docking_linear_y_gain",
                  docking_linear_y_gain_, 1.20);
        nh_.param("/line_left/docking_angular_gain",
                  docking_angular_gain_, 1.50);
        nh_.param("/line_left/docking_min_linear_speed",
                  docking_min_linear_speed_, 0.10);
        nh_.param("/line_left/docking_min_angular_speed",
                  docking_min_angular_speed_, 0.010);
        nh_.param("/line_left/docking_max_vel_x",
                  docking_max_vel_x_, 0.90);
        nh_.param("/line_left/docking_max_vel_y",
                  docking_max_vel_y_, 0.40);
        nh_.param("/line_left/docking_max_vel_theta",
                  docking_max_vel_theta_, 0.90);
        nh_.param("/line_left/docking_acc_lim_x",
                  docking_acc_lim_x_, 2.00);
        nh_.param("/line_left/docking_acc_lim_y",
                  docking_acc_lim_y_, 2.00);
        nh_.param("/line_left/docking_acc_lim_theta",
                  docking_acc_lim_theta_, 8.00);

        initial_pose_covariance_xy_ =
            std::max(0.0, initial_pose_covariance_xy_);
        initial_pose_covariance_yaw_ =
            std::max(0.0, initial_pose_covariance_yaw_);
        initial_pose_publish_count_ =
            std::max(1, initial_pose_publish_count_);
        initial_pose_publish_interval_ =
            std::max(0.0, initial_pose_publish_interval_);
        docking_trigger_distance_ =
            std::max(0.0, docking_trigger_distance_);
        docking_control_rate_ =
            std::max(1.0, docking_control_rate_);
        docking_position_tolerance_ =
            std::max(0.001, docking_position_tolerance_);
        docking_yaw_tolerance_ =
            std::max(0.001, docking_yaw_tolerance_);
        docking_linear_x_gain_ =
            std::max(0.0, docking_linear_x_gain_);
        docking_linear_y_gain_ =
            std::max(0.0, docking_linear_y_gain_);
        docking_angular_gain_ =
            std::max(0.0, docking_angular_gain_);
        docking_min_linear_speed_ =
            std::max(0.0, docking_min_linear_speed_);
        docking_min_angular_speed_ =
            std::max(0.0, docking_min_angular_speed_);
        docking_max_vel_x_ =
            std::max(0.001, docking_max_vel_x_);
        docking_max_vel_y_ =
            std::max(0.001, docking_max_vel_y_);
        docking_max_vel_theta_ =
            std::max(0.001, docking_max_vel_theta_);
        docking_min_linear_speed_ =
            std::min(docking_min_linear_speed_,
                     std::min(docking_max_vel_x_, docking_max_vel_y_));
        docking_min_angular_speed_ =
            std::min(docking_min_angular_speed_,
                     docking_max_vel_theta_);
        docking_acc_lim_x_ = std::max(0.0, docking_acc_lim_x_);
        docking_acc_lim_y_ = std::max(0.0, docking_acc_lim_y_);
        docking_acc_lim_theta_ = std::max(0.0, docking_acc_lim_theta_);

        ROS_INFO(
            "line_left参数加载完成：center_distance=%d，二值化=(%d,%d,%d)",
            center_distance,
            adaptive_block_, adaptive_c_, min_contour_area_);
        ROS_INFO(
            "AMCL初始位姿=(%.3f, %.3f, %.1f°)，"
            "停靠触发：x>%.3f且距(%.3f, %.3f)<%.3fm，最终方向=%.1f°",
            initial_pose_x_, initial_pose_y_, initial_pose_yaw_deg_,
            docking_trigger_min_x_,
            docking_goal_x_, docking_goal_y_,
            docking_trigger_distance_,
            docking_goal_yaw_deg_);
    }

    // 初始化ROS组件（客户端、发布者等）
    void initRosComponents() {
        // 初始化速度发布者
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("cmd_vel发布者已初始化");

        initial_pose_pub_ =
            nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "/initialpose", 1, false);
        ROS_INFO("AMCL初始位姿发布者已初始化：/initialpose");

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
        "/line_left/calibration_file",
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
    bool line_server_callback(line_follow::line_follow::Request& req, line_follow::line_follow::Response& resp) {
        Mat image, brightness_threshold_image, cropped, gray_img;
        bool switch_to_docking = false;

        // 每次服务启动时重新读取rosparam，允许运行期间修改坐标与停靠参数。
        loadParameters();

        // 清除上一次服务调用遗留的巡线状态。
        double_line_ = false;
        right_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();

        // 每次启动巡线服务时，先进入“初始直行找线”状态。
        // 该状态只会退出一次；检测到有效左边线后，后续丢线执行原line_left的固定左转逻辑。
        bool initial_straight_mode = true;

        // 小车尚未开始移动时，强制设置本次巡线使用的AMCL初始位姿。
        publishInitialPose();

        // 5. 初始化相机和视频录制
        if (!initCameraAndVideo()) {
            ROS_FATAL("相机或视频初始化失败，节点无法启动");
            stopRobot();
            return false;
        }

        ROS_INFO("进入初始直行找线状态：以 x_max_=%.3f m/s 直行，检测到有效白线后进入正常巡线",
                 x_max_);

        while (ros::ok()) {
            // 两项条件同时满足后，在同一控制周期直接切入停靠，不插入零速帧。
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (getRobotPose(robot_x, robot_y, robot_yaw)) {
                const double distance_to_goal =
                    std::hypot(docking_goal_x_ - robot_x,
                               docking_goal_y_ - robot_y);

                if (robot_x > docking_trigger_min_x_ &&
                    distance_to_goal < docking_trigger_distance_) {
                    switch_to_docking = true;
                    ROS_INFO(
                        "满足停靠触发条件：当前位置=(%.3f, %.3f)，x>%.3f，"
                        "距终点=%.3fm<%.3fm；无停顿切入纯PP停靠。",
                        robot_x, robot_y,
                        docking_trigger_min_x_,
                        distance_to_goal,
                        docking_trigger_distance_);
                    break;
                }
            }

            // 读取并预处理图像
            cap_.read(image);
            if (image.empty()) continue;
            cropped = image(roi_);
            flip(cropped, cropped, 1); // 翻转图像
            vector<Mat> channels;
            split(cropped, channels);
            gray_img = channels[2]; // 红色通道作为灰度图
            int brightness_threshold = brightness_threshold_calculator(gray_img,cropped);
            threshold(gray_img, brightness_threshold_image, 180, 255, THRESH_BINARY);
            threshold_image(gray_img);
            
            // imshow("test",gray_img);
            // waitKey(0);
            cv::cvtColor(gray_img, cropped, cv::COLOR_GRAY2BGR);

            if (initial_straight_mode) {
                // 初始阶段复用原line_left的左线识别，但禁止执行“丢线左转”。
                // 未找到有效左边线时，runNormalTracking() 会强制保持 x_max_ 直行；
                // 找到后则在当前帧直接输出正常PID巡线速度。
                if (runNormalTracking(gray_img, cropped, false)) {
                    initial_straight_mode = false;
                    trace_failed_count_ = 0;
                    ROS_INFO("初始直行阶段检测到有效白线，退出初始状态并进入正常巡线");
                }

                cmd_pub_.publish(twist_);
                continue;
            }

            // 新场地只保留左边巡线模式。
            // 正常情况下跟踪左侧边线；左线连续丢失后，直接执行固定左转。
            runNormalTracking(gray_img, cropped, true);

            // 发布速度指令
            cmd_pub_.publish(twist_);
        }

        if (ros::ok() && switch_to_docking) {
            runDockingControl();
        } else {
            stopRobot();
        }

        // 为下一次服务调用复位状态。
        double_line_ = false;
        right_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();
        cap_.release();
        out_.release();
        return true;
    }

    int brightness_threshold_calculator(Mat& gray_img,Mat& visualizeImg){//寻找跳变最剧烈的那个点，这个点的左值就是图像二值化阈值
        int max_brightness_change = 0;
        int best_binary_brightness = 180;//给个默认值，别一会没找到
        Point threshold_keypoint;
        for (int y = 269; y > 100; y--) {
            for (int x = 30; x < 638; x++) {
                int current = (int)gray_img.at<uchar>(y, x);
                int next = (int)gray_img.at<uchar>(y, x + 1);
                if (next>=150&&current>80){   
                    if (next - current >= max_brightness_change) {
                        max_brightness_change = next - current;
                        best_binary_brightness = next-25;
                        threshold_keypoint = Point(x,y);
                    }
                }
            }
        }
        circle(visualizeImg, threshold_keypoint, 7, Scalar(0, 255, 255), -1);
        return best_binary_brightness;
    }

    
    // 停止机器人
    void stopRobot() {
        twist_.linear.x = 0;
        twist_.linear.y = 0;
        twist_.angular.z = 0;
        cmd_pub_.publish(twist_);
    }

    static double normalizeAngle(double angle) {
        const double pi = 3.14159265358979323846;
        while (angle > pi) {
            angle -= 2.0 * pi;
        }
        while (angle < -pi) {
            angle += 2.0 * pi;
        }
        return angle;
    }

    static double applyMinimumMagnitude(double value, double minimum) {
        if (std::abs(value) < 1e-9 || minimum <= 0.0) {
            return value;
        }
        return std::copysign(std::max(std::abs(value), minimum), value);
    }

    void publishInitialPose() {
        const double pi = 3.14159265358979323846;
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.frame_id = map_frame_;
        initial_pose.pose.pose.position.x = initial_pose_x_;
        initial_pose.pose.pose.position.y = initial_pose_y_;
        initial_pose.pose.pose.position.z = 0.0;
        initial_pose.pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(initial_pose_yaw_deg_ * pi / 180.0);

        std::fill(initial_pose.pose.covariance.begin(),
                  initial_pose.pose.covariance.end(),
                  0.0);
        initial_pose.pose.covariance[0] = initial_pose_covariance_xy_;
        initial_pose.pose.covariance[7] = initial_pose_covariance_xy_;
        initial_pose.pose.covariance[35] = initial_pose_covariance_yaw_;

        for (int i = 0;
             ros::ok() && i < initial_pose_publish_count_;
             ++i) {
            initial_pose.header.stamp = ros::Time::now();
            initial_pose_pub_.publish(initial_pose);

            if (i + 1 < initial_pose_publish_count_ &&
                initial_pose_publish_interval_ > 0.0) {
                ros::Duration(initial_pose_publish_interval_).sleep();
            }
        }

        ROS_INFO(
            "已向/initialpose发布AMCL初始位姿：x=%.3f，y=%.3f，yaw=%.1f°（%d次）",
            initial_pose_x_, initial_pose_y_, initial_pose_yaw_deg_,
            initial_pose_publish_count_);
    }

    bool getRobotPose(double& x, double& y, double& yaw) {
        if (tf_listener_ == nullptr) {
            ROS_ERROR_THROTTLE(1.0, "TF监听器尚未初始化，无法读取AMCL定位。");
            return false;
        }

        try {
            tf::StampedTransform transform;
            tf_listener_->lookupTransform(
                map_frame_, base_frame_, ros::Time(0), transform);

            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            yaw = tf::getYaw(transform.getRotation());
            return std::isfinite(x) &&
                   std::isfinite(y) &&
                   std::isfinite(yaw);
        } catch (const tf::TransformException& ex) {
            ROS_WARN_THROTTLE(
                1.0,
                "读取AMCL定位失败（%s -> %s）：%s",
                map_frame_.c_str(),
                base_frame_.c_str(),
                ex.what());
            return false;
        }
    }

    bool computeDockingCommand(
        double robot_x,
        double robot_y,
        double robot_yaw,
        geometry_msgs::Twist& desired_cmd) {
        desired_cmd = geometry_msgs::Twist();

        const double dx_map = docking_goal_x_ - robot_x;
        const double dy_map = docking_goal_y_ - robot_y;
        const double distance_error = std::hypot(dx_map, dy_map);

        // 将地图坐标系位置误差转换到车体坐标系，与局部规划器终点
        // final_pose位于base_link中的含义一致。
        const double cos_yaw = std::cos(robot_yaw);
        const double sin_yaw = std::sin(robot_yaw);
        const double x_error =
            cos_yaw * dx_map + sin_yaw * dy_map;
        const double y_error =
            -sin_yaw * dx_map + cos_yaw * dy_map;

        const double pi = 3.14159265358979323846;
        const double target_yaw =
            docking_goal_yaw_deg_ * pi / 180.0;
        const double yaw_error =
            normalizeAngle(target_yaw - robot_yaw);

        if (distance_error <= docking_position_tolerance_ &&
            std::abs(yaw_error) <= docking_yaw_tolerance_) {
            ROS_WARN(
                "停靠完成：当前位置=(%.3f, %.3f)，位置误差=%.3fm，"
                "方向误差=%.3frad。",
                robot_x, robot_y, distance_error, yaw_error);
            return true;
        }

        double vx = docking_linear_x_gain_ * x_error;
        double vy = docking_linear_y_gain_ * y_error;
        double wz = docking_angular_gain_ * yaw_error;

        if (std::abs(x_error) <=
            docking_position_tolerance_ * 0.65) {
            vx = 0.0;
        }
        if (std::abs(y_error) <=
            docking_position_tolerance_ * 0.65) {
            vy = 0.0;
        }
        if (std::abs(yaw_error) <= docking_yaw_tolerance_) {
            wz = 0.0;
        }

        vx = applyMinimumMagnitude(vx, docking_min_linear_speed_);
        vy = applyMinimumMagnitude(vy, docking_min_linear_speed_);
        wz = applyMinimumMagnitude(wz, docking_min_angular_speed_);

        desired_cmd.linear.x =
            clamp(vx, -docking_max_vel_x_, docking_max_vel_x_);
        desired_cmd.linear.y =
            clamp(vy, -docking_max_vel_y_, docking_max_vel_y_);
        desired_cmd.angular.z =
            clamp(wz,
                  -docking_max_vel_theta_,
                  docking_max_vel_theta_);
        return false;
    }

    void applyDockingAccelerationLimits(
        const geometry_msgs::Twist& desired_cmd,
        double dt) {
        dt = clamp(dt, 0.01, 0.20);

        const double max_delta_x = docking_acc_lim_x_ * dt;
        const double max_delta_y = docking_acc_lim_y_ * dt;
        const double max_delta_theta = docking_acc_lim_theta_ * dt;

        // 以巡线最后一次实际指令twist_为初值，保证接管时速度连续，
        // 不先清零，也不产生人为停顿。
        twist_.linear.x = docking_acc_lim_x_ > 0.0
            ? clamp(desired_cmd.linear.x,
                    twist_.linear.x - max_delta_x,
                    twist_.linear.x + max_delta_x)
            : desired_cmd.linear.x;
        twist_.linear.y = docking_acc_lim_y_ > 0.0
            ? clamp(desired_cmd.linear.y,
                    twist_.linear.y - max_delta_y,
                    twist_.linear.y + max_delta_y)
            : desired_cmd.linear.y;
        twist_.angular.z = docking_acc_lim_theta_ > 0.0
            ? clamp(desired_cmd.angular.z,
                    twist_.angular.z - max_delta_theta,
                    twist_.angular.z + max_delta_theta)
            : desired_cmd.angular.z;
    }

    bool runDockingControl() {
        ros::Rate control_rate(docking_control_rate_);
        ros::Time last_control_time = ros::Time::now();

        while (ros::ok()) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;

            if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
                ROS_ERROR("停靠阶段无法获取AMCL定位，安全停车。");
                stopRobot();
                return false;
            }

            geometry_msgs::Twist desired_cmd;
            if (computeDockingCommand(
                    robot_x, robot_y, robot_yaw, desired_cmd)) {
                stopRobot();
                return true;
            }

            const ros::Time now = ros::Time::now();
            double dt = (now - last_control_time).toSec();
            last_control_time = now;
            applyDockingAccelerationLimits(desired_cmd, dt);
            cmd_pub_.publish(twist_);

            ROS_INFO_THROTTLE(
                0.5,
                "纯PP停靠中：当前位置=(%.3f, %.3f, %.1f°)，"
                "cmd=(%.3f, %.3f, %.3f)",
                robot_x, robot_y,
                robot_yaw * 180.0 / 3.14159265358979323846,
                twist_.linear.x,
                twist_.linear.y,
                twist_.angular.z);

            control_rate.sleep();
        }

        stopRobot();
        return false;
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

    // 右点追踪逻辑：直接沿用原始line_left实现
    void runRightPointTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        if (!point_forward_) {
            // 丢线旋转
            ROS_INFO("丢线旋转中");
            twist_.linear.x = out_forward_;
            twist_.angular.z = -out_turn_;
            out_.write(cropped);
            
            // 旋转到位后切换模式
            pose_client_.call(pose_);
            // ROS_INFO("角度%f,位姿%f",out_turn_angel_,pose_.response.pose_at[2]);
            if (pose_.response.pose_at[2] < out_turn_angel_) {
                right_point_start_ = false;
                double_line_ = true;
                x_max_ = 0.5;
                nh_.getParam("/line_left/double_P", p_);
                nh_.getParam("/line_left/double_I", i_);
                nh_.getParam("/line_left/double_D", d_);
                ROS_INFO("旋转完成，切换双边巡线 (P=%.2f)", p_);
            }
            return;
        }

        // 寻找右点并控制
        Point right_point;
        if (find_right_edge(gray_img, right_point, cropped)) {
            double error_x = 320 - right_point.x;
            pointx_integration_ += error_x * 0.02;
            pointx_integration_ = clamp(pointx_integration_, -1.0, 1.0);
            
            // 右点过低时停止前进
            if (right_point.y > 240) {
                point_forward_ = false;
            }

            // PID计算
            double point_diff = error_x - pointx_pre_error_;
            twist_.linear.x = 0.23;
            twist_.angular.z = error_x*leftpoint_p_ + pointx_integration_*leftpoint_I_ + point_diff*leftpoint_D_;
            pointx_pre_error_ = error_x;

            // 显示信息
            displayStream_ << "righterror: " << error_x << " P: " << error_x*leftpoint_p_ << " I: " << pointx_integration_*leftpoint_I_;
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        }
        out_.write(cropped);
    }

    // 正常巡线逻辑
    bool runNormalTracking(Mat& gray_img, Mat& cropped, bool enable_lost_turn = true) {
        displayStream_.str("");
        // 直接沿用原始line_left：从左下区域寻找左侧白线。
        vector<Point> start_points = find_track_edge(gray_img, 300, 70, cropped);
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
            out_.write(cropped);
            return true;
        } else {
            if (!enable_lost_turn) {
                // 初始找线阶段视野内没有线是正常情况，不能累计丢线次数，
                // 也不能触发固定左转；始终以x_max_保持正向直行。
                trace_failed_count_ = 0;
                twist_.linear.x = x_max_;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;

                displayStream_ << "初始直行找线"
                               << " 线速度: " << twist_.linear.x;
                putText(cropped, displayStream_.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
            } else {
                // 保留原来的连续5帧丢线容错，避免单帧识别波动引起误转。
                trace_failed_count_++;
                if (trace_failed_count_ > 5) {
                    // 原始line_left逻辑：angular.z > 0，丢线后固定左转。
                    twist_.linear.x = out_forward_;
                    twist_.linear.y = 0.0;
                    twist_.angular.z = std::abs(out_turn_);

                    if (trace_failed_count_ == 6) {
                        ROS_INFO("左线连续丢失，开始固定左转：线速度=%.3f，角速度=%.3f",
                                 twist_.linear.x, twist_.angular.z);
                    }

                    displayStream_ << "左线丢失，固定左转"
                                   << " 线速度: " << twist_.linear.x
                                   << " 角速度: " << twist_.angular.z;
                    putText(cropped, displayStream_.str(), Point(50, 50),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 165, 255), 1);
                }
            }
        }
        out_.write(cropped);
        return false;
    }

    // 工具函数：数值 clamping
    template <typename T>
    T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    // 图像处理：阈值化
    void threshold_image(Mat& gray) {
        Mat binary;
        adaptiveThreshold(
            gray,
            binary,
            255,
            ADAPTIVE_THRESH_MEAN_C,
            THRESH_BINARY,
            adaptive_block_,
            adaptive_c_
        );

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        Mat denoised = Mat::zeros(binary.size(), CV_8UC1);
        for (size_t i = 0; i < contours.size(); i++) {
            if (contourArea(contours[i]) > min_contour_area_) {
                drawContours(denoised, contours, i, Scalar(255), FILLED);
            }
        }

        gray = denoised.clone();
    }

    // 寻找赛道边缘起点
    vector<Point> find_track_edge(Mat& gray_img, int bottom_trace_end, int left_trace_end, Mat& visual_img) {
        bool is_now_white = false;
        vector<Point> maybe_start_point;

        // 原始line_left：从左向右寻找左侧白线的右边缘。
        for (int i = 1; i < bottom_trace_end; i++) {
            if (!is_now_white && gray_img.at<uchar>(269, i) == 255) {
                is_now_white = true;
            }
            if (is_now_white && gray_img.at<uchar>(269, i) == 0) {
                maybe_start_point.emplace_back(i+1, 269);
                circle(visual_img, Point(i+1, 269), 5, Scalar(0, 0, 255), -1);
                is_now_white = false;
            }
        }

        // 原始line_left：同时从图像左边界寻找起点。
        is_now_white = true;
        for (int i = 269; i > left_trace_end; i--) {
            if (is_now_white && gray_img.at<uchar>(i, 1) == 0) {
                is_now_white = false;
            }
            if (!is_now_white && gray_img.at<uchar>(i, 1) == 255) {
                maybe_start_point.emplace_back(1, i);
                circle(visual_img, Point(1, i), 5, Scalar(0, 0, 255), -1);
                is_now_white = true;
            }
        }
        return maybe_start_point;
    }

    // 追踪赛道边缘（修正参数：将int& racetrack改为RaceTrack& racetrack）
    bool trace_edge(Mat& gray_img, vector<Point> start_points, RaceTrack& racetrack, Mat& visual_img) {
        int point_number = start_points.size();
        vector<RaceTrack> racetracks(point_number);  // 现在可正常声明RaceTrack向量
        int height = gray_img.rows, width = gray_img.cols, search_range = 40;

        for (int idx = 0; idx < point_number; idx++) {
            bool broken = false, last_left = false, last_right = true;
            int fail_count = 0;
            Point start = start_points[idx];
            int center_x = start.x, center_y = start.y - 1;

            while (center_y > start.y - 100) {
                bool left_found = false, right_found = false;
                for (int dx = 0; dx <= search_range/2; dx++) {
                    int cand_x = center_x + dx;
                    int cand_x2 = center_x - dx;
                    bool left_check = (cand_x2 > 1);
                    bool right_check = (cand_x < width - 1);

                    // 原始line_left：记录左侧白线的右边缘。
                    if (right_check && gray_img.at<uchar>(center_y, cand_x) == 255 && gray_img.at<uchar>(center_y, cand_x + 1) == 0) {
                        racetracks[idx].points.emplace_back(cand_x, center_y);
                        right_found = true;
                        left_found = false;
                        center_x = cand_x;
                        break;
                    }
                    if (left_check && gray_img.at<uchar>(center_y, cand_x2) == 0 && gray_img.at<uchar>(center_y, cand_x2 - 1) == 255) {
                        left_found = true;
                        right_found = false;
                        center_x = cand_x2 - 1;
                        break;
                    }
                }

                // 更新方向变化计数
                if (last_left && right_found) {
                    racetracks[idx].direction_change++;
                    last_left = false;
                    last_right = true;
                }
                if (last_right && left_found) {
                    racetracks[idx].direction_change++;
                    last_left = true;
                    last_right = false;
                }

                // 处理追踪结果
                if (left_found || right_found) {
                    fail_count = 0;
                    center_y--;
                } else {
                    fail_count++;
                    center_y--;
                    if (fail_count >= 4) { broken = true; break; }
                }
                if (center_y <= 0 || racetracks[idx].points.size()>60) break;
            }

            // 计算斜率
            if (racetracks[idx].points.size() > 15) {
                Vec4f lineParams;
                fitLine(racetracks[idx].points, lineParams, DIST_L2, 0, 0.01, 0.01);
                racetracks[idx].slope = lineParams[1] / lineParams[0];
            } else {
                racetracks[idx].slope = 2.0;
            }
        }

        // 选择最优赛道
        int best_idx = -1;
        float min_dangerous = 2.1;
        for (int i = 0; i < point_number; i++) {
            if (!(racetracks[i].slope > -0.05 && racetracks[i].slope < 10)) {
                float ratio = racetracks[i].direction_change / (float)racetracks[i].points.size();
                if (ratio < min_dangerous) {
                    min_dangerous = ratio;
                    best_idx = i;
                }
            }
        }

        if (best_idx != -1) {
            racetrack = racetracks[best_idx];
            for (const auto& p : racetrack.points) {
                circle(visual_img, p, 2, Scalar(0, 255, 0), -1);
            }
            ostringstream oss;
            oss << "斜率: " << racetrack.slope << " 方向变化: " << racetrack.direction_change;
            putText(visual_img, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
            return true;
        }
        return false;
    }

    // 寻找右边缘：直接沿用原始line_left实现
    bool find_right_edge(Mat gray_img, Point& right_point, Mat& visualizeImg) {
        bool is_now_white = false;
        vector<Point> maybe_right_point;

        // 右部寻找起点
        for (int i = 269; i > 2; i--) {
            if (is_now_white && gray_img.at<uchar>(i, 634) == 0) {
                is_now_white = false;
            }
            if (!is_now_white && gray_img.at<uchar>(i, 634) == 255) {
                maybe_right_point.emplace_back(634, i);
                circle(visualizeImg, Point(634, i), 9, Scalar(255, 0, 0), -1);
                is_now_white = true;
            }
        }

        int point_number = maybe_right_point.size();
        vector<RaceTrack> racetracks(point_number);  // 现在可正常声明
        int search_range = 40;

        // 追踪右边缘
        for (int idx = 0; idx < point_number; idx++) {
            bool broken = false, last_up = false, last_down = false;
            int fail_count = 0;
            Point start = maybe_right_point[idx];
            int center_x = start.x - 1, center_y = start.y;

            while (center_x > 20) {
                bool found = false;
                for (int dy = 0; dy <= search_range/2; dy++) {
                    bool up_check = (center_y - dy > 2);
                    bool down_check = (center_y + dy < 268);

                    if (down_check && gray_img.at<uchar>(center_y + dy, center_x) == 255 && gray_img.at<uchar>(center_y + dy + 1, center_x) == 0) {
                        racetracks[idx].points.emplace_back(center_x, center_y + dy);
                        found = true;
                        center_y += dy;
                        if (last_up) racetracks[idx].direction_change++;
                        last_down = true;
                        last_up = false;
                        break;
                    }
                    if (!found && up_check && gray_img.at<uchar>(center_y - dy, center_x) == 255 && gray_img.at<uchar>(center_y - dy + 1, center_x) == 0) {
                        racetracks[idx].points.emplace_back(center_x - 1, center_y - dy);
                        found = true;
                        center_y -= dy;
                        if (last_down) racetracks[idx].direction_change++;
                        last_down = false;
                        last_up = true;
                        break;
                    }
                }

                if (found) {
                    fail_count = 0;
                    center_x--;
                } else {
                    fail_count++;
                    center_x--;
                    if (fail_count >= 10) { broken = true; break; }
                }
            }
            if (racetracks[idx].points.size() > 120) racetracks[idx].right_point = true;
        }

        // 选择最优右边缘
        int best_idx = -1;
        int lowest_y = 0;
        for (int i = 0; i < point_number; i++) {
            if (racetracks[i].right_point && racetracks[i].points[0].y > lowest_y) {
                lowest_y = racetracks[i].points[0].y;
                best_idx = i;
            }
        }

        if (best_idx != -1) {
            RaceTrack racetrack = racetracks[best_idx];  // 现在可正常使用
            Point best_point(0, 0);
            for (size_t i = 0; i < racetrack.points.size(); i += 3) {
                if (racetrack.points[i].y > best_point.y) best_point = racetrack.points[i];
                circle(visualizeImg, racetrack.points[i], 2, Scalar(255, 0, 0), -1);
            }
            circle(visualizeImg, best_point, 9, Scalar(0, 0, 255), -1);
            right_point = best_point;
            ostringstream oss;
            oss << "右点: (" << best_point.x << "," << best_point.y << ")";
            putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
            return true;
        }
        return false;
    }

    // 双边巡线误差计算
    double double_find(Mat gray_img, Mat& visual_img) {
        vector<int> left_total, right_total;
        double error = 0.0;
        vector<Point> midPoints;

        // 提取左边界
        bool falg = false;
        int failed = 0;
        for (int y = 269; y >= 50; y--) {
            int left = 0;
            bool find = false;
            for (int x = 319; x > 1; x--) {
                if (gray_img.at<uchar>(y, x) == 255) {
                    left = x;
                    find = true;
                    falg = true;
                    failed = 0;
                    break;
                }
            }
            if (falg && !find) {
                if (++failed > 10) break;
            }
            left_total.push_back(left);
        }

        // 提取右边界
        falg = false;
        failed = 0;
        for (int y = 269; y >= 50; y--) {
            int right = 639;
            bool find = false;
            for (int x = 319; x < 639; x++) {
                if (gray_img.at<uchar>(y, x) == 255) {
                    right = x;
                    find = true;
                    falg = true;
                    failed = 0;
                    break;
                }
            }
            if (falg && !find) {
                if (++failed > 10) break;
            }
            right_total.push_back(right);
        }

        // 计算误差
        float row = min(left_total.size(), right_total.size());
        for (int i = 0; i < row; i++) {
            error += (640 - (left_total[i] + right_total[i])) * (1 - i / row);
        }

        // 绘制中线
        int minSize = min(left_total.size(), right_total.size());
        for (int i = 0; i < minSize; i++) {
            int midX = (left_total[i] + right_total[i]) / 2;
            int y = 269 - i;
            midPoints.emplace_back(midX, y);
            circle(visual_img, Point(midX, y), 1, Scalar(0, 255, 255), -1);
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

            // 原始line_left：根据左侧边线向右推算赛道中线
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
    ros::init(argc, argv, "line_left");
    
    // 创建节点对象（构造函数中完成所有初始化）
    LineFollowerNode node;
    
    // 运行节点
    node.run();
    
    return 0;
}
