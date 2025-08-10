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
#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/lidar_process.h"
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
    bool left_point;               // 是否为左赛道标志
};

class LineFollowerNode {
private:
    // ROS核心组件
    ros::NodeHandle nh_;                  // 节点句柄
    ros::ServiceServer line_server_;      // 服务端
    ros::Publisher cmd_pub_;              // 速度发布者
    ros::ServiceClient client_line_board_;// lidar服务客户端
    ros::ServiceClient pose_client_;      // 位姿服务客户端
    ros::ServiceClient reconfigure_client_;// 动态配置客户端
    tf::TransformListener* tf_listener_;  // TF监听器
    MoveBaseClient* ac_;                  // MoveBase客户端

    // 消息对象
    geometry_msgs::Twist twist_;
    ztestnav2025::lidar_process board_;
    ztestnav2025::getpose_server pose_;

    // 图像处理相关
    Mat cameraMatrix_, distCoeffs_;       // 相机内参和畸变系数
    VideoCapture cap_;                    // 相机捕获
    VideoWriter out_;                     // 视频录制
    string output_file_;                  // 视频保存路径
    int fourcc_;                          // 视频编码格式
    ostringstream displayStream_;         // 信息显示流
    Rect roi_;                            // 图像裁剪区域
    Mat map1_, map2_;                     // 去畸变映射表

    // 控制参数
    double p_, i_, d_;                    // PID参数
    double leftpoint_p_, leftpoint_I_, leftpoint_D_; // 左点控制参数
    double x_max_, integration_limit_;    // 速度和积分限制
    double out_turn_, out_forward_,out_turn_angel_;       // 旋转和前进参数
    double integration_, pre_error_;      // 积分和前向误差
    double pointx_integration_, pointx_pre_error_; // 左点积分和前向误差

    // 状态变量
    bool avoid_done_;                     // 避障完成标志
    bool double_line_;                    // 双边巡线标志
    bool left_point_start_;               // 左点追踪标志
    bool point_forward_;                  // 左点前进标志
    int trace_failed_count_;              // 追踪失败计数

public:
    // 构造函数：初始化所有组件
    LineFollowerNode() : 
        nh_(""), 
        output_file_("/home/ucar/ucar_car/src/line_follow/image/line_right.avi"),
        fourcc_(VideoWriter::fourcc('X', 'V', 'I', 'D')),
        roi_(0, 210, 640, 270),
        avoid_done_(false),
        double_line_(false),
        left_point_start_(false),
        point_forward_(true),
        trace_failed_count_(0),
        integration_(0), 
        pre_error_(0),
        pointx_integration_(0),
        pointx_pre_error_(0) {

        ROS_INFO("开始初始化LineFollowerNode...");

        // 1. 初始化服务端（优先初始化）
        line_server_ = nh_.advertiseService("line_right", &LineFollowerNode::line_server_callback, this);
        ROS_INFO("line_right服务已初始化");

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
        nh_.getParam("/line_right/right_P", p_);
        nh_.getParam("/line_right/right_I", i_);
        nh_.getParam("/line_right/right_D", d_);
        nh_.getParam("/line_right/leftpoint_p", leftpoint_p_);
        nh_.getParam("/line_right/leftpoint_I", leftpoint_I_);
        nh_.getParam("/line_right/leftpoint_D", leftpoint_D_);
        nh_.getParam("/line_right/x_max_", x_max_);
        nh_.getParam("/line_right/integration_limit", integration_limit_);
        nh_.getParam("/line_right/out_forward", out_forward_);
        nh_.getParam("/line_right/out_turn", out_turn_);
        nh_.getParam("/line_right/out_turn_angel", out_turn_angel_);
        ROS_INFO("参数加载完成: P=%.2f, I=%.2f, D=%.2f", p_, i_, d_);
    }

    // 初始化ROS组件（客户端、发布者等）
    void initRosComponents() {
        // 初始化速度发布者
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("cmd_vel发布者已初始化");

        // 初始化lidar服务客户端
        ROS_INFO("等待lidar_process服务中...");
        client_line_board_ = nh_.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
        board_.request.lidar_process_start = -2;
        if (!client_line_board_.waitForExistence()) {
            ROS_FATAL("超时未连接到lidar_process服务");
            ros::shutdown();
        }
        ROS_INFO("lidar_process服务已连接");

        // 初始化位姿服务客户端
        ROS_INFO("等待坐标获取服务中...");
        pose_client_ = nh_.serviceClient<ztestnav2025::getpose_server>("/getpose_server");
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
        FileStorage fs("/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml", FileStorage::READ);
        if (!fs.isOpened()) {
            ROS_ERROR("无法打开标定文件");
            return false;
        }
        fs["camera_matrix"] >> cameraMatrix_;
        fs["distortion_coefficients"] >> distCoeffs_;
        fs.release();

        // 初始化去畸变映射表
        Mat optimalMatrix = getOptimalNewCameraMatrix(
            cameraMatrix_, distCoeffs_, Size(640, 480), 1, Size(640, 480)
        );
        initUndistortRectifyMap(
            cameraMatrix_, distCoeffs_, Mat(), optimalMatrix,
            Size(640, 480), CV_32FC1, map1_, map2_
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
        Mat image, undistorted, cropped, gray_img;
        // 5. 初始化相机和视频录制
        if (!initCameraAndVideo()) {
            ROS_FATAL("相机或视频初始化失败，节点无法启动");
            ros::shutdown();
        }
        while (ros::ok()) {
            // 避障逻辑
            handleObstacleAvoidance();

            // 读取并预处理图像
            cap_.read(image);
            if (image.empty()) continue;
            cropped = image(roi_);
            flip(cropped, cropped, 1); // 翻转图像
            vector<Mat> channels;
            split(cropped, channels);
            gray_img = channels[2]; // 红色通道作为灰度图
            threshold_image(gray_img);

            // 巡线逻辑分支
            if (double_line_) {
                runDoubleLineTracking(gray_img, cropped);
            } else if (left_point_start_) {
                runLeftPointTracking(gray_img, cropped);
            } else {
                runNormalTracking(gray_img, cropped);
            }

            // 发布速度指令
            cmd_pub_.publish(twist_);

            // 停车检测
            int stop_point_count;
            if (avoid_done_ && stop_car(gray_img, stop_point_count, cropped)) {
                ROS_INFO("巡线结束，触发停车");
                twist_.linear.x = 0;
                twist_.angular.z = 0;
                cmd_pub_.publish(twist_);
                break;
            }
        }
        bool avoid_done_ = false;                     // 避障完成标志
        bool double_line_ = false;                    // 双边巡线标志
        bool left_point_start_ = false;               // 左点追踪标志
        bool point_forward_ = true;                  // 左点前进标志
        int trace_failed_count_ = 0;              // 追踪失败计数
        return true;
    }

    // 避障逻辑处理
    void handleObstacleAvoidance() {
        if (avoid_done_) return;

        client_line_board_.call(board_);
        pose_client_.call(pose_);

        if (board_.response.lidar_results[0] != -1) {
            // 距离过近时执行避障
            if (board_.response.lidar_results[0] <= 0.41) {
                ROS_INFO("触发避障，最短距离: %.2f", board_.response.lidar_results[0]);
                executeObstacleAvoidanceSequence();
                avoid_done_ = true;
                x_max_ = 0.7;
                double_line_ = true;
                nh_.getParam("/line_right/double_P", p_);
                nh_.getParam("/line_right/double_I", i_);
                nh_.getParam("/line_right/double_D", d_);
                ROS_INFO("避障结束，切换为双边巡线 (P=%.2f)", p_);
            } else {
                // 远距离减速
                x_max_ = 0.22;
            }
        }
    }

    // 执行避障序列
    void executeObstacleAvoidanceSequence() {
        // 立即停止
        twist_.linear.x = 0;
        twist_.linear.y = 0;
        twist_.angular.z = 0;
        cmd_pub_.publish(twist_);
        ros::Duration(0.1).sleep();

        // 获取初始位姿
        pose_client_.call(pose_);
        double initial_y = pose_.response.pose_at[1];
        double target_yaw = -1.57;
        double side_step_x = 3.25;
        double track_x = 3.75;
        double forward_target_y = initial_y - board_.response.lidar_results[0] - 0.19;

        // 避障参数
        const double Kp_x = 1.5;
        const double Kp_y = 1.5;
        const double Kp_yaw = 1.0;
        const double max_vel_lateral = 0.35;
        const double max_vel_forward = 0.35;
        const double max_vel_yaw = 0.4;
        const double tolerance_x = 0.02;
        const double tolerance_y = 0.03;
        ros::Rate rate(20.0);

        // 第1步：横向平移至side_step_x
        ROS_INFO("避障第1步: 横向平移至 x=%.2f", side_step_x);
        while (ros::ok()) {
            pose_client_.call(pose_);
            double current_x = pose_.response.pose_at[0];
            double current_yaw = pose_.response.pose_at[2];
            double error_x = side_step_x - current_x;
            if (fabs(error_x) < tolerance_x) break;

            double error_yaw = target_yaw - current_yaw;
            error_yaw = atan2(sin(error_yaw), cos(error_yaw));
            twist_.linear.y = Kp_x * error_x;
            twist_.linear.y = clamp(twist_.linear.y, -max_vel_lateral, max_vel_lateral);
            twist_.linear.x = 0;
            twist_.angular.z = Kp_yaw * error_yaw;
            twist_.angular.z = clamp(twist_.angular.z, -max_vel_yaw, max_vel_yaw);
            cmd_pub_.publish(twist_);
            rate.sleep();
        }
        stopRobot();

        // 第2步：前进至目标Y点
        ROS_INFO("避障第2步: 前进至 y=%.2f", forward_target_y);
        while (ros::ok()) {
            pose_client_.call(pose_);
            double current_y = pose_.response.pose_at[1];
            double current_x = pose_.response.pose_at[0];
            double current_yaw = pose_.response.pose_at[2];
            double error_y = forward_target_y - current_y;
            if (fabs(error_y) < tolerance_y) break;

            twist_.linear.x = -Kp_x * error_y;
            twist_.linear.x = clamp(twist_.linear.x, 0.0, max_vel_forward);
            double error_x = side_step_x - current_x;
            twist_.linear.y = -Kp_y * error_x;
            twist_.linear.y = clamp(twist_.linear.y, -max_vel_lateral, max_vel_lateral);
            double error_yaw = target_yaw - current_yaw;
            error_yaw = atan2(sin(error_yaw), cos(error_yaw));
            twist_.angular.z = Kp_yaw * error_yaw;
            twist_.angular.z = clamp(twist_.angular.z, -max_vel_yaw, max_vel_yaw);
            cmd_pub_.publish(twist_);
            rate.sleep();
        }
        stopRobot();

        // 第3步：横向平移回track_x
        ROS_INFO("避障第3步: 横向平移回 x=%.2f", track_x);
        while (ros::ok()) {
            pose_client_.call(pose_);
            double current_x = pose_.response.pose_at[0];
            double current_yaw = pose_.response.pose_at[2];
            double error_x = track_x - current_x;
            if (fabs(error_x) < tolerance_x) break;

            double error_yaw = target_yaw - current_yaw;
            error_yaw = atan2(sin(error_yaw), cos(error_yaw));
            twist_.linear.y = Kp_x * error_x;
            twist_.linear.y = clamp(twist_.linear.y, -max_vel_lateral, max_vel_lateral);
            twist_.linear.x = 0;
            twist_.angular.z = Kp_yaw * error_yaw;
            twist_.angular.z = clamp(twist_.angular.z, -max_vel_yaw, max_vel_yaw);
            cmd_pub_.publish(twist_);
            rate.sleep();
        }
        stopRobot();

        // 清理相机缓存
        for (int i = 0; i < 5; i++) cap_.grab();
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
        putText(cropped, displayStream_.str(), Point(50, 50),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        out_.write(cropped);
    }

    // 左点追踪逻辑
    void runLeftPointTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        if (!point_forward_) {
            // 丢线旋转
            ROS_INFO("丢线旋转中");
            twist_.linear.x = out_forward_;
            twist_.angular.z = out_turn_;
            out_.write(cropped);
            
            // 旋转到位后切换模式
            pose_client_.call(pose_);
            // ROS_INFO("角度%f,位姿%f",out_turn_angel_,pose_.response.pose_at[2]);
            if (pose_.response.pose_at[2] < out_turn_angel_) {
                left_point_start_ = false;
                double_line_ = true;
                x_max_ = 0.5;
                nh_.getParam("/line_right/double_P", p_);
                nh_.getParam("/line_right/double_I", i_);
                nh_.getParam("/line_right/double_D", d_);
                ROS_INFO("旋转完成，切换双边巡线 (P=%.2f)", p_);
            }
            return;
        }

        // 寻找左点并控制
        Point left_point;
        if (find_left_edge(gray_img, left_point, cropped)) {
            double error_x = 320 - left_point.x;
            pointx_integration_ += error_x * 0.02;
            pointx_integration_ = clamp(pointx_integration_, -1.0, 1.0);
            
            // 左点过低时停止前进
            if (left_point.y > 240) {
                point_forward_ = false;
            }

            // PID计算
            double point_diff = error_x - pointx_pre_error_;
            twist_.linear.x = 0.23;
            twist_.angular.z = error_x*leftpoint_p_ + pointx_integration_*leftpoint_I_ + point_diff*leftpoint_D_;
            pointx_pre_error_ = error_x;

            // 显示信息
            displayStream_ << "lefterror: " << error_x << " P: " << error_x*leftpoint_p_ << " I: " << pointx_integration_*leftpoint_I_;
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        }
        out_.write(cropped);
    }

    // 正常巡线逻辑
    void runNormalTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        vector<Point> start_points = find_track_edge(gray_img, 340, 70, cropped);
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
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        } else {
            // 追踪失败计数
            trace_failed_count_++;
            if (trace_failed_count_ > 5) {
                left_point_start_ = true;
                ROS_INFO("连续追踪失败，切换至左点追踪模式");
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
    vector<Point> find_track_edge(Mat& gray_img, int bottom_trace_end, int right_trace_end, Mat& visual_img) {
        bool is_now_white = false;
        vector<Point> maybe_start_point;

        // 底部寻找
        for (int i = 639; i > bottom_trace_end; i--) {
            if (!is_now_white && gray_img.at<uchar>(269, i) == 255) {
                is_now_white = true;
            }
            if (is_now_white && gray_img.at<uchar>(269, i) == 0) {
                maybe_start_point.emplace_back(i-1, 269);
                circle(visual_img, Point(i-1, 269), 5, Scalar(0, 0, 255), -1);
                is_now_white = false;
            }
        }

        // 右部寻找
        is_now_white = true;
        for (int i = 269; i > right_trace_end; i--) {
            if (is_now_white && gray_img.at<uchar>(i, 639) == 0) {
                is_now_white = false;
            }
            if (!is_now_white && gray_img.at<uchar>(i, 639) == 255) {
                maybe_start_point.emplace_back(639, i);
                circle(visual_img, Point(639, i), 5, Scalar(0, 0, 255), -1);
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
            bool broken = false, last_left = true, last_right = false;
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

                    if (left_check && gray_img.at<uchar>(center_y, cand_x2) == 255 && gray_img.at<uchar>(center_y, cand_x2 - 1) == 0) {
                        racetracks[idx].points.emplace_back(cand_x2, center_y);
                        right_found = false;
                        left_found = true;
                        center_x = cand_x2;
                        break;
                    }
                    if (right_check && gray_img.at<uchar>(center_y, cand_x) == 0 && gray_img.at<uchar>(center_y, cand_x + 1) == 255) {
                        right_found = true;
                        left_found = false;
                        center_x = cand_x + 1;
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
                racetracks[idx].slope = -2.0;
            }
        }

        // 选择最优赛道
        int best_idx = -1;
        float min_dangerous = 2.1;
        for (int i = 0; i < point_number; i++) {
            if (!(racetracks[i].slope < 0.05 && racetracks[i].slope > -10)) {
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
            putText(visual_img, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            return true;
        }
        return false;
    }

    // 寻找左边缘
    bool find_left_edge(Mat gray_img, Point& left_point, Mat& visualizeImg) {
        bool is_now_white = false;
        vector<Point> maybe_left_point;

        // 左部寻找起点
        for (int i = 269; i > 2; i--) {
            if (is_now_white && gray_img.at<uchar>(i, 5) == 0) {
                is_now_white = false;
            }
            if (!is_now_white && gray_img.at<uchar>(i, 5) == 255) {
                maybe_left_point.emplace_back(5, i);
                circle(visualizeImg, Point(5, i), 9, Scalar(255, 0, 0), -1);
                is_now_white = true;
            }
        }

        int point_number = maybe_left_point.size();
        vector<RaceTrack> racetracks(point_number);  // 现在可正常声明
        int search_range = 40;

        // 追踪左边缘
        for (int idx = 0; idx < point_number; idx++) {
            bool broken = false, last_up = false, last_down = false;
            int fail_count = 0;
            Point start = maybe_left_point[idx];
            int center_x = start.x + 1, center_y = start.y;

            while (center_x < 620) {
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
                        racetracks[idx].points.emplace_back(center_x + 1, center_y - dy);
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
                    center_x++;
                } else {
                    fail_count++;
                    center_x++;
                    if (fail_count >= 10) { broken = true; break; }
                }
            }
            if (racetracks[idx].points.size() > 120) racetracks[idx].left_point = true;
        }

        // 选择最优左边缘
        int best_idx = -1;
        int lowest_y = 0;
        for (int i = 0; i < point_number; i++) {
            if (racetracks[i].left_point && racetracks[i].points[0].y > lowest_y) {
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
            left_point = best_point;
            ostringstream oss;
            oss << "左点: (" << best_point.x << "," << best_point.y << ")";
            putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
        double total_error = 0;
        for (size_t i = 0; i < traced_points.size(); i++) {
            int y = traced_points[i].y;
            double mid_error;
            if (i <= 30.0) {
                mid_error = (traced_points[i].x - (280 - (188 - y) * 1.34) - 320) * (1 - i / 100);
            } else {
                mid_error = (traced_points[i].x - (280 - (188 - y) * 1.34) - 320) * 0.7 * exp(-0.064 * (i - 30.0));
            }
            total_error += mid_error;
        }

        // 绘制轨迹
        for (int i = 0; i < traced_points.size(); i++) {
            int y = traced_points[i].y;
            Point pt(traced_points[i].x - (280 - (188 - y) * 1.34), traced_points[i].y);
            circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
        }

        return traced_points.empty() ? 100.0 : total_error / traced_points.size() * -1;
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "line_right");
    
    // 创建节点对象（构造函数中完成所有初始化）
    LineFollowerNode node;
    
    // 运行节点
    node.run();
    
    return 0;
}