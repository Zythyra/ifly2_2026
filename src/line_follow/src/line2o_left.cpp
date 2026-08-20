// 版本：第一段反向平移 + 左线巡线 + 动态目标YAML直出 V6（2026-08-20）
// 唯一校验标识：LINE2O_LEFT_FIXED_AVOID_TARGET_YAML_LOG_V6
// 修改基线：line2o_right V16 + line2_left V4实车参数与左线逻辑。
// 终点停靠平移全程同步将车头调整至-90°（由现有参数配置）。
// 每次巡线服务启动前先向 /initialpose 发布指定AMCL初始位姿；
// 巡线满足定位触发条件后，不发布零速度，直接无缝切入终点纯PP位姿停靠。

// 2026-08-19：普通丢线前5帧只线性衰减角速度，线速度不改；第6帧固定左转及其余逻辑保持原样。
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
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
    double slope = -2.0;           // 赛道斜率
    vector<Point> points;          // 赛道点集
    int direction_change = 0;      // 方向变化次数
    int slope_change_count = 0;    // 斜率变化次数
    bool right_point = false;      // 是否为右赛道标志
};

struct BarrierLineFit {
    bool computed = false;  // 点数足够且已完成总最小二乘计算
    bool valid = false;
    ros::Time stamp;
    double distance = std::numeric_limits<double>::infinity();
    double direction_x = 0.0;  // 挡板方向单位向量，锁定后约定指向车体左侧
    double direction_y = 1.0;
    double normal_x = 1.0;     // 挡板法向单位向量，锁定后约定指向车体前方
    double normal_y = 0.0;
    double center_x = 0.0;     // 观测挡板线段中点，LaserScan坐标系
    double center_y = 0.0;
    double center_map_x = 0.0; // 同一中点在map坐标系中的触发时锁存值
    double center_map_y = 0.0;
    bool center_map_valid = false;
    double normal_map_x = 1.0; // 同一挡板法向单位向量在map坐标系中的锁存值
    double normal_map_y = 0.0;
    bool normal_map_valid = false;
    double length = 0.0;
    double rms_error = std::numeric_limits<double>::infinity();
    double direction_angle_deg = 0.0;       // 板方向相对车头角度，正值向左
    double normal_angle_deg = 0.0;          // 板前向法线相对车头角度
    double lateral_deviation_deg = 90.0;    // 板方向相对车体横向轴的偏差
    double required_min_line_length = 0.0;  // 当前距离实际采用的最小线长
    bool inside_trigger_zone = false;
    bool foot_on_observed_segment = false;
    int point_count = 0;
    string rejection_reason = "尚未计算";
};

class LineFollowerNode {
private:
    // ROS核心组件
    ros::NodeHandle nh_;                  // 节点句柄
    ros::ServiceServer line_server_;      // 服务端
    ros::Publisher cmd_pub_;              // 速度发布者
    ros::Publisher initial_pose_pub_;     // AMCL初始位姿发布者
    ros::Subscriber scan_sub_;            // 挡板检测雷达订阅者
    ros::WallTimer scan_watchdog_timer_;  // 雷达数据中断诊断

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

    // 图像局部自适应二值化参数（由 line2o_left.yaml 配置）
    int adaptive_block_;                  // 自适应阈值邻域大小，必须为大于1的奇数
    int adaptive_c_;                      // 自适应阈值常数 C
    int min_contour_area_;                // 二值化后保留轮廓的最小面积

    // 控制参数
    double p_, i_, d_;                    // PID参数
    double rightpoint_p_, rightpoint_I_, rightpoint_D_; // 右点控制参数
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
    bool docking_require_avoidance_complete_;
    double docking_goal_x_;
    double docking_goal_y_;
    double docking_goal_yaw_deg_;
    double docking_control_rate_;
    double docking_position_tolerance_;
    double docking_yaw_tolerance_;
    double docking_relaxed_yaw_tolerance_;
    double docking_relaxed_hold_time_;
    double docking_timeout_;
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

    // 雷达挡板检测参数
    string scan_topic_;
    double barrier_min_scan_range_;
    double barrier_max_scan_range_;
    double field_min_x_;
    double field_max_x_;
    double field_min_y_;
    double field_max_y_;
    double barrier_boundary_margin_;
    double barrier_candidate_min_map_x_;
    double barrier_neighbor_max_distance_;
    int barrier_min_points_;
    double barrier_min_line_length_;
    double barrier_trigger_zone_min_line_length_;
    double barrier_max_rms_error_;
    double barrier_endpoint_margin_;
    double barrier_trigger_distance_;
    double barrier_no_drift_distance_;       // 挡板近区禁止丢线固定转向的锁存距离
    double barrier_slowdown_start_distance_;
    double barrier_slowdown_min_speed_ratio_;
    int barrier_confirm_scans_;
    double barrier_confirm_max_distance_change_;
    double barrier_confirm_max_angle_change_deg_;
    bool barrier_debug_log_enabled_;
    double barrier_debug_log_interval_;
    double barrier_scan_timeout_;

    // 一次性绕障参数；原第二、第三段参数仅保留用于配置兼容和回退
    double avoid_left_distance_;
    double avoid_forward_distance_;
    double avoid_right_distance_;
    double avoid_left_speed_;
    double avoid_forward_speed_;
    double avoid_right_speed_;
    double avoid_control_rate_;
    double avoid_position_kp_;
    double avoid_yaw_kp_;
    double avoid_max_angular_speed_;
    double avoid_rotate_min_angular_speed_;
    double avoid_rotate_tolerance_deg_;
    double avoid_rotate_timeout_;
    double avoid_heading_pause_error_deg_;
    double avoid_position_tolerance_;
    double avoid_min_linear_speed_;
    double avoid_segment_timeout_;
    double avoid_stop_hold_time_;
    double avoid_segment_pause_time_;
    double avoid_acc_lim_x_;
    double avoid_acc_lim_y_;
    double avoid_acc_lim_theta_;
    int avoid_camera_flush_frames_;
    double avoid_move_base_extension_distance_;
    double avoid_move_base_parallel_shift_distance_;
    double avoid_move_base_heading_target_x_;
    double avoid_move_base_heading_target_y_;
    double avoid_move_base_min_target_x_;
    // false时保持原挡板几何目标；true时第三段使用YAML绝对map目标位姿。
    bool avoid_use_fixed_target_;
    double avoid_fixed_target_x_;
    double avoid_fixed_target_y_;
    double avoid_fixed_target_yaw_deg_;
    double avoid_move_base_timeout_;
    double avoid_recenter_y_kp_;
    double avoid_recenter_min_y_speed_;
    double avoid_recenter_max_y_speed_;
    double avoid_recenter_line_error_tolerance_;
    int avoid_recenter_confirm_frames_;
    double avoid_recenter_timeout_;

    // 挡板状态由雷达回调和巡线服务并发访问
    std::mutex barrier_mutex_;
    BarrierLineFit candidate_fit_;
    BarrierLineFit locked_fit_;
    int barrier_confirm_count_;
    std::atomic<bool> line_service_active_;
    std::atomic<bool> obstacle_triggered_;
    std::atomic<bool> avoidance_active_;
    std::atomic<bool> avoidance_completed_;
    std::atomic<bool> avoidance_succeeded_;
    std::atomic<double> last_scan_wall_time_sec_;
    std::atomic<double> barrier_tracking_speed_limit_;
    std::atomic<bool> barrier_no_drift_active_; // 进入挡板近区后锁存，避障结束清零

    // 状态变量

    bool double_line_;                    // 双边巡线标志
    bool right_point_start_;              // 右点追踪标志
    bool point_forward_;                  // 右点前进标志
    int trace_failed_count_;              // 追踪失败计数
    double lost_start_angular_z_;          // 疑似丢线开始前最后一帧PID角速度，用于线性衰减

public:
    // 构造函数：初始化所有组件
    LineFollowerNode() : 
        nh_(""),
        tf_listener_(nullptr),
        ac_(nullptr),
        output_file_("/home/ucar/ucar_ws_copy/src/line_follow/image/line2o_left.avi"),
        fourcc_(VideoWriter::fourcc('X', 'V', 'I', 'D')),
        roi_(0, 210, 640, 270),
        integration_(0),
        pre_error_(0),
        pointx_integration_(0),
        pointx_pre_error_(0),
        barrier_confirm_count_(0),
        line_service_active_(false),
        obstacle_triggered_(false),
        avoidance_active_(false),
        avoidance_completed_(false),
        avoidance_succeeded_(false),
        last_scan_wall_time_sec_(0.0),
        barrier_tracking_speed_limit_(
            std::numeric_limits<double>::infinity()),
        barrier_no_drift_active_(false),
        double_line_(false),
        right_point_start_(false),
        point_forward_(true),
        trace_failed_count_(0),
        lost_start_angular_z_(0.0) {

        ROS_INFO(
            "启动 line2o_left V3（挡板近区禁止丢线左转 + 原V2其余逻辑不变）");

        // 1. 初始化服务端（优先初始化）
        line_server_ = nh_.advertiseService("line2o_left", &LineFollowerNode::line_server_callback, this);
        ROS_INFO("line2o_left服务已初始化");

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
        // 参数键名与另外两个巡线程序保持一致，但从 /line2o_left
        // 参数树读取独立的 line2o_left.yaml。
        nh_.getParam("/line2o_left/right_P", p_);
        nh_.getParam("/line2o_left/right_I", i_);
        nh_.getParam("/line2o_left/right_D", d_);
        // 保持line2_left.yaml现有键名，但用于镜像后的右点追踪。
        nh_.getParam("/line2o_left/leftpoint_p", rightpoint_p_);
        nh_.getParam("/line2o_left/leftpoint_I", rightpoint_I_);
        nh_.getParam("/line2o_left/leftpoint_D", rightpoint_D_);
        nh_.getParam("/line2o_left/x_max_", x_max_);
        nh_.getParam("/line2o_left/integration_limit", integration_limit_);
        nh_.getParam("/line2o_left/out_forward", out_forward_);
        nh_.getParam("/line2o_left/out_turn", out_turn_);
        nh_.getParam("/line2o_left/out_turn_angel", out_turn_angel_);
        nh_.getParam("/line2o_left/center_distance", center_distance);

        // 从 line2o_left.yaml 读取图像二值化参数。
        // YAML 中未填写时，默认值与当前实车参数保持一致：45、-22、250。
        nh_.param("/line2o_left/adaptive_block", adaptive_block_, 45);
        nh_.param("/line2o_left/adaptive_c", adaptive_c_, -22);
        nh_.param("/line2o_left/min_contour_area", min_contour_area_, 250);

        // 雷达挡板检测：遍历整帧雷达，利用map场地边界排除四周墙面。
        nh_.param<string>("/line2o_left/scan_topic", scan_topic_, "/scan");
        nh_.param("/line2o_left/barrier_min_scan_range",
                  barrier_min_scan_range_, 0.05);
        nh_.param("/line2o_left/barrier_max_scan_range",
                  barrier_max_scan_range_, 2.00);
        nh_.param("/line2o_left/field_min_x", field_min_x_, 0.0);
        nh_.param("/line2o_left/field_max_x", field_max_x_, 5.0);
        nh_.param("/line2o_left/field_min_y", field_min_y_, 0.0);
        nh_.param("/line2o_left/field_max_y", field_max_y_, 2.5);
        nh_.param("/line2o_left/barrier_boundary_margin",
                  barrier_boundary_margin_, 0.10);
        nh_.param("/line2o_left/barrier_candidate_min_map_x",
                  barrier_candidate_min_map_x_, 3.0);
        nh_.param("/line2o_left/barrier_neighbor_max_distance",
                  barrier_neighbor_max_distance_, 0.10);
        nh_.param("/line2o_left/barrier_min_points",
                  barrier_min_points_, 6);
        nh_.param("/line2o_left/barrier_min_line_length",
                  barrier_min_line_length_, 0.20);
        nh_.param("/line2o_left/barrier_trigger_zone_min_line_length",
                  barrier_trigger_zone_min_line_length_, 0.005);
        nh_.param("/line2o_left/barrier_max_rms_error",
                  barrier_max_rms_error_, 0.025);
        nh_.param("/line2o_left/barrier_endpoint_margin",
                  barrier_endpoint_margin_, 0.05);
        nh_.param("/line2o_left/barrier_trigger_distance",
                  barrier_trigger_distance_, 0.20);
        // 挡板近区丢线转向抑制：YAML可显式配置；未配置时默认0.35m。
        nh_.param("/line2o_left/barrier_no_drift_distance",
                  barrier_no_drift_distance_, 0.35);
        nh_.param("/line2o_left/barrier_slowdown_start_distance",
                  barrier_slowdown_start_distance_, 1.00);
        nh_.param("/line2o_left/barrier_slowdown_min_speed_ratio",
                  barrier_slowdown_min_speed_ratio_, 0.50);
        nh_.param("/line2o_left/barrier_confirm_scans",
                  barrier_confirm_scans_, 1);
        nh_.param("/line2o_left/barrier_confirm_max_distance_change",
                  barrier_confirm_max_distance_change_, 0.08);
        nh_.param("/line2o_left/barrier_confirm_max_angle_change_deg",
                  barrier_confirm_max_angle_change_deg_, 15.0);
        nh_.param("/line2o_left/barrier_debug_log_enabled",
                  barrier_debug_log_enabled_, true);
        nh_.param("/line2o_left/barrier_debug_log_interval",
                  barrier_debug_log_interval_, 0.50);
        nh_.param("/line2o_left/barrier_scan_timeout",
                  barrier_scan_timeout_, 1.00);

        // 锁定挡板后：沿板向左、垂直板向前、沿板向右。
        nh_.param("/line2o_left/avoid_left_distance",
                  avoid_left_distance_, 0.50);
        nh_.param("/line2o_left/avoid_forward_distance",
                  avoid_forward_distance_, 0.50);
        nh_.param("/line2o_left/avoid_right_distance",
                  avoid_right_distance_, 0.50);
        nh_.param("/line2o_left/avoid_left_speed",
                  avoid_left_speed_, 0.35);
        nh_.param("/line2o_left/avoid_forward_speed",
                  avoid_forward_speed_, 0.45);
        nh_.param("/line2o_left/avoid_right_speed",
                  avoid_right_speed_, 0.35);
        nh_.param("/line2o_left/avoid_control_rate",
                  avoid_control_rate_, 40.0);
        nh_.param("/line2o_left/avoid_position_kp",
                  avoid_position_kp_, 2.0);
        nh_.param("/line2o_left/avoid_yaw_kp",
                  avoid_yaw_kp_, 2.5);
        nh_.param("/line2o_left/avoid_max_angular_speed",
                  avoid_max_angular_speed_, 0.60);
        nh_.param("/line2o_left/avoid_rotate_min_angular_speed",
                  avoid_rotate_min_angular_speed_, 0.10);
        nh_.param("/line2o_left/avoid_rotate_tolerance_deg",
                  avoid_rotate_tolerance_deg_, 2.0);
        nh_.param("/line2o_left/avoid_rotate_timeout",
                  avoid_rotate_timeout_, 3.0);
        nh_.param("/line2o_left/avoid_heading_pause_error_deg",
                  avoid_heading_pause_error_deg_, 6.0);
        nh_.param("/line2o_left/avoid_position_tolerance",
                  avoid_position_tolerance_, 0.02);
        nh_.param("/line2o_left/avoid_min_linear_speed",
                  avoid_min_linear_speed_, 0.08);
        nh_.param("/line2o_left/avoid_segment_timeout",
                  avoid_segment_timeout_, 5.0);
        nh_.param("/line2o_left/avoid_stop_hold_time",
                  avoid_stop_hold_time_, 0.10);
        nh_.param("/line2o_left/avoid_segment_pause_time",
                  avoid_segment_pause_time_, 0.05);
        nh_.param("/line2o_left/avoid_acc_lim_x",
                  avoid_acc_lim_x_, 2.0);
        nh_.param("/line2o_left/avoid_acc_lim_y",
                  avoid_acc_lim_y_, 2.0);
        nh_.param("/line2o_left/avoid_acc_lim_theta",
                  avoid_acc_lim_theta_, 6.0);
        nh_.param("/line2o_left/avoid_camera_flush_frames",
                  avoid_camera_flush_frames_, 5);
        nh_.param("/line2o_left/avoid_move_base_extension_distance",
                  avoid_move_base_extension_distance_, 0.30);
        nh_.param("/line2o_left/avoid_move_base_parallel_shift_distance",
                  avoid_move_base_parallel_shift_distance_, 0.0);
        nh_.param("/line2o_left/avoid_move_base_heading_target_x",
                  avoid_move_base_heading_target_x_, 4.75);
        nh_.param("/line2o_left/avoid_move_base_heading_target_y",
                  avoid_move_base_heading_target_y_, 0.25);
        nh_.param("/line2o_left/avoid_move_base_min_target_x",
                  avoid_move_base_min_target_x_, 0.0);
        nh_.param("/line2o_left/avoid_use_fixed_target",
                  avoid_use_fixed_target_, false);
        nh_.param("/line2o_left/avoid_fixed_target_x",
                  avoid_fixed_target_x_, 0.0);
        nh_.param("/line2o_left/avoid_fixed_target_y",
                  avoid_fixed_target_y_, 0.0);
        nh_.param("/line2o_left/avoid_fixed_target_yaw_deg",
                  avoid_fixed_target_yaw_deg_, -90.0);
        nh_.param("/line2o_left/avoid_move_base_timeout",
                  avoid_move_base_timeout_, 10.0);
        nh_.param("/line2o_left/avoid_recenter_y_kp",
                  avoid_recenter_y_kp_, 0.004);
        nh_.param("/line2o_left/avoid_recenter_min_y_speed",
                  avoid_recenter_min_y_speed_, 0.06);
        nh_.param("/line2o_left/avoid_recenter_max_y_speed",
                  avoid_recenter_max_y_speed_, 0.30);
        nh_.param("/line2o_left/avoid_recenter_line_error_tolerance",
                  avoid_recenter_line_error_tolerance_, 8.0);
        nh_.param("/line2o_left/avoid_recenter_confirm_frames",
                  avoid_recenter_confirm_frames_, 5);
        nh_.param("/line2o_left/avoid_recenter_timeout",
                  avoid_recenter_timeout_, 5.0);

        // AMCL初始位姿。每次服务真正启动巡线前都会重新读取并发布。
        nh_.param<string>("/line2o_left/map_frame", map_frame_, "map");
        nh_.param<string>("/line2o_left/base_frame", base_frame_, "base_link");
        nh_.param("/line2o_left/initial_pose_x", initial_pose_x_, 2.50);
        nh_.param("/line2o_left/initial_pose_y", initial_pose_y_, 2.60);
        nh_.param("/line2o_left/initial_pose_yaw_deg", initial_pose_yaw_deg_, -90.0);
        nh_.param("/line2o_left/initial_pose_covariance_xy",
                  initial_pose_covariance_xy_, 0.01);
        nh_.param("/line2o_left/initial_pose_covariance_yaw",
                  initial_pose_covariance_yaw_, 0.01);
        nh_.param("/line2o_left/initial_pose_publish_count",
                  initial_pose_publish_count_, 3);
        nh_.param("/line2o_left/initial_pose_publish_interval",
                  initial_pose_publish_interval_, 0.10);

        // 巡线终止触发条件与固定终点。
        nh_.param("/line2o_left/docking_trigger_min_x",
                  docking_trigger_min_x_, 4.25);
        nh_.param("/line2o_left/docking_trigger_distance",
                  docking_trigger_distance_, 0.75);
        nh_.param("/line2o_left/docking_require_avoidance_complete",
                  docking_require_avoidance_complete_, true);
        nh_.param("/line2o_left/docking_goal_x", docking_goal_x_, 4.77);
        nh_.param("/line2o_left/docking_goal_y", docking_goal_y_, 0.25);
        nh_.param("/line2o_left/docking_goal_yaw_deg",
                  docking_goal_yaw_deg_, -90.0);

        // 终点纯PP位姿控制，参数默认照搬局部规划器原最终姿态调整。
        nh_.param("/line2o_left/docking_control_rate",
                  docking_control_rate_, 30.0);
        nh_.param("/line2o_left/docking_position_tolerance",
                  docking_position_tolerance_, 0.025);
        nh_.param("/line2o_left/docking_yaw_tolerance",
                  docking_yaw_tolerance_, 0.05);
        nh_.param("/line2o_left/docking_relaxed_yaw_tolerance",
                  docking_relaxed_yaw_tolerance_, 0.08);
        nh_.param("/line2o_left/docking_relaxed_hold_time",
                  docking_relaxed_hold_time_, 0.50);
        nh_.param("/line2o_left/docking_timeout",
                  docking_timeout_, 12.0);
        nh_.param("/line2o_left/docking_linear_x_gain",
                  docking_linear_x_gain_, 2.50);
        nh_.param("/line2o_left/docking_linear_y_gain",
                  docking_linear_y_gain_, 1.20);
        nh_.param("/line2o_left/docking_angular_gain",
                  docking_angular_gain_, 1.50);
        nh_.param("/line2o_left/docking_min_linear_speed",
                  docking_min_linear_speed_, 0.10);
        nh_.param("/line2o_left/docking_min_angular_speed",
                  docking_min_angular_speed_, 0.010);
        nh_.param("/line2o_left/docking_max_vel_x",
                  docking_max_vel_x_, 0.90);
        nh_.param("/line2o_left/docking_max_vel_y",
                  docking_max_vel_y_, 0.40);
        nh_.param("/line2o_left/docking_max_vel_theta",
                  docking_max_vel_theta_, 0.90);
        nh_.param("/line2o_left/docking_acc_lim_x",
                  docking_acc_lim_x_, 2.00);
        nh_.param("/line2o_left/docking_acc_lim_y",
                  docking_acc_lim_y_, 2.00);
        nh_.param("/line2o_left/docking_acc_lim_theta",
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
        docking_relaxed_yaw_tolerance_ = std::max(
            docking_yaw_tolerance_, docking_relaxed_yaw_tolerance_);
        docking_relaxed_hold_time_ =
            std::max(0.0, docking_relaxed_hold_time_);
        docking_timeout_ = std::max(1.0, docking_timeout_);
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
        docking_acc_lim_x_ =
            std::max(0.0, docking_acc_lim_x_);
        docking_acc_lim_y_ =
            std::max(0.0, docking_acc_lim_y_);
        docking_acc_lim_theta_ =
            std::max(0.0, docking_acc_lim_theta_);

        barrier_min_scan_range_ = std::max(0.0, barrier_min_scan_range_);
        barrier_max_scan_range_ =
            std::max(barrier_min_scan_range_ + 0.01, barrier_max_scan_range_);
        if (field_min_x_ > field_max_x_) {
            std::swap(field_min_x_, field_max_x_);
        }
        if (field_min_y_ > field_max_y_) {
            std::swap(field_min_y_, field_max_y_);
        }
        const double half_min_field_size = 0.5 * std::min(
            field_max_x_ - field_min_x_, field_max_y_ - field_min_y_);
        barrier_boundary_margin_ = clamp(
            barrier_boundary_margin_, 0.0,
            std::max(0.0, half_min_field_size - 0.001));
        barrier_candidate_min_map_x_ = clamp(
            barrier_candidate_min_map_x_, field_min_x_, field_max_x_);
        barrier_neighbor_max_distance_ =
            std::max(0.001, barrier_neighbor_max_distance_);
        barrier_min_points_ = std::max(2, barrier_min_points_);
        barrier_min_line_length_ = std::max(0.0, barrier_min_line_length_);
        barrier_trigger_zone_min_line_length_ = clamp(
            barrier_trigger_zone_min_line_length_,
            0.0,
            barrier_min_line_length_);
        barrier_max_rms_error_ = std::max(0.001, barrier_max_rms_error_);
        barrier_endpoint_margin_ = std::max(0.0, barrier_endpoint_margin_);
        barrier_trigger_distance_ = std::max(0.01, barrier_trigger_distance_);
        barrier_no_drift_distance_ = std::max(0.01, barrier_no_drift_distance_);
        barrier_slowdown_start_distance_ = std::max(
            barrier_trigger_distance_ + 0.01,
            barrier_slowdown_start_distance_);
        barrier_slowdown_min_speed_ratio_ = clamp(
            barrier_slowdown_min_speed_ratio_, 0.0, 1.0);
        barrier_confirm_scans_ = std::max(1, barrier_confirm_scans_);
        barrier_confirm_max_distance_change_ =
            std::max(0.0, barrier_confirm_max_distance_change_);
        barrier_confirm_max_angle_change_deg_ =
            clamp(barrier_confirm_max_angle_change_deg_, 0.0, 90.0);
        barrier_debug_log_interval_ =
            std::max(0.10, barrier_debug_log_interval_);
        barrier_scan_timeout_ = std::max(0.20, barrier_scan_timeout_);

        avoid_left_distance_ = std::max(0.0, avoid_left_distance_);
        avoid_forward_distance_ = std::max(0.0, avoid_forward_distance_);
        avoid_right_distance_ = std::max(0.0, avoid_right_distance_);
        avoid_left_speed_ = std::max(0.01, avoid_left_speed_);
        avoid_forward_speed_ = std::max(0.01, avoid_forward_speed_);
        avoid_right_speed_ = std::max(0.01, avoid_right_speed_);
        avoid_control_rate_ = std::max(5.0, avoid_control_rate_);
        avoid_position_kp_ = std::max(0.01, avoid_position_kp_);
        avoid_yaw_kp_ = std::max(0.0, avoid_yaw_kp_);
        avoid_max_angular_speed_ = std::max(0.0, avoid_max_angular_speed_);
        avoid_rotate_min_angular_speed_ = clamp(
            avoid_rotate_min_angular_speed_, 0.0, avoid_max_angular_speed_);
        avoid_rotate_tolerance_deg_ = clamp(
            avoid_rotate_tolerance_deg_, 0.10, 30.0);
        avoid_rotate_timeout_ = std::max(0.50, avoid_rotate_timeout_);
        avoid_heading_pause_error_deg_ =
            clamp(avoid_heading_pause_error_deg_, 0.0, 90.0);
        avoid_position_tolerance_ = std::max(0.005, avoid_position_tolerance_);
        avoid_min_linear_speed_ = std::max(0.0, avoid_min_linear_speed_);
        avoid_segment_timeout_ = std::max(0.5, avoid_segment_timeout_);
        avoid_stop_hold_time_ = std::max(0.0, avoid_stop_hold_time_);
        avoid_segment_pause_time_ = std::max(0.0, avoid_segment_pause_time_);
        avoid_acc_lim_x_ = std::max(0.0, avoid_acc_lim_x_);
        avoid_acc_lim_y_ = std::max(0.0, avoid_acc_lim_y_);
        avoid_acc_lim_theta_ = std::max(0.0, avoid_acc_lim_theta_);
        avoid_camera_flush_frames_ = std::max(0, avoid_camera_flush_frames_);
        avoid_move_base_extension_distance_ =
            std::max(0.01, avoid_move_base_extension_distance_);
        if (!std::isfinite(avoid_move_base_parallel_shift_distance_)) {
            avoid_move_base_parallel_shift_distance_ = 0.0;
        }
        if (!std::isfinite(avoid_move_base_heading_target_x_)) {
            avoid_move_base_heading_target_x_ = 4.75;
        }
        if (!std::isfinite(avoid_move_base_heading_target_y_)) {
            avoid_move_base_heading_target_y_ = 0.25;
        }
        if (!std::isfinite(avoid_move_base_min_target_x_)) {
            avoid_move_base_min_target_x_ = 0.0;
        }
        if (!std::isfinite(avoid_fixed_target_x_)) {
            avoid_fixed_target_x_ = 0.0;
        }
        if (!std::isfinite(avoid_fixed_target_y_)) {
            avoid_fixed_target_y_ = 0.0;
        }
        if (!std::isfinite(avoid_fixed_target_yaw_deg_)) {
            avoid_fixed_target_yaw_deg_ = -90.0;
        }
        avoid_move_base_timeout_ = std::max(0.50, avoid_move_base_timeout_);
        if (std::abs(avoid_recenter_y_kp_) < 1e-6) {
            avoid_recenter_y_kp_ = 0.004;
        }
        avoid_recenter_min_y_speed_ =
            std::max(0.0, avoid_recenter_min_y_speed_);
        avoid_recenter_max_y_speed_ =
            std::max(0.01, avoid_recenter_max_y_speed_);
        avoid_recenter_min_y_speed_ = std::min(
            avoid_recenter_min_y_speed_, avoid_recenter_max_y_speed_);
        avoid_recenter_line_error_tolerance_ =
            std::max(0.1, avoid_recenter_line_error_tolerance_);
        avoid_recenter_confirm_frames_ =
            std::max(1, avoid_recenter_confirm_frames_);
        avoid_recenter_timeout_ =
            std::max(0.50, avoid_recenter_timeout_);

        ROS_INFO(
            "左巡线参数加载完成：center_distance=%d，二值化=(%d,%d,%d)",
            center_distance,
            adaptive_block_, adaptive_c_, min_contour_area_);
        ROS_INFO(
            "AMCL初始位姿=(%.3f, %.3f, %.1f°)，"
            "停靠触发：x>%.3f且距(%.3f, %.3f)<%.3fm，"
            "要求避障成功=%s，最终方向=%.1f°",
            initial_pose_x_, initial_pose_y_, initial_pose_yaw_deg_,
            docking_trigger_min_x_,
            docking_goal_x_, docking_goal_y_,
            docking_trigger_distance_,
            docking_require_avoidance_complete_ ? "是" : "否",
            docking_goal_yaw_deg_);
        ROS_INFO(
            "挡板检测：仅对map_x>%.3fm的雷达点执行后续候选计算，"
            "场地x=[%.2f, %.2f]m、y=[%.2f, %.2f]m，"
            "距边界>%.3fm，最近邻<%.3fm；"
            "触发距离<%.3fm，最少%d点，远场线长>=%.3fm，"
            "触发区线长>=%.3fm，RMS<=%.3fm；"
            "%.3fm内线性减速至x_max的%.0f%%",
            barrier_candidate_min_map_x_,
            field_min_x_, field_max_x_, field_min_y_, field_max_y_,
            barrier_boundary_margin_, barrier_neighbor_max_distance_,
            barrier_trigger_distance_,
            barrier_min_points_, barrier_min_line_length_,
            barrier_trigger_zone_min_line_length_, barrier_max_rms_error_,
            barrier_slowdown_start_distance_,
            barrier_slowdown_min_speed_ratio_ * 100.0);
        ROS_INFO(
            "挡板近区丢线转向抑制：板距<=%.3fm后锁存生效，"
            "到本次避障结束前禁止因连续丢线触发固定转向；避障结束自动恢复原逻辑。",
            barrier_no_drift_distance_);
        ROS_INFO(
            "一次性绕障：先沿锁存板方向的反方向平移%.3fm（限速%.3fm/s），"
            "再从锁存板中点沿前向板法向延伸%.3fm，"
            "并沿板平移%.3fm（正值默认向map_x减小方向），"
            "目标航向指向参考点(%.3f, %.3f)，"
            "目标x不得小于%.3fm，MoveBase超时=%.2fs；"
            "第三段固定目标模式=%s，固定目标=(%.3f, %.3f, %.2f°)；"
            "到达后直接恢复正常巡线。",
            avoid_left_distance_, avoid_left_speed_,
            avoid_move_base_extension_distance_,
            avoid_move_base_parallel_shift_distance_,
            avoid_move_base_heading_target_x_,
            avoid_move_base_heading_target_y_,
            avoid_move_base_min_target_x_,
            avoid_move_base_timeout_,
            avoid_use_fixed_target_ ? "开启" : "关闭",
            avoid_fixed_target_x_, avoid_fixed_target_y_,
            avoid_fixed_target_yaw_deg_);
    }

    // 初始化ROS组件（客户端、发布者等）
    void initRosComponents() {
        // 初始化速度发布者
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("cmd_vel发布者已初始化");

        scan_sub_ = nh_.subscribe(
            scan_topic_, 1, &LineFollowerNode::scanCallback, this);
        scan_watchdog_timer_ = nh_.createWallTimer(
            ros::WallDuration(0.50),
            &LineFollowerNode::scanWatchdogCallback,
            this);
        ROS_INFO("挡板检测雷达订阅已初始化：%s", scan_topic_.c_str());

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
        "/line2o_left/calibration_file",
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
        bool avoidance_failed = false;
        bool docking_failed = false;

        // 每次开始巡线前重新读取YAML/rosparam，使场地坐标修改无需改代码。
        loadParameters();

        // 清除上一次服务调用遗留的巡线状态。
        double_line_ = false;
        right_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        lost_start_angular_z_ = 0.0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();
        resetObstacleState();

        // 小车尚未开始移动时，强制设置本次巡线使用的AMCL初始位姿。
        publishInitialPose();

        if (!initCameraAndVideo()) {
            ROS_FATAL("相机或视频初始化失败，节点无法启动");
            stopRobot();
            return false;
        }

        line_service_active_.store(true);
        ROS_INFO("line2o_left巡线启动：挡板检测已启用，本次服务最多触发一次。");

        while (ros::ok()) {
            // 雷达回调在独立线程内先发布零速；主循环随后接管一次性绕障。
            if (obstacle_triggered_.load() &&
                !avoidance_completed_.load()) {
                stopRobot();
                if (!runOneTimeAvoidance()) {
                    avoidance_failed = true;
                    break;
                }
                // 不读取旧图像，直接进入下一轮并恢复正常巡线。
                continue;
            }

            // 先检查AMCL定位触发条件；一旦满足，同一控制周期直接转入停靠，
            // 不在两种控制之间插入零速度。
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (getRobotPose(robot_x, robot_y, robot_yaw)) {
                const double distance_to_goal =
                    std::hypot(docking_goal_x_ - robot_x,
                               docking_goal_y_ - robot_y);

                const bool inside_docking_trigger =
                    robot_x > docking_trigger_min_x_ &&
                    distance_to_goal < docking_trigger_distance_;
                // 避障互锁：挡板已经触发但尚未完成，或当前仍处于避障控制
                // （包含三段避障执行期间）时，绝不允许切入终点停靠。
                const bool avoidance_in_progress =
                    avoidance_active_.load() ||
                    (obstacle_triggered_.load() &&
                     !avoidance_completed_.load());
                const bool docking_permission =
                    !avoidance_in_progress &&
                    (!docking_require_avoidance_complete_ ||
                     avoidance_succeeded_.load());

                if (inside_docking_trigger && avoidance_in_progress) {
                    ROS_WARN_THROTTLE(
                        0.5,
                        "已进入终点停靠触发区域，但当前仍在挡板三段避障中；"
                        "禁止触发终点停靠，等待避障完整结束。");
                } else if (inside_docking_trigger && !docking_permission) {
                    ROS_WARN_THROTTLE(
                        0.5,
                        "已进入终点停靠触发区域，但挡板避障尚未成功完成；"
                        "终点目标保持锁定，继续当前巡线/避障流程。");
                }

                if (inside_docking_trigger && docking_permission) {
                    switch_to_docking = true;
                    ROS_INFO(
                        "满足停靠触发条件：当前位置=(%.3f, %.3f)，x>%.3f，"
                        "距终点=%.3fm<%.3fm，避障许可已打开；"
                        "无停顿切入纯PP停靠。",
                        robot_x, robot_y,
                        docking_trigger_min_x_,
                        distance_to_goal,
                        docking_trigger_distance_);
                    break;
                }
            }

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
            threshold(gray_img, brightness_threshold_image,
                      brightness_threshold, 255, THRESH_BINARY);
            threshold_image(gray_img);
            cv::cvtColor(gray_img, cropped, cv::COLOR_GRAY2BGR);

            // 只执行原有视觉巡线；停车白线检测已完全删除。
            runNormalTracking(gray_img, cropped);
            // 小于减速起点后，按拟合直线最短距离限制巡线前向速度。
            // 只是速度上限，不改原巡线PID计算和角速度。
            applyBarrierTrackingSpeedLimit();
            // 若雷达恰好在图像处理期间触发，零速已经由雷达线程发布，
            // 此处禁止再用本帧旧巡线速度覆盖急停指令。
            if (!obstacle_triggered_.load()) {
                cmd_pub_.publish(twist_);
            }
        }

        line_service_active_.store(false);

        if (avoidance_failed) {
            stopRobot();
        } else if (ros::ok() && switch_to_docking) {
            docking_failed = !runDockingControl();
        } else {
            stopRobot();
        }

        // 为下一次服务调用复位状态。
        double_line_ = false;
        right_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        lost_start_angular_z_ = 0.0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();
        cap_.release();
        out_.release();
        return !avoidance_failed && !docking_failed;
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

    static double degToRad(double degrees) {
        return degrees * 3.14159265358979323846 / 180.0;
    }

    bool fitBarrierCluster(
        const vector<std::pair<double, double>>& points,
        const ros::Time& stamp,
        BarrierLineFit& fit) const {
        fit = BarrierLineFit();
        fit.stamp = stamp;
        fit.point_count = static_cast<int>(points.size());

        if (static_cast<int>(points.size()) < barrier_min_points_) {
            fit.rejection_reason = "点数不足";
            return false;
        }

        double center_x = 0.0;
        double center_y = 0.0;
        for (const auto& point : points) {
            center_x += point.first;
            center_y += point.second;
        }
        center_x /= static_cast<double>(points.size());
        center_y /= static_cast<double>(points.size());

        double covariance_xx = 0.0;
        double covariance_xy = 0.0;
        double covariance_yy = 0.0;
        for (const auto& point : points) {
            const double dx = point.first - center_x;
            const double dy = point.second - center_y;
            covariance_xx += dx * dx;
            covariance_xy += dx * dy;
            covariance_yy += dy * dy;
        }

        // 二维总最小二乘：协方差最大特征值对应挡板的直线方向。
        const double direction_angle =
            0.5 * std::atan2(2.0 * covariance_xy,
                             covariance_xx - covariance_yy);
        double direction_x = std::cos(direction_angle);
        double direction_y = std::sin(direction_angle);

        // 继续统一把直线方向指向车体左侧；第一段执行时对该方向取反。
        if (direction_y < 0.0 ||
            (std::abs(direction_y) < 1e-9 && direction_x < 0.0)) {
            direction_x = -direction_x;
            direction_y = -direction_y;
        }

        double normal_x = -direction_y;
        double normal_y = direction_x;
        // 保留前向法线的统一方向，供原有诊断输出和兼容逻辑使用。
        if (normal_x < 0.0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }

        const double signed_distance =
            normal_x * center_x + normal_y * center_y;
        const double line_distance = std::abs(signed_distance);

        double min_projection = std::numeric_limits<double>::infinity();
        double max_projection = -std::numeric_limits<double>::infinity();
        double squared_residual_sum = 0.0;
        for (const auto& point : points) {
            const double dx = point.first - center_x;
            const double dy = point.second - center_y;
            const double along = direction_x * dx + direction_y * dy;
            const double residual = normal_x * dx + normal_y * dy;
            min_projection = std::min(min_projection, along);
            max_projection = std::max(max_projection, along);
            squared_residual_sum += residual * residual;
        }

        const double line_length = max_projection - min_projection;
        const double rms_error = std::sqrt(
            squared_residual_sum / static_cast<double>(points.size()));

        // 使用观测线段两端投影的中点，而不是直接使用点云均值。
        // 这样不会因为挡板一端雷达点更密而把中点拉偏。
        const double midpoint_projection =
            0.5 * (min_projection + max_projection);
        const double segment_center_x =
            center_x + direction_x * midpoint_projection;
        const double segment_center_y =
            center_y + direction_y * midpoint_projection;

        // 仍计算直线相对车体横向轴的偏差，但V6仅将它用于诊断日志。
        // 场地边界和map_x>3已用于确认障碍物位置，不再因挡板
        // 稍偏左、未与车体横向轴平行而拒绝候选直线。
        const double direction_deviation_from_lateral =
            std::atan2(std::abs(direction_x), std::abs(direction_y));

        // 原点到无限直线的垂足还必须落在实际观测线段附近，避免用远处墙面的
        // 延长线错误触发挡板。
        const double foot_projection =
            -(direction_x * center_x + direction_y * center_y);
        const bool foot_on_observed_segment =
            foot_projection >= min_projection - barrier_endpoint_margin_ &&
            foot_projection <= max_projection + barrier_endpoint_margin_;

        // 即使质量检查未通过也保留拟合数值，便于现场日志直接看出失败原因。
        fit.computed = true;
        fit.distance = line_distance;
        fit.direction_x = direction_x;
        fit.direction_y = direction_y;
        fit.normal_x = normal_x;
        fit.normal_y = normal_y;
        fit.center_x = segment_center_x;
        fit.center_y = segment_center_y;
        fit.length = line_length;
        fit.rms_error = rms_error;
        fit.direction_angle_deg =
            std::atan2(direction_y, direction_x) * 180.0 /
            3.14159265358979323846;
        fit.normal_angle_deg =
            std::atan2(normal_y, normal_x) * 180.0 /
            3.14159265358979323846;
        fit.lateral_deviation_deg =
            direction_deviation_from_lateral * 180.0 /
            3.14159265358979323846;
        fit.inside_trigger_zone =
            line_distance <= barrier_trigger_distance_;
        fit.required_min_line_length = fit.inside_trigger_zone
            ? barrier_trigger_zone_min_line_length_
            : barrier_min_line_length_;
        fit.foot_on_observed_segment = foot_on_observed_segment;

        if (line_length < fit.required_min_line_length) {
            fit.rejection_reason = fit.inside_trigger_zone
                ? "触发区线段长度仍不足"
                : "远场线段长度不足";
            return false;
        }
        if (rms_error > barrier_max_rms_error_) {
            fit.rejection_reason = "RMS拟合误差过大";
            return false;
        }
        if (!foot_on_observed_segment) {
            fit.rejection_reason = "垂足不在观测线段内";
            return false;
        }

        fit.valid = true;
        fit.rejection_reason =
            fit.inside_trigger_zone &&
            line_length < barrier_min_line_length_
                ? "通过质量检查（近场短线门槛）"
                : "通过质量检查";
        return true;
    }

    void scanWatchdogCallback(const ros::WallTimerEvent&) {
        if (!line_service_active_.load() ||
            avoidance_completed_.load()) {
            return;
        }

        const double last_scan_time = last_scan_wall_time_sec_.load();
        const double now = ros::WallTime::now().toSec();
        if (last_scan_time <= 0.0 ||
            now - last_scan_time > barrier_scan_timeout_) {
            ROS_WARN_THROTTLE(
                barrier_scan_timeout_,
                "挡板检测未收到雷达数据：topic=%s，已超过%.2fs。"
                "请检查 rostopic hz %s 以及YAML中的scan_topic。",
                scan_topic_.c_str(), barrier_scan_timeout_,
                scan_topic_.c_str());
        }
    }

    double calculateBarrierTrackingSpeedLimit(
        const BarrierLineFit& fit) {
        if (!fit.valid ||
            fit.distance >= barrier_slowdown_start_distance_) {
            return std::numeric_limits<double>::infinity();
        }

        const double slowdown_span =
            barrier_slowdown_start_distance_ - barrier_trigger_distance_;
        const double distance_progress = clamp(
            (fit.distance - barrier_trigger_distance_) / slowdown_span,
            0.0,
            1.0);
        const double speed_ratio =
            barrier_slowdown_min_speed_ratio_ +
            (1.0 - barrier_slowdown_min_speed_ratio_) * distance_progress;
        return std::abs(x_max_) * speed_ratio;
    }

    void applyBarrierTrackingSpeedLimit() {
        const double speed_limit = barrier_tracking_speed_limit_.load();
        if (!std::isfinite(speed_limit)) {
            return;
        }

        // 只限制视觉巡线的前后向线速度，不改角速度、横向速度
        // 和后续绕障控制。原指令已低于上限时保持原值。
        twist_.linear.x = clamp(
            twist_.linear.x, -speed_limit, speed_limit);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        last_scan_wall_time_sec_.store(ros::WallTime::now().toSec());
        ROS_INFO_ONCE(
            "挡板检测首次收到雷达：topic=%s，frame=%s，点数=%zu，"
            "角度范围=[%.1f°, %.1f°]。",
            scan_topic_.c_str(), scan->header.frame_id.c_str(),
            scan->ranges.size(),
            scan->angle_min * 180.0 / 3.14159265358979323846,
            scan->angle_max * 180.0 / 3.14159265358979323846);

        if (!line_service_active_.load() ||
            obstacle_triggered_.load() ||
            avoidance_active_.load() ||
            avoidance_completed_.load()) {
            return;
        }

        if (tf_listener_ == nullptr || scan->header.frame_id.empty()) {
            ROS_WARN_THROTTLE(
                barrier_debug_log_interval_,
                "挡板定位暂停：TF监听器未就绪或LaserScan frame_id为空。");
            return;
        }

        // 使用最新map<-laser变换。这里只用map坐标过滤固定场地边界，
        // 实际直线仍在原LaserScan坐标系中拟合，保持V4的角度与距离定义。
        tf::StampedTransform map_from_scan;
        try {
            tf_listener_->lookupTransform(
                map_frame_, scan->header.frame_id, ros::Time(0), map_from_scan);
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(
                barrier_debug_log_interval_,
                "挡板定位暂停：无法获取%s <- %s的TF：%s",
                map_frame_.c_str(), scan->header.frame_id.c_str(), ex.what());
            return;
        }

        struct CandidatePoint {
            double laser_x;
            double laser_y;
        };

        vector<CandidatePoint> spatial_candidates;
        BarrierLineFit best_fit;
        BarrierLineFit nearest_observed_fit;
        size_t valid_range_point_count = 0;
        size_t x_region_point_count = 0;
        size_t map_filter_point_count = 0;
        size_t neighbor_point_count = 0;
        int cluster_count = 0;
        int computed_fit_count = 0;

        const auto fit_cluster = [&](
            const vector<std::pair<double, double>>& cluster) {
            ++cluster_count;
            BarrierLineFit cluster_fit;
            bool accepted = fitBarrierCluster(
                cluster, scan->header.stamp, cluster_fit);
            if (cluster_fit.computed) {
                const tf::Vector3 center_map = map_from_scan * tf::Vector3(
                    cluster_fit.center_x, cluster_fit.center_y, 0.0);
                cluster_fit.center_map_x = center_map.x();
                cluster_fit.center_map_y = center_map.y();
                cluster_fit.center_map_valid =
                    std::isfinite(cluster_fit.center_map_x) &&
                    std::isfinite(cluster_fit.center_map_y);
                if (!cluster_fit.center_map_valid) {
                    accepted = false;
                    cluster_fit.valid = false;
                    cluster_fit.rejection_reason = "挡板中点map坐标无效";
                }

                // 只应用TF旋转部分，把触发帧的挡板法向同步锁存到map坐标系。
                // 该法向稍后会根据终点位置选择正负方向。
                const tf::Vector3 normal_map =
                    map_from_scan.getBasis() * tf::Vector3(
                        cluster_fit.normal_x, cluster_fit.normal_y, 0.0);
                const double normal_map_length = std::hypot(
                    normal_map.x(), normal_map.y());
                cluster_fit.normal_map_valid =
                    std::isfinite(normal_map.x()) &&
                    std::isfinite(normal_map.y()) &&
                    normal_map_length > 1e-9;
                if (cluster_fit.normal_map_valid) {
                    cluster_fit.normal_map_x =
                        normal_map.x() / normal_map_length;
                    cluster_fit.normal_map_y =
                        normal_map.y() / normal_map_length;
                } else {
                    accepted = false;
                    cluster_fit.valid = false;
                    cluster_fit.rejection_reason = "挡板法向map向量无效";
                }

                ++computed_fit_count;
                if (!nearest_observed_fit.computed ||
                    cluster_fit.distance < nearest_observed_fit.distance) {
                    nearest_observed_fit = cluster_fit;
                }
            }
            if (accepted &&
                (!best_fit.valid || cluster_fit.distance < best_fit.distance)) {
                best_fit = cluster_fit;
            }
        };

        // LaserScan本身不含map_x，因此每个有效量测只做一次必要的坐标变换。
        // 变换后首先检查map_x > 3m；不满足时立即跳过，不再参与
        // 边界判定、最近邻计算、连通分组和直线拟合。
        for (size_t index = 0; index < scan->ranges.size(); ++index) {
            const double angle =
                scan->angle_min + static_cast<double>(index) * scan->angle_increment;
            const double range = scan->ranges[index];

            const bool valid =
                std::isfinite(range) &&
                range >= std::max(static_cast<double>(scan->range_min),
                                  barrier_min_scan_range_) &&
                range <= std::min(static_cast<double>(scan->range_max),
                                  barrier_max_scan_range_);

            if (!valid) {
                continue;
            }

            ++valid_range_point_count;
            const double laser_x = range * std::cos(angle);
            const double laser_y = range * std::sin(angle);
            const tf::Vector3 point_map =
                map_from_scan * tf::Vector3(laser_x, laser_y, 0.0);
            const double point_map_x = point_map.x();
            const double point_map_y = point_map.y();

            if (point_map_x <= barrier_candidate_min_map_x_) {
                continue;
            }
            ++x_region_point_count;

            const bool away_from_all_boundaries =
                point_map_x > field_min_x_ + barrier_boundary_margin_ &&
                point_map_x < field_max_x_ - barrier_boundary_margin_ &&
                point_map_y > field_min_y_ + barrier_boundary_margin_ &&
                point_map_y < field_max_y_ - barrier_boundary_margin_;
            if (away_from_all_boundaries) {
                spatial_candidates.push_back({laser_x, laser_y});
                ++map_filter_point_count;
            }
        }

        // 第二层：每个候选点必须在10cm范围内至少有一个同样通过
        // map过滤的雷达点。然后以同一阈值建立连通点簇，禁止跨空间点簇拟合。
        const size_t candidate_count = spatial_candidates.size();
        vector<bool> has_near_neighbor(candidate_count, false);
        for (size_t i = 0; i < candidate_count; ++i) {
            for (size_t j = i + 1; j < candidate_count; ++j) {
                const double distance = std::hypot(
                    spatial_candidates[i].laser_x - spatial_candidates[j].laser_x,
                    spatial_candidates[i].laser_y - spatial_candidates[j].laser_y);
                if (distance < barrier_neighbor_max_distance_) {
                    has_near_neighbor[i] = true;
                    has_near_neighbor[j] = true;
                }
            }
        }
        neighbor_point_count = static_cast<size_t>(std::count(
            has_near_neighbor.begin(), has_near_neighbor.end(), true));

        vector<bool> visited(candidate_count, false);
        for (size_t seed = 0; seed < candidate_count; ++seed) {
            if (visited[seed] || !has_near_neighbor[seed]) {
                continue;
            }

            vector<size_t> open_set(1, seed);
            visited[seed] = true;
            vector<std::pair<double, double>> cluster;

            for (size_t open_index = 0;
                 open_index < open_set.size(); ++open_index) {
                const size_t current = open_set[open_index];
                cluster.emplace_back(
                    spatial_candidates[current].laser_x,
                    spatial_candidates[current].laser_y);

                for (size_t other = 0; other < candidate_count; ++other) {
                    if (visited[other] || !has_near_neighbor[other]) {
                        continue;
                    }
                    const double distance = std::hypot(
                        spatial_candidates[current].laser_x -
                            spatial_candidates[other].laser_x,
                        spatial_candidates[current].laser_y -
                            spatial_candidates[other].laser_y);
                    if (distance < barrier_neighbor_max_distance_) {
                        visited[other] = true;
                        open_set.push_back(other);
                    }
                }
            }

            fit_cluster(cluster);
        }

        const double approach_speed_limit =
            calculateBarrierTrackingSpeedLimit(best_fit);
        barrier_tracking_speed_limit_.store(approach_speed_limit);

        // 只要一个通过现有质量检查的挡板进入设定近区，就锁存禁止丢线固定转向。
        // 这里故意使用锁存而不是逐帧开关，避免雷达距离在0.35m附近抖动时反复启停。
        // 挡板触发/避障本身的判定条件完全不变。
        if (best_fit.valid &&
            best_fit.distance <= barrier_no_drift_distance_ &&
            !avoidance_completed_.load()) {
            if (!barrier_no_drift_active_.exchange(true)) {
                ROS_WARN(
                    "进入挡板近区丢线转向抑制：板距=%.3fm<=%.3fm；"
                    "从现在到避障结束前，连续丢线不再触发固定转向。",
                    best_fit.distance,
                    barrier_no_drift_distance_);
            }
        }

        if (barrier_debug_log_enabled_) {
            if (nearest_observed_fit.computed) {
                ROS_INFO_THROTTLE(
                    barrier_debug_log_interval_,
                    "挡板拟合：frame=%s，有效量测=%zu，map_x>%.3fm后=%zu，"
                    "再距边界>%.3fm后=%zu，最近邻<%.3fm后=%zu，点簇=%d，"
                    "完成拟合=%d；最近直线：板角=%.2f°，法线角=%.2f°，"
                    "最短距离=%.3fm，中点map=(%.3f, %.3f)，"
                    "点数=%d，线长=%.3fm/要求>=%.3fm，"
                    "RMS=%.4fm，"
                    "横向偏差(仅诊断)=%.2f°，结果=%s。",
                    scan->header.frame_id.c_str(), valid_range_point_count,
                    barrier_candidate_min_map_x_, x_region_point_count,
                    barrier_boundary_margin_, map_filter_point_count,
                    barrier_neighbor_max_distance_, neighbor_point_count,
                    cluster_count, computed_fit_count,
                    nearest_observed_fit.direction_angle_deg,
                    nearest_observed_fit.normal_angle_deg,
                    nearest_observed_fit.distance,
                    nearest_observed_fit.center_map_x,
                    nearest_observed_fit.center_map_y,
                    nearest_observed_fit.point_count,
                    nearest_observed_fit.length,
                    nearest_observed_fit.required_min_line_length,
                    nearest_observed_fit.rms_error,
                    nearest_observed_fit.lateral_deviation_deg,
                    nearest_observed_fit.rejection_reason.c_str());
            } else {
                ROS_WARN_THROTTLE(
                    barrier_debug_log_interval_,
                    "挡板拟合：已收到雷达，但map筛选后没有可拟合直线。"
                    "frame=%s，有效量测=%zu，map_x>%.3fm后=%zu，"
                    "再距边界>%.3fm后=%zu，最近邻<%.3fm后=%zu，"
                    "点簇=%d，每簇至少需要%d点。",
                    scan->header.frame_id.c_str(), valid_range_point_count,
                    barrier_candidate_min_map_x_, x_region_point_count,
                    barrier_boundary_margin_, map_filter_point_count,
                    barrier_neighbor_max_distance_,
                    neighbor_point_count, cluster_count, barrier_min_points_);
            }

            if (best_fit.valid) {
                ROS_INFO_THROTTLE(
                    barrier_debug_log_interval_,
                    "有效挡板候选：板角=%.2f°，法线角=%.2f°，"
                    "最短距离=%.3fm，触发阈值=%.3fm，当前%s。",
                    best_fit.direction_angle_deg,
                    best_fit.normal_angle_deg,
                    best_fit.distance,
                    barrier_trigger_distance_,
                    best_fit.distance <= barrier_trigger_distance_
                        ? "达到触发条件"
                        : "尚未达到触发距离");

                if (std::isfinite(approach_speed_limit)) {
                    ROS_INFO_THROTTLE(
                        barrier_debug_log_interval_,
                        "障碍物接近减速：最短距离=%.3fm，"
                        "减速起点=%.3fm，触发距离=%.3fm，"
                        "当前巡线线速度上限=%.3fm/s（x_max=%.3fm/s）。",
                        best_fit.distance,
                        barrier_slowdown_start_distance_,
                        barrier_trigger_distance_,
                        approach_speed_limit,
                        std::abs(x_max_));
                }
            }
        }

        bool trigger_now = false;
        {
            std::lock_guard<std::mutex> lock(barrier_mutex_);

            if (!best_fit.valid ||
                best_fit.distance > barrier_trigger_distance_) {
                barrier_confirm_count_ = 0;
                candidate_fit_ = BarrierLineFit();
                return;
            }

            bool consistent = candidate_fit_.valid;
            if (consistent) {
                const double direction_dot = std::abs(
                    candidate_fit_.direction_x * best_fit.direction_x +
                    candidate_fit_.direction_y * best_fit.direction_y);
                const double min_dot = std::cos(
                    degToRad(barrier_confirm_max_angle_change_deg_));
                consistent =
                    std::abs(candidate_fit_.distance - best_fit.distance) <=
                        barrier_confirm_max_distance_change_ &&
                    direction_dot >= min_dot;
            }

            barrier_confirm_count_ = consistent
                ? barrier_confirm_count_ + 1
                : 1;
            candidate_fit_ = best_fit;

            if (barrier_confirm_count_ >= barrier_confirm_scans_) {
                locked_fit_ = best_fit;
                trigger_now = !obstacle_triggered_.exchange(true);
            }
        }

        if (trigger_now) {
            // 雷达线程先直接发零速，避免等待下一帧相机处理才停车。
            geometry_msgs::Twist emergency_stop;
            cmd_pub_.publish(emergency_stop);
            ROS_WARN(
                "挡板触发急停：板角=%.2f°，法线角=%.2f°，"
                "最短距离=%.3fm<=%.3fm，点数=%d，线长=%.3fm，"
                "当前线长门槛=%.3fm，RMS=%.4fm，"
                "锁存挡板中点map=(%.3f, %.3f)；拟合数据不再更新。",
                best_fit.direction_angle_deg, best_fit.normal_angle_deg,
                best_fit.distance, barrier_trigger_distance_,
                best_fit.point_count, best_fit.length,
                best_fit.required_min_line_length, best_fit.rms_error,
                best_fit.center_map_x, best_fit.center_map_y);
        }
    }

    void resetObstacleState() {
        std::lock_guard<std::mutex> lock(barrier_mutex_);
        candidate_fit_ = BarrierLineFit();
        locked_fit_ = BarrierLineFit();
        barrier_confirm_count_ = 0;
        obstacle_triggered_.store(false);
        avoidance_active_.store(false);
        avoidance_completed_.store(false);
        avoidance_succeeded_.store(false);
        barrier_no_drift_active_.store(false);
        barrier_tracking_speed_limit_.store(
            std::numeric_limits<double>::infinity());
    }

    void applyAvoidanceAccelerationLimits(
        const geometry_msgs::Twist& desired_cmd,
        double dt) {
        dt = clamp(dt, 0.005, 0.20);
        const double max_delta_x = avoid_acc_lim_x_ * dt;
        const double max_delta_y = avoid_acc_lim_y_ * dt;
        const double max_delta_theta = avoid_acc_lim_theta_ * dt;

        twist_.linear.x = avoid_acc_lim_x_ > 0.0
            ? clamp(desired_cmd.linear.x,
                    twist_.linear.x - max_delta_x,
                    twist_.linear.x + max_delta_x)
            : desired_cmd.linear.x;
        twist_.linear.y = avoid_acc_lim_y_ > 0.0
            ? clamp(desired_cmd.linear.y,
                    twist_.linear.y - max_delta_y,
                    twist_.linear.y + max_delta_y)
            : desired_cmd.linear.y;
        twist_.angular.z = avoid_acc_lim_theta_ > 0.0
            ? clamp(desired_cmd.angular.z,
                    twist_.angular.z - max_delta_theta,
                    twist_.angular.z + max_delta_theta)
            : desired_cmd.angular.z;
    }

    bool executeAvoidanceSegment(
        const string& segment_name,
        double direction_body_x,
        double direction_body_y,
        double distance,
        double max_speed,
        double direction_reference_yaw,
        double target_yaw) {
        if (distance <= avoid_position_tolerance_) {
            ROS_INFO("绕障%s距离为0，跳过该段。", segment_name.c_str());
            return true;
        }

        double start_x = 0.0;
        double start_y = 0.0;
        double start_yaw = 0.0;
        if (!getRobotPose(start_x, start_y, start_yaw)) {
            ROS_ERROR("绕障%s开始前无法获取定位。", segment_name.c_str());
            stopRobot();
            return false;
        }

        // 移动方向始终使用挡板触发时的车体坐标系转换到地图系；
        // 车身朝向则可以独立保持为触发朝向或旋转后的挡板法线方向。
        const double cos_locked = std::cos(direction_reference_yaw);
        const double sin_locked = std::sin(direction_reference_yaw);
        const double direction_map_x =
            cos_locked * direction_body_x - sin_locked * direction_body_y;
        const double direction_map_y =
            sin_locked * direction_body_x + cos_locked * direction_body_y;
        const double target_x = start_x + direction_map_x * distance;
        const double target_y = start_y + direction_map_y * distance;

        ros::WallRate control_rate(avoid_control_rate_);
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(avoid_segment_timeout_);
        ros::WallTime last_time = ros::WallTime::now();

        ROS_INFO(
            "绕障%s开始：起点=(%.3f, %.3f)，目标=(%.3f, %.3f)，"
            "距离=%.3fm，限速=%.3fm/s，保持yaw=%.2f°。",
            segment_name.c_str(), start_x, start_y, target_x, target_y,
            distance, max_speed,
            target_yaw * 180.0 / 3.14159265358979323846);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
                ROS_ERROR("绕障%s过程中定位丢失，安全停车。", segment_name.c_str());
                stopRobot();
                return false;
            }

            const double error_map_x = target_x - robot_x;
            const double error_map_y = target_y - robot_y;
            const double position_error = std::hypot(error_map_x, error_map_y);
            const double yaw_error = normalizeAngle(target_yaw - robot_yaw);

            if (position_error <= avoid_position_tolerance_) {
                stopRobot();
                ROS_INFO(
                    "绕障%s完成：当前位置=(%.3f, %.3f)，位置误差=%.3fm，"
                    "方向误差=%.2f°。",
                    segment_name.c_str(), robot_x, robot_y, position_error,
                    yaw_error * 180.0 / 3.14159265358979323846);
                return true;
            }

            double velocity_map_x = avoid_position_kp_ * error_map_x;
            double velocity_map_y = avoid_position_kp_ * error_map_y;
            double speed = std::hypot(velocity_map_x, velocity_map_y);
            if (speed > max_speed) {
                const double scale = max_speed / speed;
                velocity_map_x *= scale;
                velocity_map_y *= scale;
                speed = max_speed;
            } else if (speed > 1e-9 && speed < avoid_min_linear_speed_) {
                const double limited_min = std::min(avoid_min_linear_speed_, max_speed);
                const double scale = limited_min / speed;
                velocity_map_x *= scale;
                velocity_map_y *= scale;
            }

            geometry_msgs::Twist desired_cmd;
            const double cos_yaw = std::cos(robot_yaw);
            const double sin_yaw = std::sin(robot_yaw);
            desired_cmd.linear.x =
                cos_yaw * velocity_map_x + sin_yaw * velocity_map_y;
            desired_cmd.linear.y =
                -sin_yaw * velocity_map_x + cos_yaw * velocity_map_y;
            desired_cmd.angular.z = clamp(
                avoid_yaw_kp_ * yaw_error,
                -avoid_max_angular_speed_,
                avoid_max_angular_speed_);

            if (std::abs(yaw_error) >
                degToRad(avoid_heading_pause_error_deg_)) {
                desired_cmd.linear.x = 0.0;
                desired_cmd.linear.y = 0.0;
            }

            const ros::WallTime now = ros::WallTime::now();
            const double dt = (now - last_time).toSec();
            last_time = now;
            applyAvoidanceAccelerationLimits(desired_cmd, dt);
            cmd_pub_.publish(twist_);

            ROS_INFO_THROTTLE(
                0.4,
                "绕障%s：剩余=%.3fm，yaw误差=%.2f°，cmd=(%.3f, %.3f, %.3f)",
                segment_name.c_str(), position_error,
                yaw_error * 180.0 / 3.14159265358979323846,
                twist_.linear.x, twist_.linear.y, twist_.angular.z);
            control_rate.sleep();
        }

        ROS_ERROR("绕障%s超过%.2fs仍未完成，安全停车。",
                  segment_name.c_str(), avoid_segment_timeout_);
        stopRobot();
        return false;
    }

    bool executeAvoidanceRotation(
        double locked_yaw,
        const BarrierLineFit& locked_fit,
        double& perpendicular_yaw) {
        const double pi = 3.14159265358979323846;

        // normal_x在拟合阶段已强制为非负，因此这里只会选择车头前方的
        // 挡板法线。旋转增量天然位于[-90°, +90°]，不会转向反向法线。
        double rotation_delta =
            std::atan2(locked_fit.normal_y, locked_fit.normal_x);
        rotation_delta = clamp(rotation_delta, -0.5 * pi, 0.5 * pi);
        perpendicular_yaw = normalizeAngle(locked_yaw + rotation_delta);

        if (avoid_max_angular_speed_ <= 0.0) {
            ROS_ERROR("绕障旋转速度上限为0，无法转至挡板垂直方向。");
            stopRobot();
            return false;
        }

        ros::WallRate control_rate(avoid_control_rate_);
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(avoid_rotate_timeout_);
        ros::WallTime last_time = ros::WallTime::now();

        ROS_WARN(
            "前两段平移完成，开始转至挡板前向法线："
            "原yaw=%.2f°，只选前向法线，旋转增量=%.2f°，目标yaw=%.2f°。",
            locked_yaw * 180.0 / pi,
            rotation_delta * 180.0 / pi,
            perpendicular_yaw * 180.0 / pi);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
                ROS_ERROR("绕障旋转过程中定位丢失，安全停车。");
                stopRobot();
                return false;
            }

            const double yaw_error =
                normalizeAngle(perpendicular_yaw - robot_yaw);
            if (std::abs(yaw_error) <=
                degToRad(avoid_rotate_tolerance_deg_)) {
                stopRobot();
                ROS_WARN(
                    "挡板垂直方向旋转完成：当前yaw=%.2f°，目标yaw=%.2f°，"
                    "误差=%.2f°。",
                    robot_yaw * 180.0 / pi,
                    perpendicular_yaw * 180.0 / pi,
                    yaw_error * 180.0 / pi);
                return true;
            }

            geometry_msgs::Twist desired_cmd;
            double angular_speed = clamp(
                avoid_yaw_kp_ * yaw_error,
                -avoid_max_angular_speed_,
                avoid_max_angular_speed_);
            angular_speed = applyMinimumMagnitude(
                angular_speed, avoid_rotate_min_angular_speed_);
            desired_cmd.angular.z = clamp(
                angular_speed,
                -avoid_max_angular_speed_,
                avoid_max_angular_speed_);

            const ros::WallTime now = ros::WallTime::now();
            const double dt = (now - last_time).toSec();
            last_time = now;
            applyAvoidanceAccelerationLimits(desired_cmd, dt);
            cmd_pub_.publish(twist_);

            ROS_INFO_THROTTLE(
                0.25,
                "转至挡板前向法线中：当前yaw=%.2f°，目标yaw=%.2f°，"
                "剩余=%.2f°，角速度=%.3f。",
                robot_yaw * 180.0 / pi,
                perpendicular_yaw * 180.0 / pi,
                yaw_error * 180.0 / pi,
                twist_.angular.z);
            control_rate.sleep();
        }

        ROS_ERROR(
            "超过%.2fs仍未转到挡板垂直方向，安全停车。",
            avoid_rotate_timeout_);
        stopRobot();
        return false;
    }

    // 计算“原MoveBase避障”使用的最终目标位姿。
    // 几何公式、沿板方向约定、x下限和航向参考点均与原
    // executeMoveBaseGoalFromBarrierCenter() 完全一致；V17/V3仅改执行器，
    // 不改变原MoveBase目标坐标与目标姿态的计算方式。
    bool computeLegacyMoveBaseTargetPose(
        const BarrierLineFit& locked_fit,
        double& target_x,
        double& target_y,
        double& target_yaw) const {
        if (!locked_fit.center_map_valid || !locked_fit.normal_map_valid) {
            return false;
        }

        const double normal_x = locked_fit.normal_map_x;
        const double normal_y = locked_fit.normal_map_y;

        double parallel_x = normal_y;
        double parallel_y = -normal_x;
        if (parallel_x > 1e-9 ||
            (std::abs(parallel_x) <= 1e-9 && parallel_y < 0.0)) {
            parallel_x = -parallel_x;
            parallel_y = -parallel_y;
        }

        const double base_target_x = locked_fit.center_map_x +
            normal_x * avoid_move_base_extension_distance_;
        const double base_target_y = locked_fit.center_map_y +
            normal_y * avoid_move_base_extension_distance_;
        const double raw_target_x = base_target_x +
            parallel_x * avoid_move_base_parallel_shift_distance_;
        target_y = base_target_y +
            parallel_y * avoid_move_base_parallel_shift_distance_;
        target_x = std::max(raw_target_x, avoid_move_base_min_target_x_);

        const double heading_dx =
            avoid_move_base_heading_target_x_ - target_x;
        const double heading_dy =
            avoid_move_base_heading_target_y_ - target_y;
        target_yaw = std::atan2(normal_y, normal_x);
        if (std::hypot(heading_dx, heading_dy) > 1e-6) {
            target_yaw = std::atan2(heading_dy, heading_dx);
        } else {
            ROS_WARN(
                "避障点与航向参考点重合，yaw无法由连线确定；"
                "本次安全回退为前向板法向。"
            );
        }

        return std::isfinite(target_x) &&
               std::isfinite(target_y) &&
               std::isfinite(target_yaw);
    }

    // 第三段：直接对绝对map目标做二维P平移，同时对目标yaw做P控制。
    // 位置未到容差前，vx/vy与wz始终同时输出：不再因为姿态误差较大而暂停平移。
    // 只有位置和姿态都满足容差才算完整第三段完成。
    bool executeAvoidanceFinalPoseTarget(
        const string& segment_name,
        double target_x,
        double target_y,
        double target_yaw,
        double max_speed) {
        double start_x = 0.0;
        double start_y = 0.0;
        double start_yaw = 0.0;
        if (!getRobotPose(start_x, start_y, start_yaw)) {
            ROS_ERROR("%s开始前无法读取AMCL定位。", segment_name.c_str());
            stopRobot();
            return false;
        }

        ros::WallRate control_rate(avoid_control_rate_);
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(avoid_move_base_timeout_);
        ros::WallTime last_time = ros::WallTime::now();

        ROS_WARN(
            "%s开始：起点=(%.3f, %.3f, %.2f°)，最终目标=(%.3f, %.3f, %.2f°)，"
            "平移限速=%.3fm/s，位置容差=%.3fm，姿态容差=%.2f°。",
            segment_name.c_str(),
            start_x, start_y,
            start_yaw * 180.0 / 3.14159265358979323846,
            target_x, target_y,
            target_yaw * 180.0 / 3.14159265358979323846,
            max_speed, avoid_position_tolerance_,
            avoid_rotate_tolerance_deg_);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
                ROS_ERROR("%s过程中定位丢失，安全停车。", segment_name.c_str());
                stopRobot();
                return false;
            }

            const double error_map_x = target_x - robot_x;
            const double error_map_y = target_y - robot_y;
            const double position_error = std::hypot(error_map_x, error_map_y);
            const double yaw_error = normalizeAngle(target_yaw - robot_yaw);
            const bool position_ok =
                position_error <= avoid_position_tolerance_;
            const bool yaw_ok =
                std::abs(yaw_error) <= degToRad(avoid_rotate_tolerance_deg_);

            if (position_ok && yaw_ok) {
                stopRobot();
                ROS_WARN(
                    "%s完成：当前位置=(%.3f, %.3f, %.2f°)，"
                    "目标=(%.3f, %.3f, %.2f°)，位置误差=%.3fm，姿态误差=%.2f°。",
                    segment_name.c_str(),
                    robot_x, robot_y,
                    robot_yaw * 180.0 / 3.14159265358979323846,
                    target_x, target_y,
                    target_yaw * 180.0 / 3.14159265358979323846,
                    position_error,
                    yaw_error * 180.0 / 3.14159265358979323846);
                return true;
            }

            geometry_msgs::Twist desired_cmd;

            // 位置未到位时，按map坐标误差做二维P控制，再转换到当前车体坐标系。
            if (!position_ok) {
                double velocity_map_x = avoid_position_kp_ * error_map_x;
                double velocity_map_y = avoid_position_kp_ * error_map_y;
                double speed = std::hypot(velocity_map_x, velocity_map_y);
                if (speed > max_speed) {
                    const double scale = max_speed / speed;
                    velocity_map_x *= scale;
                    velocity_map_y *= scale;
                } else if (speed > 1e-9 && speed < avoid_min_linear_speed_) {
                    const double target_speed =
                        std::min(avoid_min_linear_speed_, max_speed);
                    const double scale = target_speed / speed;
                    velocity_map_x *= scale;
                    velocity_map_y *= scale;
                }

                const double cos_yaw = std::cos(robot_yaw);
                const double sin_yaw = std::sin(robot_yaw);
                desired_cmd.linear.x =
                    cos_yaw * velocity_map_x + sin_yaw * velocity_map_y;
                desired_cmd.linear.y =
                    -sin_yaw * velocity_map_x + cos_yaw * velocity_map_y;
            }

            if (!yaw_ok) {
                double angular_speed = clamp(
                    avoid_yaw_kp_ * yaw_error,
                    -avoid_max_angular_speed_,
                    avoid_max_angular_speed_);
                angular_speed = applyMinimumMagnitude(
                    angular_speed, avoid_rotate_min_angular_speed_);
                desired_cmd.angular.z = clamp(
                    angular_speed,
                    -avoid_max_angular_speed_,
                    avoid_max_angular_speed_);
            }

            // 第三段位置与姿态同步闭环：即使yaw误差较大，
            // 也不再清零vx/vy，保持一边平移一边旋转。

            const ros::WallTime now = ros::WallTime::now();
            const double dt = (now - last_time).toSec();
            last_time = now;
            applyAvoidanceAccelerationLimits(desired_cmd, dt);
            cmd_pub_.publish(twist_);

            ROS_INFO_THROTTLE(
                0.4,
                "%s：当前位置=(%.3f, %.3f, %.2f°)，剩余=%.3fm，"
                "yaw误差=%.2f°，cmd=(%.3f, %.3f, %.3f)。",
                segment_name.c_str(),
                robot_x, robot_y,
                robot_yaw * 180.0 / 3.14159265358979323846,
                position_error,
                yaw_error * 180.0 / 3.14159265358979323846,
                twist_.linear.x, twist_.linear.y, twist_.angular.z);
            control_rate.sleep();
        }

        ROS_ERROR(
            "%s超过%.2fs仍未完成最终位置和姿态调整，安全停车。",
            segment_name.c_str(), avoid_move_base_timeout_);
        stopRobot();
        return false;
    }

    bool executeMoveBaseGoalFromBarrierCenter(
        const BarrierLineFit& locked_fit) {
        if (ac_ == nullptr) {
            ROS_ERROR("MoveBase客户端未初始化，安全停车。");
            stopRobot();
            return false;
        }
        if (!locked_fit.center_map_valid || !locked_fit.normal_map_valid) {
            ROS_ERROR("锁存挡板中点或挡板法向的map数据无效，安全停车。");
            stopRobot();
            return false;
        }

        // normal_map在挡板触发帧中已统一为朝向车头前方一侧的单位法向。
        const double normal_x = locked_fit.normal_map_x;
        const double normal_y = locked_fit.normal_map_y;

        // 由法向旋转90°得到沿板单位向量，再统一其正方向：优先令map_x
        // 分量为负；若板方向恰好没有x分量，则令map_y分量为正。
        double parallel_x = normal_y;
        double parallel_y = -normal_x;
        if (parallel_x > 1e-9 ||
            (std::abs(parallel_x) <= 1e-9 && parallel_y < 0.0)) {
            parallel_x = -parallel_x;
            parallel_y = -parallel_y;
        }

        const double base_target_x = locked_fit.center_map_x +
            normal_x * avoid_move_base_extension_distance_;
        const double base_target_y = locked_fit.center_map_y +
            normal_y * avoid_move_base_extension_distance_;
        const double raw_target_x = base_target_x +
            parallel_x * avoid_move_base_parallel_shift_distance_;
        const double target_y = base_target_y +
            parallel_y * avoid_move_base_parallel_shift_distance_;
        // 保留V11的x坐标下限；y坐标保持法向延伸和沿板平移结果。
        const double target_x =
            std::max(raw_target_x, avoid_move_base_min_target_x_);
        // 位置仍完全沿用V15的几何计算；姿态单独使用
        // “最终避障点 -> YAML参考点”的连线方向。
        const double heading_dx =
            avoid_move_base_heading_target_x_ - target_x;
        const double heading_dy =
            avoid_move_base_heading_target_y_ - target_y;
        double target_yaw = std::atan2(normal_y, normal_x);
        if (std::hypot(heading_dx, heading_dy) > 1e-6) {
            target_yaw = std::atan2(heading_dy, heading_dx);
        } else {
            ROS_WARN(
                "避障点与航向参考点重合，yaw无法由连线确定；"
                "本次安全回退为前向板法向。");
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = target_x;
        goal.target_pose.pose.position.y = target_y;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(target_yaw);

        stopRobot();
        ROS_WARN(
            "发送绕障MoveBase目标：锁存挡板中点=(%.3f, %.3f)，"
            "前向板法向=(%.3f, %.3f)，沿板正方向=(%.3f, %.3f)，"
            "法向延伸=%.3fm，沿板平移=%.3fm，"
            "基础目标=(%.3f, %.3f)，原始目标x=%.3f，x下限=%.3f，"
            "最终目标=(%.3f, %.3f)，航向参考点=(%.3f, %.3f)，"
            "指向参考点的最终yaw=%.2f°。",
            locked_fit.center_map_x, locked_fit.center_map_y,
            normal_x, normal_y,
            parallel_x, parallel_y,
            avoid_move_base_extension_distance_,
            avoid_move_base_parallel_shift_distance_,
            base_target_x, base_target_y,
            raw_target_x, avoid_move_base_min_target_x_,
            target_x, target_y,
            avoid_move_base_heading_target_x_,
            avoid_move_base_heading_target_y_,
            target_yaw * 180.0 / 3.14159265358979323846);

        ac_->sendGoal(goal);
        const bool finished =
            ac_->waitForResult(ros::Duration(avoid_move_base_timeout_));
        if (!finished) {
            ac_->cancelGoal();
            stopRobot();
            ROS_ERROR(
                "绕障MoveBase目标超过%.2fs仍未完成，"
                "已取消目标并安全停车。",
                avoid_move_base_timeout_);
            return false;
        }

        const actionlib::SimpleClientGoalState state = ac_->getState();
        if (!(state == actionlib::SimpleClientGoalState::SUCCEEDED)) {
            stopRobot();
            ROS_ERROR(
                "绕障MoveBase目标未成功：state=%s，安全停车。",
                state.toString().c_str());
            return false;
        }

        stopRobot();
        ROS_WARN(
            "绕障MoveBase目标完成：target=(%.3f, %.3f, %.2f°)。",
            target_x, target_y,
            target_yaw * 180.0 / 3.14159265358979323846);
        return true;
    }

    bool runVisualLateralRecentering() {
        ROS_WARN(
            "MoveBase目标已到达，开始视觉横移回线："
            "强制linear.x=0，仅发布linear.y和angular.z；"
            "居中门槛=%.1f像素，连续%d帧，超时=%.2fs。",
            avoid_recenter_line_error_tolerance_,
            avoid_recenter_confirm_frames_,
            avoid_recenter_timeout_);

        integration_ = 0.0;
        pre_error_ = 0.0;
        trace_failed_count_ = 0;
        lost_start_angular_z_ = 0.0;
        stopRobot();

        const ros::WallTime start_time = ros::WallTime::now();
        ros::WallTime last_control_time = start_time;
        ros::Rate control_rate(avoid_control_rate_);
        int centered_frame_count = 0;

        while (ros::ok()) {
            const ros::WallTime now = ros::WallTime::now();
            if ((now - start_time).toSec() > avoid_recenter_timeout_) {
                ROS_ERROR(
                    "视觉横移回线超过%.2fs仍未连续居中，安全停车。",
                    avoid_recenter_timeout_);
                stopRobot();
                return false;
            }

            Mat image;
            if (!cap_.read(image) || image.empty()) {
                ROS_WARN_THROTTLE(
                    0.5, "视觉横移回线阶段未读取到有效相机画面。");
                control_rate.sleep();
                continue;
            }

            Mat cropped = image(roi_);
            flip(cropped, cropped, 1);
            vector<Mat> channels;
            split(cropped, channels);
            Mat gray_img = channels[2];
            const int brightness_threshold =
                brightness_threshold_calculator(gray_img, cropped);
            Mat brightness_threshold_image;
            threshold(gray_img, brightness_threshold_image,
                      brightness_threshold, 255, THRESH_BINARY);
            threshold_image(gray_img);
            cv::cvtColor(gray_img, cropped, cv::COLOR_GRAY2BGR);

            vector<Point> start_points =
                find_track_edge(gray_img, 340, 70, cropped);
            RaceTrack racetrack;
            geometry_msgs::Twist desired_cmd;

            if (trace_edge(gray_img, start_points, racetrack, cropped)) {
                trace_failed_count_ = 0;
            lost_start_angular_z_ = 0.0;
        lost_start_angular_z_ = 0.0;
                const double line_error =
                    error_calculater(racetrack.points, cropped);

                // 与正常巡线完全相同的误差、积分、微分和角速度计算。
                integration_ += line_error * 0.03;
                integration_ = clamp(
                    integration_,
                    -std::abs(line_error) / integration_limit_ - 1.0,
                    std::abs(line_error) / integration_limit_ + 1.0);
                double diff = line_error - pre_error_;
                diff = clamp(diff, -50.0, 50.0);
                desired_cmd.angular.z = clamp(
                    line_error * p_ + integration_ * i_ + diff * d_,
                    -1.0, 1.0);
                pre_error_ = line_error;

                // 回线阶段不允许前后移动。横向速度按同一个巡线误差闭环；
                // kp允许在YAML中写负值，便于现场直接反转底盘横移方向。
                desired_cmd.linear.x = 0.0;
                if (std::abs(line_error) <=
                    avoid_recenter_line_error_tolerance_) {
                    desired_cmd.linear.y = 0.0;
                    ++centered_frame_count;
                } else {
                    centered_frame_count = 0;
                    double lateral_speed = clamp(
                        avoid_recenter_y_kp_ * line_error,
                        -avoid_recenter_max_y_speed_,
                        avoid_recenter_max_y_speed_);
                    lateral_speed = applyMinimumMagnitude(
                        lateral_speed, avoid_recenter_min_y_speed_);
                    desired_cmd.linear.y = lateral_speed;
                }

                displayStream_.str("");
                displayStream_
                    << "横移回线误差: " << line_error
                    << " y速度: " << desired_cmd.linear.y
                    << " 角速度: " << desired_cmd.angular.z
                    << " 居中帧: " << centered_frame_count
                    << "/" << avoid_recenter_confirm_frames_;
                putText(
                    cropped, displayStream_.str(), Point(30, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5,
                    Scalar(255, 255, 0), 1);

                if (centered_frame_count >=
                    avoid_recenter_confirm_frames_) {
                    out_.write(cropped);
                    stopRobot();
                    trace_failed_count_ = 0;
            lost_start_angular_z_ = 0.0;
        lost_start_angular_z_ = 0.0;
                    integration_ = 0.0;
                    pre_error_ = 0.0;
                    ROS_WARN(
                        "视觉横移回线完成：|巡线误差|连续%d帧<=%.1f像素，"
                        "恢复正常前向巡线。",
                        centered_frame_count,
                        avoid_recenter_line_error_tolerance_);
                    return true;
                }

                ROS_INFO_THROTTLE(
                    0.25,
                    "视觉横移回线中：误差=%.2f像素，"
                    "cmd=(x=0.000, y=%.3f, z=%.3f)，居中=%d/%d。",
                    line_error, desired_cmd.linear.y,
                    desired_cmd.angular.z,
                    centered_frame_count,
                    avoid_recenter_confirm_frames_);
            } else {
                centered_frame_count = 0;
                ++trace_failed_count_;
                desired_cmd.linear.x = 0.0;
                desired_cmd.linear.y = 0.0;

                // 保留正常巡线连续丢失5帧后的向右搜线方向，
                // 但回线阶段仍严格禁止linear.x前进。
                if (trace_failed_count_ > 5) {
                    desired_cmd.angular.z = -std::abs(out_turn_);
                }

                ROS_WARN_THROTTLE(
                    0.5,
                    "视觉横移回线暂未找到右线：连续丢失=%d，"
                    "cmd=(x=0.000, y=0.000, z=%.3f)。",
                    trace_failed_count_, desired_cmd.angular.z);
            }

            const double dt = (now - last_control_time).toSec();
            last_control_time = now;
            applyAvoidanceAccelerationLimits(desired_cmd, dt);
            // 双重保险：即使上一阶段遗留速度或限加速度，也绝不发布x速度。
            twist_.linear.x = 0.0;
            cmd_pub_.publish(twist_);
            out_.write(cropped);
            control_rate.sleep();
        }

        stopRobot();
        return false;
    }

    bool runOneTimeAvoidance() {
        BarrierLineFit locked_fit;
        {
            std::lock_guard<std::mutex> lock(barrier_mutex_);
            locked_fit = locked_fit_;
        }
        if (!locked_fit.valid) {
            ROS_ERROR("收到挡板触发，但锁定直线无效，安全停车。");
            stopRobot();
            return false;
        }

        avoidance_active_.store(true);
        stopRobot();
        if (avoid_stop_hold_time_ > 0.0) {
            ros::WallDuration(avoid_stop_hold_time_).sleep();
        }

        double robot_x = 0.0;
        double robot_y = 0.0;
        double locked_yaw = 0.0;
        if (!getRobotPose(robot_x, robot_y, locked_yaw)) {
            ROS_ERROR("绕障开始时无法锁定车身朝向，安全停车。");
            avoidance_active_.store(false);
            avoidance_completed_.store(true);
            barrier_no_drift_active_.store(false);
            return false;
        }

        // 第三段目标选择：默认完全复用原MoveBase目标；固定模式则使用
        // YAML绝对map位姿。第一、二段路径与锁定朝向不受该开关影响。
        double final_target_x = 0.0;
        double final_target_y = 0.0;
        double final_target_yaw = 0.0;
        const char* final_target_source = nullptr;
        if (avoid_use_fixed_target_) {
            final_target_x = avoid_fixed_target_x_;
            final_target_y = avoid_fixed_target_y_;
            final_target_yaw = normalizeAngle(
                degToRad(avoid_fixed_target_yaw_deg_));
            final_target_source = "YAML固定目标";
            ROS_WARN(
                "第三段固定目标模式开启：跳过挡板几何终点计算，"
                "采用YAML绝对map目标=(%.3f, %.3f, %.2f°)。",
                final_target_x, final_target_y,
                avoid_fixed_target_yaw_deg_);
        } else {
            if (!computeLegacyMoveBaseTargetPose(
                    locked_fit,
                    final_target_x, final_target_y, final_target_yaw)) {
                ROS_ERROR("无法计算原MoveBase最终目标位姿，安全终止三段避障。");
                stopRobot();
                avoidance_active_.store(false);
                avoidance_completed_.store(true);
                barrier_no_drift_active_.store(false);
                return false;
            }
            final_target_source = "挡板几何动态目标";

            // false标定模式：将本次实际采用的第三段目标直接按YAML格式输出。
            // 下次只需复制这四行并把开关保持为true，即可固化本次目标。
            ROS_WARN(
                "========== 本次动态避障目标【可直接写入line2o_left.yaml】 ==========\n"
                "avoid_use_fixed_target: true\n"
                "avoid_fixed_target_x: %.3f\n"
                "avoid_fixed_target_y: %.3f\n"
                "avoid_fixed_target_yaw_deg: %.2f\n"
                "================================================================",
                final_target_x,
                final_target_y,
                final_target_yaw * 180.0 / 3.14159265358979323846);
        }

        ROS_WARN(
            "进入三段式避障：锁定yaw=%.2f°，板方向=(%.3f, %.3f)，"
            "板前向法向=(%.3f, %.3f)，挡板中点map=(%.3f, %.3f)。",
            locked_yaw * 180.0 / 3.14159265358979323846,
            locked_fit.direction_x, locked_fit.direction_y,
            locked_fit.normal_x, locked_fit.normal_y,
            locked_fit.center_map_x, locked_fit.center_map_y);
        ROS_WARN(
            "三段式避障最终目标："
            "target=(%.3f, %.3f, %.2f°)，source=%s；"
            "流程=%s %.3fm -> 第二段垂直板前移 %.3fm -> "
            "第三段P控制平移+旋转到最终目标位姿。",
            final_target_x, final_target_y,
            final_target_yaw * 180.0 / 3.14159265358979323846,
            final_target_source,
            "第一段沿板反向平移", avoid_left_distance_, avoid_forward_distance_);

        const bool first_shift_ok = executeAvoidanceSegment(
            "第一段沿板反向平移",
            -locked_fit.direction_x, -locked_fit.direction_y,
            avoid_left_distance_, avoid_left_speed_,
            locked_yaw, locked_yaw);
        if (first_shift_ok && avoid_segment_pause_time_ > 0.0) {
            ros::WallDuration(avoid_segment_pause_time_).sleep();
        }

        const bool forward_ok = first_shift_ok && executeAvoidanceSegment(
            "第二段垂直板向前平移",
            locked_fit.normal_x, locked_fit.normal_y,
            avoid_forward_distance_, avoid_forward_speed_,
            locked_yaw, locked_yaw);
        if (forward_ok && avoid_segment_pause_time_ > 0.0) {
            ros::WallDuration(avoid_segment_pause_time_).sleep();
        }

        const bool final_pose_ok = forward_ok && executeAvoidanceFinalPoseTarget(
            "第三段最终位姿P控制",
            final_target_x, final_target_y, final_target_yaw,
            avoid_right_speed_);

        if (final_pose_ok) {
            for (int i = 0; i < avoid_camera_flush_frames_; ++i) {
                cap_.grab();
            }

            // 只有完整三段全部成功后才打开最终停靠许可。
            avoidance_succeeded_.store(true);
        }

        stopRobot();
        avoidance_active_.store(false);
        avoidance_completed_.store(true);
        obstacle_triggered_.store(false);
        barrier_no_drift_active_.store(false);
        barrier_tracking_speed_limit_.store(
            std::numeric_limits<double>::infinity());
        ROS_WARN(
            "一次性避障结束：已取消挡板近区丢线转向抑制，"
            "后续恢复原丢线固定转向逻辑。");

        if (!final_pose_ok) {
            return false;
        }

        integration_ = 0.0;
        pre_error_ = 0.0;
        trace_failed_count_ = 0;
        lost_start_angular_z_ = 0.0;
        ROS_WARN(
            "第一段沿板反向平移、第二段垂直板前移和第三段最终位姿调整均已完成；挡板检测永久关闭并恢复line2o_left正常巡线。");
        return true;
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

        // 位置和航向两条控制通道同时工作：只要车头还没有
        // 进入-90°目标容差，平移过程中也持续输出wz转向纠偏。
        double vx = docking_linear_x_gain_ * x_error;
        double vy = docking_linear_y_gain_ * y_error;
        double wz = 0.0;
        if (std::abs(yaw_error) > docking_yaw_tolerance_) {
            wz = applyMinimumMagnitude(
                docking_angular_gain_ * yaw_error,
                docking_min_angular_speed_);
        }

        if (std::abs(x_error) <=
            docking_position_tolerance_ * 0.65) {
            vx = 0.0;
        }
        if (std::abs(y_error) <=
            docking_position_tolerance_ * 0.65) {
            vy = 0.0;
        }
        vx = applyMinimumMagnitude(vx, docking_min_linear_speed_);
        vy = applyMinimumMagnitude(vy, docking_min_linear_speed_);

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
        // 第一道硬互锁：只要挡板避障尚在进行（尤其MoveBase目标执行期间），
        // 即使未来其它分支误调用本函数，也禁止进入终点停靠。
        const bool avoidance_in_progress =
            avoidance_active_.load() ||
            (obstacle_triggered_.load() &&
             !avoidance_completed_.load());
        if (avoidance_in_progress) {
            ROS_ERROR(
                "拒绝进入纯PP终点停靠：当前仍处于挡板三段避障阶段。");
            stopRobot();
            return false;
        }

        // 第二道互锁：避障完成后仍需满足原有的避障成功许可。
        if (docking_require_avoidance_complete_ &&
            !avoidance_succeeded_.load()) {
            ROS_ERROR(
                "拒绝进入纯PP终点停靠：挡板避障尚未成功完成，已安全停车。");
            stopRobot();
            return false;
        }

        // 不继承巡线末帧的旧转向量；仅清空角速度内部状态，
        // 不发布停车指令，vx/vy仍从巡线速度平滑接管。
        twist_.angular.z = 0.0;
        ROS_WARN(
            "进入终点停靠：平移全程同步调整车头至%.1f°。",
            docking_goal_yaw_deg_);

        ros::Rate control_rate(docking_control_rate_);
        ros::Time last_control_time = ros::Time::now();
        const ros::WallTime docking_start_time = ros::WallTime::now();
        ros::WallTime relaxed_hold_start;
        bool relaxed_hold_active = false;

        while (ros::ok()) {
            const ros::WallTime wall_now = ros::WallTime::now();
            if ((wall_now - docking_start_time).toSec() > docking_timeout_) {
                ROS_ERROR(
                    "纯PP停靠超过%.2fs仍未满足完成条件，"
                    "已安全停车并退出本次服务，防止控制循环永久卡住。",
                    docking_timeout_);
                stopRobot();
                return false;
            }

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

            const double distance_error = std::hypot(
                docking_goal_x_ - robot_x,
                docking_goal_y_ - robot_y);
            const double target_yaw = degToRad(docking_goal_yaw_deg_);
            const double yaw_error = std::abs(
                normalizeAngle(target_yaw - robot_yaw));
            const bool within_relaxed_completion =
                distance_error <= docking_position_tolerance_ &&
                yaw_error <= docking_relaxed_yaw_tolerance_;

            if (within_relaxed_completion) {
                if (!relaxed_hold_active) {
                    relaxed_hold_active = true;
                    relaxed_hold_start = wall_now;
                }
                if ((wall_now - relaxed_hold_start).toSec() >=
                    docking_relaxed_hold_time_) {
                    stopRobot();
                    ROS_WARN(
                        "停靠防卡死完成：位置误差=%.3fm<=%.3fm，"
                        "方向误差=%.3frad处于宽松门槛%.3frad内并保持%.2fs；"
                        "停止控制并退出本次服务。",
                        distance_error, docking_position_tolerance_,
                        yaw_error, docking_relaxed_yaw_tolerance_,
                        docking_relaxed_hold_time_);
                    return true;
                }
            } else {
                relaxed_hold_active = false;
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

    // 右点追踪逻辑（与line2_left保持一致）
    void runRightPointTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        if (!point_forward_) {
            // 丢线旋转
            ROS_INFO("右点完成后的镜像转向中");
            // out_forward/out_turn在左巡线主流程中用于左线丢失后的固定左转。
            twist_.linear.x = 0.075;
            twist_.angular.z = 1.0;
            out_.write(cropped);
            
            // 旋转到位后切换模式
            pose_client_.call(pose_);
            // ROS_INFO("角度%f,位姿%f",out_turn_angel_,pose_.response.pose_at[2]);
            if (pose_.response.pose_at[2] > -out_turn_angel_) {
                right_point_start_ = false;
                double_line_ = true;
                x_max_ = 0.5;
                nh_.getParam("/line2o_left/double_P", p_);
                nh_.getParam("/line2o_left/double_I", i_);
                nh_.getParam("/line2o_left/double_D", d_);
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

        // 左巡线：从图像左下方和左边界寻找左侧白线的“右边缘”。
        vector<Point> start_points =
            find_track_edge(gray_img, 300, 70, cropped);
        RaceTrack racetrack;

        if (trace_edge(gray_img, start_points, racetrack, cropped)) {
            trace_failed_count_ = 0;
            lost_start_angular_z_ = 0.0;
        lost_start_angular_z_ = 0.0;
            double line_error =
                error_calculater(racetrack.points, cropped);

            integration_ += line_error * 0.03;
            integration_ = clamp(
                integration_,
                -abs(line_error) / integration_limit_ - 1,
                abs(line_error) / integration_limit_ + 1
            );

            double diff = line_error - pre_error_;
            diff = clamp(diff, -50.0, 50.0);

            twist_.linear.x =
                x_max_ / exp(abs(line_error) / 130.0);

            twist_.angular.z = clamp(
                line_error * p_ +
                integration_ * i_ +
                diff * d_,
                -1.0,
                1.0
            );

            pre_error_ = line_error;

            displayStream_
                << "正常误差: " << line_error
                << " P: " << line_error * p_
                << " I: " << integration_ * i_
                << " D: " << diff * d_
                << " 角速度: " << twist_.angular.z;

            putText(
                cropped,
                displayStream_.str(),
                Point(50, 50),
                FONT_HERSHEY_SIMPLEX,
                0.5,
                Scalar(255, 255, 0),
                1
            );
        } else {
            // 与line2_left一致：左线连续丢失5帧后固定左转。
            trace_failed_count_++;

            // 前5个疑似丢线帧只对角速度做线性衰减。
            // 线速度保持原代码行为，不在这里修改linear.x/linear.y。
            //
            // 第1帧记录丢线前最后一帧PID角速度，然后按剩余比例：
            //   wz = lost_start_wz * (1 - failed_count / 5)
            // 第5帧时wz刚好衰减到0；
            // 第6帧及以后仍执行下面原有固定左转逻辑。
            if (trace_failed_count_ == 1) {
                lost_start_angular_z_ = twist_.angular.z;
            }

            if (trace_failed_count_ <= 5) {
                const double remaining =
                    std::max(
                        0.0,
                        1.0 -
                        static_cast<double>(trace_failed_count_) / 5.0
                    );

                twist_.angular.z =
                    lost_start_angular_z_ * remaining;

                ROS_INFO(
                    "左线疑似丢失 %d/5：角速度线性衰减，"
                    "起始wz=%.3f，当前wz=%.3f",
                    trace_failed_count_,
                    lost_start_angular_z_,
                    twist_.angular.z
                );

                displayStream_
                    << "左线疑似丢失 "
                    << trace_failed_count_
                    << "/5，角速度线性衰减"
                    << " start_wz:" << lost_start_angular_z_
                    << " wz:" << twist_.angular.z;

                putText(
                    cropped,
                    displayStream_.str(),
                    Point(50, 50),
                    FONT_HERSHEY_SIMPLEX,
                    0.5,
                    Scalar(0, 255, 255),
                    1
                );
            }

            if (trace_failed_count_ > 5) {
                twist_.linear.x = out_forward_;
                twist_.linear.y = 0.0;

                // V3新增：挡板进入barrier_no_drift_distance后，到本次避障结束前，
                // 即使视觉连续丢线也只保持低速直行，不允许触发固定左转。
                // 避障完成后barrier_no_drift_active_会清零，原固定左转逻辑自动恢复。
                if (barrier_no_drift_active_.load() &&
                    !avoidance_completed_.load()) {
                    twist_.angular.z = 0.0;

                    if (trace_failed_count_ == 6) {
                        ROS_WARN(
                            "挡板近区抑制已生效：左线连续丢失，但板已进入%.3fm范围；"
                            "仅以vx=%.3f直行，禁止固定左转。",
                            barrier_no_drift_distance_,
                            twist_.linear.x
                        );
                    }

                    displayStream_
                        << "挡板近区左线丢失，禁止左转"
                        << " 线速度: " << twist_.linear.x
                        << " 角速度: 0";
                } else {
                    // 原V2逻辑：ROS约定 angular.z > 0 为逆时针，即向左转。
                    twist_.angular.z = std::abs(out_turn_);

                    if (trace_failed_count_ == 6) {
                        ROS_INFO(
                            "左线连续丢失，开始固定左转：线速度=%.3f，角速度=%.3f",
                            twist_.linear.x,
                            twist_.angular.z
                        );
                    }

                    displayStream_
                        << "左线丢失，固定左转"
                        << " 线速度: " << twist_.linear.x
                        << " 角速度: " << twist_.angular.z;
                }

                putText(
                    cropped,
                    displayStream_.str(),
                    Point(50, 50),
                    FONT_HERSHEY_SIMPLEX,
                    0.5,
                    Scalar(0, 165, 255),
                    1
                );
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
    vector<Point> find_track_edge(
        Mat& gray_img,
        int bottom_trace_end,
        int left_trace_end,
        Mat& visual_img
    ) {
        bool is_now_white = false;
        vector<Point> maybe_start_point;

        // 左巡线底部寻找：从左向右找左侧白线的右边缘。
        for (int x = 1; x < bottom_trace_end; ++x) {
            if (!is_now_white &&
                gray_img.at<uchar>(269, x) == 255) {
                is_now_white = true;
            }

            if (is_now_white &&
                gray_img.at<uchar>(269, x) == 0) {
                maybe_start_point.emplace_back(x - 1, 269);
                circle(
                    visual_img,
                    Point(x - 1, 269),
                    5,
                    Scalar(0, 0, 255),
                    -1
                );
                is_now_white = false;
            }
        }

        // 从图像左边界补充寻找起点。
        is_now_white = true;
        for (int y = 269; y > left_trace_end; --y) {
            if (is_now_white &&
                gray_img.at<uchar>(y, 0) == 0) {
                is_now_white = false;
            }

            if (!is_now_white &&
                gray_img.at<uchar>(y, 0) == 255) {
                maybe_start_point.emplace_back(0, y);
                circle(
                    visual_img,
                    Point(0, y),
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
        const int point_number =
            static_cast<int>(start_points.size());
        vector<RaceTrack> racetracks(point_number);

        const int width = gray_img.cols;
        const int search_range = 40;

        for (int idx = 0; idx < point_number; ++idx) {
            bool last_right = true;
            bool last_left = false;
            int fail_count = 0;

            Point start = start_points[idx];
            int center_x = start.x;
            int center_y = start.y - 1;

            while (center_y > start.y - 100) {
                bool right_found = false;
                bool left_found = false;

                for (int dx = 0; dx <= search_range / 2; ++dx) {
                    const int cand_right = center_x + dx;
                    const int cand_left = center_x - dx;

                    // 左侧白线的右边缘：当前像素白，右邻像素黑。
                    if (cand_right > 0 &&
                        cand_right < width - 1 &&
                        gray_img.at<uchar>(
                            center_y, cand_right
                        ) == 255 &&
                        gray_img.at<uchar>(
                            center_y, cand_right + 1
                        ) == 0) {

                        racetracks[idx].points.emplace_back(
                            cand_right,
                            center_y
                        );

                        right_found = true;
                        left_found = false;
                        center_x = cand_right;
                        break;
                    }

                    // 碰到相反边缘时只修正搜索中心，不把它作为目标边缘点。
                    if (cand_left > 1 &&
                        cand_left < width - 1 &&
                        gray_img.at<uchar>(
                            center_y, cand_left
                        ) == 0 &&
                        gray_img.at<uchar>(
                            center_y, cand_left - 1
                        ) == 255) {

                        left_found = true;
                        right_found = false;
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
                    last_left = false;
                    last_right = true;
                }

                if (right_found || left_found) {
                    fail_count = 0;
                    --center_y;
                } else {
                    ++fail_count;
                    --center_y;

                    if (fail_count >= 4) {
                        break;
                    }
                }

                if (center_y <= 0 ||
                    racetracks[idx].points.size() > 60) {
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

                if (std::abs(line_params[0]) > 1e-6) {
                    racetracks[idx].slope =
                        line_params[1] / line_params[0];
                }
            }
        }

        // 水平镜像后，左巡线接受 slope<=-0.05 或 slope>=10。
        int best_idx = -1;
        float min_dangerous = 2.1f;

        for (int i = 0; i < point_number; ++i) {
            if (racetracks[i].points.empty()) {
                continue;
            }

            const double slope = racetracks[i].slope;

            if (!(slope > -0.05 && slope < 10.0)) {
                const float ratio =
                    racetracks[i].direction_change /
                    static_cast<float>(
                        racetracks[i].points.size()
                    );

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

        for (const auto& p : racetrack.points) {
            circle(
                visual_img,
                p,
                2,
                Scalar(0, 255, 0),
                -1
            );
        }

        ostringstream oss;
        oss
            << "斜率: " << racetrack.slope
            << " 方向变化: "
            << racetrack.direction_change;

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

    // 寻找右边缘（line2_left镜像点追踪）
    bool find_right_edge(
        Mat gray_img,
        Point& right_point,
        Mat& visualizeImg
    ) {
        bool is_now_white = false;
        vector<Point> maybe_right_point;

        // 从图像右边缘寻找起点。
        for (int i = 269; i > 2; --i) {
            if (is_now_white &&
                gray_img.at<uchar>(i, 634) == 0) {
                is_now_white = false;
            }

            if (!is_now_white &&
                gray_img.at<uchar>(i, 634) == 255) {
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

        const int point_number =
            static_cast<int>(maybe_right_point.size());
        vector<RaceTrack> racetracks(point_number);
        const int search_range = 40;

        for (int idx = 0; idx < point_number; ++idx) {
            bool last_up = false;
            bool last_down = false;
            int fail_count = 0;

            Point start = maybe_right_point[idx];
            int center_x = start.x - 1;
            int center_y = start.y;

            while (center_x > 19) {
                bool found = false;

                for (int dy = 0; dy <= search_range / 2; ++dy) {
                    const bool up_check =
                        center_y - dy > 2;
                    const bool down_check =
                        center_y + dy < 268;

                    if (down_check &&
                        gray_img.at<uchar>(
                            center_y + dy, center_x
                        ) == 255 &&
                        gray_img.at<uchar>(
                            center_y + dy + 1, center_x
                        ) == 0) {

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

                    if (!found &&
                        up_check &&
                        gray_img.at<uchar>(
                            center_y - dy, center_x
                        ) == 255 &&
                        gray_img.at<uchar>(
                            center_y - dy + 1, center_x
                        ) == 0) {

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
                    --center_x;
                } else {
                    ++fail_count;
                    --center_x;

                    if (fail_count >= 10) {
                        break;
                    }
                }
            }

            if (racetracks[idx].points.size() > 120) {
                racetracks[idx].right_point = true;
            }
        }

        int best_idx = -1;
        int lowest_y = 0;

        for (int i = 0; i < point_number; ++i) {
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
        Point best_point(0, 0);

        for (size_t i = 0;
             i < racetrack.points.size();
             i += 3) {

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
        oss
            << "右点: ("
            << best_point.x
            << ","
            << best_point.y
            << ")";

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
    ros::init(argc, argv, "line2o_left");
    
    // 创建节点对象（构造函数中完成所有初始化）
    LineFollowerNode node;
    
    // 运行节点
    node.run();
    
    return 0;
}