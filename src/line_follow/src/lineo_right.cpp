// 版本：lineo_right 挡板近区禁止丢线右转 + 虚线常开 + 可固定绕障目标点/朝向 V30（2026-08-17）
// 唯一校验标识：LINEO_RIGHT_BARRIER_NO_DRIFT_ZONE_V10
// 避障流程：非边界雷达点拟合挡板 -> 右移 -> 前进 -> P控制平移到最终绕障点；
// 默认动态模式：停车车头射线与挡板求交，沿停车车头前向穿板后继续延伸设定距离得到目标点；
// 固定模式：直接使用YAML给定的绝对map坐标(x,y)和yaw作为绕障目标/全程保持朝向。
// 虚线识别由dashed_line_enable总开关直接控制，服务启动后立即可用；避障后仍按配置永久关闭。
// 新增：避障前进入挡板设定近区后锁存“禁止丢线右转”，允许减速直行等待挡板正常触发；避障后自动取消。
// 启动保护与终点纯PP停靠逻辑保持V12不变。
// 正常巡线新增丢线状态机：疑似丢线时按失败次数线性减速，达到阈值后锁存甩尾；
// 单帧假线不会清空丢线证据；漂移恢复采用线性软退出，连续识别越多，vy/wz越小，确认完成后恢复PID。

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <random>
#include <string>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <clocale>
#include <atomic>
#include <limits>
#include <mutex>
#include "line_follow/line_follow.h"
#include "ucarmain2026/getpose_server.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

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

// 虚线候选白块：只保留每个白块的“左边缘”，这样与原省赛右线追踪
// 使用的白线左边缘坐标体系完全一致，后续center_distance/PID无需改动。
struct DashedComponent {
    Rect bbox;
    double area = 0.0;
    vector<Point> left_edge_points;
    Point2d center;
};

// 精简挡板拟合结果。所有坐标和方向均在map坐标系中锁存。
struct BarrierLineFit {
    bool valid = false;
    ros::Time stamp;
    double center_map_x = 0.0;
    double center_map_y = 0.0;
    double direction_map_x = 0.0;
    double direction_map_y = 1.0;
    double normal_map_x = 1.0;  // 始终选择与触发时车头同向的板法向
    double normal_map_y = 0.0;
    double distance = std::numeric_limits<double>::infinity();
    double length = 0.0;
    double rms_error = std::numeric_limits<double>::infinity();
    int point_count = 0;
};

class LineFollowerNode {
private:
    // ROS核心组件
    ros::NodeHandle nh_;                  // 节点句柄
    ros::ServiceServer line_server_;      // 服务端
    ros::Publisher cmd_pub_;              // 速度发布者
    ros::Publisher initial_pose_pub_;     // AMCL初始位姿发布者
    ros::Subscriber scan_sub_;            // 雷达挡板检测

    ros::ServiceClient pose_client_;      // 位姿服务客户端
    tf::TransformListener* tf_listener_;  // TF监听器

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

    // 实线二值化参数。V21开始真正从YAML读取；默认值保持V20代码实际使用值。
    int adaptive_block_;
    double adaptive_c_;
    double min_contour_area_;

    // 虚线专用低面积二值图 + 几何重建参数。
    bool dashed_line_enable_;
    int dashed_adaptive_block_;
    double dashed_adaptive_c_;
    double dashed_min_contour_area_;
    double dashed_max_contour_area_;
    int dashed_min_component_count_;
    int dashed_min_component_count_without_prior_;
    int dashed_min_component_width_;
    int dashed_min_component_height_;
    int dashed_max_component_width_;
    int dashed_max_component_height_;
    int dashed_min_x_;
    int dashed_min_y_;
    double dashed_component_line_tolerance_px_;
    double dashed_fit_max_error_px_;
    int dashed_min_y_span_;
    int dashed_virtual_max_points_;
    int dashed_bottom_extrapolation_px_;
    int dashed_top_extrapolation_px_;
    bool dashed_use_prior_;
    double dashed_prior_x_tolerance_px_;
    double dashed_prior_angle_tolerance_deg_;

    // V22：虚线只在理论右边线附近的动态走廊内参与候选，避免中央/左侧反光。
    bool dashed_use_expected_corridor_;
    double dashed_corridor_left_margin_px_;
    double dashed_corridor_right_margin_px_;
    double dashed_right_bias_weight_;
    double dashed_pattern_score_weight_;

    // V24遗留兼容参数：V25已停用，不再参与正常白线判定。
    // 只有原trace_edge得到的连续线同时满足长度、理论右侧走廊和上一帧连续性，
    // 才能直接进入PID；否则才允许启动低阈值虚线重建。
    int continuous_right_min_points_;
    double continuous_right_corridor_ratio_;
    double continuous_right_max_prior_jump_px_;
    double continuous_right_max_prior_angle_deg_;

    // V22：连续虚线进入锁定模式；锁定后优先追踪同一条虚线，禁止单帧反光抢线。
    int dashed_lock_confirm_frames_;
    int dashed_lock_release_miss_frames_;
    int dashed_solid_relock_confirm_frames_;
    double dashed_lock_x_tolerance_px_;
    double dashed_lock_angle_tolerance_deg_;

    // V23：DASHED_LOCK内虚线被反光/二值化短暂抹掉时，进入HOLD而不是普通丢线。
    int dashed_hold_frames_;
    double dashed_hold_speed_;
    double dashed_hold_angular_decay_;

    bool disable_dashed_after_avoidance_;

    bool dashed_debug_draw_;

    // 控制参数
    double p_, i_, d_;                    // PID参数
    double leftpoint_p_, leftpoint_I_, leftpoint_D_; // 左点控制参数
    double x_max_, integration_limit_;    // 速度和积分限制
    double out_turn_, out_forward_, out_left_speed_, out_turn_angel_; // 丢线漂移：前进、左移、右转参数
    int lost_confirm_frames_;             // 累计失败达到该帧数后确认丢线并进入漂移
    int lost_cancel_confirm_frames_;      // 疑似丢线后，需连续成功多少帧才取消丢线怀疑
    int drift_recover_confirm_frames_;    // 漂移后，需连续成功多少帧才退出漂移
    double lost_confirm_linear_y_;        // 疑似丢线减速阶段横向速度，默认0
    double lost_confirm_angular_z_;       // 疑似丢线减速阶段角速度，默认0
    double start_straight_speed_;          // 启动阶段直行速度
    double start_straight_distance_;       // 启动后完全屏蔽视觉的保护直行距离
    double start_heading_yaw_deg_;         // 启动直行目标朝向，当前赛道固定为-90°
    double start_heading_kp_;              // 启动直行朝向P控制增益
    double start_heading_max_angular_speed_; // 启动直行最大角速度
    double start_heading_deadband_deg_;    // 朝向误差死区
    double start_heading_pause_error_deg_; // 朝向偏差过大时暂停平移、优先纠正朝向
    double start_control_rate_;            // 启动定向直行控制频率
    int start_line_confirm_frames_;        // 保护距离结束后连续识别多少帧才交给PID
    double integration_, pre_error_;      // 积分和前向误差
    double pointx_integration_, pointx_pre_error_; // 左点积分和前向误差

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
    double docking_trigger_max_y_;
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

    // 精简雷达挡板检测参数
    string scan_topic_;
    double barrier_min_scan_range_;
    double barrier_max_scan_range_;
    double field_min_x_;
    double field_max_x_;
    double field_min_y_;
    double field_max_y_;
    double barrier_boundary_margin_;
    double barrier_candidate_max_map_x_;
    double barrier_neighbor_max_distance_;
    int barrier_min_points_;
    double barrier_min_line_length_;
    double barrier_max_rms_error_;
    double barrier_endpoint_margin_;
    double barrier_trigger_distance_;
    double barrier_lost_mode_trigger_distance_;
    double barrier_no_drift_distance_;          // 避障前进入该板距后禁止丢线右转/甩尾，直到避障结束
    double barrier_slowdown_start_distance_;
    double barrier_slowdown_min_speed_ratio_;
    bool barrier_debug_log_enabled_;
    double barrier_debug_log_interval_;

    // 三段闭环避障参数：车体右移、前进越板、最后P控制到停车车头射线与板交点之后的绕障点
    double avoid_right_distance_;
    double avoid_forward_distance_;
    double avoid_target_extension_distance_;  // 车头射线与板交点沿停车车头前向穿板后继续延伸距离
    // V28：现场标定后可直接覆盖动态计算的最终绕障目标。
    bool avoid_use_fixed_target_;
    double avoid_fixed_target_x_;
    double avoid_fixed_target_y_;
    double avoid_fixed_target_yaw_deg_;
    double avoid_right_speed_;
    double avoid_forward_speed_;
    double avoid_left_speed_;
    double avoid_control_rate_;
    double avoid_position_kp_;
    double avoid_yaw_kp_;
    double avoid_max_angular_speed_;
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

    // 雷达回调和巡线服务由AsyncSpinner并发执行
    std::mutex barrier_mutex_;
    // 统一仲裁“雷达急停”和“启动/视觉巡线速度”，防止零速被旧帧覆盖。
    std::mutex tracking_cmd_publish_mutex_;
    BarrierLineFit latest_barrier_fit_;
    BarrierLineFit locked_barrier_fit_;
    std::atomic<bool> line_service_active_;
    std::atomic<bool> obstacle_triggered_;
    std::atomic<bool> avoidance_active_;
    std::atomic<bool> avoidance_completed_;
    std::atomic<bool> barrier_no_drift_active_;  // 进入挡板近区后锁存，避障完成后释放
    std::atomic<double> barrier_tracking_speed_limit_;

    // 状态变量

    bool double_line_;                    // 双边巡线标志
    bool left_point_start_;               // 左点追踪标志
    bool point_forward_;                  // 左点前进标志
    int trace_failed_count_;              // 疑似丢线阶段累计失败次数（单帧成功不直接清零）
    int lost_cancel_success_count_;       // 疑似丢线阶段连续重新识别成功次数
    int drift_recover_success_count_;     // 漂移阶段连续重新识别成功次数
    bool drift_recovery_active_;          // 已确认丢线并锁存漂移恢复


    // 上一帧有效右边线先验：x = a*y + b。只用于虚线候选消歧，
    // 不参与PID本身，因此不会改变原实线控制行为。
    bool line_prior_valid_;
    double line_prior_a_;
    double line_prior_b_;
    double line_prior_angle_rad_;

    // 虚线运行时门控：服务开始时按dashed_line_enable立即启用；
    // 避障成功后可按配置永久关闭到本次服务结束。
    bool dashed_runtime_enabled_;
    bool dashed_permanently_disabled_;
    bool dashed_lock_active_;
    bool dashed_hold_active_;
    int dashed_lock_confirm_count_;
    int dashed_lock_miss_count_;
    int dashed_solid_relock_count_;

    // V24：保存虚线路段最近一次可靠视觉（连续边或虚线重建）完成PID后的控制量。
    // DASHED_HOLD只读取，不反写，避免把外推控制当成新的视觉观测。
    bool last_dashed_cmd_valid_;
    double last_dashed_vx_;
    double last_dashed_wz_;

public:
    // 构造函数：初始化所有组件
    LineFollowerNode() : 
        nh_(""),
        tf_listener_(nullptr),
        output_file_("/home/ucar/ucar_ws_copy/src/line_follow/image/lineo_right.avi"),
        fourcc_(VideoWriter::fourcc('X', 'V', 'I', 'D')),
        roi_(0, 210, 640, 270),

        line_service_active_(false),
        obstacle_triggered_(false),
        avoidance_active_(false),
        avoidance_completed_(false),
        barrier_no_drift_active_(false),
        barrier_tracking_speed_limit_(
            std::numeric_limits<double>::infinity()),
        double_line_(false),
        left_point_start_(false),
        point_forward_(true),
        trace_failed_count_(0),
        lost_cancel_success_count_(0),
        drift_recover_success_count_(0),
        drift_recovery_active_(false),
        line_prior_valid_(false),
        line_prior_a_(0.0),
        line_prior_b_(0.0),
        line_prior_angle_rad_(0.0),
        dashed_runtime_enabled_(false),
        dashed_permanently_disabled_(false),
        dashed_lock_active_(false),
        dashed_hold_active_(false),
        dashed_lock_confirm_count_(0),
        dashed_lock_miss_count_(0),
        dashed_solid_relock_count_(0),
        last_dashed_cmd_valid_(false),
        last_dashed_vx_(0.0),
        last_dashed_wz_(0.0),
        integration_(0), 
        pre_error_(0),
        pointx_integration_(0),
        pointx_pre_error_(0) {

        ROS_INFO("启动 lineo_right V30（虚线立即可用 + 固定/动态绕障目标切换 + 原稳定实线优先）");

        // 1. 初始化服务端（优先初始化）
        line_server_ = nh_.advertiseService("lineo_right", &LineFollowerNode::line_server_callback, this);
        ROS_INFO("lineo_right服务已初始化");

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
        nh_.getParam("/lineo_right/right_P", p_);
        nh_.getParam("/lineo_right/right_I", i_);
        nh_.getParam("/lineo_right/right_D", d_);
        nh_.getParam("/lineo_right/leftpoint_p", leftpoint_p_);
        nh_.getParam("/lineo_right/leftpoint_I", leftpoint_I_);
        nh_.getParam("/lineo_right/leftpoint_D", leftpoint_D_);
        nh_.getParam("/lineo_right/x_max_", x_max_);
        nh_.param("/lineo_right/start_straight_speed", start_straight_speed_, 0.50);
        nh_.param("/lineo_right/start_straight_distance", start_straight_distance_, 0.20);
        nh_.param("/lineo_right/start_heading_yaw_deg", start_heading_yaw_deg_, -90.0);
        nh_.param("/lineo_right/start_heading_kp", start_heading_kp_, 2.5);
        nh_.param("/lineo_right/start_heading_max_angular_speed",
                  start_heading_max_angular_speed_, 0.60);
        nh_.param("/lineo_right/start_heading_deadband_deg",
                  start_heading_deadband_deg_, 0.5);
        nh_.param("/lineo_right/start_heading_pause_error_deg",
                  start_heading_pause_error_deg_, 6.0);
        nh_.param("/lineo_right/start_control_rate", start_control_rate_, 30.0);
        nh_.param("/lineo_right/start_line_confirm_frames",
                  start_line_confirm_frames_, 3);
        nh_.getParam("/lineo_right/integration_limit", integration_limit_);
        nh_.getParam("/lineo_right/out_forward", out_forward_);
        nh_.param("/lineo_right/out_left_speed", out_left_speed_, 0.16);
        nh_.getParam("/lineo_right/out_turn", out_turn_);
        // 丢线状态机参数。疑似丢线阶段的vx由x_max_与out_forward_自动线性插值。
        nh_.param("/lineo_right/lost_confirm_frames", lost_confirm_frames_, 5);
        nh_.param("/lineo_right/lost_cancel_confirm_frames", lost_cancel_confirm_frames_, 2);
        nh_.param("/lineo_right/drift_recover_confirm_frames", drift_recover_confirm_frames_, 2);
        nh_.param("/lineo_right/lost_confirm_linear_y", lost_confirm_linear_y_, 0.0);
        nh_.param("/lineo_right/lost_confirm_angular_z", lost_confirm_angular_z_, 0.0);
        nh_.getParam("/lineo_right/out_turn_angel", out_turn_angel_);
        nh_.getParam("/lineo_right/center_distance", center_distance);

        // 实线图像处理参数：默认值严格保持V20硬编码行为。
        nh_.param("/lineo_right/adaptive_block", adaptive_block_, 45);
        nh_.param("/lineo_right/adaptive_c", adaptive_c_, -15.0);
        nh_.param("/lineo_right/min_contour_area", min_contour_area_, 250.0);

        // 虚线并不修改原实线二值图，而是从原始红通道单独建立一张
        // 更宽松的二值图；V22正常巡线时会与实线同时评估，并由锁定状态机仲裁。
        nh_.param("/lineo_right/dashed_line_enable", dashed_line_enable_, true);
        nh_.param("/lineo_right/dashed_adaptive_block", dashed_adaptive_block_, 45);
        nh_.param("/lineo_right/dashed_adaptive_c", dashed_adaptive_c_, -12.0);
        nh_.param("/lineo_right/dashed_min_contour_area", dashed_min_contour_area_, 35.0);
        nh_.param("/lineo_right/dashed_max_contour_area", dashed_max_contour_area_, 5000.0);
        nh_.param("/lineo_right/dashed_min_component_count", dashed_min_component_count_, 2);
        nh_.param("/lineo_right/dashed_min_component_count_without_prior",
                  dashed_min_component_count_without_prior_, 3);
        nh_.param("/lineo_right/dashed_min_component_width", dashed_min_component_width_, 4);
        nh_.param("/lineo_right/dashed_min_component_height", dashed_min_component_height_, 3);
        nh_.param("/lineo_right/dashed_max_component_width", dashed_max_component_width_, 160);
        nh_.param("/lineo_right/dashed_max_component_height", dashed_max_component_height_, 100);
        nh_.param("/lineo_right/dashed_min_x", dashed_min_x_, 300);
        nh_.param("/lineo_right/dashed_min_y", dashed_min_y_, 55);
        nh_.param("/lineo_right/dashed_component_line_tolerance_px",
                  dashed_component_line_tolerance_px_, 20.0);
        nh_.param("/lineo_right/dashed_fit_max_error_px", dashed_fit_max_error_px_, 8.0);
        nh_.param("/lineo_right/dashed_min_y_span", dashed_min_y_span_, 35);
        nh_.param("/lineo_right/dashed_virtual_max_points", dashed_virtual_max_points_, 60);
        nh_.param("/lineo_right/dashed_bottom_extrapolation_px",
                  dashed_bottom_extrapolation_px_, 25);
        nh_.param("/lineo_right/dashed_top_extrapolation_px",
                  dashed_top_extrapolation_px_, 8);
        nh_.param("/lineo_right/dashed_use_prior", dashed_use_prior_, true);
        nh_.param("/lineo_right/dashed_prior_x_tolerance_px",
                  dashed_prior_x_tolerance_px_, 80.0);
        nh_.param("/lineo_right/dashed_prior_angle_tolerance_deg",
                  dashed_prior_angle_tolerance_deg_, 35.0);

        // V22：虚线只在“根据原center_distance标定推算出的右边线走廊”里搜索。
        // 这比固定右1/3更稳：近处走廊自动靠右，远处则适当向左放宽。
        nh_.param("/lineo_right/dashed_use_expected_corridor",
                  dashed_use_expected_corridor_, true);
        nh_.param("/lineo_right/dashed_corridor_left_margin_px",
                  dashed_corridor_left_margin_px_, 115.0);
        nh_.param("/lineo_right/dashed_corridor_right_margin_px",
                  dashed_corridor_right_margin_px_, 65.0);
        nh_.param("/lineo_right/dashed_right_bias_weight",
                  dashed_right_bias_weight_, 0.30);
        nh_.param("/lineo_right/dashed_pattern_score_weight",
                  dashed_pattern_score_weight_, 20.0);

        // V24遗留门控参数读取：V25为兼容旧YAML保留读取，但不再参与正常白线判定。
        nh_.param("/lineo_right/continuous_right_min_points",
                  continuous_right_min_points_, 18);
        nh_.param("/lineo_right/continuous_right_corridor_ratio",
                  continuous_right_corridor_ratio_, 0.70);
        nh_.param("/lineo_right/continuous_right_max_prior_jump_px",
                  continuous_right_max_prior_jump_px_, 70.0);
        nh_.param("/lineo_right/continuous_right_max_prior_angle_deg",
                  continuous_right_max_prior_angle_deg_, 30.0);

        // V22：连续识别到可靠虚线后进入锁定模式。锁定后虚线优先，
        // 只有连续可靠实线重新出现或虚线长期消失才退出，防止反光单帧抢线。
        nh_.param("/lineo_right/dashed_lock_confirm_frames",
                  dashed_lock_confirm_frames_, 2);
        nh_.param("/lineo_right/dashed_lock_release_miss_frames",
                  dashed_lock_release_miss_frames_, 6);
        nh_.param("/lineo_right/dashed_solid_relock_confirm_frames",
                  dashed_solid_relock_confirm_frames_, 2);
        nh_.param("/lineo_right/dashed_lock_x_tolerance_px",
                  dashed_lock_x_tolerance_px_, 55.0);
        nh_.param("/lineo_right/dashed_lock_angle_tolerance_deg",
                  dashed_lock_angle_tolerance_deg_, 25.0);

        // V23：锁定虚线短暂消失时先按最后可靠虚线控制量减速保持，
        // 期间完全屏蔽LOST_CONFIRM/DRIFT_RECOVERY。
        nh_.param("/lineo_right/dashed_hold_frames",
                  dashed_hold_frames_, 6);
        nh_.param("/lineo_right/dashed_hold_speed",
                  dashed_hold_speed_, 0.40);
        nh_.param("/lineo_right/dashed_hold_angular_decay",
                  dashed_hold_angular_decay_, 0.70);

        nh_.param("/lineo_right/disable_dashed_after_avoidance",
                  disable_dashed_after_avoidance_, true);

        nh_.param("/lineo_right/dashed_debug_draw", dashed_debug_draw_, true);

        // AMCL初始位姿。每次服务真正启动巡线前都会重新读取并发布。
        nh_.param<string>("/lineo_right/map_frame", map_frame_, "map");
        nh_.param<string>("/lineo_right/base_frame", base_frame_, "base_link");
        nh_.param("/lineo_right/initial_pose_x", initial_pose_x_, 2.50);
        nh_.param("/lineo_right/initial_pose_y", initial_pose_y_, 2.60);
        nh_.param("/lineo_right/initial_pose_yaw_deg", initial_pose_yaw_deg_, -90.0);
        nh_.param("/lineo_right/initial_pose_covariance_xy",
                  initial_pose_covariance_xy_, 0.01);
        nh_.param("/lineo_right/initial_pose_covariance_yaw",
                  initial_pose_covariance_yaw_, 0.01);
        nh_.param("/lineo_right/initial_pose_publish_count",
                  initial_pose_publish_count_, 3);
        nh_.param("/lineo_right/initial_pose_publish_interval",
                  initial_pose_publish_interval_, 0.10);

        // 新版停靠触发：y小于阈值且距离固定终点足够近。
        nh_.param("/lineo_right/docking_trigger_max_y",
                  docking_trigger_max_y_, 0.75);
        nh_.param("/lineo_right/docking_trigger_distance",
                  docking_trigger_distance_, 0.75);
        nh_.param("/lineo_right/docking_goal_x", docking_goal_x_, 0.25);
        nh_.param("/lineo_right/docking_goal_y", docking_goal_y_, 0.25);
        nh_.param("/lineo_right/docking_goal_yaw_deg",
                  docking_goal_yaw_deg_, 180.0);

        // 终点纯PP位姿控制。
        nh_.param("/lineo_right/docking_control_rate",
                  docking_control_rate_, 30.0);
        nh_.param("/lineo_right/docking_position_tolerance",
                  docking_position_tolerance_, 0.025);
        nh_.param("/lineo_right/docking_yaw_tolerance",
                  docking_yaw_tolerance_, 0.05);
        nh_.param("/lineo_right/docking_linear_x_gain",
                  docking_linear_x_gain_, 2.50);
        nh_.param("/lineo_right/docking_linear_y_gain",
                  docking_linear_y_gain_, 1.20);
        nh_.param("/lineo_right/docking_angular_gain",
                  docking_angular_gain_, 1.50);
        nh_.param("/lineo_right/docking_min_linear_speed",
                  docking_min_linear_speed_, 0.10);
        nh_.param("/lineo_right/docking_min_angular_speed",
                  docking_min_angular_speed_, 0.010);
        nh_.param("/lineo_right/docking_max_vel_x",
                  docking_max_vel_x_, 0.90);
        nh_.param("/lineo_right/docking_max_vel_y",
                  docking_max_vel_y_, 0.40);
        nh_.param("/lineo_right/docking_max_vel_theta",
                  docking_max_vel_theta_, 0.90);
        nh_.param("/lineo_right/docking_acc_lim_x",
                  docking_acc_lim_x_, 2.00);
        nh_.param("/lineo_right/docking_acc_lim_y",
                  docking_acc_lim_y_, 2.00);
        nh_.param("/lineo_right/docking_acc_lim_theta",
                  docking_acc_lim_theta_, 8.00);

        // 精简挡板检测：仅使用map_x<2.25且远离场地边界的雷达点。
        nh_.param<string>("/lineo_right/scan_topic", scan_topic_, "/scan");
        nh_.param("/lineo_right/barrier_min_scan_range",
                  barrier_min_scan_range_, 0.05);
        nh_.param("/lineo_right/barrier_max_scan_range",
                  barrier_max_scan_range_, 2.00);
        nh_.param("/lineo_right/field_min_x", field_min_x_, 0.0);
        nh_.param("/lineo_right/field_max_x", field_max_x_, 5.0);
        nh_.param("/lineo_right/field_min_y", field_min_y_, 0.0);
        nh_.param("/lineo_right/field_max_y", field_max_y_, 2.5);
        nh_.param("/lineo_right/barrier_boundary_margin",
                  barrier_boundary_margin_, 0.10);
        nh_.param("/lineo_right/barrier_candidate_max_map_x",
                  barrier_candidate_max_map_x_, 2.25);
        nh_.param("/lineo_right/barrier_neighbor_max_distance",
                  barrier_neighbor_max_distance_, 0.10);
        nh_.param("/lineo_right/barrier_min_points",
                  barrier_min_points_, 6);
        nh_.param("/lineo_right/barrier_min_line_length",
                  barrier_min_line_length_, 0.15);
        nh_.param("/lineo_right/barrier_max_rms_error",
                  barrier_max_rms_error_, 0.03);
        nh_.param("/lineo_right/barrier_endpoint_margin",
                  barrier_endpoint_margin_, 0.05);
        nh_.param("/lineo_right/barrier_trigger_distance",
                  barrier_trigger_distance_, 0.25);
        nh_.param("/lineo_right/barrier_lost_mode_trigger_distance",
                  barrier_lost_mode_trigger_distance_, 0.45);
        nh_.param("/lineo_right/barrier_no_drift_distance",
                  barrier_no_drift_distance_, 0.35);
        nh_.param("/lineo_right/barrier_slowdown_start_distance",
                  barrier_slowdown_start_distance_, 1.00);
        nh_.param("/lineo_right/barrier_slowdown_min_speed_ratio",
                  barrier_slowdown_min_speed_ratio_, 0.20);
        nh_.param("/lineo_right/barrier_debug_log_enabled",
                  barrier_debug_log_enabled_, true);
        nh_.param("/lineo_right/barrier_debug_log_interval",
                  barrier_debug_log_interval_, 0.50);

        // 三段避障：前两段保持固定距离；第三段P控制到停车车头射线与挡板交点后延伸得到的绕障点。
        nh_.param("/lineo_right/avoid_right_distance",
                  avoid_right_distance_, 0.50);
        nh_.param("/lineo_right/avoid_forward_distance",
                  avoid_forward_distance_, 0.50);
        nh_.param("/lineo_right/avoid_target_extension_distance",
                  avoid_target_extension_distance_, 0.30);
        nh_.param("/lineo_right/avoid_use_fixed_target",
                  avoid_use_fixed_target_, false);
        nh_.param("/lineo_right/avoid_fixed_target_x",
                  avoid_fixed_target_x_, 0.0);
        nh_.param("/lineo_right/avoid_fixed_target_y",
                  avoid_fixed_target_y_, 0.0);
        nh_.param("/lineo_right/avoid_fixed_target_yaw_deg",
                  avoid_fixed_target_yaw_deg_, -90.0);
        nh_.param("/lineo_right/avoid_right_speed",
                  avoid_right_speed_, 0.35);
        nh_.param("/lineo_right/avoid_forward_speed",
                  avoid_forward_speed_, 0.45);
        nh_.param("/lineo_right/avoid_left_speed",
                  avoid_left_speed_, 0.35);
        nh_.param("/lineo_right/avoid_control_rate",
                  avoid_control_rate_, 40.0);
        nh_.param("/lineo_right/avoid_position_kp",
                  avoid_position_kp_, 2.0);
        nh_.param("/lineo_right/avoid_yaw_kp",
                  avoid_yaw_kp_, 2.5);
        nh_.param("/lineo_right/avoid_max_angular_speed",
                  avoid_max_angular_speed_, 0.60);
        nh_.param("/lineo_right/avoid_heading_pause_error_deg",
                  avoid_heading_pause_error_deg_, 6.0);
        nh_.param("/lineo_right/avoid_position_tolerance",
                  avoid_position_tolerance_, 0.02);
        nh_.param("/lineo_right/avoid_min_linear_speed",
                  avoid_min_linear_speed_, 0.08);
        nh_.param("/lineo_right/avoid_segment_timeout",
                  avoid_segment_timeout_, 5.0);
        nh_.param("/lineo_right/avoid_stop_hold_time",
                  avoid_stop_hold_time_, 0.10);
        nh_.param("/lineo_right/avoid_segment_pause_time",
                  avoid_segment_pause_time_, 0.05);
        nh_.param("/lineo_right/avoid_acc_lim_x",
                  avoid_acc_lim_x_, 2.0);
        nh_.param("/lineo_right/avoid_acc_lim_y",
                  avoid_acc_lim_y_, 2.0);
        nh_.param("/lineo_right/avoid_acc_lim_theta",
                  avoid_acc_lim_theta_, 6.0);
        nh_.param("/lineo_right/avoid_camera_flush_frames",
                  avoid_camera_flush_frames_, 5);

        // 参数保护，避免运行中误设负数导致异常。
        adaptive_block_ = sanitizeAdaptiveBlock(adaptive_block_);
        min_contour_area_ = std::max(0.0, min_contour_area_);
        dashed_adaptive_block_ = sanitizeAdaptiveBlock(dashed_adaptive_block_);
        dashed_min_contour_area_ = std::max(0.0, dashed_min_contour_area_);
        dashed_max_contour_area_ = std::max(
            dashed_min_contour_area_ + 1.0, dashed_max_contour_area_);
        dashed_min_component_count_ = std::max(2, dashed_min_component_count_);
        dashed_min_component_count_without_prior_ = std::max(
            dashed_min_component_count_, dashed_min_component_count_without_prior_);
        dashed_min_component_width_ = std::max(1, dashed_min_component_width_);
        dashed_min_component_height_ = std::max(1, dashed_min_component_height_);
        dashed_max_component_width_ = std::max(
            dashed_min_component_width_, dashed_max_component_width_);
        dashed_max_component_height_ = std::max(
            dashed_min_component_height_, dashed_max_component_height_);
        dashed_min_x_ = clamp(dashed_min_x_, 0, 638);
        dashed_min_y_ = clamp(dashed_min_y_, 0, 268);
        dashed_component_line_tolerance_px_ = std::max(2.0, dashed_component_line_tolerance_px_);
        dashed_fit_max_error_px_ = std::max(1.0, dashed_fit_max_error_px_);
        dashed_min_y_span_ = std::max(10, dashed_min_y_span_);
        dashed_virtual_max_points_ = std::max(15, dashed_virtual_max_points_);
        dashed_bottom_extrapolation_px_ = std::max(0, dashed_bottom_extrapolation_px_);
        dashed_top_extrapolation_px_ = std::max(0, dashed_top_extrapolation_px_);
        dashed_prior_x_tolerance_px_ = std::max(5.0, dashed_prior_x_tolerance_px_);
        dashed_prior_angle_tolerance_deg_ = clamp(
            dashed_prior_angle_tolerance_deg_, 1.0, 90.0);
        dashed_corridor_left_margin_px_ = std::max(10.0, dashed_corridor_left_margin_px_);
        dashed_corridor_right_margin_px_ = std::max(10.0, dashed_corridor_right_margin_px_);
        dashed_right_bias_weight_ = std::max(0.0, dashed_right_bias_weight_);
        dashed_pattern_score_weight_ = std::max(0.0, dashed_pattern_score_weight_);
        continuous_right_min_points_ = std::max(8, continuous_right_min_points_);
        continuous_right_corridor_ratio_ = clamp(
            continuous_right_corridor_ratio_, 0.10, 1.0);
        continuous_right_max_prior_jump_px_ =
            std::max(5.0, continuous_right_max_prior_jump_px_);
        continuous_right_max_prior_angle_deg_ = clamp(
            continuous_right_max_prior_angle_deg_, 1.0, 90.0);
        dashed_lock_confirm_frames_ = std::max(1, dashed_lock_confirm_frames_);
        dashed_lock_release_miss_frames_ = std::max(2, dashed_lock_release_miss_frames_);
        dashed_solid_relock_confirm_frames_ = std::max(1, dashed_solid_relock_confirm_frames_);
        dashed_lock_x_tolerance_px_ = std::max(5.0, dashed_lock_x_tolerance_px_);
        dashed_lock_angle_tolerance_deg_ = clamp(
            dashed_lock_angle_tolerance_deg_, 1.0, 90.0);
        dashed_hold_frames_ = std::max(1, dashed_hold_frames_);
        dashed_hold_speed_ = std::max(0.0, dashed_hold_speed_);
        dashed_hold_angular_decay_ = clamp(
            dashed_hold_angular_decay_, 0.0, 1.0);

        start_straight_speed_ = std::max(0.0, start_straight_speed_);
        start_straight_distance_ = std::max(0.0, start_straight_distance_);
        start_heading_kp_ = std::max(0.0, start_heading_kp_);
        start_heading_max_angular_speed_ =
            std::max(0.0, start_heading_max_angular_speed_);
        start_heading_deadband_deg_ = std::max(0.0, start_heading_deadband_deg_);
        start_heading_pause_error_deg_ =
            std::max(start_heading_deadband_deg_, start_heading_pause_error_deg_);
        start_control_rate_ = std::max(1.0, start_control_rate_);
        start_line_confirm_frames_ = std::max(1, start_line_confirm_frames_);
        out_forward_ = std::max(0.0, out_forward_);
        out_left_speed_ = std::max(0.0, out_left_speed_);
        lost_confirm_frames_ = std::max(1, lost_confirm_frames_);
        lost_cancel_confirm_frames_ = std::max(1, lost_cancel_confirm_frames_);
        drift_recover_confirm_frames_ = std::max(1, drift_recover_confirm_frames_);
        // 线性减速公式要求out_forward不高于x_max；若运行中误设，自动夹紧以避免“丢线反而加速”。
        out_forward_ = std::min(out_forward_, std::max(0.0, x_max_));
        initial_pose_covariance_xy_ = std::max(0.0, initial_pose_covariance_xy_);
        initial_pose_covariance_yaw_ = std::max(0.0, initial_pose_covariance_yaw_);
        initial_pose_publish_count_ = std::max(1, initial_pose_publish_count_);
        initial_pose_publish_interval_ = std::max(0.0, initial_pose_publish_interval_);
        docking_trigger_distance_ = std::max(0.0, docking_trigger_distance_);
        docking_control_rate_ = std::max(1.0, docking_control_rate_);
        docking_position_tolerance_ = std::max(0.001, docking_position_tolerance_);
        docking_yaw_tolerance_ = std::max(0.001, docking_yaw_tolerance_);
        docking_linear_x_gain_ = std::max(0.0, docking_linear_x_gain_);
        docking_linear_y_gain_ = std::max(0.0, docking_linear_y_gain_);
        docking_angular_gain_ = std::max(0.0, docking_angular_gain_);
        docking_min_linear_speed_ = std::max(0.0, docking_min_linear_speed_);
        docking_min_angular_speed_ = std::max(0.0, docking_min_angular_speed_);
        docking_max_vel_x_ = std::max(0.001, docking_max_vel_x_);
        docking_max_vel_y_ = std::max(0.001, docking_max_vel_y_);
        docking_max_vel_theta_ = std::max(0.001, docking_max_vel_theta_);
        docking_min_linear_speed_ =
            std::min(docking_min_linear_speed_,
                     std::min(docking_max_vel_x_, docking_max_vel_y_));
        docking_min_angular_speed_ =
            std::min(docking_min_angular_speed_, docking_max_vel_theta_);
        docking_acc_lim_x_ = std::max(0.0, docking_acc_lim_x_);
        docking_acc_lim_y_ = std::max(0.0, docking_acc_lim_y_);
        docking_acc_lim_theta_ = std::max(0.0, docking_acc_lim_theta_);

        barrier_min_scan_range_ = std::max(0.0, barrier_min_scan_range_);
        barrier_max_scan_range_ = std::max(
            barrier_min_scan_range_ + 0.01, barrier_max_scan_range_);
        if (field_min_x_ > field_max_x_) {
            std::swap(field_min_x_, field_max_x_);
        }
        if (field_min_y_ > field_max_y_) {
            std::swap(field_min_y_, field_max_y_);
        }
        barrier_candidate_max_map_x_ = clamp(
            barrier_candidate_max_map_x_, field_min_x_, field_max_x_);
        const double half_min_field_size = 0.5 * std::min(
            field_max_x_ - field_min_x_, field_max_y_ - field_min_y_);
        barrier_boundary_margin_ = clamp(
            barrier_boundary_margin_, 0.0,
            std::max(0.0, half_min_field_size - 0.001));
        barrier_neighbor_max_distance_ =
            std::max(0.001, barrier_neighbor_max_distance_);
        barrier_min_points_ = std::max(2, barrier_min_points_);
        barrier_min_line_length_ = std::max(0.0, barrier_min_line_length_);
        barrier_max_rms_error_ = std::max(0.001, barrier_max_rms_error_);
        barrier_endpoint_margin_ = std::max(0.0, barrier_endpoint_margin_);
        barrier_trigger_distance_ = std::max(0.01, barrier_trigger_distance_);
        barrier_lost_mode_trigger_distance_ = std::max(
            barrier_trigger_distance_, barrier_lost_mode_trigger_distance_);
        // 禁止丢线右转区至少覆盖正常挡板触发距离，避免在进入正常触发前又重新允许甩尾。
        barrier_no_drift_distance_ = std::max(
            barrier_trigger_distance_, barrier_no_drift_distance_);
        barrier_slowdown_start_distance_ = std::max(
            barrier_trigger_distance_ + 0.01,
            barrier_slowdown_start_distance_);
        barrier_slowdown_min_speed_ratio_ = clamp(
            barrier_slowdown_min_speed_ratio_, 0.0, 1.0);
        barrier_debug_log_interval_ =
            std::max(0.10, barrier_debug_log_interval_);

        avoid_right_distance_ = std::max(0.0, avoid_right_distance_);
        avoid_forward_distance_ = std::max(0.0, avoid_forward_distance_);
        avoid_target_extension_distance_ =
            std::max(0.0, avoid_target_extension_distance_);
        if (!std::isfinite(avoid_fixed_target_x_)) avoid_fixed_target_x_ = 0.0;
        if (!std::isfinite(avoid_fixed_target_y_)) avoid_fixed_target_y_ = 0.0;
        if (!std::isfinite(avoid_fixed_target_yaw_deg_)) avoid_fixed_target_yaw_deg_ = -90.0;
        avoid_right_speed_ = std::max(0.01, avoid_right_speed_);
        avoid_forward_speed_ = std::max(0.01, avoid_forward_speed_);
        avoid_left_speed_ = std::max(0.01, avoid_left_speed_);
        avoid_control_rate_ = std::max(5.0, avoid_control_rate_);
        avoid_position_kp_ = std::max(0.01, avoid_position_kp_);
        avoid_yaw_kp_ = std::max(0.0, avoid_yaw_kp_);
        avoid_max_angular_speed_ = std::max(0.0, avoid_max_angular_speed_);
        avoid_heading_pause_error_deg_ = clamp(
            avoid_heading_pause_error_deg_, 0.0, 90.0);
        avoid_position_tolerance_ = std::max(0.005, avoid_position_tolerance_);
        avoid_min_linear_speed_ = std::max(0.0, avoid_min_linear_speed_);
        avoid_segment_timeout_ = std::max(0.50, avoid_segment_timeout_);
        avoid_stop_hold_time_ = std::max(0.0, avoid_stop_hold_time_);
        avoid_segment_pause_time_ = std::max(0.0, avoid_segment_pause_time_);
        avoid_acc_lim_x_ = std::max(0.0, avoid_acc_lim_x_);
        avoid_acc_lim_y_ = std::max(0.0, avoid_acc_lim_y_);
        avoid_acc_lim_theta_ = std::max(0.0, avoid_acc_lim_theta_);
        avoid_camera_flush_frames_ = std::max(0, avoid_camera_flush_frames_);

        ROS_INFO("lineo_right参数加载完成：center_distance=%d", center_distance);
        ROS_INFO(
            "实线二值化：block=%d，C=%.1f，min_area=%.1f；"
            "虚线重建=%s，block=%d，C=%.1f，area=[%.1f, %.1f]，"
            "有先验至少%d段/无先验至少%d段，min_x=%d，min_y=%d，"
            "共线阈值=%.1fpx，拟合RMS<=%.1fpx。",
            adaptive_block_, adaptive_c_, min_contour_area_,
            dashed_line_enable_ ? "ON" : "OFF",
            dashed_adaptive_block_, dashed_adaptive_c_,
            dashed_min_contour_area_, dashed_max_contour_area_,
            dashed_min_component_count_, dashed_min_component_count_without_prior_,
            dashed_min_x_, dashed_min_y_,
            dashed_component_line_tolerance_px_, dashed_fit_max_error_px_);
        ROS_INFO(
            "V22虚线防误检：动态右侧走廊=%s，左/右余量=%.0f/%.0fpx，"
            "靠右加权=%.2f，1:1节奏加权=%.1f；连续%d帧锁定，锁定后同线容差=%.0fpx/%.1f°，"
            "兼容参数solid_relock=%d帧/release_miss=%d帧（V24主仲裁不再靠连续线解锁）；"
            "避障成功后禁用虚线=%s。",
            dashed_use_expected_corridor_ ? "ON" : "OFF",
            dashed_corridor_left_margin_px_, dashed_corridor_right_margin_px_,
            dashed_right_bias_weight_, dashed_pattern_score_weight_,
            dashed_lock_confirm_frames_,
            dashed_lock_x_tolerance_px_, dashed_lock_angle_tolerance_deg_,
            dashed_solid_relock_confirm_frames_, dashed_lock_release_miss_frames_,
            disable_dashed_after_avoidance_ ? "YES" : "NO");
        ROS_INFO(
            "V25提示：以下连续线门控参数仅为旧配置兼容、已停用：points>=%d，走廊>=%.0f%%，"
            "有先验时位置跳变<=%.1fpx、角度跳变<=%.1f°；通过后本帧完全跳过虚线重建。",
            continuous_right_min_points_, continuous_right_corridor_ratio_ * 100.0,
            continuous_right_max_prior_jump_px_, continuous_right_max_prior_angle_deg_);
        ROS_INFO(
            "V23虚线丢失互锁：DASHED_LOCK内最多HOLD %d帧，HOLD vx上限=%.3fm/s，"
            "最后可靠wz每帧乘%.2f；HOLD期间禁止LOST_CONFIRM/DRIFT。",
            dashed_hold_frames_, dashed_hold_speed_, dashed_hold_angular_decay_);
        ROS_INFO(
            "启动保护：先屏蔽视觉直行%.3fm，目标朝向=%.1f°，速度=%.3fm/s，"
            "朝向Kp=%.2f，最大wz=%.2f，连续%d帧确认白线后交给PID",
            start_straight_distance_, start_heading_yaw_deg_,
            start_straight_speed_, start_heading_kp_,
            start_heading_max_angular_speed_, start_line_confirm_frames_);
        ROS_INFO(
            "AMCL初始位姿=(%.3f, %.3f, %.1f°)，"
            "停靠触发：y<%.3f且距(%.3f, %.3f)<%.3fm，最终方向=%.1f°",
            initial_pose_x_, initial_pose_y_, initial_pose_yaw_deg_,
            docking_trigger_max_y_,
            docking_goal_x_, docking_goal_y_,
            docking_trigger_distance_,
            docking_goal_yaw_deg_);
        ROS_INFO(
            "丢线状态机：%d次失败确认丢线；疑似丢线连续%d帧有效线才取消；"
            "漂移连续%d帧有效线才恢复PID；疑似阶段vy=%.3f,wz=%.3f",
            lost_confirm_frames_, lost_cancel_confirm_frames_,
            drift_recover_confirm_frames_, lost_confirm_linear_y_,
            lost_confirm_angular_z_);
        ROS_INFO(
            "精简挡板检测：仅map_x<%.3fm且距场地边界>%.3fm，"
            "删除%.3fm内无邻点的孤立点；最少%d点，线长>=%.3fm，"
            "RMS<=%.3fm，触发距离<=%.3fm；正常巡线在%.3fm内"
            "线性减速至x_max的%.0f%%。",
            barrier_candidate_max_map_x_, barrier_boundary_margin_,
            barrier_neighbor_max_distance_, barrier_min_points_,
            barrier_min_line_length_, barrier_max_rms_error_,
            barrier_trigger_distance_, barrier_slowdown_start_distance_,
            barrier_slowdown_min_speed_ratio_ * 100.0);
        ROS_INFO(
            "挡板安全互锁：正常触发<=%.3fm；疑似/确认丢线时提前触发<=%.3fm；"
            "避障前板距<=%.3fm后锁存禁止丢线右转/甩尾，避障完成后自动释放。"
            "三段避障：右移%.3fm、前进%.3fm、最后P控制到绕障点；"
            "动态目标延伸=%.3fm；固定目标模式=%s，固定目标=(%.3f, %.3f, %.1f°)。",
            barrier_trigger_distance_, barrier_lost_mode_trigger_distance_,
            barrier_no_drift_distance_,
            avoid_right_distance_, avoid_forward_distance_,
            avoid_target_extension_distance_,
            avoid_use_fixed_target_ ? "ON" : "OFF",
            avoid_fixed_target_x_, avoid_fixed_target_y_, avoid_fixed_target_yaw_deg_);
        ROS_INFO(
            "虚线运行时门控：服务启动后按dashed_line_enable=%s立即可用；"
            "避障成功后按disable_dashed_after_avoidance=%s决定是否永久关闭。",
            dashed_line_enable_ ? "true" : "false",
            disable_dashed_after_avoidance_ ? "true" : "false");
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

        // 初始化TF监听器
        tf_listener_ = new tf::TransformListener();
        ROS_INFO("TF变换监听器已初始化");

        scan_sub_ = nh_.subscribe(
            scan_topic_, 1, &LineFollowerNode::scanCallback, this);
        ROS_INFO("精简挡板检测已订阅：%s", scan_topic_.c_str());
    }

    // 加载相机标定文件
    bool loadCalibrationFile() {
    std::string calibration_file;

    nh_.param<std::string>(
        "/lineo_right/calibration_file",
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

        // 每次开始巡线前重新读取rosparam，允许运行中直接修改启动保护、初始位姿和停靠参数。
        loadParameters();

        // 清除上一次服务调用遗留的巡线状态。
        double_line_ = false;
        left_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        lost_cancel_success_count_ = 0;
        drift_recover_success_count_ = 0;
        drift_recovery_active_ = false;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();
        resetLineVisualPrior();
        resetDashedRuntimeForService();
        resetObstacleState();

        // 启动阶段拆成两层：
        // A. protected_straight_mode：前start_straight_distance米完全不看视觉，AMCL定向直行；
        // B. initial_straight_mode：走满保护距离后才看视觉，但仍保持-90°直行，
        //    连续start_line_confirm_frames帧识别有效右线后才交给正常PID。
        bool protected_straight_mode = (start_straight_distance_ > 1e-4);
        bool initial_straight_mode = true;
        bool protected_start_pose_captured = false;
        double protected_start_x = 0.0;
        double protected_start_y = 0.0;
        int initial_line_confirm_count = 0;

        // 小车尚未开始运动时，先强制设置本次巡线使用的AMCL初始位姿。
        publishInitialPose();

        if (!initCameraAndVideo()) {
            ROS_FATAL("相机或视频初始化失败，节点无法启动");
            stopRobot();
            return false;
        }

        line_service_active_.store(true);

        ros::Rate start_control_rate(start_control_rate_);

        ROS_INFO(
            "启动第一阶段：前%.3fm完全屏蔽视觉；目标朝向=%.1f°，速度=%.3fm/s。"
            "朝向偏差>%.1f°时暂停平移并先纠正车头。",
            start_straight_distance_, start_heading_yaw_deg_, start_straight_speed_,
            start_heading_pause_error_deg_);

        while (ros::ok()) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            const bool pose_ok = getRobotPose(robot_x, robot_y, robot_yaw);

            // 雷达线程或丢线安全逻辑锁存挡板后，在视觉巡线和终点停靠
            // 判断之前立即接管，执行右移、前进、左移三段闭环平移。
            if (obstacle_triggered_.load() &&
                !avoidance_completed_.load()) {
                stopRobot();
                if (!runOneTimeAvoidance()) {
                    avoidance_failed = true;
                    ROS_ERROR("精简挡板避障失败，终止本次巡线并安全停车。");
                    break;
                }

                // 避障完成后直接回到原正常巡线，不重新执行启动保护/找线阶段。
                protected_straight_mode = false;
                initial_straight_mode = false;
                initial_line_confirm_count = 0;
                trace_failed_count_ = 0;
                lost_cancel_success_count_ = 0;
                drift_recover_success_count_ = 0;
                drift_recovery_active_ = false;
                integration_ = 0.0;
                pre_error_ = 0.0;
                // 三段平移后相机视角已经发生明显变化，旧视觉锁和先验全部作废。
                // 默认配置下runOneTimeAvoidance会在避障成功后关闭本次服务的虚线通道；
                // 若YAML关闭该机制，则只清空旧虚线锁/先验并重新识别。
                resetDashedLockState();
                resetLineVisualPrior();
                continue;
            }

            // 新版停靠触发：y小于阈值，同时距固定终点小于阈值。
            // 满足条件后直接切纯PP，不在巡线与停靠之间插入零速度。
            if (pose_ok) {
                const double distance_to_goal =
                    std::hypot(docking_goal_x_ - robot_x,
                               docking_goal_y_ - robot_y);

                if (robot_y < docking_trigger_max_y_ &&
                    distance_to_goal < docking_trigger_distance_) {
                    switch_to_docking = true;
                    ROS_INFO(
                        "满足停靠触发条件：当前位置=(%.3f, %.3f)，y<%.3f，"
                        "距终点=%.3fm<%.3fm；无停顿切入纯PP停靠。",
                        robot_x, robot_y,
                        docking_trigger_max_y_,
                        distance_to_goal,
                        docking_trigger_distance_);
                    break;
                }
            }

            // -----------------------------------------------------------------
            // 启动阶段A：保护直行。
            // 在走满设定距离以前完全不读取/不使用视觉结果，因此地面反光不可能抢走控制权。
            // -----------------------------------------------------------------
            if (protected_straight_mode) {
                if (!pose_ok) {
                    // 这一阶段的核心约束就是保持-90°，没有AMCL朝向就不允许盲目前进。
                    twist_ = geometry_msgs::Twist();
                    publishTrackingCommandSafely();
                    ROS_WARN_THROTTLE(1.0,
                        "启动保护直行阶段暂时读不到AMCL，保持停车，等待定位恢复。"
                    );
                    start_control_rate.sleep();
                    continue;
                }

                if (!protected_start_pose_captured) {
                    // /initialpose刚发布后的极短时间内，TF仍可能还是上一次任务结束时的旧位姿。
                    // 必须等AMCL已经回到本次设定起点附近再锁定“20cm计程起点”，否则一次TF跳变
                    // 就可能被误算成已经行驶了很远或负距离。
                    const double pi = 3.14159265358979323846;
                    const double target_yaw = start_heading_yaw_deg_ * pi / 180.0;
                    const double pose_jump_distance =
                        std::hypot(robot_x - initial_pose_x_, robot_y - initial_pose_y_);
                    const double pose_yaw_error =
                        std::abs(normalizeAngle(target_yaw - robot_yaw));

                    if (pose_jump_distance > 0.35 || pose_yaw_error > 30.0 * pi / 180.0) {
                        twist_ = geometry_msgs::Twist();
                        publishTrackingCommandSafely();
                        // 仍然抓取并丢弃相机帧，防止后续开放视觉时读到启动前缓存的旧画面。
                        cap_.grab();
                        ROS_WARN_THROTTLE(
                            0.5,
                            "等待AMCL初始位姿生效：当前=(%.3f, %.3f, %.1f°)，"
                            "目标=(%.3f, %.3f, %.1f°)",
                            robot_x, robot_y, robot_yaw * 180.0 / pi,
                            initial_pose_x_, initial_pose_y_, start_heading_yaw_deg_);
                        start_control_rate.sleep();
                        continue;
                    }

                    protected_start_x = robot_x;
                    protected_start_y = robot_y;
                    protected_start_pose_captured = true;
                    ROS_INFO("启动保护直行起点锁定：(%.3f, %.3f)，当前yaw=%.1f°",
                             protected_start_x, protected_start_y,
                             robot_yaw * 180.0 / pi);
                }

                const double progress = computeStartForwardProgress(
                    protected_start_x, protected_start_y, robot_x, robot_y);

                if (progress < start_straight_distance_) {
                    computeStartStraightCommand(robot_yaw, twist_);
                    publishTrackingCommandSafely();

                    // “屏蔽视觉”不是停止读取相机：持续grab并丢弃帧，避免摄像头缓存积压。
                    // 否则20cm结束后的第一帧可能仍是起点处带反光的旧画面。
                    cap_.grab();

                    const double target_yaw =
                        start_heading_yaw_deg_ * 3.14159265358979323846 / 180.0;
                    const double yaw_error_deg =
                        normalizeAngle(target_yaw - robot_yaw) *
                        180.0 / 3.14159265358979323846;
                    ROS_INFO_THROTTLE(
                        0.5,
                        "启动保护直行：进度=%.3f/%.3fm，yaw=%.2f°，误差=%.2f°，"
                        "cmd=(vx=%.3f, vy=%.3f, wz=%.3f)",
                        progress, start_straight_distance_,
                        robot_yaw * 180.0 / 3.14159265358979323846,
                        yaw_error_deg,
                        twist_.linear.x, twist_.linear.y, twist_.angular.z);

                    start_control_rate.sleep();
                    continue;
                }

                protected_straight_mode = false;
                initial_line_confirm_count = 0;
                ROS_INFO(
                    "启动保护距离完成：前向投影已行驶%.3fm。现在才开放白线识别；"
                    "在正式接管前仍保持%.1f°定向直行，并要求连续%d帧有效右线。",
                    progress, start_heading_yaw_deg_, start_line_confirm_frames_);
                // 不停车，直接在本循环继续读图，平滑进入第二阶段。
            }

            // -----------------------------------------------------------------
            // 启动阶段B以及之后：从这里开始才真正读取图像。
            // -----------------------------------------------------------------
            cap_.read(image);
            if (image.empty()) {
                if (initial_straight_mode && pose_ok) {
                    computeStartStraightCommand(robot_yaw, twist_);
                    publishTrackingCommandSafely();
                }
                continue;
            }
            cropped = image(roi_);
            flip(cropped, cropped, 1);
            vector<Mat> channels;
            split(cropped, channels);

            // 同一帧同时生成两张互不影响的二值图：
            // 1) gray_img：完全保留原省赛实线通道的强去噪；
            // 2) dashed_gray：只给虚线重建器使用，允许保留1cm白色小块。
            Mat raw_red = channels[2].clone();
            gray_img = raw_red.clone();
            Mat dashed_gray;
            if (dashedRuntimeEnabled()) {
                buildDashedBinary(raw_red, dashed_gray);
            }

            int brightness_threshold = brightness_threshold_calculator(gray_img, cropped);
            threshold(gray_img, brightness_threshold_image, 180, 255, THRESH_BINARY);
            threshold_image(gray_img);
            cv::cvtColor(gray_img, cropped, cv::COLOR_GRAY2BGR);

            if (initial_straight_mode) {
                if (!pose_ok) {
                    // 还没进入正式巡线，因此此时不能失去-90°约束。
                    twist_ = geometry_msgs::Twist();
                    publishTrackingCommandSafely();
                    initial_line_confirm_count = 0;
                    ROS_WARN_THROTTLE(1.0,
                        "初始找线阶段暂时读不到AMCL，为避免朝向漂移，保持停车。"
                    );
                    continue;
                }

                // 可以开始看白线，但不允许单帧检测结果立刻接管车辆。
                const bool found_right_line = runNormalTracking(gray_img, dashed_gray, cropped, false);

                if (found_right_line) {
                    ++initial_line_confirm_count;
                    if (initial_line_confirm_count >= start_line_confirm_frames_) {
                        initial_straight_mode = false;
                        trace_failed_count_ = 0;
                        ROS_INFO(
                            "右侧白线连续%d帧识别有效，确认不是孤立反光；"
                            "退出启动定向直行，正式进入右线PID巡线。",
                            start_line_confirm_frames_);
                        // 当前帧保留runNormalTracking已经算出的PID速度，直接无缝接管。
                    } else {
                        // 还没达到连续确认帧数：忽略本帧PID输出，继续保持-90°直行。
                        // 同时清掉候选白线对PID积分/微分历史的影响，避免反光虽然没接管车辆，
                        // 却提前污染正式巡线时的控制器状态。
                        integration_ = 0.0;
                        pre_error_ = 0.0;
                        computeStartStraightCommand(robot_yaw, twist_);
                        ROS_INFO_THROTTLE(
                            0.5,
                            "检测到候选右线，确认中：%d/%d帧；仍保持%.1f°直行。",
                            initial_line_confirm_count, start_line_confirm_frames_,
                            start_heading_yaw_deg_);
                    }
                } else {
                    initial_line_confirm_count = 0;
                    integration_ = 0.0;
                    pre_error_ = 0.0;
                    // runNormalTracking(false)内部虽然也会给直行速度，但这里统一覆盖为
                    // AMCL闭环定向速度，保证整个“正式巡线前”的朝向都锁在-90°附近。
                    computeStartStraightCommand(robot_yaw, twist_);
                }

                publishTrackingCommandSafely();
                continue;
            }

            // 正常巡线阶段：右线连续丢失超过容错帧数后，执行已经验证可行的全向甩尾找线。
            const bool normal_tracking_valid =
                runNormalTracking(gray_img, dashed_gray, cropped, true);
            // 雷达可能在runNormalTracking执行过程中并发触发；丢线首帧也
            // 可能在函数内部主动提前触发。此处再次硬检查，触发后当前帧
            // 只能发布零速，下一循环立即进入三段避障。
            if (obstacle_triggered_.load() &&
                !avoidance_completed_.load()) {
                twist_ = geometry_msgs::Twist();
                publishTrackingCommandSafely();
                continue;
            }
            if (normal_tracking_valid) {
                // 函数内部还会检查trace_failed_count_、drift_recovery_active_
                // 和avoidance_completed_，确保疑似丢线/甩尾/软退出以及
                // 避障完成后的正常巡线均不受挡板距离减速影响。
                applyBarrierNormalTrackingSpeedLimit();
            }
            publishTrackingCommandSafely();

            // 注意：旧版 stop_car() 白线停车不再参与主流程，避免在AMCL停靠前提前停车。
        }

        line_service_active_.store(false);

        if (ros::ok() && switch_to_docking && !avoidance_failed) {
            runDockingControl();
        } else {
            stopRobot();
        }

        // 为下一次服务调用复位状态。
        double_line_ = false;
        left_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        lost_cancel_success_count_ = 0;
        drift_recover_success_count_ = 0;
        drift_recovery_active_ = false;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();
        resetLineVisualPrior();
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

    bool publishTrackingCommandSafely() {
        // 与雷达触发急停共用同一把锁，使“检查触发状态 + 发布速度”
        // 成为一个不可被急停插入的整体操作：
        // 1. 巡线先拿锁：旧速度先发，雷达随后必定以零速覆盖；
        // 2. 雷达先拿锁：先发零速，巡线随后看到触发标志，只能继续发零速。
        std::lock_guard<std::mutex> lock(tracking_cmd_publish_mutex_);
        const bool stop_latched =
            obstacle_triggered_.load() &&
            !avoidance_completed_.load();
        if (stop_latched) {
            geometry_msgs::Twist emergency_stop;
            cmd_pub_.publish(emergency_stop);
            ROS_WARN_THROTTLE(
                0.5,
                "挡板急停已锁存：丢弃当前启动/巡线帧的旧速度指令。"
                "等待主循环进入一次性避障。");
            return false;
        }

        cmd_pub_.publish(twist_);
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

    bool fitBarrierCluster(
        const vector<std::pair<double, double>>& points,
        double robot_x,
        double robot_y,
        double robot_yaw,
        const ros::Time& stamp,
        BarrierLineFit& fit) const {
        fit = BarrierLineFit();
        fit.stamp = stamp;
        fit.point_count = static_cast<int>(points.size());
        if (fit.point_count < barrier_min_points_) {
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

        // 二维总最小二乘：最大方差方向即挡板方向。
        const double direction_angle = 0.5 * std::atan2(
            2.0 * covariance_xy, covariance_xx - covariance_yy);
        double direction_x = std::cos(direction_angle);
        double direction_y = std::sin(direction_angle);
        // 固定直线方向符号，便于日志稳定；法向符号稍后按车头决定。
        if (direction_x < 0.0 ||
            (std::abs(direction_x) < 1e-9 && direction_y < 0.0)) {
            direction_x = -direction_x;
            direction_y = -direction_y;
        }

        double min_projection = std::numeric_limits<double>::infinity();
        double max_projection = -std::numeric_limits<double>::infinity();
        double squared_residual_sum = 0.0;
        for (const auto& point : points) {
            const double dx = point.first - center_x;
            const double dy = point.second - center_y;
            const double along = direction_x * dx + direction_y * dy;
            const double residual = -direction_y * dx + direction_x * dy;
            min_projection = std::min(min_projection, along);
            max_projection = std::max(max_projection, along);
            squared_residual_sum += residual * residual;
        }

        const double line_length = max_projection - min_projection;
        const double rms_error = std::sqrt(
            squared_residual_sum / static_cast<double>(points.size()));
        const double midpoint_projection =
            0.5 * (min_projection + max_projection);
        const double segment_center_x =
            center_x + direction_x * midpoint_projection;
        const double segment_center_y =
            center_y + direction_y * midpoint_projection;

        double normal_x = -direction_y;
        double normal_y = direction_x;
        const double forward_x = std::cos(robot_yaw);
        const double forward_y = std::sin(robot_yaw);
        // 两个板法向中只选择与触发时车头同向的那个，后续位置和yaw都使用它。
        if (normal_x * forward_x + normal_y * forward_y < 0.0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }

        const double robot_to_center_x = segment_center_x - robot_x;
        const double robot_to_center_y = segment_center_y - robot_y;
        const double forward_projection =
            robot_to_center_x * forward_x + robot_to_center_y * forward_y;
        const double line_distance = std::abs(
            robot_to_center_x * normal_x + robot_to_center_y * normal_y);
        const double foot_projection =
            direction_x * (robot_x - center_x) +
            direction_y * (robot_y - center_y);
        const bool foot_on_segment =
            foot_projection >= min_projection - barrier_endpoint_margin_ &&
            foot_projection <= max_projection + barrier_endpoint_margin_;

        fit.center_map_x = segment_center_x;
        fit.center_map_y = segment_center_y;
        fit.direction_map_x = direction_x;
        fit.direction_map_y = direction_y;
        fit.normal_map_x = normal_x;
        fit.normal_map_y = normal_y;
        fit.distance = line_distance;
        fit.length = line_length;
        fit.rms_error = rms_error;
        fit.valid =
            forward_projection > 0.0 &&
            foot_on_segment &&
            line_length >= barrier_min_line_length_ &&
            rms_error <= barrier_max_rms_error_;
        return fit.valid;
    }

    double calculateBarrierTrackingSpeedLimit(
        const BarrierLineFit& fit) {
        if (!fit.valid ||
            fit.distance >= barrier_slowdown_start_distance_ ||
            avoidance_completed_.load()) {
            return std::numeric_limits<double>::infinity();
        }

        const double slowdown_span =
            barrier_slowdown_start_distance_ - barrier_trigger_distance_;
        const double distance_progress = clamp(
            (fit.distance - barrier_trigger_distance_) / slowdown_span,
            0.0, 1.0);
        const double speed_ratio =
            barrier_slowdown_min_speed_ratio_ +
            (1.0 - barrier_slowdown_min_speed_ratio_) * distance_progress;
        return std::abs(x_max_) * speed_ratio;
    }

    void applyBarrierNormalTrackingSpeedLimit() {
        // 双重保护：即使调用位置今后调整，疑似丢线、漂移恢复和
        // 已完成避障的状态也绝不接受挡板距离减速。
        if (trace_failed_count_ > 0 ||
            drift_recovery_active_ ||
            avoidance_completed_.load()) {
            return;
        }

        const double speed_limit = barrier_tracking_speed_limit_.load();
        if (!std::isfinite(speed_limit)) {
            return;
        }

        // 只压低正常PID巡线产生的前后向线速度，不改变vy和wz。
        twist_.linear.x = clamp(
            twist_.linear.x, -speed_limit, speed_limit);
        ROS_INFO_THROTTLE(
            0.5,
            "挡板距离减速仅作用于正常巡线：当前vx上限=%.3fm/s，"
            "PID输出截断后vx=%.3fm/s。",
            speed_limit, twist_.linear.x);
    }

    bool latchObstacleAndPublishStop(
        const BarrierLineFit& fit,
        double trigger_distance,
        const char* trigger_reason) {
        // 一次性避障成功后，本次服务永久关闭挡板触发及其丢线禁止。
        // 后续再次丢线时必须完整恢复原丢线计数、甩尾和软退出状态机。
        if (avoidance_completed_.load()) {
            return false;
        }
        if (!fit.valid || fit.distance > trigger_distance) {
            return false;
        }

        bool trigger_now = false;
        {
            std::lock_guard<std::mutex> lock(barrier_mutex_);
            if (!obstacle_triggered_.load()) {
                locked_barrier_fit_ = fit;
                trigger_now = !obstacle_triggered_.exchange(true);
            }
        }

        if (trigger_now) {
            geometry_msgs::Twist emergency_stop;
            {
                // 与全部启动/巡线速度共用同一把发布锁。触发标志先锁存，
                // 因此本零速之后不可能再发出丢线、漂移或PID非零指令。
                std::lock_guard<std::mutex> lock(
                    tracking_cmd_publish_mutex_);
                cmd_pub_.publish(emergency_stop);
            }
            ROS_WARN(
                "挡板触发[%s]：距离=%.3fm<=%.3fm，中点=(%.3f, %.3f)，"
                "板方向=(%.3f, %.3f)，前向法向=(%.3f, %.3f)；"
                "硬锁存零速，禁止丢线/漂移继续发布。",
                trigger_reason, fit.distance, trigger_distance,
                fit.center_map_x, fit.center_map_y,
                fit.direction_map_x, fit.direction_map_y,
                fit.normal_map_x, fit.normal_map_y);
        }
        return obstacle_triggered_.load();
    }

    bool latchLatestBarrierForLineLoss() {
        if (avoidance_completed_.load()) {
            return false;
        }

        BarrierLineFit latest_fit;
        {
            std::lock_guard<std::mutex> lock(barrier_mutex_);
            latest_fit = latest_barrier_fit_;
        }
        return latchObstacleAndPublishStop(
            latest_fit,
            barrier_lost_mode_trigger_distance_,
            "丢线安全提前触发");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        if (!line_service_active_.load() ||
            obstacle_triggered_.load() ||
            avoidance_active_.load() ||
            avoidance_completed_.load()) {
            return;
        }
        if (tf_listener_ == nullptr || scan->header.frame_id.empty()) {
            return;
        }

        tf::StampedTransform map_from_scan;
        tf::StampedTransform map_from_base;
        try {
            tf_listener_->lookupTransform(
                map_frame_, scan->header.frame_id, ros::Time(0), map_from_scan);
            tf_listener_->lookupTransform(
                map_frame_, base_frame_, ros::Time(0), map_from_base);
        } catch (const tf::TransformException& ex) {
            ROS_WARN_THROTTLE(
                barrier_debug_log_interval_,
                "挡板检测等待TF：%s", ex.what());
            return;
        }

        const double robot_x = map_from_base.getOrigin().x();
        const double robot_y = map_from_base.getOrigin().y();
        const double robot_yaw = tf::getYaw(map_from_base.getRotation());

        vector<std::pair<double, double>> candidates;
        size_t valid_range_count = 0;
        size_t x_region_count = 0;
        for (size_t index = 0; index < scan->ranges.size(); ++index) {
            const double range = scan->ranges[index];
            const bool valid_range =
                std::isfinite(range) &&
                range >= std::max(
                    static_cast<double>(scan->range_min),
                    barrier_min_scan_range_) &&
                range <= std::min(
                    static_cast<double>(scan->range_max),
                    barrier_max_scan_range_);
            if (!valid_range) {
                continue;
            }
            ++valid_range_count;

            const double angle = scan->angle_min +
                static_cast<double>(index) * scan->angle_increment;
            const tf::Vector3 point_map = map_from_scan * tf::Vector3(
                range * std::cos(angle), range * std::sin(angle), 0.0);
            const double map_x = point_map.x();
            const double map_y = point_map.y();

            // 严格小于2.25；等于或大于阈值的点不参与任何后续计算。
            if (!(map_x < barrier_candidate_max_map_x_)) {
                continue;
            }
            ++x_region_count;

            const bool non_boundary =
                map_x > field_min_x_ + barrier_boundary_margin_ &&
                map_x < field_max_x_ - barrier_boundary_margin_ &&
                map_y > field_min_y_ + barrier_boundary_margin_ &&
                map_y < field_max_y_ - barrier_boundary_margin_;
            if (non_boundary) {
                candidates.emplace_back(map_x, map_y);
            }
        }

        // 删除在0.10m内没有任何邻点的孤立量测；随后使用同一距离阈值
        // 建立连通点簇，防止把相隔较远的物体强行拟合成同一块板。
        const size_t count = candidates.size();
        vector<bool> has_neighbor(count, false);
        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                const double distance = std::hypot(
                    candidates[i].first - candidates[j].first,
                    candidates[i].second - candidates[j].second);
                if (distance <= barrier_neighbor_max_distance_) {
                    has_neighbor[i] = true;
                    has_neighbor[j] = true;
                }
            }
        }

        vector<bool> visited(count, false);
        BarrierLineFit best_fit;
        int cluster_count = 0;
        size_t retained_point_count = 0;
        for (size_t seed = 0; seed < count; ++seed) {
            if (visited[seed] || !has_neighbor[seed]) {
                continue;
            }
            ++cluster_count;
            vector<size_t> open_set(1, seed);
            vector<std::pair<double, double>> cluster;
            visited[seed] = true;
            for (size_t open_index = 0;
                 open_index < open_set.size(); ++open_index) {
                const size_t current = open_set[open_index];
                cluster.push_back(candidates[current]);
                ++retained_point_count;
                for (size_t other = 0; other < count; ++other) {
                    if (visited[other] || !has_neighbor[other]) {
                        continue;
                    }
                    const double distance = std::hypot(
                        candidates[current].first - candidates[other].first,
                        candidates[current].second - candidates[other].second);
                    if (distance <= barrier_neighbor_max_distance_) {
                        visited[other] = true;
                        open_set.push_back(other);
                    }
                }
            }

            BarrierLineFit fit;
            if (fitBarrierCluster(
                    cluster, robot_x, robot_y, robot_yaw,
                    scan->header.stamp, fit) &&
                (!best_fit.valid || fit.distance < best_fit.distance)) {
                best_fit = fit;
            }
        }

        // 保存最近一次雷达拟合。视觉线程一旦出现第一帧丢线，可直接使用
        // 该结果提前锁存避障，无需等到四帧漂移或下一次雷达回调。
        {
            std::lock_guard<std::mutex> lock(barrier_mutex_);
            latest_barrier_fit_ = best_fit;
        }

        // V30：一旦避障前检测到合格挡板已经进入设定近区，就锁存“禁止丢线右转”。
        // 这里采用锁存而不是逐帧开关，避免雷达拟合在挡板近处偶发一帧抖动时，
        // 视觉线程突然重新进入DRIFT_RECOVERY。该锁只在一次避障完成后释放。
        if (best_fit.valid &&
            best_fit.distance <= barrier_no_drift_distance_ &&
            !avoidance_completed_.load()) {
            bool expected = false;
            if (barrier_no_drift_active_.compare_exchange_strong(expected, true)) {
                ROS_WARN(
                    "进入挡板近区丢线右转抑制：板距=%.3fm<=%.3fm；"
                    "从现在到避障完成前，普通丢线最多减速直行，不允许进入右转甩尾。",
                    best_fit.distance, barrier_no_drift_distance_);
            }
        }

        const double approach_speed_limit =
            calculateBarrierTrackingSpeedLimit(best_fit);
        barrier_tracking_speed_limit_.store(approach_speed_limit);

        if (barrier_debug_log_enabled_) {
            if (best_fit.valid) {
                ROS_INFO_THROTTLE(
                    barrier_debug_log_interval_,
                    "精简挡板拟合：有效量测=%zu，map_x<%.3f后=%zu，"
                    "非边界候选=%zu，0.10m邻域保留=%zu，点簇=%d；"
                    "最近板中点=(%.3f, %.3f)，距离=%.3fm，点数=%d，"
                    "线长=%.3fm，RMS=%.4fm，前向法向=(%.3f, %.3f)。",
                    valid_range_count, barrier_candidate_max_map_x_,
                    x_region_count, candidates.size(), retained_point_count,
                    cluster_count, best_fit.center_map_x,
                    best_fit.center_map_y, best_fit.distance,
                    best_fit.point_count, best_fit.length,
                    best_fit.rms_error, best_fit.normal_map_x,
                    best_fit.normal_map_y);
                if (std::isfinite(approach_speed_limit)) {
                    ROS_INFO_THROTTLE(
                        barrier_debug_log_interval_,
                        "挡板接近减速：距离=%.3fm，减速起点=%.3fm，"
                        "触发距离=%.3fm，正常巡线vx上限=%.3fm/s；"
                        "丢线状态和避障完成后不使用该上限。",
                        best_fit.distance,
                        barrier_slowdown_start_distance_,
                        barrier_trigger_distance_, approach_speed_limit);
                }
            } else {
                ROS_INFO_THROTTLE(
                    barrier_debug_log_interval_,
                    "精简挡板拟合：有效量测=%zu，map_x<%.3f后=%zu，"
                    "非边界候选=%zu，0.10m邻域保留=%zu，点簇=%d，"
                    "暂无合格挡板。",
                    valid_range_count, barrier_candidate_max_map_x_,
                    x_region_count, candidates.size(), retained_point_count,
                    cluster_count);
            }
        }

        latchObstacleAndPublishStop(
            best_fit, barrier_trigger_distance_, "正常距离触发");
    }

    void resetObstacleState() {
        std::lock_guard<std::mutex> lock(barrier_mutex_);
        latest_barrier_fit_ = BarrierLineFit();
        locked_barrier_fit_ = BarrierLineFit();
        obstacle_triggered_.store(false);
        avoidance_active_.store(false);
        avoidance_completed_.store(false);
        barrier_no_drift_active_.store(false);
        barrier_tracking_speed_limit_.store(
            std::numeric_limits<double>::infinity());
    }

    // 与前面避障版保持一致：避障段的速度不能从停车零速瞬间跳到目标值，
    // 否则全向底盘在侧移起步时可能产生明显的前向耦合冲击。
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
        double locked_yaw) {
        if (distance <= avoid_position_tolerance_) {
            ROS_INFO("%s距离为0，跳过该段。", segment_name.c_str());
            return true;
        }

        double start_x = 0.0;
        double start_y = 0.0;
        double start_yaw = 0.0;
        if (!getRobotPose(start_x, start_y, start_yaw)) {
            ROS_ERROR("%s开始前无法读取AMCL定位。", segment_name.c_str());
            stopRobot();
            return false;
        }

        // 三段方向都固定在触发时的车体坐标系：前方+x、左侧+y、右侧-y。
        const double cos_locked = std::cos(locked_yaw);
        const double sin_locked = std::sin(locked_yaw);
        const double direction_map_x =
            cos_locked * direction_body_x - sin_locked * direction_body_y;
        const double direction_map_y =
            sin_locked * direction_body_x + cos_locked * direction_body_y;
        const double target_x = start_x + direction_map_x * distance;
        const double target_y = start_y + direction_map_y * distance;

        ros::WallRate control_rate(avoid_control_rate_);
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(avoid_segment_timeout_);
        ros::WallTime last_time = ros::WallTime::now();
        ROS_WARN(
            "%s：起点=(%.3f, %.3f)，目标=(%.3f, %.3f)，"
            "距离=%.3fm，限速=%.3fm/s，保持yaw=%.2f°。",
            segment_name.c_str(), start_x, start_y, target_x, target_y,
            distance, max_speed, locked_yaw * 180.0 /
                3.14159265358979323846);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
                ROS_ERROR("%s过程中定位丢失。", segment_name.c_str());
                stopRobot();
                return false;
            }

            const double error_x = target_x - robot_x;
            const double error_y = target_y - robot_y;
            const double position_error = std::hypot(error_x, error_y);
            const double yaw_error = normalizeAngle(locked_yaw - robot_yaw);
            if (position_error <= avoid_position_tolerance_) {
                stopRobot();
                ROS_WARN("%s完成，位置误差=%.3fm。",
                         segment_name.c_str(), position_error);
                return true;
            }

            double velocity_map_x = avoid_position_kp_ * error_x;
            double velocity_map_y = avoid_position_kp_ * error_y;
            double speed = std::hypot(velocity_map_x, velocity_map_y);
            if (speed > max_speed) {
                const double scale = max_speed / speed;
                velocity_map_x *= scale;
                velocity_map_y *= scale;
            } else if (speed > 1e-9 && speed < avoid_min_linear_speed_) {
                const double target_speed = std::min(
                    avoid_min_linear_speed_, max_speed);
                const double scale = target_speed / speed;
                velocity_map_x *= scale;
                velocity_map_y *= scale;
            }

            geometry_msgs::Twist desired_cmd;
            const double c = std::cos(robot_yaw);
            const double s = std::sin(robot_yaw);
            desired_cmd.linear.x =
                c * velocity_map_x + s * velocity_map_y;
            desired_cmd.linear.y =
                -s * velocity_map_x + c * velocity_map_y;
            desired_cmd.angular.z = clamp(
                avoid_yaw_kp_ * yaw_error,
                -avoid_max_angular_speed_, avoid_max_angular_speed_);
            if (std::abs(yaw_error) >
                avoid_heading_pause_error_deg_ *
                    3.14159265358979323846 / 180.0) {
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
                "%s：剩余=%.3fm，yaw误差=%.2f°，"
                "期望cmd=(%.3f, %.3f, %.3f)，斜坡cmd=(%.3f, %.3f, %.3f)",
                segment_name.c_str(), position_error,
                yaw_error * 180.0 / 3.14159265358979323846,
                desired_cmd.linear.x, desired_cmd.linear.y,
                desired_cmd.angular.z,
                twist_.linear.x, twist_.linear.y, twist_.angular.z);
            control_rate.sleep();
        }

        ROS_ERROR("%s超过%.2fs仍未完成。",
                  segment_name.c_str(), avoid_segment_timeout_);
        stopRobot();
        return false;
    }

    // 根据“真正停车后的车位姿/车头朝向 + 触发时锁存的挡板直线”计算绕障点：
    // 1) 以停车位置R为起点，沿停车时车头朝向h作前向射线 R + t*h；
    // 2) 求该射线与锁存挡板直线的交点I，要求t>0且交点落在挡板有效线段范围内；
    // 3) 从I继续沿同一停车车头前向穿过挡板，再延伸avoid_target_extension_distance_得到目标T。
    // 目标点和保持yaw均在第一段避障前一次性锁定，后续点云和车辆平移不会改变它们。
    bool computeAvoidanceTargetPoint(
        const BarrierLineFit& fit,
        double stopped_robot_x,
        double stopped_robot_y,
        double stopped_robot_yaw,
        double& intersection_x,
        double& intersection_y,
        double& target_x,
        double& target_y) const {
        if (!fit.valid || !std::isfinite(stopped_robot_yaw)) {
            return false;
        }

        double board_dir_x = fit.direction_map_x;
        double board_dir_y = fit.direction_map_y;
        const double board_dir_norm = std::hypot(board_dir_x, board_dir_y);
        if (board_dir_norm < 1e-9) {
            return false;
        }
        board_dir_x /= board_dir_norm;
        board_dir_y /= board_dir_norm;

        // 停车时的当前车头朝向就是本次避障全程保持的方向，同时也是求交射线方向。
        const double heading_x = std::cos(stopped_robot_yaw);
        const double heading_y = std::sin(stopped_robot_yaw);

        // 二维直线求交：R + t*h = C + u*d。
        // cross(a,b)=a.x*b.y-a.y*b.x；若分母接近0，说明车头射线与挡板近似平行。
        const double denominator =
            heading_x * board_dir_y - heading_y * board_dir_x;
        if (std::abs(denominator) < 1e-6) {
            ROS_ERROR(
                "停车车头射线与挡板近似平行，无法稳定计算交点：heading=(%.3f, %.3f)，"
                "board=(%.3f, %.3f)。",
                heading_x, heading_y, board_dir_x, board_dir_y);
            return false;
        }

        const double center_from_robot_x = fit.center_map_x - stopped_robot_x;
        const double center_from_robot_y = fit.center_map_y - stopped_robot_y;

        const double t =
            (center_from_robot_x * board_dir_y -
             center_from_robot_y * board_dir_x) / denominator;
        const double u =
            (center_from_robot_x * heading_y -
             center_from_robot_y * heading_x) / denominator;

        // 只接受车头前方的真实交点，防止把车后方的数学交点当成绕障基准。
        if (!(t > 1e-4) || !std::isfinite(t) || !std::isfinite(u)) {
            ROS_ERROR(
                "停车车头射线与挡板交点不在车头前方：t=%.3f，u=%.3f。",
                t, u);
            return false;
        }

        // BarrierLineFit.center_map_*是拟合挡板有效线段中点，length是有效线段长度。
        // 要求交点落在实际挡板线段附近，而不是仅与无限延长线相交。
        const double half_length = 0.5 * std::max(0.0, fit.length);
        if (std::abs(u) > half_length + barrier_endpoint_margin_) {
            ROS_ERROR(
                "停车车头射线只与挡板延长线相交，交点超出有效板段："
                "沿板偏移=%.3fm，允许<=%.3fm。",
                std::abs(u), half_length + barrier_endpoint_margin_);
            return false;
        }

        intersection_x = stopped_robot_x + t * heading_x;
        intersection_y = stopped_robot_y + t * heading_y;

        // “交点向后延伸30cm”按车辆行驶方向解释为：从交点继续沿停车时车头指向，
        // 穿过挡板到板后侧再延伸设定距离。这样目标点始终位于挡板之后。
        target_x = intersection_x +
            heading_x * avoid_target_extension_distance_;
        target_y = intersection_y +
            heading_y * avoid_target_extension_distance_;

        return std::isfinite(intersection_x) &&
               std::isfinite(intersection_y) &&
               std::isfinite(target_x) &&
               std::isfinite(target_y);
    }

    // 第三段专用：不再按车体固定方向走固定距离，而是对绝对map目标点做二维P控制。
    // yaw仍锁定为停车后进入避障时的locked_yaw，速度/加速度/超时/容差全部沿用现有避障参数。
    bool executeAvoidanceTargetPoint(
        const string& segment_name,
        double target_x,
        double target_y,
        double max_speed,
        double locked_yaw) {
        double start_x = 0.0;
        double start_y = 0.0;
        double start_yaw = 0.0;
        if (!getRobotPose(start_x, start_y, start_yaw)) {
            ROS_ERROR("%s开始前无法读取AMCL定位。", segment_name.c_str());
            stopRobot();
            return false;
        }

        const double initial_distance =
            std::hypot(target_x - start_x, target_y - start_y);
        if (initial_distance <= avoid_position_tolerance_) {
            ROS_INFO("%s起点已在目标容差内，直接完成。", segment_name.c_str());
            stopRobot();
            return true;
        }

        ros::WallRate control_rate(avoid_control_rate_);
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(avoid_segment_timeout_);
        ros::WallTime last_time = ros::WallTime::now();

        ROS_WARN(
            "%s：起点=(%.3f, %.3f)，绝对绕障目标=(%.3f, %.3f)，"
            "初始距离=%.3fm，限速=%.3fm/s，保持yaw=%.2f°。",
            segment_name.c_str(), start_x, start_y, target_x, target_y,
            initial_distance, max_speed,
            locked_yaw * 180.0 / 3.14159265358979323846);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            double robot_x = 0.0;
            double robot_y = 0.0;
            double robot_yaw = 0.0;
            if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
                ROS_ERROR("%s过程中定位丢失。", segment_name.c_str());
                stopRobot();
                return false;
            }

            const double error_x = target_x - robot_x;
            const double error_y = target_y - robot_y;
            const double position_error = std::hypot(error_x, error_y);
            const double yaw_error = normalizeAngle(locked_yaw - robot_yaw);

            if (position_error <= avoid_position_tolerance_) {
                stopRobot();
                ROS_WARN(
                    "%s完成：当前位置=(%.3f, %.3f)，目标=(%.3f, %.3f)，位置误差=%.3fm。",
                    segment_name.c_str(), robot_x, robot_y,
                    target_x, target_y, position_error);
                return true;
            }

            double velocity_map_x = avoid_position_kp_ * error_x;
            double velocity_map_y = avoid_position_kp_ * error_y;
            double speed = std::hypot(velocity_map_x, velocity_map_y);
            if (speed > max_speed) {
                const double scale = max_speed / speed;
                velocity_map_x *= scale;
                velocity_map_y *= scale;
            } else if (speed > 1e-9 && speed < avoid_min_linear_speed_) {
                const double target_speed = std::min(
                    avoid_min_linear_speed_, max_speed);
                const double scale = target_speed / speed;
                velocity_map_x *= scale;
                velocity_map_y *= scale;
            }

            geometry_msgs::Twist desired_cmd;
            const double c = std::cos(robot_yaw);
            const double s = std::sin(robot_yaw);
            desired_cmd.linear.x =
                c * velocity_map_x + s * velocity_map_y;
            desired_cmd.linear.y =
                -s * velocity_map_x + c * velocity_map_y;
            desired_cmd.angular.z = clamp(
                avoid_yaw_kp_ * yaw_error,
                -avoid_max_angular_speed_, avoid_max_angular_speed_);

            if (std::abs(yaw_error) >
                avoid_heading_pause_error_deg_ *
                    3.14159265358979323846 / 180.0) {
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
                "%s：当前位置=(%.3f, %.3f)，目标=(%.3f, %.3f)，剩余=%.3fm，"
                "yaw误差=%.2f°，期望cmd=(%.3f, %.3f, %.3f)，"
                "斜坡cmd=(%.3f, %.3f, %.3f)",
                segment_name.c_str(), robot_x, robot_y, target_x, target_y,
                position_error,
                yaw_error * 180.0 / 3.14159265358979323846,
                desired_cmd.linear.x, desired_cmd.linear.y,
                desired_cmd.angular.z,
                twist_.linear.x, twist_.linear.y, twist_.angular.z);
            control_rate.sleep();
        }

        ROS_ERROR("%s超过%.2fs仍未到达绕障点。",
                  segment_name.c_str(), avoid_segment_timeout_);
        stopRobot();
        return false;
    }

    void holdRobotStopped(double duration) {
        if (duration <= 0.0) {
            return;
        }
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(duration);
        ros::WallRate stop_rate(avoid_control_rate_);
        while (ros::ok() && ros::WallTime::now() < deadline) {
            stopRobot();
            stop_rate.sleep();
        }
    }

    bool runOneTimeAvoidance() {
        BarrierLineFit fit;
        {
            std::lock_guard<std::mutex> lock(barrier_mutex_);
            fit = locked_barrier_fit_;
        }
        if (!fit.valid) {
            ROS_ERROR("挡板已触发但锁存拟合无效。");
            avoidance_completed_.store(true);
            return false;
        }

        avoidance_active_.store(true);
        // 不只发布一次零速，而是在保持时间内持续刷新零速，清掉底盘控制器
        // 中可能残留的最后一条漂移命令，然后才开始第一段平移。
        stopRobot();
        holdRobotStopped(avoid_stop_hold_time_);

        double robot_x = 0.0;
        double robot_y = 0.0;
        double locked_yaw = 0.0;
        if (!getRobotPose(robot_x, robot_y, locked_yaw)) {
            avoidance_active_.store(false);
            avoidance_completed_.store(true);
            return false;
        }

        // V28目标选择：
        // - false：沿用V27，真正停车后的当前车头射线与锁存挡板求交，再沿车头前向穿板延伸；
        // - true ：直接使用YAML给出的绝对map目标(x,y,yaw)，该yaw也替代停车yaw成为三段全程保持朝向。
        const double stopped_yaw = locked_yaw;
        double barrier_intersection_x = std::numeric_limits<double>::quiet_NaN();
        double barrier_intersection_y = std::numeric_limits<double>::quiet_NaN();
        double avoidance_target_x = 0.0;
        double avoidance_target_y = 0.0;
        const char* target_source = nullptr;

        if (avoid_use_fixed_target_) {
            avoidance_target_x = avoid_fixed_target_x_;
            avoidance_target_y = avoid_fixed_target_y_;
            locked_yaw = normalizeAngle(
                avoid_fixed_target_yaw_deg_ * 3.14159265358979323846 / 180.0);
            target_source = "YAML固定目标";
            ROS_WARN(
                "固定绕障目标模式开启：忽略本次挡板几何计算，三段避障使用YAML目标=(%.3f, %.3f, %.2f°)。"
                "停车时实际yaw=%.2f°，控制器将保持/纠正到设定yaw。",
                avoidance_target_x, avoidance_target_y,
                avoid_fixed_target_yaw_deg_,
                stopped_yaw * 180.0 / 3.14159265358979323846);
        } else {
            if (!computeAvoidanceTargetPoint(
                    fit, robot_x, robot_y, stopped_yaw,
                    barrier_intersection_x, barrier_intersection_y,
                    avoidance_target_x, avoidance_target_y)) {
                ROS_ERROR("无法根据停车车头射线与锁存挡板直线计算绕障点，安全终止避障。");
                stopRobot();
                avoidance_active_.store(false);
                avoidance_completed_.store(true);
                return false;
            }
            locked_yaw = stopped_yaw;
            target_source = "本次动态计算";
            ROS_WARN(
                "动态绕障目标：停车位姿=(%.3f, %.3f, %.2f°)，车头射线与挡板交点=(%.3f, %.3f)，"
                "沿停车车头前向穿板后再延伸%.3fm。",
                robot_x, robot_y,
                stopped_yaw * 180.0 / 3.14159265358979323846,
                barrier_intersection_x, barrier_intersection_y,
                avoid_target_extension_distance_);
        }

        // V28：避障真正开始前单独打印最终采用的目标，方便比赛场地测试后直接抄回YAML。
        ROS_WARN(
            "========== 最终绕障点【可写入YAML】：x=%.3f, y=%.3f, yaw_deg=%.2f, source=%s ==========" ,
            avoidance_target_x, avoidance_target_y,
            locked_yaw * 180.0 / 3.14159265358979323846,
            target_source);
        ROS_WARN(
            "进入三段避障：流程=右移%.3fm -> 前进%.3fm -> P控制到最终绕障点；"
            "三段全程保持最终选定yaw=%.2f°，不使用MoveBase。",
            avoid_right_distance_, avoid_forward_distance_,
            locked_yaw * 180.0 / 3.14159265358979323846);

        const bool right_ok = executeAvoidanceSegment(
            "第一段向右平移", 0.0, -1.0,
            avoid_right_distance_, avoid_right_speed_, locked_yaw);
        if (right_ok && avoid_segment_pause_time_ > 0.0) {
            holdRobotStopped(avoid_segment_pause_time_);
        }

        const bool forward_ok = right_ok && executeAvoidanceSegment(
            "第二段向前平移", 1.0, 0.0,
            avoid_forward_distance_, avoid_forward_speed_, locked_yaw);
        if (forward_ok && avoid_segment_pause_time_ > 0.0) {
            holdRobotStopped(avoid_segment_pause_time_);
        }

        const bool left_ok = forward_ok && executeAvoidanceTargetPoint(
            "第三段P控制到绕障点",
            avoidance_target_x, avoidance_target_y,
            avoid_left_speed_, locked_yaw);

        if (left_ok) {
            for (int i = 0; i < avoid_camera_flush_frames_; ++i) {
                cap_.grab();
            }
            ROS_WARN("三段避障完成：第三段已闭环到达最终绕障点，直接恢复原lineo_right巡线。");
        }

        stopRobot();
        avoidance_active_.store(false);
        avoidance_completed_.store(true);
        const bool no_drift_was_active = barrier_no_drift_active_.exchange(false);
        if (no_drift_was_active) {
            ROS_WARN("三段避障结束：已取消挡板近区丢线右转抑制，后续恢复原丢线/甩尾逻辑。");
        }
        {
            // 清掉触发标志和最近挡板缓存，确保三段避障完成后不再用旧板
            // 拦截丢线逻辑；本次服务后续丢线完全按原状态机处理。
            std::lock_guard<std::mutex> lock(barrier_mutex_);
            latest_barrier_fit_ = BarrierLineFit();
            locked_barrier_fit_ = BarrierLineFit();
            obstacle_triggered_.store(false);
        }
        barrier_tracking_speed_limit_.store(
            std::numeric_limits<double>::infinity());

        if (left_ok) {
            trace_failed_count_ = 0;
            lost_cancel_success_count_ = 0;
            drift_recover_success_count_ = 0;
            drift_recovery_active_ = false;
            integration_ = 0.0;
            pre_error_ = 0.0;
            disableDashedForRestOfServiceAfterAvoidance();
            ROS_WARN("避障后丢线禁止状态已取消，恢复原丢线/漂移状态机。");
        }
        return left_ok;
    }

    // 启动阶段定向直行控制：
    // 1) 平移目标先在map坐标系中固定为start_heading_yaw_deg_方向；
    // 2) 再根据当前robot_yaw转换成base_link下的vx/vy，因此即使车头有轻微角度误差，
    //    平移轨迹仍尽量沿地图系-90°方向，不会因为车头偏几度就斜着走；
    // 3) wz独立闭环把车头压回目标角度。误差过大时暂停平移，只纠正朝向。
    void computeStartStraightCommand(double robot_yaw, geometry_msgs::Twist& cmd) {
        const double pi = 3.14159265358979323846;
        const double target_yaw = start_heading_yaw_deg_ * pi / 180.0;
        const double yaw_error = normalizeAngle(target_yaw - robot_yaw);
        const double deadband = start_heading_deadband_deg_ * pi / 180.0;
        const double pause_error = start_heading_pause_error_deg_ * pi / 180.0;

        cmd = geometry_msgs::Twist();

        double wz = start_heading_kp_ * yaw_error;
        wz = clamp(wz, -start_heading_max_angular_speed_,
                        start_heading_max_angular_speed_);
        if (std::abs(yaw_error) <= deadband) {
            wz = 0.0;
        }
        cmd.angular.z = wz;

        if (std::abs(yaw_error) > pause_error) {
            // 朝向偏差已经明显，继续前进只会把车带离赛道中心。
            // 此时先停住平移，原地把车头拉回-90°附近。
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            return;
        }

        // 目标平移速度固定在map坐标系的目标朝向上。
        const double vx_map = start_straight_speed_ * std::cos(target_yaw);
        const double vy_map = start_straight_speed_ * std::sin(target_yaw);

        // map -> base_link速度变换。全向底盘允许vx、vy同时输出。
        const double c = std::cos(robot_yaw);
        const double sn = std::sin(robot_yaw);
        cmd.linear.x = c * vx_map + sn * vy_map;
        cmd.linear.y = -sn * vx_map + c * vy_map;
    }

    double computeStartForwardProgress(double start_x, double start_y,
                                       double robot_x, double robot_y) const {
        const double pi = 3.14159265358979323846;
        const double target_yaw = start_heading_yaw_deg_ * pi / 180.0;
        const double dx = robot_x - start_x;
        const double dy = robot_y - start_y;
        // 只统计目标前进方向上的投影距离。对于-90°，基本等价于 start_y - robot_y。
        return dx * std::cos(target_yaw) + dy * std::sin(target_yaw);
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

        for (int i = 0; ros::ok() && i < initial_pose_publish_count_; ++i) {
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
            return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
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

        // 将地图坐标系中的位置误差转换到base_link坐标系，
        // 全向底盘可以同时输出vx、vy并独立调整最终朝向。
        const double cos_yaw = std::cos(robot_yaw);
        const double sin_yaw = std::sin(robot_yaw);
        const double x_error = cos_yaw * dx_map + sin_yaw * dy_map;
        const double y_error = -sin_yaw * dx_map + cos_yaw * dy_map;

        const double pi = 3.14159265358979323846;
        const double target_yaw = docking_goal_yaw_deg_ * pi / 180.0;
        const double yaw_error = normalizeAngle(target_yaw - robot_yaw);

        if (distance_error <= docking_position_tolerance_ &&
            std::abs(yaw_error) <= docking_yaw_tolerance_) {
            ROS_WARN(
                "停靠完成：当前位置=(%.3f, %.3f)，位置误差=%.3fm，方向误差=%.3frad。",
                robot_x, robot_y, distance_error, yaw_error);
            return true;
        }

        double vx = docking_linear_x_gain_ * x_error;
        double vy = docking_linear_y_gain_ * y_error;
        double wz = docking_angular_gain_ * yaw_error;

        if (std::abs(x_error) <= docking_position_tolerance_ * 0.65) {
            vx = 0.0;
        }
        if (std::abs(y_error) <= docking_position_tolerance_ * 0.65) {
            vy = 0.0;
        }
        if (std::abs(yaw_error) <= docking_yaw_tolerance_) {
            wz = 0.0;
        }

        vx = applyMinimumMagnitude(vx, docking_min_linear_speed_);
        vy = applyMinimumMagnitude(vy, docking_min_linear_speed_);
        wz = applyMinimumMagnitude(wz, docking_min_angular_speed_);

        desired_cmd.linear.x = clamp(vx, -docking_max_vel_x_, docking_max_vel_x_);
        desired_cmd.linear.y = clamp(vy, -docking_max_vel_y_, docking_max_vel_y_);
        desired_cmd.angular.z =
            clamp(wz, -docking_max_vel_theta_, docking_max_vel_theta_);
        return false;
    }

    void applyDockingAccelerationLimits(
        const geometry_msgs::Twist& desired_cmd,
        double dt) {
        dt = clamp(dt, 0.01, 0.20);

        const double max_delta_x = docking_acc_lim_x_ * dt;
        const double max_delta_y = docking_acc_lim_y_ * dt;
        const double max_delta_theta = docking_acc_lim_theta_ * dt;

        // 以巡线最后一帧的twist_为初值，保证接管时不先插入零速度。
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
            if (computeDockingCommand(robot_x, robot_y, robot_yaw, desired_cmd)) {
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
                "纯PP停靠中：当前位置=(%.3f, %.3f, %.1f°)，cmd=(%.3f, %.3f, %.3f)",
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

    // 左点追踪逻辑
    void runLeftPointTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        if (!point_forward_) {
            // 丢线旋转
            ROS_INFO("丢线旋转中");
            // 此处保持原赛道“左点完成后转向”的原有速度；
            // out_forward/out_turn 仅用于右线丢失后的固定右转。
            twist_.linear.x = 0.075;
            twist_.angular.z = -1.0;
            out_.write(cropped);
            
            // 旋转到位后切换模式
            pose_client_.call(pose_);
            // ROS_INFO("角度%f,位姿%f",out_turn_angel_,pose_.response.pose_at[2]);
            if (pose_.response.pose_at[2] < out_turn_angel_) {
                left_point_start_ = false;
                double_line_ = true;
                x_max_ = 0.5;
                nh_.getParam("/lineo_right/double_P", p_);
                nh_.getParam("/lineo_right/double_I", i_);
                nh_.getParam("/lineo_right/double_D", d_);
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
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        }
        out_.write(cropped);
    }

    // 根据疑似丢线失败次数计算当前前进速度。
    // 若lost_confirm_frames_=x，第n次失败：
    // vx = x_max_ - n * (x_max_ - out_forward_) / x
    // 因而第x次失败时vx严格等于out_forward_。
    double lostConfirmForwardSpeed(int failed_count) const {
        const int x = std::max(1, lost_confirm_frames_);
        const int n = std::max(0, std::min(failed_count, x));
        const double start_v = std::max(0.0, x_max_);
        const double end_v = std::min(std::max(0.0, out_forward_), start_v);
        const double step = (start_v - end_v) / static_cast<double>(x);
        return std::max(end_v, start_v - step * static_cast<double>(n));
    }

    void setLostConfirmCommand() {
        twist_.linear.x = lostConfirmForwardSpeed(trace_failed_count_);
        twist_.linear.y = lost_confirm_linear_y_;
        twist_.angular.z = lost_confirm_angular_z_;
    }

    void setDriftRecoveryCommand() {
        // 全向底盘丢线恢复：缓慢前进 + 向左横移 + 车头向右旋转。
        twist_.linear.x = out_forward_;
        twist_.linear.y = std::abs(out_left_speed_);
        twist_.angular.z = -std::abs(out_turn_);
    }

    // 漂移恢复软退出：与丢线渐进减速采用同样的线性插值思想。
    // 若drift_recover_confirm_frames_=x，连续重新识别到线n帧（1 <= n < x）时：
    // remaining = 1 - n / x
    // vy = out_left_speed_ * remaining
    // wz = -out_turn_ * remaining
    // vx在确认期间保持out_forward_，避免候选线刚出现就突然加速。
    // 第x帧不再调用本函数，而是直接退出漂移并使用当前有效轨迹恢复正常PID。
    void setDriftSoftExitCommand(int recover_success_count) {
        const int x = std::max(1, drift_recover_confirm_frames_);
        const int n = std::max(0, std::min(recover_success_count, x));
        const double remaining = std::max(0.0,
            1.0 - static_cast<double>(n) / static_cast<double>(x));

        twist_.linear.x = out_forward_;
        twist_.linear.y = std::abs(out_left_speed_) * remaining;
        twist_.angular.z = -std::abs(out_turn_) * remaining;
    }

    // V29正常巡线视觉优先级：
    // 1) 挡板急停/避障；
    // 2) 本对话最早稳定版 find_track_edge + trace_edge：成功即直接采用，不做任何位置门控；
    // 3) 只有原稳定实线追踪明确失败，才启动虚线低阈值重建（二级视觉源）；
    // 4) DASHED_LOCK内两种视觉源都暂时缺失时进入DASHED_HOLD；
    // 5) 最后才允许进入原LOST_CONFIRM / DRIFT_RECOVERY。
    //
    // DASHED_LOCK只表示“当前仍处在虚线路段”，不再代表“每帧必须用虚线重建”。
    // 如果1cm白/1cm蓝在当前画面恰好连成连续白边，就直接使用原trace_edge结果；
    // 下一帧重新断开时，再自然回退到虚线重建。
    bool runNormalTracking(Mat& gray_img, const Mat& dashed_img, Mat& cropped,
                           bool enable_lost_turn = true) {
        displayStream_.str("");

        if (enable_lost_turn && obstacle_triggered_.load() &&
            !avoidance_completed_.load()) {
            twist_ = geometry_msgs::Twist();
            return false;
        }

        // 一级视觉源：完整复用原省赛 find_track_edge + trace_edge。
        vector<Point> start_points = find_track_edge(gray_img, 340, 70, cropped);
        RaceTrack continuous_track;
        const bool raw_continuous_found = trace_edge(
            gray_img, start_points, continuous_track, cropped);

        // V25关键修复：正常白线完全回到本对话最早稳定版的判定。
        // find_track_edge + trace_edge 只要成功，就无条件作为本帧右线使用；
        // 不再经过动态右侧走廊、上一帧prior、点数比例等任何额外门控。
        // 只有原稳定版 trace_edge 本帧明确失败，才允许启动虚线重建。
        const bool reliable_continuous_found = raw_continuous_found;

        RaceTrack racetrack;
        bool line_found = false;
        bool dashed_used = false;
        bool continuous_used = false;
        bool dashed_hold_this_frame = false;
        bool dashed_lock_released_after_hold = false;
        int dashed_hold_miss_snapshot = 0;

        if (!enable_lost_turn) {
            // 启动阶段：有连续线直接用；没有连续线才尝试虚线补位。
            if (reliable_continuous_found) {
                racetrack = continuous_track;
                line_found = true;
                continuous_used = true;
            } else if (dashedRuntimeEnabled() && !dashed_img.empty()) {
                RaceTrack dashed_track;
                if (traceDashedRightEdge(
                        dashed_img, dashed_track, cropped, false)) {
                    racetrack = dashed_track;
                    line_found = true;
                    dashed_used = true;
                }
            }
        } else {
            // -----------------------------------------------------------------
            // V29一级优先：原稳定版trace_edge一旦成功，本帧不调用traceDashedRightEdge。
            // 因此低阈值图里的零散反光白块根本没有机会参与本帧PID。
            // -----------------------------------------------------------------
            if (reliable_continuous_found) {
                racetrack = continuous_track;
                line_found = true;
                continuous_used = true;

                if (dashed_lock_active_) {
                    // 当前仍处在虚线路段，只是这一帧虚线在视觉上连成了连续边。
                    // 直接使用更可靠的连续边，但不退出DASHED_LOCK。
                    dashed_hold_active_ = false;
                    dashed_lock_miss_count_ = 0;
                    dashed_solid_relock_count_ = 0;
                    ROS_INFO_THROTTLE(
                        0.5,
                        "V29 DASHED_ZONE：本帧原稳定版trace_edge成功，直接使用CONTINUOUS，"
                        "跳过虚线白块重建并保持DASHED_LOCK。");
                } else {
                    dashed_lock_confirm_count_ = 0;
                    dashed_hold_active_ = false;
                }
            } else {
                // 只有原稳定版trace_edge明确失败，才启动二级虚线重建。
                RaceTrack dashed_track;
                bool dashed_found = false;
                if (dashedRuntimeEnabled() && !dashed_img.empty()) {
                    dashed_found = traceDashedRightEdge(
                        dashed_img, dashed_track, cropped, dashed_lock_active_);
                }

                if (dashedRuntimeEnabled() && dashed_lock_active_) {
                    if (dashed_found) {
                        racetrack = dashed_track;
                        line_found = true;
                        dashed_used = true;
                        dashed_hold_active_ = false;
                        dashed_lock_miss_count_ = 0;
                        dashed_solid_relock_count_ = 0;
                    } else {
                        // DASHED_LOCK里：连续边和虚线重建都失败，才进入HOLD。
                        ++dashed_lock_miss_count_;
                        dashed_hold_active_ = true;
                        dashed_hold_miss_snapshot = dashed_lock_miss_count_;

                        if (dashed_lock_miss_count_ <= dashed_hold_frames_) {
                            dashed_hold_this_frame = true;
                            ROS_WARN_THROTTLE(
                                0.5,
                                "DASHED_HOLD：原稳定实线和锁定虚线本帧均缺失，miss=%d/%d；"
                                "屏蔽普通丢线/甩尾。",
                                dashed_lock_miss_count_, dashed_hold_frames_);
                        } else {
                            const int expired_miss = dashed_lock_miss_count_;
                            resetDashedLockState();
                            resetLineVisualPrior();
                            clearLineLossStateForReliableDashed(true);
                            dashed_lock_released_after_hold = true;
                            ROS_ERROR_THROTTLE(
                                0.5,
                                "DASHED_HOLD连续%d帧仍没有原稳定实线/虚线，释放虚线锁；"
                                "当前帧安全直行，下一帧才从普通丢线0/%d重新计数。",
                                expired_miss, lost_confirm_frames_);
                        }
                    }
                } else {
                    // 尚未锁定虚线：只有一级连续线失败时才允许尝试建立虚线。
                    if (dashed_found) {
                        racetrack = dashed_track;
                        line_found = true;
                        dashed_used = true;
                        dashed_hold_active_ = false;
                        ++dashed_lock_confirm_count_;

                        if (dashed_lock_confirm_count_ >=
                            dashed_lock_confirm_frames_) {
                            dashed_lock_active_ = true;
                            dashed_lock_miss_count_ = 0;
                            dashed_solid_relock_count_ = 0;
                            ROS_WARN(
                                "连续%d帧确认真实虚线，进入DASHED_LOCK。"
                                "V29后续始终先跑原稳定trace_edge，只有trace_edge失败时才重建虚线。",
                                dashed_lock_confirm_frames_);
                        }
                    } else {
                        dashed_lock_confirm_count_ = 0;
                        dashed_hold_active_ = false;
                    }
                }
            }
        }

        // 只有真正采用的视觉轨迹更新下一帧先验；HOLD不更新。
        if (line_found) {
            updateLineVisualPrior(racetrack);
        }

        if (!enable_lost_turn) {
            trace_failed_count_ = 0;
            lost_cancel_success_count_ = 0;
            drift_recover_success_count_ = 0;
            drift_recovery_active_ = false;

            if (!line_found) {
                twist_.linear.x = start_straight_speed_;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;
                displayStream_ << "初始直行找右线 线速度: " << twist_.linear.x;
                putText(cropped, displayStream_.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
                out_.write(cropped);
                return false;
            }

            double line_error = error_calculater(racetrack.points, cropped);
            integration_ += line_error * 0.03;
            integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1,
                                   abs(line_error)/integration_limit_ +1);
            double diff = clamp(line_error - pre_error_, -50.0, 50.0);
            twist_.linear.x = x_max_ / exp(abs(line_error) / 100.0);
            twist_.linear.y = 0.0;
            twist_.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
            pre_error_ = line_error;
            out_.write(cropped);
            return true;
        }

        // 挡板优先级始终高于HOLD和普通丢线。
        if (!line_found && latchLatestBarrierForLineLoss()) {
            twist_ = geometry_msgs::Twist();
            clearLineLossStateForReliableDashed(false);
            dashed_hold_active_ = false;
            displayStream_ << "视觉缺失且挡板接近：硬锁存停车，等待三段避障";
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            out_.write(cropped);
            return false;
        }

        if (dashed_hold_this_frame) {
            clearLineLossStateForReliableDashed(true);
            setDashedHoldCommand(dashed_hold_miss_snapshot);

            const double decay = std::pow(
                dashed_hold_angular_decay_,
                static_cast<double>(std::max(1, dashed_hold_miss_snapshot)));
            displayStream_ << "DASHED_HOLD "
                           << dashed_hold_miss_snapshot << "/"
                           << dashed_hold_frames_
                           << " vx:" << twist_.linear.x
                           << " wz:" << twist_.angular.z
                           << " decay:" << decay;
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);
            out_.write(cropped);
            return false;
        }

        if (dashed_lock_released_after_hold) {
            twist_ = geometry_msgs::Twist();
            twist_.linear.x = std::min(std::max(0.0, dashed_hold_speed_),
                                       std::max(0.0, x_max_));
            twist_.linear.y = 0.0;
            twist_.angular.z = 0.0;
            displayStream_ << "DASHED_HOLD EXPIRED -> LOST next frame"
                           << " vx:" << twist_.linear.x;
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 165, 255), 1);
            out_.write(cropped);
            return false;
        }

        // 虚线路段内，无论本帧来源是连续边还是虚线重建，都属于高可信观测。
        const bool reliable_dashed_zone_visual =
            line_found && (dashed_used || dashed_lock_active_);
        if (reliable_dashed_zone_visual) {
            const bool had_loss_state =
                trace_failed_count_ > 0 || drift_recovery_active_ ||
                drift_recover_success_count_ > 0 || lost_cancel_success_count_ > 0;
            clearLineLossStateForReliableDashed(true);
            if (had_loss_state) {
                ROS_WARN_THROTTLE(
                    0.5,
                    "虚线路段可靠视觉恢复：立即取消原LOST_CONFIRM/DRIFT状态，直接恢复PID。");
            }
        }

        // -----------------------------------------------------------------
        // V30：挡板近区禁止丢线右转/甩尾。
        // 一旦雷达在避障前确认板距进入barrier_no_drift_distance_，该状态锁存到避障结束。
        // - 当前有线：立即清掉可能残留的LOST/DRIFT状态，直接走下面正常PID；
        // - 当前无任何可靠视觉：仍允许疑似丢线逐帧减速，但失败计数最多停在阈值，
        //   绝不进入DRIFT_RECOVERY。车辆保持wz=lost_confirm_angular_z（默认0）继续靠近，
        //   直到barrier_trigger_distance / barrier_lost_mode_trigger_distance正常触发避障。
        // 避障完成后barrier_no_drift_active_立即释放，后半程完全恢复原状态机。
        // -----------------------------------------------------------------
        if (barrier_no_drift_active_.load() &&
            !avoidance_completed_.load()) {
            if (drift_recovery_active_) {
                ROS_WARN_THROTTLE(
                    0.5,
                    "已进入挡板%.3fm近区：取消当前丢线右转/甩尾锁存，等待挡板触发避障。",
                    barrier_no_drift_distance_);
            }
            drift_recovery_active_ = false;
            drift_recover_success_count_ = 0;

            if (line_found) {
                trace_failed_count_ = 0;
                lost_cancel_success_count_ = 0;
            } else {
                lost_cancel_success_count_ = 0;
                trace_failed_count_ = std::min(
                    trace_failed_count_ + 1,
                    std::max(1, lost_confirm_frames_));
                setLostConfirmCommand();

                displayStream_ << "BARRIER NO-DRIFT <="
                               << barrier_no_drift_distance_
                               << "m fail:" << trace_failed_count_
                               << "/" << lost_confirm_frames_
                               << " vx:" << twist_.linear.x
                               << " wz:" << twist_.angular.z;
                putText(cropped, displayStream_.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
                out_.write(cropped);
                ROS_WARN_THROTTLE(
                    0.5,
                    "挡板近区视觉缺失：板距已进入<=%.3fm抑制区，"
                    "丢线计数=%d/%d，仅减速直行(vx=%.3f,wz=%.3f)，禁止右转甩尾。",
                    barrier_no_drift_distance_, trace_failed_count_,
                    lost_confirm_frames_, twist_.linear.x, twist_.angular.z);
                return false;
            }
        }

        // ---------------- 已确认普通丢线：锁存漂移 ----------------
        if (drift_recovery_active_) {
            if (line_found) {
                ++drift_recover_success_count_;
                if (drift_recover_success_count_ < drift_recover_confirm_frames_) {
                    setDriftSoftExitCommand(drift_recover_success_count_);
                    const double remaining = std::max(0.0,
                        1.0 - static_cast<double>(drift_recover_success_count_) /
                              static_cast<double>(std::max(1, drift_recover_confirm_frames_)));
                    displayStream_ << "漂移软退出，候选恢复线 "
                                   << drift_recover_success_count_ << "/"
                                   << drift_recover_confirm_frames_
                                   << " 剩余甩尾:" << remaining * 100.0 << "%"
                                   << " vx:" << twist_.linear.x
                                   << " vy:" << twist_.linear.y
                                   << " wz:" << twist_.angular.z;
                    putText(cropped, displayStream_.str(), Point(50, 50),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 165, 255), 1);
                    out_.write(cropped);
                    return true;
                }

                ROS_INFO("右线连续%d帧重新识别有效，退出漂移并恢复PID巡线",
                         drift_recover_confirm_frames_);
                drift_recovery_active_ = false;
                drift_recover_success_count_ = 0;
                trace_failed_count_ = 0;
                lost_cancel_success_count_ = 0;
                integration_ = 0.0;
                pre_error_ = 0.0;
            } else {
                drift_recover_success_count_ = 0;
                setDriftRecoveryCommand();
                displayStream_ << "右线丢失，锁存左移甩尾找线"
                               << " vx:" << twist_.linear.x
                               << " vy:" << twist_.linear.y
                               << " wz:" << twist_.angular.z;
                putText(cropped, displayStream_.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 165, 255), 1);
                out_.write(cropped);
                return false;
            }
        }

        // ---------------- 正常/疑似普通丢线阶段：当前帧找到线 ----------------
        if (line_found) {
            if (trace_failed_count_ > 0) {
                ++lost_cancel_success_count_;
                if (lost_cancel_success_count_ < lost_cancel_confirm_frames_) {
                    setLostConfirmCommand();
                    displayStream_ << "疑似丢线，候选恢复实线 "
                                   << lost_cancel_success_count_ << "/"
                                   << lost_cancel_confirm_frames_
                                   << "，保持减速 vx:" << twist_.linear.x
                                   << " fail:" << trace_failed_count_;
                    putText(cropped, displayStream_.str(), Point(50, 50),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
                    out_.write(cropped);
                    return true;
                }

                ROS_INFO("疑似丢线后连续%d帧重新识别有效，取消丢线判定（此前失败%d次）",
                         lost_cancel_confirm_frames_, trace_failed_count_);
                trace_failed_count_ = 0;
                lost_cancel_success_count_ = 0;
            } else {
                lost_cancel_success_count_ = 0;
            }

            double line_error = error_calculater(racetrack.points, cropped);
            integration_ += line_error * 0.03;
            integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1,
                                   abs(line_error)/integration_limit_ +1);
            double diff = clamp(line_error - pre_error_, -50.0, 50.0);

            twist_.linear.x = x_max_ / exp(abs(line_error) / 100.0);
            twist_.linear.y = 0.0;
            twist_.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
            pre_error_ = line_error;

            // DASHED_LOCK区域里连续边也可能只是虚线恰好连成一条线；
            // HOLD应记住最近一次可靠虚线路段控制量，而不只记低阈值虚线拟合帧。
            if (dashed_used || dashed_lock_active_) {
                rememberReliableDashedCommand();
            }

            const char* source_name = dashed_used
                ? (dashed_lock_active_ ? "DASHED_LOCK " : "DASHED ")
                : ((dashed_lock_active_ && continuous_used)
                    ? "CONTINUOUS@DASHED_ZONE "
                    : "CONTINUOUS ");
            displayStream_ << source_name
                           << "error: " << line_error
                           << " P: " << line_error*p_
                           << " I: " << integration_*i_
                           << " D: " << diff*d_
                           << " 角速度: " << twist_.angular.z;
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
            out_.write(cropped);
            return true;
        }

        // ---------------- 真正普通丢线：原状态机完全保留 ----------------
        lost_cancel_success_count_ = 0;
        ++trace_failed_count_;

        if (trace_failed_count_ >= lost_confirm_frames_) {
            trace_failed_count_ = lost_confirm_frames_;
            drift_recovery_active_ = true;
            drift_recover_success_count_ = 0;
            resetLineVisualPrior();
            setDriftRecoveryCommand();

            const double trans_speed = std::hypot(twist_.linear.x, twist_.linear.y);
            ROS_INFO("右线达到丢线阈值%d帧，锁存漂移：vx=%.3f，vy=%.3f(向左)，wz=%.3f(向右)，合成平移=%.3f",
                     lost_confirm_frames_, twist_.linear.x, twist_.linear.y,
                     twist_.angular.z, trans_speed);

            displayStream_ << "确认丢线，锁存左移甩尾"
                           << " vx:" << twist_.linear.x
                           << " vy:" << twist_.linear.y
                           << " wz:" << twist_.angular.z;
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 165, 255), 1);
        } else {
            setLostConfirmCommand();
            const double step = (std::max(0.0, x_max_) -
                                std::min(std::max(0.0, out_forward_), std::max(0.0, x_max_))) /
                                static_cast<double>(std::max(1, lost_confirm_frames_));
            displayStream_ << "疑似丢线 " << trace_failed_count_ << "/"
                           << lost_confirm_frames_
                           << "，逐帧减速 vx:" << twist_.linear.x
                           << " step:" << step;
            putText(cropped, displayStream_.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
        }

        out_.write(cropped);
        return false;
    }

    // 工具函数：数值 clamping
    template <typename T>
    T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    // OpenCV自适应阈值要求block为>1的奇数。现场改YAML时自动修正，避免直接崩溃。
    static int sanitizeAdaptiveBlock(int value) {
        value = std::max(3, value);
        if (value % 2 == 0) {
            ++value;
        }
        return value;
    }

    void buildFilteredBinary(const Mat& source,
                             Mat& output,
                             int adaptive_block,
                             double adaptive_c,
                             double min_contour_area) {
        Mat binary;
        adaptiveThreshold(
            source, binary, 255,
            ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY,
            adaptive_block, adaptive_c);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        Mat contour_input = binary.clone();
        findContours(
            contour_input, contours, hierarchy,
            RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        output = Mat::zeros(binary.size(), CV_8UC1);
        for (size_t i = 0; i < contours.size(); ++i) {
            if (contourArea(contours[i]) > min_contour_area) {
                drawContours(output, contours, static_cast<int>(i), Scalar(255), FILLED);
            }
        }
    }

    // 原实线通道：保持V20实际使用的算法，只是把3个硬编码值改为YAML参数。
    void threshold_image(Mat& gray) {
        Mat source = gray.clone();
        buildFilteredBinary(
            source, gray,
            adaptive_block_, adaptive_c_, min_contour_area_);
    }

    // 虚线专用通道：不覆盖gray_img，允许保留面积明显小于250px²的1cm白块。
    void buildDashedBinary(const Mat& source, Mat& dashed_binary) {
        buildFilteredBinary(
            source, dashed_binary,
            dashed_adaptive_block_, dashed_adaptive_c_,
            dashed_min_contour_area_);
    }

    static double normalizeLineAngle(double angle) {
        const double pi = 3.14159265358979323846;
        while (angle < 0.0) {
            angle += pi;
        }
        while (angle >= pi) {
            angle -= pi;
        }
        return angle;
    }

    static double lineAngleDifference(double first, double second) {
        const double pi = 3.14159265358979323846;
        double diff = std::abs(
            normalizeLineAngle(first) - normalizeLineAngle(second));
        if (diff > pi * 0.5) {
            diff = pi - diff;
        }
        return std::abs(diff);
    }

    // 与error_calculater使用完全同一套透视关系：当车辆位于目标中心附近时，
    // 右边线理论像素位置约为 320 + center_offset(y)。V22只把它用于候选过滤，
    // 不参与PID，因此不会改动省赛已经调好的控制标定。
    double expectedRightLineX(double y) const {
        return 320.0 + static_cast<double>(center_distance) +
               (y - 140.0) * 1.40;
    }

    bool isPointInsideExpectedRightCorridor(double x, double y) const {
        if (x < static_cast<double>(dashed_min_x_)) {
            return false;
        }
        if (!dashed_use_expected_corridor_) {
            return true;
        }

        const double expected_x = expectedRightLineX(y);
        const double min_x = std::max(
            static_cast<double>(dashed_min_x_),
            expected_x - dashed_corridor_left_margin_px_);
        const double max_x = std::min(
            639.0,
            expected_x + dashed_corridor_right_margin_px_);
        return x >= min_x && x <= max_x;
    }

    bool fitTrackLineModel(
        const RaceTrack& track,
        double& a,
        double& b,
        double& angle,
        double& min_y,
        double& max_y) const {
        if (track.points.size() < 8) {
            return false;
        }

        Vec4f line;
        fitLine(track.points, line, DIST_L2, 0, 0.01, 0.01);
        const double vx = line[0];
        const double vy = line[1];
        if (!std::isfinite(vx) || !std::isfinite(vy) ||
            std::abs(vy) < 0.05) {
            return false;
        }

        a = vx / vy;
        b = line[2] - a * line[3];
        angle = normalizeLineAngle(std::atan2(vy, vx));
        min_y = std::numeric_limits<double>::infinity();
        max_y = -std::numeric_limits<double>::infinity();
        for (const Point& p : track.points) {
            min_y = std::min(min_y, static_cast<double>(p.y));
            max_y = std::max(max_y, static_cast<double>(p.y));
        }
        return std::isfinite(a) && std::isfinite(b) &&
               std::isfinite(min_y) && std::isfinite(max_y);
    }

    bool isTrackConsistentWithPrior(
        const RaceTrack& track,
        double x_tolerance_px,
        double angle_tolerance_deg) const {
        if (!line_prior_valid_) {
            return false;
        }

        double a = 0.0, b = 0.0, angle = 0.0;
        double min_y = 0.0, max_y = 0.0;
        if (!fitTrackLineModel(track, a, b, angle, min_y, max_y)) {
            return false;
        }

        const double ref_y = 0.5 * (min_y + max_y);
        const double candidate_x = a * ref_y + b;
        const double prior_x = line_prior_a_ * ref_y + line_prior_b_;
        const double angle_diff_deg = lineAngleDifference(
            angle, line_prior_angle_rad_) *
            180.0 / 3.14159265358979323846;

        return std::abs(candidate_x - prior_x) <= x_tolerance_px &&
               angle_diff_deg <= angle_tolerance_deg;
    }

    double trackExpectedRightCorridorRatio(const RaceTrack& track) const {
        if (track.points.empty()) {
            return 0.0;
        }

        int checked = 0;
        int inside = 0;
        for (size_t i = 0; i < track.points.size(); i += 3) {
            const Point& p = track.points[i];
            ++checked;
            if (isPointInsideExpectedRightCorridor(p.x, p.y)) {
                ++inside;
            }
        }
        return checked > 0
            ? static_cast<double>(inside) / static_cast<double>(checked)
            : 0.0;
    }

    bool isTrackInsideExpectedRightCorridor(const RaceTrack& track) const {
        return track.points.size() >= 8 &&
               trackExpectedRightCorridorRatio(track) >= 0.70;
    }

    // V24遗留函数：V25正常白线不再调用此门控，保留仅为源码兼容/便于回退。
    bool isReliableContinuousRightTrack(const RaceTrack& track) const {
        if (static_cast<int>(track.points.size()) < continuous_right_min_points_) {
            return false;
        }

        if (trackExpectedRightCorridorRatio(track) <
            continuous_right_corridor_ratio_) {
            return false;
        }

        if (line_prior_valid_ &&
            !isTrackConsistentWithPrior(
                track,
                continuous_right_max_prior_jump_px_,
                continuous_right_max_prior_angle_deg_)) {
            return false;
        }

        return true;
    }

    void resetLineVisualPrior() {
        line_prior_valid_ = false;
        line_prior_a_ = 0.0;
        line_prior_b_ = 0.0;
        line_prior_angle_rad_ = 0.0;
    }

    void resetLastDashedCommand() {
        last_dashed_cmd_valid_ = false;
        last_dashed_vx_ = 0.0;
        last_dashed_wz_ = 0.0;
    }

    void resetDashedLockState() {
        dashed_lock_active_ = false;
        dashed_hold_active_ = false;
        dashed_lock_confirm_count_ = 0;
        dashed_lock_miss_count_ = 0;
        dashed_solid_relock_count_ = 0;
        resetLastDashedCommand();
    }

    void rememberReliableDashedCommand() {
        last_dashed_cmd_valid_ = true;
        last_dashed_vx_ = twist_.linear.x;
        last_dashed_wz_ = twist_.angular.z;
    }

    void clearLineLossStateForReliableDashed(bool reset_pid_history) {
        const bool had_loss_state =
            trace_failed_count_ > 0 ||
            lost_cancel_success_count_ > 0 ||
            drift_recover_success_count_ > 0 ||
            drift_recovery_active_;

        trace_failed_count_ = 0;
        lost_cancel_success_count_ = 0;
        drift_recover_success_count_ = 0;
        drift_recovery_active_ = false;

        if (reset_pid_history && had_loss_state) {
            integration_ = 0.0;
            pre_error_ = 0.0;
        }
    }

    void setDashedHoldCommand(int miss_count) {
        const int n = std::max(1, miss_count);
        twist_ = geometry_msgs::Twist();

        // HOLD只允许继续前进，不允许继承任何横移甩尾量。
        // 若最后可靠虚线PID本身低于HOLD限速，则保持较低值，不反向加速。
        const double reliable_vx = last_dashed_cmd_valid_
            ? std::max(0.0, last_dashed_vx_)
            : dashed_hold_speed_;
        twist_.linear.x = std::min(reliable_vx, dashed_hold_speed_);
        twist_.linear.y = 0.0;

        // 从最后一帧“真实虚线PID”的wz开始指数衰减。
        // miss=1时乘一次decay，随后逐帧趋近0，避免长期保持旧大转角。
        const double decay = std::pow(
            dashed_hold_angular_decay_, static_cast<double>(n));
        twist_.angular.z = last_dashed_cmd_valid_
            ? last_dashed_wz_ * decay
            : 0.0;
    }

    void resetDashedRuntimeForService() {
        // V29：回退V28“先丢线再开启虚线”的门控。
        // 每次服务开始时直接按照总开关决定虚线通道是否可用。
        dashed_runtime_enabled_ = dashed_line_enable_;
        dashed_permanently_disabled_ = false;
        resetDashedLockState();
        ROS_INFO("V29虚线识别运行时状态：%s（由dashed_line_enable直接决定）。",
                 dashed_runtime_enabled_ ? "开启" : "关闭");
    }

    void disableDashedForRestOfServiceAfterAvoidance() {
        if (!disable_dashed_after_avoidance_) {
            return;
        }
        dashed_runtime_enabled_ = false;
        dashed_permanently_disabled_ = true;
        resetDashedLockState();
        resetLineVisualPrior();
        ROS_WARN("三段避障已完成：按配置永久关闭本次lineo_right服务后续虚线识别，只保留原实线巡线。此状态持续到下一次服务调用；下次将重新按dashed_line_enable初始化。");
    }

    bool dashedRuntimeEnabled() const {
        return dashed_line_enable_ && dashed_runtime_enabled_ &&
               !dashed_permanently_disabled_;
    }

    void updateLineVisualPrior(const RaceTrack& track) {
        if (track.points.size() < 8) {
            return;
        }

        Vec4f line;
        fitLine(track.points, line, DIST_L2, 0, 0.01, 0.01);
        const double vx = line[0];
        const double vy = line[1];
        if (!std::isfinite(vx) || !std::isfinite(vy) ||
            std::abs(vy) < 0.05) {
            return;
        }

        line_prior_a_ = vx / vy;
        line_prior_b_ = line[2] - line_prior_a_ * line[3];
        line_prior_angle_rad_ = normalizeLineAngle(std::atan2(vy, vx));
        line_prior_valid_ = std::isfinite(line_prior_a_) &&
                            std::isfinite(line_prior_b_);
    }

    vector<DashedComponent> collectDashedComponents(
        const Mat& dashed_binary,
        Mat& visual_img) {
        vector<DashedComponent> components;
        if (dashed_binary.empty()) {
            return components;
        }

        if (dashed_debug_draw_ && dashed_use_expected_corridor_) {
            vector<Point> corridor_left;
            vector<Point> corridor_right;
            for (int y = dashed_min_y_; y < dashed_binary.rows; y += 6) {
                const double expected_x = expectedRightLineX(y);
                const int left_x = cvRound(clamp(
                    std::max(static_cast<double>(dashed_min_x_),
                             expected_x - dashed_corridor_left_margin_px_),
                    0.0, 639.0));
                const int right_x = cvRound(clamp(
                    expected_x + dashed_corridor_right_margin_px_,
                    0.0, 639.0));
                corridor_left.emplace_back(left_x, y);
                corridor_right.emplace_back(right_x, y);
            }
            if (corridor_left.size() >= 2) {
                for (size_t i = 1; i < corridor_left.size(); ++i) {
                    cv::line(
                        visual_img, corridor_left[i - 1], corridor_left[i],
                        Scalar(255, 255, 0), 1);
                    cv::line(
                        visual_img, corridor_right[i - 1], corridor_right[i],
                        Scalar(255, 255, 0), 1);
                }
            }
        }

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        Mat contour_input = dashed_binary.clone();
        // CHAIN_APPROX_NONE保留每一行的轮廓像素，便于准确抽取白块左边缘。
        findContours(
            contour_input, contours, hierarchy,
            RETR_EXTERNAL, CHAIN_APPROX_NONE);

        for (size_t i = 0; i < contours.size(); ++i) {
            const double area = contourArea(contours[i]);
            if (area < dashed_min_contour_area_ ||
                area > dashed_max_contour_area_) {
                continue;
            }

            const Rect box = boundingRect(contours[i]);
            if (box.width < dashed_min_component_width_ ||
                box.height < dashed_min_component_height_ ||
                box.width > dashed_max_component_width_ ||
                box.height > dashed_max_component_height_) {
                continue;
            }
            if (box.y + box.height - 1 < dashed_min_y_) {
                continue;
            }

            vector<int> left_x(dashed_binary.rows, dashed_binary.cols + 1);
            vector<unsigned char> seen(dashed_binary.rows, 0);
            for (const Point& p : contours[i]) {
                if (p.y < 0 || p.y >= dashed_binary.rows ||
                    p.x < 0 || p.x >= dashed_binary.cols) {
                    continue;
                }
                if (!seen[p.y] || p.x < left_x[p.y]) {
                    left_x[p.y] = p.x;
                    seen[p.y] = 1;
                }
            }

            DashedComponent component;
            component.bbox = box;
            component.area = area;
            double sum_x = 0.0;
            double sum_y = 0.0;
            for (int y = std::max(box.y, dashed_min_y_);
                 y < box.y + box.height && y < dashed_binary.rows; ++y) {
                if (!seen[y]) {
                    continue;
                }
                const int x = left_x[y];
                component.left_edge_points.emplace_back(x, y);
                sum_x += x;
                sum_y += y;
            }

            if (component.left_edge_points.size() < 3) {
                continue;
            }
            component.center.x = sum_x /
                static_cast<double>(component.left_edge_points.size());
            component.center.y = sum_y /
                static_cast<double>(component.left_edge_points.size());

            // V22：不是只做一个固定min_x，而是进一步要求白块中心落在
            // expectedRightLineX(y)附近的动态走廊内。越靠近车（y越大），
            // 允许区域自然越靠右，因此中央大面积反光通常在这里就被彻底删除。
            if (!isPointInsideExpectedRightCorridor(
                    component.center.x, component.center.y)) {
                continue;
            }

            if (dashed_debug_draw_) {
                rectangle(visual_img, box, Scalar(255, 255, 0), 1);
                circle(
                    visual_img,
                    Point(cvRound(component.center.x), cvRound(component.center.y)),
                    3, Scalar(255, 255, 0), -1);
            }
            components.push_back(component);
        }
        return components;
    }

    double computeDashedPatternScore(
        const vector<DashedComponent>& components,
        const vector<int>& indices) const {
        if (indices.size() < 2) {
            return 0.0;
        }

        vector<int> ordered = indices;
        std::sort(
            ordered.begin(), ordered.end(),
            [&components](int lhs, int rhs) {
                return components[lhs].center.y < components[rhs].center.y;
            });

        double score_sum = 0.0;
        int valid_pairs = 0;
        for (size_t i = 1; i < ordered.size(); ++i) {
            const DashedComponent& first = components[ordered[i - 1]];
            const DashedComponent& second = components[ordered[i]];

            const double center_spacing = std::hypot(
                second.center.x - first.center.x,
                second.center.y - first.center.y);
            const double first_size = std::hypot(
                static_cast<double>(first.bbox.width),
                static_cast<double>(first.bbox.height));
            const double second_size = std::hypot(
                static_cast<double>(second.bbox.width),
                static_cast<double>(second.bbox.height));
            const double local_dash_size = std::max(
                2.0, 0.5 * (first_size + second_size));
            const double ratio = center_spacing / local_dash_size;

            // 真实1cm白/1cm蓝的相邻白块中心间距通常与白块自身长度同量级，
            // 理想ratio约在2附近。透视、白线宽度、漏掉一段都会使它变化，
            // 因此这里只做非常宽松的软评分，绝不作为硬门槛。
            double pair_score = 0.0;
            if (ratio >= 0.8 && ratio <= 6.0) {
                const double log_error = std::abs(std::log(
                    std::max(1e-6, ratio / 2.0)));
                pair_score = std::max(
                    0.0,
                    1.0 - log_error / std::log(4.0));
            }
            score_sum += pair_score;
            ++valid_pairs;
        }

        return valid_pairs > 0
            ? score_sum / static_cast<double>(valid_pairs)
            : 0.0;
    }

    bool selectDashedComponentSet(
        const vector<DashedComponent>& components,
        bool enforce_prior,
        double prior_x_tolerance_px,
        double prior_angle_tolerance_deg,
        vector<int>& best_indices,
        int required_count_override = 0) {
        best_indices.clear();
        if (components.size() < 2) {
            return false;
        }

        const int required_count = required_count_override > 0
            ? required_count_override
            : (enforce_prior
                ? dashed_min_component_count_
                : dashed_min_component_count_without_prior_);
        double best_score = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < components.size(); ++i) {
            for (size_t j = i + 1; j < components.size(); ++j) {
                const double dy = components[j].center.y - components[i].center.y;
                if (std::abs(dy) < 6.0) {
                    continue;
                }

                // 用两个白块中心建立x=a*y+b候选，再统计其它白块是否共线。
                const double a =
                    (components[j].center.x - components[i].center.x) / dy;
                const double b = components[i].center.x -
                    a * components[i].center.y;
                if (!std::isfinite(a) || !std::isfinite(b)) {
                    continue;
                }

                vector<int> inliers;
                double residual_sum = 0.0;
                int min_y = 100000;
                int max_y = -100000;
                double center_x_sum = 0.0;
                for (size_t k = 0; k < components.size(); ++k) {
                    const double predicted_x = a * components[k].center.y + b;
                    const double residual = std::abs(
                        components[k].center.x - predicted_x);
                    if (residual <= dashed_component_line_tolerance_px_) {
                        inliers.push_back(static_cast<int>(k));
                        residual_sum += residual;
                        min_y = std::min(min_y, components[k].bbox.y);
                        max_y = std::max(
                            max_y,
                            components[k].bbox.y + components[k].bbox.height - 1);
                        center_x_sum += components[k].center.x;
                    }
                }

                if (static_cast<int>(inliers.size()) < required_count) {
                    continue;
                }
                const int span = max_y - min_y;
                if (span < dashed_min_y_span_) {
                    continue;
                }

                const double ref_y = 0.5 * (min_y + max_y);
                const double candidate_x = a * ref_y + b;
                const double candidate_angle = normalizeLineAngle(
                    std::atan2(1.0, a));

                double prior_x_error = 0.0;
                double prior_angle_error_deg = 0.0;
                if (enforce_prior && dashed_use_prior_ && line_prior_valid_) {
                    const double prior_x =
                        line_prior_a_ * ref_y + line_prior_b_;
                    prior_x_error = std::abs(candidate_x - prior_x);
                    if (prior_x_error > prior_x_tolerance_px) {
                        continue;
                    }

                    prior_angle_error_deg = lineAngleDifference(
                        candidate_angle, line_prior_angle_rad_) *
                        180.0 / 3.14159265358979323846;
                    if (prior_angle_error_deg > prior_angle_tolerance_deg) {
                        continue;
                    }
                }

                const int count = static_cast<int>(inliers.size());
                const double mean_residual = residual_sum /
                    static_cast<double>(std::max(1, count));
                const double mean_x = center_x_sum /
                    static_cast<double>(std::max(1, count));
                const double pattern_score =
                    computeDashedPatternScore(components, inliers);

                // V22不再只在“完全并列”时选更靠右，而是显式加入靠右得分。
                // 但白块数量仍是最大项：多一个真实虚线段通常比单纯右移几十像素更重要。
                // 有先验时再对位置/角度偏离进行连续惩罚，减少候选在相邻帧间跳线。
                const double score =
                    static_cast<double>(count) * 120.0 +
                    static_cast<double>(span) * 0.60 -
                    mean_residual * 7.0 +
                    mean_x * dashed_right_bias_weight_ +
                    pattern_score * dashed_pattern_score_weight_ -
                    prior_x_error * 0.60 -
                    prior_angle_error_deg * 1.20;

                if (score > best_score) {
                    best_score = score;
                    best_indices = inliers;
                }
            }
        }
        return !best_indices.empty();
    }

    bool traceDashedRightEdge(
        const Mat& dashed_binary,
        RaceTrack& racetrack,
        Mat& visual_img,
        bool strict_lock_gate) {
        racetrack = RaceTrack();
        vector<DashedComponent> components =
            collectDashedComponents(dashed_binary, visual_img);

        if (components.size() < 2) {
            return false;
        }

        vector<int> inlier_indices;
        bool used_prior_gate = false;

        if (strict_lock_gate) {
            // DASHED_LOCK中必须继续跟“上一帧同一条线”。即使画面右边另有
            // 更亮/更靠右的反光，也不允许无先验重新选线。
            if (!dashed_use_prior_ || !line_prior_valid_) {
                return false;
            }
            used_prior_gate = selectDashedComponentSet(
                components, true,
                dashed_lock_x_tolerance_px_,
                dashed_lock_angle_tolerance_deg_,
                inlier_indices,
                dashed_min_component_count_);
            if (!used_prior_gate) {
                return false;
            }
        } else {
            if (dashed_use_prior_ && line_prior_valid_) {
                // V25：2段只能给已存在的DASHED_LOCK续锁；建立新虚线即使有prior也至少要求3段。
                used_prior_gate = selectDashedComponentSet(
                    components, true,
                    dashed_prior_x_tolerance_px_,
                    dashed_prior_angle_tolerance_deg_,
                    inlier_indices,
                    dashed_min_component_count_without_prior_);
            }
            // 非锁定阶段允许至少3个白块自行重建，目的是第一次进入虚线区时
            // 能从零建立轨迹；动态右侧走廊和靠右打分负责抑制中央反光。
            if (inlier_indices.empty()) {
                used_prior_gate = false;
                if (!selectDashedComponentSet(
                        components, false,
                        dashed_prior_x_tolerance_px_,
                        dashed_prior_angle_tolerance_deg_,
                        inlier_indices,
                        dashed_min_component_count_without_prior_)) {
                    return false;
                }
            }
        }

        vector<Point> fit_points;
        fit_points.reserve(200);
        int observed_min_y = dashed_binary.rows;
        int observed_max_y = -1;
        for (int index : inlier_indices) {
            const DashedComponent& component = components[index];
            for (const Point& p : component.left_edge_points) {
                fit_points.push_back(p);
                observed_min_y = std::min(observed_min_y, p.y);
                observed_max_y = std::max(observed_max_y, p.y);
                if (dashed_debug_draw_ && (p.y % 4 == 0)) {
                    circle(visual_img, p, 1, Scalar(255, 0, 255), -1);
                }
            }
        }
        if (fit_points.size() < 8 ||
            observed_max_y - observed_min_y < dashed_min_y_span_) {
            return false;
        }

        Vec4f line;
        fitLine(fit_points, line, DIST_HUBER, 0, 0.01, 0.01);
        const double vx = line[0];
        const double vy = line[1];
        const double x0 = line[2];
        const double y0 = line[3];
        if (!std::isfinite(vx) || !std::isfinite(vy) ||
            !std::isfinite(x0) || !std::isfinite(y0) ||
            std::abs(vy) < 0.05) {
            return false;
        }

        double squared_residual_sum = 0.0;
        for (const Point& p : fit_points) {
            // fitLine给的是单位方向向量，二维叉积绝对值即垂直距离。
            const double residual =
                -vy * (static_cast<double>(p.x) - x0) +
                 vx * (static_cast<double>(p.y) - y0);
            squared_residual_sum += residual * residual;
        }
        const double rms = std::sqrt(
            squared_residual_sum / static_cast<double>(fit_points.size()));
        if (rms > dashed_fit_max_error_px_) {
            return false;
        }

        const int virtual_y_max = std::min(
            dashed_binary.rows - 1,
            observed_max_y + dashed_bottom_extrapolation_px_);
        const int virtual_y_min = std::max(
            dashed_min_y_,
            observed_min_y - dashed_top_extrapolation_px_);
        if (virtual_y_max - virtual_y_min < dashed_min_y_span_) {
            return false;
        }

        const int total_span = virtual_y_max - virtual_y_min;
        const int y_step = std::max(
            1,
            static_cast<int>(std::ceil(
                static_cast<double>(total_span + 1) /
                static_cast<double>(dashed_virtual_max_points_))));

        racetrack.points.clear();
        for (int y = virtual_y_max; y >= virtual_y_min; y -= y_step) {
            const double t = (static_cast<double>(y) - y0) / vy;
            const double x = x0 + t * vx;
            if (x < 1.0 || x > dashed_binary.cols - 2.0) {
                continue;
            }
            racetrack.points.emplace_back(cvRound(x), y);
            if (static_cast<int>(racetrack.points.size()) >=
                dashed_virtual_max_points_) {
                break;
            }
        }

        if (racetrack.points.size() < 15) {
            return false;
        }
        racetrack.slope = std::abs(vx) > 1e-6
            ? vy / vx
            : std::copysign(1e6, vy);
        racetrack.direction_change = 0;
        racetrack.slope_change_count = 0;
        racetrack.left_point = false;

        if (dashed_debug_draw_) {
            for (const Point& p : racetrack.points) {
                circle(visual_img, p, 2, Scalar(0, 0, 255), -1);
            }
            if (!racetrack.points.empty()) {
                cv::line(
                    visual_img,
                    racetrack.points.front(), racetrack.points.back(),
                    Scalar(0, 0, 255), 1);
            }
            ostringstream oss;
            oss << (strict_lock_gate ? "DASHED_LOCK seg=" : "DASHED seg=")
                << inlier_indices.size()
                << " rms=" << std::fixed << std::setprecision(1) << rms
                << " prior=" << (used_prior_gate ? "Y" : "N");
            putText(
                visual_img, oss.str(), Point(20, 125),
                FONT_HERSHEY_SIMPLEX, 0.45, Scalar(0, 0, 255), 1);
        }

        ROS_INFO_THROTTLE(
            0.5,
            "虚线右边线重建成功：候选白块=%zu，采用=%zu，RMS=%.2fpx，"
            "y覆盖=%d~%d，虚拟点=%zu，先验门控=%s，锁定门控=%s。",
            components.size(), inlier_indices.size(), rms,
            observed_min_y, observed_max_y, racetrack.points.size(),
            used_prior_gate ? "是" : "否",
            strict_lock_gate ? "是" : "否");
        return true;
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
            putText(visual_img, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
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

            // 根据右侧边线推算赛道中线
            double estimated_center_x =
                traced_points[i].x - center_offset;

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
                cvRound(point.x - center_offset),
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
    ros::init(argc, argv, "lineo_right");
    
    // 创建节点对象（构造函数中完成所有初始化）
    LineFollowerNode node;
    
    // 运行节点
    node.run();
    
    return 0;
}