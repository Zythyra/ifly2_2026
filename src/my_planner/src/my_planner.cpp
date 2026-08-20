// 交付构建标识：IFLY2026_C5_4_PATROL_REFERENCE_MASTER_C5_R2_20260812
// Patrol-R2：固定巡检Reference强所有权 + master costmap C5 + optimized path Patrol PP。
// 巡检Reference完全忽略GlobalPlanner/inflation几何；C5仍直接读取master costmap（含墙体inflation与真实路障）。
// 局部规划器基线：MYPLANNER_MPC_C5_4_REPLAN_POLICY_STABILIZATION_PATROL_REFERENCE_MASTER_C5_R2
// 保留控制器基线标识：MYPLANNER_MPC_C4_0_4_ORIGINAL_FINAL_POSE_CONTROL
// 保留C5.3几何优化、导航/MPC/巡检框架；修复末端重规划和过度重规划。
#include "my_planner.h"

#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <boost/thread/locks.hpp>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <clocale>
#include <cmath>
#include <limits>
#include <stdexcept>

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner
{

MyPlanner::MyPlanner()
    : initialized_(false),
      tf_listener_(NULL),
      costmap_ros_(NULL),
      enable_path_replanning_(true),
      collision_replan_confirm_counter_(0),
      c5_failure_replan_confirm_counter_(0),
      c5_failure_replan_candidate_(false),
      runtime_parameters_initialized_(false),
      patrol_path_locked_(false),
      active_plan_is_patrol_(false),
      patrol_path_revision_(0),
      applied_patrol_path_revision_(0),
      patrol_goal_position_tolerance_(0.05),
      patrol_goal_yaw_tolerance_(0.15),
      patrol_pp_enabled_(true),
      patrol_pp_align_tolerance_(1.5 * M_PI / 180.0),
      patrol_pp_settle_omega_(0.03),
      patrol_pp_settle_frames_(4),
      patrol_align_kp_(1.50),
      patrol_align_max_wz_(0.40),
      patrol_align_near_angle_(10.0 * M_PI / 180.0),
      patrol_align_near_wz_(0.18),
      patrol_heading_hold_enabled_(true),
      patrol_heading_kp_(1.50),
      patrol_heading_max_wz_(0.30),
      patrol_heading_deadband_(0.5 * M_PI / 180.0),
      patrol_heading_acc_lim_(1.50),
      patrol_pp_lookahead_dist_(0.35),
      patrol_pp_preview_start_(0.15),
      patrol_pp_preview_end_(0.90),
      patrol_pp_lateral_gain_(2.80),
      patrol_pp_lateral_deadband_(0.01),
      patrol_pp_max_vy_(0.40),
      patrol_pp_acc_lim_y_(1.60),
      patrol_pp_avoid_offset_rate_(0.50),
      patrol_pp_return_offset_rate_(0.15),
      patrol_pp_goal_slowdown_distance_(0.30),
      patrol_pp_goal_position_tolerance_(0.015),
      patrol_pp_settle_counter_(0),
      patrol_pp_filtered_offset_(0.0),
      patrol_pp_alignment_required_(false),
      has_goal_(false),
      target_index_(0),
      goal_reached_(false),
      control_state_(ControlState::WAITING_FOR_PLAN),
      mpc_consecutive_failures_(0),
      mpc_locked_to_pp_(false),
      odom_received_(false)
{
    setlocale(LC_ALL, "");
}

MyPlanner::~MyPlanner()
{
    if (tf_listener_ != NULL)
    {
        delete tf_listener_;
        tf_listener_ = NULL;
    }
}

double MyPlanner::clampValue(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}

double MyPlanner::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double MyPlanner::applyMinimumMagnitude(double value, double minimum)
{
    if (value == 0.0 || minimum <= 0.0 || std::abs(value) >= minimum)
        return value;

    return std::copysign(minimum, value);
}


double MyPlanner::filterMeasuredOmega(
    double raw_omega,
    const ros::Time& stamp)
{
    if (c3_measured_omega_filter_cutoff_hz_ <= 0.0)
        return raw_omega;

    if (!omega_filter_state_.initialized)
    {
        omega_filter_state_.initialized = true;
        omega_filter_state_.x1 = raw_omega;
        omega_filter_state_.x2 = raw_omega;
        omega_filter_state_.y1 = raw_omega;
        omega_filter_state_.y2 = raw_omega;
        omega_filter_state_.last_stamp = stamp;
        return raw_omega;
    }

    double dt = (stamp - omega_filter_state_.last_stamp).toSec();
    if (!std::isfinite(dt) || dt <= 1.0e-4 || dt > 0.50)
    {
        omega_filter_state_ = ButterworthFilterState();
        omega_filter_state_.initialized = true;
        omega_filter_state_.x1 = raw_omega;
        omega_filter_state_.x2 = raw_omega;
        omega_filter_state_.y1 = raw_omega;
        omega_filter_state_.y2 = raw_omega;
        omega_filter_state_.last_stamp = stamp;
        return raw_omega;
    }

    omega_filter_state_.last_stamp = stamp;
    const double sample_rate = 1.0 / dt;
    const double cutoff = std::min(
        c3_measured_omega_filter_cutoff_hz_, 0.45 * sample_rate);
    if (cutoff <= 1.0e-3)
        return raw_omega;

    // 标准二阶Butterworth低通，双线性变换离散化。
    const double k = std::tan(M_PI * cutoff / sample_rate);
    const double norm = 1.0 / (1.0 + std::sqrt(2.0) * k + k * k);
    const double b0 = k * k * norm;
    const double b1 = 2.0 * b0;
    const double b2 = b0;
    const double a1 = 2.0 * (k * k - 1.0) * norm;
    const double a2 = (1.0 - std::sqrt(2.0) * k + k * k) * norm;

    const double filtered = b0 * raw_omega
        + b1 * omega_filter_state_.x1
        + b2 * omega_filter_state_.x2
        - a1 * omega_filter_state_.y1
        - a2 * omega_filter_state_.y2;

    omega_filter_state_.x2 = omega_filter_state_.x1;
    omega_filter_state_.x1 = raw_omega;
    omega_filter_state_.y2 = omega_filter_state_.y1;
    omega_filter_state_.y1 = filtered;
    return filtered;
}

void MyPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& message)
{
    if (!message)
        return;

    const ros::Time stamp = message->header.stamp.isZero()
        ? ros::Time::now() : message->header.stamp;

    std::lock_guard<std::mutex> lock(measured_state_mutex_);
    measured_body_twist_ = message->twist.twist;
    measured_body_twist_.angular.z = filterMeasuredOmega(
        message->twist.twist.angular.z, stamp);
    last_odom_stamp_ = stamp;
    odom_received_ = true;
}

MyPlanner::MeasuredBodyState MyPlanner::getMeasuredBodyState(
    const ros::Time& now) const
{
    MeasuredBodyState result;
    {
        std::lock_guard<std::mutex> lock(measured_state_mutex_);
        if (c3_use_odometry_ && odom_received_)
        {
            result.age = std::max(0.0, (now - last_odom_stamp_).toSec());
            if (std::isfinite(result.age) && result.age <= c3_odom_timeout_)
            {
                result.vx = measured_body_twist_.linear.x;
                result.vy = measured_body_twist_.linear.y;
                result.omega = measured_body_twist_.angular.z;
                result.valid = std::isfinite(result.vx)
                    && std::isfinite(result.vy)
                    && std::isfinite(result.omega);
            }
        }
    }

    if (!result.valid)
    {
        result.vx = last_cmd_vel_.linear.x;
        result.vy = last_cmd_vel_.linear.y;
        result.omega = last_cmd_vel_.angular.z;
        result.age = -1.0;
    }
    return result;
}

geometry_msgs::Twist MyPlanner::commandAtTime(
    const std::deque<TimedCommand>& history,
    const ros::Time& query_time) const
{
    if (history.empty())
        return last_cmd_vel_;

    geometry_msgs::Twist result = history.front().command;
    for (std::deque<TimedCommand>::const_iterator it = history.begin();
         it != history.end(); ++it)
    {
        if (it->stamp > query_time)
            break;
        result = it->command;
    }
    return result;
}

MyPlanner::DelayCompensatedState MyPlanner::predictStateThroughInputDelay(
    const MeasuredBodyState& measured,
    const ros::Time& now) const
{
    DelayCompensatedState state;
    state.vx = measured.vx;
    state.vy = measured.vy;
    state.omega = measured.omega;

    if (c3_input_delay_ <= 1.0e-4)
        return state;

    std::deque<TimedCommand> history;
    {
        std::lock_guard<std::mutex> lock(command_history_mutex_);
        history = command_history_;
    }

    const double integration_dt = std::max(
        0.002, std::min(c3_delay_integration_dt_, c3_input_delay_));
    double elapsed = 0.0;
    while (elapsed < c3_input_delay_ - 1.0e-9)
    {
        const double dt = std::min(
            integration_dt, c3_input_delay_ - elapsed);
        const ros::Time command_time = now
            - ros::Duration(c3_input_delay_ - elapsed);
        const geometry_msgs::Twist command = commandAtTime(
            history, command_time);

        const double alpha_v = c3_translational_response_tau_ <= 1.0e-4
            ? 1.0
            : 1.0 - std::exp(-dt / c3_translational_response_tau_);
        const double alpha_omega = c3_angular_response_tau_ <= 1.0e-4
            ? 1.0
            : 1.0 - std::exp(-dt / c3_angular_response_tau_);

        state.vx += alpha_v * (command.linear.x - state.vx);
        state.vy += alpha_v * (command.linear.y - state.vy);
        state.omega += alpha_omega * (command.angular.z - state.omega);

        const double c = std::cos(state.yaw);
        const double ss = std::sin(state.yaw);
        state.x += dt * (c * state.vx - ss * state.vy);
        state.y += dt * (ss * state.vx + c * state.vy);
        state.yaw = normalizeAngle(state.yaw + dt * state.omega);
        elapsed += dt;
    }
    return state;
}

void MyPlanner::recordPublishedCommand(
    const geometry_msgs::Twist& command,
    const ros::Time& stamp)
{
    std::lock_guard<std::mutex> lock(command_history_mutex_);
    TimedCommand item;
    item.stamp = stamp;
    item.command = command;
    command_history_.push_back(item);

    const ros::Time keep_after = stamp - ros::Duration(
        std::max(1.0, c3_input_delay_ + 0.50));
    while (!command_history_.empty()
           && command_history_.front().stamp < keep_after)
    {
        command_history_.pop_front();
    }
}

void MyPlanner::initialize(std::string name,
                           tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
{
    if (initialized_)
    {
        ROS_WARN("MyPlanner 已经初始化，忽略重复初始化请求。");
        return;
    }

    if (tf == NULL || costmap_ros == NULL || costmap_ros->getCostmap() == NULL)
        throw std::runtime_error("MyPlanner 初始化失败：TF 或代价地图为空。");

    tf_listener_ = new tf::TransformListener();
    costmap_ros_ = costmap_ros;
    base_frame_ = costmap_ros_->getBaseFrameID();
    costmap_frame_ = costmap_ros_->getGlobalFrameID();

    ros::NodeHandle private_nh("~/" + name);
    private_nh_ = private_nh;
    // -------------------------------------------------------------------------
    // Stage 0.1：基础类 PP，并恢复全向底盘横移纠偏
    // -------------------------------------------------------------------------
    private_nh.param("lookahead_dist", lookahead_dist_, 0.20);
    private_nh.param("path_linear_x_gain", path_linear_x_gain_, 1.50);
    private_nh.param("path_linear_y_gain", path_linear_y_gain_, 2.00);
    private_nh.param("path_angular_y_gain", path_angular_y_gain_, 9.00);
    private_nh.param("lateral_search_points", lateral_search_points_, 10);

    // -------------------------------------------------------------------------
    // 保留二代车现有的非累积路径治愈
    // -------------------------------------------------------------------------
    private_nh.param("enable_path_healing", enable_path_healing_, false);
    private_nh.param("path_healing_points_behind",
                     path_healing_points_behind_, 5);
    private_nh.param("path_healing_points_ahead",
                     path_healing_points_ahead_, 60);
    private_nh.param("path_healing_iterations",
                     path_healing_iterations_, 3);
    private_nh.param("path_healing_max_step",
                     path_healing_max_step_, 0.01);
    private_nh.param("path_healing_gradient_deadband",
                     path_healing_gradient_deadband_, 4.0);
    private_nh.param("path_healing_gradient_scale",
                     path_healing_gradient_scale_, 20.0);
    private_nh.param("publish_path_healing_debug",
                     publish_path_healing_debug_, true);
    private_nh.param("path_healing_debug_publish_rate",
                     path_healing_debug_publish_rate_, 5.0);

    // -------------------------------------------------------------------------
    // 保留前方全局路径碰撞检查；命中障碍后可返回false触发全局重规划。
    // enable_path_replanning支持运行时修改，默认开启。
    // -------------------------------------------------------------------------
    private_nh.param("enable_path_replanning",
                     enable_path_replanning_, true);
    private_nh.param("patrol_goal_position_tolerance",
                     patrol_goal_position_tolerance_, 0.05);
    private_nh.param("patrol_goal_yaw_tolerance",
                     patrol_goal_yaw_tolerance_, 0.15);

    // 固定巡检Path专用全向PP。普通导航和停靠仍由controller_mode决定，
    // 这些参数只在“固定路径已锁定且当前setPlan确实套用了该路径”时生效。
    private_nh.param("patrol_pp_enabled", patrol_pp_enabled_, true);
    double patrol_pp_align_tolerance_deg = 1.5;
    private_nh.param("patrol_pp_align_tolerance_deg",
                     patrol_pp_align_tolerance_deg, 1.5);
    private_nh.param("patrol_pp_settle_omega",
                     patrol_pp_settle_omega_, 0.03);
    private_nh.param("patrol_pp_settle_frames",
                     patrol_pp_settle_frames_, 4);
    private_nh.param("patrol_align_kp",
                     patrol_align_kp_, 1.50);
    private_nh.param("patrol_align_max_wz",
                     patrol_align_max_wz_, 0.40);
    double patrol_align_near_angle_deg = 10.0;
    private_nh.param("patrol_align_near_angle_deg",
                     patrol_align_near_angle_deg, 10.0);
    private_nh.param("patrol_align_near_wz",
                     patrol_align_near_wz_, 0.18);
    private_nh.param("patrol_heading_hold_enabled",
                     patrol_heading_hold_enabled_, true);
    private_nh.param("patrol_heading_kp",
                     patrol_heading_kp_, 1.50);
    private_nh.param("patrol_heading_max_wz",
                     patrol_heading_max_wz_, 0.30);
    double patrol_heading_deadband_deg = 0.5;
    private_nh.param("patrol_heading_deadband_deg",
                     patrol_heading_deadband_deg, 0.5);
    private_nh.param("patrol_heading_acc_lim",
                     patrol_heading_acc_lim_, 1.50);
    private_nh.param("patrol_pp_lookahead_dist",
                     patrol_pp_lookahead_dist_, 0.35);
    private_nh.param("patrol_pp_preview_start",
                     patrol_pp_preview_start_, 0.15);
    private_nh.param("patrol_pp_preview_end",
                     patrol_pp_preview_end_, 0.90);
    private_nh.param("patrol_pp_lateral_gain",
                     patrol_pp_lateral_gain_, 2.80);
    private_nh.param("patrol_pp_lateral_deadband",
                     patrol_pp_lateral_deadband_, 0.01);
    private_nh.param("patrol_pp_max_vy",
                     patrol_pp_max_vy_, 0.40);
    private_nh.param("patrol_pp_acc_lim_y",
                     patrol_pp_acc_lim_y_, 1.60);
    private_nh.param("patrol_pp_avoid_offset_rate",
                     patrol_pp_avoid_offset_rate_, 0.50);
    private_nh.param("patrol_pp_return_offset_rate",
                     patrol_pp_return_offset_rate_, 0.15);
    private_nh.param("patrol_pp_goal_slowdown_distance",
                     patrol_pp_goal_slowdown_distance_, 0.30);
    private_nh.param("patrol_pp_goal_position_tolerance",
                     patrol_pp_goal_position_tolerance_, 0.015);
    // C5.4：全局重规划与C5 active path继续解耦。
    // raw path仅以真正LETHAL(254)为默认拓扑重规划触发；253继续交给C5局部优化。
    private_nh.param("collision_check_lookahead_distance",
                     collision_check_lookahead_distance_, 0.60);
    private_nh.param("collision_replan_confirm_frames",
                     collision_replan_confirm_frames_, 2);
    private_nh.param("collision_replan_on_unknown",
                     collision_replan_on_unknown_, false);
    private_nh.param("c5_failure_replan_confirm_frames",
                     c5_failure_replan_confirm_frames_, 3);

    int collision_cost_threshold = 254;
    private_nh.param("collision_cost_threshold",
                     collision_cost_threshold, 254);
    collision_cost_threshold =
        std::max(1, std::min(collision_cost_threshold, 255));
    collision_cost_threshold_ =
        static_cast<unsigned char>(collision_cost_threshold);

    // -------------------------------------------------------------------------
    // 条件式初始姿态对准。阈值使用路径切线与车头的夹角，并带滞回。
    // -------------------------------------------------------------------------
    private_nh.param("enable_initial_rotation",
                     enable_initial_rotation_, true);
    double initial_align_enter_angle_deg = 30.0;
    double initial_align_exit_angle_deg = 22.0;
    private_nh.param("initial_align_enter_angle_deg",
                     initial_align_enter_angle_deg, 30.0);
    private_nh.param("initial_align_exit_angle_deg",
                     initial_align_exit_angle_deg, 22.0);
    private_nh.param("initial_path_tangent_lookahead",
                     initial_path_tangent_lookahead_, 0.16);
    private_nh.param("initial_angular_gain",
                     initial_angular_gain_, 2.00);
    private_nh.param("initial_min_angular_speed",
                     initial_min_angular_speed_, 0.10);
    private_nh.param("initial_max_angular_speed",
                     initial_max_angular_speed_, 0.30);

    // -------------------------------------------------------------------------
    // C3.2原始终点位姿调整：进入阈值后立即由独立XYZ比例控制接管。
    // -------------------------------------------------------------------------
    private_nh.param("goal_dist_threshold",
                     goal_dist_threshold_, 0.15);
    private_nh.param("goal_position_tolerance",
                     goal_position_tolerance_, 0.025);
    private_nh.param("goal_yaw_tolerance",
                     goal_yaw_tolerance_, 0.05);
    private_nh.param("final_linear_x_gain",
                     final_linear_x_gain_, 0.80);
    private_nh.param("final_linear_y_gain",
                     final_linear_y_gain_, 0.80);
    private_nh.param("final_angular_gain",
                     final_angular_gain_, 1.50);
    private_nh.param("final_min_linear_speed",
                     final_min_linear_speed_, 0.03);
    private_nh.param("final_min_angular_speed",
                     final_min_angular_speed_, 0.08);
    private_nh.param("final_max_vel_x",
                     final_max_vel_x_, 0.20);
    private_nh.param("final_max_vel_y",
                     final_max_vel_y_, 0.10);
    private_nh.param("final_max_vel_theta",
                     final_max_vel_theta_, 0.90);

    // -------------------------------------------------------------------------
    // 最外层实车速度和加速度保护
    // -------------------------------------------------------------------------
    private_nh.param("max_vel_x", max_vel_x_, 0.60);
    private_nh.param("max_vel_y", max_vel_y_, 0.30);
    private_nh.param("max_vel_theta", max_vel_theta_, 2.50);
    private_nh.param("acc_lim_x", acc_lim_x_, 2.00);
    private_nh.param("acc_lim_y", acc_lim_y_, 2.00);
    private_nh.param("acc_lim_theta", acc_lim_theta_, 8.00);



    // -------------------------------------------------------------------------
    // C3：C2双曲率速度规划＋车头预瞄主动漂移参考。
    // 路径位置不平滑、不优化；运动方向chi与车头姿态psi解耦。
    // -------------------------------------------------------------------------
    private_nh.param("controller_mode", controller_mode_, std::string("mpc"));
    private_nh.param("mpc_horizon_steps", mpc_horizon_steps_, 20);
    private_nh.param("mpc_dt", mpc_dt_, 0.05);
    private_nh.param("mpc_terminal_stop_enabled",
                     mpc_terminal_stop_enabled_, false);
    private_nh.param("mpc_terminal_yaw_blend_distance",
                     mpc_terminal_yaw_blend_distance_, 0.25);
    private_nh.param("mpc_terminal_progress_fade_distance",
                     mpc_terminal_progress_fade_distance_, 0.30);
    private_nh.param("mpc_terminal_fallback_stop_distance",
                     mpc_terminal_fallback_stop_distance_, 0.20);
    private_nh.param("mpc_reference_search_behind_points", mpc_reference_search_behind_points_, 160);
    private_nh.param("mpc_reference_search_ahead_points", mpc_reference_search_ahead_points_, 1000);

    private_nh.param("c2_resample_distance", c2_resample_distance_, 0.02);
    private_nh.param("c2_duplicate_point_distance", c2_duplicate_point_distance_, 0.003);
    private_nh.param("c2_tracking_curvature_distance", c2_tracking_curvature_distance_, 0.08);
    private_nh.param("c2_speed_curvature_distance", c2_speed_curvature_distance_, 0.12);
    private_nh.param("c2_curvature_median_window", c2_curvature_median_window_, 5);
    private_nh.param("c2_curvature_preview_distance", c2_curvature_preview_distance_, 0.45);
    private_nh.param("c2_hold_speed_after_curve", c2_hold_speed_after_curve_, 0.12);
    private_nh.param("c2_max_reference_speed", c2_max_reference_speed_, 0.45);
    private_nh.param("c2_min_curve_speed", c2_min_curve_speed_, 0.30);
    private_nh.param("c2_curve_lateral_acc_limit", c2_curve_lateral_acc_limit_, 0.22);
    private_nh.param("c2_reference_acceleration", c2_reference_acceleration_, 0.90);
    private_nh.param("c2_reference_deceleration", c2_reference_deceleration_, 1.50);

    private_nh.param("c3_enable_active_drift", c3_enable_active_drift_, true);
    private_nh.param("c3_yaw_preview_distance", c3_yaw_preview_distance_, 0.10);
    private_nh.param("c3_yaw_preview_gain", c3_yaw_preview_gain_, 0.40);
    private_nh.param("c3_yaw_preview_curvature_deadband", c3_yaw_preview_curvature_deadband_, 0.25);
    private_nh.param("c3_yaw_preview_full_curvature", c3_yaw_preview_full_curvature_, 1.50);

    double c3_beta_max_low_speed_deg = 35.0;
    double c3_beta_max_mid_speed_deg = 28.0;
    double c3_beta_max_high_speed_deg = 22.0;
    double c3_beta_rate_limit_deg_per_s = 90.0;
    private_nh.param("c3_beta_max_low_speed_deg", c3_beta_max_low_speed_deg, 35.0);
    private_nh.param("c3_beta_max_mid_speed_deg", c3_beta_max_mid_speed_deg, 28.0);
    private_nh.param("c3_beta_max_high_speed_deg", c3_beta_max_high_speed_deg, 22.0);
    private_nh.param("c3_beta_low_speed_threshold", c3_beta_low_speed_threshold_, 0.35);
    private_nh.param("c3_beta_high_speed_threshold", c3_beta_high_speed_threshold_, 0.60);
    private_nh.param("c3_beta_rate_limit_deg_per_s", c3_beta_rate_limit_deg_per_s, 90.0);
    private_nh.param("c3_reference_omega_limit", c3_reference_omega_limit_, 2.40);
    private_nh.param("c3_reference_omega_accel_rate", c3_reference_omega_accel_rate_, 8.00);
    private_nh.param("c3_reference_omega_decel_rate", c3_reference_omega_decel_rate_, 14.00);
    private_nh.param("c3_reference_omega_reverse_rate", c3_reference_omega_reverse_rate_, 16.00);
    private_nh.param("c3_omega_curvature_feedforward_gain",
                     c3_omega_curvature_feedforward_gain_, 0.30);

    private_nh.param("c3_use_odometry", c3_use_odometry_, true);
    private_nh.param("c3_odom_topic", c3_odom_topic_, std::string("/odom"));
    private_nh.param("c3_odom_timeout", c3_odom_timeout_, 0.20);
    private_nh.param("c3_measured_omega_filter_cutoff_hz",
                     c3_measured_omega_filter_cutoff_hz_, 5.0);
    private_nh.param("c3_input_delay", c3_input_delay_, 0.05);
    private_nh.param("c3_translational_response_tau",
                     c3_translational_response_tau_, 0.08);
    private_nh.param("c3_angular_response_tau",
                     c3_angular_response_tau_, 0.12);
    private_nh.param("c3_delay_integration_dt",
                     c3_delay_integration_dt_, 0.01);

    const double deg_to_rad = M_PI / 180.0;
    c3_beta_max_low_speed_ = c3_beta_max_low_speed_deg * deg_to_rad;
    c3_beta_max_mid_speed_ = c3_beta_max_mid_speed_deg * deg_to_rad;
    c3_beta_max_high_speed_ = c3_beta_max_high_speed_deg * deg_to_rad;
    c3_beta_rate_limit_ = c3_beta_rate_limit_deg_per_s * deg_to_rad;
    initial_align_enter_angle_ = initial_align_enter_angle_deg * deg_to_rad;
    initial_align_exit_angle_ = initial_align_exit_angle_deg * deg_to_rad;
    patrol_pp_align_tolerance_ =
        patrol_pp_align_tolerance_deg * deg_to_rad;
    patrol_align_near_angle_ =
        patrol_align_near_angle_deg * deg_to_rad;
    patrol_heading_deadband_ =
        patrol_heading_deadband_deg * deg_to_rad;

    private_nh.param("mpc_weight_longitudinal", mpc_weight_longitudinal_, 3.0);
    private_nh.param("mpc_weight_lateral", mpc_weight_lateral_, 110.0);
    private_nh.param("mpc_weight_yaw_straight", mpc_weight_yaw_straight_, 12.0);
    private_nh.param("mpc_weight_yaw_curve", mpc_weight_yaw_curve_, 8.0);
    private_nh.param("mpc_weight_omega_state_straight",
                     mpc_weight_omega_state_straight_, 8.0);
    private_nh.param("mpc_weight_omega_state_curve",
                     mpc_weight_omega_state_curve_, 0.40);
    private_nh.param("mpc_terminal_position_weight_scale",
                     mpc_terminal_position_weight_scale_, 4.0);
    private_nh.param("mpc_terminal_yaw_weight_scale",
                     mpc_terminal_yaw_weight_scale_, 4.0);
    private_nh.param("mpc_terminal_omega_weight_scale",
                     mpc_terminal_omega_weight_scale_, 8.0);
    private_nh.param("mpc_weight_tangent_velocity", mpc_weight_tangent_velocity_, 4.0);
    private_nh.param("mpc_weight_path_normal_velocity", mpc_weight_path_normal_velocity_, 35.0);
    private_nh.param("mpc_weight_progress", mpc_weight_progress_, 0.80);

    private_nh.param("mpc_weight_vx", mpc_weight_vx_, 0.20);
    private_nh.param("mpc_weight_vy", mpc_weight_vy_, 0.20);
    private_nh.param("mpc_weight_omega_straight",
                     mpc_weight_omega_straight_, 0.90);
    private_nh.param("mpc_weight_omega_curve",
                     mpc_weight_omega_curve_, 0.15);
    private_nh.param("mpc_weight_delta_vx", mpc_weight_delta_vx_, 10.0);
    private_nh.param("mpc_weight_delta_vy", mpc_weight_delta_vy_, 12.0);
    private_nh.param("mpc_weight_delta_omega", mpc_weight_delta_omega_, 8.0);

    private_nh.param("mpc_min_vx_ratio", mpc_min_vx_ratio_, 0.50);
    private_nh.param("mpc_min_vx", mpc_min_vx_, 0.0);
    private_nh.param("mpc_max_vx", mpc_max_vx_, 0.45);
    private_nh.param("mpc_min_vy", mpc_min_vy_, -0.30);
    private_nh.param("mpc_max_vy", mpc_max_vy_, 0.30);
    private_nh.param("mpc_min_omega", mpc_min_omega_, -2.50);
    private_nh.param("mpc_max_omega", mpc_max_omega_, 2.50);
    private_nh.param("mpc_max_translational_speed", mpc_max_translational_speed_, 0.70);
    private_nh.param("mpc_velocity_polygon_sides", mpc_velocity_polygon_sides_, 16);

    private_nh.param("mpc_max_accel_x", mpc_max_accel_x_, 2.00);
    private_nh.param("mpc_max_decel_x", mpc_max_decel_x_, 2.00);
    private_nh.param("mpc_max_accel_y", mpc_max_accel_y_, 2.00);
    private_nh.param("mpc_max_accel_theta", mpc_max_accel_theta_, 8.00);

    private_nh.param("mpc_osqp_max_iterations", mpc_osqp_max_iterations_, 180);
    private_nh.param("mpc_osqp_eps_abs", mpc_osqp_eps_abs_, 0.001);
    private_nh.param("mpc_osqp_eps_rel", mpc_osqp_eps_rel_, 0.001);
    private_nh.param("mpc_osqp_polish", mpc_osqp_polish_, false);
    private_nh.param("mpc_osqp_verbose", mpc_osqp_verbose_, false);
    private_nh.param("mpc_max_total_time_ms", mpc_max_total_time_ms_, 15.0);
    private_nh.param("mpc_max_consecutive_failures", mpc_max_consecutive_failures_, 5);
    private_nh.param("mpc_lock_to_pp_after_failures", mpc_lock_to_pp_after_failures_, true);
    private_nh.param("mpc_publish_debug_paths", mpc_publish_debug_paths_, true);

    private_nh.param("debug_log", debug_log_, true);

    lookahead_dist_ = std::max(0.01, lookahead_dist_);
    path_healing_points_behind_ =
        std::max(0, path_healing_points_behind_);
    path_healing_points_ahead_ =
        std::max(1, path_healing_points_ahead_);
    path_healing_iterations_ =
        std::max(0, path_healing_iterations_);
    path_healing_gradient_scale_ =
        std::max(1e-6, path_healing_gradient_scale_);
    path_healing_debug_publish_rate_ = clampValue(
        path_healing_debug_publish_rate_, 0.2, 30.0);
    collision_check_lookahead_distance_ =
        std::max(0.05, collision_check_lookahead_distance_);
    collision_replan_confirm_frames_ =
        std::max(1, collision_replan_confirm_frames_);
    c5_failure_replan_confirm_frames_ =
        std::max(1, c5_failure_replan_confirm_frames_);
    collision_replan_confirm_counter_ = 0;
    c5_failure_replan_confirm_counter_ = 0;
    c5_failure_replan_candidate_ = false;
    c5_failure_replan_reason_.clear();
    lateral_search_points_ = std::max(0, lateral_search_points_);

    if (controller_mode_ != "mpc" && controller_mode_ != "pp")
    {
        ROS_WARN("未知controller_mode=%s，自动使用mpc。", controller_mode_.c_str());
        controller_mode_ = "mpc";
    }

    mpc_horizon_steps_ = std::max(5, mpc_horizon_steps_);
    mpc_dt_ = clampValue(mpc_dt_, 0.02, 0.20);
    mpc_terminal_yaw_blend_distance_ = std::max(
        c2_resample_distance_, mpc_terminal_yaw_blend_distance_);
    mpc_terminal_progress_fade_distance_ = std::max(
        c2_resample_distance_, mpc_terminal_progress_fade_distance_);
    mpc_terminal_fallback_stop_distance_ = std::max(
        0.0, mpc_terminal_fallback_stop_distance_);
    mpc_reference_search_behind_points_ = std::max(1, mpc_reference_search_behind_points_);
    mpc_reference_search_ahead_points_ = std::max(20, mpc_reference_search_ahead_points_);

    c2_resample_distance_ = clampValue(c2_resample_distance_, 0.005, 0.10);
    c2_duplicate_point_distance_ = clampValue(
        c2_duplicate_point_distance_, 1e-5, 0.5 * c2_resample_distance_);
    c2_tracking_curvature_distance_ = std::max(
        c2_resample_distance_, c2_tracking_curvature_distance_);
    c2_speed_curvature_distance_ = std::max(
        c2_tracking_curvature_distance_, c2_speed_curvature_distance_);
    c2_curvature_median_window_ = std::max(1, c2_curvature_median_window_);
    if (c2_curvature_median_window_ % 2 == 0)
        ++c2_curvature_median_window_;
    c2_curvature_preview_distance_ = std::max(
        c2_resample_distance_, c2_curvature_preview_distance_);
    c2_hold_speed_after_curve_ = std::max(0.0, c2_hold_speed_after_curve_);
    c2_max_reference_speed_ = clampValue(
        c2_max_reference_speed_, 0.05, std::max(0.05, max_vel_x_));
    c2_min_curve_speed_ = clampValue(
        c2_min_curve_speed_, 0.02, c2_max_reference_speed_);
    c2_curve_lateral_acc_limit_ = std::max(1e-4, c2_curve_lateral_acc_limit_);
    c2_reference_acceleration_ = std::max(0.01, c2_reference_acceleration_);
    c2_reference_deceleration_ = std::max(0.01, c2_reference_deceleration_);
    c3_yaw_preview_distance_ = std::max(c2_resample_distance_, c3_yaw_preview_distance_);
    c3_yaw_preview_gain_ = clampValue(c3_yaw_preview_gain_, 0.0, 1.5);
    c3_yaw_preview_curvature_deadband_ = std::max(0.0, c3_yaw_preview_curvature_deadband_);
    c3_yaw_preview_full_curvature_ = std::max(
        c3_yaw_preview_curvature_deadband_ + 1e-3,
        c3_yaw_preview_full_curvature_);
    c3_beta_max_low_speed_ = clampValue(c3_beta_max_low_speed_, 0.0, 1.2);
    c3_beta_max_mid_speed_ = clampValue(c3_beta_max_mid_speed_, 0.0, c3_beta_max_low_speed_);
    c3_beta_max_high_speed_ = clampValue(c3_beta_max_high_speed_, 0.0, c3_beta_max_mid_speed_);
    c3_beta_low_speed_threshold_ = std::max(0.01, c3_beta_low_speed_threshold_);
    c3_beta_high_speed_threshold_ = std::max(
        c3_beta_low_speed_threshold_ + 0.01,
        c3_beta_high_speed_threshold_);
    c3_beta_rate_limit_ = std::max(0.01, c3_beta_rate_limit_);
    c3_reference_omega_limit_ = std::max(0.05, c3_reference_omega_limit_);
    c3_reference_omega_accel_rate_ = std::max(0.05, c3_reference_omega_accel_rate_);
    c3_reference_omega_decel_rate_ = std::max(
        c3_reference_omega_accel_rate_, c3_reference_omega_decel_rate_);
    c3_reference_omega_reverse_rate_ = std::max(
        c3_reference_omega_decel_rate_, c3_reference_omega_reverse_rate_);
    c3_omega_curvature_feedforward_gain_ = clampValue(
        c3_omega_curvature_feedforward_gain_, 0.0, 1.0);
    c3_odom_timeout_ = std::max(0.02, c3_odom_timeout_);
    c3_measured_omega_filter_cutoff_hz_ = std::max(
        0.0, c3_measured_omega_filter_cutoff_hz_);
    c3_input_delay_ = clampValue(c3_input_delay_, 0.0, 0.50);
    c3_translational_response_tau_ = std::max(0.0, c3_translational_response_tau_);
    c3_angular_response_tau_ = std::max(0.01, c3_angular_response_tau_);
    c3_delay_integration_dt_ = clampValue(c3_delay_integration_dt_, 0.002, 0.05);
    mpc_terminal_position_weight_scale_ = std::max(1.0, mpc_terminal_position_weight_scale_);
    mpc_terminal_yaw_weight_scale_ = std::max(1.0, mpc_terminal_yaw_weight_scale_);
    mpc_terminal_omega_weight_scale_ = std::max(1.0, mpc_terminal_omega_weight_scale_);
    mpc_weight_yaw_straight_ = std::max(0.0, mpc_weight_yaw_straight_);
    mpc_weight_yaw_curve_ = std::max(0.0, mpc_weight_yaw_curve_);
    mpc_weight_tangent_velocity_ = std::max(0.0, mpc_weight_tangent_velocity_);
    mpc_weight_path_normal_velocity_ = std::max(0.0, mpc_weight_path_normal_velocity_);
    mpc_weight_progress_ = std::max(0.0, mpc_weight_progress_);
    mpc_weight_omega_state_straight_ = std::max(0.0, mpc_weight_omega_state_straight_);
    mpc_weight_omega_state_curve_ = std::max(0.0, mpc_weight_omega_state_curve_);
    mpc_weight_omega_straight_ = std::max(0.0, mpc_weight_omega_straight_);
    mpc_weight_omega_curve_ = std::max(0.0, mpc_weight_omega_curve_);
    mpc_min_vx_ratio_ = clampValue(mpc_min_vx_ratio_, 0.0, 1.0);
    mpc_max_vx_ = std::max(mpc_min_vx_, mpc_max_vx_);
    mpc_max_vy_ = std::max(mpc_min_vy_, mpc_max_vy_);
    mpc_max_omega_ = std::max(mpc_min_omega_, mpc_max_omega_);
    mpc_max_translational_speed_ = std::max(
        0.05, std::min(mpc_max_translational_speed_, std::hypot(mpc_max_vx_, std::max(std::abs(mpc_min_vy_), std::abs(mpc_max_vy_)))));
    mpc_velocity_polygon_sides_ = std::max(8, mpc_velocity_polygon_sides_);
    mpc_max_accel_x_ = std::max(0.01, mpc_max_accel_x_);
    mpc_max_decel_x_ = std::max(0.01, mpc_max_decel_x_);
    mpc_max_accel_y_ = std::max(0.01, mpc_max_accel_y_);
    mpc_max_accel_theta_ = std::max(0.01, mpc_max_accel_theta_);
    mpc_osqp_max_iterations_ = std::max(20, mpc_osqp_max_iterations_);
    mpc_max_total_time_ms_ = std::max(1.0, mpc_max_total_time_ms_);
    mpc_max_consecutive_failures_ = std::max(1, mpc_max_consecutive_failures_);

    initial_align_enter_angle_ = clampValue(
        initial_align_enter_angle_, 0.05, M_PI);
    initial_align_exit_angle_ = clampValue(
        initial_align_exit_angle_, 0.02, initial_align_enter_angle_);
    initial_path_tangent_lookahead_ = std::max(
        0.02, initial_path_tangent_lookahead_);
    goal_dist_threshold_ = std::max(
        goal_position_tolerance_, goal_dist_threshold_);
    patrol_goal_position_tolerance_ = std::max(
        0.005, patrol_goal_position_tolerance_);
    patrol_goal_yaw_tolerance_ = clampValue(
        patrol_goal_yaw_tolerance_, 0.01, M_PI);
    patrol_pp_align_tolerance_ = clampValue(
        patrol_pp_align_tolerance_, 0.5 * deg_to_rad, 10.0 * deg_to_rad);
    patrol_pp_settle_omega_ = clampValue(
        patrol_pp_settle_omega_, 0.005, 0.50);
    patrol_pp_settle_frames_ = std::max(1, patrol_pp_settle_frames_);
    patrol_align_kp_ = std::max(0.01, patrol_align_kp_);
    patrol_align_max_wz_ = clampValue(
        std::abs(patrol_align_max_wz_), 0.01, max_vel_theta_);
    patrol_align_near_angle_ = clampValue(
        patrol_align_near_angle_, patrol_pp_align_tolerance_, 30.0 * deg_to_rad);
    patrol_align_near_wz_ = clampValue(
        std::abs(patrol_align_near_wz_), 0.005, patrol_align_max_wz_);
    patrol_heading_kp_ = std::max(0.0, patrol_heading_kp_);
    patrol_heading_max_wz_ = clampValue(
        std::abs(patrol_heading_max_wz_), 0.0, max_vel_theta_);
    patrol_heading_deadband_ = clampValue(
        patrol_heading_deadband_, 0.0, patrol_pp_align_tolerance_);
    patrol_heading_acc_lim_ = std::max(0.01, patrol_heading_acc_lim_);
    patrol_pp_lookahead_dist_ = std::max(0.05, patrol_pp_lookahead_dist_);
    patrol_pp_preview_start_ = std::max(0.0, patrol_pp_preview_start_);
    patrol_pp_preview_end_ = std::max(
        patrol_pp_preview_start_ + c2_resample_distance_,
        patrol_pp_preview_end_);
    patrol_pp_lateral_gain_ = std::max(0.0, patrol_pp_lateral_gain_);
    patrol_pp_lateral_deadband_ = std::max(
        0.0, patrol_pp_lateral_deadband_);
    patrol_pp_max_vy_ = std::max(0.01, std::abs(patrol_pp_max_vy_));
    patrol_pp_acc_lim_y_ = std::max(0.01, patrol_pp_acc_lim_y_);
    patrol_pp_avoid_offset_rate_ = std::max(
        0.01, patrol_pp_avoid_offset_rate_);
    patrol_pp_return_offset_rate_ = std::max(
        0.01, patrol_pp_return_offset_rate_);
    patrol_pp_goal_position_tolerance_ = std::max(
        0.005, patrol_pp_goal_position_tolerance_);
    patrol_pp_goal_slowdown_distance_ = std::max(
        patrol_pp_goal_position_tolerance_,
        patrol_pp_goal_slowdown_distance_);


    target_index_ = 0;
    goal_reached_ = false;
    control_state_ = ControlState::WAITING_FOR_PLAN;
    last_cmd_vel_ = geometry_msgs::Twist();
    measured_body_twist_ = geometry_msgs::Twist();
    last_odom_stamp_ = ros::Time(0);
    odom_received_ = false;
    omega_filter_state_ = ButterworthFilterState();
    resetPatrolPpState();
    last_control_time_ = ros::Time(0);
    resetMpcState();

    if (c3_use_odometry_)
    {
        odom_sub_ = private_nh.subscribe<nav_msgs::Odometry>(
            c3_odom_topic_, 20, &MyPlanner::odomCallback, this);
    }

    mpc_reference_path_pub_ =
        private_nh.advertise<nav_msgs::Path>("mpc_reference_path", 1, false);
    mpc_predicted_path_pub_ =
        private_nh.advertise<nav_msgs::Path>("mpc_predicted_path", 1, false);

    raw_global_path_pub_ =
        private_nh.advertise<nav_msgs::Path>("raw_global_path", 1, true);
    healed_global_path_pub_ =
        private_nh.advertise<nav_msgs::Path>("healed_global_path", 1, true);
    healed_window_path_pub_ =
        private_nh.advertise<nav_msgs::Path>("healed_window_path", 1, true);

    // Patrol-R2：这是巡检真正的“全局参考线”。它由外部固定Path直接给出，
    // 完全不经过move_base GlobalPlanner或path healing，因此RViz中应始终是理论直线。
    patrol_reference_path_pub_ =
        private_nh.advertise<nav_msgs::Path>("patrol_reference_path", 1, true);

    patrol_path_sub_ = private_nh.subscribe<nav_msgs::Path>(
        "patrol_path", 1, &MyPlanner::patrolPathCallback, this);
    patrol_path_lock_service_ = private_nh.advertiseService(
        "lock_patrol_path", &MyPlanner::lockPatrolPathCallback, this);
    controller_reset_service_ = private_nh.advertiseService(
        "reset_controller_state",
        &MyPlanner::resetControllerStateCallback,
        this);

    // Patrol-R2继续复用唯一C5实例：普通导航与巡检都直接读取master costmap。
    // 区别只在Reference来源：普通导航来自move_base；巡检来自staged直线Path并跳过path healing。
    clearance_path_optimizer_.initialize(
        private_nh, costmap_ros_->getCostmap(), costmap_frame_);
    runtime_parameters_initialized_ = true;

    ROS_INFO("路径重规划开关：%s（raw路径前视%.2fm，确认%d帧，阈值=%u）；可运行时修改参数 "
             "/move_base/%s/enable_path_replanning。",
             enable_path_replanning_ ? "开启" : "关闭",
             collision_check_lookahead_distance_,
             collision_replan_confirm_frames_,
             static_cast<unsigned int>(collision_cost_threshold_),
             name.c_str());

    ROS_INFO("固定巡检路径接口已启动：patrol_path、lock_patrol_path、"
             "reset_controller_state；终点匹配阈值=%.3fm/%.1f度。",
             patrol_goal_position_tolerance_,
             patrol_goal_yaw_tolerance_ * 180.0 / M_PI);

    ROS_WARN("Patrol-R2巡检路径链：staged直线Reference -> master costmap C5(保留墙体inflation) "
             "-> optimized path -> Patrol PP；巡检跳过GlobalPlanner几何和path healing。\n"
             "RViz权威直线参考：~patrol_reference_path；C5结果：~clearance_optimizer/c5_optimized_path。");

    ROS_INFO("固定巡检全向PP：%s；快速对准接管阈值=%.1f度（不停车等待停稳），"
             "对准Kp/max/near=%.2f/%.2f/%.2frad/s，"
             "航向保持=%s(Kp=%.2f，max=%.2frad/s，死区=%.1f度)，"
             "前视=%.2fm，横向窗口=%.2f~%.2fm，max_vy=%.2fm/s。",
             patrol_pp_enabled_ ? "启用" : "关闭",
             patrol_pp_align_tolerance_ * 180.0 / M_PI,
             patrol_align_kp_, patrol_align_max_wz_, patrol_align_near_wz_,
             patrol_heading_hold_enabled_ ? "启用" : "关闭",
             patrol_heading_kp_, patrol_heading_max_wz_,
             patrol_heading_deadband_ * 180.0 / M_PI,
             patrol_pp_lookahead_dist_,
             patrol_pp_preview_start_, patrol_pp_preview_end_,
             patrol_pp_max_vy_);

    ROS_INFO("路径治愈可视化：%s，发布频率=%.1fHz；"
             "话题为 raw_global_path / healed_global_path / healed_window_path。",
             publish_path_healing_debug_ ? "开启" : "关闭",
             path_healing_debug_publish_rate_);

    initialized_ = true;

    ROS_WARN("IFLY2026_FIXED_PATH_PATROL_V11_PRECISE_CORNER_DEADZONE_FIX_20260808："
             "MyPlanner MPC-C4.1.4 FIXED-PATROL-PATH 启动："
             "mode=%s，路径合速度=%.2f~%.2fm/s，N=%d，dt=%.3fs；"
             "odom=%s topic=%s timeout=%.2fs，input_delay=%.3fs，tau(v/w)=%.3f/%.3fs；"
             "omegaRate(acc/dec/rev)=%.1f/%.1f/%.1f，curvatureFF=%.2f，"
             "progressWeight=%.2f，速度圆=%.2fm/s(%d边)；"
             "初始对准=%.1f/%.1fdeg，原始终点接管距离=%.3fm。"
             "base=%s，costmap=%s。",
             controller_mode_.c_str(), c2_min_curve_speed_, c2_max_reference_speed_,
             mpc_horizon_steps_, mpc_dt_,
             c3_use_odometry_ ? "启用" : "关闭", c3_odom_topic_.c_str(),
             c3_odom_timeout_, c3_input_delay_,
             c3_translational_response_tau_, c3_angular_response_tau_,
             c3_reference_omega_accel_rate_, c3_reference_omega_decel_rate_,
             c3_reference_omega_reverse_rate_,
             c3_omega_curvature_feedforward_gain_, mpc_weight_progress_,
             mpc_max_translational_speed_, mpc_velocity_polygon_sides_,
             initial_align_enter_angle_ * 180.0 / M_PI,
             initial_align_exit_angle_ * 180.0 / M_PI,
             goal_dist_threshold_,
             base_frame_.c_str(), costmap_frame_.c_str());
}

bool MyPlanner::transformPose(const std::string& target_frame,
                              const geometry_msgs::PoseStamped& input,
                              geometry_msgs::PoseStamped& output)
{
    geometry_msgs::PoseStamped pose = input;
    pose.header.stamp = ros::Time(0);

    try
    {
        tf_listener_->transformPose(target_frame, pose, output);
        return true;
    }
    catch (const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "MyPlanner 坐标变换失败：%s", ex.what());
        return false;
    }
}

bool MyPlanner::isNewGoal(const geometry_msgs::PoseStamped& goal) const
{
    if (!has_goal_ || goal.header.frame_id != goal_pose_.header.frame_id)
        return true;

    const double dx =
        goal.pose.position.x - goal_pose_.pose.position.x;
    const double dy =
        goal.pose.position.y - goal_pose_.pose.position.y;
    const double position_change = std::hypot(dx, dy);

    const double old_yaw = tf::getYaw(goal_pose_.pose.orientation);
    const double new_yaw = tf::getYaw(goal.pose.orientation);
    const double yaw_change =
        std::abs(normalizeAngle(new_yaw - old_yaw));

    return position_change > 0.05 || yaw_change > 0.12;
}

const char* MyPlanner::controlStateName(ControlState state)
{
    switch (state)
    {
        case ControlState::WAITING_FOR_PLAN:
            return "WAITING_FOR_PLAN";
        case ControlState::INITIAL_ALIGN:
            return "INITIAL_ALIGN";
        case ControlState::PATROL_SETTLING:
            return "PATROL_SETTLING";
        case ControlState::PATH_TRACKING:
            return "PATH_TRACKING";
        case ControlState::FINAL_SETTLING:
            return "FINAL_SETTLING";
        case ControlState::GOAL_HOLD:
            return "GOAL_HOLD";
        case ControlState::FAILURE_STOP:
            return "FAILURE_STOP";
        default:
            return "UNKNOWN";
    }
}

void MyPlanner::transitionTo(
    ControlState next_state,
    const std::string& reason)
{
    if (control_state_ == next_state)
        return;

    const ControlState previous = control_state_;
    control_state_ = next_state;

    if (next_state != ControlState::GOAL_HOLD)
        goal_reached_ = false;

    ROS_INFO("控制状态：%s -> %s；%s",
             controlStateName(previous),
             controlStateName(next_state),
             reason.c_str());
}

bool MyPlanner::isPatrolPpActive() const
{
    std::lock_guard<std::mutex> lock(patrol_path_mutex_);
    // Patrol-R2：固定巡检是否生效只由“PP启用 + Path锁定 + staged Reference有效”决定。
    // 不再依赖active_plan_is_patrol_。后者只保留为诊断状态，避免第二段以后
    // 因setPlan()/lock回调时序交错而错误掉回普通C5路径链。
    return patrol_pp_enabled_
        && patrol_path_locked_
        && staged_patrol_plan_.size() >= 2;
}

bool MyPlanner::isPatrolPpAlignmentRequired() const
{
    std::lock_guard<std::mutex> lock(patrol_path_mutex_);
    return patrol_pp_alignment_required_;
}

void MyPlanner::markPatrolPpAlignmentComplete()
{
    std::lock_guard<std::mutex> lock(patrol_path_mutex_);
    patrol_pp_alignment_required_ = false;
}

void MyPlanner::resetPatrolPpState()
{
    patrol_pp_settle_counter_ = 0;
    patrol_pp_filtered_offset_ = 0.0;
}

bool MyPlanner::computeInitialPathTangentError(double& angle_error)
{
    angle_error = 0.0;
    // 巡检开始和行进中的航向闭环都必须对准原始固定直线，不能对准
    // 净空优化器产生的绕障弧线，避免车头跟随路障左右摆动。
    const std::vector<geometry_msgs::PoseStamped>& tangent_plan =
        isPatrolPpActive() ? raw_plan_ : global_plan_;

    if (tangent_plan.size() < 2)
        return false;

    const int path_size = static_cast<int>(tangent_plan.size());
    const int search_end = std::min(
        path_size - 1, std::max(20, mpc_reference_search_ahead_points_));

    std::string plan_frame = tangent_plan.front().header.frame_id;
    if (plan_frame.empty())
        plan_frame = costmap_frame_;

    tf::StampedTransform plan_to_base;
    try
    {
        if (plan_frame == base_frame_)
            plan_to_base.setIdentity();
        else
            tf_listener_->lookupTransform(
                base_frame_, plan_frame, ros::Time(0), plan_to_base);
    }
    catch (const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "初始路径切线TF查询失败：%s", ex.what());
        return false;
    }

    std::vector<PathPoint2D> local_points;
    local_points.reserve(static_cast<std::size_t>(search_end + 1));

    for (int index = 0; index <= search_end; ++index)
    {
        const geometry_msgs::PoseStamped& source =
            tangent_plan[static_cast<std::size_t>(index)];
        const std::string source_frame = source.header.frame_id.empty()
            ? plan_frame : source.header.frame_id;

        PathPoint2D point;
        if (source_frame == plan_frame)
        {
            const tf::Vector3 input(
                source.pose.position.x, source.pose.position.y, 0.0);
            const tf::Vector3 output = plan_to_base * input;
            point.x = output.x();
            point.y = output.y();
        }
        else
        {
            geometry_msgs::PoseStamped local_pose;
            if (!transformPose(base_frame_, source, local_pose))
                continue;
            point.x = local_pose.pose.position.x;
            point.y = local_pose.pose.position.y;
        }
        point.source_index = index;

        if (!local_points.empty()
            && std::hypot(
                   point.x - local_points.back().x,
                   point.y - local_points.back().y)
               < c2_duplicate_point_distance_)
        {
            continue;
        }
        local_points.push_back(point);
    }

    if (local_points.size() < 2)
        return false;

    std::size_t closest = 0;
    double closest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < local_points.size(); ++i)
    {
        const double distance = std::hypot(
            local_points[i].x,
            local_points[i].y);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest = i;
        }
    }

    std::size_t ahead = closest;
    double accumulated = 0.0;
    while (ahead + 1 < local_points.size()
           && accumulated < initial_path_tangent_lookahead_)
    {
        accumulated += std::hypot(
            local_points[ahead + 1].x - local_points[ahead].x,
            local_points[ahead + 1].y - local_points[ahead].y);
        ++ahead;
    }

    if (ahead == closest)
        return false;

    const double dx = local_points[ahead].x - local_points[closest].x;
    const double dy = local_points[ahead].y - local_points[closest].y;
    if (std::hypot(dx, dy) < 1.0e-4)
        return false;

    angle_error = normalizeAngle(std::atan2(dy, dx));
    target_index_ = std::max(
        target_index_, local_points[closest].source_index);
    return true;
}

bool MyPlanner::shouldEnterInitialAlign(double angle_error) const
{
    return enable_initial_rotation_
        && std::abs(angle_error) > initial_align_enter_angle_;
}

void MyPlanner::resetForNewGoal()
{
    target_index_ = 0;
    collision_replan_confirm_counter_ = 0;
    c5_failure_replan_confirm_counter_ = 0;
    c5_failure_replan_candidate_ = false;
    c5_failure_replan_reason_.clear();
    goal_reached_ = false;
    control_state_ = ControlState::WAITING_FOR_PLAN;
    last_control_time_ = ros::Time(0);
    resetPatrolPpState();
    // 新目标到来时底盘并不会瞬间静止，必须保留上一命令和延迟历史，
    // 否则外层加速度限制与C3.2输入延迟预测会把运动中的车辆误认为零速。
    resetMpcState();
}

void MyPlanner::stopImmediately(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    last_cmd_vel_ = geometry_msgs::Twist();
    recordPublishedCommand(last_cmd_vel_, ros::Time::now());
}

void MyPlanner::patrolPathCallback(
    const nav_msgs::Path::ConstPtr& message)
{
    if (!message || message->poses.size() < 2)
    {
        ROS_ERROR("拒绝外部巡检路径：路径点少于2个。");
        return;
    }

    std::vector<geometry_msgs::PoseStamped> candidate = message->poses;
    const std::string default_frame = message->header.frame_id.empty()
        ? costmap_frame_ : message->header.frame_id;

    const ros::Time publish_stamp = ros::Time::now();
    for (std::size_t i = 0; i < candidate.size(); ++i)
    {
        geometry_msgs::PoseStamped& pose = candidate[i];
        if (pose.header.frame_id.empty())
            pose.header.frame_id = default_frame;
        if (pose.header.frame_id != default_frame
            || !std::isfinite(pose.pose.position.x)
            || !std::isfinite(pose.pose.position.y))
        {
            ROS_ERROR("拒绝外部巡检路径：第%zu个点的坐标或坐标系无效。", i);
            return;
        }
        pose.header.stamp = ros::Time(0);
    }

    // Patrol-R2：单独发布权威直线Reference，避免把move_base GlobalPlanner/plan
    // （它仍可能被墙体inflation推弯）误认为实际巡检参考线。
    nav_msgs::Path reference_msg;
    reference_msg.header.frame_id = default_frame;
    reference_msg.header.stamp = publish_stamp;
    reference_msg.poses = candidate;
    for (std::size_t i = 0; i < reference_msg.poses.size(); ++i)
        reference_msg.poses[i].header.stamp = publish_stamp;
    patrol_reference_path_pub_.publish(reference_msg);

    {
        std::lock_guard<std::mutex> lock(patrol_path_mutex_);
        staged_patrol_plan_.swap(candidate);
        ++patrol_path_revision_;
        if (patrol_path_locked_)
        {
            // 固定Path在锁保持期间更新时，新Reference立即取得所有权。
            active_plan_is_patrol_ = true;
            patrol_pp_alignment_required_ = true;
        }
        ROS_INFO("已暂存巡检直线Reference：修订号=%u，点数=%zu，终点=(%.3f, %.3f)。",
                 static_cast<unsigned int>(patrol_path_revision_),
                 staged_patrol_plan_.size(),
                 staged_patrol_plan_.back().pose.position.x,
                 staged_patrol_plan_.back().pose.position.y);
    }
}

bool MyPlanner::lockPatrolPathCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response)
{
    std::lock_guard<std::mutex> lock(patrol_path_mutex_);
    if (request.data)
    {
        if (staged_patrol_plan_.size() < 2)
        {
            response.success = false;
            response.message = "尚未收到至少包含2个点的patrol_path";
            return true;
        }
        patrol_path_locked_ = true;
        // Patrol-R2：锁定本身就是Reference所有权声明，不再等待后续某次setPlan()
        // 才把active_plan_is_patrol_置true。控制循环会每周期再次强制同步
        // staged_patrol_plan_，因此move_base后续下发的普通全局路线无法覆盖它。
        active_plan_is_patrol_ = true;
        // 不能只依赖外部reset_controller_state切换控制状态。服务回调、
        // setPlan和控制循环可能异步交错，因此每次重新锁定固定Path时
        // 使用独立闩锁强制执行一次“快速旋转到5度内->边走边纠偏”。
        patrol_pp_alignment_required_ = true;
        response.success = true;
        response.message = "固定巡检路径已锁定";
        ROS_WARN("固定巡检路径已锁定：修订号=%u，点数=%zu。",
                 static_cast<unsigned int>(patrol_path_revision_),
                 staged_patrol_plan_.size());
        return true;
    }

    patrol_path_locked_ = false;
    active_plan_is_patrol_ = false;
    patrol_pp_alignment_required_ = false;
    staged_patrol_plan_.clear();
    resetPatrolPpState();
    response.success = true;
    response.message = "固定巡检路径已解除";
    ROS_WARN("固定巡检路径已解除并清空暂存路线；"
             "后续setPlan恢复使用move_base全局路径。");
    return true;
}

void MyPlanner::resetControllerStateForExternalControl()
{
    target_index_ = 0;
    goal_reached_ = false;
    control_state_ = ControlState::WAITING_FOR_PLAN;
    last_cmd_vel_ = geometry_msgs::Twist();
    last_control_time_ = ros::Time(0);
    resetPatrolPpState();

    {
        std::lock_guard<std::mutex> lock(patrol_path_mutex_);
        // 若复位发生在固定Path仍锁定期间，下一控制周期必须重新对准；
        // 普通导航或尚未锁定新巡检Path时则不保留旧要求。
        patrol_pp_alignment_required_ = patrol_path_locked_;
    }

    {
        std::lock_guard<std::mutex> lock(command_history_mutex_);
        command_history_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(measured_state_mutex_);
        measured_body_twist_ = geometry_msgs::Twist();
        last_odom_stamp_ = ros::Time(0);
        odom_received_ = false;
        omega_filter_state_ = ButterworthFilterState();
    }

    clearance_path_optimizer_.reset();
    resetMpcState();
}

bool MyPlanner::resetControllerStateCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
{
    resetControllerStateForExternalControl();
    response.success = true;
    response.message = "控制器速度历史、输入延迟历史和MPC状态已复位";
    ROS_WARN("外部请求已复位局部规划器控制状态；下一条路径将从静止状态启动。");
    return true;
}

bool MyPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (plan.empty())
    {
        ROS_ERROR("MyPlanner 收到空路径。");
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> effective_plan = plan;
    bool using_patrol_path = false;
    bool patrol_path_changed = false;
    std::uint32_t patrol_revision = 0;

    {
        std::lock_guard<std::mutex> lock(patrol_path_mutex_);
        if (patrol_path_locked_)
        {
            if (staged_patrol_plan_.size() < 2)
            {
                ROS_ERROR("固定巡检路径处于锁定状态，但暂存路径无效。");
                return false;
            }

            const geometry_msgs::PoseStamped& move_base_goal = plan.back();
            const geometry_msgs::PoseStamped& patrol_goal =
                staged_patrol_plan_.back();
            const double goal_distance = std::hypot(
                move_base_goal.pose.position.x - patrol_goal.pose.position.x,
                move_base_goal.pose.position.y - patrol_goal.pose.position.y);
            const double goal_yaw_error = std::abs(normalizeAngle(
                tf::getYaw(move_base_goal.pose.orientation)
                - tf::getYaw(patrol_goal.pose.orientation)));

            if (goal_distance > patrol_goal_position_tolerance_
                || goal_yaw_error > patrol_goal_yaw_tolerance_)
            {
                // Patrol-R2：锁定固定巡检Reference后，move_base的setPlan只负责维持
                // BaseLocalPlanner控制循环，不再拥有巡检几何路径的覆盖权。
                // 因此即使某次全局重规划/时序交错导致其终点暂时不匹配，
                // 也只记录告警，仍强制使用当前staged patrol Path。
                ROS_WARN_THROTTLE(
                    1.0,
                    "Patrol-R2固定Reference强锁：忽略move_base路径终点差异"
                    "%.3fm/%.1f度，继续强制使用staged patrol Path终点(%.3f, %.3f)。",
                    goal_distance, goal_yaw_error * 180.0 / M_PI,
                    patrol_goal.pose.position.x,
                    patrol_goal.pose.position.y);
            }

            effective_plan = staged_patrol_plan_;
            using_patrol_path = true;
            patrol_revision = patrol_path_revision_;
            patrol_path_changed =
                patrol_revision != applied_patrol_path_revision_;
            if (patrol_path_changed)
                patrol_pp_alignment_required_ = true;
        }
        active_plan_is_patrol_ = using_patrol_path;
    }

    const bool new_goal = patrol_path_changed
        || isNewGoal(effective_plan.back());

    raw_plan_ = effective_plan;
    global_plan_ = effective_plan;
    // 同一目标的move_base全局路径更新保留C5 warm start；
    // 只有真正新目标/新固定巡检Path才清空上一条轨迹历史。
    if (new_goal)
        clearance_path_optimizer_.reset();
    goal_pose_ = effective_plan.back();
    has_goal_ = true;

    if (using_patrol_path)
        applied_patrol_path_revision_ = patrol_revision;

    // Patrol-R2：普通导航setPlan仍从新raw plan重新建立索引；固定巡检锁定期间，
    // move_base即使重复setPlan也不能打断当前固定Path进度。只有staged patrol
    // Path修订号真正变化时才把target_index和确认状态重新从头建立。
    if (!using_patrol_path || patrol_path_changed)
    {
        target_index_ = 0;
        collision_replan_confirm_counter_ = 0;
        c5_failure_replan_confirm_counter_ = 0;
        c5_failure_replan_candidate_ = false;
        c5_failure_replan_reason_.clear();
    }

    if (new_goal)
    {
        resetForNewGoal();
        ROS_INFO("收到%s，状态已重置；路径点数：%zu。",
                 using_patrol_path ? "新的固定巡检路径" : "新目标",
                 effective_plan.size());
    }
    else
    {
        if (control_state_ == ControlState::FAILURE_STOP)
            transitionTo(ControlState::PATH_TRACKING, "收到同目标的新全局路径");
        resetMpcState();
        ROS_INFO("同一目标的%s已更新；路径点数：%zu。",
                 using_patrol_path ? "固定巡检路径" : "全局路径",
                 effective_plan.size());
    }

    return true;
}


void MyPlanner::refreshRuntimeParameters()
{
    // C5.4不再读取clearance_optimizer/shadow_mode：
    // C5为唯一轨迹优化器，旧巡检脚本即使继续rosparam set shadow_mode也不会旁路C5。
    bool requested_clearance_optimizer_enabled =
        clearance_path_optimizer_.enabled();
    bool requested_enable_path_replanning =
        enable_path_replanning_;
    double requested_c2_max_reference_speed =
        c2_max_reference_speed_;
    double requested_mpc_max_vx =
        mpc_max_vx_;
    double requested_mpc_max_translational_speed =
        mpc_max_translational_speed_;
    double requested_max_vel_x =
        max_vel_x_;

    private_nh_.getParamCached(
        "clearance_optimizer/enabled",
        requested_clearance_optimizer_enabled);
    private_nh_.getParamCached(
        "enable_path_replanning",
        requested_enable_path_replanning);
    private_nh_.getParamCached(
        "c2_max_reference_speed",
        requested_c2_max_reference_speed);
    private_nh_.getParamCached(
        "mpc_max_vx",
        requested_mpc_max_vx);
    private_nh_.getParamCached(
        "mpc_max_translational_speed",
        requested_mpc_max_translational_speed);
    private_nh_.getParamCached(
        "max_vel_x",
        requested_max_vel_x);

    if (!std::isfinite(
            requested_c2_max_reference_speed))
    {
        requested_c2_max_reference_speed =
            c2_max_reference_speed_;
    }

    if (!std::isfinite(
            requested_mpc_max_vx))
    {
        requested_mpc_max_vx =
            mpc_max_vx_;
    }

    if (!std::isfinite(
            requested_mpc_max_translational_speed))
    {
        requested_mpc_max_translational_speed =
            mpc_max_translational_speed_;
    }

    if (!std::isfinite(
            requested_max_vel_x))
    {
        requested_max_vel_x =
            max_vel_x_;
    }

    const double new_max_vel_x =
        std::max(
            0.05,
            requested_max_vel_x);

    const double new_c2_max_reference_speed =
        clampValue(
            requested_c2_max_reference_speed,
            0.05,
            new_max_vel_x);

    const double new_mpc_max_vx =
        std::max(
            mpc_min_vx_,
            requested_mpc_max_vx);

    const double maximum_mpc_translational_capability =
        std::hypot(
            new_mpc_max_vx,
            std::max(
                std::abs(mpc_min_vy_),
                std::abs(mpc_max_vy_)));

    const double new_mpc_max_translational_speed =
        std::max(
            0.05,
            std::min(
                requested_mpc_max_translational_speed,
                maximum_mpc_translational_capability));

    const bool speed_changed =
        std::abs(
            new_c2_max_reference_speed
            - c2_max_reference_speed_) > 1.0e-9
        || std::abs(
            new_mpc_max_vx
            - mpc_max_vx_) > 1.0e-9
        || std::abs(
            new_mpc_max_translational_speed
            - mpc_max_translational_speed_) > 1.0e-9
        || std::abs(
            new_max_vel_x
            - max_vel_x_) > 1.0e-9;

    c2_max_reference_speed_ =
        new_c2_max_reference_speed;
    mpc_max_vx_ =
        new_mpc_max_vx;
    mpc_max_translational_speed_ =
        new_mpc_max_translational_speed;
    max_vel_x_ =
        new_max_vel_x;

    if (speed_changed)
    {
        ROS_WARN(
            "运行时速度参数已更新："
            "c2_max_reference_speed=%.3f，"
            "mpc_max_vx=%.3f，"
            "mpc_max_translational_speed=%.3f，"
            "max_vel_x=%.3f。",
            c2_max_reference_speed_,
            mpc_max_vx_,
            mpc_max_translational_speed_,
            max_vel_x_);
    }

    if (requested_clearance_optimizer_enabled
        != clearance_path_optimizer_.enabled())
    {
        clearance_path_optimizer_.setEnabled(
            requested_clearance_optimizer_enabled);

        // C5状态切换后，旧的失败/重规划确认不能带到新模式。
        c5_failure_replan_confirm_counter_ = 0;
        c5_failure_replan_candidate_ = false;
        c5_failure_replan_reason_.clear();

        ROS_WARN(
            "运行时 clearance_optimizer/enabled 已真正切换为 %s。",
            clearance_path_optimizer_.enabled()
                ? "true" : "false");
    }

    if (requested_enable_path_replanning
        != enable_path_replanning_)
    {
        enable_path_replanning_ =
            requested_enable_path_replanning;
        collision_replan_confirm_counter_ = 0;
        c5_failure_replan_confirm_counter_ = 0;
        c5_failure_replan_candidate_ = false;
        c5_failure_replan_reason_.clear();

        ROS_WARN(
            "运行时路径重规划已%s：%s",
            enable_path_replanning_
                ? "开启" : "关闭",
            enable_path_replanning_
                ? "从本周期起独立检查raw global path死区；C5 active path继续负责soft inflation。"
                : "跳过raw global path死区确认，继续跟踪C5 active path。");
    }

    runtime_parameters_initialized_ = true;
}

void MyPlanner::updateHealedPath()
{
    if (raw_plan_.empty())
        return;

    // 每个控制周期从原始路径重新开始，避免路径点累计漂移。
    global_plan_ = raw_plan_;

    target_index_ = std::max(
        0,
        std::min(target_index_,
                 static_cast<int>(global_plan_.size()) - 1));

    const int heal_start =
        std::max(0, target_index_ - path_healing_points_behind_);
    const int heal_end =
        std::min(static_cast<int>(global_plan_.size()),
                 target_index_ + path_healing_points_ahead_);

    if (!enable_path_healing_ || path_healing_iterations_ <= 0)
    {
        publishPathHealingDebug(heal_start, heal_start);
        return;
    }

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (costmap == NULL)
    {
        publishPathHealingDebug(heal_start, heal_start);
        return;
    }

    const double resolution = costmap->getResolution();
    const double max_step =
        std::min(std::max(0.0, path_healing_max_step_), resolution);

    for (int i = heal_start; i < heal_end; ++i)
    {
        const geometry_msgs::PoseStamped original_point = raw_plan_[i];

        geometry_msgs::PoseStamped point_costmap;
        if (!transformPose(costmap_frame_, original_point, point_costmap))
            continue;

        double wx = point_costmap.pose.position.x;
        double wy = point_costmap.pose.position.y;

        for (int iteration = 0;
             iteration < path_healing_iterations_;
             ++iteration)
        {
            double gradient_x = 0.0;
            double gradient_y = 0.0;

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
                    lock(*(costmap->getMutex()));

                unsigned int mx = 0;
                unsigned int my = 0;

                if (!costmap->worldToMap(wx, wy, mx, my)
                    || mx == 0
                    || my == 0
                    || mx + 1 >= costmap->getSizeInCellsX()
                    || my + 1 >= costmap->getSizeInCellsY())
                {
                    break;
                }

                const unsigned char center_cost =
                    costmap->getCost(mx, my);

                // 空旷区域不动；已经进入障碍栅格时交给重规划处理。
                if (center_cost == 0
                    || center_cost >= collision_cost_threshold_)
                {
                    break;
                }

                const int cost_up =
                    static_cast<int>(costmap->getCost(mx, my + 1));
                const int cost_down =
                    static_cast<int>(costmap->getCost(mx, my - 1));
                const int cost_left =
                    static_cast<int>(costmap->getCost(mx - 1, my));
                const int cost_right =
                    static_cast<int>(costmap->getCost(mx + 1, my));

                gradient_x =
                    static_cast<double>(cost_left - cost_right);
                gradient_y =
                    static_cast<double>(cost_down - cost_up);
            }

            const double gradient_norm =
                std::hypot(gradient_x, gradient_y);

            if (gradient_norm <= path_healing_gradient_deadband_)
                break;

            const double strength = std::min(
                1.0,
                (gradient_norm - path_healing_gradient_deadband_)
                / path_healing_gradient_scale_);

            const double push_step = max_step * strength;

            wx += gradient_x / gradient_norm * push_step;
            wy += gradient_y / gradient_norm * push_step;
        }

        point_costmap.pose.position.x = wx;
        point_costmap.pose.position.y = wy;
        point_costmap.header.stamp = ros::Time(0);

        const std::string original_frame =
            original_point.header.frame_id.empty()
                ? costmap_frame_
                : original_point.header.frame_id;

        geometry_msgs::PoseStamped healed_point;

        if (original_frame == costmap_frame_)
        {
            healed_point = point_costmap;
            healed_point.header.frame_id = original_frame;
            global_plan_[i] = healed_point;
        }
        else if (transformPose(original_frame,
                               point_costmap,
                               healed_point))
        {
            global_plan_[i] = healed_point;
        }
    }

    publishPathHealingDebug(heal_start, heal_end);
}


void MyPlanner::updateClearanceOptimizedPath()
{
    // 每个控制周期先清本周期C5失败候选。只有明确的“无安全active path”状态
    // 才参与C5连续失败重规划确认；TF/地图快照/普通数值波动不触发global replan。
    c5_failure_replan_candidate_ = false;
    c5_failure_replan_reason_.clear();

    if (!clearance_path_optimizer_.enabled()
        || global_plan_.size() < 3)
    {
        return;
    }

    // 每周期global_plan_刚由updateHealedPath()恢复到当前raw plan。
    // C5生成的optimized_full_path就是本周期唯一active path。
    std::vector<geometry_msgs::PoseStamped>
        plan_costmap;
    plan_costmap.reserve(
        global_plan_.size());

    for (std::size_t i = 0;
         i < global_plan_.size();
         ++i)
    {
        geometry_msgs::PoseStamped transformed;

        if (!transformPose(
                costmap_frame_,
                global_plan_[i],
                transformed))
        {
            ROS_WARN_THROTTLE(
                1.0,
                "C5.4输入路径TF失败；"
                "本周期保留raw path并等待下一周期。");
            return;
        }

        transformed.header.frame_id =
            costmap_frame_;
        transformed.header.stamp =
            ros::Time(0);

        plan_costmap.push_back(
            transformed);
    }

    geometry_msgs::PoseStamped robot_origin;
    robot_origin.header.frame_id =
        base_frame_;
    robot_origin.header.stamp =
        ros::Time(0);
    robot_origin.pose.orientation.w =
        1.0;

    geometry_msgs::PoseStamped robot_costmap;

    if (!transformPose(
            costmap_frame_,
            robot_origin,
            robot_costmap))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "C5.4机器人位姿TF失败；"
            "本周期保留raw path并等待下一周期。");
        return;
    }

    ClearancePathOptimizer::Output output;

    const bool optimized =
        clearance_path_optimizer_.optimize(
            plan_costmap,
            robot_costmap,
            output);

    if (!optimized || !output.valid)
    {
        // 只有C5明确确认“fresh/HOLD/reuse均无安全active path”的状态，
        // 才作为第二条global replan触发源。普通优化数值失败、TF失败、
        // 局部窗口末端不足等情况不应通过重新跑global planner来掩盖。
        const std::string& status = output.report.status;
        if (status == "HARD_FAIL"
            || status == "SAFETY_VALIDATION_FAILED")
        {
            c5_failure_replan_candidate_ = true;
            c5_failure_replan_reason_ = status;
        }
        return;
    }

    // 一旦当前周期重新取得安全active path，立即清掉C5失败确认。
    c5_failure_replan_confirm_counter_ = 0;

    // 输出点数与输入完全一致，target_index_、MPC、最终姿态和
    // 固定巡检状态机的索引语义全部保持不变。
    global_plan_ =
        output.optimized_full_path;
}

void MyPlanner::publishPathDebug(
    const std::vector<geometry_msgs::PoseStamped>& source_path,
    ros::Publisher& publisher,
    int start_index,
    int end_index,
    double z_offset)
{
    if (source_path.empty() || publisher.getNumSubscribers() == 0)
        return;

    const int point_count = static_cast<int>(source_path.size());
    const int begin = std::max(0, std::min(start_index, point_count));
    const int end = std::max(begin, std::min(end_index, point_count));
    if (begin >= end)
        return;

    std::string path_frame = source_path[begin].header.frame_id;
    if (path_frame.empty())
        path_frame = costmap_frame_;

    nav_msgs::Path debug_path;
    debug_path.header.frame_id = path_frame;
    debug_path.header.stamp = ros::Time::now();
    debug_path.poses.reserve(end - begin);

    for (int i = begin; i < end; ++i)
    {
        geometry_msgs::PoseStamped pose = source_path[i];

        if (pose.header.frame_id.empty())
        {
            pose.header.frame_id = path_frame;
        }
        else if (pose.header.frame_id != path_frame)
        {
            geometry_msgs::PoseStamped transformed_pose;
            if (!transformPose(path_frame, pose, transformed_pose))
                continue;
            pose = transformed_pose;
        }

        pose.header.stamp = debug_path.header.stamp;
        pose.pose.position.z += z_offset;
        debug_path.poses.push_back(pose);
    }

    if (!debug_path.poses.empty())
        publisher.publish(debug_path);
}

void MyPlanner::publishPathHealingDebug(int heal_start, int heal_end)
{
    if (!publish_path_healing_debug_)
        return;

    if (raw_global_path_pub_.getNumSubscribers() == 0
        && healed_global_path_pub_.getNumSubscribers() == 0
        && healed_window_path_pub_.getNumSubscribers() == 0)
    {
        return;
    }

    const ros::Time now = ros::Time::now();
    const double minimum_interval =
        1.0 / path_healing_debug_publish_rate_;

    if (!last_path_healing_debug_publish_time_.isZero()
        && (now - last_path_healing_debug_publish_time_).toSec()
               < minimum_interval)
    {
        return;
    }

    last_path_healing_debug_publish_time_ = now;

    publishPathDebug(raw_plan_,
                     raw_global_path_pub_,
                     0,
                     static_cast<int>(raw_plan_.size()),
                     0.03);
    publishPathDebug(global_plan_,
                     healed_global_path_pub_,
                     0,
                     static_cast<int>(global_plan_.size()),
                     0.06);
    publishPathDebug(global_plan_,
                     healed_window_path_pub_,
                     heal_start,
                     heal_end,
                     0.09);
}

bool MyPlanner::checkPathCollision()
{
    // C5.4职责分离：
    // - C5 active path负责局部soft inflation规避；
    // - move_base全局重规划只看raw_plan_是否真正进入死区。
    // 不能再检查global_plan_，否则C5把路径推出障碍后会“吃掉”重规划触发条件。
    if (raw_plan_.empty())
    {
        collision_replan_confirm_counter_ = 0;
        return false;
    }

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (costmap == NULL)
    {
        collision_replan_confirm_counter_ = 0;
        return false;
    }

    const int raw_size = static_cast<int>(raw_plan_.size());
    const int check_start = std::max(
        0,
        std::min(target_index_, raw_size - 1));

    bool collision_found = false;
    int collision_index = -1;
    unsigned int collision_cost = 0;
    double collision_arc_distance = 0.0;
    double accumulated_distance = 0.0;

    for (int i = check_start; i < raw_size; ++i)
    {
        if (i > check_start)
        {
            const double dx =
                raw_plan_[i].pose.position.x
                - raw_plan_[i - 1].pose.position.x;
            const double dy =
                raw_plan_[i].pose.position.y
                - raw_plan_[i - 1].pose.position.y;
            accumulated_distance += std::hypot(dx, dy);

            if (accumulated_distance
                > collision_check_lookahead_distance_)
            {
                break;
            }
        }

        geometry_msgs::PoseStamped point_costmap;
        if (!transformPose(costmap_frame_,
                           raw_plan_[i],
                           point_costmap))
        {
            // TF偶发失败不能凭空制造一次重规划确认。
            continue;
        }

        unsigned int mx = 0;
        unsigned int my = 0;
        unsigned char cost = 0;

        {
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
                lock(*(costmap->getMutex()));

            if (!costmap->worldToMap(
                    point_costmap.pose.position.x,
                    point_costmap.pose.position.y,
                    mx,
                    my))
            {
                // rolling local costmap边界外不是障碍，不用于触发全局重规划。
                continue;
            }

            cost = costmap->getCost(mx, my);
        }

        const bool is_unknown =
            cost == costmap_2d::NO_INFORMATION;
        const bool is_replan_collision =
            (!is_unknown && cost >= collision_cost_threshold_)
            || (is_unknown && collision_replan_on_unknown_);

        if (is_replan_collision)
        {
            collision_found = true;
            collision_index = i;
            collision_cost = static_cast<unsigned int>(cost);
            collision_arc_distance = accumulated_distance;
            break;
        }
    }

    if (!collision_found)
    {
        if (collision_replan_confirm_counter_ > 0)
        {
            ROS_INFO_THROTTLE(
                1.0,
                "C5.4 raw路径LETHAL候选已消失；取消全局重规划确认。");
        }
        collision_replan_confirm_counter_ = 0;
        return true;
    }

    collision_replan_confirm_counter_ = std::min(
        collision_replan_confirm_counter_ + 1,
        collision_replan_confirm_frames_);

    if (collision_replan_confirm_counter_
        < collision_replan_confirm_frames_)
    {
        ROS_WARN_THROTTLE(
            0.5,
            "C5.4检测到raw全局路径LETHAL候选：index=%d，前视弧长=%.3fm，"
            "cost=%u，确认=%d/%d；本周期继续使用安全C5 active path。",
            collision_index,
            collision_arc_distance,
            collision_cost,
            collision_replan_confirm_counter_,
            collision_replan_confirm_frames_);
        return true;
    }

    ROS_WARN(
        "C5.4确认raw全局路径进入LETHAL，停止并请求move_base全局重规划："
        "index=%d，前视弧长=%.3fm，cost=%u，确认=%d/%d。",
        collision_index,
        collision_arc_distance,
        collision_cost,
        collision_replan_confirm_counter_,
        collision_replan_confirm_frames_);

    // 保持计数饱和，直到setPlan收到新raw plan；避免旧路径上又从1/2重新等待。
    collision_replan_confirm_counter_ = collision_replan_confirm_frames_;
    return false;
}

bool MyPlanner::checkC5FailureReplan()
{
    if (!c5_failure_replan_candidate_)
    {
        if (c5_failure_replan_confirm_counter_ > 0)
        {
            ROS_INFO_THROTTLE(
                1.0,
                "C5.4安全active path已恢复；取消C5失败全局重规划确认。");
        }
        c5_failure_replan_confirm_counter_ = 0;
        return true;
    }

    c5_failure_replan_confirm_counter_ = std::min(
        c5_failure_replan_confirm_counter_ + 1,
        c5_failure_replan_confirm_frames_);

    if (c5_failure_replan_confirm_counter_
        < c5_failure_replan_confirm_frames_)
    {
        ROS_WARN_THROTTLE(
            0.5,
            "C5.4连续无安全active path候选：reason=%s，确认=%d/%d；"
            "本周期不立即重规划，等待短时恢复/下一周期C5重新求解。",
            c5_failure_replan_reason_.c_str(),
            c5_failure_replan_confirm_counter_,
            c5_failure_replan_confirm_frames_);
        return true;
    }

    ROS_WARN(
        "C5.4确认C5连续无法提供安全active path，停止并请求move_base全局重规划："
        "reason=%s，确认=%d/%d。",
        c5_failure_replan_reason_.c_str(),
        c5_failure_replan_confirm_counter_,
        c5_failure_replan_confirm_frames_);

    c5_failure_replan_confirm_counter_ =
        c5_failure_replan_confirm_frames_;
    return false;
}

bool MyPlanner::selectTrackingTarget(
    geometry_msgs::PoseStamped& target_pose)
{
    if (global_plan_.empty())
        return false;

    target_index_ = std::max(
        0,
        std::min(target_index_,
                 static_cast<int>(global_plan_.size()) - 1));

    bool transformed_any_point = false;

    for (int i = target_index_;
         i < static_cast<int>(global_plan_.size());
         ++i)
    {
        geometry_msgs::PoseStamped pose_base;

        if (!transformPose(base_frame_, global_plan_[i], pose_base))
            continue;

        transformed_any_point = true;
        target_pose = pose_base;

        const double distance =
            std::hypot(pose_base.pose.position.x,
                       pose_base.pose.position.y);

        // 与上传代码一致：选择第一个与车体距离超过固定前视距离的点。
        if (distance > lookahead_dist_)
        {
            target_index_ = i;
            return true;
        }
    }

    // 距离终点不足前视距离时，使用最后一个可转换的路径点。
    if (transformed_any_point)
    {
        target_index_ =
            static_cast<int>(global_plan_.size()) - 1;
        return true;
    }

    return false;
}

bool MyPlanner::selectPatrolTrackingTarget(
    geometry_msgs::PoseStamped& target_pose)
{
    if (global_plan_.empty())
        return false;

    const int path_size = static_cast<int>(global_plan_.size());
    const int search_start = std::max(0, std::min(target_index_, path_size - 1));

    geometry_msgs::PoseStamped last_valid;
    bool have_last_valid = false;
    for (int i = search_start; i < path_size; ++i)
    {
        geometry_msgs::PoseStamped point_base;
        if (!transformPose(base_frame_, global_plan_[i], point_base))
            continue;

        last_valid = point_base;
        have_last_valid = true;
        const double distance = std::hypot(
            point_base.pose.position.x,
            point_base.pose.position.y);

        if (point_base.pose.position.x > 0.0
            && distance >= patrol_pp_lookahead_dist_)
        {
            target_index_ = i;
            target_pose = point_base;
            return true;
        }
    }

    if (have_last_valid)
    {
        target_index_ = path_size - 1;
        target_pose = last_valid;
        return true;
    }

    return false;
}

double MyPlanner::computeLateralDeviation(
    const geometry_msgs::PoseStamped& target_pose)
{
    // 恢复二代车原有的全向横移纠偏：
    // 在前视点之前的短区间内，选择绝对值最小的横向偏差。
    // 这样 vy 主要用于把车体拉回路径附近，不直接追逐远处前视点。
    double lateral_deviation = target_pose.pose.position.y;

    const int search_start = std::max(
        0, target_index_ - lateral_search_points_);

    for (int i = search_start; i < target_index_; ++i)
    {
        geometry_msgs::PoseStamped point_base;

        if (!transformPose(base_frame_, global_plan_[i], point_base))
            continue;

        if (std::abs(point_base.pose.position.y)
            < std::abs(lateral_deviation))
        {
            lateral_deviation = point_base.pose.position.y;
        }
    }

    return lateral_deviation;
}


bool MyPlanner::computePatrolPreviewErrors(
    double& raw_lateral_error,
    double& optimized_lateral_error)
{
    raw_lateral_error = 0.0;
    optimized_lateral_error = 0.0;

    if (raw_plan_.size() < 2
        || global_plan_.size() != raw_plan_.size())
    {
        return false;
    }

    std::string raw_frame =
        raw_plan_.front().header.frame_id;
    if (raw_frame.empty())
        raw_frame = costmap_frame_;

    std::string optimized_frame =
        global_plan_.front().header.frame_id;
    if (optimized_frame.empty())
        optimized_frame = costmap_frame_;

    tf::StampedTransform raw_to_base;
    tf::StampedTransform optimized_to_base;

    try
    {
        if (raw_frame == base_frame_)
            raw_to_base.setIdentity();
        else
            tf_listener_->lookupTransform(
                base_frame_,
                raw_frame,
                ros::Time(0),
                raw_to_base);

        if (optimized_frame == raw_frame)
            optimized_to_base = raw_to_base;
        else if (optimized_frame == base_frame_)
            optimized_to_base.setIdentity();
        else
            tf_listener_->lookupTransform(
                base_frame_,
                optimized_frame,
                ros::Time(0),
                optimized_to_base);
    }
    catch (const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "巡检C5预瞄窗口TF查询失败：%s",
            ex.what());
        return false;
    }

    std::vector<PathPoint2D> raw_local;
    raw_local.reserve(raw_plan_.size());

    for (std::size_t i = 0;
         i < raw_plan_.size();
         ++i)
    {
        const geometry_msgs::PoseStamped& pose =
            raw_plan_[i];

        const std::string frame =
            pose.header.frame_id.empty()
                ? raw_frame
                : pose.header.frame_id;

        PathPoint2D point;

        if (frame == raw_frame)
        {
            const tf::Vector3 source(
                pose.pose.position.x,
                pose.pose.position.y,
                0.0);

            const tf::Vector3 result =
                raw_to_base * source;

            point.x = result.x();
            point.y = result.y();
        }
        else
        {
            geometry_msgs::PoseStamped point_base;
            if (!transformPose(
                    base_frame_,
                    pose,
                    point_base))
            {
                return false;
            }

            point.x =
                point_base.pose.position.x;
            point.y =
                point_base.pose.position.y;
        }

        point.source_index =
            static_cast<int>(i);

        raw_local.push_back(point);
    }

    std::size_t closest = 0;
    double closest_distance =
        std::numeric_limits<double>::max();

    for (std::size_t i = 0;
         i < raw_local.size();
         ++i)
    {
        const double distance =
            std::hypot(
                raw_local[i].x,
                raw_local[i].y);

        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest = i;
        }
    }

    raw_lateral_error =
        raw_local[closest].y;

    double accumulated = 0.0;
    double weighted_optimized_y_sum = 0.0;
    double weight_sum = 0.0;

    const double preview_center =
        0.5
        * (patrol_pp_preview_start_
           + patrol_pp_preview_end_);

    const double preview_half_width =
        std::max(
            1.0e-3,
            0.5
            * (patrol_pp_preview_end_
               - patrol_pp_preview_start_));

    for (std::size_t i = closest;
         i < raw_local.size();
         ++i)
    {
        if (i > closest)
        {
            accumulated += std::hypot(
                raw_local[i].x
                    - raw_local[i - 1].x,
                raw_local[i].y
                    - raw_local[i - 1].y);
        }

        if (accumulated
            > patrol_pp_preview_end_)
        {
            break;
        }

        if (accumulated
            < patrol_pp_preview_start_)
        {
            continue;
        }

        const geometry_msgs::PoseStamped&
            optimized_pose =
                global_plan_[i];

        const std::string frame =
            optimized_pose.header.frame_id.empty()
                ? optimized_frame
                : optimized_pose.header.frame_id;

        double optimized_y = 0.0;

        if (frame == optimized_frame)
        {
            const tf::Vector3 source(
                optimized_pose.pose.position.x,
                optimized_pose.pose.position.y,
                0.0);

            const tf::Vector3 result =
                optimized_to_base * source;

            optimized_y = result.y();
        }
        else
        {
            geometry_msgs::PoseStamped point_base;

            if (!transformPose(
                    base_frame_,
                    optimized_pose,
                    point_base))
            {
                continue;
            }

            optimized_y =
                point_base.pose.position.y;
        }

        const double normalized_distance =
            std::abs(
                accumulated
                - preview_center)
            / preview_half_width;

        const double weight =
            std::max(
                0.50,
                1.0
                - 0.50
                * normalized_distance);

        // Patrol-R2直接对master-costmap C5 active path在车体坐标系中的y误差做预瞄加权。
        weighted_optimized_y_sum +=
            weight * optimized_y;
        weight_sum += weight;
    }

    if (weight_sum <= 1.0e-9)
        return false;

    optimized_lateral_error =
        weighted_optimized_y_sum
        / weight_sum;

    return std::isfinite(raw_lateral_error)
        && std::isfinite(
            optimized_lateral_error);
}


void MyPlanner::computePatrolPurePursuitCommand(
    const geometry_msgs::PoseStamped& target_pose,
    double control_dt,
    geometry_msgs::Twist& desired_cmd,
    double& raw_lateral_error,
    double& optimized_lateral_error,
    double& lateral_error)
{
    desired_cmd = geometry_msgs::Twist();

    if (!computePatrolPreviewErrors(
            raw_lateral_error,
            optimized_lateral_error))
    {
        // C5预瞄暂时不可用时，直接使用已经由
        // selectPatrolTrackingTarget()从global_plan_(C5 optimized path)
        // 取得的前视点y，仍然不回到旧raw-offset滤波模型。
        raw_lateral_error =
            target_pose.pose.position.y;
        optimized_lateral_error =
            target_pose.pose.position.y;
    }

    lateral_error =
        optimized_lateral_error;

    if (std::abs(lateral_error)
        < patrol_pp_lateral_deadband_)
    {
        lateral_error = 0.0;
    }

    double vy = clampValue(
        patrol_pp_lateral_gain_
            * lateral_error,
        -patrol_pp_max_vy_,
        patrol_pp_max_vy_);

    const double speed_cap =
        std::max(
            0.05,
            std::min(
                c2_max_reference_speed_,
                std::min(
                    mpc_max_translational_speed_,
                    max_vel_x_)));

    if (std::abs(vy) > speed_cap)
        vy = std::copysign(speed_cap, vy);

    const double vx_speed_circle =
        std::sqrt(
            std::max(
                0.0,
                speed_cap * speed_cap
                    - vy * vy));

    const double vx_pp =
        std::max(
            0.0,
            target_pose.pose.position.x
                * path_linear_x_gain_);

    desired_cmd.linear.x =
        std::min(
            vx_pp,
            vx_speed_circle);

    desired_cmd.linear.y = vy;
    desired_cmd.angular.z = 0.0;

    (void)control_dt;
    // Patrol-R2继续删除位置参考的二次offset滤波；
    // 横向动作平滑仍由applyPatrolVelocityLimits中的vy加速度限制负责。
    patrol_pp_filtered_offset_ = 0.0;
}

bool MyPlanner::computePatrolFinalPositionCommand(
    const geometry_msgs::PoseStamped& final_pose,
    geometry_msgs::Twist& desired_cmd)
{
    desired_cmd = geometry_msgs::Twist();
    const double x_error = final_pose.pose.position.x;
    const double y_error = final_pose.pose.position.y;
    const double distance_error = std::hypot(x_error, y_error);

    if (distance_error <= patrol_pp_goal_position_tolerance_)
    {
        goal_reached_ = true;
        transitionTo(
            ControlState::GOAL_HOLD,
            "巡检全向PP到达终点位置，忽略终点航向误差");
        ROS_WARN("巡检路线到达终点：位置误差=%.3fm，停止全部速度。",
                 distance_error);
        return true;
    }

    double vx = final_linear_x_gain_ * x_error;
    double vy = final_linear_y_gain_ * y_error;
    if (std::abs(x_error) <= patrol_pp_goal_position_tolerance_ * 0.65)
        vx = 0.0;
    if (std::abs(y_error) <= patrol_pp_goal_position_tolerance_ * 0.65)
        vy = 0.0;

    vx = applyMinimumMagnitude(vx, final_min_linear_speed_);
    vy = applyMinimumMagnitude(vy, final_min_linear_speed_);

    const double speed_cap = std::max(
        0.05,
        std::min(c2_max_reference_speed_,
                 std::min(mpc_max_translational_speed_, max_vel_x_)));
    desired_cmd.linear.x = clampValue(
        vx, -std::min(final_max_vel_x_, speed_cap),
        std::min(final_max_vel_x_, speed_cap));
    desired_cmd.linear.y = clampValue(
        vy, -std::min(patrol_pp_max_vy_, speed_cap),
        std::min(patrol_pp_max_vy_, speed_cap));

    const double speed = std::hypot(
        desired_cmd.linear.x, desired_cmd.linear.y);
    if (speed > speed_cap + 1.0e-9)
    {
        const double scale = speed_cap / speed;
        desired_cmd.linear.x *= scale;
        desired_cmd.linear.y *= scale;
    }
    desired_cmd.angular.z = 0.0;
    return false;
}

void MyPlanner::applyPatrolVelocityLimits(
    const geometry_msgs::Twist& desired_cmd,
    geometry_msgs::Twist& limited_cmd,
    double dt)
{
    geometry_msgs::Twist patrol_target = desired_cmd;

    // 巡检横移使用更保守的独立加速度上限，避免整车快速左右甩动。
    const double max_delta_y = patrol_pp_acc_lim_y_ * dt;
    patrol_target.linear.y = clampValue(
        patrol_target.linear.y,
        last_cmd_vel_.linear.y - max_delta_y,
        last_cmd_vel_.linear.y + max_delta_y);

    // 巡检航向保持使用独立的小角速度和角加速度限制。它只纠正底盘偏航，
    // 目标方向来自raw_plan_切线，不会追随优化路径的绕障弯曲。
    patrol_target.angular.z = clampValue(
        patrol_target.angular.z,
        -patrol_heading_max_wz_, patrol_heading_max_wz_);
    const double max_delta_wz = patrol_heading_acc_lim_ * dt;
    patrol_target.angular.z = clampValue(
        patrol_target.angular.z,
        last_cmd_vel_.angular.z - max_delta_wz,
        last_cmd_vel_.angular.z + max_delta_wz);

    applyVelocityAndAccelerationLimits(patrol_target, limited_cmd, dt);
}

bool MyPlanner::computeInitialRotationCommand(
    double angle_error,
    geometry_msgs::Twist& desired_cmd)
{
    desired_cmd = geometry_msgs::Twist();

    const bool patrol_pp_active = isPatrolPpActive();
    // 普通导航仍服从enable_initial_rotation_；固定巡检PP先快速旋转到
    // 约5度内，随后不停车，由行进中的raw_plan_航向闭环继续收敛。
    if (!enable_initial_rotation_ && !patrol_pp_active)
        return false;

    const double exit_tolerance = patrol_pp_active
        ? patrol_pp_align_tolerance_
        : initial_align_exit_angle_;

    if (std::abs(angle_error) <= exit_tolerance)
    {
        if (patrol_pp_active)
        {
            markPatrolPpAlignmentComplete();
            transitionTo(
                ControlState::PATH_TRACKING,
                "巡检车头已进入5度接管范围，立即开始行进中航向纠偏");
            ROS_WARN("巡检快速对准完成：不停车等待停稳，优化路径vx/vy与原始线航向保持立即接管。" );
        }
        else
        {
            transitionTo(
                ControlState::PATH_TRACKING,
                "车头已进入MPC可实现漂移角范围");
        }
        return false;
    }

    // V11：固定巡检与普通导航使用完全相同的纯比例初始姿态调整。
    // tolerance只负责判定“是否已经对准”；只要还在tolerance外，
    // 就通过普通导航已有的最小角速度跨过底盘死区。
    double angular_speed =
        std::abs(angle_error) * initial_angular_gain_;
    angular_speed =
        clampValue(angular_speed,
                   initial_min_angular_speed_,
                   initial_max_angular_speed_);

    desired_cmd.angular.z =
        std::copysign(angular_speed, angle_error);

    return true;
}

double MyPlanner::computePatrolHeadingHoldCommand(double angle_error) const
{
    if (!patrol_heading_hold_enabled_
        || std::abs(angle_error) <= patrol_heading_deadband_)
    {
        return 0.0;
    }

    double wz = clampValue(
        patrol_heading_kp_ * angle_error,
        -patrol_heading_max_wz_, patrol_heading_max_wz_);

    // V11：deadband外必须跨过底盘实际角速度死区。
    // 复用普通导航initial_min_angular_speed_，避免再维护一套重复参数。
    if (std::abs(wz) < initial_min_angular_speed_)
        wz = std::copysign(initial_min_angular_speed_, angle_error);

    return clampValue(
        wz, -patrol_heading_max_wz_, patrol_heading_max_wz_);
}

bool MyPlanner::computeFinalPoseCommand(
    const geometry_msgs::PoseStamped& final_pose,
    geometry_msgs::Twist& desired_cmd)
{
    desired_cmd = geometry_msgs::Twist();

    const double x_error = final_pose.pose.position.x;
    const double y_error = final_pose.pose.position.y;
    const double distance_error = std::hypot(x_error, y_error);
    const double yaw_error =
        normalizeAngle(tf::getYaw(final_pose.pose.orientation));

    if (distance_error <= goal_position_tolerance_
        && std::abs(yaw_error) <= goal_yaw_tolerance_)
    {
        goal_reached_ = true;
        transitionTo(
            ControlState::GOAL_HOLD,
            "C3.2原始终点位姿调整达到位姿阈值");

        ROS_WARN("到达终点：位置误差=%.3fm，角度误差=%.3frad。",
                 distance_error,
                 yaw_error);
        return true;
    }

    double vx = final_linear_x_gain_ * x_error;
    double vy = final_linear_y_gain_ * y_error;
    double wz = final_angular_gain_ * yaw_error;

    if (std::abs(x_error) <= goal_position_tolerance_ * 0.65)
        vx = 0.0;

    if (std::abs(y_error) <= goal_position_tolerance_ * 0.65)
        vy = 0.0;

    if (std::abs(yaw_error) <= goal_yaw_tolerance_)
        wz = 0.0;

    vx = applyMinimumMagnitude(vx, final_min_linear_speed_);
    vy = applyMinimumMagnitude(vy, final_min_linear_speed_);
    wz = applyMinimumMagnitude(wz, final_min_angular_speed_);

    desired_cmd.linear.x =
        clampValue(vx, -final_max_vel_x_, final_max_vel_x_);
    desired_cmd.linear.y =
        clampValue(vy, -final_max_vel_y_, final_max_vel_y_);
    desired_cmd.angular.z =
        clampValue(wz,
                   -final_max_vel_theta_,
                   final_max_vel_theta_);

    return false;
}

void MyPlanner::applyVelocityAndAccelerationLimits(
    const geometry_msgs::Twist& desired_cmd,
    geometry_msgs::Twist& limited_cmd,
    double dt)
{
    geometry_msgs::Twist target = desired_cmd;

    target.linear.x =
        clampValue(target.linear.x, -max_vel_x_, max_vel_x_);
    target.linear.y =
        clampValue(target.linear.y, -max_vel_y_, max_vel_y_);
    target.angular.z =
        clampValue(target.angular.z,
                   -max_vel_theta_,
                   max_vel_theta_);

    const double max_delta_x = std::max(0.0, acc_lim_x_) * dt;
    const double max_delta_y = std::max(0.0, acc_lim_y_) * dt;
    const double max_delta_theta =
        std::max(0.0, acc_lim_theta_) * dt;

    limited_cmd = geometry_msgs::Twist();

    limited_cmd.linear.x =
        clampValue(target.linear.x,
                   last_cmd_vel_.linear.x - max_delta_x,
                   last_cmd_vel_.linear.x + max_delta_x);

    limited_cmd.linear.y =
        clampValue(target.linear.y,
                   last_cmd_vel_.linear.y - max_delta_y,
                   last_cmd_vel_.linear.y + max_delta_y);

    limited_cmd.angular.z =
        clampValue(target.angular.z,
                   last_cmd_vel_.angular.z - max_delta_theta,
                   last_cmd_vel_.angular.z + max_delta_theta);

    last_cmd_vel_ = limited_cmd;
    recordPublishedCommand(limited_cmd, ros::Time::now());
}

bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();

    if (!initialized_ || costmap_ros_ == NULL)
        return false;

    // 所有运行时参数在路径处理和MPC求解前刷新，保证同一控制周期生效。
    refreshRuntimeParameters();

    bool patrol_pp_active = false;
    std::vector<geometry_msgs::PoseStamped> forced_patrol_plan;
    std::uint32_t forced_revision = 0;
    {
        // Patrol-R2：在同一把互斥锁内完成“是否巡检 + Reference快照”，避免路线切换时
        // lock/unlock与控制循环交错造成一帧状态不一致。
        std::lock_guard<std::mutex> lock(patrol_path_mutex_);
        patrol_pp_active = patrol_pp_enabled_
            && patrol_path_locked_
            && staged_patrol_plan_.size() >= 2;
        if (patrol_pp_active)
        {
            forced_patrol_plan = staged_patrol_plan_;
            forced_revision = patrol_path_revision_;
            active_plan_is_patrol_ = true;
        }
    }

    // Patrol-R2：固定巡检Reference具有最高路径所有权。只要锁开启且staged Path有效，
    // 每个控制周期都从staged_patrol_plan_重新取得权威直线Reference，并覆盖raw_plan_。
    // move_base即使下发一条沿墙体inflation外侧的弯曲GlobalPlanner路线，也只能
    // 作为控制循环“外壳”；真正的巡检Reference始终由staged Path决定。
    if (patrol_pp_active)
    {
        const bool revision_changed =
            forced_revision != applied_patrol_path_revision_;

        raw_plan_ = forced_patrol_plan;
        global_plan_ = forced_patrol_plan;
        goal_pose_ = forced_patrol_plan.back();
        has_goal_ = true;

        if (revision_changed)
        {
            applied_patrol_path_revision_ = forced_revision;
            target_index_ = 0;
            collision_replan_confirm_counter_ = 0;
            c5_failure_replan_confirm_counter_ = 0;
            c5_failure_replan_candidate_ = false;
            c5_failure_replan_reason_.clear();
            resetPatrolPpState();
            ROS_WARN(
                "Patrol-R2巡检Reference切换：修订号=%u，点数=%zu，"
                "raw/reference已强制使用理论直线；后续几何优化只交给C5。",
                static_cast<unsigned int>(forced_revision),
                forced_patrol_plan.size());
        }
    }

    if (raw_plan_.empty())
        return false;

    const ros::Time now = ros::Time::now();

    double dt = 0.05;
    if (!last_control_time_.isZero())
        dt = (now - last_control_time_).toSec();

    last_control_time_ = now;
    dt = clampValue(dt, 0.01, 0.20);

    if (controller_mode_ == "mpc"
        && std::abs(dt - mpc_dt_) > 0.015)
    {
        ROS_WARN_THROTTLE(
            2.0,
            "C4.0实际控制周期%.3fs与mpc_dt=%.3fs差异较大；"
            "建议controller_frequency与mpc_dt匹配。",
            dt, mpc_dt_);
    }

    // C5.4：最终位姿控制是独立控制阶段。一旦进入FINAL_SETTLING/GOAL_HOLD，
    // 不再运行C5或global replan，避免终点附近raw path的253/254、短窗口和
    // costmap边界把已经接管的C3.2最终位姿调整重新打回FAILURE_STOP。
    if (control_state_ == ControlState::GOAL_HOLD)
    {
        stopImmediately(cmd_vel);
        return true;
    }

    geometry_msgs::PoseStamped final_pose;
    if (!transformPose(base_frame_,
                       goal_pose_,
                       final_pose))
    {
        transitionTo(ControlState::FAILURE_STOP, "终点位姿TF失败");
        stopImmediately(cmd_vel);
        return false;
    }

    const double final_distance =
        std::hypot(final_pose.pose.position.x,
                   final_pose.pose.position.y);

    if (!patrol_pp_active
        && control_state_ != ControlState::FINAL_SETTLING
        && final_distance < goal_dist_threshold_)
    {
        transitionTo(
            ControlState::FINAL_SETTLING,
            "进入C3.2原始终点位姿调整");
        ROS_INFO("距离目标 %.3fm，进入终点位姿调整。",
                 final_distance);
    }

    geometry_msgs::Twist desired_cmd;

    if (!patrol_pp_active
        && control_state_ == ControlState::FINAL_SETTLING)
    {
        // FINAL_SETTLING阶段完全绕过C5与global replan，只运行原C3.2终点控制。
        collision_replan_confirm_counter_ = 0;
        c5_failure_replan_confirm_counter_ = 0;
        c5_failure_replan_candidate_ = false;
        c5_failure_replan_reason_.clear();

        if (computeFinalPoseCommand(final_pose, desired_cmd))
        {
            stopImmediately(cmd_vel);
            return true;
        }

        applyVelocityAndAccelerationLimits(
            desired_cmd, cmd_vel, dt);
        return true;
    }

    // 1. 路径链重新收敛：
    // 普通导航：move_base raw -> path healing -> master costmap C5 -> MPC。
    // 固定巡检：staged直线Reference ->【跳过path healing】-> master costmap C5
    //          -> optimized path -> Patrol PP。
    // 因而墙体inflation允许把optimized path推离墙面；但raw/reference本身永远保持理论直线。
    if (patrol_pp_active)
    {
        global_plan_ = raw_plan_;  // C5的输入Reference必须每周期从理论直线重新开始。
        target_index_ = std::max(
            0,
            std::min(target_index_,
                     static_cast<int>(global_plan_.size()) - 1));

        // 巡检明确跳过旧path healing，只运行唯一C5；C5直接读取master costmap，
        // 因此墙体inflation和赛道内真实路障都参与同一套连续优化。
        updateClearanceOptimizedPath();

        // 安全规则：巡检关闭global replan并不等于允许C5失败后退回raw直线硬冲。
        // 当C5明确确认fresh/HOLD/reuse均无安全active path时，本周期停车并等待下周期恢复。
        if (c5_failure_replan_candidate_)
        {
            transitionTo(
                ControlState::FAILURE_STOP,
                std::string("巡检C5无安全optimized path：") + c5_failure_replan_reason_);
            stopImmediately(cmd_vel);
            ROS_ERROR_THROTTLE(
                0.5,
                "[巡检C5][STOP] master costmap下无安全optimized path，reason=%s；"
                "本周期强制停车，绝不fallback到raw直线。",
                c5_failure_replan_reason_.c_str());
            return true;
        }

        // 仅用于终端判断“C5是否真的在改线”。raw和optimized点数在C5中保持一致。
        double maximum_deviation = 0.0;
        if (global_plan_.size() == raw_plan_.size())
        {
            for (std::size_t i = 0; i < global_plan_.size(); ++i)
            {
                const double dx = global_plan_[i].pose.position.x
                    - raw_plan_[i].pose.position.x;
                const double dy = global_plan_[i].pose.position.y
                    - raw_plan_[i].pose.position.y;
                maximum_deviation = std::max(
                    maximum_deviation,
                    std::hypot(dx, dy));
            }
        }

        if (maximum_deviation > 0.020)
        {
            ROS_WARN_THROTTLE(
                0.5,
                "[巡检C5][ACTIVE] Reference保持理论直线，C5正在使用master inflation优化；"
                "当前最大几何偏移=%.3fm，Patrol PP跟踪optimized path。",
                maximum_deviation);
        }
        else
        {
            ROS_INFO_THROTTLE(
                2.0,
                "[巡检C5][STRAIGHT] Reference为固定直线；当前C5偏移仅%.3fm。",
                maximum_deviation);
        }
    }
    else
    {
        // 普通move_base导航保持完整C5.4原逻辑。
        updateHealedPath();
        updateClearanceOptimizedPath();
    }

    // 2. 全局重规划只属于普通导航。
    // 巡检Reference是任务强制直线，即使raw直线穿过254也不能让GlobalPlanner把它改弯；
    // 真实障碍由上面的master-costmap C5局部优化处理，C5明确无安全解则停车等待恢复。
    if (!patrol_pp_active && enable_path_replanning_)
    {
        if (!checkPathCollision())
        {
            transitionTo(ControlState::FAILURE_STOP, "raw全局路径确认进入LETHAL");
            stopImmediately(cmd_vel);
            return false;
        }

        if (!checkC5FailureReplan())
        {
            transitionTo(ControlState::FAILURE_STOP, "C5连续无法提供安全active path");
            stopImmediately(cmd_vel);
            return false;
        }
    }
    else
    {
        // 固定巡检不请求GlobalPlanner重规划；普通导航关闭开关时也清确认状态。
        collision_replan_confirm_counter_ = 0;
        c5_failure_replan_confirm_counter_ = 0;
    }

    // V7续巡保护：对准要求使用独立闩锁，不依赖某一次setPlan是否成功
    // 把control_state_留在WAITING_FOR_PLAN。只要新固定Path尚未完成对准，
    // 任何被异步更新到PATH_TRACKING/GOAL_HOLD等状态的情况都会退回初始化。
    if (patrol_pp_active
        && isPatrolPpAlignmentRequired()
        && control_state_ != ControlState::WAITING_FOR_PLAN
        && control_state_ != ControlState::INITIAL_ALIGN
        && control_state_ != ControlState::PATROL_SETTLING)
    {
        resetPatrolPpState();
        transitionTo(
            ControlState::WAITING_FOR_PLAN,
            "新的或断点续巡固定Path尚未完成车头对准，强制重新初始化");
        ROS_WARN("巡检续巡对准闩锁生效：禁止带着停靠后的残余航向直接进入平移跟踪。");
    }

    // 3. 新目标只在确有必要时进入初始对准；依据路径切线而非前视点方位。
    if (control_state_ == ControlState::WAITING_FOR_PLAN)
    {
        double tangent_error = 0.0;
        if (!computeInitialPathTangentError(tangent_error))
        {
            transitionTo(ControlState::FAILURE_STOP, "无法计算初始路径切线");
            stopImmediately(cmd_vel);
            return false;
        }

        if (patrol_pp_active)
        {
            if (std::abs(tangent_error) > patrol_pp_align_tolerance_)
            {
                transitionTo(
                    ControlState::INITIAL_ALIGN,
                    "巡检车头需要转入原始固定路径的精确对准范围");
            }
            else
            {
                markPatrolPpAlignmentComplete();
                transitionTo(
                    ControlState::PATH_TRACKING,
                    "巡检车头已在精确对准范围内，直接开始路径跟踪");
                ROS_WARN("巡检无需额外停稳：当前航向已在%.1f度内，立即开始路径跟踪。",
                         patrol_pp_align_tolerance_ * 180.0 / M_PI);
            }
        }
        else if (shouldEnterInitialAlign(tangent_error))
        {
            transitionTo(
                ControlState::INITIAL_ALIGN,
                "初始车头与路径切线夹角超出MPC可接管范围");
        }
        else
        {
            transitionTo(
                ControlState::PATH_TRACKING,
                "初始姿态处于MPC可实现漂移角范围");
        }
    }

    if (control_state_ == ControlState::INITIAL_ALIGN)
    {
        double tangent_error = 0.0;
        if (!computeInitialPathTangentError(tangent_error))
        {
            transitionTo(ControlState::FAILURE_STOP, "初始对准期间路径切线不可用");
            stopImmediately(cmd_vel);
            return false;
        }

        if (computeInitialRotationCommand(tangent_error, desired_cmd))
        {
            applyVelocityAndAccelerationLimits(
                desired_cmd, cmd_vel, dt);
            return true;
        }
    }

    if (control_state_ == ControlState::PATROL_SETTLING)
    {
        // 兼容复位前残留的旧V9状态：V10不再执行停车精调阶段。
        if (patrol_pp_active)
            markPatrolPpAlignmentComplete();
        transitionTo(
            ControlState::PATH_TRACKING,
            "V10取消巡检停车精调，立即进入边走边航向保持");
    }

    if (control_state_ == ControlState::FAILURE_STOP)
        transitionTo(ControlState::PATH_TRACKING, "故障条件已消失，恢复路径跟踪");

    // 固定巡检Path专用分支：普通导航和停靠不会进入这里。
    // 终点仅以x/y判定到达，不跟踪move_base目标姿态；减速接近期间仍
    // 继续保持raw_plan_巡检航向。
    if (patrol_pp_active)
    {
        double patrol_heading_error = 0.0;
        double patrol_heading_wz = 0.0;
        if (computeInitialPathTangentError(patrol_heading_error))
        {
            patrol_heading_wz =
                computePatrolHeadingHoldCommand(patrol_heading_error);
        }
        else
        {
            ROS_WARN_THROTTLE(
                1.0,
                "巡检航向保持暂时无法取得原始路径切线，本周期wz置零。" );
        }

        if (final_distance < patrol_pp_goal_slowdown_distance_)
        {
            if (computePatrolFinalPositionCommand(final_pose, desired_cmd))
            {
                stopImmediately(cmd_vel);
                return true;
            }

            desired_cmd.angular.z = patrol_heading_wz;
            applyPatrolVelocityLimits(desired_cmd, cmd_vel, dt);
            ROS_INFO_THROTTLE(
                0.5,
                "巡检PP终点位置调整：distance=%.3f，航向误差=%.2f度，"
                "cmd=(%.3f,%.3f,%.3f)。",
                final_distance, patrol_heading_error * 180.0 / M_PI,
                cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            return true;
        }

        geometry_msgs::PoseStamped patrol_target;
        if (!selectPatrolTrackingTarget(patrol_target))
        {
            transitionTo(
                ControlState::FAILURE_STOP,
                "巡检PP无法取得前视点");
            stopImmediately(cmd_vel);
            return false;
        }

        double raw_lateral_error = 0.0;
        double optimized_lateral_error = 0.0;
        double lateral_error = 0.0;

        computePatrolPurePursuitCommand(
            patrol_target,
            dt,
            desired_cmd,
            raw_lateral_error,
            optimized_lateral_error,
            lateral_error);

        desired_cmd.angular.z =
            patrol_heading_wz;

        applyPatrolVelocityLimits(
            desired_cmd,
            cmd_vel,
            dt);

        ROS_INFO_THROTTLE(
            0.5,
            "巡检C5全向PP：raw_y=%.3f，optimized预瞄y=%.3f，"
            "实际横向误差=%.3f，航向误差=%.2f度，target_x=%.3f，"
            "cmd=(%.3f,%.3f,%.3f)，index=%d。",
            raw_lateral_error,
            optimized_lateral_error,
            lateral_error,
            patrol_heading_error * 180.0 / M_PI,
            patrol_target.pose.position.x,
            cmd_vel.linear.x,
            cmd_vel.linear.y,
            cmd_vel.angular.z,
            target_index_);
        return true;
    }

    // 5. 路径跟踪仍完整保留C4.0.3的MPC与PP回退。
    geometry_msgs::PoseStamped target_pose;
    if (!selectTrackingTarget(target_pose))
        target_pose = final_pose;

    double lateral_deviation = 0.0;
    geometry_msgs::Twist pp_cmd;
    computePurePursuitCommand(target_pose, pp_cmd, lateral_deviation);

    bool used_mpc = false;
    MpcSolveReport mpc_report;

    if (controller_mode_ == "mpc" && !mpc_locked_to_pp_)
    {
        if (computeMpcCommand(dt, desired_cmd, mpc_report))
        {
            used_mpc = true;
            mpc_consecutive_failures_ = 0;
        }
        else
        {
            ++mpc_consecutive_failures_;

            const bool terminal_braking_fallback =
                mpc_terminal_stop_enabled_
                && final_distance <= mpc_terminal_fallback_stop_distance_;
            if (terminal_braking_fallback)
            {
                desired_cmd = geometry_msgs::Twist();
            }
            else
            {
                desired_cmd = pp_cmd;
                desired_cmd.linear.x = clampValue(
                    desired_cmd.linear.x, 0.0, c2_max_reference_speed_);
                desired_cmd.linear.y = clampValue(
                    desired_cmd.linear.y, mpc_min_vy_, mpc_max_vy_);
                desired_cmd.angular.z = clampValue(
                    desired_cmd.angular.z, mpc_min_omega_, mpc_max_omega_);
            }

            ROS_WARN_THROTTLE(
                0.5,
                "C4.0 MPC本周期失败，%s：status=%s，reason=%s，连续失败=%d。",
                terminal_braking_fallback ? "终点区受控刹车" : "回退稳定PP",
                mpc_report.status.c_str(),
                mpc_report.failure_reason.c_str(),
                mpc_consecutive_failures_);

            if (!terminal_braking_fallback
                && mpc_lock_to_pp_after_failures_
                && mpc_consecutive_failures_ >= mpc_max_consecutive_failures_)
            {
                mpc_locked_to_pp_ = true;
                ROS_ERROR("C4.0 MPC连续失败%d次，本条全局路径普通段锁定PP；收到新路径后恢复MPC。",
                          mpc_consecutive_failures_);
            }
        }
    }
    else
    {
        desired_cmd = pp_cmd;
        if (controller_mode_ == "mpc")
        {
            desired_cmd.linear.x = clampValue(
                desired_cmd.linear.x, 0.0, c2_max_reference_speed_);
        }
    }

    // 与QP速度圆保持一致：即使回退PP或数值误差，也不允许vx/vy合速度超限。
    if (controller_mode_ == "mpc")
    {
        const double translational_speed = std::hypot(
            desired_cmd.linear.x, desired_cmd.linear.y);
        if (translational_speed > mpc_max_translational_speed_ + 1e-9)
        {
            const double scale = mpc_max_translational_speed_ / translational_speed;
            desired_cmd.linear.x *= scale;
            desired_cmd.linear.y *= scale;
        }
    }

    applyVelocityAndAccelerationLimits(desired_cmd, cmd_vel, dt);

    if (debug_log_)
    {
        if (used_mpc)
        {
            ROS_INFO_THROTTLE(
                0.5,
                "MPC-C4.0[%s]：ref0=(pos %.3f,%.3f; psi=%.3f chi=%.3f; "
                "vpath=%.3f beta0=%.1fdeg betaMax=%.1fdeg planMax=%.1fdeg; "
                "meas=(%.3f,%.3f,%.3f,%s age=%.3f) delay=(%.3f,%.3f,%.3f,%.3f); "
                "omegaSeed=%.3f guard=%d; u=%.3f,%.3f,%.3f omegaState=%.3f)，"
                "cmd=(%.3f,%.3f,%.3f)，k(track=%.3f preview=%.3f strength=%.2f)，"
                "vlimit=%.3f，preview=%.3fm，resampled=%d，"
                "solve=%.2fms total=%.2fms iter=%d status=%s，index=%d。",
                controlStateName(control_state_),
                mpc_report.first_reference.x,
                mpc_report.first_reference.y,
                mpc_report.first_reference.yaw,
                mpc_report.first_reference.motion_yaw,
                mpc_report.first_path_speed,
                mpc_report.first_drift_beta * 180.0 / M_PI,
                mpc_report.max_abs_drift_beta * 180.0 / M_PI,
                mpc_report.max_abs_planned_beta * 180.0 / M_PI,
                mpc_report.measured_vx,
                mpc_report.measured_vy,
                mpc_report.measured_omega,
                mpc_report.using_odometry ? "odom" : "cmd",
                mpc_report.odom_age,
                mpc_report.delay_x,
                mpc_report.delay_y,
                mpc_report.delay_yaw,
                mpc_report.delay_omega,
                mpc_report.initial_omega_seed,
                mpc_report.beta_speed_guard_steps,
                mpc_report.first_reference.vx,
                mpc_report.first_reference.vy,
                mpc_report.first_reference.omega_cmd,
                mpc_report.first_reference.omega,
                cmd_vel.linear.x,
                cmd_vel.linear.y,
                cmd_vel.angular.z,
                mpc_report.first_tracking_curvature,
                mpc_report.preview_abs_curvature,
                mpc_report.first_curve_strength,
                mpc_report.first_speed_limit,
                mpc_report.preview_distance,
                mpc_report.resampled_points,
                mpc_report.solve_ms,
                mpc_report.total_ms,
                mpc_report.iterations,
                mpc_report.status.c_str(),
                target_index_);
        }
        else
        {
            ROS_INFO_THROTTLE(
                0.5,
                "MPC-C4.0[%s]回退/刹车：target=(%.3f,%.3f)，lateral=%.3f，"
                "desired=(%.3f,%.3f,%.3f)，cmd=(%.3f,%.3f,%.3f)，locked=%s，index=%d。",
                controlStateName(control_state_),
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                lateral_deviation,
                desired_cmd.linear.x,
                desired_cmd.linear.y,
                desired_cmd.angular.z,
                cmd_vel.linear.x,
                cmd_vel.linear.y,
                cmd_vel.angular.z,
                mpc_locked_to_pp_ ? "是" : "否",
                target_index_);
        }
    }

    return true;
}

bool MyPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace my_planner
