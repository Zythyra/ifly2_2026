// 交付构建标识：MYPLANNER_MPC_C3_2_MEASURED_STATE_DELAY_PROGRESS_20260731
#include "my_planner.h"

#include <pluginlib/class_list_macros.h>
#include <boost/thread/locks.hpp>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <clocale>
#include <cmath>
#include <stdexcept>

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner
{

MyPlanner::MyPlanner()
    : initialized_(false),
      tf_listener_(NULL),
      costmap_ros_(NULL),
      has_goal_(false),
      target_index_(0),
      pose_adjusting_(false),
      goal_reached_(false),
      initial_rotation_done_(false),
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

    // -------------------------------------------------------------------------
    // 保留前方全局路径碰撞检查；命中障碍后返回 false 触发全局重规划
    // -------------------------------------------------------------------------
    private_nh.param("collision_check_lookahead_points",
                     collision_check_lookahead_points_, 10);

    int collision_cost_threshold = 253;
    private_nh.param("collision_cost_threshold",
                     collision_cost_threshold, 253);
    collision_cost_threshold =
        std::max(1, std::min(collision_cost_threshold, 255));
    collision_cost_threshold_ =
        static_cast<unsigned char>(collision_cost_threshold);

    // -------------------------------------------------------------------------
    // 初始姿态调整
    // -------------------------------------------------------------------------
    private_nh.param("enable_initial_rotation",
                     enable_initial_rotation_, true);
    private_nh.param("initial_yaw_tolerance",
                     initial_yaw_tolerance_, 0.10);
    private_nh.param("initial_angular_gain",
                     initial_angular_gain_, 2.00);
    private_nh.param("initial_min_angular_speed",
                     initial_min_angular_speed_, 0.10);
    private_nh.param("initial_max_angular_speed",
                     initial_max_angular_speed_, 0.30);

    // -------------------------------------------------------------------------
    // 终点位姿调整
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
    private_nh.param("mpc_min_reference_length", mpc_min_reference_length_, 0.25);
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
    collision_check_lookahead_points_ =
        std::max(1, collision_check_lookahead_points_);
    lateral_search_points_ = std::max(0, lateral_search_points_);

    if (controller_mode_ != "mpc" && controller_mode_ != "pp")
    {
        ROS_WARN("未知controller_mode=%s，自动使用mpc。", controller_mode_.c_str());
        controller_mode_ = "mpc";
    }

    mpc_horizon_steps_ = std::max(5, mpc_horizon_steps_);
    mpc_dt_ = clampValue(mpc_dt_, 0.02, 0.20);
    mpc_min_reference_length_ = std::max(0.05, mpc_min_reference_length_);
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


    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    initial_rotation_done_ = !enable_initial_rotation_;
    last_cmd_vel_ = geometry_msgs::Twist();
    measured_body_twist_ = geometry_msgs::Twist();
    last_odom_stamp_ = ros::Time(0);
    odom_received_ = false;
    omega_filter_state_ = ButterworthFilterState();
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

    initialized_ = true;

    ROS_WARN("MyPlanner MPC-C3.2 MEASURED-STATE-DELAY-PROGRESS 启动："
             "mode=%s，路径合速度=%.2f~%.2fm/s，N=%d，dt=%.3fs；"
             "odom=%s topic=%s timeout=%.2fs，input_delay=%.3fs，tau(v/w)=%.3f/%.3fs；"
             "omegaRate(acc/dec/rev)=%.1f/%.1f/%.1f，curvatureFF=%.2f，"
             "progressWeight=%.2f，速度圆=%.2fm/s(%d边)。base=%s，costmap=%s。",
             controller_mode_.c_str(), c2_min_curve_speed_, c2_max_reference_speed_,
             mpc_horizon_steps_, mpc_dt_,
             c3_use_odometry_ ? "启用" : "关闭", c3_odom_topic_.c_str(),
             c3_odom_timeout_, c3_input_delay_,
             c3_translational_response_tau_, c3_angular_response_tau_,
             c3_reference_omega_accel_rate_, c3_reference_omega_decel_rate_,
             c3_reference_omega_reverse_rate_,
             c3_omega_curvature_feedforward_gain_, mpc_weight_progress_,
             mpc_max_translational_speed_, mpc_velocity_polygon_sides_,
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

void MyPlanner::resetForNewGoal()
{
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    initial_rotation_done_ = !enable_initial_rotation_;
    last_cmd_vel_ = geometry_msgs::Twist();
    last_control_time_ = ros::Time(0);
    resetMpcState();
}

void MyPlanner::stopImmediately(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    last_cmd_vel_ = geometry_msgs::Twist();
    recordPublishedCommand(last_cmd_vel_, ros::Time::now());
}

bool MyPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (plan.empty())
    {
        ROS_ERROR("MyPlanner 收到空路径。");
        return false;
    }

    const bool new_goal = isNewGoal(plan.back());

    raw_plan_ = plan;
    global_plan_ = plan;
    goal_pose_ = plan.back();
    has_goal_ = true;

    // 全局规划器每次更新路径后，从新路径起点重新搜索前视点。
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    resetMpcState();

    if (new_goal)
    {
        resetForNewGoal();
        ROS_INFO("收到新目标，状态已重置；路径点数：%zu。", plan.size());
    }
    else
    {
        ROS_INFO("同一目标的全局路径已更新；路径点数：%zu。", plan.size());
    }

    return true;
}

void MyPlanner::updateHealedPath()
{
    if (raw_plan_.empty())
        return;

    // 每个控制周期从原始路径重新开始，避免路径点累计漂移。
    global_plan_ = raw_plan_;

    if (!enable_path_healing_ || path_healing_iterations_ <= 0)
        return;

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (costmap == NULL)
        return;

    target_index_ = std::max(
        0,
        std::min(target_index_,
                 static_cast<int>(global_plan_.size()) - 1));

    const int heal_start =
        std::max(0, target_index_ - path_healing_points_behind_);
    const int heal_end =
        std::min(static_cast<int>(global_plan_.size()),
                 target_index_ + path_healing_points_ahead_);

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
}

bool MyPlanner::checkPathCollision()
{
    if (global_plan_.empty())
        return false;

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (costmap == NULL)
        return false;

    target_index_ = std::max(
        0,
        std::min(target_index_,
                 static_cast<int>(global_plan_.size()) - 1));

    const int check_end =
        std::min(target_index_ + collision_check_lookahead_points_,
                 static_cast<int>(global_plan_.size()));

    for (int i = target_index_; i < check_end; ++i)
    {
        geometry_msgs::PoseStamped point_costmap;
        if (!transformPose(costmap_frame_,
                           global_plan_[i],
                           point_costmap))
        {
            continue;
        }

        unsigned int mx = 0;
        unsigned int my = 0;

        if (!costmap->worldToMap(point_costmap.pose.position.x,
                                 point_costmap.pose.position.y,
                                 mx,
                                 my))
        {
            continue;
        }

        unsigned char cost = 0;
        {
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
                lock(*(costmap->getMutex()));
            cost = costmap->getCost(mx, my);
        }

        if (cost >= collision_cost_threshold_)
        {
            ROS_WARN("前方全局路径检测到障碍物，停止并请求重新规划。"
                     "index=%d，cost=%u。",
                     i,
                     static_cast<unsigned int>(cost));
            return false;
        }
    }

    return true;
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

bool MyPlanner::computeInitialRotationCommand(
    const geometry_msgs::PoseStamped& target_pose,
    geometry_msgs::Twist& desired_cmd)
{
    desired_cmd = geometry_msgs::Twist();

    if (!enable_initial_rotation_)
    {
        initial_rotation_done_ = true;
        return false;
    }

    const double angle_to_target =
        std::atan2(target_pose.pose.position.y,
                   target_pose.pose.position.x);

    if (std::abs(angle_to_target) < initial_yaw_tolerance_)
    {
        initial_rotation_done_ = true;
        ROS_INFO("初始姿态已对准前视点，开始基础 PP 跟踪。");
        return false;
    }

    double angular_speed =
        std::abs(angle_to_target) * initial_angular_gain_;

    angular_speed =
        clampValue(angular_speed,
                   initial_min_angular_speed_,
                   initial_max_angular_speed_);

    desired_cmd.angular.z =
        std::copysign(angular_speed, angle_to_target);

    return true;
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

    if (!initialized_
        || raw_plan_.empty()
        || costmap_ros_ == NULL)
    {
        return false;
    }

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
            "C3.2实际控制周期%.3fs与mpc_dt=%.3fs差异较大；"
            "建议controller_frequency与mpc_dt匹配。",
            dt, mpc_dt_);
    }

    // 1. 每周期从原始路径重新生成非累积治愈路径。
    updateHealedPath();

    // 2. 前方路径有障碍时停车并返回 false，保留全局重规划机制。
    if (!checkPathCollision())
    {
        stopImmediately(cmd_vel);
        return false;
    }

    // 3. 终点附近进入独立位姿调整。
    geometry_msgs::PoseStamped final_pose;

    if (!transformPose(base_frame_,
                       global_plan_.back(),
                       final_pose))
    {
        stopImmediately(cmd_vel);
        return false;
    }

    const double final_distance =
        std::hypot(final_pose.pose.position.x,
                   final_pose.pose.position.y);

    if (!pose_adjusting_
        && final_distance < goal_dist_threshold_)
    {
        pose_adjusting_ = true;

        ROS_INFO("距离目标 %.3fm，进入终点位姿调整。",
                 final_distance);
    }

    geometry_msgs::Twist desired_cmd;

    if (pose_adjusting_)
    {
        if (computeFinalPoseCommand(final_pose, desired_cmd))
        {
            stopImmediately(cmd_vel);
            return true;
        }

        applyVelocityAndAccelerationLimits(
            desired_cmd, cmd_vel, dt);
        return true;
    }

    // 4. 选择第一个距离车体超过 lookahead_dist 的路径点。
    geometry_msgs::PoseStamped target_pose;

    if (!selectTrackingTarget(target_pose))
    {
        stopImmediately(cmd_vel);
        return false;
    }

    // 5. 新目标开始时，保留现有的初始姿态调整层。
    if (!initial_rotation_done_)
    {
        if (computeInitialRotationCommand(target_pose, desired_cmd))
        {
            applyVelocityAndAccelerationLimits(
                desired_cmd, cmd_vel, dt);
            return true;
        }
    }

    // 6. 先生成稳定PP命令，作为MPC失败时的无缝回退。
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

            // C3中PP回退限制到最高路径合速度，防止单次QP失败突然跳回高速PP。
            desired_cmd = pp_cmd;
            desired_cmd.linear.x = clampValue(
                desired_cmd.linear.x, 0.0, c2_max_reference_speed_);
            desired_cmd.linear.y = clampValue(
                desired_cmd.linear.y, mpc_min_vy_, mpc_max_vy_);
            desired_cmd.angular.z = clampValue(
                desired_cmd.angular.z, mpc_min_omega_, mpc_max_omega_);

            ROS_WARN_THROTTLE(
                0.5,
                "C3.2 MPC本周期失败，回退稳定PP：status=%s，reason=%s，连续失败=%d。",
                mpc_report.status.c_str(),
                mpc_report.failure_reason.c_str(),
                mpc_consecutive_failures_);

            if (mpc_lock_to_pp_after_failures_
                && mpc_consecutive_failures_ >= mpc_max_consecutive_failures_)
            {
                mpc_locked_to_pp_ = true;
                ROS_ERROR("C3.2 MPC连续失败%d次，本条全局路径剩余过程锁定PP；收到新路径后恢复MPC。",
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
                "MPC-C3.2：ref0=(pos %.3f,%.3f; psi=%.3f chi=%.3f; "
                "vpath=%.3f beta0=%.1fdeg betaMax=%.1fdeg planMax=%.1fdeg; "
                "meas=(%.3f,%.3f,%.3f,%s age=%.3f) delay=(%.3f,%.3f,%.3f,%.3f); "
                "omegaSeed=%.3f guard=%d; u=%.3f,%.3f,%.3f omegaState=%.3f)，"
                "cmd=(%.3f,%.3f,%.3f)，k(track=%.3f preview=%.3f strength=%.2f)，"
                "vlimit=%.3f，preview=%.3fm，resampled=%d，"
                "solve=%.2fms total=%.2fms iter=%d status=%s，index=%d。",
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
                "MPC-C3.2回退PP：target=(%.3f,%.3f)，lateral=%.3f，"
                "desired=(%.3f,%.3f,%.3f)，cmd=(%.3f,%.3f,%.3f)，locked=%s，index=%d。",
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