// 交付构建标识：MYPLANNER_PP_C0_5_STABLE_PP_FIXED_HALF_X_BRAKE_20260726
#include "my_planner.h"

#include <pluginlib/class_list_macros.h>
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
      has_goal_(false),
      target_index_(0),
      pose_adjusting_(false),
      goal_reached_(false),
      initial_rotation_done_(false),
      lookahead_dist_(0.26),
      path_linear_x_gain_(2.20),
      path_linear_y_gain_(2.00),
      path_angular_y_gain_(10.00),
      lateral_search_points_(10),
      enable_path_healing_(true),
      path_healing_points_behind_(5),
      path_healing_points_ahead_(60),
      path_healing_iterations_(3),
      path_healing_max_step_(0.01),
      path_healing_gradient_deadband_(4.0),
      path_healing_gradient_scale_(20.0),
      collision_check_lookahead_points_(20),
      collision_cost_threshold_(253),
      enable_initial_rotation_(true),
      initial_yaw_tolerance_(0.10),
      initial_angular_gain_(8.0),
      initial_min_angular_speed_(0.10),
      initial_max_angular_speed_(1.40),
      goal_dist_threshold_(0.50),
      goal_position_tolerance_(0.025),
      goal_yaw_tolerance_(0.05),
      final_linear_x_gain_(1.60),
      final_linear_y_gain_(1.20),
      final_angular_gain_(6.50),
      final_min_linear_speed_(0.03),
      final_min_angular_speed_(0.08),
      final_max_vel_x_(0.30),
      final_max_vel_y_(0.20),
      final_max_vel_theta_(0.90),
      max_vel_x_(0.80),
      max_vel_y_(0.55),
      max_vel_theta_(4.00),
      acc_lim_x_(8.00),
      acc_lim_y_(8.00),
      acc_lim_theta_(15.00),
      debug_log_(true),
      pp_safety_enable_(true),
      pp_safety_snapshot_max_age_(0.80),
      pp_safety_prediction_dt_(0.05),
      pp_safety_prediction_horizon_(0.20),
      pp_x_brake_margin_threshold_(-0.01),
      pp_safety_footprint_margin_(0.005),
      pp_x_brake_min_outside_steps_(2),
      pp_x_brake_scale_(0.50),
      pp_x_brake_enter_cycles_(2),
      pp_x_brake_exit_cycles_(3),
      pp_x_brake_unsafe_count_(0),
      pp_x_brake_safe_count_(0),
      safety_mode_(SAFETY_DISABLED),
      enable_corridor_visualization_(true),
      corridor_update_frequency_(2.0),
      local_path_behind_distance_(0.10),
      local_path_horizon_distance_(1.10),
      local_path_resample_distance_(0.03),
      corridor_skeleton_corner_angle_deg_(20.0),
      corridor_skeleton_corner_window_(0.06),
      corridor_skeleton_max_segment_length_(0.15),
      corridor_skeleton_min_segment_length_(0.06),
      corridor_initial_half_width_(0.35),
      corridor_longitudinal_extension_(0.12),
      corridor_post_shrink_longitudinal_reserve_(0.08),
      corridor_map_boundary_margin_(0.01),
      corridor_hard_cost_threshold_(254),
      corridor_treat_unknown_as_obstacle_(true),
      corridor_obstacle_padding_(0.0),
      corridor_max_obstacle_cuts_(120),
      corridor_min_polygon_area_(0.0005),
      corridor_use_costmap_footprint_(true),
      corridor_robot_half_length_(0.17),
      corridor_robot_half_width_(0.13),
      corridor_extra_margin_(0.015),
      corridor_min_overlap_area_(0.0025),
      corridor_reference_validation_step_(0.01),
      corridor_min_usable_chain_length_(0.35),
      corridor_preferred_chain_length_(0.75),
      corridor_terminal_ignore_distance_(0.10),
      corridor_projection_search_behind_points_(120),
      corridor_projection_search_ahead_points_(500),
      corridor_debug_log_(true),
      corridor_reference_revision_(0),
      force_corridor_update_(true),
      corridor_worker_stop_(false),
      corridor_update_requested_(false),
      corridor_clear_requested_(false),
      corridor_worker_busy_(false),
      corridor_visualization_cleared_(true),
      corridor_plan_generation_(0),
      corridor_progress_segment_index_(0),
      corridor_worker_seen_generation_(0)
{
    setlocale(LC_ALL, "");
}

MyPlanner::~MyPlanner()
{
    stopCorridorWorker();

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

    // 最早稳定版PP参数。
    private_nh.param("lookahead_dist", lookahead_dist_, 0.26);
    private_nh.param("path_linear_x_gain", path_linear_x_gain_, 2.20);
    private_nh.param("path_linear_y_gain", path_linear_y_gain_, 2.00);
    private_nh.param("path_angular_y_gain", path_angular_y_gain_, 10.00);
    private_nh.param("lateral_search_points", lateral_search_points_, 10);

    private_nh.param("enable_path_healing", enable_path_healing_, true);
    private_nh.param("path_healing_points_behind", path_healing_points_behind_, 5);
    private_nh.param("path_healing_points_ahead", path_healing_points_ahead_, 60);
    private_nh.param("path_healing_iterations", path_healing_iterations_, 3);
    private_nh.param("path_healing_max_step", path_healing_max_step_, 0.01);
    private_nh.param("path_healing_gradient_deadband", path_healing_gradient_deadband_, 4.0);
    private_nh.param("path_healing_gradient_scale", path_healing_gradient_scale_, 20.0);

    private_nh.param("collision_check_lookahead_points", collision_check_lookahead_points_, 20);
    int collision_cost_threshold = 253;
    private_nh.param("collision_cost_threshold", collision_cost_threshold, 253);
    collision_cost_threshold_ = static_cast<unsigned char>(
        std::max(1, std::min(collision_cost_threshold, 255)));

    private_nh.param("enable_initial_rotation", enable_initial_rotation_, true);
    private_nh.param("initial_yaw_tolerance", initial_yaw_tolerance_, 0.10);
    private_nh.param("initial_angular_gain", initial_angular_gain_, 8.00);
    private_nh.param("initial_min_angular_speed", initial_min_angular_speed_, 0.10);
    private_nh.param("initial_max_angular_speed", initial_max_angular_speed_, 1.40);

    private_nh.param("goal_dist_threshold", goal_dist_threshold_, 0.50);
    private_nh.param("goal_position_tolerance", goal_position_tolerance_, 0.025);
    private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.05);
    private_nh.param("final_linear_x_gain", final_linear_x_gain_, 1.60);
    private_nh.param("final_linear_y_gain", final_linear_y_gain_, 1.20);
    private_nh.param("final_angular_gain", final_angular_gain_, 6.50);
    private_nh.param("final_min_linear_speed", final_min_linear_speed_, 0.03);
    private_nh.param("final_min_angular_speed", final_min_angular_speed_, 0.08);
    private_nh.param("final_max_vel_x", final_max_vel_x_, 0.30);
    private_nh.param("final_max_vel_y", final_max_vel_y_, 0.20);
    private_nh.param("final_max_vel_theta", final_max_vel_theta_, 0.90);

    private_nh.param("max_vel_x", max_vel_x_, 0.80);
    private_nh.param("max_vel_y", max_vel_y_, 0.55);
    private_nh.param("max_vel_theta", max_vel_theta_, 4.00);
    private_nh.param("acc_lim_x", acc_lim_x_, 8.00);
    private_nh.param("acc_lim_y", acc_lim_y_, 8.00);
    private_nh.param("acc_lim_theta", acc_lim_theta_, 15.00);

    // 简化固定倍率X抑制器：只判断名义命令，触发后固定将linear.x乘0.5。
    private_nh.param("pp_safety_enable", pp_safety_enable_, true);
    private_nh.param("pp_safety_snapshot_max_age", pp_safety_snapshot_max_age_, 0.80);
    private_nh.param("pp_safety_prediction_dt", pp_safety_prediction_dt_, 0.05);
    private_nh.param("pp_safety_prediction_horizon", pp_safety_prediction_horizon_, 0.20);
    private_nh.param("pp_x_brake_margin_threshold", pp_x_brake_margin_threshold_, -0.01);
    private_nh.param("pp_safety_footprint_margin", pp_safety_footprint_margin_, 0.005);
    private_nh.param("pp_x_brake_min_outside_steps", pp_x_brake_min_outside_steps_, 2);
    private_nh.param("pp_x_brake_scale", pp_x_brake_scale_, 0.50);
    private_nh.param("pp_x_brake_enter_cycles", pp_x_brake_enter_cycles_, 2);
    private_nh.param("pp_x_brake_exit_cycles", pp_x_brake_exit_cycles_, 3);

    // 异步安全框参数。安全框只用于检测，不改变PP跟踪目标。
    private_nh.param("enable_corridor_visualization", enable_corridor_visualization_, true);
    private_nh.param("corridor_update_frequency", corridor_update_frequency_, 2.0);
    private_nh.param("local_path_behind_distance", local_path_behind_distance_, 0.10);
    private_nh.param("local_path_horizon_distance", local_path_horizon_distance_, 1.10);
    private_nh.param("local_path_resample_distance", local_path_resample_distance_, 0.03);
    private_nh.param("corridor_skeleton_corner_angle_deg", corridor_skeleton_corner_angle_deg_, 20.0);
    private_nh.param("corridor_skeleton_corner_window", corridor_skeleton_corner_window_, 0.06);
    private_nh.param("corridor_skeleton_max_segment_length", corridor_skeleton_max_segment_length_, 0.15);
    private_nh.param("corridor_skeleton_min_segment_length", corridor_skeleton_min_segment_length_, 0.06);
    private_nh.param("corridor_initial_half_width", corridor_initial_half_width_, 0.35);
    private_nh.param("corridor_longitudinal_extension", corridor_longitudinal_extension_, 0.12);
    private_nh.param("corridor_post_shrink_longitudinal_reserve", corridor_post_shrink_longitudinal_reserve_, 0.08);
    private_nh.param("corridor_map_boundary_margin", corridor_map_boundary_margin_, 0.01);

    int hard_cost_threshold = 254;
    private_nh.param("corridor_hard_cost_threshold", hard_cost_threshold, 254);
    corridor_hard_cost_threshold_ = static_cast<unsigned char>(
        std::max(1, std::min(hard_cost_threshold, 255)));

    private_nh.param("corridor_treat_unknown_as_obstacle", corridor_treat_unknown_as_obstacle_, true);
    private_nh.param("corridor_obstacle_padding", corridor_obstacle_padding_, 0.0);
    private_nh.param("corridor_max_obstacle_cuts", corridor_max_obstacle_cuts_, 120);
    private_nh.param("corridor_min_polygon_area", corridor_min_polygon_area_, 0.0005);
    private_nh.param("corridor_use_costmap_footprint", corridor_use_costmap_footprint_, true);
    private_nh.param("corridor_robot_half_length", corridor_robot_half_length_, 0.17);
    private_nh.param("corridor_robot_half_width", corridor_robot_half_width_, 0.13);
    private_nh.param("corridor_extra_margin", corridor_extra_margin_, 0.015);
    private_nh.param("corridor_min_overlap_area", corridor_min_overlap_area_, 0.0025);
    private_nh.param("corridor_reference_validation_step", corridor_reference_validation_step_, 0.01);
    private_nh.param("corridor_min_usable_chain_length", corridor_min_usable_chain_length_, 0.35);
    private_nh.param("corridor_preferred_chain_length", corridor_preferred_chain_length_, 0.75);
    private_nh.param("corridor_terminal_ignore_distance", corridor_terminal_ignore_distance_, 0.10);
    private_nh.param("corridor_projection_search_behind_points", corridor_projection_search_behind_points_, 120);
    private_nh.param("corridor_projection_search_ahead_points", corridor_projection_search_ahead_points_, 500);
    private_nh.param("corridor_debug_log", corridor_debug_log_, true);
    private_nh.param("debug_log", debug_log_, true);

    lookahead_dist_ = std::max(0.01, lookahead_dist_);
    lateral_search_points_ = std::max(0, lateral_search_points_);
    path_healing_points_behind_ = std::max(0, path_healing_points_behind_);
    path_healing_points_ahead_ = std::max(1, path_healing_points_ahead_);
    path_healing_iterations_ = std::max(0, path_healing_iterations_);
    path_healing_gradient_scale_ = std::max(1e-6, path_healing_gradient_scale_);
    collision_check_lookahead_points_ = std::max(1, collision_check_lookahead_points_);

    pp_safety_snapshot_max_age_ = std::max(0.10, pp_safety_snapshot_max_age_);
    pp_safety_prediction_dt_ = clampValue(pp_safety_prediction_dt_, 0.01, 0.10);
    pp_safety_prediction_horizon_ = std::max(pp_safety_prediction_dt_, pp_safety_prediction_horizon_);
    pp_safety_footprint_margin_ = std::max(0.0, pp_safety_footprint_margin_);
    pp_x_brake_min_outside_steps_ = std::max(1, pp_x_brake_min_outside_steps_);
    // 用户要求最多只削减到0.5倍率，因此参数下限强制为0.5。
    pp_x_brake_scale_ = clampValue(pp_x_brake_scale_, 0.50, 1.00);
    pp_x_brake_enter_cycles_ = std::max(1, pp_x_brake_enter_cycles_);
    pp_x_brake_exit_cycles_ = std::max(1, pp_x_brake_exit_cycles_);

    corridor_update_frequency_ = std::max(0.1, corridor_update_frequency_);
    local_path_behind_distance_ = std::max(0.0, local_path_behind_distance_);
    local_path_horizon_distance_ = std::max(0.20, local_path_horizon_distance_);
    local_path_resample_distance_ = std::max(0.01, local_path_resample_distance_);
    corridor_skeleton_corner_angle_deg_ = clampValue(corridor_skeleton_corner_angle_deg_, 1.0, 170.0);
    corridor_skeleton_corner_window_ = std::max(local_path_resample_distance_, corridor_skeleton_corner_window_);
    corridor_skeleton_min_segment_length_ = std::max(local_path_resample_distance_, corridor_skeleton_min_segment_length_);
    corridor_skeleton_max_segment_length_ = std::max(corridor_skeleton_min_segment_length_, corridor_skeleton_max_segment_length_);
    corridor_initial_half_width_ = std::max(0.05, corridor_initial_half_width_);
    corridor_longitudinal_extension_ = std::max(0.0, corridor_longitudinal_extension_);
    corridor_post_shrink_longitudinal_reserve_ = std::max(0.02, corridor_post_shrink_longitudinal_reserve_);
    corridor_map_boundary_margin_ = std::max(0.0, corridor_map_boundary_margin_);
    corridor_obstacle_padding_ = std::max(0.0, corridor_obstacle_padding_);
    corridor_max_obstacle_cuts_ = std::max(1, corridor_max_obstacle_cuts_);
    corridor_min_polygon_area_ = std::max(1e-6, corridor_min_polygon_area_);
    corridor_robot_half_length_ = std::max(0.01, corridor_robot_half_length_);
    corridor_robot_half_width_ = std::max(0.01, corridor_robot_half_width_);
    corridor_extra_margin_ = std::max(0.0, corridor_extra_margin_);
    corridor_min_overlap_area_ = std::max(0.0, corridor_min_overlap_area_);
    corridor_reference_validation_step_ = std::max(0.005, corridor_reference_validation_step_);
    corridor_min_usable_chain_length_ = std::max(0.05, corridor_min_usable_chain_length_);
    corridor_preferred_chain_length_ = std::max(corridor_min_usable_chain_length_, corridor_preferred_chain_length_);
    corridor_terminal_ignore_distance_ = std::max(0.02, corridor_terminal_ignore_distance_);
    corridor_projection_search_behind_points_ = std::max(1, corridor_projection_search_behind_points_);
    corridor_projection_search_ahead_points_ = std::max(10, corridor_projection_search_ahead_points_);

    if (pp_safety_enable_ && !enable_corridor_visualization_)
    {
        ROS_WARN("仅X减速安全盾依赖异步安全框，已自动启用安全框worker。");
        enable_corridor_visualization_ = true;
    }

    corridor_robot_footprint_.clear();
    if (corridor_use_costmap_footprint_)
    {
        const std::vector<geometry_msgs::Point> footprint =
            costmap_ros_->getRobotFootprint();
        if (footprint.size() >= 3)
        {
            double half_length = 0.0;
            double half_width = 0.0;
            for (std::size_t i = 0; i < footprint.size(); ++i)
            {
                corridor_robot_footprint_.push_back(
                    Point2D(footprint[i].x, footprint[i].y));
                half_length = std::max(half_length, std::abs(footprint[i].x));
                half_width = std::max(half_width, std::abs(footprint[i].y));
            }
            if (half_length > 0.01 && half_width > 0.01)
            {
                corridor_robot_half_length_ = half_length;
                corridor_robot_half_width_ = half_width;
            }
            else
            {
                corridor_robot_footprint_.clear();
            }
        }
    }

    if (corridor_robot_footprint_.size() < 3)
    {
        corridor_robot_footprint_.clear();
        corridor_robot_footprint_.push_back(Point2D( corridor_robot_half_length_,  corridor_robot_half_width_));
        corridor_robot_footprint_.push_back(Point2D( corridor_robot_half_length_, -corridor_robot_half_width_));
        corridor_robot_footprint_.push_back(Point2D(-corridor_robot_half_length_, -corridor_robot_half_width_));
        corridor_robot_footprint_.push_back(Point2D(-corridor_robot_half_length_,  corridor_robot_half_width_));
    }

    corridor_markers_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("safe_corridors", 1, true);
    corridor_reference_path_pub_ = private_nh.advertise<nav_msgs::Path>("safety_reference_path", 1, true);
    pp_raw_prediction_pub_ = private_nh.advertise<nav_msgs::Path>("pp_raw_prediction", 1, false);
    pp_safe_prediction_pub_ = private_nh.advertise<nav_msgs::Path>("pp_safe_prediction", 1, false);
    pp_unsafe_footprint_pub_ = private_nh.advertise<visualization_msgs::Marker>("pp_unsafe_footprint", 1, false);

    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    initial_rotation_done_ = !enable_initial_rotation_;
    last_cmd_vel_ = geometry_msgs::Twist();
    last_control_time_ = ros::Time(0);
    last_corridor_request_time_ = ros::Time(0);
    force_corridor_update_ = true;
    pp_x_brake_unsafe_count_ = 0;
    pp_x_brake_safe_count_ = 0;
    safety_mode_ = pp_safety_enable_ ? SAFETY_SAFE : SAFETY_DISABLED;

    initialized_ = true;
    startCorridorWorker();

    ROS_WARN("MyPlanner PP-C0.5 STABLE-PP-FIXED-HALF-X-BRAKE 启动："
             "控制主体为最早稳定治愈路径PP；安全框不参与目标选择。"
             "连续2周期明确出框后linear.x固定乘0.5，linear.y与angular.z保持PP原值；不停车、不由安全盾重规划。"
             "lookahead=%.3f，K=(%.2f,%.2f,%.2f)，base=%s，costmap=%s。",
             lookahead_dist_, path_linear_x_gain_, path_linear_y_gain_,
             path_angular_y_gain_, base_frame_.c_str(), costmap_frame_.c_str());
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
    last_corridor_request_time_ = ros::Time(0);
    force_corridor_update_ = true;
    pp_x_brake_unsafe_count_ = 0;
    pp_x_brake_safe_count_ = 0;
    safety_mode_ = pp_safety_enable_ ? SAFETY_SAFE : SAFETY_DISABLED;
}

void MyPlanner::stopImmediately(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    last_cmd_vel_ = geometry_msgs::Twist();
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
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    force_corridor_update_ = true;
    pp_x_brake_unsafe_count_ = 0;
    pp_x_brake_safe_count_ = 0;
    safety_mode_ = pp_safety_enable_ ? SAFETY_SAFE : SAFETY_DISABLED;

    if (new_goal)
    {
        resetForNewGoal();
        ROS_INFO("收到新目标，状态已重置；路径点数：%zu。", plan.size());
    }
    else
    {
        ROS_INFO("同一目标的全局路径已更新；路径点数：%zu。", plan.size());
    }

    invalidateCorridorReferenceSnapshot();
    rebuildCorridorPlanCache(plan);
    // 下一控制周期先完成路径治愈，再由force_corridor_update_生成安全框。
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

void MyPlanner::limitCommandWithoutCommit(
    const geometry_msgs::Twist& desired_cmd,
    geometry_msgs::Twist& limited_cmd,
    double dt,
    double accel_x,
    double accel_y,
    double accel_theta) const
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

    const double max_delta_x = std::max(0.0, accel_x) * dt;
    const double max_delta_y = std::max(0.0, accel_y) * dt;
    const double max_delta_theta =
        std::max(0.0, accel_theta) * dt;

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
}

void MyPlanner::applyVelocityAndAccelerationLimits(
    const geometry_msgs::Twist& desired_cmd,
    geometry_msgs::Twist& limited_cmd,
    double dt)
{
    limitCommandWithoutCommit(
        desired_cmd,
        limited_cmd,
        dt,
        acc_lim_x_,
        acc_lim_y_,
        acc_lim_theta_);
    last_cmd_vel_ = limited_cmd;
}

void MyPlanner::computePurePursuitCommand(
    const geometry_msgs::PoseStamped& target_pose,
    double lateral_deviation,
    geometry_msgs::Twist& desired_cmd) const
{
    desired_cmd = geometry_msgs::Twist();
    desired_cmd.linear.x =
        target_pose.pose.position.x * path_linear_x_gain_;
    desired_cmd.linear.y =
        lateral_deviation * path_linear_y_gain_;
    desired_cmd.angular.z =
        target_pose.pose.position.y * path_angular_y_gain_;
}

bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();

    if (!initialized_ || raw_plan_.empty() || costmap_ros_ == NULL)
        return false;

    const ros::Time now = ros::Time::now();
    double dt = 0.05;
    if (!last_control_time_.isZero())
        dt = (now - last_control_time_).toSec();
    last_control_time_ = now;
    dt = clampValue(dt, 0.01, 0.20);

    // 与最早稳定版一致：每周期重新生成非累积治愈路径。
    updateHealedPath();

    // 安全框只读取治愈路径，不改变PP目标。
    updateCorridorVisualizationIfNeeded();

    if (!checkPathCollision())
    {
        stopImmediately(cmd_vel);
        return false;
    }

    geometry_msgs::PoseStamped final_pose;
    if (!transformPose(base_frame_, global_plan_.back(), final_pose))
    {
        stopImmediately(cmd_vel);
        return false;
    }

    const double final_distance =
        std::hypot(final_pose.pose.position.x,
                   final_pose.pose.position.y);

    if (!pose_adjusting_ && final_distance < goal_dist_threshold_)
    {
        pose_adjusting_ = true;
        requestCorridorClear();
        invalidateCorridorReferenceSnapshot();
        ROS_INFO("距离目标 %.3fm，进入终点位姿调整。", final_distance);
    }

    geometry_msgs::Twist desired_cmd;

    if (pose_adjusting_)
    {
        if (computeFinalPoseCommand(final_pose, desired_cmd))
        {
            stopImmediately(cmd_vel);
            return true;
        }
        applyVelocityAndAccelerationLimits(desired_cmd, cmd_vel, dt);
        return true;
    }

    geometry_msgs::PoseStamped target_pose;
    if (!selectTrackingTarget(target_pose))
    {
        stopImmediately(cmd_vel);
        return false;
    }

    if (!initial_rotation_done_)
    {
        if (computeInitialRotationCommand(target_pose, desired_cmd))
        {
            applyVelocityAndAccelerationLimits(desired_cmd, cmd_vel, dt);
            return true;
        }
    }

    const double lateral_deviation =
        computeLateralDeviation(target_pose);
    computePurePursuitCommand(
        target_pose,
        lateral_deviation,
        desired_cmd);

    // 先得到稳定PP正常会发布的限速命令，再让安全盾只削减x。
    geometry_msgs::Twist nominal_cmd;
    limitCommandWithoutCommit(
        desired_cmd,
        nominal_cmd,
        dt,
        acc_lim_x_,
        acc_lim_y_,
        acc_lim_theta_);

    SafetyCheckReport nominal_report;
    SafetyCheckReport selected_report;
    double selected_x_scale = 1.0;
    bool unsafe_now = false;

    if (pp_safety_enable_)
    {
        applyFixedXBrakeSafetyShield(
            nominal_cmd,
            cmd_vel,
            nominal_report,
            selected_report,
            selected_x_scale,
            unsafe_now);
    }
    else
    {
        cmd_vel = nominal_cmd;
        safety_mode_ = SAFETY_DISABLED;
        pp_x_brake_unsafe_count_ = 0;
        pp_x_brake_safe_count_ = 0;
    }

    // 简化安全盾绝不停车、绝不返回false、绝不主动请求重规划。
    // 输出只可能是原PP命令，或linear.x固定乘pp_x_brake_scale_。
    last_cmd_vel_ = cmd_vel;

    publishSafetyDebug(nominal_report, selected_report);

    if (debug_log_)
    {
        ROS_INFO_THROTTLE(
            0.5,
            "PP-C0.5稳定PP：target=(%.3f,%.3f) dist=%.3f alpha=%.3f "
            "lateral=%.3f raw=(%.3f,%.3f,%.3f) "
            "nominal=(%.3f,%.3f,%.3f) cmd=(%.3f,%.3f,%.3f) "
            "shield=%s x_scale=%.2f trigger=%d count(unsafe=%d safe=%d) "
            "margin(raw=%.3f actual=%.3f) outside(raw=%d actual=%d) index=%d。",
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            std::hypot(target_pose.pose.position.x,
                       target_pose.pose.position.y),
            std::atan2(target_pose.pose.position.y,
                       target_pose.pose.position.x),
            lateral_deviation,
            desired_cmd.linear.x,
            desired_cmd.linear.y,
            desired_cmd.angular.z,
            nominal_cmd.linear.x,
            nominal_cmd.linear.y,
            nominal_cmd.angular.z,
            cmd_vel.linear.x,
            cmd_vel.linear.y,
            cmd_vel.angular.z,
            safetyModeName(),
            selected_x_scale,
            unsafe_now ? 1 : 0,
            pp_x_brake_unsafe_count_,
            pp_x_brake_safe_count_,
            nominal_report.min_margin,
            selected_report.min_margin,
            nominal_report.outside_count,
            selected_report.outside_count,
            target_index_);
    }

    return true;
}

bool MyPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace my_planner