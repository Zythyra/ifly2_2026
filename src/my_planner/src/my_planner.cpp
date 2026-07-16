// 交付构建标识：MYPLANNER_V2_SIMBASE_20260715
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
      initial_rotation_done_(false)
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

    // 仿真版核心跟踪参数
    private_nh.param("lookahead_dist", lookahead_dist_, 0.20);
    private_nh.param("path_linear_x_gain", path_linear_x_gain_, 1.00);
    private_nh.param("path_linear_y_gain", path_linear_y_gain_, 1.50);
    private_nh.param("path_angular_y_gain", path_angular_y_gain_, 8.00);
    private_nh.param("lateral_search_points", lateral_search_points_, 10);

    // 仿真版非累积路径治愈
    private_nh.param("enable_path_healing", enable_path_healing_, true);
    private_nh.param("path_healing_points_behind",
                     path_healing_points_behind_, 5);
    private_nh.param("path_healing_points_ahead",
                     path_healing_points_ahead_, 60);
    private_nh.param("path_healing_iterations", path_healing_iterations_, 3);
    private_nh.param("path_healing_max_step", path_healing_max_step_, 0.01);
    private_nh.param("path_healing_gradient_deadband",
                     path_healing_gradient_deadband_, 4.0);
    private_nh.param("path_healing_gradient_scale",
                     path_healing_gradient_scale_, 20.0);

    // 一代车/仿真版单栅格路径障碍检查
    private_nh.param("collision_check_lookahead_points",
                     collision_check_lookahead_points_, 10);
    int collision_cost_threshold = 253;
    private_nh.param("collision_cost_threshold",
                     collision_cost_threshold, 253);
    collision_cost_threshold = std::max(
        1, std::min(collision_cost_threshold, 255));
    collision_cost_threshold_ =
        static_cast<unsigned char>(collision_cost_threshold);

    // 初始姿态调整
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

    // 终点位姿调整
    private_nh.param("goal_dist_threshold", goal_dist_threshold_, 0.15);
    private_nh.param("goal_position_tolerance",
                     goal_position_tolerance_, 0.025);
    private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.05);
    private_nh.param("final_linear_x_gain", final_linear_x_gain_, 0.80);
    private_nh.param("final_linear_y_gain", final_linear_y_gain_, 0.80);
    private_nh.param("final_angular_gain", final_angular_gain_, 1.50);
    private_nh.param("final_min_linear_speed",
                     final_min_linear_speed_, 0.03);
    private_nh.param("final_min_angular_speed",
                     final_min_angular_speed_, 0.08);
    private_nh.param("final_max_vel_x", final_max_vel_x_, 0.06);
    private_nh.param("final_max_vel_y", final_max_vel_y_, 0.04);
    private_nh.param("final_max_vel_theta", final_max_vel_theta_, 0.25);

    // 实车速度和加速度保护
    private_nh.param("max_vel_x", max_vel_x_, 0.08);
    private_nh.param("max_vel_y", max_vel_y_, 0.04);
    private_nh.param("max_vel_theta", max_vel_theta_, 0.30);
    private_nh.param("acc_lim_x", acc_lim_x_, 0.16);
    private_nh.param("acc_lim_y", acc_lim_y_, 0.12);
    private_nh.param("acc_lim_theta", acc_lim_theta_, 1.20);

    private_nh.param("debug_log", debug_log_, true);

    lateral_search_points_ = std::max(0, lateral_search_points_);
    path_healing_points_behind_ = std::max(0, path_healing_points_behind_);
    path_healing_points_ahead_ = std::max(1, path_healing_points_ahead_);
    path_healing_iterations_ = std::max(0, path_healing_iterations_);
    collision_check_lookahead_points_ =
        std::max(1, collision_check_lookahead_points_);
    path_healing_gradient_scale_ =
        std::max(1e-6, path_healing_gradient_scale_);

    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    initial_rotation_done_ = !enable_initial_rotation_;
    last_cmd_vel_ = geometry_msgs::Twist();
    last_control_time_ = ros::Time(0);

    initialized_ = true;
    ROS_WARN("MyPlanner V2.0-SIMBASE-20260715 启动："
             "采用仿真一代车移植控制律。"
             "base=%s，costmap=%s。",
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

    const double dx = goal.pose.position.x - goal_pose_.pose.position.x;
    const double dy = goal.pose.position.y - goal_pose_.pose.position.y;
    const double position_change = std::hypot(dx, dy);

    const double old_yaw = tf::getYaw(goal_pose_.pose.orientation);
    const double new_yaw = tf::getYaw(goal.pose.orientation);
    const double yaw_change = std::abs(normalizeAngle(new_yaw - old_yaw));

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

    // 与仿真版一致：每次全局路径更新都从路径索引0重新向前搜索。
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;

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

    // 仿真版本的关键：每个控制周期都从原始路径重新开始，绝不累计漂移。
    global_plan_ = raw_plan_;

    if (!enable_path_healing_ || path_healing_iterations_ <= 0)
        return;

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (costmap == NULL)
        return;

    target_index_ = std::max(
        0, std::min(target_index_, static_cast<int>(global_plan_.size()) - 1));

    const int heal_start = std::max(
        0, target_index_ - path_healing_points_behind_);
    const int heal_end = std::min(
        static_cast<int>(global_plan_.size()),
        target_index_ + path_healing_points_ahead_);

    const double resolution = costmap->getResolution();
    const double max_step = std::min(
        std::max(0.0, path_healing_max_step_), resolution);

    for (int i = heal_start; i < heal_end; ++i)
    {
        const geometry_msgs::PoseStamped original_point = raw_plan_[i];
        geometry_msgs::PoseStamped point_costmap;
        if (!transformPose(costmap_frame_, original_point, point_costmap))
            continue;

        double wx = point_costmap.pose.position.x;
        double wy = point_costmap.pose.position.y;

        for (int iteration = 0;
             iteration < path_healing_iterations_; ++iteration)
        {
            double gradient_x = 0.0;
            double gradient_y = 0.0;
            unsigned char center_cost = 0;

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
                    lock(*(costmap->getMutex()));

                unsigned int mx = 0;
                unsigned int my = 0;
                if (!costmap->worldToMap(wx, wy, mx, my)
                    || mx == 0 || my == 0
                    || mx + 1 >= costmap->getSizeInCellsX()
                    || my + 1 >= costmap->getSizeInCellsY())
                {
                    break;
                }

                center_cost = costmap->getCost(mx, my);
                if (center_cost == 0
                    || center_cost >= collision_cost_threshold_)
                {
                    break;
                }

                const int cost_up = costmap->getCost(mx, my + 1);
                const int cost_down = costmap->getCost(mx, my - 1);
                const int cost_left = costmap->getCost(mx - 1, my);
                const int cost_right = costmap->getCost(mx + 1, my);

                gradient_x = static_cast<double>(cost_left - cost_right);
                gradient_y = static_cast<double>(cost_down - cost_up);
            }

            const double gradient_norm = std::hypot(gradient_x, gradient_y);
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
        else if (transformPose(original_frame, point_costmap, healed_point))
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
        0, std::min(target_index_, static_cast<int>(global_plan_.size()) - 1));

    // 与仿真版一致：从当前 target_index_ 开始检查前方固定数量路径点，
    // 每个路径点只读取所在的单个代价地图栅格。
    const int check_end = std::min(
        target_index_ + collision_check_lookahead_points_,
        static_cast<int>(global_plan_.size()));

    for (int i = target_index_; i < check_end; ++i)
    {
        geometry_msgs::PoseStamped point_costmap;
        if (!transformPose(costmap_frame_, global_plan_[i], point_costmap))
            continue;

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap->worldToMap(point_costmap.pose.position.x,
                                 point_costmap.pose.position.y,
                                 mx, my))
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
                     i, static_cast<unsigned int>(cost));
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
        0, std::min(target_index_, static_cast<int>(global_plan_.size()) - 1));

    bool transformed_any_point = false;
    for (int i = target_index_;
         i < static_cast<int>(global_plan_.size()); ++i)
    {
        geometry_msgs::PoseStamped pose_base;
        if (!transformPose(base_frame_, global_plan_[i], pose_base))
            continue;

        transformed_any_point = true;
        target_pose = pose_base;

        const double distance = std::hypot(
            pose_base.pose.position.x, pose_base.pose.position.y);

        // 严格保留仿真版选择方式：第一个距离车体超过 lookahead 的点。
        if (distance > lookahead_dist_)
        {
            target_index_ = i;
            return true;
        }
    }

    if (transformed_any_point)
    {
        target_index_ = static_cast<int>(global_plan_.size()) - 1;
        return true;
    }

    return false;
}

double MyPlanner::computeLateralDeviation(
    const geometry_msgs::PoseStamped& target_pose)
{
    double min_y_deviation = target_pose.pose.position.y;
    const int search_start = std::max(
        0, target_index_ - lateral_search_points_);

    // 严格保留仿真版逻辑：在前视点前方索引区间中，选择绝对值最小的y。
    for (int i = search_start; i < target_index_; ++i)
    {
        geometry_msgs::PoseStamped point_base;
        if (!transformPose(base_frame_, global_plan_[i], point_base))
            continue;

        if (std::abs(point_base.pose.position.y)
            < std::abs(min_y_deviation))
        {
            min_y_deviation = point_base.pose.position.y;
        }
    }

    return min_y_deviation;
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

    const double angle_to_target = std::atan2(
        target_pose.pose.position.y,
        target_pose.pose.position.x);

    if (std::abs(angle_to_target) < initial_yaw_tolerance_)
    {
        initial_rotation_done_ = true;
        ROS_INFO("初始姿态已对准，开始正常路径跟踪。");
        return false;
    }

    double angular_speed = std::abs(angle_to_target) * initial_angular_gain_;
    angular_speed = clampValue(angular_speed,
                               initial_min_angular_speed_,
                               initial_max_angular_speed_);
    desired_cmd.angular.z = std::copysign(angular_speed, angle_to_target);
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
    const double yaw_error = normalizeAngle(
        tf::getYaw(final_pose.pose.orientation));

    if (distance_error <= goal_position_tolerance_
        && std::abs(yaw_error) <= goal_yaw_tolerance_)
    {
        goal_reached_ = true;
        ROS_WARN("到达终点：位置误差=%.3fm，角度误差=%.3frad。",
                 distance_error, yaw_error);
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

    desired_cmd.linear.x = clampValue(
        vx, -final_max_vel_x_, final_max_vel_x_);
    desired_cmd.linear.y = clampValue(
        vy, -final_max_vel_y_, final_max_vel_y_);
    desired_cmd.angular.z = clampValue(
        wz, -final_max_vel_theta_, final_max_vel_theta_);
    return false;
}

void MyPlanner::applyVelocityAndAccelerationLimits(
    const geometry_msgs::Twist& desired_cmd,
    geometry_msgs::Twist& limited_cmd,
    double dt)
{
    geometry_msgs::Twist target = desired_cmd;
    target.linear.x = clampValue(target.linear.x, -max_vel_x_, max_vel_x_);
    target.linear.y = clampValue(target.linear.y, -max_vel_y_, max_vel_y_);
    target.angular.z = clampValue(target.angular.z,
                                  -max_vel_theta_, max_vel_theta_);

    const double max_delta_x = std::max(0.0, acc_lim_x_) * dt;
    const double max_delta_y = std::max(0.0, acc_lim_y_) * dt;
    const double max_delta_theta = std::max(0.0, acc_lim_theta_) * dt;

    limited_cmd = geometry_msgs::Twist();
    limited_cmd.linear.x = clampValue(
        target.linear.x,
        last_cmd_vel_.linear.x - max_delta_x,
        last_cmd_vel_.linear.x + max_delta_x);
    limited_cmd.linear.y = clampValue(
        target.linear.y,
        last_cmd_vel_.linear.y - max_delta_y,
        last_cmd_vel_.linear.y + max_delta_y);
    limited_cmd.angular.z = clampValue(
        target.angular.z,
        last_cmd_vel_.angular.z - max_delta_theta,
        last_cmd_vel_.angular.z + max_delta_theta);

    last_cmd_vel_ = limited_cmd;
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

    // 1. 仿真版非累积路径治愈。
    updateHealedPath();

    // 2. 仿真版10点单栅格障碍检查。
    if (!checkPathCollision())
    {
        stopImmediately(cmd_vel);
        return false;
    }

    // 3. 终点附近直接调整最终位姿。
    geometry_msgs::PoseStamped final_pose;
    if (!transformPose(base_frame_, global_plan_.back(), final_pose))
    {
        stopImmediately(cmd_vel);
        return false;
    }

    const double final_distance = std::hypot(
        final_pose.pose.position.x,
        final_pose.pose.position.y);
    if (!pose_adjusting_ && final_distance < goal_dist_threshold_)
    {
        pose_adjusting_ = true;
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

    // 4. 仿真版前视点：从当前 target_index_ 向前找到第一个
    //    与车体距离超过 lookahead_dist 的路径点。
    geometry_msgs::PoseStamped target_pose;
    if (!selectTrackingTarget(target_pose))
    {
        stopImmediately(cmd_vel);
        return false;
    }

    // 5. 新目标开始时先对准当前前视点。
    if (!initial_rotation_done_)
    {
        if (computeInitialRotationCommand(target_pose, desired_cmd))
        {
            applyVelocityAndAccelerationLimits(desired_cmd, cmd_vel, dt);
            return true;
        }
    }

    // 6. 严格保留仿真版正常跟踪控制律：
    //    vx 使用 target.x；vy 使用回看区间内最小横向偏差；
    //    wz 直接使用 target.y，不改成 atan2 航向角。
    const double lateral_deviation = computeLateralDeviation(target_pose);

    desired_cmd = geometry_msgs::Twist();
    desired_cmd.linear.x =
        target_pose.pose.position.x * path_linear_x_gain_;
    desired_cmd.linear.y =
        lateral_deviation * path_linear_y_gain_;
    desired_cmd.angular.z =
        target_pose.pose.position.y * path_angular_y_gain_;

    applyVelocityAndAccelerationLimits(desired_cmd, cmd_vel, dt);

    if (debug_log_)
    {
        ROS_INFO_THROTTLE(
            0.5,
            "V2跟踪：target=(%.3f, %.3f)，dist=%.3f，"
            "lateral=%.3f，raw=(%.3f, %.3f, %.3f)，"
            "cmd=(%.3f, %.3f, %.3f)，index=%d。",
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            std::hypot(target_pose.pose.position.x,
                       target_pose.pose.position.y),
            lateral_deviation,
            desired_cmd.linear.x,
            desired_cmd.linear.y,
            desired_cmd.angular.z,
            cmd_vel.linear.x,
            cmd_vel.linear.y,
            cmd_vel.angular.z,
            target_index_);
    }

    return true;
}

bool MyPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace my_planner