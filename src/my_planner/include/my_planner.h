#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>
#include <vector>

namespace my_planner
{

class MyPlanner : public nav_core::BaseLocalPlanner
{
public:
    MyPlanner();
    ~MyPlanner();

    void initialize(std::string name,
                    tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();

private:
    static double clampValue(double value, double lower, double upper);
    static double normalizeAngle(double angle);
    static double applyMinimumMagnitude(double value, double minimum);

    bool transformPose(const std::string& target_frame,
                       const geometry_msgs::PoseStamped& input,
                       geometry_msgs::PoseStamped& output);

    bool isNewGoal(const geometry_msgs::PoseStamped& goal) const;
    void resetForNewGoal();
    void stopImmediately(geometry_msgs::Twist& cmd_vel);

    // 仿真版核心路径逻辑
    void updateHealedPath();
    bool checkPathCollision();
    bool selectTrackingTarget(geometry_msgs::PoseStamped& target_pose);
    double computeLateralDeviation(
        const geometry_msgs::PoseStamped& target_pose);

    // 三种控制状态
    bool computeInitialRotationCommand(
        const geometry_msgs::PoseStamped& target_pose,
        geometry_msgs::Twist& desired_cmd);

    bool computeFinalPoseCommand(
        const geometry_msgs::PoseStamped& final_pose,
        geometry_msgs::Twist& desired_cmd);

    void applyVelocityAndAccelerationLimits(
        const geometry_msgs::Twist& desired_cmd,
        geometry_msgs::Twist& limited_cmd,
        double dt);

private:
    bool initialized_;
    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    std::string base_frame_;
    std::string costmap_frame_;

    // raw_plan_ 永远保存全局规划器下发的原始路径。
    // global_plan_ 是当前周期从 raw_plan_ 重新生成的治愈路径。
    std::vector<geometry_msgs::PoseStamped> raw_plan_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped goal_pose_;
    bool has_goal_;

    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool initial_rotation_done_;

    // 仿真版路径跟踪控制律
    double lookahead_dist_;
    double path_linear_x_gain_;
    double path_linear_y_gain_;
    double path_angular_y_gain_;
    int lateral_search_points_;

    // 非累积路径治愈
    bool enable_path_healing_;
    int path_healing_points_behind_;
    int path_healing_points_ahead_;
    int path_healing_iterations_;
    double path_healing_max_step_;
    double path_healing_gradient_deadband_;
    double path_healing_gradient_scale_;

    // 一代车/仿真版单栅格路径检查
    int collision_check_lookahead_points_;
    unsigned char collision_cost_threshold_;

    // 初始姿态调整
    bool enable_initial_rotation_;
    double initial_yaw_tolerance_;
    double initial_angular_gain_;
    double initial_min_angular_speed_;
    double initial_max_angular_speed_;

    // 终点位姿调整
    double goal_dist_threshold_;
    double goal_position_tolerance_;
    double goal_yaw_tolerance_;
    double final_linear_x_gain_;
    double final_linear_y_gain_;
    double final_angular_gain_;
    double final_min_linear_speed_;
    double final_min_angular_speed_;
    double final_max_vel_x_;
    double final_max_vel_y_;
    double final_max_vel_theta_;

    // 实车最外层速度和加速度保护
    double max_vel_x_;
    double max_vel_y_;
    double max_vel_theta_;
    double acc_lim_x_;
    double acc_lim_y_;
    double acc_lim_theta_;

    bool debug_log_;
    geometry_msgs::Twist last_cmd_vel_;
    ros::Time last_control_time_;
};

}  // namespace my_planner

#endif  // MY_PLANNER_H_