#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <deque>
#include <mutex>
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
    struct PathPoint2D
    {
        double x;
        double y;
        double s;
        double yaw;
        double curvature_track;
        double curvature_speed;
        double speed_limit;
        int source_index;

        PathPoint2D()
            : x(0.0), y(0.0), s(0.0), yaw(0.0),
              curvature_track(0.0), curvature_speed(0.0),
              speed_limit(0.0), source_index(-1)
        {
        }
    };

    struct MpcReferencePoint
    {
        double x;
        double y;
        double yaw;          // 车头姿态参考 psi
        double motion_yaw;   // 路径运动方向 chi
        double vx;
        double vy;
        double omega;          // 期望实际角速度状态
        double omega_cmd;      // 一阶响应模型对应的角速度命令参考
        double curvature;
        double speed_limit;
        double path_speed;
        double drift_beta;      // 实际有效beta = chi - psi
        double planned_beta;    // 经过速度相关限幅与变化率限制的计划beta
        double beta_limit;      // 当前速度对应的beta上限
        double curve_strength;

        MpcReferencePoint()
            : x(0.0), y(0.0), yaw(0.0), motion_yaw(0.0),
              vx(0.0), vy(0.0), omega(0.0), omega_cmd(0.0),
              curvature(0.0), speed_limit(0.0),
              path_speed(0.0), drift_beta(0.0), planned_beta(0.0),
              beta_limit(0.0), curve_strength(0.0)
        {
        }
    };

    struct MpcSolveReport
    {
        bool success;
        bool solved_inaccurate;
        double projection_s;
        double remaining_length;
        double preview_distance;
        double setup_ms;
        double solve_ms;
        double total_ms;
        double objective;
        double preview_abs_curvature;
        double first_tracking_curvature;
        double first_speed_limit;
        double first_path_speed;
        double first_drift_beta;
        double first_curve_strength;
        double max_abs_drift_beta;
        double max_abs_planned_beta;
        double initial_omega_seed;
        double measured_vx;
        double measured_vy;
        double measured_omega;
        double odom_age;
        double delay_x;
        double delay_y;
        double delay_yaw;
        double delay_omega;
        bool using_odometry;
        int beta_speed_guard_steps;
        int resampled_points;
        int iterations;
        std::string status;
        std::string failure_reason;
        MpcReferencePoint first_reference;
        std::vector<PathPoint2D> reference_path;
        std::vector<PathPoint2D> predicted_path;

        MpcSolveReport()
            : success(false), solved_inaccurate(false),
              projection_s(0.0), remaining_length(0.0),
              preview_distance(0.0), setup_ms(0.0), solve_ms(0.0),
              total_ms(0.0), objective(0.0),
              preview_abs_curvature(0.0),
              first_tracking_curvature(0.0),
              first_speed_limit(0.0),
              first_path_speed(0.0), first_drift_beta(0.0),
              first_curve_strength(0.0), max_abs_drift_beta(0.0),
              max_abs_planned_beta(0.0), initial_omega_seed(0.0),
              measured_vx(0.0), measured_vy(0.0), measured_omega(0.0),
              odom_age(0.0), delay_x(0.0), delay_y(0.0), delay_yaw(0.0),
              delay_omega(0.0), using_odometry(false),
              beta_speed_guard_steps(0), resampled_points(0), iterations(0)
        {
        }
    };

    static double clampValue(double value, double lower, double upper);
    static double normalizeAngle(double angle);
    static double applyMinimumMagnitude(double value, double minimum);

    struct MeasuredBodyState
    {
        double vx;
        double vy;
        double omega;
        double age;
        bool valid;

        MeasuredBodyState()
            : vx(0.0), vy(0.0), omega(0.0), age(0.0), valid(false)
        {
        }
    };

    struct DelayCompensatedState
    {
        double x;
        double y;
        double yaw;
        double vx;
        double vy;
        double omega;

        DelayCompensatedState()
            : x(0.0), y(0.0), yaw(0.0),
              vx(0.0), vy(0.0), omega(0.0)
        {
        }
    };

    struct TimedCommand
    {
        ros::Time stamp;
        geometry_msgs::Twist command;
    };

    struct ButterworthFilterState
    {
        bool initialized;
        double x1;
        double x2;
        double y1;
        double y2;
        ros::Time last_stamp;

        ButterworthFilterState()
            : initialized(false), x1(0.0), x2(0.0),
              y1(0.0), y2(0.0), last_stamp(0)
        {
        }
    };

    bool transformPose(const std::string& target_frame,
                       const geometry_msgs::PoseStamped& input,
                       geometry_msgs::PoseStamped& output);


    void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
    double filterMeasuredOmega(double raw_omega, const ros::Time& stamp);
    MeasuredBodyState getMeasuredBodyState(const ros::Time& now) const;
    DelayCompensatedState predictStateThroughInputDelay(
        const MeasuredBodyState& measured,
        const ros::Time& now) const;
    geometry_msgs::Twist commandAtTime(
        const std::deque<TimedCommand>& history,
        const ros::Time& query_time) const;
    void recordPublishedCommand(
        const geometry_msgs::Twist& command,
        const ros::Time& stamp);

    bool isNewGoal(const geometry_msgs::PoseStamped& goal) const;
    void resetForNewGoal();
    void stopImmediately(geometry_msgs::Twist& cmd_vel);

    // 原稳定PP外围机制与失败回退。
    void updateHealedPath();
    bool checkPathCollision();
    bool selectTrackingTarget(geometry_msgs::PoseStamped& target_pose);
    double computeLateralDeviation(
        const geometry_msgs::PoseStamped& target_pose);
    bool computeInitialRotationCommand(
        const geometry_msgs::PoseStamped& target_pose,
        geometry_msgs::Twist& desired_cmd);
    bool computeFinalPoseCommand(
        const geometry_msgs::PoseStamped& final_pose,
        geometry_msgs::Twist& desired_cmd);
    void computePurePursuitCommand(
        const geometry_msgs::PoseStamped& target_pose,
        geometry_msgs::Twist& desired_cmd,
        double& lateral_deviation);
    void applyVelocityAndAccelerationLimits(
        const geometry_msgs::Twist& desired_cmd,
        geometry_msgs::Twist& limited_cmd,
        double dt);

    // C3：无安全框全向LTV-MPC＋双曲率速度规划＋主动漂移参考。
    void resetMpcState();
    bool buildMpcReferenceTrajectory(
        double control_dt,
        std::vector<MpcReferencePoint>& reference,
        MpcSolveReport& report);
    bool computeMpcCommand(
        double control_dt,
        geometry_msgs::Twist& desired_cmd,
        MpcSolveReport& report);
    bool solveLtvMpcQp(
        const std::vector<MpcReferencePoint>& reference,
        double control_dt,
        geometry_msgs::Twist& desired_cmd,
        MpcSolveReport& report);
    void publishMpcDebugPaths(const MpcSolveReport& report);

private:
    bool initialized_;
    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    std::string base_frame_;
    std::string costmap_frame_;

    std::vector<geometry_msgs::PoseStamped> raw_plan_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped goal_pose_;
    bool has_goal_;

    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool initial_rotation_done_;

    // 稳定PP参数。
    double lookahead_dist_;
    double path_linear_x_gain_;
    double path_linear_y_gain_;
    double path_angular_y_gain_;
    int lateral_search_points_;

    // C3控制器与参考轨迹。
    std::string controller_mode_;
    int mpc_horizon_steps_;
    double mpc_dt_;
    double mpc_min_reference_length_;

    // C2保留：等距线性重采样、双曲率通道与速度传播。
    double c2_resample_distance_;
    double c2_duplicate_point_distance_;
    double c2_tracking_curvature_distance_;
    double c2_speed_curvature_distance_;
    int c2_curvature_median_window_;
    double c2_curvature_preview_distance_;
    double c2_hold_speed_after_curve_;
    double c2_max_reference_speed_;
    double c2_min_curve_speed_;
    double c2_curve_lateral_acc_limit_;
    double c2_reference_acceleration_;
    double c2_reference_deceleration_;
    int mpc_reference_search_behind_points_;
    int mpc_reference_search_ahead_points_;

    // C3主动漂移：路径运动方向chi与车头姿态psi解耦。
    bool c3_enable_active_drift_;
    double c3_yaw_preview_distance_;
    double c3_yaw_preview_gain_;
    double c3_yaw_preview_curvature_deadband_;
    double c3_yaw_preview_full_curvature_;
    double c3_beta_max_low_speed_;
    double c3_beta_max_mid_speed_;
    double c3_beta_max_high_speed_;
    double c3_beta_low_speed_threshold_;
    double c3_beta_high_speed_threshold_;
    double c3_beta_rate_limit_;
    double c3_reference_omega_limit_;
    double c3_reference_omega_accel_rate_;
    double c3_reference_omega_decel_rate_;
    double c3_reference_omega_reverse_rate_;
    double c3_omega_curvature_feedforward_gain_;

    // TEB/MPPI/Autoware式真实初始状态、输入延迟与一阶执行器响应。
    bool c3_use_odometry_;
    std::string c3_odom_topic_;
    double c3_odom_timeout_;
    double c3_measured_omega_filter_cutoff_hz_;
    double c3_input_delay_;
    double c3_translational_response_tau_;
    double c3_angular_response_tau_;
    double c3_delay_integration_dt_;

    // 状态误差权重：位置按路径切线坐标系，航向权重随曲率调度。
    double mpc_weight_longitudinal_;
    double mpc_weight_lateral_;
    double mpc_weight_yaw_straight_;
    double mpc_weight_yaw_curve_;
    double mpc_weight_omega_state_straight_;
    double mpc_weight_omega_state_curve_;
    double mpc_terminal_position_weight_scale_;
    double mpc_terminal_yaw_weight_scale_;
    double mpc_terminal_omega_weight_scale_;

    // 控制速度按路径切向/法向分解；漂移时允许车体vy，但抑制横穿路径。
    double mpc_weight_tangent_velocity_;
    double mpc_weight_path_normal_velocity_;
    double mpc_weight_progress_;

    // 相对参考控制量及控制变化权重。
    double mpc_weight_vx_;
    double mpc_weight_vy_;
    double mpc_weight_omega_straight_;
    double mpc_weight_omega_curve_;
    double mpc_weight_delta_vx_;
    double mpc_weight_delta_vy_;
    double mpc_weight_delta_omega_;

    // 速度约束。vx不得超过逐步参考分量，并至少保留指定倍率。
    double mpc_min_vx_ratio_;
    double mpc_min_vx_;
    double mpc_max_vx_;
    double mpc_min_vy_;
    double mpc_max_vy_;
    double mpc_min_omega_;
    double mpc_max_omega_;
    double mpc_max_translational_speed_;
    int mpc_velocity_polygon_sides_;

    // 控制变化约束。
    double mpc_max_accel_x_;
    double mpc_max_decel_x_;
    double mpc_max_accel_y_;
    double mpc_max_accel_theta_;

    // OSQP及回退策略。
    int mpc_osqp_max_iterations_;
    double mpc_osqp_eps_abs_;
    double mpc_osqp_eps_rel_;
    bool mpc_osqp_polish_;
    bool mpc_osqp_verbose_;
    double mpc_max_total_time_ms_;
    int mpc_max_consecutive_failures_;
    bool mpc_lock_to_pp_after_failures_;
    int mpc_consecutive_failures_;
    bool mpc_locked_to_pp_;
    bool mpc_publish_debug_paths_;
    ros::Publisher mpc_reference_path_pub_;
    ros::Publisher mpc_predicted_path_pub_;
    ros::Subscriber odom_sub_;

    mutable std::mutex measured_state_mutex_;
    geometry_msgs::Twist measured_body_twist_;
    ros::Time last_odom_stamp_;
    bool odom_received_;
    ButterworthFilterState omega_filter_state_;

    mutable std::mutex command_history_mutex_;
    std::deque<TimedCommand> command_history_;

    // 非累积路径治愈。C3默认关闭，打开后MPC与PP共同跟踪治愈路径。
    bool enable_path_healing_;
    int path_healing_points_behind_;
    int path_healing_points_ahead_;
    int path_healing_iterations_;
    double path_healing_max_step_;
    double path_healing_gradient_deadband_;
    double path_healing_gradient_scale_;

    int collision_check_lookahead_points_;
    unsigned char collision_cost_threshold_;

    bool enable_initial_rotation_;
    double initial_yaw_tolerance_;
    double initial_angular_gain_;
    double initial_min_angular_speed_;
    double initial_max_angular_speed_;

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