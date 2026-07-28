#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
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
    struct Point2D
    {
        double x;
        double y;
        Point2D() : x(0.0), y(0.0) {}
        Point2D(double input_x, double input_y) : x(input_x), y(input_y) {}
    };

    struct PathPoint2D
    {
        double x;
        double y;
        double s;
        double yaw;
        double curvature;
        double clearance;
        int corridor_index;

        PathPoint2D()
            : x(0.0), y(0.0), s(0.0), yaw(0.0), curvature(0.0),
              clearance(0.0), corridor_index(-1)
        {
        }
    };

    struct ConvexPolygon
    {
        std::vector<Point2D> vertices;
    };

    struct CostmapSnapshot
    {
        unsigned int size_x;
        unsigned int size_y;
        double resolution;
        double origin_x;
        double origin_y;
        std::vector<unsigned char> data;

        CostmapSnapshot()
            : size_x(0), size_y(0), resolution(0.0),
              origin_x(0.0), origin_y(0.0)
        {
        }
    };

    struct CorridorSegment
    {
        std::size_t index;
        PathPoint2D start;
        PathPoint2D end;
        double yaw;
        double length;
        ConvexPolygon initial_polygon;
        ConvexPolygon clipped_polygon;
        ConvexPolygon center_polygon;
        ConvexPolygon overlap_polygon;
        std::vector<Point2D> cut_obstacles;
        int candidate_obstacle_count;
        int applied_cut_count;
        double clipped_area;
        double center_area;
        double overlap_area_previous;
        bool clipped_valid;
        bool center_valid;
        bool reference_inside_center;
        bool overlap_with_previous;
        bool chain_valid;
        std::string failure_reason;

        CorridorSegment()
            : index(0), yaw(0.0), length(0.0), candidate_obstacle_count(0),
              applied_cut_count(0), clipped_area(0.0), center_area(0.0),
              overlap_area_previous(0.0), clipped_valid(false),
              center_valid(false), reference_inside_center(false),
              overlap_with_previous(true), chain_valid(false)
        {
        }
    };

    struct CorridorReferenceSnapshot
    {
        bool valid;
        std::uint64_t plan_generation;
        std::uint64_t snapshot_revision;
        ros::Time stamp;
        std::string frame_id;
        std::string status;
        double projection_global_s;
        double remaining_path_length;
        double usable_length;
        std::vector<CorridorSegment> corridors;
        std::vector<PathPoint2D> reference_path;

        CorridorReferenceSnapshot()
            : valid(false), plan_generation(0), snapshot_revision(0),
              projection_global_s(0.0), remaining_path_length(0.0),
              usable_length(0.0)
        {
        }
    };

    struct SafetyCheckReport
    {
        bool valid;
        bool safe;
        int outside_count;
        int first_unsafe_step;
        double min_margin;
        double horizon;
        std::vector<PathPoint2D> predicted_path;
        ConvexPolygon first_unsafe_footprint;
        std::string failure_reason;

        SafetyCheckReport()
            : valid(false), safe(false), outside_count(0),
              first_unsafe_step(-1), min_margin(0.0), horizon(0.0)
        {
        }
    };

    enum SafetyMode
    {
        SAFETY_DISABLED = 0,
        SAFETY_SAFE = 1,
        SAFETY_X_BRAKE = 2
    };

    static double clampValue(double value, double lower, double upper);
    static double normalizeAngle(double angle);
    static double applyMinimumMagnitude(double value, double minimum);

    bool transformPose(const std::string& target_frame,
                       const geometry_msgs::PoseStamped& input,
                       geometry_msgs::PoseStamped& output);
    bool isNewGoal(const geometry_msgs::PoseStamped& goal) const;
    void resetForNewGoal();
    void stopImmediately(geometry_msgs::Twist& cmd_vel);

    // 最早稳定版全向PP逻辑。
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
        double lateral_deviation,
        geometry_msgs::Twist& desired_cmd) const;
    void limitCommandWithoutCommit(
        const geometry_msgs::Twist& desired_cmd,
        geometry_msgs::Twist& limited_cmd,
        double dt,
        double accel_x,
        double accel_y,
        double accel_theta) const;
    void applyVelocityAndAccelerationLimits(
        const geometry_msgs::Twist& desired_cmd,
        geometry_msgs::Twist& limited_cmd,
        double dt);

    // 异步安全框。只读取治愈路径，不生成或跟踪框优化曲线。
    void updateCorridorVisualizationIfNeeded();
    void startCorridorWorker();
    void stopCorridorWorker();
    void requestCorridorUpdate(bool force_update);
    void requestCorridorClear();
    void corridorWorkerLoop();
    void computeAndPublishCorridorSnapshot();
    void clearCorridorVisualization();
    void rebuildCorridorPlanCache(
        const std::vector<geometry_msgs::PoseStamped>& plan);
    void refreshCorridorPlanCache(
        const std::vector<geometry_msgs::PoseStamped>& plan);
    bool copyCorridorPlanCache(
        std::vector<PathPoint2D>& cached_plan,
        std::string& plan_frame,
        std::uint64_t& generation) const;
    bool corridorPlanGenerationIsCurrent(std::uint64_t generation) const;
    bool getRobotPoseInCostmap(
        geometry_msgs::PoseStamped& robot_pose_costmap);
    bool buildCostmapSnapshot(CostmapSnapshot& snapshot);
    bool snapshotWorldToMap(const CostmapSnapshot& snapshot,
                            double wx,
                            double wy,
                            unsigned int& mx,
                            unsigned int& my) const;
    Point2D snapshotMapToWorld(const CostmapSnapshot& snapshot,
                               unsigned int mx,
                               unsigned int my) const;
    unsigned char snapshotCost(const CostmapSnapshot& snapshot,
                               unsigned int mx,
                               unsigned int my) const;
    bool snapshotCellIsHardObstacle(const CostmapSnapshot& snapshot,
                                    unsigned int mx,
                                    unsigned int my) const;
    bool snapshotCellIsObstacleBoundary(const CostmapSnapshot& snapshot,
                                        unsigned int mx,
                                        unsigned int my) const;
    bool buildLocalPathWindow(
        const std::vector<PathPoint2D>& cached_plan,
        const std::string& plan_frame,
        const geometry_msgs::PoseStamped& robot_pose_costmap,
        std::size_t& progress_segment_index,
        bool full_projection_search,
        std::vector<PathPoint2D>& support_path,
        std::vector<PathPoint2D>& forward_path,
        double& projection_global_s,
        double& remaining_path_length);
    bool interpolatePathPoint(const std::vector<PathPoint2D>& path,
                              double query_s,
                              PathPoint2D& output) const;
    void updatePathYaw(std::vector<PathPoint2D>& path) const;
    bool buildSkeletonPath(const std::vector<PathPoint2D>& forward_path,
                           std::vector<PathPoint2D>& skeleton_path) const;
    double estimateCornerAngle(const std::vector<PathPoint2D>& path,
                               std::size_t index) const;
    ConvexPolygon makeInitialCorridor(const CostmapSnapshot& snapshot,
                                      const PathPoint2D& start,
                                      const PathPoint2D& end) const;
    std::vector<Point2D> collectObstacleBoundaryPoints(
        const CostmapSnapshot& snapshot,
        const ConvexPolygon& polygon,
        const PathPoint2D& segment_start,
        const PathPoint2D& segment_end) const;
    bool buildSegmentCorridor(const CostmapSnapshot& snapshot,
                              std::size_t index,
                              const PathPoint2D& start,
                              const PathPoint2D& end,
                              CorridorSegment& corridor) const;
    bool buildSegmentCorridors(const CostmapSnapshot& snapshot,
                               const std::vector<PathPoint2D>& skeleton_path,
                               std::vector<CorridorSegment>& corridors) const;
    ConvexPolygon clipPolygonByHalfPlane(const ConvexPolygon& polygon,
                                         double normal_x,
                                         double normal_y,
                                         double offset) const;
    ConvexPolygon intersectConvexPolygons(const ConvexPolygon& first,
                                          const ConvexPolygon& second) const;
    ConvexPolygon shrinkPolygonForFootprint(const ConvexPolygon& polygon,
                                            double vehicle_yaw) const;
    bool pointInConvexPolygon(const ConvexPolygon& polygon,
                              const Point2D& point,
                              double tolerance = 1e-8) const;
    bool segmentInsidePolygon(const ConvexPolygon& polygon,
                              const PathPoint2D& start,
                              const PathPoint2D& end,
                              double sample_step) const;
    Point2D closestPointOnSegment(const Point2D& point,
                                  const PathPoint2D& start,
                                  const PathPoint2D& end) const;
    double pointToSegmentDistance(const Point2D& point,
                                  const PathPoint2D& start,
                                  const PathPoint2D& end) const;
    double polygonSignedArea(const ConvexPolygon& polygon) const;
    double polygonArea(const ConvexPolygon& polygon) const;
    Point2D polygonCentroid(const ConvexPolygon& polygon) const;
    void ensureCounterClockwise(ConvexPolygon& polygon) const;
    bool polygonIsUsable(const ConvexPolygon& polygon) const;
    std::size_t computeUsableCorridorPrefix(
        const std::vector<CorridorSegment>& corridors,
        double& usable_length,
        int& first_failure_index,
        std::string& first_failure_reason) const;
    void publishReferencePath(const std::vector<PathPoint2D>& path,
                              const ros::Publisher& publisher,
                              double z) const;
    void publishCorridorMarkers(
        const std::vector<CorridorSegment>& corridors,
        std::size_t usable_prefix_count,
        bool preferred_length_available);

    // 快照与仅X减速安全盾。
    bool copyCorridorReferenceSnapshot(
        CorridorReferenceSnapshot& snapshot) const;
    void storeCorridorReferenceSnapshot(
        const CorridorReferenceSnapshot& snapshot);
    void invalidateCorridorReferenceSnapshot();
    double signedPointMarginInConvexPolygon(
        const ConvexPolygon& polygon,
        const Point2D& point) const;
    std::vector<Point2D> buildFootprintSamplePoints(
        double x,
        double y,
        double yaw,
        double extra_margin) const;
    double footprintMarginInCorridorUnion(
        double x,
        double y,
        double yaw,
        const CorridorReferenceSnapshot& snapshot,
        ConvexPolygon* footprint_output) const;
    bool evaluateCommandSafety(
        const geometry_msgs::Twist& command,
        const CorridorReferenceSnapshot& snapshot,
        const geometry_msgs::PoseStamped& robot_pose_costmap,
        SafetyCheckReport& report) const;
    bool applyFixedXBrakeSafetyShield(
        const geometry_msgs::Twist& nominal_cmd,
        geometry_msgs::Twist& safe_cmd,
        SafetyCheckReport& nominal_report,
        SafetyCheckReport& selected_report,
        double& selected_x_scale,
        bool& unsafe_now);
    void publishSafetyDebug(const SafetyCheckReport& nominal_report,
                            const SafetyCheckReport& selected_report) const;
    const char* safetyModeName() const;

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

    double lookahead_dist_;
    double path_linear_x_gain_;
    double path_linear_y_gain_;
    double path_angular_y_gain_;
    int lateral_search_points_;

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

    // 简化前向速度抑制器：只在明确预测出框时固定缩放linear.x。
    bool pp_safety_enable_;
    double pp_safety_snapshot_max_age_;
    double pp_safety_prediction_dt_;
    double pp_safety_prediction_horizon_;
    double pp_x_brake_margin_threshold_;
    double pp_safety_footprint_margin_;
    int pp_x_brake_min_outside_steps_;
    double pp_x_brake_scale_;
    int pp_x_brake_enter_cycles_;
    int pp_x_brake_exit_cycles_;
    int pp_x_brake_unsafe_count_;
    int pp_x_brake_safe_count_;
    SafetyMode safety_mode_;

    // 安全框参数。
    bool enable_corridor_visualization_;
    double corridor_update_frequency_;
    double local_path_behind_distance_;
    double local_path_horizon_distance_;
    double local_path_resample_distance_;
    double corridor_skeleton_corner_angle_deg_;
    double corridor_skeleton_corner_window_;
    double corridor_skeleton_max_segment_length_;
    double corridor_skeleton_min_segment_length_;
    double corridor_initial_half_width_;
    double corridor_longitudinal_extension_;
    double corridor_post_shrink_longitudinal_reserve_;
    double corridor_map_boundary_margin_;
    unsigned char corridor_hard_cost_threshold_;
    bool corridor_treat_unknown_as_obstacle_;
    double corridor_obstacle_padding_;
    int corridor_max_obstacle_cuts_;
    double corridor_min_polygon_area_;
    bool corridor_use_costmap_footprint_;
    double corridor_robot_half_length_;
    double corridor_robot_half_width_;
    double corridor_extra_margin_;
    std::vector<Point2D> corridor_robot_footprint_;
    double corridor_min_overlap_area_;
    double corridor_reference_validation_step_;
    double corridor_min_usable_chain_length_;
    double corridor_preferred_chain_length_;
    double corridor_terminal_ignore_distance_;
    int corridor_projection_search_behind_points_;
    int corridor_projection_search_ahead_points_;
    bool corridor_debug_log_;

    ros::Publisher corridor_markers_pub_;
    ros::Publisher corridor_reference_path_pub_;
    ros::Publisher pp_raw_prediction_pub_;
    ros::Publisher pp_safe_prediction_pub_;
    ros::Publisher pp_unsafe_footprint_pub_;

    mutable std::mutex corridor_reference_mutex_;
    CorridorReferenceSnapshot corridor_reference_snapshot_;
    std::uint64_t corridor_reference_revision_;

    ros::Time last_corridor_request_time_;
    bool force_corridor_update_;
    std::thread corridor_worker_thread_;
    mutable std::mutex corridor_worker_mutex_;
    std::condition_variable corridor_worker_condition_;
    bool corridor_worker_stop_;
    bool corridor_update_requested_;
    bool corridor_clear_requested_;
    std::atomic<bool> corridor_worker_busy_;
    std::atomic<bool> corridor_visualization_cleared_;

    mutable std::mutex corridor_plan_cache_mutex_;
    std::vector<PathPoint2D> corridor_cached_plan_;
    std::string corridor_cached_plan_frame_;
    std::uint64_t corridor_plan_generation_;
    std::size_t corridor_progress_segment_index_;
    std::uint64_t corridor_worker_seen_generation_;
};

}  // namespace my_planner

#endif  // MY_PLANNER_H_