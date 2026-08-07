#ifndef CLEARANCE_PATH_OPTIMIZER_H_
#define CLEARANCE_PATH_OPTIMIZER_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace my_planner
{

// C4.1 第一版：只负责几何参考路径优化，不计算速度，不发布 cmd_vel，
// 也不拥有“停车/请求重规划”的权限。
class ClearancePathOptimizer
{
public:
    struct Report
    {
        bool fresh_result;
        bool accepted;
        bool used_cache;
        double compute_ms;
        double minimum_clearance;
        double maximum_lateral_offset;
        double maximum_curvature;
        int reference_points;
        int candidate_count;
        std::string status;
        std::string detail;

        Report();
    };

    struct Output
    {
        bool valid;
        std::vector<geometry_msgs::PoseStamped> reference_path;
        std::vector<geometry_msgs::PoseStamped> lattice_seed_path;
        std::vector<geometry_msgs::PoseStamped> optimized_local_path;
        std::vector<geometry_msgs::PoseStamped> optimized_full_path;
        Report report;

        Output();
    };

    ClearancePathOptimizer();

    void initialize(ros::NodeHandle& planner_private_nh,
                    costmap_2d::Costmap2D* costmap,
                    const std::string& costmap_frame);

    void reset();

    bool enabled() const;
    bool shadowMode() const;
    void setShadowMode(bool shadow_mode);

    // plan_costmap 中所有点必须已经位于 costmap_frame。
    // 返回 false 只表示本轮没有可输出的优化路径；调用者不得据此停车或重规划。
    bool optimize(
        const std::vector<geometry_msgs::PoseStamped>& plan_costmap,
        const geometry_msgs::PoseStamped& robot_pose_costmap,
        Output& output);

private:
    struct Config
    {
        bool enabled;
        bool shadow_mode;
        bool reject_unknown;
        bool publish_debug_paths;
        bool prefer_left_on_first_solution;
        double update_rate;
        double forward_window;
        double backward_window;
        double longitudinal_step;
        double lateral_step;
        double max_lateral_offset;
        double max_lateral_change_per_step;
        double commitment_distance;
        double end_blend_distance;
        double preferred_clearance;
        double distance_field_max_distance;
        double robot_half_length;
        double robot_half_width;
        double extra_clearance_margin;
        double weight_clearance;
        double weight_inflation;
        double weight_reference;
        double weight_history;
        double weight_lateral_slope;
        double weight_lateral_curvature;
        int elastic_iterations;
        double elastic_step_size;
        double elastic_max_step;
        double elastic_weight_seed;
        double elastic_weight_reference;
        double elastic_weight_history;
        double elastic_weight_smoothness;
        double elastic_weight_clearance;
        double centerline_check_step;
        double max_accepted_curvature;
        double max_history_jump;

        Config();
    };

    struct MapSnapshot
    {
        unsigned int size_x;
        unsigned int size_y;
        double resolution;
        double origin_x;
        double origin_y;
        std::vector<unsigned char> costs;
        std::vector<double> obstacle_distance;

        MapSnapshot();
    };

    struct ReferencePoint
    {
        double x;
        double y;
        double yaw;
        double normal_x;
        double normal_y;
        double local_s;
        double plan_s;

        ReferencePoint();
    };

    struct Candidate
    {
        bool valid;
        double d;
        double x;
        double y;
        double clearance;
        unsigned char cost;
        double node_cost;

        Candidate();
    };

    bool buildMapSnapshot(MapSnapshot& map) const;
    void buildEuclideanDistanceField(MapSnapshot& map) const;
    static void squaredDistanceTransform1D(
        const std::vector<double>& input,
        std::vector<double>& output);

    bool buildReferencePath(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const geometry_msgs::PoseStamped& robot_pose,
        std::vector<double>& plan_s,
        double& window_start_s,
        double& window_end_s,
        std::vector<ReferencePoint>& reference) const;

    bool interpolatePlan(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const std::vector<double>& plan_s,
        double query_s,
        double& x,
        double& y) const;

    bool worldToMapContinuous(
        const MapSnapshot& map,
        double wx,
        double wy,
        double& mx,
        double& my) const;

    bool queryDistanceAndGradient(
        const MapSnapshot& map,
        double wx,
        double wy,
        double& distance,
        double& gradient_x,
        double& gradient_y) const;

    bool evaluateCandidate(
        const MapSnapshot& map,
        const ReferencePoint& reference,
        double lateral_offset,
        Candidate& candidate) const;

    double historyOffsetForReference(
        const ReferencePoint& reference,
        bool& available) const;

    bool centerlineSegmentIsValid(
        const MapSnapshot& map,
        double x0,
        double y0,
        double x1,
        double y1) const;

    bool runLatticeDynamicProgramming(
        const MapSnapshot& map,
        const std::vector<ReferencePoint>& reference,
        std::vector<double>& seed_offsets,
        std::vector<double>& clearance_targets,
        int& candidate_count,
        std::string& failure_reason) const;

    void runElasticRefinement(
        const MapSnapshot& map,
        const std::vector<ReferencePoint>& reference,
        const std::vector<double>& clearance_targets,
        const std::vector<double>& seed_offsets,
        std::vector<double>& optimized_offsets) const;

    bool offsetsToLocalPath(
        const MapSnapshot& map,
        const std::vector<ReferencePoint>& reference,
        const std::vector<double>& offsets,
        std::vector<geometry_msgs::PoseStamped>& path,
        double& minimum_clearance,
        double& maximum_curvature) const;

    void buildFullPath(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const std::vector<double>& plan_s,
        double window_start_s,
        double window_end_s,
        const std::vector<ReferencePoint>& reference,
        const std::vector<double>& offsets,
        std::vector<geometry_msgs::PoseStamped>& full_path) const;

    bool validateOutputCenterline(
        const MapSnapshot& map,
        const std::vector<geometry_msgs::PoseStamped>& path) const;

    void publishOutput(const Output& output);
    void publishPath(const std::vector<geometry_msgs::PoseStamped>& path,
                     const ros::Publisher& publisher,
                     double z_offset) const;

    static double clamp(double value, double lower, double upper);
    static double normalizeAngle(double angle);
    static double elapsedMilliseconds(const ros::WallTime& begin);

private:
    bool initialized_;
    Config config_;
    costmap_2d::Costmap2D* costmap_;
    std::string costmap_frame_;
    ros::Time last_update_time_;
    Output cached_output_;
    std::vector<geometry_msgs::PoseStamped> last_accepted_local_path_;

    ros::Publisher reference_path_pub_;
    ros::Publisher lattice_seed_path_pub_;
    ros::Publisher optimized_path_pub_;
};

}  // namespace my_planner

#endif  // CLEARANCE_PATH_OPTIMIZER_H_
