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

// C5.4：Stable Active-Path Cubic B-Spline Optimizer
//
// 设计目标：
// 1. C5成为唯一净空轨迹优化器，不再区分C4/C5，也不再使用shadow_mode旁路。
// 2. 每个控制周期滚动生成当前active path；优化偶发失败时优先复用上一轮
//    横向offset重建当前窗口，避免MPC/巡检PP瞬间退回raw path。
// 3. 区分LETHAL / UNKNOWN / OUTSIDE_MAP；局部地图边缘只缩短前视窗口，
//    不再把“超出rolling costmap”误判为障碍。
// 4. raw path已经进入hard区域时，先沿路径法向搜索escape seed，再继续同一套
//    B-Spline连续优化。
// 5. 采用成熟在线轨迹优化器的“candidate / active trajectory”分离：fresh候选只有通过
//    安全验收才替换active path；数值未改善不再等价于失败。
// 6. 对全向底盘，曲率保留为软代价和质量统计，默认不再作为硬可行性门槛。
// 7. fresh / HOLD均失败时，允许在很短时间内继续复用最后一条已验证active path。
class ClearancePathOptimizer
{
public:
    struct Report
    {
        bool fresh_result;
        bool accepted;
        bool used_cache;
        bool used_escape_seed;
        bool reused_active_path;
        bool curvature_warning;
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
        std::vector<geometry_msgs::PoseStamped> seed_path;
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

    bool optimize(
        const std::vector<geometry_msgs::PoseStamped>& plan_costmap,
        const geometry_msgs::PoseStamped& robot_pose_costmap,
        Output& output);

private:
    struct Config
    {
        bool enabled;
        bool reject_unknown;
        bool publish_debug_paths;
        bool prefer_left_on_first_solution;
        bool hold_last_valid;

        double forward_window;
        double backward_window;
        double min_window_length;
        double control_point_spacing;
        double sample_step;
        double map_edge_margin;

        double commitment_distance;
        double start_blend_distance;
        double end_blend_distance;

        int activation_cost;
        int deactivation_cost;
        int hard_cost_threshold;
        double max_lateral_offset;
        double max_offset_delta_per_control;

        int max_iterations;
        double finite_difference_epsilon;
        double learning_rate;
        double max_offset_step;
        double convergence_delta;

        double weight_cost;
        double weight_reference;
        double weight_slope;
        double weight_curvature;
        double hard_collision_penalty;

        double initial_probe_offset;
        double escape_probe_step;
        double warm_start_weight;
        double warm_start_max_distance;

        bool reuse_last_active;
        double active_reuse_max_age;
        double active_reuse_goal_tolerance;
        double curvature_warn_threshold;
        double curvature_hard_limit;

        Config();
    };

    struct Vec2
    {
        double x;
        double y;

        Vec2();
        Vec2(double x_in, double y_in);

        Vec2 operator+(const Vec2& rhs) const;
        Vec2 operator-(const Vec2& rhs) const;
        Vec2 operator*(double scale) const;
        Vec2 operator/(double scale) const;
    };

    struct MapSnapshot
    {
        unsigned int size_x;
        unsigned int size_y;
        double resolution;
        double origin_x;
        double origin_y;
        std::vector<unsigned char> costs;

        MapSnapshot();
    };

    enum class MapSampleState
    {
        VALID = 0,
        LETHAL,
        UNKNOWN,
        OUTSIDE_MAP,
        INVALID_MAP
    };

    struct MapSample
    {
        double cost;
        MapSampleState state;

        MapSample();
        MapSample(double cost_in, MapSampleState state_in);
    };

    enum class OutputValidationStatus
    {
        VALID = 0,
        INVALID_PATH,
        HARD_COLLISION,
        CURVATURE_EMERGENCY
    };

    struct LocalProblem
    {
        std::vector<double> plan_s;
        double total_s;
        double window_start_s;
        double window_end_s;
        int nearest_index;

        std::vector<double> control_s;
        std::vector<Vec2> anchors;
        std::vector<Vec2> normals;
        std::vector<bool> fixed;

        LocalProblem();
    };

    bool buildMapSnapshot(MapSnapshot& map) const;

    bool buildLocalProblem(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const geometry_msgs::PoseStamped& robot_pose,
        const MapSnapshot& map,
        LocalProblem& problem) const;

    bool interpolatePlan(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const std::vector<double>& plan_s,
        double query_s,
        Vec2& point) const;

    Vec2 rawNormalAtArc(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const std::vector<double>& plan_s,
        double query_s) const;

    bool pointInsideMap(
        const MapSnapshot& map,
        double wx,
        double wy,
        double margin) const;

    MapSample sampleMap(
        const MapSnapshot& map,
        double wx,
        double wy) const;

    bool isHardSample(const MapSample& sample) const;

    double normalizedSoftCost(double raw_cost) const;

    bool evaluateRawWindow(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const MapSnapshot& map,
        double& maximum_cost,
        bool& hard_collision) const;

    std::vector<Vec2> buildControlPoints(
        const LocalProblem& problem,
        const std::vector<double>& offsets) const;

    Vec2 evaluateClampedCubicBspline(
        const std::vector<Vec2>& control_points,
        double u) const;

    double objective(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const MapSnapshot& map,
        const std::vector<double>& offsets,
        double* maximum_cost,
        bool* hard_collision) const;

    bool mapPreviousOffsets(
        const LocalProblem& problem,
        double weight,
        std::vector<double>& offsets) const;

    void initializeOffsets(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const MapSnapshot& map,
        std::vector<double>& offsets,
        std::vector<double>& seed_offsets) const;

    bool findEscapeSeed(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const MapSnapshot& map,
        std::vector<double>& offsets) const;

    void enforceOffsetConstraints(
        const LocalProblem& problem,
        std::vector<double>& offsets) const;

    bool optimizeOffsets(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const MapSnapshot& map,
        std::vector<double>& offsets,
        double& final_objective,
        double& final_maximum_cost) const;

    void offsetsToSampledPath(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const std::vector<double>& offsets,
        std::vector<geometry_msgs::PoseStamped>& path) const;

    void buildFullPath(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const std::vector<double>& offsets,
        std::vector<geometry_msgs::PoseStamped>& full_path) const;

    OutputValidationStatus validateOutput(
        const MapSnapshot& map,
        const std::vector<geometry_msgs::PoseStamped>& local_path,
        double& maximum_cost,
        double& maximum_curvature) const;

    bool buildHoldOutput(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const LocalProblem& problem,
        const MapSnapshot& map,
        Output& output,
        const std::string& reason);

    bool buildCachedActiveOutput(
        const std::vector<geometry_msgs::PoseStamped>& plan,
        const MapSnapshot& map,
        Output& output,
        const std::string& reason);

    void cacheActiveOutput(const Output& output);

    void updateWarmStart(
        const LocalProblem& problem,
        const std::vector<double>& offsets);

    void publishOutput(const Output& output);
    void publishPath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const ros::Publisher& publisher,
        double z_offset) const;

    void logStateTransition(
        const std::string& status,
        const std::string& detail);

    static double clamp(double value, double lower, double upper);
    static double smoothStep(double value);
    static double norm(const Vec2& value);
    static double squaredNorm(const Vec2& value);
    static double elapsedMilliseconds(const ros::WallTime& begin);

private:
    bool initialized_;
    bool avoidance_active_;
    Config config_;
    costmap_2d::Costmap2D* costmap_;
    std::string costmap_frame_;

    std::vector<Vec2> previous_anchors_;
    std::vector<double> previous_offsets_;

    // candidate / active trajectory分离：只缓存已经通过安全验收的active path。
    std::vector<geometry_msgs::PoseStamped> last_active_local_path_;
    std::vector<geometry_msgs::PoseStamped> last_active_full_path_;
    ros::WallTime last_active_stamp_;

    std::string last_runtime_state_;

    ros::Publisher reference_path_pub_;
    ros::Publisher seed_path_pub_;
    ros::Publisher optimized_path_pub_;
};

}  // namespace my_planner

#endif  // CLEARANCE_PATH_OPTIMIZER_H_
