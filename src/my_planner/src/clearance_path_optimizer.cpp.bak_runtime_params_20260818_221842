// C5.4_BUILD：IFLY2026_C5_4_REPLAN_POLICY_STABILIZATION_20260810
#include "clearance_path_optimizer.h"

#include <boost/thread/locks.hpp>
#include <costmap_2d/cost_values.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace my_planner
{


ClearancePathOptimizer::Report::Report()
    : fresh_result(false),
      accepted(false),
      used_cache(false),
      used_escape_seed(false),
      reused_active_path(false),
      curvature_warning(false),
      compute_ms(0.0),
      minimum_clearance(0.0),
      maximum_lateral_offset(0.0),
      maximum_curvature(0.0),
      reference_points(0),
      candidate_count(0)
{
}

ClearancePathOptimizer::Output::Output()
    : valid(false)
{
}


ClearancePathOptimizer::Config::Config()
    : enabled(true),
      reject_unknown(true),
      publish_debug_paths(true),
      prefer_left_on_first_solution(true),
      hold_last_valid(true),
      forward_window(1.50),
      backward_window(0.08),
      min_window_length(0.35),
      control_point_spacing(0.12),
      sample_step(0.025),
      map_edge_margin(0.05),
      commitment_distance(0.10),
      start_blend_distance(0.10),
      end_blend_distance(0.22),
      activation_cost(70),
      deactivation_cost(50),
      hard_cost_threshold(254),
      max_lateral_offset(0.30),
      max_offset_delta_per_control(0.07),
      max_iterations(8),
      finite_difference_epsilon(0.004),
      learning_rate(0.030),
      max_offset_step(0.018),
      convergence_delta(1.0e-4),
      weight_cost(28.0),
      weight_reference(2.5),
      weight_slope(18.0),
      weight_curvature(70.0),
      hard_collision_penalty(1500.0),
      initial_probe_offset(0.025),
      escape_probe_step(0.025),
      warm_start_weight(0.75),
      warm_start_max_distance(0.22),
      reuse_last_active(true),
      active_reuse_max_age(0.35),
      active_reuse_goal_tolerance(0.10),
      curvature_warn_threshold(12.0),
      curvature_hard_limit(0.0)
{
}

ClearancePathOptimizer::Vec2::Vec2()
    : x(0.0), y(0.0)
{
}

ClearancePathOptimizer::Vec2::Vec2(double x_in, double y_in)
    : x(x_in), y(y_in)
{
}

ClearancePathOptimizer::Vec2 ClearancePathOptimizer::Vec2::operator+(
    const Vec2& rhs) const
{
    return Vec2(x + rhs.x, y + rhs.y);
}

ClearancePathOptimizer::Vec2 ClearancePathOptimizer::Vec2::operator-(
    const Vec2& rhs) const
{
    return Vec2(x - rhs.x, y - rhs.y);
}

ClearancePathOptimizer::Vec2 ClearancePathOptimizer::Vec2::operator*(
    double scale) const
{
    return Vec2(x * scale, y * scale);
}

ClearancePathOptimizer::Vec2 ClearancePathOptimizer::Vec2::operator/(
    double scale) const
{
    if (std::abs(scale) <= 1.0e-12)
        return Vec2();
    return Vec2(x / scale, y / scale);
}

ClearancePathOptimizer::MapSnapshot::MapSnapshot()
    : size_x(0),
      size_y(0),
      resolution(0.0),
      origin_x(0.0),
      origin_y(0.0)
{
}


ClearancePathOptimizer::MapSample::MapSample()
    : cost(255.0),
      state(MapSampleState::INVALID_MAP)
{
}

ClearancePathOptimizer::MapSample::MapSample(
    double cost_in,
    MapSampleState state_in)
    : cost(cost_in),
      state(state_in)
{
}

ClearancePathOptimizer::LocalProblem::LocalProblem()
    : total_s(0.0),
      window_start_s(0.0),
      window_end_s(0.0),
      nearest_index(0)
{
}


ClearancePathOptimizer::ClearancePathOptimizer()
    : initialized_(false),
      avoidance_active_(false),
      costmap_(NULL)
{
}

double ClearancePathOptimizer::clamp(
    double value,
    double lower,
    double upper)
{
    return std::max(lower, std::min(value, upper));
}

double ClearancePathOptimizer::smoothStep(double value)
{
    const double t = clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double ClearancePathOptimizer::norm(const Vec2& value)
{
    return std::hypot(value.x, value.y);
}

double ClearancePathOptimizer::squaredNorm(const Vec2& value)
{
    return value.x * value.x + value.y * value.y;
}

double ClearancePathOptimizer::elapsedMilliseconds(
    const ros::WallTime& begin)
{
    return (ros::WallTime::now() - begin).toSec() * 1000.0;
}


void ClearancePathOptimizer::initialize(
    ros::NodeHandle& planner_private_nh,
    costmap_2d::Costmap2D* costmap,
    const std::string& costmap_frame)
{
    costmap_ = costmap;
    costmap_frame_ = costmap_frame;

    ros::NodeHandle nh(planner_private_nh, "clearance_optimizer");

    nh.param("enabled", config_.enabled, config_.enabled);
    nh.param("reject_unknown", config_.reject_unknown, config_.reject_unknown);
    nh.param("publish_debug_paths",
             config_.publish_debug_paths,
             config_.publish_debug_paths);
    nh.param("prefer_left_on_first_solution",
             config_.prefer_left_on_first_solution,
             config_.prefer_left_on_first_solution);
    nh.param("hold_last_valid",
             config_.hold_last_valid,
             config_.hold_last_valid);

    nh.param("forward_window", config_.forward_window, config_.forward_window);
    nh.param("backward_window", config_.backward_window, config_.backward_window);
    nh.param("min_window_length",
             config_.min_window_length,
             config_.min_window_length);
    nh.param("control_point_spacing",
             config_.control_point_spacing,
             config_.control_point_spacing);
    nh.param("sample_step", config_.sample_step, config_.sample_step);
    nh.param("map_edge_margin",
             config_.map_edge_margin,
             config_.map_edge_margin);

    nh.param("commitment_distance",
             config_.commitment_distance,
             config_.commitment_distance);
    nh.param("start_blend_distance",
             config_.start_blend_distance,
             config_.start_blend_distance);
    nh.param("end_blend_distance",
             config_.end_blend_distance,
             config_.end_blend_distance);

    nh.param("activation_cost", config_.activation_cost, config_.activation_cost);
    nh.param("deactivation_cost",
             config_.deactivation_cost,
             config_.deactivation_cost);
    nh.param("hard_cost_threshold",
             config_.hard_cost_threshold,
             config_.hard_cost_threshold);
    nh.param("max_lateral_offset",
             config_.max_lateral_offset,
             config_.max_lateral_offset);
    nh.param("max_offset_delta_per_control",
             config_.max_offset_delta_per_control,
             config_.max_offset_delta_per_control);

    nh.param("max_iterations", config_.max_iterations, config_.max_iterations);
    nh.param("finite_difference_epsilon",
             config_.finite_difference_epsilon,
             config_.finite_difference_epsilon);
    nh.param("learning_rate", config_.learning_rate, config_.learning_rate);
    nh.param("max_offset_step", config_.max_offset_step, config_.max_offset_step);
    nh.param("convergence_delta",
             config_.convergence_delta,
             config_.convergence_delta);

    nh.param("weight_cost", config_.weight_cost, config_.weight_cost);
    nh.param("weight_reference",
             config_.weight_reference,
             config_.weight_reference);
    nh.param("weight_slope", config_.weight_slope, config_.weight_slope);
    nh.param("weight_curvature",
             config_.weight_curvature,
             config_.weight_curvature);
    nh.param("hard_collision_penalty",
             config_.hard_collision_penalty,
             config_.hard_collision_penalty);

    nh.param("initial_probe_offset",
             config_.initial_probe_offset,
             config_.initial_probe_offset);
    nh.param("escape_probe_step",
             config_.escape_probe_step,
             config_.escape_probe_step);
    nh.param("warm_start_weight",
             config_.warm_start_weight,
             config_.warm_start_weight);
    nh.param("warm_start_max_distance",
             config_.warm_start_max_distance,
             config_.warm_start_max_distance);

    nh.param("reuse_last_active",
             config_.reuse_last_active,
             config_.reuse_last_active);
    nh.param("active_reuse_max_age",
             config_.active_reuse_max_age,
             config_.active_reuse_max_age);
    nh.param("active_reuse_goal_tolerance",
             config_.active_reuse_goal_tolerance,
             config_.active_reuse_goal_tolerance);
    nh.param("curvature_warn_threshold",
             config_.curvature_warn_threshold,
             config_.curvature_warn_threshold);
    nh.param("curvature_hard_limit",
             config_.curvature_hard_limit,
             config_.curvature_hard_limit);

    config_.forward_window = std::max(0.30, config_.forward_window);
    config_.backward_window = std::max(0.0, config_.backward_window);
    config_.min_window_length = std::max(0.20, config_.min_window_length);
    config_.control_point_spacing = clamp(
        config_.control_point_spacing, 0.06, 0.30);
    config_.sample_step = clamp(config_.sample_step, 0.01, 0.10);
    config_.map_edge_margin = std::max(0.0, config_.map_edge_margin);

    config_.commitment_distance = std::max(0.0, config_.commitment_distance);
    config_.start_blend_distance = std::max(0.02, config_.start_blend_distance);
    config_.end_blend_distance = std::max(0.02, config_.end_blend_distance);

    config_.activation_cost = std::max(
        1, std::min(config_.activation_cost, 253));
    config_.deactivation_cost = std::max(
        0, std::min(config_.deactivation_cost, config_.activation_cost - 1));
    config_.hard_cost_threshold = std::max(
        config_.activation_cost + 1,
        std::min(config_.hard_cost_threshold, 254));
    config_.max_lateral_offset =
        std::max(0.02, config_.max_lateral_offset);
    config_.max_offset_delta_per_control =
        std::max(0.01, config_.max_offset_delta_per_control);

    config_.max_iterations =
        std::max(1, std::min(config_.max_iterations, 30));
    config_.finite_difference_epsilon = clamp(
        config_.finite_difference_epsilon, 0.001, 0.02);
    config_.learning_rate = clamp(config_.learning_rate, 0.001, 0.20);
    config_.max_offset_step = clamp(config_.max_offset_step, 0.002, 0.05);
    config_.convergence_delta = std::max(
        1.0e-8, config_.convergence_delta);

    config_.weight_cost = std::max(0.0, config_.weight_cost);
    config_.weight_reference = std::max(0.0, config_.weight_reference);
    config_.weight_slope = std::max(0.0, config_.weight_slope);
    config_.weight_curvature = std::max(0.0, config_.weight_curvature);
    config_.hard_collision_penalty =
        std::max(100.0, config_.hard_collision_penalty);

    config_.initial_probe_offset = clamp(
        config_.initial_probe_offset,
        0.0,
        config_.max_lateral_offset);
    config_.escape_probe_step = clamp(
        config_.escape_probe_step,
        0.005,
        config_.max_lateral_offset);
    config_.warm_start_weight = clamp(
        config_.warm_start_weight, 0.0, 1.0);
    config_.warm_start_max_distance =
        std::max(0.02, config_.warm_start_max_distance);

    config_.active_reuse_max_age = clamp(
        config_.active_reuse_max_age, 0.0, 1.0);
    config_.active_reuse_goal_tolerance = std::max(
        0.01, config_.active_reuse_goal_tolerance);
    config_.curvature_warn_threshold = std::max(
        0.0, config_.curvature_warn_threshold);
    config_.curvature_hard_limit = std::max(
        0.0, config_.curvature_hard_limit);

    if (config_.publish_debug_paths)
    {
        reference_path_pub_ = nh.advertise<nav_msgs::Path>(
            "c5_reference_path", 1, true);
        seed_path_pub_ = nh.advertise<nav_msgs::Path>(
            "c5_seed_path", 1, true);
        optimized_path_pub_ = nh.advertise<nav_msgs::Path>(
            "c5_optimized_path", 1, true);
    }

    initialized_ = true;
    reset();

    ROS_WARN("C5.4稳定active-path优化器启动：enabled=%s，"
             "窗口=后%.2f/前%.2fm，控制点间距=%.2fm，采样=%.3fm，"
             "激活/释放cost=%d/%d，hard=%d，横移=±%.2fm，"
             "escape步长=%.3fm，HOLD=%s，active复用=%s/%.2fs，"
             "曲率warn/hard=%.1f/%.1f。",
             config_.enabled ? "是" : "否",
             config_.backward_window,
             config_.forward_window,
             config_.control_point_spacing,
             config_.sample_step,
             config_.activation_cost,
             config_.deactivation_cost,
             config_.hard_cost_threshold,
             config_.max_lateral_offset,
             config_.escape_probe_step,
             config_.hold_last_valid ? "开启" : "关闭",
             config_.reuse_last_active ? "开启" : "关闭",
             config_.active_reuse_max_age,
             config_.curvature_warn_threshold,
             config_.curvature_hard_limit);
}


void ClearancePathOptimizer::reset()
{
    previous_anchors_.clear();
    previous_offsets_.clear();
    last_active_local_path_.clear();
    last_active_full_path_.clear();
    last_active_stamp_ = ros::WallTime(0);
    avoidance_active_ = false;
    last_runtime_state_.clear();

    // C5.4明确禁止在普通reset时发布空Path。
    // RViz始终保留最后一次有效路径，直到下一条active path覆盖。
}

bool ClearancePathOptimizer::enabled() const
{
    return initialized_ && config_.enabled;
}

bool ClearancePathOptimizer::buildMapSnapshot(MapSnapshot& map) const
{
    if (costmap_ == NULL)
        return false;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
        lock(*(costmap_->getMutex()));

    map.size_x = costmap_->getSizeInCellsX();
    map.size_y = costmap_->getSizeInCellsY();
    map.resolution = costmap_->getResolution();
    map.origin_x = costmap_->getOriginX();
    map.origin_y = costmap_->getOriginY();

    const std::size_t cell_count =
        static_cast<std::size_t>(map.size_x)
        * static_cast<std::size_t>(map.size_y);

    if (cell_count == 0 || map.resolution <= 0.0)
        return false;

    const unsigned char* source = costmap_->getCharMap();
    if (source == NULL)
        return false;

    map.costs.assign(source, source + cell_count);
    return true;
}

bool ClearancePathOptimizer::interpolatePlan(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const std::vector<double>& plan_s,
    double query_s,
    Vec2& point) const
{
    if (plan.empty() || plan_s.size() != plan.size())
        return false;

    if (plan.size() == 1)
    {
        point = Vec2(plan.front().pose.position.x,
                     plan.front().pose.position.y);
        return true;
    }

    query_s = clamp(query_s, 0.0, plan_s.back());

    std::vector<double>::const_iterator upper = std::lower_bound(
        plan_s.begin(), plan_s.end(), query_s);

    if (upper == plan_s.begin())
    {
        point = Vec2(plan.front().pose.position.x,
                     plan.front().pose.position.y);
        return true;
    }

    if (upper == plan_s.end())
    {
        point = Vec2(plan.back().pose.position.x,
                     plan.back().pose.position.y);
        return true;
    }

    const std::size_t hi = static_cast<std::size_t>(upper - plan_s.begin());
    const std::size_t lo = hi - 1;
    const double ds = plan_s[hi] - plan_s[lo];
    const double ratio = ds <= 1.0e-9
        ? 0.0
        : (query_s - plan_s[lo]) / ds;

    const Vec2 a(plan[lo].pose.position.x, plan[lo].pose.position.y);
    const Vec2 b(plan[hi].pose.position.x, plan[hi].pose.position.y);
    point = a * (1.0 - ratio) + b * ratio;
    return true;
}

ClearancePathOptimizer::Vec2 ClearancePathOptimizer::rawNormalAtArc(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const std::vector<double>& plan_s,
    double query_s) const
{
    const double delta = std::max(0.03, 0.5 * config_.control_point_spacing);

    Vec2 before;
    Vec2 after;
    interpolatePlan(plan, plan_s, query_s - delta, before);
    interpolatePlan(plan, plan_s, query_s + delta, after);

    Vec2 tangent = after - before;
    double tangent_norm = norm(tangent);

    if (tangent_norm < 1.0e-8)
    {
        tangent = Vec2(1.0, 0.0);
        tangent_norm = 1.0;
    }

    tangent = tangent / tangent_norm;
    return Vec2(-tangent.y, tangent.x);
}


bool ClearancePathOptimizer::buildLocalProblem(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const geometry_msgs::PoseStamped& robot_pose,
    const MapSnapshot& map,
    LocalProblem& problem) const
{
    if (plan.size() < 4)
        return false;

    problem.plan_s.assign(plan.size(), 0.0);
    for (std::size_t i = 1; i < plan.size(); ++i)
    {
        const double dx =
            plan[i].pose.position.x - plan[i - 1].pose.position.x;
        const double dy =
            plan[i].pose.position.y - plan[i - 1].pose.position.y;
        problem.plan_s[i] =
            problem.plan_s[i - 1] + std::hypot(dx, dy);
    }

    problem.total_s = problem.plan_s.back();
    if (problem.total_s < config_.min_window_length)
        return false;

    double nearest_distance = std::numeric_limits<double>::max();
    problem.nearest_index = 0;

    for (std::size_t i = 0; i < plan.size(); ++i)
    {
        const double dx =
            plan[i].pose.position.x - robot_pose.pose.position.x;
        const double dy =
            plan[i].pose.position.y - robot_pose.pose.position.y;
        const double distance = std::hypot(dx, dy);

        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            problem.nearest_index = static_cast<int>(i);
        }
    }

    const double nearest_s = problem.plan_s[
        static_cast<std::size_t>(problem.nearest_index)];

    double desired_start_s = std::max(
        0.0, nearest_s - config_.backward_window);
    const double desired_end_s = std::min(
        problem.total_s, nearest_s + config_.forward_window);

    // C5.4：rolling local costmap边缘只缩短窗口，不再视为unknown/hard。
    // 先确保窗口起点在地图内部（正常情况下机器人附近一定满足）。
    Vec2 sample_point;
    while (desired_start_s < desired_end_s)
    {
        if (!interpolatePlan(plan, problem.plan_s, desired_start_s, sample_point))
            return false;

        if (pointInsideMap(
                map,
                sample_point.x,
                sample_point.y,
                config_.map_edge_margin))
        {
            break;
        }

        desired_start_s += config_.sample_step;
    }

    double map_limited_end_s = desired_start_s;
    bool saw_inside = false;

    for (double s = desired_start_s;
         s <= desired_end_s + 1.0e-9;
         s += config_.sample_step)
    {
        const double query_s = std::min(s, desired_end_s);
        if (!interpolatePlan(plan, problem.plan_s, query_s, sample_point))
            return false;

        if (!pointInsideMap(
                map,
                sample_point.x,
                sample_point.y,
                config_.map_edge_margin))
        {
            break;
        }

        saw_inside = true;
        map_limited_end_s = query_s;

        if (query_s >= desired_end_s - 1.0e-9)
            break;
    }

    if (!saw_inside)
        return false;

    problem.window_start_s = desired_start_s;
    problem.window_end_s = std::min(desired_end_s, map_limited_end_s);

    const double window_length =
        problem.window_end_s - problem.window_start_s;

    if (window_length < config_.min_window_length)
        return false;

    const int interval_count = std::max(
        5,
        static_cast<int>(std::ceil(
            window_length / config_.control_point_spacing)));
    const int control_count = interval_count + 1;

    problem.control_s.clear();
    problem.anchors.clear();
    problem.normals.clear();
    problem.fixed.clear();

    problem.control_s.reserve(control_count);
    problem.anchors.reserve(control_count);
    problem.normals.reserve(control_count);
    problem.fixed.reserve(control_count);

    for (int i = 0; i < control_count; ++i)
    {
        const double ratio = static_cast<double>(i)
            / static_cast<double>(control_count - 1);
        const double s =
            problem.window_start_s + ratio * window_length;

        Vec2 anchor;
        if (!interpolatePlan(plan, problem.plan_s, s, anchor))
            return false;

        problem.control_s.push_back(s);
        problem.anchors.push_back(anchor);
        problem.normals.push_back(
            rawNormalAtArc(plan, problem.plan_s, s));

        const double from_start =
            s - problem.window_start_s;

        // 只固定近车承诺段。末端由end_blend_distance连续回接raw path，
        // 不再把整段末端控制点锁死，否则障碍刚进入前视远端时无法提前侧移。
        problem.fixed.push_back(
            from_start <= config_.commitment_distance);
    }

    return problem.anchors.size() >= 4;
}


bool ClearancePathOptimizer::pointInsideMap(
    const MapSnapshot& map,
    double wx,
    double wy,
    double margin) const
{
    if (map.size_x < 2 || map.size_y < 2 || map.resolution <= 0.0)
        return false;

    const double min_x = map.origin_x + margin;
    const double min_y = map.origin_y + margin;
    const double max_x =
        map.origin_x + static_cast<double>(map.size_x) * map.resolution - margin;
    const double max_y =
        map.origin_y + static_cast<double>(map.size_y) * map.resolution - margin;

    return wx >= min_x && wx <= max_x
        && wy >= min_y && wy <= max_y;
}

ClearancePathOptimizer::MapSample ClearancePathOptimizer::sampleMap(
    const MapSnapshot& map,
    double wx,
    double wy) const
{
    if (map.size_x < 2 || map.size_y < 2 || map.resolution <= 0.0)
        return MapSample(255.0, MapSampleState::INVALID_MAP);

    // 硬状态按“采样点实际所在cell”判定；双线性邻居只用于连续soft cost。
    // 这样不会因为相邻一个LETHAL cell就把当前仍处于soft inflation的点误判为hard。
    const int mx = static_cast<int>(std::floor(
        (wx - map.origin_x) / map.resolution));
    const int my = static_cast<int>(std::floor(
        (wy - map.origin_y) / map.resolution));

    if (mx < 0 || my < 0
        || mx >= static_cast<int>(map.size_x)
        || my >= static_cast<int>(map.size_y))
    {
        return MapSample(255.0, MapSampleState::OUTSIDE_MAP);
    }

    const std::size_t center_index =
        static_cast<std::size_t>(my) * map.size_x
        + static_cast<std::size_t>(mx);
    const unsigned char center_cost = map.costs[center_index];

    MapSampleState center_state = MapSampleState::VALID;
    if (center_cost == costmap_2d::NO_INFORMATION)
        center_state = MapSampleState::UNKNOWN;
    else if (center_cost >= static_cast<unsigned char>(config_.hard_cost_threshold))
        center_state = MapSampleState::LETHAL;

    const double gx =
        (wx - map.origin_x) / map.resolution - 0.5;
    const double gy =
        (wy - map.origin_y) / map.resolution - 0.5;

    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if (x0 < 0 || y0 < 0
        || x1 >= static_cast<int>(map.size_x)
        || y1 >= static_cast<int>(map.size_y))
    {
        // 点本身还在图内但已无法做完整双线性插值时，使用所在cell cost。
        const double fallback_cost =
            center_cost == costmap_2d::NO_INFORMATION
                ? 253.0
                : static_cast<double>(center_cost);
        return MapSample(fallback_cost, center_state);
    }

    const std::size_t i00 =
        static_cast<std::size_t>(y0) * map.size_x
        + static_cast<std::size_t>(x0);
    const std::size_t i10 =
        static_cast<std::size_t>(y0) * map.size_x
        + static_cast<std::size_t>(x1);
    const std::size_t i01 =
        static_cast<std::size_t>(y1) * map.size_x
        + static_cast<std::size_t>(x0);
    const std::size_t i11 =
        static_cast<std::size_t>(y1) * map.size_x
        + static_cast<std::size_t>(x1);

    const unsigned char c00 = map.costs[i00];
    const unsigned char c10 = map.costs[i10];
    const unsigned char c01 = map.costs[i01];
    const unsigned char c11 = map.costs[i11];

    const double tx = gx - static_cast<double>(x0);
    const double ty = gy - static_cast<double>(y0);

    // 邻域unknown仍按高软代价参与梯度，不直接把当前点判为unknown。
    const double d00 =
        c00 == costmap_2d::NO_INFORMATION ? 253.0 : c00;
    const double d10 =
        c10 == costmap_2d::NO_INFORMATION ? 253.0 : c10;
    const double d01 =
        c01 == costmap_2d::NO_INFORMATION ? 253.0 : c01;
    const double d11 =
        c11 == costmap_2d::NO_INFORMATION ? 253.0 : c11;

    const double c0 = d00 * (1.0 - tx) + d10 * tx;
    const double c1 = d01 * (1.0 - tx) + d11 * tx;
    const double interpolated_cost =
        c0 * (1.0 - ty) + c1 * ty;

    return MapSample(interpolated_cost, center_state);
}

bool ClearancePathOptimizer::isHardSample(
    const MapSample& sample) const
{
    if (sample.state == MapSampleState::LETHAL
        || sample.state == MapSampleState::OUTSIDE_MAP
        || sample.state == MapSampleState::INVALID_MAP)
    {
        return true;
    }

    if (sample.state == MapSampleState::UNKNOWN
        && config_.reject_unknown)
    {
        return true;
    }

    // soft cost使用双线性连续值；hard只由采样点所在cell状态决定。
    return false;
}

double ClearancePathOptimizer::normalizedSoftCost(double raw_cost) const
{
    if (raw_cost <= static_cast<double>(config_.activation_cost))
        return 0.0;

    if (raw_cost >= static_cast<double>(config_.hard_cost_threshold))
        return 1.0;

    const double denominator = std::max(
        1.0,
        static_cast<double>(
            config_.hard_cost_threshold - config_.activation_cost));

    return clamp(
        (raw_cost - config_.activation_cost) / denominator,
        0.0,
        1.0);
}


bool ClearancePathOptimizer::evaluateRawWindow(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const MapSnapshot& map,
    double& maximum_cost,
    bool& hard_collision) const
{
    maximum_cost = 0.0;
    hard_collision = false;

    const double length =
        problem.window_end_s - problem.window_start_s;
    const int sample_count = std::max(
        8,
        static_cast<int>(
            std::ceil(length / config_.sample_step)) + 1);

    for (int i = 0; i < sample_count; ++i)
    {
        const double ratio =
            static_cast<double>(i)
            / static_cast<double>(sample_count - 1);
        const double s =
            problem.window_start_s + ratio * length;

        Vec2 raw;
        if (!interpolatePlan(plan, problem.plan_s, s, raw))
            return false;

        const MapSample sample =
            sampleMap(map, raw.x, raw.y);

        maximum_cost = std::max(
            maximum_cost, sample.cost);

        if (isHardSample(sample))
            hard_collision = true;
    }

    return true;
}

std::vector<ClearancePathOptimizer::Vec2>
ClearancePathOptimizer::buildControlPoints(
    const LocalProblem& problem,
    const std::vector<double>& offsets) const
{
    std::vector<Vec2> controls;
    controls.reserve(problem.anchors.size());

    for (std::size_t i = 0; i < problem.anchors.size(); ++i)
    {
        const double offset = i < offsets.size() ? offsets[i] : 0.0;
        controls.push_back(
            problem.anchors[i] + problem.normals[i] * offset);
    }

    return controls;
}

ClearancePathOptimizer::Vec2
ClearancePathOptimizer::evaluateClampedCubicBspline(
    const std::vector<Vec2>& control_points,
    double u) const
{
    const int degree = 3;
    const int count = static_cast<int>(control_points.size());

    if (count == 0)
        return Vec2();

    if (count < degree + 1)
    {
        const double scaled = clamp(u, 0.0, 1.0)
            * static_cast<double>(count - 1);
        const int lo = std::max(
            0,
            std::min(count - 1, static_cast<int>(std::floor(scaled))));
        const int hi = std::min(count - 1, lo + 1);
        const double ratio = scaled - static_cast<double>(lo);
        return control_points[lo] * (1.0 - ratio)
            + control_points[hi] * ratio;
    }

    u = clamp(u, 0.0, 1.0);

    const int n = count - 1;
    const int knot_count = count + degree + 1;
    std::vector<double> knots(static_cast<std::size_t>(knot_count), 0.0);

    const int internal_count = count - degree - 1;
    for (int j = 1; j <= internal_count; ++j)
    {
        knots[degree + j] = static_cast<double>(j)
            / static_cast<double>(internal_count + 1);
    }
    for (int i = count; i < knot_count; ++i)
        knots[i] = 1.0;

    int span = n;
    if (u < 1.0)
    {
        for (int k = degree; k <= n; ++k)
        {
            if (u >= knots[k] && u < knots[k + 1])
            {
                span = k;
                break;
            }
        }
    }

    std::vector<Vec2> d(static_cast<std::size_t>(degree + 1));
    for (int j = 0; j <= degree; ++j)
        d[j] = control_points[span - degree + j];

    for (int r = 1; r <= degree; ++r)
    {
        for (int j = degree; j >= r; --j)
        {
            const int index = span - degree + j;
            const double denominator =
                knots[index + degree - r + 1] - knots[index];
            const double alpha = std::abs(denominator) <= 1.0e-12
                ? 0.0
                : (u - knots[index]) / denominator;

            d[j] = d[j - 1] * (1.0 - alpha) + d[j] * alpha;
        }
    }

    return d[degree];
}


double ClearancePathOptimizer::objective(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const MapSnapshot& map,
    const std::vector<double>& offsets,
    double* maximum_cost,
    bool* hard_collision) const
{
    if (maximum_cost != NULL)
        *maximum_cost = 0.0;
    if (hard_collision != NULL)
        *hard_collision = false;

    const std::vector<Vec2> controls =
        buildControlPoints(problem, offsets);

    const double length =
        problem.window_end_s - problem.window_start_s;
    const int sample_count = std::max(
        8,
        static_cast<int>(
            std::ceil(length / config_.sample_step)) + 1);

    double cost_term = 0.0;
    double geometry_reference_term = 0.0;
    double local_max_cost = 0.0;
    bool hard = false;

    for (int i = 0; i < sample_count; ++i)
    {
        const double u =
            static_cast<double>(i)
            / static_cast<double>(sample_count - 1);
        const double s =
            problem.window_start_s + u * length;

        const Vec2 spline =
            evaluateClampedCubicBspline(controls, u);

        Vec2 raw;
        interpolatePlan(plan, problem.plan_s, s, raw);

        const MapSample sample =
            sampleMap(map, spline.x, spline.y);

        local_max_cost = std::max(
            local_max_cost, sample.cost);

        if (isHardSample(sample))
        {
            hard = true;
            cost_term += config_.hard_collision_penalty;
        }

        const double soft =
            normalizedSoftCost(sample.cost);
        cost_term += soft * soft;

        geometry_reference_term +=
            squaredNorm(spline - raw);
    }

    cost_term /= static_cast<double>(sample_count);
    geometry_reference_term /=
        static_cast<double>(sample_count);

    double offset_reference_term = 0.0;
    for (std::size_t i = 0; i < offsets.size(); ++i)
        offset_reference_term += offsets[i] * offsets[i];

    double slope_term = 0.0;
    for (std::size_t i = 1; i < offsets.size(); ++i)
    {
        const double delta =
            offsets[i] - offsets[i - 1];
        slope_term += delta * delta;
    }

    double curvature_term = 0.0;
    for (std::size_t i = 1; i + 1 < offsets.size(); ++i)
    {
        const double second =
            offsets[i - 1]
            - 2.0 * offsets[i]
            + offsets[i + 1];
        curvature_term += second * second;
    }

    const double count = std::max(
        1.0,
        static_cast<double>(offsets.size()));

    offset_reference_term /= count;
    slope_term /= count;
    curvature_term /= count;

    if (maximum_cost != NULL)
        *maximum_cost = local_max_cost;
    if (hard_collision != NULL)
        *hard_collision = hard;

    return config_.weight_cost * cost_term
        + config_.weight_reference
            * (geometry_reference_term
               + offset_reference_term)
        + config_.weight_slope * slope_term
        + config_.weight_curvature * curvature_term;
}

void ClearancePathOptimizer::enforceOffsetConstraints(
    const LocalProblem& problem,
    std::vector<double>& offsets) const
{
    if (offsets.size() != problem.anchors.size())
        offsets.assign(problem.anchors.size(), 0.0);

    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        if (problem.fixed[i])
            offsets[i] = 0.0;
        else
            offsets[i] = clamp(
                offsets[i],
                -config_.max_lateral_offset,
                config_.max_lateral_offset);
    }

    // 前向、反向各做一次斜率限制，避免单侧传播造成锯齿。
    for (std::size_t i = 1; i < offsets.size(); ++i)
    {
        if (problem.fixed[i])
            continue;

        offsets[i] = clamp(
            offsets[i],
            offsets[i - 1] - config_.max_offset_delta_per_control,
            offsets[i - 1] + config_.max_offset_delta_per_control);
    }

    for (std::size_t reverse = offsets.size(); reverse > 1; --reverse)
    {
        const std::size_t i = reverse - 2;
        if (problem.fixed[i])
            continue;

        offsets[i] = clamp(
            offsets[i],
            offsets[i + 1] - config_.max_offset_delta_per_control,
            offsets[i + 1] + config_.max_offset_delta_per_control);
    }

    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        if (problem.fixed[i])
            offsets[i] = 0.0;
    }
}


bool ClearancePathOptimizer::mapPreviousOffsets(
    const LocalProblem& problem,
    double weight,
    std::vector<double>& offsets) const
{
    offsets.assign(problem.anchors.size(), 0.0);

    if (previous_anchors_.empty()
        || previous_anchors_.size() != previous_offsets_.size())
    {
        return false;
    }

    bool mapped_any = false;

    for (std::size_t i = 0; i < problem.anchors.size(); ++i)
    {
        if (problem.fixed[i])
            continue;

        double best_distance =
            std::numeric_limits<double>::max();
        int best_index = -1;

        for (std::size_t j = 0;
             j < previous_anchors_.size();
             ++j)
        {
            const double distance = norm(
                problem.anchors[i]
                - previous_anchors_[j]);

            if (distance < best_distance)
            {
                best_distance = distance;
                best_index = static_cast<int>(j);
            }
        }

        if (best_index >= 0
            && best_distance
                   <= config_.warm_start_max_distance)
        {
            offsets[i] = clamp(weight, 0.0, 1.0)
                * previous_offsets_[
                    static_cast<std::size_t>(best_index)];
            mapped_any = true;
        }
    }

    enforceOffsetConstraints(problem, offsets);
    return mapped_any;
}


void ClearancePathOptimizer::initializeOffsets(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const MapSnapshot& map,
    std::vector<double>& offsets,
    std::vector<double>& seed_offsets) const
{
    const bool used_warm_start =
        mapPreviousOffsets(
            problem,
            config_.warm_start_weight,
            offsets);

    // 首次无历史时给左右各一个小探针，解决对称inflation场中
    // 零偏移梯度接近0的问题。
    if (!used_warm_start
        && config_.initial_probe_offset > 1.0e-6)
    {
        std::vector<double> left(
            problem.anchors.size(), 0.0);
        std::vector<double> right(
            problem.anchors.size(), 0.0);

        for (std::size_t i = 0;
             i < problem.anchors.size();
             ++i)
        {
            if (!problem.fixed[i])
            {
                left[i] =
                    config_.initial_probe_offset;
                right[i] =
                    -config_.initial_probe_offset;
            }
        }

        enforceOffsetConstraints(problem, left);
        enforceOffsetConstraints(problem, right);

        const double left_cost =
            objective(
                plan, problem, map,
                left, NULL, NULL);
        const double right_cost =
            objective(
                plan, problem, map,
                right, NULL, NULL);

        if (std::abs(left_cost - right_cost)
            < 1.0e-9)
        {
            offsets =
                config_.prefer_left_on_first_solution
                    ? left : right;
        }
        else
        {
            offsets =
                left_cost < right_cost
                    ? left : right;
        }
    }

    enforceOffsetConstraints(problem, offsets);
    seed_offsets = offsets;
}


bool ClearancePathOptimizer::findEscapeSeed(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const MapSnapshot& map,
    std::vector<double>& offsets) const
{
    bool current_hard = false;
    objective(
        plan, problem, map,
        offsets, NULL, &current_hard);

    if (!current_hard)
        return true;

    const double step = std::max(
        0.005, config_.escape_probe_step);

    bool found = false;
    double best_objective =
        std::numeric_limits<double>::max();
    std::vector<double> best_offsets;

    // 优先顺序只用于完全对称时的确定性，不限制另一侧参与比较。
    const int signs[2] =
    {
        config_.prefer_left_on_first_solution ? 1 : -1,
        config_.prefer_left_on_first_solution ? -1 : 1
    };

    for (double amplitude = step;
         amplitude
             <= config_.max_lateral_offset + 1.0e-9;
         amplitude += step)
    {
        for (int side = 0; side < 2; ++side)
        {
            const double signed_amplitude =
                static_cast<double>(signs[side])
                * amplitude;

            std::vector<double> candidate(
                problem.anchors.size(), 0.0);

            const double length = std::max(
                1.0e-6,
                problem.window_end_s
                    - problem.window_start_s);

            for (std::size_t i = 0;
                 i < candidate.size();
                 ++i)
            {
                if (problem.fixed[i])
                    continue;

                const double s =
                    problem.control_s[i];

                const double start_alpha =
                    smoothStep(
                        (s
                         - problem.window_start_s
                         - config_.commitment_distance)
                        / config_.start_blend_distance);

                const double end_alpha =
                    smoothStep(
                        (problem.window_end_s - s)
                        / config_.end_blend_distance);

                const double profile =
                    std::min(start_alpha, end_alpha);

                candidate[i] =
                    signed_amplitude * profile;

                (void)length;
            }

            enforceOffsetConstraints(
                problem, candidate);

            bool hard = false;
            const double candidate_objective =
                objective(
                    plan, problem, map,
                    candidate, NULL, &hard);

            if (!hard
                && candidate_objective
                       < best_objective)
            {
                best_objective =
                    candidate_objective;
                best_offsets = candidate;
                found = true;
            }
        }

        // 找到能完全脱离hard的最小幅值层后就停止扩大横移。
        if (found)
            break;
    }

    if (!found)
        return false;

    offsets = best_offsets;
    enforceOffsetConstraints(problem, offsets);
    return true;
}

bool ClearancePathOptimizer::optimizeOffsets(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const MapSnapshot& map,
    std::vector<double>& offsets,
    double& final_objective,
    double& final_maximum_cost) const
{
    if (offsets.size() != problem.anchors.size())
        return false;

    double current_maximum_cost = 0.0;
    bool current_hard = false;
    double current_objective = objective(
        plan,
        problem,
        map,
        offsets,
        &current_maximum_cost,
        &current_hard);

    if (current_hard || !std::isfinite(current_objective))
        return false;

    std::vector<double> gradient(offsets.size(), 0.0);

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration)
    {
        std::fill(gradient.begin(), gradient.end(), 0.0);

        for (std::size_t i = 0; i < offsets.size(); ++i)
        {
            if (problem.fixed[i])
                continue;

            std::vector<double> plus = offsets;
            std::vector<double> minus = offsets;

            plus[i] += config_.finite_difference_epsilon;
            minus[i] -= config_.finite_difference_epsilon;
            enforceOffsetConstraints(problem, plus);
            enforceOffsetConstraints(problem, minus);

            const double plus_cost = objective(
                plan, problem, map, plus, NULL, NULL);
            const double minus_cost = objective(
                plan, problem, map, minus, NULL, NULL);

            if (!std::isfinite(plus_cost) || !std::isfinite(minus_cost))
            {
                gradient[i] = 0.0;
            }
            else
            {
                gradient[i] = (plus_cost - minus_cost)
                    / (2.0 * config_.finite_difference_epsilon);
            }
        }

        std::vector<double> candidate = offsets;
        for (std::size_t i = 0; i < candidate.size(); ++i)
        {
            if (problem.fixed[i])
                continue;

            double delta = -config_.learning_rate * gradient[i];
            delta = clamp(
                delta,
                -config_.max_offset_step,
                config_.max_offset_step);
            candidate[i] += delta;
        }
        enforceOffsetConstraints(problem, candidate);

        double candidate_maximum_cost = 0.0;
        bool candidate_hard = false;
        double candidate_objective = objective(
            plan,
            problem,
            map,
            candidate,
            &candidate_maximum_cost,
            &candidate_hard);

        // 固定学习率出现过冲时进行最多4次回溯。
        double scale = 0.5;
        int backtrack = 0;
        while ((candidate_hard
                || !std::isfinite(candidate_objective)
                || candidate_objective > current_objective)
               && backtrack < 4)
        {
            candidate = offsets;
            for (std::size_t i = 0; i < candidate.size(); ++i)
            {
                if (problem.fixed[i])
                    continue;

                double delta = -config_.learning_rate
                    * scale * gradient[i];
                delta = clamp(
                    delta,
                    -config_.max_offset_step,
                    config_.max_offset_step);
                candidate[i] += delta;
            }
            enforceOffsetConstraints(problem, candidate);

            candidate_objective = objective(
                plan,
                problem,
                map,
                candidate,
                &candidate_maximum_cost,
                &candidate_hard);

            scale *= 0.5;
            ++backtrack;
        }

        if (candidate_hard
            || !std::isfinite(candidate_objective)
            || candidate_objective > current_objective)
        {
            // 与TEB/在线平滑器类似：本轮没有更优step时保留当前可行seed，
            // 不把“没有进一步改善”定义成规划失败。
            break;
        }

        const double improvement =
            current_objective - candidate_objective;

        offsets.swap(candidate);
        current_objective = candidate_objective;
        current_maximum_cost = candidate_maximum_cost;

        if (improvement < config_.convergence_delta)
            break;
    }

    bool final_hard = false;
    final_objective = objective(
        plan,
        problem,
        map,
        offsets,
        &final_maximum_cost,
        &final_hard);

    if (final_hard
        || !std::isfinite(final_objective)
        || !std::isfinite(final_maximum_cost))
    {
        return false;
    }

    // 成熟在线轨迹优化器采用“可行候选替换active trajectory”的语义：
    // 只要最终seed/candidate安全且数值有效，就允许进入后续安全验收；
    // 不再要求每个滚动周期都必须相对raw显著降低objective/max cost。
    return true;
}

void ClearancePathOptimizer::offsetsToSampledPath(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const std::vector<double>& offsets,
    std::vector<geometry_msgs::PoseStamped>& path) const
{
    path.clear();

    const std::vector<Vec2> controls = buildControlPoints(problem, offsets);
    const double length = problem.window_end_s - problem.window_start_s;
    const int sample_count = std::max(
        8,
        static_cast<int>(std::ceil(length / config_.sample_step)) + 1);

    path.reserve(static_cast<std::size_t>(sample_count));

    for (int i = 0; i < sample_count; ++i)
    {
        const double u = static_cast<double>(i)
            / static_cast<double>(sample_count - 1);
        const double s = problem.window_start_s + u * length;

        Vec2 raw;
        interpolatePlan(plan, problem.plan_s, s, raw);
        const Vec2 spline = evaluateClampedCubicBspline(controls, u);

        double start_alpha = 1.0;
        if (s <= problem.window_start_s + config_.commitment_distance)
        {
            start_alpha = 0.0;
        }
        else
        {
            start_alpha = smoothStep(
                (s - problem.window_start_s - config_.commitment_distance)
                / config_.start_blend_distance);
        }

        const double end_alpha = smoothStep(
            (problem.window_end_s - s) / config_.end_blend_distance);
        const double alpha = std::min(start_alpha, end_alpha);

        const Vec2 blended = raw * (1.0 - alpha) + spline * alpha;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_frame_;
        pose.header.stamp = ros::Time(0);
        pose.pose.position.x = blended.x;
        pose.pose.position.y = blended.y;
        pose.pose.orientation.w = 1.0;
        path.push_back(pose);
    }
}

void ClearancePathOptimizer::buildFullPath(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const std::vector<double>& offsets,
    std::vector<geometry_msgs::PoseStamped>& full_path) const
{
    full_path = plan;

    const std::vector<Vec2> controls = buildControlPoints(problem, offsets);
    const double length = problem.window_end_s - problem.window_start_s;
    if (length <= 1.0e-9)
        return;

    for (std::size_t i = 0; i < full_path.size(); ++i)
    {
        const double s = problem.plan_s[i];
        if (s < problem.window_start_s || s > problem.window_end_s)
            continue;

        const double u = clamp(
            (s - problem.window_start_s) / length,
            0.0,
            1.0);

        const Vec2 raw(
            plan[i].pose.position.x,
            plan[i].pose.position.y);
        const Vec2 spline = evaluateClampedCubicBspline(controls, u);

        double start_alpha = 1.0;
        if (s <= problem.window_start_s + config_.commitment_distance)
        {
            start_alpha = 0.0;
        }
        else
        {
            start_alpha = smoothStep(
                (s - problem.window_start_s - config_.commitment_distance)
                / config_.start_blend_distance);
        }

        const double end_alpha = smoothStep(
            (problem.window_end_s - s) / config_.end_blend_distance);
        const double alpha = std::min(start_alpha, end_alpha);

        const Vec2 blended = raw * (1.0 - alpha) + spline * alpha;
        full_path[i].pose.position.x = blended.x;
        full_path[i].pose.position.y = blended.y;

        // orientation严格保留原plan，避免影响MyPlanner现有的初始姿态、
        // 最终姿态和固定巡检航向逻辑。
    }
}


ClearancePathOptimizer::OutputValidationStatus
ClearancePathOptimizer::validateOutput(
    const MapSnapshot& map,
    const std::vector<geometry_msgs::PoseStamped>& local_path,
    double& maximum_cost,
    double& maximum_curvature) const
{
    maximum_cost = 0.0;
    maximum_curvature = 0.0;

    if (local_path.size() < 3)
        return OutputValidationStatus::INVALID_PATH;

    // 第一层：真正的安全硬约束只检查LETHAL / UNKNOWN / OUTSIDE。
    for (std::size_t i = 0; i < local_path.size(); ++i)
    {
        const MapSample sample = sampleMap(
            map,
            local_path[i].pose.position.x,
            local_path[i].pose.position.y);

        maximum_cost = std::max(maximum_cost, sample.cost);

        if (isHardSample(sample))
            return OutputValidationStatus::HARD_COLLISION;
    }

    // 第二层：曲率只作为质量/可跟踪性统计。
    // 本车为全向底盘，几何路径曲率并不是非完整运动学的硬可行性约束。
    for (std::size_t i = 1; i + 1 < local_path.size(); ++i)
    {
        const double x0 = local_path[i - 1].pose.position.x;
        const double y0 = local_path[i - 1].pose.position.y;
        const double x1 = local_path[i].pose.position.x;
        const double y1 = local_path[i].pose.position.y;
        const double x2 = local_path[i + 1].pose.position.x;
        const double y2 = local_path[i + 1].pose.position.y;

        const double a = std::hypot(x1 - x0, y1 - y0);
        const double b = std::hypot(x2 - x1, y2 - y1);
        const double c = std::hypot(x2 - x0, y2 - y0);
        const double denominator = a * b * c;
        if (denominator <= 1.0e-9)
            continue;

        const double twice_area = std::abs(
            (x1 - x0) * (y2 - y0)
            - (y1 - y0) * (x2 - x0));

        const double curvature =
            2.0 * twice_area / denominator;

        if (std::isfinite(curvature))
        {
            maximum_curvature = std::max(
                maximum_curvature,
                curvature);
        }
    }

    // 默认0表示关闭曲率硬门槛；只有显式配置非零 emergency limit
    // 才允许它作为最后的异常轨迹保险。
    if (config_.curvature_hard_limit > 0.0
        && maximum_curvature > config_.curvature_hard_limit)
    {
        return OutputValidationStatus::CURVATURE_EMERGENCY;
    }

    return OutputValidationStatus::VALID;
}


bool ClearancePathOptimizer::buildHoldOutput(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const LocalProblem& problem,
    const MapSnapshot& map,
    Output& output,
    const std::string& reason)
{
    if (!config_.hold_last_valid)
        return false;

    std::vector<double> hold_offsets;
    if (!mapPreviousOffsets(
            problem,
            1.0,
            hold_offsets))
    {
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> hold_local;
    offsetsToSampledPath(
        plan,
        problem,
        hold_offsets,
        hold_local);

    double maximum_cost = 0.0;
    double maximum_curvature = 0.0;

    const OutputValidationStatus validation = validateOutput(
        map,
        hold_local,
        maximum_cost,
        maximum_curvature);

    if (validation != OutputValidationStatus::VALID)
        return false;

    output.valid = true;
    output.optimized_local_path = hold_local;
    output.seed_path = hold_local;

    buildFullPath(
        plan,
        problem,
        hold_offsets,
        output.optimized_full_path);

    output.report.fresh_result = false;
    output.report.accepted = true;
    output.report.used_cache = true;
    output.report.used_escape_seed = false;
    output.report.reused_active_path = false;
    output.report.curvature_warning =
        config_.curvature_warn_threshold > 0.0
        && maximum_curvature > config_.curvature_warn_threshold;
    output.report.status = "HOLD_C5";
    output.report.detail =
        "当前fresh优化不可用，复用上一轮offset重建当前窗口：" + reason;
    output.report.maximum_curvature =
        maximum_curvature;

    output.report.maximum_lateral_offset = 0.0;
    for (std::size_t i = 0;
         i < hold_offsets.size();
         ++i)
    {
        output.report.maximum_lateral_offset =
            std::max(
                output.report.maximum_lateral_offset,
                std::abs(hold_offsets[i]));
    }

    updateWarmStart(
        problem,
        hold_offsets);

    avoidance_active_ = true;
    cacheActiveOutput(output);
    publishOutput(output);
    logStateTransition(
        output.report.status,
        output.report.detail);
    return true;
}


bool ClearancePathOptimizer::buildCachedActiveOutput(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const MapSnapshot& map,
    Output& output,
    const std::string& reason)
{
    if (!config_.reuse_last_active
        || last_active_local_path_.empty()
        || last_active_full_path_.empty()
        || last_active_stamp_.isZero())
    {
        return false;
    }

    const double age =
        (ros::WallTime::now() - last_active_stamp_).toSec();
    if (!std::isfinite(age)
        || age < 0.0
        || age > config_.active_reuse_max_age)
    {
        return false;
    }

    // 保持MyPlanner现有target_index语义：只在点数一致且终点仍属于同一目标时复用。
    if (last_active_full_path_.size() != plan.size()
        || plan.empty())
    {
        return false;
    }

    const geometry_msgs::PoseStamped& cached_goal =
        last_active_full_path_.back();
    const geometry_msgs::PoseStamped& current_goal =
        plan.back();

    const double goal_distance = std::hypot(
        cached_goal.pose.position.x - current_goal.pose.position.x,
        cached_goal.pose.position.y - current_goal.pose.position.y);

    if (goal_distance > config_.active_reuse_goal_tolerance)
        return false;

    double maximum_cost = 0.0;
    double maximum_curvature = 0.0;
    const OutputValidationStatus validation = validateOutput(
        map,
        last_active_local_path_,
        maximum_cost,
        maximum_curvature);

    if (validation != OutputValidationStatus::VALID)
        return false;

    output.valid = true;
    output.seed_path = last_active_local_path_;
    output.optimized_local_path = last_active_local_path_;
    output.optimized_full_path = last_active_full_path_;

    output.report.fresh_result = false;
    output.report.accepted = true;
    output.report.used_cache = true;
    output.report.used_escape_seed = false;
    output.report.reused_active_path = true;
    output.report.curvature_warning =
        config_.curvature_warn_threshold > 0.0
        && maximum_curvature > config_.curvature_warn_threshold;
    output.report.maximum_curvature = maximum_curvature;
    output.report.status = "REUSE_ACTIVE_C5";
    output.report.detail =
        "fresh/HOLD暂不可用，短时继续使用最后一条已验证active path："
        + reason;

    // 注意：这里不能刷新last_active_stamp_，否则旧轨迹会被无限续命。
    avoidance_active_ = true;
    publishOutput(output);
    logStateTransition(output.report.status, output.report.detail);
    return true;
}

void ClearancePathOptimizer::cacheActiveOutput(
    const Output& output)
{
    if (!output.valid
        || output.optimized_local_path.empty()
        || output.optimized_full_path.empty())
    {
        return;
    }

    last_active_local_path_ = output.optimized_local_path;
    last_active_full_path_ = output.optimized_full_path;
    last_active_stamp_ = ros::WallTime::now();
}

void ClearancePathOptimizer::updateWarmStart(
    const LocalProblem& problem,
    const std::vector<double>& offsets)
{
    previous_anchors_ = problem.anchors;
    previous_offsets_ = offsets;
}


void ClearancePathOptimizer::publishPath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    const ros::Publisher& publisher,
    double z_offset) const
{
    if (!config_.publish_debug_paths
        || path.empty())
    {
        return;
    }

    nav_msgs::Path message;
    message.header.frame_id = costmap_frame_;
    message.header.stamp = ros::Time::now();
    message.poses = path;

    for (std::size_t i = 0;
         i < message.poses.size();
         ++i)
    {
        message.poses[i].header =
            message.header;
        message.poses[i].pose.position.z +=
            z_offset;
    }

    publisher.publish(message);
}


void ClearancePathOptimizer::publishOutput(
    const Output& output)
{
    if (!config_.publish_debug_paths)
        return;

    publishPath(
        output.reference_path,
        reference_path_pub_,
        0.02);

    publishPath(
        output.seed_path,
        seed_path_pub_,
        0.04);

    // optimized话题只发布“当前实际有效active path”。
    // 失败分支不发布空Path，也不拿被拒绝的candidate覆盖最后有效结果。
    if (output.valid)
    {
        publishPath(
            output.optimized_local_path,
            optimized_path_pub_,
            0.06);
    }
}


void ClearancePathOptimizer::logStateTransition(
    const std::string& status,
    const std::string& detail)
{
    if (status == last_runtime_state_)
        return;

    ROS_WARN("C5.4状态：%s -> %s；%s",
             last_runtime_state_.empty()
                 ? "INIT"
                 : last_runtime_state_.c_str(),
             status.c_str(),
             detail.c_str());

    last_runtime_state_ = status;
}


bool ClearancePathOptimizer::optimize(
    const std::vector<geometry_msgs::PoseStamped>& plan_costmap,
    const geometry_msgs::PoseStamped& robot_pose_costmap,
    Output& output)
{
    output = Output();
    const ros::WallTime begin =
        ros::WallTime::now();

    if (!enabled() || plan_costmap.size() < 4)
        return false;

    MapSnapshot map;
    if (!buildMapSnapshot(map))
    {
        output.report.status =
            "MAP_SNAPSHOT_FAILED";
        output.report.detail =
            "无法取得当前costmap快照。";
        output.report.compute_ms =
            elapsedMilliseconds(begin);
        logStateTransition(
            output.report.status,
            output.report.detail);
        return false;
    }

    LocalProblem problem;
    if (!buildLocalProblem(
            plan_costmap,
            robot_pose_costmap,
            map,
            problem))
    {
        if (buildCachedActiveOutput(
                plan_costmap,
                map,
                output,
                "当前局部可用前视长度暂时不足"))
        {
            output.report.compute_ms = elapsedMilliseconds(begin);
            return true;
        }

        output.report.status = "LOCAL_WINDOW_FAILED";
        output.report.detail =
            "当前路径在局部地图中的可用前视长度不足，且无可复用active path。";
        output.report.compute_ms = elapsedMilliseconds(begin);
        logStateTransition(
            output.report.status,
            output.report.detail);
        return false;
    }

    output.report.reference_points =
        static_cast<int>(
            problem.anchors.size());
    output.report.candidate_count =
        static_cast<int>(
            problem.anchors.size());

    std::vector<double> raw_offsets(
        problem.anchors.size(), 0.0);

    offsetsToSampledPath(
        plan_costmap,
        problem,
        raw_offsets,
        output.reference_path);

    double raw_maximum_cost = 0.0;
    bool raw_hard = false;

    if (!evaluateRawWindow(
            plan_costmap,
            problem,
            map,
            raw_maximum_cost,
            raw_hard))
    {
        if (buildCachedActiveOutput(
                plan_costmap,
                map,
                output,
                "当前raw局部窗口评估失败"))
        {
            output.report.compute_ms = elapsedMilliseconds(begin);
            return true;
        }

        output.report.status = "RAW_EVALUATION_FAILED";
        output.report.detail =
            "无法评估当前raw局部窗口，且无可复用active path。";
        output.report.compute_ms = elapsedMilliseconds(begin);
        publishOutput(output);
        logStateTransition(
            output.report.status,
            output.report.detail);
        return false;
    }

    // C5.4保留激活/释放滞回，避免cost在阈值附近时
    // SAFE_RAW与C5路径每周期来回切换。
    const bool should_release =
        avoidance_active_
        && !raw_hard
        && raw_maximum_cost
               < static_cast<double>(
                   config_.deactivation_cost);

    const bool should_stay_safe =
        !avoidance_active_
        && !raw_hard
        && raw_maximum_cost
               < static_cast<double>(
                   config_.activation_cost);

    if (should_release || should_stay_safe)
    {
        avoidance_active_ = false;
        previous_anchors_.clear();
        previous_offsets_.clear();

        output.valid = true;
        output.seed_path =
            output.reference_path;
        output.optimized_local_path =
            output.reference_path;
        output.optimized_full_path =
            plan_costmap;

        output.report.fresh_result = true;
        output.report.accepted = true;
        output.report.used_cache = false;
        output.report.used_escape_seed = false;
        output.report.reused_active_path = false;
        output.report.curvature_warning = false;
        output.report.maximum_curvature = 0.0;
        output.report.maximum_lateral_offset = 0.0;
        output.report.compute_ms =
            elapsedMilliseconds(begin);
        output.report.status = "SAFE_RAW";
        output.report.detail =
            should_release
                ? "障碍代价已降到释放阈值以下，平滑回到raw path。"
                : "当前局部路径安全，直接使用raw path。";

        cacheActiveOutput(output);
        publishOutput(output);
        logStateTransition(
            output.report.status,
            output.report.detail);
        return true;
    }

    avoidance_active_ = true;

    std::vector<double> offsets;
    std::vector<double> seed_offsets;

    initializeOffsets(
        plan_costmap,
        problem,
        map,
        offsets,
        seed_offsets);

    bool seed_hard = false;
    objective(
        plan_costmap,
        problem,
        map,
        offsets,
        NULL,
        &seed_hard);

    bool used_escape_seed = false;

    // raw或warm seed进入LETHAL/UNKNOWN时，不再像C5.0那样立即拒绝；
    // 沿raw path法向逐层搜索最小安全escape seed。
    if (raw_hard || seed_hard)
    {
        if (findEscapeSeed(
                plan_costmap,
                problem,
                map,
                offsets))
        {
            used_escape_seed = true;
            seed_offsets = offsets;
        }
        else
        {
            output.seed_path =
                output.reference_path;

            if (buildHoldOutput(
                    plan_costmap,
                    problem,
                    map,
                    output,
                    "raw/seed进入hard且未找到escape seed"))
            {
                output.report.compute_ms =
                    elapsedMilliseconds(begin);
                return true;
            }

            if (buildCachedActiveOutput(
                    plan_costmap,
                    map,
                    output,
                    "raw/seed进入hard且escape与HOLD均不可用"))
            {
                output.report.compute_ms = elapsedMilliseconds(begin);
                return true;
            }

            output.report.status = "HARD_FAIL";
            output.report.detail =
                "raw path进入hard，escape、HOLD和短时active复用均不可用。";
            output.report.compute_ms = elapsedMilliseconds(begin);

            publishOutput(output);
            logStateTransition(
                output.report.status,
                output.report.detail);
            return false;
        }
    }

    offsetsToSampledPath(
        plan_costmap,
        problem,
        seed_offsets,
        output.seed_path);

    double final_objective = 0.0;
    double final_maximum_cost = raw_maximum_cost;

    const bool had_previous =
        !previous_offsets_.empty();

    if (!optimizeOffsets(
            plan_costmap,
            problem,
            map,
            offsets,
            final_objective,
            final_maximum_cost))
    {
        if (buildHoldOutput(
                plan_costmap,
                problem,
                map,
                output,
                "fresh连续优化未生成数值有效可行结果"))
        {
            output.report.compute_ms =
                elapsedMilliseconds(begin);
            return true;
        }

        if (buildCachedActiveOutput(
                plan_costmap,
                map,
                output,
                "fresh数值优化失败且HOLD不可用"))
        {
            output.report.compute_ms = elapsedMilliseconds(begin);
            return true;
        }

        output.report.status = "OPTIMIZATION_FAILED";
        output.report.detail =
            "fresh数值优化、HOLD和短时active复用均不可用。";
        output.report.compute_ms = elapsedMilliseconds(begin);

        publishOutput(output);
        logStateTransition(
            output.report.status,
            output.report.detail);
        return false;
    }

    offsetsToSampledPath(
        plan_costmap,
        problem,
        offsets,
        output.optimized_local_path);

    double validated_maximum_cost = 0.0;
    double maximum_curvature = 0.0;

    const OutputValidationStatus validation = validateOutput(
        map,
        output.optimized_local_path,
        validated_maximum_cost,
        maximum_curvature);

    if (validation != OutputValidationStatus::VALID)
    {
        const std::string failure_reason =
            validation == OutputValidationStatus::HARD_COLLISION
                ? "fresh B-Spline安全验证命中hard/unknown"
                : (validation == OutputValidationStatus::CURVATURE_EMERGENCY
                    ? "fresh B-Spline超过显式曲率emergency limit"
                    : "fresh B-Spline输出无效");

        if (buildHoldOutput(
                plan_costmap,
                problem,
                map,
                output,
                failure_reason))
        {
            output.report.compute_ms = elapsedMilliseconds(begin);
            return true;
        }

        if (buildCachedActiveOutput(
                plan_costmap,
                map,
                output,
                failure_reason))
        {
            output.report.compute_ms = elapsedMilliseconds(begin);
            return true;
        }

        output.report.status =
            validation == OutputValidationStatus::HARD_COLLISION
                ? "SAFETY_VALIDATION_FAILED"
                : (validation == OutputValidationStatus::CURVATURE_EMERGENCY
                    ? "CURVATURE_EMERGENCY_FAILED"
                    : "OUTPUT_INVALID");
        output.report.detail =
            "fresh、HOLD和短时active复用均不可用：" + failure_reason;
        output.report.compute_ms = elapsedMilliseconds(begin);

        publishOutput(output);
        logStateTransition(
            output.report.status,
            output.report.detail);
        return false;
    }

    buildFullPath(
        plan_costmap,
        problem,
        offsets,
        output.optimized_full_path);

    output.valid = true;
    output.report.fresh_result = true;
    output.report.accepted = true;
    output.report.used_cache = had_previous;
    output.report.used_escape_seed =
        used_escape_seed;
    output.report.reused_active_path = false;
    output.report.curvature_warning =
        config_.curvature_warn_threshold > 0.0
        && maximum_curvature > config_.curvature_warn_threshold;
    output.report.minimum_clearance = 0.0;
    output.report.maximum_curvature =
        maximum_curvature;

    output.report.maximum_lateral_offset = 0.0;
    for (std::size_t i = 0;
         i < offsets.size();
         ++i)
    {
        output.report.maximum_lateral_offset =
            std::max(
                output.report.maximum_lateral_offset,
                std::abs(offsets[i]));
    }

    output.report.compute_ms =
        elapsedMilliseconds(begin);

    output.report.status =
        used_escape_seed
            ? "ESCAPE_C5"
            : "FRESH_C5";

    output.report.detail =
        used_escape_seed
            ? "从hard raw path构造escape seed后完成C5连续优化。"
            : "warm-start cubic B-spline active path已更新。";

    updateWarmStart(
        problem,
        offsets);

    cacheActiveOutput(output);
    publishOutput(output);
    logStateTransition(
        output.report.status,
        output.report.detail);

    if (output.report.curvature_warning)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "C5.4曲率质量提示：max=%.2f > warn=%.2f；"
            "全向底盘默认不以此拒绝轨迹，继续由平滑代价与MPC速度约束控制可跟踪性。",
            output.report.maximum_curvature,
            config_.curvature_warn_threshold);
    }

    ROS_INFO_THROTTLE(
        0.5,
        "C5.4路径[%s]：控制点=%d，raw最大cost=%.1f，"
        "active最大cost=%.1f，最大横移=%.3fm，"
        "最大曲率=%.2f，耗时=%.2fms。",
        output.report.status.c_str(),
        output.report.candidate_count,
        raw_maximum_cost,
        validated_maximum_cost,
        output.report.maximum_lateral_offset,
        output.report.maximum_curvature,
        output.report.compute_ms);

    return true;
}

}  // namespace my_planner
