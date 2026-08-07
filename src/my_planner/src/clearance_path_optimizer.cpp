#include "clearance_path_optimizer.h"

#include <boost/thread/locks.hpp>
#include <costmap_2d/cost_values.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace my_planner
{

namespace
{

const double kInfinity = std::numeric_limits<double>::infinity();

double square(double value)
{
    return value * value;
}

}  // namespace

ClearancePathOptimizer::Report::Report()
    : fresh_result(false),
      accepted(false),
      used_cache(false),
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
      shadow_mode(true),
      reject_unknown(true),
      publish_debug_paths(true),
      prefer_left_on_first_solution(true),
      update_rate(5.0),
      forward_window(1.40),
      backward_window(0.10),
      longitudinal_step(0.04),
      lateral_step(0.02),
      max_lateral_offset(0.35),
      max_lateral_change_per_step(0.04),
      commitment_distance(0.12),
      end_blend_distance(0.20),
      preferred_clearance(0.10),
      distance_field_max_distance(2.00),
      robot_half_length(0.17),
      robot_half_width(0.13),
      extra_clearance_margin(0.00),
      weight_clearance(120.0),
      weight_inflation(2.0),
      weight_reference(6.0),
      weight_history(10.0),
      weight_lateral_slope(80.0),
      weight_lateral_curvature(250.0),
      elastic_iterations(15),
      elastic_step_size(0.015),
      elastic_max_step(0.006),
      elastic_weight_seed(12.0),
      elastic_weight_reference(2.0),
      elastic_weight_history(6.0),
      elastic_weight_smoothness(100.0),
      elastic_weight_clearance(60.0),
      centerline_check_step(0.01),
      max_accepted_curvature(12.0),
      max_history_jump(0.18)
{
}

ClearancePathOptimizer::MapSnapshot::MapSnapshot()
    : size_x(0), size_y(0), resolution(0.0), origin_x(0.0), origin_y(0.0)
{
}

ClearancePathOptimizer::ReferencePoint::ReferencePoint()
    : x(0.0), y(0.0), yaw(0.0), normal_x(0.0), normal_y(1.0),
      local_s(0.0), plan_s(0.0)
{
}

ClearancePathOptimizer::Candidate::Candidate()
    : valid(false), d(0.0), x(0.0), y(0.0), clearance(0.0), cost(0),
      node_cost(kInfinity)
{
}

ClearancePathOptimizer::ClearancePathOptimizer()
    : initialized_(false), costmap_(NULL)
{
}

double ClearancePathOptimizer::clamp(
    double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}

double ClearancePathOptimizer::normalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
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
    nh.param("shadow_mode", config_.shadow_mode, config_.shadow_mode);
    nh.param("reject_unknown", config_.reject_unknown, config_.reject_unknown);
    nh.param("publish_debug_paths", config_.publish_debug_paths,
             config_.publish_debug_paths);
    nh.param("prefer_left_on_first_solution",
             config_.prefer_left_on_first_solution,
             config_.prefer_left_on_first_solution);
    nh.param("update_rate", config_.update_rate, config_.update_rate);
    nh.param("forward_window", config_.forward_window,
             config_.forward_window);
    nh.param("backward_window", config_.backward_window,
             config_.backward_window);
    nh.param("longitudinal_step", config_.longitudinal_step,
             config_.longitudinal_step);
    nh.param("lateral_step", config_.lateral_step, config_.lateral_step);
    nh.param("max_lateral_offset", config_.max_lateral_offset,
             config_.max_lateral_offset);
    nh.param("max_lateral_change_per_step",
             config_.max_lateral_change_per_step,
             config_.max_lateral_change_per_step);
    nh.param("commitment_distance", config_.commitment_distance,
             config_.commitment_distance);
    nh.param("end_blend_distance", config_.end_blend_distance,
             config_.end_blend_distance);
    nh.param("preferred_clearance", config_.preferred_clearance,
             config_.preferred_clearance);
    nh.param("distance_field_max_distance",
             config_.distance_field_max_distance,
             config_.distance_field_max_distance);
    nh.param("robot_half_length", config_.robot_half_length,
             config_.robot_half_length);
    nh.param("robot_half_width", config_.robot_half_width,
             config_.robot_half_width);
    nh.param("extra_clearance_margin", config_.extra_clearance_margin,
             config_.extra_clearance_margin);
    nh.param("weight_clearance", config_.weight_clearance,
             config_.weight_clearance);
    nh.param("weight_inflation", config_.weight_inflation,
             config_.weight_inflation);
    nh.param("weight_reference", config_.weight_reference,
             config_.weight_reference);
    nh.param("weight_history", config_.weight_history,
             config_.weight_history);
    nh.param("weight_lateral_slope", config_.weight_lateral_slope,
             config_.weight_lateral_slope);
    nh.param("weight_lateral_curvature",
             config_.weight_lateral_curvature,
             config_.weight_lateral_curvature);
    nh.param("elastic_iterations", config_.elastic_iterations,
             config_.elastic_iterations);
    nh.param("elastic_step_size", config_.elastic_step_size,
             config_.elastic_step_size);
    nh.param("elastic_max_step", config_.elastic_max_step,
             config_.elastic_max_step);
    nh.param("elastic_weight_seed", config_.elastic_weight_seed,
             config_.elastic_weight_seed);
    nh.param("elastic_weight_reference", config_.elastic_weight_reference,
             config_.elastic_weight_reference);
    nh.param("elastic_weight_history", config_.elastic_weight_history,
             config_.elastic_weight_history);
    nh.param("elastic_weight_smoothness", config_.elastic_weight_smoothness,
             config_.elastic_weight_smoothness);
    nh.param("elastic_weight_clearance", config_.elastic_weight_clearance,
             config_.elastic_weight_clearance);
    nh.param("centerline_check_step", config_.centerline_check_step,
             config_.centerline_check_step);
    nh.param("max_accepted_curvature", config_.max_accepted_curvature,
             config_.max_accepted_curvature);
    nh.param("max_history_jump", config_.max_history_jump,
             config_.max_history_jump);

    config_.update_rate = clamp(config_.update_rate, 0.5, 20.0);
    config_.forward_window = std::max(0.30, config_.forward_window);
    config_.backward_window = std::max(0.0, config_.backward_window);
    config_.longitudinal_step = clamp(
        config_.longitudinal_step, 0.02, 0.10);
    config_.lateral_step = clamp(config_.lateral_step, 0.01, 0.05);
    config_.max_lateral_offset = std::max(
        config_.lateral_step, config_.max_lateral_offset);
    config_.max_lateral_change_per_step = std::max(
        config_.lateral_step, config_.max_lateral_change_per_step);
    config_.commitment_distance = std::max(0.0, config_.commitment_distance);
    config_.end_blend_distance = std::max(
        config_.longitudinal_step, config_.end_blend_distance);
    config_.preferred_clearance = std::max(0.0, config_.preferred_clearance);
    config_.distance_field_max_distance = std::max(
        config_.preferred_clearance + config_.robot_half_length + 0.10,
        config_.distance_field_max_distance);
    config_.robot_half_length = std::max(0.01, config_.robot_half_length);
    config_.robot_half_width = std::max(0.01, config_.robot_half_width);
    config_.extra_clearance_margin = std::max(
        0.0, config_.extra_clearance_margin);
    config_.elastic_iterations = std::max(0, config_.elastic_iterations);
    config_.elastic_step_size = std::max(0.0, config_.elastic_step_size);
    config_.elastic_max_step = std::max(0.0, config_.elastic_max_step);
    config_.centerline_check_step = clamp(
        config_.centerline_check_step, 0.005, config_.longitudinal_step);
    config_.max_accepted_curvature = std::max(
        0.5, config_.max_accepted_curvature);
    config_.max_history_jump = std::max(
        config_.lateral_step, config_.max_history_jump);

    reference_path_pub_ = nh.advertise<nav_msgs::Path>(
        "reference_path", 1, true);
    lattice_seed_path_pub_ = nh.advertise<nav_msgs::Path>(
        "lattice_seed_path", 1, true);
    optimized_path_pub_ = nh.advertise<nav_msgs::Path>(
        "optimized_path", 1, true);

    initialized_ = true;
    reset();

    ROS_WARN("C4.1独立净空路径优化器启动：enabled=%s，shadow=%s，"
             "窗口=后%.2f/前%.2fm，采样=%.2f/%.2fm，横移=±%.2fm，"
             "车体半长/半宽=%.3f/%.3fm，期望净空=%.3fm，频率=%.1fHz。",
             config_.enabled ? "是" : "否",
             config_.shadow_mode ? "是（仅观察，不交给MPC）" : "否（交给MPC）",
             config_.backward_window, config_.forward_window,
             config_.longitudinal_step, config_.lateral_step,
             config_.max_lateral_offset,
             config_.robot_half_length, config_.robot_half_width,
             config_.preferred_clearance, config_.update_rate);
}

void ClearancePathOptimizer::reset()
{
    last_update_time_ = ros::Time(0);
    cached_output_ = Output();
    last_accepted_local_path_.clear();

    // 三条调试路径为latched话题。清空它们，避免进入旁路模式后RViz仍
    // 显示上一个区域遗留的优化路径，让人误以为MPC仍在跟踪旧结果。
    if (initialized_ && config_.publish_debug_paths)
    {
        nav_msgs::Path empty_path;
        empty_path.header.stamp = ros::Time::now();
        empty_path.header.frame_id = costmap_frame_;
        reference_path_pub_.publish(empty_path);
        lattice_seed_path_pub_.publish(empty_path);
        optimized_path_pub_.publish(empty_path);
    }
}

bool ClearancePathOptimizer::enabled() const
{
    return initialized_ && config_.enabled;
}

bool ClearancePathOptimizer::shadowMode() const
{
    return config_.shadow_mode;
}

void ClearancePathOptimizer::setShadowMode(bool shadow_mode)
{
    if (config_.shadow_mode == shadow_mode)
        return;

    config_.shadow_mode = shadow_mode;
    reset();
}

bool ClearancePathOptimizer::buildMapSnapshot(MapSnapshot& map) const
{
    if (costmap_ == NULL)
        return false;

    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
            lock(*(costmap_->getMutex()));

        map.size_x = costmap_->getSizeInCellsX();
        map.size_y = costmap_->getSizeInCellsY();
        map.resolution = costmap_->getResolution();
        map.origin_x = costmap_->getOriginX();
        map.origin_y = costmap_->getOriginY();

        const std::size_t cell_count =
            static_cast<std::size_t>(map.size_x) * map.size_y;
        map.costs.assign(costmap_->getCharMap(),
                         costmap_->getCharMap() + cell_count);
    }

    if (map.size_x == 0 || map.size_y == 0 || map.resolution <= 0.0)
        return false;

    buildEuclideanDistanceField(map);
    return true;
}

void ClearancePathOptimizer::squaredDistanceTransform1D(
    const std::vector<double>& input,
    std::vector<double>& output)
{
    const int count = static_cast<int>(input.size());
    output.assign(input.size(), kInfinity);
    if (count == 0)
        return;

    std::vector<int> sites;
    sites.reserve(input.size());
    for (int i = 0; i < count; ++i)
    {
        if (std::isfinite(input[static_cast<std::size_t>(i)]))
            sites.push_back(i);
    }

    if (sites.empty())
        return;

    std::vector<int> envelope(sites.size(), 0);
    std::vector<double> boundaries(sites.size() + 1, kInfinity);
    int envelope_size = 0;
    envelope[0] = sites[0];
    boundaries[0] = -kInfinity;
    boundaries[1] = kInfinity;

    for (std::size_t index = 1; index < sites.size(); ++index)
    {
        const int q = sites[index];
        double intersection = 0.0;

        while (true)
        {
            const int p = envelope[static_cast<std::size_t>(envelope_size)];
            intersection =
                ((input[static_cast<std::size_t>(q)] + square(q))
                 - (input[static_cast<std::size_t>(p)] + square(p)))
                / (2.0 * static_cast<double>(q - p));

            if (intersection > boundaries[static_cast<std::size_t>(envelope_size)]
                || envelope_size == 0)
            {
                break;
            }
            --envelope_size;
        }

        ++envelope_size;
        envelope[static_cast<std::size_t>(envelope_size)] = q;
        boundaries[static_cast<std::size_t>(envelope_size)] = intersection;
        boundaries[static_cast<std::size_t>(envelope_size + 1)] = kInfinity;
    }

    int active = 0;
    for (int q = 0; q < count; ++q)
    {
        while (active < envelope_size
               && boundaries[static_cast<std::size_t>(active + 1)] < q)
        {
            ++active;
        }

        const int p = envelope[static_cast<std::size_t>(active)];
        output[static_cast<std::size_t>(q)] =
            square(q - p) + input[static_cast<std::size_t>(p)];
    }
}

void ClearancePathOptimizer::buildEuclideanDistanceField(
    MapSnapshot& map) const
{
    const std::size_t cell_count =
        static_cast<std::size_t>(map.size_x) * map.size_y;
    std::vector<double> row_pass(cell_count, kInfinity);
    std::vector<double> input;
    std::vector<double> output;

    input.resize(map.size_x);
    for (unsigned int y = 0; y < map.size_y; ++y)
    {
        for (unsigned int x = 0; x < map.size_x; ++x)
        {
            const std::size_t index =
                static_cast<std::size_t>(y) * map.size_x + x;
            input[x] = map.costs[index] == costmap_2d::LETHAL_OBSTACLE
                ? 0.0 : kInfinity;
        }

        squaredDistanceTransform1D(input, output);
        for (unsigned int x = 0; x < map.size_x; ++x)
        {
            row_pass[static_cast<std::size_t>(y) * map.size_x + x] =
                output[x];
        }
    }

    map.obstacle_distance.assign(
        cell_count, config_.distance_field_max_distance);
    input.resize(map.size_y);

    for (unsigned int x = 0; x < map.size_x; ++x)
    {
        for (unsigned int y = 0; y < map.size_y; ++y)
        {
            input[y] = row_pass[
                static_cast<std::size_t>(y) * map.size_x + x];
        }

        squaredDistanceTransform1D(input, output);
        for (unsigned int y = 0; y < map.size_y; ++y)
        {
            const double cells = output[y];
            const double distance = std::isfinite(cells)
                ? std::sqrt(std::max(0.0, cells)) * map.resolution
                : config_.distance_field_max_distance;
            map.obstacle_distance[
                static_cast<std::size_t>(y) * map.size_x + x] =
                std::min(distance, config_.distance_field_max_distance);
        }
    }
}

bool ClearancePathOptimizer::interpolatePlan(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const std::vector<double>& plan_s,
    double query_s,
    double& x,
    double& y) const
{
    if (plan.empty() || plan.size() != plan_s.size())
        return false;

    if (query_s <= plan_s.front())
    {
        x = plan.front().pose.position.x;
        y = plan.front().pose.position.y;
        return true;
    }
    if (query_s >= plan_s.back())
    {
        x = plan.back().pose.position.x;
        y = plan.back().pose.position.y;
        return true;
    }

    const std::vector<double>::const_iterator upper =
        std::upper_bound(plan_s.begin(), plan_s.end(), query_s);
    std::size_t second = static_cast<std::size_t>(
        std::distance(plan_s.begin(), upper));
    second = std::max<std::size_t>(1, std::min(second, plan.size() - 1));
    const std::size_t first = second - 1;

    const double length = plan_s[second] - plan_s[first];
    const double ratio = length > 1.0e-9
        ? clamp((query_s - plan_s[first]) / length, 0.0, 1.0)
        : 0.0;
    x = plan[first].pose.position.x
        + ratio * (plan[second].pose.position.x
                   - plan[first].pose.position.x);
    y = plan[first].pose.position.y
        + ratio * (plan[second].pose.position.y
                   - plan[first].pose.position.y);
    return true;
}

bool ClearancePathOptimizer::buildReferencePath(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const geometry_msgs::PoseStamped& robot_pose,
    std::vector<double>& plan_s,
    double& window_start_s,
    double& window_end_s,
    std::vector<ReferencePoint>& reference) const
{
    reference.clear();
    if (plan.size() < 3)
        return false;

    plan_s.assign(plan.size(), 0.0);
    for (std::size_t i = 1; i < plan.size(); ++i)
    {
        plan_s[i] = plan_s[i - 1] + std::hypot(
            plan[i].pose.position.x - plan[i - 1].pose.position.x,
            plan[i].pose.position.y - plan[i - 1].pose.position.y);
    }
    if (plan_s.back() < 2.0 * config_.longitudinal_step)
        return false;

    const double robot_x = robot_pose.pose.position.x;
    const double robot_y = robot_pose.pose.position.y;
    double closest_s = 0.0;
    double closest_squared = kInfinity;

    for (std::size_t i = 0; i + 1 < plan.size(); ++i)
    {
        const double x0 = plan[i].pose.position.x;
        const double y0 = plan[i].pose.position.y;
        const double dx = plan[i + 1].pose.position.x - x0;
        const double dy = plan[i + 1].pose.position.y - y0;
        const double segment_squared = dx * dx + dy * dy;
        const double ratio = segment_squared > 1.0e-12
            ? clamp(((robot_x - x0) * dx + (robot_y - y0) * dy)
                        / segment_squared,
                    0.0, 1.0)
            : 0.0;
        const double px = x0 + ratio * dx;
        const double py = y0 + ratio * dy;
        const double distance_squared = square(robot_x - px) + square(robot_y - py);
        if (distance_squared < closest_squared)
        {
            closest_squared = distance_squared;
            closest_s = plan_s[i] + ratio * std::sqrt(segment_squared);
        }
    }

    window_start_s = std::max(0.0, closest_s - config_.backward_window);
    window_end_s = std::min(plan_s.back(), closest_s + config_.forward_window);
    if (window_end_s - window_start_s < 2.0 * config_.longitudinal_step)
        return false;

    std::vector<double> queries;
    for (double s = window_start_s;
         s < window_end_s - 1.0e-9;
         s += config_.longitudinal_step)
    {
        queries.push_back(s);
    }
    queries.push_back(window_end_s);

    reference.reserve(queries.size());
    for (std::size_t i = 0; i < queries.size(); ++i)
    {
        ReferencePoint point;
        point.plan_s = queries[i];
        point.local_s = queries[i] - window_start_s;
        if (!interpolatePlan(plan, plan_s, queries[i], point.x, point.y))
            return false;

        const double tangent_distance = 0.5 * config_.longitudinal_step;
        const double before_s = std::max(0.0, queries[i] - tangent_distance);
        const double after_s = std::min(plan_s.back(), queries[i] + tangent_distance);
        double before_x = point.x;
        double before_y = point.y;
        double after_x = point.x;
        double after_y = point.y;
        interpolatePlan(plan, plan_s, before_s, before_x, before_y);
        interpolatePlan(plan, plan_s, after_s, after_x, after_y);
        point.yaw = std::atan2(after_y - before_y, after_x - before_x);
        point.normal_x = -std::sin(point.yaw);
        point.normal_y = std::cos(point.yaw);
        reference.push_back(point);
    }

    return reference.size() >= 3;
}

bool ClearancePathOptimizer::worldToMapContinuous(
    const MapSnapshot& map,
    double wx,
    double wy,
    double& mx,
    double& my) const
{
    mx = (wx - map.origin_x) / map.resolution - 0.5;
    my = (wy - map.origin_y) / map.resolution - 0.5;
    return mx >= 0.0 && my >= 0.0
        && mx <= static_cast<double>(map.size_x - 1)
        && my <= static_cast<double>(map.size_y - 1);
}

bool ClearancePathOptimizer::queryDistanceAndGradient(
    const MapSnapshot& map,
    double wx,
    double wy,
    double& distance,
    double& gradient_x,
    double& gradient_y) const
{
    double gx = 0.0;
    double gy = 0.0;
    if (!worldToMapContinuous(map, wx, wy, gx, gy))
        return false;

    const unsigned int x0 = static_cast<unsigned int>(std::floor(gx));
    const unsigned int y0 = static_cast<unsigned int>(std::floor(gy));
    const unsigned int x1 = std::min(x0 + 1, map.size_x - 1);
    const unsigned int y1 = std::min(y0 + 1, map.size_y - 1);
    const double tx = gx - x0;
    const double ty = gy - y0;

    const double d00 = map.obstacle_distance[
        static_cast<std::size_t>(y0) * map.size_x + x0];
    const double d10 = map.obstacle_distance[
        static_cast<std::size_t>(y0) * map.size_x + x1];
    const double d01 = map.obstacle_distance[
        static_cast<std::size_t>(y1) * map.size_x + x0];
    const double d11 = map.obstacle_distance[
        static_cast<std::size_t>(y1) * map.size_x + x1];

    distance = (1.0 - ty) * ((1.0 - tx) * d00 + tx * d10)
        + ty * ((1.0 - tx) * d01 + tx * d11);
    gradient_x = ((1.0 - ty) * (d10 - d00) + ty * (d11 - d01))
        / map.resolution;
    gradient_y = ((1.0 - tx) * (d01 - d00) + tx * (d11 - d10))
        / map.resolution;
    return true;
}

bool ClearancePathOptimizer::evaluateCandidate(
    const MapSnapshot& map,
    const ReferencePoint& reference,
    double lateral_offset,
    Candidate& candidate) const
{
    candidate = Candidate();
    candidate.d = lateral_offset;
    candidate.x = reference.x + lateral_offset * reference.normal_x;
    candidate.y = reference.y + lateral_offset * reference.normal_y;

    const int mx = static_cast<int>(
        std::floor((candidate.x - map.origin_x) / map.resolution));
    const int my = static_cast<int>(
        std::floor((candidate.y - map.origin_y) / map.resolution));
    if (mx < 0 || my < 0
        || mx >= static_cast<int>(map.size_x)
        || my >= static_cast<int>(map.size_y))
    {
        return false;
    }

    candidate.cost = map.costs[
        static_cast<std::size_t>(my) * map.size_x
        + static_cast<unsigned int>(mx)];

    if (candidate.cost == costmap_2d::LETHAL_OBSTACLE
        || (config_.reject_unknown
            && candidate.cost == costmap_2d::NO_INFORMATION))
    {
        return false;
    }

    double distance = 0.0;
    double gradient_x = 0.0;
    double gradient_y = 0.0;
    if (!queryDistanceAndGradient(
            map, candidate.x, candidate.y,
            distance, gradient_x, gradient_y))
    {
        return false;
    }

    const double gradient_norm = std::hypot(gradient_x, gradient_y);
    double support_radius = std::max(
        config_.robot_half_length, config_.robot_half_width);
    if (gradient_norm > 1.0e-6)
    {
        const double nx = gradient_x / gradient_norm;
        const double ny = gradient_y / gradient_norm;
        const double body_x_x = std::cos(reference.yaw);
        const double body_x_y = std::sin(reference.yaw);
        const double body_y_x = -body_x_y;
        const double body_y_y = body_x_x;
        support_radius =
            config_.robot_half_length
                * std::abs(nx * body_x_x + ny * body_x_y)
            + config_.robot_half_width
                * std::abs(nx * body_y_x + ny * body_y_y);
    }

    candidate.clearance = distance - support_radius
        - config_.extra_clearance_margin;
    candidate.valid = true;
    return true;
}

double ClearancePathOptimizer::historyOffsetForReference(
    const ReferencePoint& reference,
    bool& available) const
{
    available = false;
    if (last_accepted_local_path_.empty())
        return 0.0;

    double best_squared = square(0.20);
    double best_offset = 0.0;
    for (std::size_t i = 0; i < last_accepted_local_path_.size(); ++i)
    {
        const double dx =
            last_accepted_local_path_[i].pose.position.x - reference.x;
        const double dy =
            last_accepted_local_path_[i].pose.position.y - reference.y;
        const double distance_squared = dx * dx + dy * dy;
        if (distance_squared < best_squared)
        {
            best_squared = distance_squared;
            best_offset = dx * reference.normal_x + dy * reference.normal_y;
            available = true;
        }
    }
    return best_offset;
}

bool ClearancePathOptimizer::centerlineSegmentIsValid(
    const MapSnapshot& map,
    double x0,
    double y0,
    double x1,
    double y1) const
{
    const double length = std::hypot(x1 - x0, y1 - y0);
    const int samples = std::max(
        1, static_cast<int>(std::ceil(length / config_.centerline_check_step)));
    for (int i = 0; i <= samples; ++i)
    {
        const double ratio = static_cast<double>(i) / samples;
        const double x = x0 + ratio * (x1 - x0);
        const double y = y0 + ratio * (y1 - y0);
        const int mx = static_cast<int>(
            std::floor((x - map.origin_x) / map.resolution));
        const int my = static_cast<int>(
            std::floor((y - map.origin_y) / map.resolution));
        if (mx < 0 || my < 0
            || mx >= static_cast<int>(map.size_x)
            || my >= static_cast<int>(map.size_y))
        {
            return false;
        }
        const unsigned char cost = map.costs[
            static_cast<std::size_t>(my) * map.size_x
            + static_cast<unsigned int>(mx)];
        if (cost == costmap_2d::LETHAL_OBSTACLE
            || (config_.reject_unknown
                && cost == costmap_2d::NO_INFORMATION))
        {
            return false;
        }
    }
    return true;
}

bool ClearancePathOptimizer::runLatticeDynamicProgramming(
    const MapSnapshot& map,
    const std::vector<ReferencePoint>& reference,
    std::vector<double>& seed_offsets,
    std::vector<double>& clearance_targets,
    int& candidate_count,
    std::string& failure_reason) const
{
    seed_offsets.clear();
    clearance_targets.clear();
    candidate_count = 0;
    if (reference.size() < 3)
    {
        failure_reason = "参考路径点不足";
        return false;
    }

    const int side_steps = std::max(
        1, static_cast<int>(std::floor(
               config_.max_lateral_offset / config_.lateral_step)));
    std::vector<double> lateral_values;
    for (int i = -side_steps; i <= side_steps; ++i)
        lateral_values.push_back(i * config_.lateral_step);

    const int layer_count = static_cast<int>(reference.size());
    const int lateral_count = static_cast<int>(lateral_values.size());
    const int zero_index = side_steps;
    candidate_count = lateral_count;

    std::vector<std::vector<Candidate> > candidates(
        reference.size(), std::vector<Candidate>(lateral_values.size()));
    clearance_targets.resize(reference.size(), 0.0);

    for (int layer = 0; layer < layer_count; ++layer)
    {
        double maximum_clearance = -kInfinity;
        unsigned int minimum_cost = 255;
        for (int lateral = 0; lateral < lateral_count; ++lateral)
        {
            Candidate& candidate = candidates[static_cast<std::size_t>(layer)]
                                            [static_cast<std::size_t>(lateral)];
            if (!evaluateCandidate(
                    map, reference[static_cast<std::size_t>(layer)],
                    lateral_values[static_cast<std::size_t>(lateral)],
                    candidate))
            {
                continue;
            }
            maximum_clearance = std::max(
                maximum_clearance, candidate.clearance);
            minimum_cost = std::min<unsigned int>(minimum_cost, candidate.cost);
        }

        if (!std::isfinite(maximum_clearance))
        {
            failure_reason = "某一横截面没有地图内中心线候选";
            return false;
        }

        const double target_clearance = std::min(
            config_.preferred_clearance, maximum_clearance);
        clearance_targets[static_cast<std::size_t>(layer)] = target_clearance;

        bool history_available = false;
        const double history_offset = historyOffsetForReference(
            reference[static_cast<std::size_t>(layer)], history_available);
        const double remaining = reference.back().local_s
            - reference[static_cast<std::size_t>(layer)].local_s;
        const double end_blend = clamp(
            1.0 - remaining / config_.end_blend_distance, 0.0, 1.0);

        for (int lateral = 0; lateral < lateral_count; ++lateral)
        {
            Candidate& candidate = candidates[static_cast<std::size_t>(layer)]
                                            [static_cast<std::size_t>(lateral)];
            if (!candidate.valid)
                continue;

            const double clearance_deficit = std::max(
                0.0, target_clearance - candidate.clearance);
            const double inflation_difference =
                (static_cast<double>(candidate.cost) - minimum_cost) / 252.0;
            const double reference_weight = config_.weight_reference
                * (1.0 + 8.0 * end_blend * end_blend);

            candidate.node_cost =
                config_.weight_clearance * square(clearance_deficit)
                + config_.weight_inflation * square(inflation_difference)
                + reference_weight * square(candidate.d);

            if (history_available)
            {
                candidate.node_cost += config_.weight_history
                    * square(candidate.d - history_offset);
            }
            else
            {
                const bool preferred_side =
                    config_.prefer_left_on_first_solution
                        ? candidate.d > 0.0 : candidate.d < 0.0;
                if (!preferred_side && std::abs(candidate.d) > 1.0e-9)
                    candidate.node_cost += 1.0e-6;
            }
        }
    }

    const double committed_until =
        config_.backward_window + config_.commitment_distance;
    const auto nodeAllowed = [&](int layer, int lateral) -> bool
    {
        const Candidate& candidate = candidates[static_cast<std::size_t>(layer)]
                                              [static_cast<std::size_t>(lateral)];
        if (!candidate.valid)
            return false;
        if (reference[static_cast<std::size_t>(layer)].local_s
                <= committed_until + 1.0e-9
            && lateral != zero_index)
        {
            return false;
        }
        if (layer == layer_count - 1 && lateral != zero_index)
            return false;
        return true;
    };

    if (!nodeAllowed(0, zero_index))
    {
        failure_reason = "近端承诺段原路径中心位于致命/未知栅格";
        return false;
    }

    const int state_count = lateral_count * lateral_count;
    std::vector<double> previous_cost(
        static_cast<std::size_t>(state_count), kInfinity);
    std::vector<double> current_cost(
        static_cast<std::size_t>(state_count), kInfinity);
    std::vector<std::vector<int> > parents(
        reference.size(), std::vector<int>(static_cast<std::size_t>(state_count), -1));

    for (int first = 0; first < lateral_count; ++first)
    {
        if (!nodeAllowed(0, first))
            continue;
        for (int second = 0; second < lateral_count; ++second)
        {
            if (!nodeAllowed(1, second))
                continue;
            const Candidate& a = candidates[0][static_cast<std::size_t>(first)];
            const Candidate& b = candidates[1][static_cast<std::size_t>(second)];
            if (std::abs(b.d - a.d)
                    > config_.max_lateral_change_per_step + 1.0e-9
                || !centerlineSegmentIsValid(map, a.x, a.y, b.x, b.y))
            {
                continue;
            }
            previous_cost[static_cast<std::size_t>(first * lateral_count + second)] =
                a.node_cost + b.node_cost
                + config_.weight_lateral_slope * square(b.d - a.d);
        }
    }

    for (int layer = 2; layer < layer_count; ++layer)
    {
        std::fill(current_cost.begin(), current_cost.end(), kInfinity);
        for (int before = 0; before < lateral_count; ++before)
        {
            for (int previous = 0; previous < lateral_count; ++previous)
            {
                const double accumulated = previous_cost[
                    static_cast<std::size_t>(before * lateral_count + previous)];
                if (!std::isfinite(accumulated))
                    continue;

                const Candidate& previous_candidate =
                    candidates[static_cast<std::size_t>(layer - 1)]
                              [static_cast<std::size_t>(previous)];
                for (int current = 0; current < lateral_count; ++current)
                {
                    if (!nodeAllowed(layer, current))
                        continue;
                    const Candidate& current_candidate =
                        candidates[static_cast<std::size_t>(layer)]
                                  [static_cast<std::size_t>(current)];
                    if (std::abs(current_candidate.d - previous_candidate.d)
                            > config_.max_lateral_change_per_step + 1.0e-9
                        || !centerlineSegmentIsValid(
                            map,
                            previous_candidate.x, previous_candidate.y,
                            current_candidate.x, current_candidate.y))
                    {
                        continue;
                    }

                    const double second_difference = current_candidate.d
                        - 2.0 * previous_candidate.d
                        + candidates[static_cast<std::size_t>(layer - 2)]
                                    [static_cast<std::size_t>(before)].d;
                    const double transition_cost =
                        config_.weight_lateral_slope
                            * square(current_candidate.d - previous_candidate.d)
                        + config_.weight_lateral_curvature
                            * square(second_difference);
                    const double total = accumulated
                        + current_candidate.node_cost + transition_cost;
                    const int state = previous * lateral_count + current;
                    if (total < current_cost[static_cast<std::size_t>(state)])
                    {
                        current_cost[static_cast<std::size_t>(state)] = total;
                        parents[static_cast<std::size_t>(layer)]
                               [static_cast<std::size_t>(state)] = before;
                    }
                }
            }
        }
        previous_cost.swap(current_cost);
    }

    int best_state = -1;
    double best_cost = kInfinity;
    for (int state = 0; state < state_count; ++state)
    {
        if (previous_cost[static_cast<std::size_t>(state)] < best_cost)
        {
            best_cost = previous_cost[static_cast<std::size_t>(state)];
            best_state = state;
        }
    }
    if (best_state < 0 || !std::isfinite(best_cost))
    {
        failure_reason = "横向格点之间不存在连续中心线路径";
        return false;
    }

    std::vector<int> indices(reference.size(), zero_index);
    indices[reference.size() - 2] = best_state / lateral_count;
    indices[reference.size() - 1] = best_state % lateral_count;
    for (int layer = layer_count - 1; layer >= 2; --layer)
    {
        const int state =
            indices[static_cast<std::size_t>(layer - 1)] * lateral_count
            + indices[static_cast<std::size_t>(layer)];
        const int parent = parents[static_cast<std::size_t>(layer)]
                                  [static_cast<std::size_t>(state)];
        if (parent < 0)
        {
            failure_reason = "横向格点回溯失败";
            return false;
        }
        indices[static_cast<std::size_t>(layer - 2)] = parent;
    }

    seed_offsets.resize(reference.size(), 0.0);
    for (std::size_t i = 0; i < reference.size(); ++i)
    {
        seed_offsets[i] = lateral_values[
            static_cast<std::size_t>(indices[i])];
    }
    return true;
}

void ClearancePathOptimizer::runElasticRefinement(
    const MapSnapshot& map,
    const std::vector<ReferencePoint>& reference,
    const std::vector<double>& clearance_targets,
    const std::vector<double>& seed_offsets,
    std::vector<double>& optimized_offsets) const
{
    optimized_offsets = seed_offsets;
    if (optimized_offsets.size() < 3 || config_.elastic_iterations <= 0)
        return;

    std::vector<double> history_offsets(reference.size(), 0.0);
    std::vector<bool> history_available(reference.size(), false);
    for (std::size_t i = 0; i < reference.size(); ++i)
    {
        bool available = false;
        history_offsets[i] = historyOffsetForReference(reference[i], available);
        history_available[i] = available;
    }

    const double committed_until =
        config_.backward_window + config_.commitment_distance;

    for (int iteration = 0; iteration < config_.elastic_iterations; ++iteration)
    {
        const std::vector<double> previous = optimized_offsets;
        for (std::size_t i = 1; i + 1 < previous.size(); ++i)
        {
            if (reference[i].local_s <= committed_until + 1.0e-9)
            {
                optimized_offsets[i] = 0.0;
                continue;
            }

            double gradient =
                2.0 * config_.elastic_weight_seed
                    * (previous[i] - seed_offsets[i])
                + 2.0 * config_.elastic_weight_reference * previous[i];

            if (history_available[i])
            {
                gradient += 2.0 * config_.elastic_weight_history
                    * (previous[i] - history_offsets[i]);
            }

            double a_before = 0.0;
            double a_center = previous[i + 1]
                - 2.0 * previous[i] + previous[i - 1];
            double a_after = 0.0;
            if (i >= 2)
            {
                a_before = previous[i]
                    - 2.0 * previous[i - 1] + previous[i - 2];
            }
            if (i + 2 < previous.size())
            {
                a_after = previous[i + 2]
                    - 2.0 * previous[i + 1] + previous[i];
            }
            gradient += 2.0 * config_.elastic_weight_smoothness
                * (a_before - 2.0 * a_center + a_after);

            Candidate candidate;
            if (evaluateCandidate(map, reference[i], previous[i], candidate))
            {
                const double deficit = std::max(
                    0.0, clearance_targets[i] - candidate.clearance);
                if (deficit > 0.0)
                {
                    double distance = 0.0;
                    double gradient_x = 0.0;
                    double gradient_y = 0.0;
                    if (queryDistanceAndGradient(
                            map, candidate.x, candidate.y,
                            distance, gradient_x, gradient_y))
                    {
                        const double dq_dd =
                            gradient_x * reference[i].normal_x
                            + gradient_y * reference[i].normal_y;
                        gradient += -2.0 * config_.elastic_weight_clearance
                            * deficit * dq_dd;
                    }
                }
            }

            const double change = clamp(
                -config_.elastic_step_size * gradient,
                -config_.elastic_max_step,
                config_.elastic_max_step);
            optimized_offsets[i] = clamp(
                previous[i] + change,
                -config_.max_lateral_offset,
                config_.max_lateral_offset);
        }

        optimized_offsets.front() = 0.0;
        optimized_offsets.back() = 0.0;
        for (std::size_t i = 1; i < optimized_offsets.size(); ++i)
        {
            optimized_offsets[i] = clamp(
                optimized_offsets[i],
                optimized_offsets[i - 1] - config_.max_lateral_change_per_step,
                optimized_offsets[i - 1] + config_.max_lateral_change_per_step);
        }
        optimized_offsets.back() = 0.0;
        for (std::size_t i = optimized_offsets.size() - 1; i > 0; --i)
        {
            optimized_offsets[i - 1] = clamp(
                optimized_offsets[i - 1],
                optimized_offsets[i] - config_.max_lateral_change_per_step,
                optimized_offsets[i] + config_.max_lateral_change_per_step);
        }
        for (std::size_t i = 0; i < reference.size(); ++i)
        {
            if (reference[i].local_s <= committed_until + 1.0e-9)
                optimized_offsets[i] = 0.0;
        }
    }

    // 最后执行保净空的低通平滑，专门消除2cm格点量化造成的锯齿。
    // 每个提议点只有在不明显损失当前可达净空时才被接受，因此不会
    // 像普通样条那样在障碍物附近直接切角。
    for (int pass = 0; pass < 20; ++pass)
    {
        const std::vector<double> previous = optimized_offsets;
        for (std::size_t i = 1; i + 1 < previous.size(); ++i)
        {
            if (reference[i].local_s <= committed_until + 1.0e-9)
                continue;

            const double proposed = clamp(
                0.25 * previous[i - 1]
                    + 0.50 * previous[i]
                    + 0.25 * previous[i + 1],
                -config_.max_lateral_offset,
                config_.max_lateral_offset);

            Candidate current_candidate;
            Candidate proposed_candidate;
            if (!evaluateCandidate(
                    map, reference[i], previous[i], current_candidate)
                || !evaluateCandidate(
                    map, reference[i], proposed, proposed_candidate))
            {
                continue;
            }

            const double required_clearance = std::min(
                clearance_targets[i], current_candidate.clearance) - 0.003;
            if (proposed_candidate.clearance >= required_clearance)
                optimized_offsets[i] = proposed;
        }

        optimized_offsets.front() = 0.0;
        optimized_offsets.back() = 0.0;
    }

    // 近端承诺段和窗口末端使用五次smoothstep连接，令连接处的一阶、
    // 二阶导数同时趋近于0，避免“固定直线突然接上一段斜线”的曲率尖峰。
    const double blend_length = std::max(
        0.24, 6.0 * config_.longitudinal_step);
    for (std::size_t i = 0; i < optimized_offsets.size(); ++i)
    {
        const double start_t = clamp(
            (reference[i].local_s - committed_until) / blend_length,
            0.0, 1.0);
        const double remaining = reference.back().local_s - reference[i].local_s;
        const double end_t = clamp(
            remaining / config_.end_blend_distance, 0.0, 1.0);
        const double start_blend = start_t * start_t * start_t
            * (10.0 - 15.0 * start_t + 6.0 * start_t * start_t);
        const double end_blend = end_t * end_t * end_t
            * (10.0 - 15.0 * end_t + 6.0 * end_t * end_t);
        optimized_offsets[i] *= std::min(start_blend, end_blend);
    }
}

bool ClearancePathOptimizer::offsetsToLocalPath(
    const MapSnapshot& map,
    const std::vector<ReferencePoint>& reference,
    const std::vector<double>& offsets,
    std::vector<geometry_msgs::PoseStamped>& path,
    double& minimum_clearance,
    double& maximum_curvature) const
{
    path.clear();
    minimum_clearance = kInfinity;
    maximum_curvature = 0.0;
    if (reference.size() != offsets.size() || reference.size() < 2)
        return false;

    path.reserve(reference.size());
    for (std::size_t i = 0; i < reference.size(); ++i)
    {
        Candidate candidate;
        if (!evaluateCandidate(map, reference[i], offsets[i], candidate))
            return false;

        minimum_clearance = std::min(
            minimum_clearance, candidate.clearance);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_frame_;
        pose.header.stamp = ros::Time(0);
        pose.pose.position.x = candidate.x;
        pose.pose.position.y = candidate.y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(reference[i].yaw);
        path.push_back(pose);
    }

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after = std::min(i + 1, path.size() - 1);
        const double yaw = std::atan2(
            path[after].pose.position.y - path[before].pose.position.y,
            path[after].pose.position.x - path[before].pose.position.x);
        path[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }

    for (std::size_t i = 1; i + 1 < path.size(); ++i)
    {
        const double ax = path[i].pose.position.x - path[i - 1].pose.position.x;
        const double ay = path[i].pose.position.y - path[i - 1].pose.position.y;
        const double bx = path[i + 1].pose.position.x - path[i].pose.position.x;
        const double by = path[i + 1].pose.position.y - path[i].pose.position.y;
        const double cx = path[i + 1].pose.position.x - path[i - 1].pose.position.x;
        const double cy = path[i + 1].pose.position.y - path[i - 1].pose.position.y;
        const double denominator =
            std::hypot(ax, ay) * std::hypot(bx, by) * std::hypot(cx, cy);
        if (denominator > 1.0e-9)
        {
            const double curvature =
                2.0 * std::abs(ax * by - ay * bx) / denominator;
            maximum_curvature = std::max(maximum_curvature, curvature);
        }
    }
    return std::isfinite(minimum_clearance);
}

void ClearancePathOptimizer::buildFullPath(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const std::vector<double>& plan_s,
    double window_start_s,
    double window_end_s,
    const std::vector<ReferencePoint>& reference,
    const std::vector<double>& offsets,
    std::vector<geometry_msgs::PoseStamped>& full_path) const
{
    full_path = plan;
    if (reference.empty() || reference.size() != offsets.size())
        return;

    std::size_t ref_index = 0;
    int first_changed = static_cast<int>(plan.size());
    int last_changed = -1;

    for (std::size_t i = 0; i < plan.size(); ++i)
    {
        if (plan_s[i] < window_start_s - 1.0e-9
            || plan_s[i] > window_end_s + 1.0e-9)
        {
            continue;
        }

        while (ref_index + 1 < reference.size()
               && reference[ref_index + 1].plan_s < plan_s[i])
        {
            ++ref_index;
        }

        const std::size_t next = std::min(ref_index + 1, reference.size() - 1);
        const double interval = reference[next].plan_s - reference[ref_index].plan_s;
        const double ratio = interval > 1.0e-9
            ? clamp((plan_s[i] - reference[ref_index].plan_s) / interval,
                    0.0, 1.0)
            : 0.0;
        const double offset = offsets[ref_index]
            + ratio * (offsets[next] - offsets[ref_index]);

        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after = std::min(i + 1, plan.size() - 1);
        const double yaw = std::atan2(
            plan[after].pose.position.y - plan[before].pose.position.y,
            plan[after].pose.position.x - plan[before].pose.position.x);
        full_path[i].pose.position.x += -std::sin(yaw) * offset;
        full_path[i].pose.position.y += std::cos(yaw) * offset;
        full_path[i].header.frame_id = costmap_frame_;
        full_path[i].header.stamp = ros::Time(0);
        first_changed = std::min(first_changed, static_cast<int>(i));
        last_changed = std::max(last_changed, static_cast<int>(i));
    }

    if (last_changed >= first_changed)
    {
        const int begin = std::max(0, first_changed - 1);
        const int end = std::min(
            static_cast<int>(full_path.size()) - 1, last_changed + 1);
        for (int i = begin; i <= end; ++i)
        {
            const int before = std::max(0, i - 1);
            const int after = std::min(
                static_cast<int>(full_path.size()) - 1, i + 1);
            const double yaw = std::atan2(
                full_path[static_cast<std::size_t>(after)].pose.position.y
                    - full_path[static_cast<std::size_t>(before)].pose.position.y,
                full_path[static_cast<std::size_t>(after)].pose.position.x
                    - full_path[static_cast<std::size_t>(before)].pose.position.x);
            full_path[static_cast<std::size_t>(i)].pose.orientation =
                tf::createQuaternionMsgFromYaw(yaw);
        }
    }
}

bool ClearancePathOptimizer::validateOutputCenterline(
    const MapSnapshot& map,
    const std::vector<geometry_msgs::PoseStamped>& path) const
{
    if (path.size() < 2)
        return false;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        if (!centerlineSegmentIsValid(
                map,
                path[i - 1].pose.position.x,
                path[i - 1].pose.position.y,
                path[i].pose.position.x,
                path[i].pose.position.y))
        {
            return false;
        }
    }
    return true;
}

void ClearancePathOptimizer::publishPath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    const ros::Publisher& publisher,
    double z_offset) const
{
    if (path.empty() || publisher.getNumSubscribers() == 0)
        return;
    nav_msgs::Path message;
    message.header.frame_id = costmap_frame_;
    message.header.stamp = ros::Time::now();
    message.poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        geometry_msgs::PoseStamped pose = path[i];
        pose.header = message.header;
        pose.pose.position.z += z_offset;
        message.poses.push_back(pose);
    }
    publisher.publish(message);
}

void ClearancePathOptimizer::publishOutput(const Output& output)
{
    if (!config_.publish_debug_paths)
        return;
    publishPath(output.reference_path, reference_path_pub_, 0.04);
    publishPath(output.lattice_seed_path, lattice_seed_path_pub_, 0.07);
    publishPath(output.optimized_local_path, optimized_path_pub_, 0.10);
}

bool ClearancePathOptimizer::optimize(
    const std::vector<geometry_msgs::PoseStamped>& plan_costmap,
    const geometry_msgs::PoseStamped& robot_pose_costmap,
    Output& output)
{
    output = Output();
    if (!enabled() || plan_costmap.size() < 3 || costmap_ == NULL)
        return false;

    const ros::Time now = ros::Time::now();
    const double minimum_interval = 1.0 / config_.update_rate;
    if (!last_update_time_.isZero()
        && (now - last_update_time_).toSec() < minimum_interval
        && cached_output_.valid)
    {
        output = cached_output_;
        output.report.fresh_result = false;
        output.report.used_cache = true;
        output.report.status = "RATE_LIMIT_CACHE";
        publishOutput(output);
        return true;
    }
    last_update_time_ = now;

    const ros::WallTime begin = ros::WallTime::now();
    MapSnapshot map;
    std::vector<double> plan_s;
    double window_start_s = 0.0;
    double window_end_s = 0.0;
    std::vector<ReferencePoint> reference;
    std::vector<double> seed_offsets;
    std::vector<double> clearance_targets;
    std::vector<double> optimized_offsets;
    std::string failure_reason;
    int candidate_count = 0;

    const auto fallbackToCache = [&](const std::string& status,
                                     const std::string& detail) -> bool
    {
        if (!cached_output_.valid)
        {
            output.report.status = status;
            output.report.detail = detail;
            output.report.compute_ms = elapsedMilliseconds(begin);
            ROS_WARN_THROTTLE(
                1.0,
                "C4.1路径优化器本轮无输出：%s（%s）。shadow模式不会停车或重规划。",
                status.c_str(), detail.c_str());
            publishOutput(output);
            return false;
        }

        output = cached_output_;
        output.report.fresh_result = false;
        output.report.accepted = false;
        output.report.used_cache = true;
        output.report.status = status + "_CACHE";
        output.report.detail = detail;
        output.report.compute_ms = elapsedMilliseconds(begin);
        ROS_WARN_THROTTLE(
            1.0,
            "C4.1路径优化器本轮候选未采用，继续发布上一条路径：%s（%s）。",
            status.c_str(), detail.c_str());
        publishOutput(output);
        return true;
    };

    if (!buildMapSnapshot(map))
        return fallbackToCache("MAP_SNAPSHOT_FAILED", "无法复制局部代价地图");

    if (!buildReferencePath(
            plan_costmap, robot_pose_costmap,
            plan_s, window_start_s, window_end_s, reference))
    {
        return fallbackToCache("REFERENCE_FAILED", "局部参考路径构建失败");
    }

    output.reference_path.reserve(reference.size());
    for (std::size_t i = 0; i < reference.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_frame_;
        pose.pose.position.x = reference[i].x;
        pose.pose.position.y = reference[i].y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(reference[i].yaw);
        output.reference_path.push_back(pose);
    }

    if (!runLatticeDynamicProgramming(
            map, reference, seed_offsets, clearance_targets,
            candidate_count, failure_reason))
    {
        return fallbackToCache("LATTICE_FAILED", failure_reason);
    }

    double seed_minimum_clearance = 0.0;
    double seed_maximum_curvature = 0.0;
    if (!offsetsToLocalPath(
            map, reference, seed_offsets, output.lattice_seed_path,
            seed_minimum_clearance, seed_maximum_curvature)
        || !validateOutputCenterline(map, output.lattice_seed_path))
    {
        return fallbackToCache(
            "SEED_VALIDATION_FAILED", "格点种子中心线验证失败");
    }

    runElasticRefinement(
        map, reference, clearance_targets, seed_offsets, optimized_offsets);

    double minimum_clearance = 0.0;
    double maximum_curvature = 0.0;
    bool refined_valid = offsetsToLocalPath(
        map, reference, optimized_offsets, output.optimized_local_path,
        minimum_clearance, maximum_curvature);
    refined_valid = refined_valid
        && validateOutputCenterline(map, output.optimized_local_path)
        && maximum_curvature <= config_.max_accepted_curvature;

    if (!refined_valid)
    {
        optimized_offsets = seed_offsets;
        output.optimized_local_path = output.lattice_seed_path;
        minimum_clearance = seed_minimum_clearance;
        maximum_curvature = seed_maximum_curvature;
    }

    if (maximum_curvature > config_.max_accepted_curvature)
    {
        return fallbackToCache(
            "CURVATURE_REJECTED", "格点种子曲率仍超过验收上限");
    }

    double maximum_history_jump = 0.0;
    bool compared_history = false;
    for (std::size_t i = 0; i < reference.size(); ++i)
    {
        bool available = false;
        const double history_offset = historyOffsetForReference(reference[i], available);
        if (available)
        {
            compared_history = true;
            maximum_history_jump = std::max(
                maximum_history_jump,
                std::abs(optimized_offsets[i] - history_offset));
        }
    }
    if (compared_history && maximum_history_jump > config_.max_history_jump)
    {
        return fallbackToCache(
            "HISTORY_JUMP_REJECTED", "相对上一条路径的横向跳变过大");
    }

    buildFullPath(
        plan_costmap, plan_s, window_start_s, window_end_s,
        reference, optimized_offsets, output.optimized_full_path);

    output.valid = output.optimized_full_path.size() == plan_costmap.size();
    output.report.fresh_result = true;
    output.report.accepted = output.valid;
    output.report.used_cache = false;
    output.report.compute_ms = elapsedMilliseconds(begin);
    output.report.minimum_clearance = minimum_clearance;
    output.report.maximum_curvature = maximum_curvature;
    output.report.reference_points = static_cast<int>(reference.size());
    output.report.candidate_count = candidate_count;
    output.report.status = refined_valid ? "ACCEPTED_REFINED" : "ACCEPTED_SEED";
    output.report.detail = config_.shadow_mode
        ? "shadow模式，仅发布可视化" : "优化路径已允许交给控制器";

    for (std::size_t i = 0; i < optimized_offsets.size(); ++i)
    {
        output.report.maximum_lateral_offset = std::max(
            output.report.maximum_lateral_offset,
            std::abs(optimized_offsets[i]));
    }

    if (!output.valid)
        return fallbackToCache("FULL_PATH_FAILED", "完整路径拼接失败");

    last_accepted_local_path_ = output.optimized_local_path;
    cached_output_ = output;
    publishOutput(output);

    ROS_INFO_THROTTLE(
        0.5,
        "C4.1路径优化[%s]：点=%d×候选=%d，最大横移=%.3fm，"
        "最小估算净空=%.3fm，最大曲率=%.2f，耗时=%.2fms。",
        output.report.status.c_str(),
        output.report.reference_points,
        output.report.candidate_count,
        output.report.maximum_lateral_offset,
        output.report.minimum_clearance,
        output.report.maximum_curvature,
        output.report.compute_ms);
    return true;
}

}  // namespace my_planner
