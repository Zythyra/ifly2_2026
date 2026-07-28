#include "my_planner.h"

#include <boost/thread/locks.hpp>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace
{

using my_planner::MyPlanner;

geometry_msgs::Point toGeometryPoint(double x, double y, double z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

std_msgs::ColorRGBA makeColor(
    double red,
    double green,
    double blue,
    double alpha)
{
    std_msgs::ColorRGBA color;
    color.r = red;
    color.g = green;
    color.b = blue;
    color.a = alpha;
    return color;
}

}  // namespace

namespace my_planner
{

void MyPlanner::startCorridorWorker()
{
    std::lock_guard<std::mutex> lock(corridor_worker_mutex_);

    if (corridor_worker_thread_.joinable())
        return;

    corridor_worker_stop_ = false;
    corridor_update_requested_ = false;
    corridor_clear_requested_ = false;

    corridor_worker_thread_ =
        std::thread(&MyPlanner::corridorWorkerLoop, this);
}

void MyPlanner::stopCorridorWorker()
{
    {
        std::lock_guard<std::mutex> lock(corridor_worker_mutex_);
        corridor_worker_stop_ = true;
        corridor_update_requested_ = false;
        corridor_clear_requested_ = false;
    }

    corridor_worker_condition_.notify_all();

    if (corridor_worker_thread_.joinable())
        corridor_worker_thread_.join();
}

void MyPlanner::requestCorridorUpdate(bool force_update)
{
    if (!enable_corridor_visualization_)
        return;

    {
        std::lock_guard<std::mutex> lock(corridor_worker_mutex_);
        corridor_update_requested_ = true;
        corridor_clear_requested_ = false;

        if (force_update)
            force_corridor_update_ = true;
    }

    corridor_worker_condition_.notify_one();
}

void MyPlanner::requestCorridorClear()
{
    if (!enable_corridor_visualization_)
        return;

    {
        std::lock_guard<std::mutex> lock(corridor_worker_mutex_);
        corridor_update_requested_ = false;
        corridor_clear_requested_ = true;
    }

    corridor_worker_condition_.notify_one();
}

void MyPlanner::updateCorridorVisualizationIfNeeded()
{
    // 20Hz控制线程只在到达更新周期且worker空闲时，
    // 将当前非累积治愈路径复制到缓存并唤醒worker。
    if (!enable_corridor_visualization_)
        return;

    if (raw_plan_.empty() || pose_adjusting_ || goal_reached_)
    {
        if (!corridor_visualization_cleared_.load())
            requestCorridorClear();

        return;
    }

    const ros::Time now = ros::Time::now();
    const double period = 1.0 / corridor_update_frequency_;

    if (!force_corridor_update_
        && !last_corridor_request_time_.isZero()
        && (now - last_corridor_request_time_).toSec() < period)
    {
        return;
    }

    if (corridor_worker_busy_.load())
        return;

    // 与MPC版不同，这里明确使用当前周期的治愈路径生成安全框。
    // refresh不增加plan_generation，保证相邻走廊QP能够使用时间连续项。
    refreshCorridorPlanCache(global_plan_);

    last_corridor_request_time_ = now;
    force_corridor_update_ = false;
    requestCorridorUpdate(false);
}

void MyPlanner::corridorWorkerLoop()
{
    while (ros::ok())
    {
        bool do_update = false;
        bool do_clear = false;

        {
            std::unique_lock<std::mutex> lock(corridor_worker_mutex_);

            corridor_worker_condition_.wait(
                lock,
                [&]()
                {
                    return corridor_worker_stop_
                           || corridor_update_requested_
                           || corridor_clear_requested_;
                });

            if (corridor_worker_stop_)
                break;

            do_clear = corridor_clear_requested_;
            do_update = corridor_update_requested_;

            corridor_clear_requested_ = false;
            corridor_update_requested_ = false;
        }

        if (do_clear)
            clearCorridorVisualization();

        if (do_update)
        {
            corridor_worker_busy_.store(true);
            computeAndPublishCorridorSnapshot();
            corridor_worker_busy_.store(false);
        }
    }
}

void MyPlanner::rebuildCorridorPlanCache(
    const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::vector<PathPoint2D> cached_plan;
    std::string plan_frame;

    if (!plan.empty())
    {
        plan_frame = plan.front().header.frame_id;

        if (plan_frame.empty())
            plan_frame = costmap_frame_;

        cached_plan.reserve(plan.size());

        double cumulative_s = 0.0;

        for (std::size_t i = 0; i < plan.size(); ++i)
        {
            geometry_msgs::PoseStamped normalized_pose = plan[i];

            if (normalized_pose.header.frame_id.empty())
                normalized_pose.header.frame_id = plan_frame;

            if (normalized_pose.header.frame_id != plan_frame)
            {
                geometry_msgs::PoseStamped transformed_pose;

                if (!transformPose(
                        plan_frame,
                        normalized_pose,
                        transformed_pose))
                {
                    continue;
                }

                normalized_pose = transformed_pose;
            }

            PathPoint2D point;
            point.x = normalized_pose.pose.position.x;
            point.y = normalized_pose.pose.position.y;

            if (!cached_plan.empty())
            {
                const double segment_length =
                    std::hypot(
                        point.x - cached_plan.back().x,
                        point.y - cached_plan.back().y);

                if (segment_length < 1e-5)
                    continue;

                cumulative_s += segment_length;
            }

            point.s = cumulative_s;
            cached_plan.push_back(point);
        }

        updatePathYaw(cached_plan);
    }

    {
        std::lock_guard<std::mutex> lock(
            corridor_plan_cache_mutex_);

        corridor_cached_plan_.swap(cached_plan);
        corridor_cached_plan_frame_ = plan_frame;
        ++corridor_plan_generation_;
    }
}


void MyPlanner::refreshCorridorPlanCache(
    const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::vector<PathPoint2D> cached_plan;
    std::string plan_frame;

    if (!plan.empty())
    {
        plan_frame = plan.front().header.frame_id;
        if (plan_frame.empty())
            plan_frame = costmap_frame_;

        cached_plan.reserve(plan.size());
        double cumulative_s = 0.0;

        for (std::size_t i = 0; i < plan.size(); ++i)
        {
            geometry_msgs::PoseStamped normalized_pose = plan[i];

            if (normalized_pose.header.frame_id.empty())
                normalized_pose.header.frame_id = plan_frame;

            if (normalized_pose.header.frame_id != plan_frame)
            {
                geometry_msgs::PoseStamped transformed_pose;
                if (!transformPose(
                        plan_frame,
                        normalized_pose,
                        transformed_pose))
                {
                    continue;
                }
                normalized_pose = transformed_pose;
            }

            PathPoint2D point;
            point.x = normalized_pose.pose.position.x;
            point.y = normalized_pose.pose.position.y;

            if (!cached_plan.empty())
            {
                const double segment_length =
                    std::hypot(
                        point.x - cached_plan.back().x,
                        point.y - cached_plan.back().y);

                if (segment_length < 1e-5)
                    continue;

                cumulative_s += segment_length;
            }

            point.s = cumulative_s;
            cached_plan.push_back(point);
        }

        updatePathYaw(cached_plan);
    }

    std::lock_guard<std::mutex> lock(
        corridor_plan_cache_mutex_);

    corridor_cached_plan_.swap(cached_plan);
    corridor_cached_plan_frame_ = plan_frame;
}

bool MyPlanner::copyCorridorPlanCache(
    std::vector<PathPoint2D>& cached_plan,
    std::string& plan_frame,
    std::uint64_t& generation) const
{
    std::lock_guard<std::mutex> lock(
        corridor_plan_cache_mutex_);

    cached_plan = corridor_cached_plan_;
    plan_frame = corridor_cached_plan_frame_;
    generation = corridor_plan_generation_;

    return cached_plan.size() >= 2 && !plan_frame.empty();
}

bool MyPlanner::corridorPlanGenerationIsCurrent(
    std::uint64_t generation) const
{
    std::lock_guard<std::mutex> lock(
        corridor_plan_cache_mutex_);

    return generation == corridor_plan_generation_;
}

void MyPlanner::clearCorridorVisualization()
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_all;
    delete_all.header.frame_id = costmap_frame_;
    delete_all.header.stamp = ros::Time::now();
    delete_all.action = visualization_msgs::Marker::DELETEALL;

    marker_array.markers.push_back(delete_all);
    corridor_markers_pub_.publish(marker_array);

    nav_msgs::Path empty_path;
    empty_path.header.frame_id = costmap_frame_;
    empty_path.header.stamp = ros::Time::now();
    corridor_reference_path_pub_.publish(empty_path);

    invalidateCorridorReferenceSnapshot();
    corridor_visualization_cleared_.store(true);
}

void MyPlanner::computeAndPublishCorridorSnapshot()
{
    typedef std::chrono::steady_clock Clock;

    const Clock::time_point total_begin = Clock::now();

    std::vector<PathPoint2D> cached_plan;
    std::string plan_frame;
    std::uint64_t generation = 0;

    if (!copyCorridorPlanCache(
            cached_plan,
            plan_frame,
            generation))
    {
        clearCorridorVisualization();
        return;
    }

    const bool generation_changed =
        generation != corridor_worker_seen_generation_;

    if (generation_changed)
    {
        corridor_progress_segment_index_ = 0;
        corridor_worker_seen_generation_ = generation;
    }

    geometry_msgs::PoseStamped robot_pose_costmap;

    if (!getRobotPoseInCostmap(robot_pose_costmap))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "PP-C0.5-SAFETY-CORRIDOR：worker无法获取机器人在%s中的位姿。",
            costmap_frame_.c_str());
        return;
    }

    const Clock::time_point costmap_begin = Clock::now();

    CostmapSnapshot snapshot;
    if (!buildCostmapSnapshot(snapshot))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "PP-C0.5-SAFETY-CORRIDOR：worker生成local costmap快照失败。");
        return;
    }

    const Clock::time_point costmap_end = Clock::now();
    const Clock::time_point path_begin = Clock::now();

    std::vector<PathPoint2D> support_path;
    std::vector<PathPoint2D> forward_path;
    double projection_global_s = 0.0;
    double remaining_path_length = 0.0;

    if (!buildLocalPathWindow(
            cached_plan,
            plan_frame,
            robot_pose_costmap,
            corridor_progress_segment_index_,
            generation_changed,
            support_path,
            forward_path,
            projection_global_s,
            remaining_path_length))
    {
        if (remaining_path_length
            <= corridor_terminal_ignore_distance_)
        {
            clearCorridorVisualization();

            if (corridor_debug_log_)
            {
                ROS_INFO_THROTTLE(
                    1.0,
                    "PP-C0.5-SAFETY-CORRIDOR：终点剩余路径仅%.3fm，"
                    "正常停止走廊可视化。",
                    remaining_path_length);
            }
        }
        else
        {
            ROS_WARN_THROTTLE(
                1.0,
                "PP-C0.5-SAFETY-CORRIDOR：worker生成前方稠密局部参考路径失败，"
                "剩余长度=%.3fm。",
                remaining_path_length);
        }

        return;
    }

    std::vector<PathPoint2D> skeleton_path;

    if (!buildSkeletonPath(forward_path, skeleton_path))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "PP-C0.5-SAFETY-CORRIDOR：走廊骨架提取失败。");
        return;
    }

    const Clock::time_point path_end = Clock::now();
    const Clock::time_point corridor_begin = Clock::now();

    std::vector<CorridorSegment> corridors;
    buildSegmentCorridors(snapshot, skeleton_path, corridors);

    // 安全盾检查实际footprint是否位于障碍物裁剪后的自由多边形并集。
    // 因此这里不再要求“车体中心走廊”有效，也不求解任何OSQP曲线。
    double usable_length = 0.0;
    std::size_t prefix_count = 0;
    int first_failure_index = -1;
    std::string first_failure_reason;

    for (std::size_t i = 0; i < corridors.size(); ++i)
    {
        if (!corridors[i].clipped_valid
            || !polygonIsUsable(corridors[i].clipped_polygon))
        {
            first_failure_index = static_cast<int>(i);
            first_failure_reason = corridors[i].failure_reason.empty()
                ? "障碍物裁剪后的自由多边形无效"
                : corridors[i].failure_reason;
            break;
        }

        corridors[i].chain_valid = true;
        usable_length += corridors[i].length;
        ++prefix_count;
    }

    const bool full_chain =
        !corridors.empty() && prefix_count == corridors.size();
    const bool minimum_length_available =
        usable_length >= corridor_min_usable_chain_length_;
    const bool preferred_length_available =
        full_chain || usable_length >= corridor_preferred_chain_length_;

    const Clock::time_point corridor_end = Clock::now();

    // setPlan可能在worker计算期间更新路径，旧结果不得覆盖新路径。
    if (!corridorPlanGenerationIsCurrent(generation))
    {
        if (corridor_debug_log_)
        {
            ROS_INFO(
                "PP-C0.5-SAFETY-CORRIDOR：全局路径在走廊计算期间已更新，"
                "丢弃generation=%llu的旧结果。",
                static_cast<unsigned long long>(generation));
        }
        return;
    }

    std::vector<PathPoint2D> safety_reference_path;
    if (prefix_count > 0)
    {
        const double reference_limit = usable_length + 1e-6;
        for (std::size_t i = 0; i < forward_path.size(); ++i)
        {
            if (forward_path[i].s <= reference_limit)
                safety_reference_path.push_back(forward_path[i]);
            else
                break;
        }
    }

    if (minimum_length_available
        && prefix_count > 0
        && safety_reference_path.size() >= 2)
    {
        CorridorReferenceSnapshot snapshot_result;
        snapshot_result.valid = true;
        snapshot_result.plan_generation = generation;
        snapshot_result.stamp = ros::Time::now();
        snapshot_result.frame_id = costmap_frame_;
        snapshot_result.status = full_chain ? "FULL" : "PREFIX";
        snapshot_result.projection_global_s = projection_global_s;
        snapshot_result.remaining_path_length = remaining_path_length;
        snapshot_result.usable_length = usable_length;
        snapshot_result.reference_path = safety_reference_path;
        snapshot_result.corridors.assign(
            corridors.begin(),
            corridors.begin() + static_cast<std::ptrdiff_t>(prefix_count));
        storeCorridorReferenceSnapshot(snapshot_result);
        publishReferencePath(
            safety_reference_path,
            corridor_reference_path_pub_,
            0.065);
    }
    else
    {
        invalidateCorridorReferenceSnapshot();
        nav_msgs::Path empty_reference;
        empty_reference.header.frame_id = costmap_frame_;
        empty_reference.header.stamp = ros::Time::now();
        corridor_reference_path_pub_.publish(empty_reference);
    }

    const Clock::time_point publish_begin = Clock::now();

    publishCorridorMarkers(
        corridors,
        prefix_count,
        preferred_length_available);

    corridor_visualization_cleared_.store(prefix_count == 0);

    const Clock::time_point publish_end = Clock::now();

    int clipped_valid_count = 0;
    int center_valid_count = 0;
    int reference_inside_count = 0;
    int total_candidates = 0;
    int total_cuts = 0;

    for (std::size_t i = 0; i < corridors.size(); ++i)
    {
        if (corridors[i].clipped_valid)
            ++clipped_valid_count;

        if (corridors[i].center_valid)
            ++center_valid_count;

        if (corridors[i].reference_inside_center)
            ++reference_inside_count;

        total_candidates += corridors[i].candidate_obstacle_count;
        total_cuts += corridors[i].applied_cut_count;
    }

    const double costmap_ms =
        std::chrono::duration<double, std::milli>(
            costmap_end - costmap_begin).count();

    const double path_ms =
        std::chrono::duration<double, std::milli>(
            path_end - path_begin).count();

    const double corridor_ms =
        std::chrono::duration<double, std::milli>(
            corridor_end - corridor_begin).count();

    const double publish_ms =
        std::chrono::duration<double, std::milli>(
            publish_end - publish_begin).count();

    const double total_ms =
        std::chrono::duration<double, std::milli>(
            publish_end - total_begin).count();

    const char* status = "INVALID";

    if (full_chain)
        status = "FULL";
    else if (preferred_length_available)
        status = "PREFERRED_PREFIX";
    else if (minimum_length_available)
        status = "SHORT_PREFIX";

    if (corridor_debug_log_)
    {
        ROS_INFO(
            "PP-C0.5-SAFETY-CORRIDOR：status=%s，projection_s=%.3f，remaining=%.3f，"
            "raw=%zu，skeleton=%zu，segment=%zu，prefix=%zu，"
            "prefix_length=%.3f，裁剪有效=%d，中心有效=%d，"
            "原参考段位于中心走廊=%d，候选障碍=%d，切割=%d，"
            "耗时ms(total=%.2f costmap=%.2f path=%.2f "
            "corridor=%.2f publish=%.2f)。",
            status,
            projection_global_s,
            remaining_path_length,
            forward_path.size(),
            skeleton_path.size(),
            corridors.size(),
            prefix_count,
            usable_length,
            clipped_valid_count,
            center_valid_count,
            reference_inside_count,
            total_candidates,
            total_cuts,
            total_ms,
            costmap_ms,
            path_ms,
            corridor_ms,
            publish_ms);
    }


    if (!full_chain && first_failure_index >= 0)
    {
        const CorridorSegment& failed =
            corridors[static_cast<std::size_t>(first_failure_index)];

        ROS_WARN(
            "PP-C0.5-SAFETY-CORRIDOR：连续走廊在segment=%d处截断，"
            "可用前缀=%.3fm，原因=%s，"
            "C=%.4f，A=%.4f，I=%.4f。"
            "%s",
            first_failure_index,
            usable_length,
            first_failure_reason.c_str(),
            failed.clipped_area,
            failed.center_area,
            failed.overlap_area_previous,
            minimum_length_available
                ? "保留失败前可用自由框链。"
                : "连续长度不足最小阈值，本周期安全盾不使用该走廊。");
    }
}

bool MyPlanner::getRobotPoseInCostmap(
    geometry_msgs::PoseStamped& robot_pose_costmap)
{
    geometry_msgs::PoseStamped robot_origin;
    robot_origin.header.frame_id = base_frame_;
    robot_origin.header.stamp = ros::Time(0);
    robot_origin.pose.orientation.w = 1.0;

    return transformPose(
        costmap_frame_,
        robot_origin,
        robot_pose_costmap);
}

bool MyPlanner::buildCostmapSnapshot(CostmapSnapshot& snapshot)
{
    if (costmap_ros_ == NULL || costmap_ros_->getCostmap() == NULL)
        return false;

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t>
        lock(*(costmap->getMutex()));

    snapshot.size_x = costmap->getSizeInCellsX();
    snapshot.size_y = costmap->getSizeInCellsY();
    snapshot.resolution = costmap->getResolution();
    snapshot.origin_x = costmap->getOriginX();
    snapshot.origin_y = costmap->getOriginY();

    const std::size_t cell_count =
        static_cast<std::size_t>(snapshot.size_x)
        * static_cast<std::size_t>(snapshot.size_y);

    const unsigned char* char_map = costmap->getCharMap();

    if (char_map == NULL || cell_count == 0)
        return false;

    snapshot.data.assign(char_map, char_map + cell_count);
    return true;
}

bool MyPlanner::snapshotWorldToMap(
    const CostmapSnapshot& snapshot,
    double wx,
    double wy,
    unsigned int& mx,
    unsigned int& my) const
{
    if (snapshot.resolution <= 0.0
        || wx < snapshot.origin_x
        || wy < snapshot.origin_y)
    {
        return false;
    }

    const double cell_x =
        (wx - snapshot.origin_x) / snapshot.resolution;
    const double cell_y =
        (wy - snapshot.origin_y) / snapshot.resolution;

    if (cell_x < 0.0
        || cell_y < 0.0
        || cell_x >= static_cast<double>(snapshot.size_x)
        || cell_y >= static_cast<double>(snapshot.size_y))
    {
        return false;
    }

    mx = static_cast<unsigned int>(cell_x);
    my = static_cast<unsigned int>(cell_y);
    return true;
}

MyPlanner::Point2D MyPlanner::snapshotMapToWorld(
    const CostmapSnapshot& snapshot,
    unsigned int mx,
    unsigned int my) const
{
    return Point2D(
        snapshot.origin_x
            + (static_cast<double>(mx) + 0.5) * snapshot.resolution,
        snapshot.origin_y
            + (static_cast<double>(my) + 0.5) * snapshot.resolution);
}

unsigned char MyPlanner::snapshotCost(
    const CostmapSnapshot& snapshot,
    unsigned int mx,
    unsigned int my) const
{
    if (mx >= snapshot.size_x || my >= snapshot.size_y)
        return costmap_2d::NO_INFORMATION;

    const std::size_t index =
        static_cast<std::size_t>(my)
        * static_cast<std::size_t>(snapshot.size_x)
        + static_cast<std::size_t>(mx);

    if (index >= snapshot.data.size())
        return costmap_2d::NO_INFORMATION;

    return snapshot.data[index];
}

bool MyPlanner::snapshotCellIsHardObstacle(
    const CostmapSnapshot& snapshot,
    unsigned int mx,
    unsigned int my) const
{
    const unsigned char cost = snapshotCost(snapshot, mx, my);

    if (cost == costmap_2d::NO_INFORMATION)
        return corridor_treat_unknown_as_obstacle_;

    return cost >= corridor_hard_cost_threshold_;
}

bool MyPlanner::snapshotCellIsObstacleBoundary(
    const CostmapSnapshot& snapshot,
    unsigned int mx,
    unsigned int my) const
{
    if (!snapshotCellIsHardObstacle(snapshot, mx, my))
        return false;

    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            if (dx == 0 && dy == 0)
                continue;

            const int neighbour_x = static_cast<int>(mx) + dx;
            const int neighbour_y = static_cast<int>(my) + dy;

            if (neighbour_x < 0
                || neighbour_y < 0
                || neighbour_x >= static_cast<int>(snapshot.size_x)
                || neighbour_y >= static_cast<int>(snapshot.size_y))
            {
                return true;
            }

            if (!snapshotCellIsHardObstacle(
                    snapshot,
                    static_cast<unsigned int>(neighbour_x),
                    static_cast<unsigned int>(neighbour_y)))
            {
                return true;
            }
        }
    }

    return false;
}

bool MyPlanner::buildLocalPathWindow(
    const std::vector<PathPoint2D>& cached_plan,
    const std::string& plan_frame,
    const geometry_msgs::PoseStamped& robot_pose_costmap,
    std::size_t& progress_segment_index,
    bool full_projection_search,
    std::vector<PathPoint2D>& support_path,
    std::vector<PathPoint2D>& forward_path,
    double& projection_global_s,
    double& remaining_path_length)
{
    support_path.clear();
    forward_path.clear();
    projection_global_s = 0.0;
    remaining_path_length = 0.0;

    if (cached_plan.size() < 2 || plan_frame.empty())
        return false;

    tf::StampedTransform plan_to_costmap;

    try
    {
        if (plan_frame == costmap_frame_)
        {
            plan_to_costmap.setIdentity();
        }
        else
        {
            tf_listener_->lookupTransform(
                costmap_frame_,
                plan_frame,
                ros::Time(0),
                plan_to_costmap);
        }
    }
    catch (const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "PP-C0.5-SAFETY-CORRIDOR：查询%s到%s的单次路径变换失败：%s",
            plan_frame.c_str(),
            costmap_frame_.c_str(),
            ex.what());
        return false;
    }

    const tf::Transform costmap_to_plan =
        plan_to_costmap.inverse();

    const tf::Vector3 robot_costmap(
        robot_pose_costmap.pose.position.x,
        robot_pose_costmap.pose.position.y,
        0.0);

    const tf::Vector3 robot_plan =
        costmap_to_plan * robot_costmap;

    const std::size_t last_segment_index =
        cached_plan.size() - 2;

    progress_segment_index =
        std::min(progress_segment_index, last_segment_index);

    std::size_t search_begin = 0;
    std::size_t search_end = last_segment_index;

    if (!full_projection_search)
    {
        const std::size_t behind =
            static_cast<std::size_t>(
                corridor_projection_search_behind_points_);

        const std::size_t ahead =
            static_cast<std::size_t>(
                corridor_projection_search_ahead_points_);

        search_begin =
            progress_segment_index > behind
                ? progress_segment_index - behind
                : 0;

        search_end = std::min(
            last_segment_index,
            progress_segment_index + ahead);
    }

    double closest_distance =
        std::numeric_limits<double>::max();

    double closest_s = cached_plan.front().s;
    std::size_t closest_segment_index = search_begin;

    auto search_projection =
        [&](std::size_t begin_index, std::size_t end_index)
        {
            for (std::size_t i = begin_index;
                 i <= end_index && i + 1 < cached_plan.size();
                 ++i)
            {
                const PathPoint2D& first = cached_plan[i];
                const PathPoint2D& second = cached_plan[i + 1];

                const double dx = second.x - first.x;
                const double dy = second.y - first.y;
                const double length_squared = dx * dx + dy * dy;

                if (length_squared < 1e-12)
                    continue;

                const double ratio = clampValue(
                    ((robot_plan.x() - first.x) * dx
                     + (robot_plan.y() - first.y) * dy)
                    / length_squared,
                    0.0,
                    1.0);

                const double projection_x =
                    first.x + ratio * dx;
                const double projection_y =
                    first.y + ratio * dy;

                const double distance =
                    std::hypot(
                        robot_plan.x() - projection_x,
                        robot_plan.y() - projection_y);

                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_segment_index = i;
                    closest_s =
                        first.s
                        + ratio * std::sqrt(length_squared);
                }
            }
        };

    search_projection(search_begin, search_end);

    // 局部搜索异常时退回一次全路径搜索。
    if (!full_projection_search && closest_distance > 0.80)
    {
        closest_distance =
            std::numeric_limits<double>::max();
        search_projection(0, last_segment_index);
    }

    progress_segment_index = closest_segment_index;
    projection_global_s = closest_s;
    remaining_path_length =
        std::max(0.0, cached_plan.back().s - closest_s);

    if (remaining_path_length < 0.005)
        return false;

    const double support_start_s = std::max(
        cached_plan.front().s,
        closest_s - local_path_behind_distance_);

    const double forward_end_s = std::min(
        cached_plan.back().s,
        closest_s + local_path_horizon_distance_);

    std::vector<PathPoint2D> support_plan_frame;

    for (double query_s = support_start_s;
         query_s < closest_s - 1e-8;
         query_s += local_path_resample_distance_)
    {
        PathPoint2D point;

        if (interpolatePathPoint(cached_plan, query_s, point))
        {
            point.s = query_s - closest_s;
            support_plan_frame.push_back(point);
        }
    }

    PathPoint2D projection_point;

    if (!interpolatePathPoint(
            cached_plan,
            closest_s,
            projection_point))
    {
        return false;
    }

    projection_point.s = 0.0;
    support_plan_frame.push_back(projection_point);

    for (double query_s = closest_s + local_path_resample_distance_;
         query_s < forward_end_s - 1e-8;
         query_s += local_path_resample_distance_)
    {
        PathPoint2D point;

        if (interpolatePathPoint(cached_plan, query_s, point))
        {
            point.s = query_s - closest_s;
            support_plan_frame.push_back(point);
        }
    }

    PathPoint2D final_point;

    if (!interpolatePathPoint(
            cached_plan,
            forward_end_s,
            final_point))
    {
        return false;
    }

    final_point.s = forward_end_s - closest_s;

    if (support_plan_frame.empty()
        || std::hypot(
               final_point.x - support_plan_frame.back().x,
               final_point.y - support_plan_frame.back().y)
           > 1e-5)
    {
        support_plan_frame.push_back(final_point);
    }

    const double transform_yaw =
        tf::getYaw(plan_to_costmap.getRotation());

    support_path.reserve(support_plan_frame.size());

    for (std::size_t i = 0;
         i < support_plan_frame.size();
         ++i)
    {
        const tf::Vector3 input(
            support_plan_frame[i].x,
            support_plan_frame[i].y,
            0.0);

        const tf::Vector3 output =
            plan_to_costmap * input;

        PathPoint2D transformed = support_plan_frame[i];
        transformed.x = output.x();
        transformed.y = output.y();
        transformed.yaw =
            normalizeAngle(
                support_plan_frame[i].yaw + transform_yaw);

        support_path.push_back(transformed);
    }

    if (support_path.size() < 2)
        return false;

    updatePathYaw(support_path);

    for (std::size_t i = 0;
         i < support_path.size();
         ++i)
    {
        if (support_path[i].s >= -1e-8)
            forward_path.push_back(support_path[i]);
    }

    if (forward_path.size() < 2)
        return false;

    forward_path.front().s = 0.0;
    return true;
}

bool MyPlanner::interpolatePathPoint(
    const std::vector<PathPoint2D>& path,
    double query_s,
    PathPoint2D& output) const
{
    if (path.empty())
        return false;

    if (query_s <= path.front().s)
    {
        output = path.front();
        output.s = query_s;
        return true;
    }

    if (query_s >= path.back().s)
    {
        output = path.back();
        output.s = query_s;
        return true;
    }

    for (std::size_t i = 0; i + 1 < path.size(); ++i)
    {
        if (query_s > path[i + 1].s)
            continue;

        const double ds = path[i + 1].s - path[i].s;

        if (ds < 1e-9)
        {
            output = path[i];
            output.s = query_s;
            return true;
        }

        const double ratio =
            (query_s - path[i].s) / ds;

        output.x =
            path[i].x
            + ratio * (path[i + 1].x - path[i].x);
        output.y =
            path[i].y
            + ratio * (path[i + 1].y - path[i].y);
        output.s = query_s;
        output.yaw =
            path[i].yaw
            + ratio
              * normalizeAngle(path[i + 1].yaw - path[i].yaw);
        output.curvature =
            path[i].curvature
            + ratio
              * (path[i + 1].curvature - path[i].curvature);
        output.clearance =
            path[i].clearance
            + ratio
              * (path[i + 1].clearance - path[i].clearance);
        output.corridor_index =
            ratio < 0.5
                ? path[i].corridor_index
                : path[i + 1].corridor_index;
        return true;
    }

    return false;
}

void MyPlanner::updatePathYaw(
    std::vector<PathPoint2D>& path) const
{
    if (path.empty())
        return;

    if (path.size() == 1)
    {
        path.front().yaw = 0.0;
        return;
    }

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        std::size_t first_index = i;
        std::size_t second_index = i;

        if (i == 0)
        {
            first_index = 0;
            second_index = 1;
        }
        else if (i + 1 >= path.size())
        {
            first_index = path.size() - 2;
            second_index = path.size() - 1;
        }
        else
        {
            first_index = i - 1;
            second_index = i + 1;
        }

        const double dx =
            path[second_index].x - path[first_index].x;
        const double dy =
            path[second_index].y - path[first_index].y;

        if (std::hypot(dx, dy) > 1e-7)
            path[i].yaw = std::atan2(dy, dx);
        else if (i > 0)
            path[i].yaw = path[i - 1].yaw;
        else
            path[i].yaw = 0.0;
    }
}

double MyPlanner::estimateCornerAngle(
    const std::vector<PathPoint2D>& path,
    std::size_t index) const
{
    if (path.size() < 3
        || index == 0
        || index + 1 >= path.size())
    {
        return 0.0;
    }

    std::size_t before = index;
    std::size_t after = index;

    while (before > 0
           && path[index].s - path[before].s
              < corridor_skeleton_corner_window_)
    {
        --before;
    }

    while (after + 1 < path.size()
           && path[after].s - path[index].s
              < corridor_skeleton_corner_window_)
    {
        ++after;
    }

    if (before == index || after == index)
        return 0.0;

    const double first_x = path[index].x - path[before].x;
    const double first_y = path[index].y - path[before].y;
    const double second_x = path[after].x - path[index].x;
    const double second_y = path[after].y - path[index].y;

    const double first_norm = std::hypot(first_x, first_y);
    const double second_norm = std::hypot(second_x, second_y);

    if (first_norm < 1e-7 || second_norm < 1e-7)
        return 0.0;

    const double cosine = clampValue(
        (first_x * second_x + first_y * second_y)
        / (first_norm * second_norm),
        -1.0,
        1.0);

    return std::acos(cosine);
}

bool MyPlanner::buildSkeletonPath(
    const std::vector<PathPoint2D>& forward_path,
    std::vector<PathPoint2D>& skeleton_path) const
{
    skeleton_path.clear();

    if (forward_path.size() < 2)
        return false;

    skeleton_path.push_back(forward_path.front());
    std::size_t last_selected = 0;

    const double corner_threshold =
        corridor_skeleton_corner_angle_deg_
        * std::acos(-1.0) / 180.0;

    for (std::size_t i = 1;
         i + 1 < forward_path.size();
         ++i)
    {
        const double distance_from_last =
            forward_path[i].s - forward_path[last_selected].s;

        const bool force_by_length =
            distance_from_last
            >= corridor_skeleton_max_segment_length_;

        const bool corner =
            estimateCornerAngle(forward_path, i)
            >= corner_threshold;

        const bool keep_corner =
            corner
            && distance_from_last
               >= corridor_skeleton_min_segment_length_;

        if (force_by_length || keep_corner)
        {
            skeleton_path.push_back(forward_path[i]);
            last_selected = i;
        }
    }

    // 若最后一段过长，继续按最大长度插入原稠密路径点。
    while (forward_path.back().s
           - skeleton_path.back().s
           > corridor_skeleton_max_segment_length_)
    {
        const double desired_s =
            skeleton_path.back().s
            + corridor_skeleton_max_segment_length_;

        std::size_t best_index = last_selected + 1;

        while (best_index + 1 < forward_path.size()
               && forward_path[best_index].s < desired_s)
        {
            ++best_index;
        }

        if (best_index >= forward_path.size() - 1)
            break;

        skeleton_path.push_back(forward_path[best_index]);
        last_selected = best_index;
    }

    if (std::hypot(
            forward_path.back().x - skeleton_path.back().x,
            forward_path.back().y - skeleton_path.back().y)
        > 1e-5)
    {
        skeleton_path.push_back(forward_path.back());
    }

    // 合并异常短小的中间段，避免产生退化矩形。
    if (skeleton_path.size() >= 3)
    {
        std::vector<PathPoint2D> filtered;
        filtered.push_back(skeleton_path.front());

        for (std::size_t i = 1;
             i + 1 < skeleton_path.size();
             ++i)
        {
            const double distance =
                std::hypot(
                    skeleton_path[i].x - filtered.back().x,
                    skeleton_path[i].y - filtered.back().y);

            if (distance >= 0.5 * corridor_skeleton_min_segment_length_)
                filtered.push_back(skeleton_path[i]);
        }

        filtered.push_back(skeleton_path.back());
        skeleton_path.swap(filtered);
    }

    updatePathYaw(skeleton_path);
    return skeleton_path.size() >= 2;
}

MyPlanner::ConvexPolygon MyPlanner::makeInitialCorridor(
    const CostmapSnapshot& snapshot,
    const PathPoint2D& start,
    const PathPoint2D& end) const
{
    ConvexPolygon polygon;

    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::hypot(dx, dy);

    if (length < 1e-7)
        return polygon;

    const double tangent_x = dx / length;
    const double tangent_y = dy / length;
    const double normal_x = -tangent_y;
    const double normal_y = tangent_x;

    // 每个走廊段最终还要按完整车体做Minkowski收缩。
    // 若只固定延伸0.12m，而车体半长约0.17m，收缩后纵向几乎必然归零。
    // 这里根据真实footprint分别计算车头和车尾支持距离，并额外保留一段
    // 收缩后的有效纵向长度，保证短骨架段仍能生成可用中心走廊。
    double forward_support = corridor_robot_half_length_;
    double backward_support = corridor_robot_half_length_;

    if (!corridor_robot_footprint_.empty())
    {
        forward_support = 0.0;
        backward_support = 0.0;

        for (std::size_t i = 0;
             i < corridor_robot_footprint_.size();
             ++i)
        {
            forward_support =
                std::max(
                    forward_support,
                    corridor_robot_footprint_[i].x);

            backward_support =
                std::max(
                    backward_support,
                    -corridor_robot_footprint_[i].x);
        }

        forward_support = std::max(0.0, forward_support);
        backward_support = std::max(0.0, backward_support);
    }

    const double start_extension =
        std::max(
            corridor_longitudinal_extension_,
            backward_support
                + corridor_extra_margin_
                + corridor_post_shrink_longitudinal_reserve_);

    const double end_extension =
        std::max(
            corridor_longitudinal_extension_,
            forward_support
                + corridor_extra_margin_
                + corridor_post_shrink_longitudinal_reserve_);

    const Point2D extended_start(
        start.x - start_extension * tangent_x,
        start.y - start_extension * tangent_y);

    const Point2D extended_end(
        end.x + end_extension * tangent_x,
        end.y + end_extension * tangent_y);

    // 顶点按逆时针排列。
    polygon.vertices.push_back(
        Point2D(
            extended_start.x
                + corridor_initial_half_width_ * normal_x,
            extended_start.y
                + corridor_initial_half_width_ * normal_y));

    polygon.vertices.push_back(
        Point2D(
            extended_start.x
                - corridor_initial_half_width_ * normal_x,
            extended_start.y
                - corridor_initial_half_width_ * normal_y));

    polygon.vertices.push_back(
        Point2D(
            extended_end.x
                - corridor_initial_half_width_ * normal_x,
            extended_end.y
                - corridor_initial_half_width_ * normal_y));

    polygon.vertices.push_back(
        Point2D(
            extended_end.x
                + corridor_initial_half_width_ * normal_x,
            extended_end.y
                + corridor_initial_half_width_ * normal_y));

    ensureCounterClockwise(polygon);

    const double minimum_x =
        snapshot.origin_x + corridor_map_boundary_margin_;
    const double minimum_y =
        snapshot.origin_y + corridor_map_boundary_margin_;
    const double maximum_x =
        snapshot.origin_x
        + static_cast<double>(snapshot.size_x) * snapshot.resolution
        - corridor_map_boundary_margin_;
    const double maximum_y =
        snapshot.origin_y
        + static_cast<double>(snapshot.size_y) * snapshot.resolution
        - corridor_map_boundary_margin_;

    polygon = clipPolygonByHalfPlane(polygon, -1.0, 0.0, -minimum_x);
    polygon = clipPolygonByHalfPlane(polygon,  1.0, 0.0,  maximum_x);
    polygon = clipPolygonByHalfPlane(polygon, 0.0, -1.0, -minimum_y);
    polygon = clipPolygonByHalfPlane(polygon, 0.0,  1.0,  maximum_y);

    ensureCounterClockwise(polygon);
    return polygon;
}

std::vector<MyPlanner::Point2D>
MyPlanner::collectObstacleBoundaryPoints(
    const CostmapSnapshot& snapshot,
    const ConvexPolygon& polygon,
    const PathPoint2D& segment_start,
    const PathPoint2D& segment_end) const
{
    std::vector<Point2D> obstacles;

    if (!polygonIsUsable(polygon))
        return obstacles;

    double minimum_x = polygon.vertices.front().x;
    double maximum_x = polygon.vertices.front().x;
    double minimum_y = polygon.vertices.front().y;
    double maximum_y = polygon.vertices.front().y;

    for (std::size_t i = 1; i < polygon.vertices.size(); ++i)
    {
        minimum_x = std::min(minimum_x, polygon.vertices[i].x);
        maximum_x = std::max(maximum_x, polygon.vertices[i].x);
        minimum_y = std::min(minimum_y, polygon.vertices[i].y);
        maximum_y = std::max(maximum_y, polygon.vertices[i].y);
    }

    unsigned int minimum_mx = 0;
    unsigned int minimum_my = 0;
    unsigned int maximum_mx = snapshot.size_x - 1;
    unsigned int maximum_my = snapshot.size_y - 1;

    snapshotWorldToMap(
        snapshot,
        std::max(minimum_x, snapshot.origin_x),
        std::max(minimum_y, snapshot.origin_y),
        minimum_mx,
        minimum_my);

    const double map_maximum_x =
        snapshot.origin_x
        + static_cast<double>(snapshot.size_x) * snapshot.resolution
        - 1e-9;

    const double map_maximum_y =
        snapshot.origin_y
        + static_cast<double>(snapshot.size_y) * snapshot.resolution
        - 1e-9;

    snapshotWorldToMap(
        snapshot,
        std::min(maximum_x, map_maximum_x),
        std::min(maximum_y, map_maximum_y),
        maximum_mx,
        maximum_my);

    for (unsigned int my = minimum_my;
         my <= maximum_my;
         ++my)
    {
        for (unsigned int mx = minimum_mx;
             mx <= maximum_mx;
             ++mx)
        {
            if (!snapshotCellIsObstacleBoundary(snapshot, mx, my))
                continue;

            const Point2D point =
                snapshotMapToWorld(snapshot, mx, my);

            if (pointInConvexPolygon(polygon, point, 1e-9))
                obstacles.push_back(point);
        }
    }

    std::sort(
        obstacles.begin(),
        obstacles.end(),
        [&](const Point2D& first, const Point2D& second)
        {
            return pointToSegmentDistance(
                       first,
                       segment_start,
                       segment_end)
                   < pointToSegmentDistance(
                       second,
                       segment_start,
                       segment_end);
        });

    return obstacles;
}

bool MyPlanner::buildSegmentCorridor(
    const CostmapSnapshot& snapshot,
    std::size_t index,
    const PathPoint2D& start,
    const PathPoint2D& end,
    CorridorSegment& corridor) const
{
    corridor = CorridorSegment();
    corridor.index = index;
    corridor.start = start;
    corridor.end = end;

    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    corridor.length = std::hypot(dx, dy);

    if (corridor.length < 1e-5)
    {
        corridor.failure_reason = "骨架段过短";
        return false;
    }

    corridor.yaw = std::atan2(dy, dx);
    corridor.initial_polygon =
        makeInitialCorridor(snapshot, start, end);

    if (!polygonIsUsable(corridor.initial_polygon))
    {
        corridor.failure_reason = "初始矩形被地图边界裁空";
        return false;
    }

    std::vector<Point2D> obstacle_points =
        collectObstacleBoundaryPoints(
            snapshot,
            corridor.initial_polygon,
            start,
            end);

    corridor.candidate_obstacle_count =
        static_cast<int>(obstacle_points.size());

    ConvexPolygon current = corridor.initial_polygon;

    const double automatic_padding =
        0.5 * std::sqrt(2.0) * snapshot.resolution;

    const double obstacle_padding =
        corridor_obstacle_padding_ > 0.0
            ? corridor_obstacle_padding_
            : automatic_padding;

    for (std::size_t obstacle_index = 0;
         obstacle_index < obstacle_points.size();
         ++obstacle_index)
    {
        const Point2D& obstacle = obstacle_points[obstacle_index];

        if (!pointInConvexPolygon(current, obstacle, 1e-9))
            continue;

        if (corridor.applied_cut_count >= corridor_max_obstacle_cuts_)
        {
            corridor.failure_reason =
                "达到最大半平面切割数后仍有硬障碍位于走廊内";
            corridor.clipped_polygon = current;
            corridor.clipped_area = polygonArea(current);
            return false;
        }

        const Point2D closest =
            closestPointOnSegment(obstacle, start, end);

        const double obstacle_dx = obstacle.x - closest.x;
        const double obstacle_dy = obstacle.y - closest.y;
        const double distance =
            std::hypot(obstacle_dx, obstacle_dy);

        if (distance <= obstacle_padding + 1e-6)
        {
            corridor.failure_reason =
                "骨架段距离硬障碍小于栅格安全偏移";
            corridor.clipped_polygon = current;
            corridor.clipped_area = polygonArea(current);
            return false;
        }

        const double normal_x = obstacle_dx / distance;
        const double normal_y = obstacle_dy / distance;

        // 保留包含原骨架段的一侧。边界相对障碍中心向骨架方向偏移。
        const double offset =
            normal_x * obstacle.x
            + normal_y * obstacle.y
            - obstacle_padding;

        if (normal_x * closest.x
            + normal_y * closest.y
            > offset + 1e-7)
        {
            corridor.failure_reason =
                "半平面方向异常，无法保留骨架侧";
            corridor.clipped_polygon = current;
            corridor.clipped_area = polygonArea(current);
            return false;
        }

        ConvexPolygon clipped =
            clipPolygonByHalfPlane(
                current,
                normal_x,
                normal_y,
                offset);

        if (!polygonIsUsable(clipped)
            || polygonArea(clipped) < corridor_min_polygon_area_)
        {
            corridor.failure_reason =
                "障碍半平面裁剪后多边形退化";
            corridor.clipped_polygon = clipped;
            corridor.clipped_area = polygonArea(clipped);
            return false;
        }

        current = clipped;
        corridor.cut_obstacles.push_back(obstacle);
        ++corridor.applied_cut_count;
    }

    // 最终重新扫描一次裁剪结果，确保没有hard-cost边界点残留。
    // 这一步是安全兜底，防止切割数限制或数值误差留下障碍。
    const std::vector<Point2D> residual_obstacles =
        collectObstacleBoundaryPoints(
            snapshot,
            current,
            start,
            end);

    if (!residual_obstacles.empty())
    {
        corridor.failure_reason =
            "半平面裁剪结束后仍检测到硬障碍边界点";
        corridor.clipped_polygon = current;
        corridor.clipped_area = polygonArea(current);
        return false;
    }

    corridor.clipped_polygon = current;
    corridor.clipped_area = polygonArea(current);

    if (!polygonIsUsable(current)
        || corridor.clipped_area < corridor_min_polygon_area_)
    {
        corridor.failure_reason = "裁剪走廊面积过小";
        return false;
    }

    if (!segmentInsidePolygon(
            current,
            start,
            end,
            corridor_reference_validation_step_))
    {
        corridor.failure_reason =
            "裁剪走廊未完整包含原骨架段";
        return false;
    }

    corridor.clipped_valid = true;

    corridor.center_polygon =
        shrinkPolygonForFootprint(current, corridor.yaw);
    corridor.center_area =
        polygonArea(corridor.center_polygon);

    if (!polygonIsUsable(corridor.center_polygon)
        || corridor.center_area < corridor_min_polygon_area_)
    {
        corridor.failure_reason =
            "扣除车辆矩形轮廓后无中心可行区域";
        return false;
    }

    corridor.center_valid = true;

    corridor.reference_inside_center =
        segmentInsidePolygon(
            corridor.center_polygon,
            start,
            end,
            corridor_reference_validation_step_);

    if (!corridor.reference_inside_center)
    {
        corridor.failure_reason =
            "原骨架段不在车体中心可行走廊内，后续QP需要横向修正";
    }

    return true;
}

bool MyPlanner::buildSegmentCorridors(
    const CostmapSnapshot& snapshot,
    const std::vector<PathPoint2D>& skeleton_path,
    std::vector<CorridorSegment>& corridors) const
{
    corridors.clear();

    if (skeleton_path.size() < 2)
        return false;

    corridors.reserve(skeleton_path.size() - 1);

    for (std::size_t i = 0;
         i + 1 < skeleton_path.size();
         ++i)
    {
        CorridorSegment corridor;
        buildSegmentCorridor(
            snapshot,
            i,
            skeleton_path[i],
            skeleton_path[i + 1],
            corridor);

        if (i == 0)
        {
            corridor.overlap_with_previous = true;
            corridor.overlap_area_previous = 0.0;
            corridor.chain_valid = corridor.center_valid;
        }
        else
        {
            CorridorSegment& previous = corridors.back();

            if (previous.center_valid && corridor.center_valid)
            {
                corridor.overlap_polygon =
                    intersectConvexPolygons(
                        previous.center_polygon,
                        corridor.center_polygon);

                corridor.overlap_area_previous =
                    polygonArea(corridor.overlap_polygon);

                corridor.overlap_with_previous =
                    polygonIsUsable(corridor.overlap_polygon)
                    && corridor.overlap_area_previous
                       >= corridor_min_overlap_area_;
            }
            else
            {
                corridor.overlap_with_previous = false;
                corridor.overlap_area_previous = 0.0;
            }

            corridor.chain_valid =
                previous.chain_valid
                && corridor.center_valid
                && corridor.overlap_with_previous;

            if (corridor.center_valid
                && !corridor.overlap_with_previous
                && corridor.failure_reason.empty())
            {
                corridor.failure_reason =
                    "与前一车体中心走廊交集不足";
            }
        }

        corridors.push_back(corridor);
    }

    return !corridors.empty();
}

MyPlanner::ConvexPolygon MyPlanner::clipPolygonByHalfPlane(
    const ConvexPolygon& polygon,
    double normal_x,
    double normal_y,
    double offset) const
{
    ConvexPolygon result;

    if (polygon.vertices.size() < 3)
        return result;

    const double normal_length =
        std::hypot(normal_x, normal_y);

    if (normal_length < 1e-12)
        return polygon;

    normal_x /= normal_length;
    normal_y /= normal_length;
    offset /= normal_length;

    const double tolerance = 1e-9;

    for (std::size_t i = 0;
         i < polygon.vertices.size();
         ++i)
    {
        const Point2D& current = polygon.vertices[i];
        const Point2D& next =
            polygon.vertices[
                (i + 1) % polygon.vertices.size()];

        const double current_value =
            normal_x * current.x
            + normal_y * current.y
            - offset;

        const double next_value =
            normal_x * next.x
            + normal_y * next.y
            - offset;

        const bool current_inside =
            current_value <= tolerance;
        const bool next_inside =
            next_value <= tolerance;

        if (current_inside)
            result.vertices.push_back(current);

        if (current_inside != next_inside)
        {
            const double denominator =
                current_value - next_value;

            if (std::abs(denominator) > 1e-12)
            {
                const double ratio =
                    current_value / denominator;

                result.vertices.push_back(
                    Point2D(
                        current.x
                            + ratio * (next.x - current.x),
                        current.y
                            + ratio * (next.y - current.y)));
            }
        }
    }

    // 清理几乎重复的相邻顶点。
    ConvexPolygon cleaned;

    for (std::size_t i = 0;
         i < result.vertices.size();
         ++i)
    {
        if (cleaned.vertices.empty()
            || std::hypot(
                   result.vertices[i].x
                       - cleaned.vertices.back().x,
                   result.vertices[i].y
                       - cleaned.vertices.back().y)
               > 1e-8)
        {
            cleaned.vertices.push_back(result.vertices[i]);
        }
    }

    if (cleaned.vertices.size() >= 2
        && std::hypot(
               cleaned.vertices.front().x
                   - cleaned.vertices.back().x,
               cleaned.vertices.front().y
                   - cleaned.vertices.back().y)
           <= 1e-8)
    {
        cleaned.vertices.pop_back();
    }

    ensureCounterClockwise(cleaned);
    return cleaned;
}

MyPlanner::ConvexPolygon MyPlanner::intersectConvexPolygons(
    const ConvexPolygon& first,
    const ConvexPolygon& second) const
{
    if (!polygonIsUsable(first)
        || !polygonIsUsable(second))
    {
        return ConvexPolygon();
    }

    ConvexPolygon clipping_polygon = second;
    ensureCounterClockwise(clipping_polygon);

    ConvexPolygon result = first;

    for (std::size_t i = 0;
         i < clipping_polygon.vertices.size();
         ++i)
    {
        const Point2D& first_edge_point =
            clipping_polygon.vertices[i];

        const Point2D& second_edge_point =
            clipping_polygon.vertices[
                (i + 1) % clipping_polygon.vertices.size()];

        const double edge_x =
            second_edge_point.x - first_edge_point.x;
        const double edge_y =
            second_edge_point.y - first_edge_point.y;

        const double edge_length =
            std::hypot(edge_x, edge_y);

        if (edge_length < 1e-12)
            continue;

        // 对逆时针多边形，右法向量为外法向量。
        const double outward_x = edge_y / edge_length;
        const double outward_y = -edge_x / edge_length;
        const double offset =
            outward_x * first_edge_point.x
            + outward_y * first_edge_point.y;

        result = clipPolygonByHalfPlane(
            result,
            outward_x,
            outward_y,
            offset);

        if (!polygonIsUsable(result))
            return ConvexPolygon();
    }

    ensureCounterClockwise(result);
    return result;
}

MyPlanner::ConvexPolygon MyPlanner::shrinkPolygonForFootprint(
    const ConvexPolygon& polygon,
    double vehicle_yaw) const
{
    if (!polygonIsUsable(polygon))
        return ConvexPolygon();

    ConvexPolygon source = polygon;
    ensureCounterClockwise(source);

    ConvexPolygon result = source;

    const double forward_x = std::cos(vehicle_yaw);
    const double forward_y = std::sin(vehicle_yaw);
    const double left_x = -forward_y;
    const double left_y = forward_x;

    for (std::size_t i = 0;
         i < source.vertices.size();
         ++i)
    {
        const Point2D& first = source.vertices[i];
        const Point2D& second =
            source.vertices[(i + 1) % source.vertices.size()];

        const double edge_x = second.x - first.x;
        const double edge_y = second.y - first.y;
        const double edge_length =
            std::hypot(edge_x, edge_y);

        if (edge_length < 1e-12)
            continue;

        const double outward_x = edge_y / edge_length;
        const double outward_y = -edge_x / edge_length;

        double vehicle_support = 0.0;

        if (!corridor_robot_footprint_.empty())
        {
            vehicle_support =
                -std::numeric_limits<double>::infinity();

            for (std::size_t footprint_index = 0;
                 footprint_index < corridor_robot_footprint_.size();
                 ++footprint_index)
            {
                const Point2D& local_point =
                    corridor_robot_footprint_[footprint_index];

                const double rotated_x =
                    forward_x * local_point.x
                    + left_x * local_point.y;

                const double rotated_y =
                    forward_y * local_point.x
                    + left_y * local_point.y;

                vehicle_support =
                    std::max(
                        vehicle_support,
                        outward_x * rotated_x
                            + outward_y * rotated_y);
            }

            vehicle_support = std::max(0.0, vehicle_support);
        }
        else
        {
            const double longitudinal_projection =
                std::abs(
                    outward_x * forward_x
                    + outward_y * forward_y);

            const double lateral_projection =
                std::abs(
                    outward_x * left_x
                    + outward_y * left_y);

            vehicle_support =
                corridor_robot_half_length_
                    * longitudinal_projection
                + corridor_robot_half_width_
                    * lateral_projection;
        }

        vehicle_support += corridor_extra_margin_;

        const double offset =
            outward_x * first.x
            + outward_y * first.y
            - vehicle_support;

        result = clipPolygonByHalfPlane(
            result,
            outward_x,
            outward_y,
            offset);

        if (!polygonIsUsable(result))
            return ConvexPolygon();
    }

    ensureCounterClockwise(result);
    return result;
}

bool MyPlanner::pointInConvexPolygon(
    const ConvexPolygon& polygon,
    const Point2D& point,
    double tolerance) const
{
    if (polygon.vertices.size() < 3)
        return false;

    ConvexPolygon source = polygon;
    ensureCounterClockwise(source);

    for (std::size_t i = 0;
         i < source.vertices.size();
         ++i)
    {
        const Point2D& first = source.vertices[i];
        const Point2D& second =
            source.vertices[(i + 1) % source.vertices.size()];

        const double cross =
            (second.x - first.x) * (point.y - first.y)
            - (second.y - first.y) * (point.x - first.x);

        if (cross < -tolerance)
            return false;
    }

    return true;
}

bool MyPlanner::segmentInsidePolygon(
    const ConvexPolygon& polygon,
    const PathPoint2D& start,
    const PathPoint2D& end,
    double sample_step) const
{
    if (!polygonIsUsable(polygon))
        return false;

    const double length =
        std::hypot(end.x - start.x, end.y - start.y);

    const int sample_count = std::max(
        1,
        static_cast<int>(
            std::ceil(length / std::max(0.005, sample_step))));

    for (int index = 0;
         index <= sample_count;
         ++index)
    {
        const double ratio =
            static_cast<double>(index)
            / static_cast<double>(sample_count);

        const Point2D point(
            start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y));

        if (!pointInConvexPolygon(polygon, point, 1e-7))
            return false;
    }

    return true;
}

MyPlanner::Point2D MyPlanner::closestPointOnSegment(
    const Point2D& point,
    const PathPoint2D& start,
    const PathPoint2D& end) const
{
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;

    if (length_squared < 1e-12)
        return Point2D(start.x, start.y);

    const double ratio = clampValue(
        ((point.x - start.x) * dx
         + (point.y - start.y) * dy)
        / length_squared,
        0.0,
        1.0);

    return Point2D(
        start.x + ratio * dx,
        start.y + ratio * dy);
}

double MyPlanner::pointToSegmentDistance(
    const Point2D& point,
    const PathPoint2D& start,
    const PathPoint2D& end) const
{
    const Point2D closest =
        closestPointOnSegment(point, start, end);

    return std::hypot(
        point.x - closest.x,
        point.y - closest.y);
}

double MyPlanner::polygonSignedArea(
    const ConvexPolygon& polygon) const
{
    if (polygon.vertices.size() < 3)
        return 0.0;

    double twice_area = 0.0;

    for (std::size_t i = 0;
         i < polygon.vertices.size();
         ++i)
    {
        const Point2D& first = polygon.vertices[i];
        const Point2D& second =
            polygon.vertices[(i + 1) % polygon.vertices.size()];

        twice_area +=
            first.x * second.y
            - first.y * second.x;
    }

    return 0.5 * twice_area;
}

double MyPlanner::polygonArea(
    const ConvexPolygon& polygon) const
{
    return std::abs(polygonSignedArea(polygon));
}

MyPlanner::Point2D MyPlanner::polygonCentroid(
    const ConvexPolygon& polygon) const
{
    if (polygon.vertices.empty())
        return Point2D();

    const double signed_area = polygonSignedArea(polygon);

    if (std::abs(signed_area) < 1e-12)
    {
        double sum_x = 0.0;
        double sum_y = 0.0;

        for (std::size_t i = 0;
             i < polygon.vertices.size();
             ++i)
        {
            sum_x += polygon.vertices[i].x;
            sum_y += polygon.vertices[i].y;
        }

        return Point2D(
            sum_x / static_cast<double>(polygon.vertices.size()),
            sum_y / static_cast<double>(polygon.vertices.size()));
    }

    double centroid_x = 0.0;
    double centroid_y = 0.0;

    for (std::size_t i = 0;
         i < polygon.vertices.size();
         ++i)
    {
        const Point2D& first = polygon.vertices[i];
        const Point2D& second =
            polygon.vertices[(i + 1) % polygon.vertices.size()];

        const double cross =
            first.x * second.y
            - second.x * first.y;

        centroid_x += (first.x + second.x) * cross;
        centroid_y += (first.y + second.y) * cross;
    }

    const double factor = 1.0 / (6.0 * signed_area);

    return Point2D(
        centroid_x * factor,
        centroid_y * factor);
}

void MyPlanner::ensureCounterClockwise(
    ConvexPolygon& polygon) const
{
    if (polygonSignedArea(polygon) < 0.0)
    {
        std::reverse(
            polygon.vertices.begin(),
            polygon.vertices.end());
    }
}

bool MyPlanner::polygonIsUsable(
    const ConvexPolygon& polygon) const
{
    return polygon.vertices.size() >= 3
           && polygonArea(polygon) > 1e-10;
}

std::size_t MyPlanner::computeUsableCorridorPrefix(
    const std::vector<CorridorSegment>& corridors,
    double& usable_length,
    int& first_failure_index,
    std::string& first_failure_reason) const
{
    usable_length = 0.0;
    first_failure_index = -1;
    first_failure_reason.clear();

    std::size_t prefix_count = 0;

    for (std::size_t i = 0; i < corridors.size(); ++i)
    {
        const CorridorSegment& corridor = corridors[i];

        bool valid = true;
        std::string reason;

        if (!corridor.clipped_valid)
        {
            valid = false;
            reason = corridor.failure_reason.empty()
                         ? "硬障碍裁剪失败"
                         : corridor.failure_reason;
        }
        else if (!corridor.center_valid)
        {
            valid = false;
            reason = corridor.failure_reason.empty()
                         ? "车体中心走廊无效"
                         : corridor.failure_reason;
        }
        else if (i > 0 && !corridor.overlap_with_previous)
        {
            valid = false;
            reason = corridor.failure_reason.empty()
                         ? "与前一中心走廊交集不足"
                         : corridor.failure_reason;
        }

        if (!valid)
        {
            first_failure_index = static_cast<int>(i);
            first_failure_reason = reason;
            break;
        }

        ++prefix_count;
        // start/end.s are measured along the dense forward path.  This is the
        // correct coordinate for the reference optimizer and speed preview;
        // summing segment chords underestimates curved prefixes.
        usable_length = std::max(usable_length, corridor.end.s);
    }

    return prefix_count;
}

void MyPlanner::publishCorridorMarkers(
    const std::vector<CorridorSegment>& corridors,
    std::size_t usable_prefix_count,
    bool preferred_length_available)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_all;
    delete_all.header.frame_id = costmap_frame_;
    delete_all.header.stamp = ros::Time::now();
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    const ros::Time stamp = ros::Time::now();

    const std_msgs::ColorRGBA outline_color =
        preferred_length_available
            ? makeColor(0.05, 1.0, 0.20, 1.0)
            : makeColor(1.0, 0.72, 0.05, 1.0);

    usable_prefix_count =
        std::min(usable_prefix_count, corridors.size());

    for (std::size_t i = 0;
         i < usable_prefix_count;
         ++i)
    {
        const ConvexPolygon& polygon =
            corridors[i].center_polygon;

        if (!polygonIsUsable(polygon))
            continue;

        visualization_msgs::Marker marker;
        marker.header.frame_id = costmap_frame_;
        marker.header.stamp = stamp;
        marker.ns = "final_center_corridors";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.018;
        marker.color = outline_color;

        for (std::size_t point_index = 0;
             point_index <= polygon.vertices.size();
             ++point_index)
        {
            const Point2D& point =
                polygon.vertices[
                    point_index % polygon.vertices.size()];

            marker.points.push_back(
                toGeometryPoint(point.x, point.y, 0.045));
        }

        marker_array.markers.push_back(marker);
    }

    corridor_markers_pub_.publish(marker_array);
}

}  // namespace my_planner