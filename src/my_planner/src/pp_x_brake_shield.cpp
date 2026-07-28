#include "my_planner.h"

#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace my_planner
{

bool MyPlanner::copyCorridorReferenceSnapshot(
    CorridorReferenceSnapshot& snapshot) const
{
    std::lock_guard<std::mutex> lock(corridor_reference_mutex_);
    snapshot = corridor_reference_snapshot_;
    return snapshot.valid;
}

void MyPlanner::storeCorridorReferenceSnapshot(
    const CorridorReferenceSnapshot& snapshot)
{
    std::lock_guard<std::mutex> lock(corridor_reference_mutex_);
    corridor_reference_snapshot_ = snapshot;
    corridor_reference_snapshot_.snapshot_revision =
        ++corridor_reference_revision_;
}

void MyPlanner::invalidateCorridorReferenceSnapshot()
{
    std::lock_guard<std::mutex> lock(corridor_reference_mutex_);
    corridor_reference_snapshot_ = CorridorReferenceSnapshot();
    corridor_reference_snapshot_.snapshot_revision =
        ++corridor_reference_revision_;
}

double MyPlanner::signedPointMarginInConvexPolygon(
    const ConvexPolygon& polygon,
    const Point2D& point) const
{
    if (!polygonIsUsable(polygon))
        return -std::numeric_limits<double>::infinity();

    const double orientation =
        polygonSignedArea(polygon) >= 0.0 ? 1.0 : -1.0;
    double minimum_margin =
        std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < polygon.vertices.size(); ++i)
    {
        const Point2D& first = polygon.vertices[i];
        const Point2D& second =
            polygon.vertices[(i + 1) % polygon.vertices.size()];
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double length = std::hypot(dx, dy);
        if (length < 1e-9)
            continue;

        const double cross =
            dx * (point.y - first.y)
            - dy * (point.x - first.x);
        minimum_margin = std::min(
            minimum_margin,
            orientation * cross / length);
    }

    return minimum_margin;
}

std::vector<MyPlanner::Point2D> MyPlanner::buildFootprintSamplePoints(
    double x,
    double y,
    double yaw,
    double extra_margin) const
{
    const double half_length =
        corridor_robot_half_length_ + std::max(0.0, extra_margin);
    const double half_width =
        corridor_robot_half_width_ + std::max(0.0, extra_margin);

    const Point2D local_points[8] = {
        Point2D( half_length,  half_width),
        Point2D( half_length, -half_width),
        Point2D(-half_length, -half_width),
        Point2D(-half_length,  half_width),
        Point2D( half_length, 0.0),
        Point2D(0.0, -half_width),
        Point2D(-half_length, 0.0),
        Point2D(0.0,  half_width)};

    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    std::vector<Point2D> points;
    points.reserve(8);

    for (int i = 0; i < 8; ++i)
    {
        points.push_back(Point2D(
            x + cosine * local_points[i].x
              - sine * local_points[i].y,
            y + sine * local_points[i].x
              + cosine * local_points[i].y));
    }

    return points;
}

double MyPlanner::footprintMarginInCorridorUnion(
    double x,
    double y,
    double yaw,
    const CorridorReferenceSnapshot& snapshot,
    ConvexPolygon* footprint_output) const
{
    if (snapshot.corridors.empty())
        return -std::numeric_limits<double>::infinity();

    const std::vector<Point2D> samples =
        buildFootprintSamplePoints(
            x, y, yaw, pp_safety_footprint_margin_);

    if (footprint_output != NULL)
    {
        footprint_output->vertices.clear();
        footprint_output->vertices.push_back(samples[0]);
        footprint_output->vertices.push_back(samples[1]);
        footprint_output->vertices.push_back(samples[2]);
        footprint_output->vertices.push_back(samples[3]);
    }

    // 每个footprint采样点只需位于附近自由框的并集中。
    // 不再要求整车被单个短框完整包含。
    double footprint_margin =
        std::numeric_limits<double>::max();

    for (std::size_t point_index = 0;
         point_index < samples.size();
         ++point_index)
    {
        double point_margin =
            -std::numeric_limits<double>::infinity();

        for (std::size_t corridor_index = 0;
             corridor_index < snapshot.corridors.size();
             ++corridor_index)
        {
            const ConvexPolygon& polygon =
                snapshot.corridors[corridor_index].clipped_polygon;
            if (!polygonIsUsable(polygon))
                continue;

            point_margin = std::max(
                point_margin,
                signedPointMarginInConvexPolygon(
                    polygon, samples[point_index]));
        }

        footprint_margin = std::min(
            footprint_margin,
            point_margin);
    }

    return footprint_margin;
}

bool MyPlanner::evaluateCommandSafety(
    const geometry_msgs::Twist& command,
    const CorridorReferenceSnapshot& snapshot,
    const geometry_msgs::PoseStamped& robot_pose_costmap,
    SafetyCheckReport& report) const
{
    report = SafetyCheckReport();

    if (!snapshot.valid || snapshot.corridors.empty())
    {
        report.failure_reason = "安全框快照无效";
        return false;
    }

    report.horizon = pp_safety_prediction_horizon_;
    const int steps = std::max(
        1,
        static_cast<int>(std::ceil(
            report.horizon / pp_safety_prediction_dt_)));

    double x = robot_pose_costmap.pose.position.x;
    double y = robot_pose_costmap.pose.position.y;
    double yaw = tf::getYaw(robot_pose_costmap.pose.orientation);

    report.min_margin =
        std::numeric_limits<double>::max();
    report.predicted_path.reserve(
        static_cast<std::size_t>(steps));

    for (int step = 1; step <= steps; ++step)
    {
        const double cosine = std::cos(yaw);
        const double sine = std::sin(yaw);

        x += pp_safety_prediction_dt_
             * (cosine * command.linear.x
                - sine * command.linear.y);
        y += pp_safety_prediction_dt_
             * (sine * command.linear.x
                + cosine * command.linear.y);
        yaw = normalizeAngle(
            yaw
            + pp_safety_prediction_dt_
              * command.angular.z);

        ConvexPolygon footprint;
        const double margin = footprintMarginInCorridorUnion(
            x, y, yaw, snapshot, &footprint);

        report.min_margin = std::min(
            report.min_margin, margin);

        PathPoint2D predicted;
        predicted.x = x;
        predicted.y = y;
        predicted.s = step * pp_safety_prediction_dt_;
        predicted.yaw = yaw;
        predicted.clearance = margin;
        report.predicted_path.push_back(predicted);

        if (!std::isfinite(margin)
            || margin < pp_x_brake_margin_threshold_)
        {
            ++report.outside_count;
            if (report.first_unsafe_step < 0)
            {
                report.first_unsafe_step = step;
                report.first_unsafe_footprint = footprint;
            }
        }
    }

    report.valid = true;
    report.safe =
        report.outside_count < pp_x_brake_min_outside_steps_
        || report.min_margin >= pp_x_brake_margin_threshold_;
    return true;
}

bool MyPlanner::applyFixedXBrakeSafetyShield(
    const geometry_msgs::Twist& nominal_cmd,
    geometry_msgs::Twist& safe_cmd,
    SafetyCheckReport& nominal_report,
    SafetyCheckReport& selected_report,
    double& selected_x_scale,
    bool& unsafe_now)
{
    safe_cmd = nominal_cmd;
    selected_x_scale = 1.0;
    unsafe_now = false;
    nominal_report = SafetyCheckReport();
    selected_report = SafetyCheckReport();

    CorridorReferenceSnapshot snapshot;
    if (!copyCorridorReferenceSnapshot(snapshot))
    {
        safety_mode_ = SAFETY_DISABLED;
        pp_x_brake_unsafe_count_ = 0;
        pp_x_brake_safe_count_ = 0;
        return true;
    }

    const double age = std::max(
        0.0,
        (ros::Time::now() - snapshot.stamp).toSec());
    if (age > pp_safety_snapshot_max_age_)
    {
        safety_mode_ = SAFETY_DISABLED;
        pp_x_brake_unsafe_count_ = 0;
        pp_x_brake_safe_count_ = 0;
        return true;
    }

    geometry_msgs::PoseStamped robot_pose_costmap;
    if (!getRobotPoseInCostmap(robot_pose_costmap))
    {
        safety_mode_ = SAFETY_DISABLED;
        pp_x_brake_unsafe_count_ = 0;
        pp_x_brake_safe_count_ = 0;
        return true;
    }

    if (!evaluateCommandSafety(
            nominal_cmd,
            snapshot,
            robot_pose_costmap,
            nominal_report))
    {
        safety_mode_ = SAFETY_DISABLED;
        pp_x_brake_unsafe_count_ = 0;
        pp_x_brake_safe_count_ = 0;
        return true;
    }

    // 只有越界深度超过1cm，并且至少两个预测步越界，才认为本周期明确出框。
    unsafe_now =
        nominal_report.outside_count >= pp_x_brake_min_outside_steps_
        && nominal_report.min_margin < pp_x_brake_margin_threshold_;

    if (safety_mode_ == SAFETY_X_BRAKE)
    {
        if (unsafe_now)
        {
            pp_x_brake_safe_count_ = 0;
        }
        else
        {
            ++pp_x_brake_safe_count_;
            if (pp_x_brake_safe_count_ >= pp_x_brake_exit_cycles_)
            {
                safety_mode_ = SAFETY_SAFE;
                pp_x_brake_unsafe_count_ = 0;
                pp_x_brake_safe_count_ = 0;
            }
        }
    }
    else
    {
        if (unsafe_now)
        {
            ++pp_x_brake_unsafe_count_;
            if (pp_x_brake_unsafe_count_ >= pp_x_brake_enter_cycles_)
            {
                safety_mode_ = SAFETY_X_BRAKE;
                pp_x_brake_safe_count_ = 0;
            }
        }
        else
        {
            pp_x_brake_unsafe_count_ = 0;
            pp_x_brake_safe_count_ = 0;
            safety_mode_ = SAFETY_SAFE;
        }
    }

    if (safety_mode_ == SAFETY_X_BRAKE)
    {
        // 唯一控制修改：前向x速度固定乘0.5（参数被强制限制在[0.5,1.0]）。
        safe_cmd.linear.x =
            pp_x_brake_scale_ * nominal_cmd.linear.x;
        safe_cmd.linear.y = nominal_cmd.linear.y;
        safe_cmd.angular.z = nominal_cmd.angular.z;
        selected_x_scale = pp_x_brake_scale_;

        // 仅用于日志与RViz观察，不参与是否停车、是否重规划的决策。
        if (!evaluateCommandSafety(
                safe_cmd,
                snapshot,
                robot_pose_costmap,
                selected_report))
        {
            selected_report = nominal_report;
        }
    }
    else
    {
        selected_report = nominal_report;
    }

    // 无论预测结果如何，本模块始终返回true；真实障碍仍由原PP路径碰撞检查处理。
    return true;
}

const char* MyPlanner::safetyModeName() const
{
    switch (safety_mode_)
    {
    case SAFETY_SAFE:
        return "SAFE";
    case SAFETY_X_BRAKE:
        return "X_BRAKE";
    case SAFETY_DISABLED:
    default:
        return "DISABLED";
    }
}

void MyPlanner::publishSafetyDebug(
    const SafetyCheckReport& nominal_report,
    const SafetyCheckReport& selected_report) const
{
    const auto publish_path = [this](
        const SafetyCheckReport& report,
        const ros::Publisher& publisher)
    {
        nav_msgs::Path path;
        path.header.frame_id = costmap_frame_;
        path.header.stamp = ros::Time::now();

        for (std::size_t i = 0;
             i < report.predicted_path.size();
             ++i)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = report.predicted_path[i].x;
            pose.pose.position.y = report.predicted_path[i].y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                report.predicted_path[i].yaw);
            path.poses.push_back(pose);
        }

        publisher.publish(path);
    };

    publish_path(nominal_report, pp_raw_prediction_pub_);
    publish_path(selected_report, pp_safe_prediction_pub_);

    visualization_msgs::Marker marker;
    marker.header.frame_id = costmap_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pp_x_brake_unsafe_footprint";
    marker.id = 0;

    const ConvexPolygon* footprint = NULL;
    if (!nominal_report.first_unsafe_footprint.vertices.empty())
        footprint = &nominal_report.first_unsafe_footprint;
    else if (!selected_report.first_unsafe_footprint.vertices.empty())
        footprint = &selected_report.first_unsafe_footprint;

    if (footprint == NULL)
    {
        marker.action = visualization_msgs::Marker::DELETE;
        pp_unsafe_footprint_pub_.publish(marker);
        return;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.012;
    marker.color.r = 1.0;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.color.a = 0.95;

    for (std::size_t i = 0;
         i < footprint->vertices.size();
         ++i)
    {
        geometry_msgs::Point point;
        point.x = footprint->vertices[i].x;
        point.y = footprint->vertices[i].y;
        point.z = 0.08;
        marker.points.push_back(point);
    }

    if (!footprint->vertices.empty())
    {
        geometry_msgs::Point point;
        point.x = footprint->vertices.front().x;
        point.y = footprint->vertices.front().y;
        point.z = 0.08;
        marker.points.push_back(point);
    }

    pp_unsafe_footprint_pub_.publish(marker);
}


void MyPlanner::publishReferencePath(
    const std::vector<PathPoint2D>& path,
    const ros::Publisher& publisher,
    double z) const
{
    nav_msgs::Path message;
    message.header.frame_id = costmap_frame_;
    message.header.stamp = ros::Time::now();
    message.poses.reserve(path.size());

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = message.header;
        pose.pose.position.x = path[i].x;
        pose.pose.position.y = path[i].y;
        pose.pose.position.z = z;
        pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(path[i].yaw);
        message.poses.push_back(pose);
    }

    publisher.publish(message);
}

}  // namespace my_planner