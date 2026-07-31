// 交付构建标识：MYPLANNER_MPC_C3_2_MEASURED_STATE_DELAY_PROGRESS_20260731
#include "my_planner.h"

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <tf/tf.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace my_planner
{

namespace
{

typedef std::chrono::steady_clock MpcClock;

bool finiteTwist(const geometry_msgs::Twist& command)
{
    return std::isfinite(command.linear.x)
           && std::isfinite(command.linear.y)
           && std::isfinite(command.angular.z);
}

const char* osqpStatusName(OsqpEigen::Status status)
{
    switch (status)
    {
        case OsqpEigen::Status::Solved:
            return "SOLVED";
        case OsqpEigen::Status::SolvedInaccurate:
            return "SOLVED_INACCURATE";
        case OsqpEigen::Status::MaxIterReached:
            return "MAX_ITER";
        case OsqpEigen::Status::PrimalInfeasible:
            return "PRIMAL_INFEASIBLE";
        case OsqpEigen::Status::PrimalInfeasibleInaccurate:
            return "PRIMAL_INFEASIBLE_INACCURATE";
        case OsqpEigen::Status::DualInfeasible:
            return "DUAL_INFEASIBLE";
        case OsqpEigen::Status::DualInfeasibleInaccurate:
            return "DUAL_INFEASIBLE_INACCURATE";
        case OsqpEigen::Status::Sigint:
            return "SIGINT";
        case OsqpEigen::Status::NonCvx:
            return "NON_CONVEX";
        case OsqpEigen::Status::Unsolved:
        default:
            return "UNSOLVED";
    }
}

double medianValue(std::vector<double> values)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 1)
        return values[middle];
    return 0.5 * (values[middle - 1] + values[middle]);
}

double moveToward(double current, double target, double positive_step, double negative_step)
{
    if (target >= current)
        return std::min(target, current + std::max(0.0, positive_step));
    return std::max(target, current - std::max(0.0, negative_step));
}

double interpolateLinear(double x0, double x1, double y0, double y1, double x)
{
    if (x1 <= x0 + 1e-12)
        return y1;
    const double ratio = std::max(0.0, std::min(1.0, (x - x0) / (x1 - x0)));
    return y0 + ratio * (y1 - y0);
}

}  // namespace

void MyPlanner::resetMpcState()
{
    mpc_consecutive_failures_ = 0;
    mpc_locked_to_pp_ = false;
}

void MyPlanner::computePurePursuitCommand(
    const geometry_msgs::PoseStamped& target_pose,
    geometry_msgs::Twist& desired_cmd,
    double& lateral_deviation)
{
    lateral_deviation = computeLateralDeviation(target_pose);
    desired_cmd = geometry_msgs::Twist();
    desired_cmd.linear.x =
        target_pose.pose.position.x * path_linear_x_gain_;
    desired_cmd.linear.y =
        lateral_deviation * path_linear_y_gain_;
    desired_cmd.angular.z =
        target_pose.pose.position.y * path_angular_y_gain_;
}

bool MyPlanner::buildMpcReferenceTrajectory(
    double control_dt,
    std::vector<MpcReferencePoint>& reference,
    MpcSolveReport& report)
{
    reference.clear();
    report.reference_path.clear();

    if (global_plan_.size() < 2)
    {
        report.status = "REFERENCE_FAILED";
        report.failure_reason = "global_plan点数不足";
        return false;
    }

    const int path_size = static_cast<int>(global_plan_.size());
    const int search_start = std::max(
        0, target_index_ - mpc_reference_search_behind_points_);
    const int search_end = std::min(
        path_size - 1,
        target_index_ + mpc_reference_search_ahead_points_);

    if (search_end <= search_start)
    {
        report.status = "REFERENCE_FAILED";
        report.failure_reason = "MPC参考搜索区间为空";
        return false;
    }

    std::string plan_frame =
        global_plan_[static_cast<std::size_t>(search_start)].header.frame_id;
    if (plan_frame.empty())
        plan_frame = costmap_frame_;

    tf::StampedTransform plan_to_base;
    try
    {
        if (plan_frame == base_frame_)
        {
            plan_to_base.setIdentity();
        }
        else
        {
            tf_listener_->lookupTransform(
                base_frame_, plan_frame, ros::Time(0), plan_to_base);
        }
    }
    catch (const tf::TransformException& ex)
    {
        report.status = "REFERENCE_TF_FAILED";
        report.failure_reason =
            std::string("MPC参考单次TF查询失败：") + ex.what();
        return false;
    }

    // 1. 转到base_link并删除空间重复点。只删点，不平滑XY，避免窄路内切。
    std::vector<PathPoint2D> local_path;
    local_path.reserve(static_cast<std::size_t>(search_end - search_start + 1));
    double cumulative_s = 0.0;

    for (int index = search_start; index <= search_end; ++index)
    {
        const geometry_msgs::PoseStamped& source =
            global_plan_[static_cast<std::size_t>(index)];
        const std::string source_frame =
            source.header.frame_id.empty() ? plan_frame : source.header.frame_id;

        PathPoint2D point;
        if (source_frame == plan_frame)
        {
            const tf::Vector3 input(
                source.pose.position.x,
                source.pose.position.y,
                0.0);
            const tf::Vector3 output = plan_to_base * input;
            point.x = output.x();
            point.y = output.y();
        }
        else
        {
            geometry_msgs::PoseStamped point_base;
            if (!transformPose(base_frame_, source, point_base))
                continue;
            point.x = point_base.pose.position.x;
            point.y = point_base.pose.position.y;
        }

        point.source_index = index;
        if (!local_path.empty())
        {
            const double segment_length = std::hypot(
                point.x - local_path.back().x,
                point.y - local_path.back().y);
            if (segment_length < c2_duplicate_point_distance_)
                continue;
            cumulative_s += segment_length;
        }
        point.s = cumulative_s;
        local_path.push_back(point);
    }

    if (local_path.size() < 3)
    {
        report.status = "REFERENCE_FAILED";
        report.failure_reason = "MPC参考去重后点数不足";
        return false;
    }

    // 2. 将机器人原点投影到局部路径。
    double closest_distance = std::numeric_limits<double>::max();
    double projection_s = local_path.front().s;
    std::size_t closest_segment = 0;

    for (std::size_t i = 0; i + 1 < local_path.size(); ++i)
    {
        const PathPoint2D& first = local_path[i];
        const PathPoint2D& second = local_path[i + 1];
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared < 1e-12)
            continue;

        const double ratio = clampValue(
            -(first.x * dx + first.y * dy) / length_squared,
            0.0, 1.0);
        const double px = first.x + ratio * dx;
        const double py = first.y + ratio * dy;
        const double distance = std::hypot(px, py);

        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest_segment = i;
            projection_s = first.s + ratio * std::sqrt(length_squared);
        }
    }

    target_index_ = std::max(
        target_index_, local_path[closest_segment].source_index);

    const double remaining_length = std::max(
        0.0, local_path.back().s - projection_s);
    report.projection_s = projection_s;
    report.remaining_length = remaining_length;

    if (remaining_length < mpc_min_reference_length_)
    {
        std::ostringstream stream;
        stream << "MPC参考剩余长度不足："
               << remaining_length << " < " << mpc_min_reference_length_;
        report.status = "REFERENCE_TOO_SHORT";
        report.failure_reason = stream.str();
        return false;
    }

    const auto interpolatePath =
        [&](const std::vector<PathPoint2D>& path,
            double query_s,
            PathPoint2D& output) -> bool
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

            const auto upper_it = std::lower_bound(
                path.begin(), path.end(), query_s,
                [](const PathPoint2D& point, double value)
                {
                    return point.s < value;
                });
            if (upper_it == path.begin() || upper_it == path.end())
                return false;

            const PathPoint2D& second = *upper_it;
            const PathPoint2D& first = *(upper_it - 1);
            const double ds = second.s - first.s;
            if (ds < 1e-12)
            {
                output = first;
                output.s = query_s;
                return true;
            }

            const double ratio = (query_s - first.s) / ds;
            output.x = first.x + ratio * (second.x - first.x);
            output.y = first.y + ratio * (second.y - first.y);
            output.s = query_s;
            output.yaw = first.yaw + ratio * (second.yaw - first.yaw);
            output.curvature_track = first.curvature_track
                + ratio * (second.curvature_track - first.curvature_track);
            output.curvature_speed = first.curvature_speed
                + ratio * (second.curvature_speed - first.curvature_speed);
            output.speed_limit = first.speed_limit
                + ratio * (second.speed_limit - first.speed_limit);
            output.source_index = first.source_index;
            return true;
        };

    // 3. 按累计弧长做2cm线性等距重采样，不使用三次样条。
    std::vector<PathPoint2D> resampled;
    const double total_length = local_path.back().s - local_path.front().s;
    const int expected_count =
        static_cast<int>(std::ceil(total_length / c2_resample_distance_)) + 2;
    resampled.reserve(static_cast<std::size_t>(std::max(3, expected_count)));

    for (double query_s = local_path.front().s;
         query_s < local_path.back().s;
         query_s += c2_resample_distance_)
    {
        PathPoint2D point;
        if (interpolatePath(local_path, query_s, point))
            resampled.push_back(point);
    }
    PathPoint2D last_point;
    if (interpolatePath(local_path, local_path.back().s, last_point))
    {
        if (resampled.empty()
            || std::abs(last_point.s - resampled.back().s) > 1e-6)
        {
            resampled.push_back(last_point);
        }
    }

    if (resampled.size() < 5)
    {
        report.status = "RESAMPLE_FAILED";
        report.failure_reason = "C3等距重采样后点数不足";
        return false;
    }
    report.resampled_points = static_cast<int>(resampled.size());

    // 4. 先用固定物理距离的对称弦方向生成连续yaw。
    // 不直接使用相邻路径点方向，避免栅格锯齿被放大。
    double previous_yaw = 0.0;
    bool has_previous_yaw = false;

    for (std::size_t i = 0; i < resampled.size(); ++i)
    {
        PathPoint2D before;
        PathPoint2D after;
        const double before_s = std::max(
            resampled.front().s,
            resampled[i].s - c2_tracking_curvature_distance_);
        const double after_s = std::min(
            resampled.back().s,
            resampled[i].s + c2_tracking_curvature_distance_);

        if (!interpolatePath(resampled, before_s, before)
            || !interpolatePath(resampled, after_s, after))
        {
            report.status = "YAW_INTERPOLATION_FAILED";
            report.failure_reason = "C3固定距离切线取点失败";
            return false;
        }

        const double tangent_x = after.x - before.x;
        const double tangent_y = after.y - before.y;
        double yaw = has_previous_yaw ? previous_yaw : 0.0;
        if (std::hypot(tangent_x, tangent_y) > 1e-7)
            yaw = std::atan2(tangent_y, tangent_x);
        if (has_previous_yaw)
            yaw = previous_yaw + normalizeAngle(yaw - previous_yaw);
        else
        {
            yaw = normalizeAngle(yaw);
            has_previous_yaw = true;
        }

        resampled[i].yaw = yaw;
        previous_yaw = yaw;
    }

    // 5. 双曲率通道均由“平滑yaw对弧长求导”得到。
    // κ_track使用±8cm，κ_speed使用±12cm；比三点圆直接使用中心点更抗锯齿。
    std::vector<double> raw_speed_curvature(resampled.size(), 0.0);
    for (std::size_t i = 0; i < resampled.size(); ++i)
    {
        PathPoint2D track_before;
        PathPoint2D track_after;
        PathPoint2D speed_before;
        PathPoint2D speed_after;

        const double track_before_s = std::max(
            resampled.front().s,
            resampled[i].s - c2_tracking_curvature_distance_);
        const double track_after_s = std::min(
            resampled.back().s,
            resampled[i].s + c2_tracking_curvature_distance_);
        const double speed_before_s = std::max(
            resampled.front().s,
            resampled[i].s - c2_speed_curvature_distance_);
        const double speed_after_s = std::min(
            resampled.back().s,
            resampled[i].s + c2_speed_curvature_distance_);

        if (!interpolatePath(resampled, track_before_s, track_before)
            || !interpolatePath(resampled, track_after_s, track_after)
            || !interpolatePath(resampled, speed_before_s, speed_before)
            || !interpolatePath(resampled, speed_after_s, speed_after))
        {
            report.status = "CURVATURE_INTERPOLATION_FAILED";
            report.failure_reason = "C3曲率固定距离取点失败";
            return false;
        }

        const double track_ds = std::max(
            1e-6, track_after_s - track_before_s);
        const double speed_ds = std::max(
            1e-6, speed_after_s - speed_before_s);

        resampled[i].curvature_track =
            normalizeAngle(track_after.yaw - track_before.yaw) / track_ds;
        raw_speed_curvature[i] =
            normalizeAngle(speed_after.yaw - speed_before.yaw) / speed_ds;
    }

    // 6. 只对限速曲率再做中值滤波，不改变路径坐标和MPC跟踪曲率。
    const int median_half = c2_curvature_median_window_ / 2;
    for (std::size_t i = 0; i < resampled.size(); ++i)
    {
        std::vector<double> window;
        const int begin = std::max(0, static_cast<int>(i) - median_half);
        const int end = std::min(
            static_cast<int>(resampled.size()) - 1,
            static_cast<int>(i) + median_half);
        window.reserve(static_cast<std::size_t>(end - begin + 1));
        for (int j = begin; j <= end; ++j)
            window.push_back(raw_speed_curvature[static_cast<std::size_t>(j)]);
        resampled[i].curvature_speed = medianValue(window);
    }

    // 7. 每个空间点向前预览0.45m、向后保留0.12m，取过滤后的最大绝对曲率。
    for (std::size_t i = 0; i < resampled.size(); ++i)
    {
        const double lower_s = resampled[i].s - c2_hold_speed_after_curve_;
        const double upper_s = resampled[i].s + c2_curvature_preview_distance_;
        double max_abs_curvature = 0.0;

        for (std::size_t j = 0; j < resampled.size(); ++j)
        {
            if (resampled[j].s < lower_s)
                continue;
            if (resampled[j].s > upper_s)
                break;
            max_abs_curvature = std::max(
                max_abs_curvature,
                std::abs(resampled[j].curvature_speed));
        }

        double speed_limit = c2_max_reference_speed_;
        if (max_abs_curvature > 1e-5)
        {
            speed_limit = std::sqrt(
                c2_curve_lateral_acc_limit_ / max_abs_curvature);
        }
        resampled[i].speed_limit = clampValue(
            speed_limit,
            c2_min_curve_speed_,
            c2_max_reference_speed_);
    }

    // 8. 空间域双向传播：弯前允许较快减速，出弯较慢恢复速度。
    for (int i = static_cast<int>(resampled.size()) - 2; i >= 0; --i)
    {
        const double ds = std::max(
            1e-6,
            resampled[static_cast<std::size_t>(i + 1)].s
                - resampled[static_cast<std::size_t>(i)].s);
        const double next_speed =
            resampled[static_cast<std::size_t>(i + 1)].speed_limit;
        const double reachable = std::sqrt(std::max(
            0.0,
            next_speed * next_speed
                + 2.0 * c2_reference_deceleration_ * ds));
        resampled[static_cast<std::size_t>(i)].speed_limit = std::min(
            resampled[static_cast<std::size_t>(i)].speed_limit,
            reachable);
    }

    for (std::size_t i = 1; i < resampled.size(); ++i)
    {
        const double ds = std::max(
            1e-6,
            resampled[i].s - resampled[i - 1].s);
        const double previous_speed = resampled[i - 1].speed_limit;
        const double reachable = std::sqrt(std::max(
            0.0,
            previous_speed * previous_speed
                + 2.0 * c2_reference_acceleration_ * ds));
        resampled[i].speed_limit = std::min(
            resampled[i].speed_limit,
            reachable);
    }

    PathPoint2D projection_point;
    if (!interpolatePath(resampled, projection_s, projection_point))
    {
        report.status = "PROJECTION_LOOKUP_FAILED";
        report.failure_reason = "C3无法在等距路径上查询投影点";
        return false;
    }

    report.first_tracking_curvature = projection_point.curvature_track;
    report.first_speed_limit = projection_point.speed_limit;
    const double preview_upper = projection_s + c2_curvature_preview_distance_;
    for (std::size_t i = 0; i < resampled.size(); ++i)
    {
        if (resampled[i].s < projection_s - c2_hold_speed_after_curve_)
            continue;
        if (resampled[i].s > preview_upper)
            break;
        report.preview_abs_curvature = std::max(
            report.preview_abs_curvature,
            std::abs(resampled[i].curvature_speed));
    }

    // 9. C3.2：使用里程计实测速度作为当前状态，并按已发布命令重放输入延迟。
    // 上一控制序列只代表过去发出的命令，不再被当作真实车辆状态。
    const ros::Time now = ros::Time::now();
    const MeasuredBodyState measured = getMeasuredBodyState(now);
    const DelayCompensatedState delay_state =
        predictStateThroughInputDelay(measured, now);

    report.measured_vx = measured.vx;
    report.measured_vy = measured.vy;
    report.measured_omega = measured.omega;
    report.odom_age = measured.age;
    report.using_odometry = measured.valid;
    report.delay_x = delay_state.x;
    report.delay_y = delay_state.y;
    report.delay_yaw = delay_state.yaw;
    report.delay_omega = delay_state.omega;

    // Autoware/MPCC常用的空间参考方式：将延迟期间已经不可避免的前进量
    // 投影到路径切向，MPC从延迟结束时的预测位置开始跟踪。
    const double delay_progress = std::max(
        0.0,
        delay_state.x * std::cos(projection_point.yaw)
        + delay_state.y * std::sin(projection_point.yaw));
    const double delayed_projection_s = std::min(
        resampled.back().s,
        projection_s + delay_progress);

    const int state_count = mpc_horizon_steps_ + 1;
    reference.resize(static_cast<std::size_t>(state_count));
    report.reference_path.resize(static_cast<std::size_t>(state_count));

    const double first_dt = clampValue(control_dt, 0.01, 0.20);
    const double previous_path_speed = clampValue(
        std::hypot(measured.vx, measured.vy),
        0.0, c2_max_reference_speed_);

    PathPoint2D delayed_projection_point;
    if (!interpolatePath(
            resampled, delayed_projection_s, delayed_projection_point))
    {
        report.status = "DELAY_PROJECTION_LOOKUP_FAILED";
        report.failure_reason = "C3.2无法查询延迟补偿后的路径投影点";
        return false;
    }

    const double delayed_remaining_length = std::max(
        0.0, resampled.back().s - delayed_projection_s);
    if (delayed_remaining_length < mpc_min_reference_length_)
    {
        report.status = "REFERENCE_TOO_SHORT_AFTER_DELAY";
        report.failure_reason =
            "输入延迟补偿后MPC参考剩余长度不足";
        return false;
    }
    report.first_speed_limit = delayed_projection_point.speed_limit;

    double reference_path_speed = moveToward(
        previous_path_speed,
        delayed_projection_point.speed_limit,
        c2_reference_acceleration_ * first_dt,
        c2_reference_deceleration_ * first_dt);

    double previous_beta = 0.0;
    if (previous_path_speed > 0.03)
    {
        previous_beta = clampValue(
            std::atan2(measured.vy, measured.vx),
            -c3_beta_max_low_speed_, c3_beta_max_low_speed_);
    }

    std::vector<double> desired_body_yaw(
        static_cast<std::size_t>(state_count), delay_state.yaw);
    double query_s = delayed_projection_s;

    for (int step = 0; step < state_count; ++step)
    {
        PathPoint2D point;
        PathPoint2D preview_point;
        if (!interpolatePath(resampled, query_s, point)
            || !interpolatePath(
                resampled,
                std::min(
                    resampled.back().s,
                    query_s + c3_yaw_preview_distance_),
                preview_point))
        {
            report.status = "REFERENCE_INTERPOLATION_FAILED";
            report.failure_reason = "C3.2时间参考或车头预瞄插值失败";
            reference.clear();
            report.reference_path.clear();
            return false;
        }

        if (step > 0)
        {
            reference_path_speed = moveToward(
                reference_path_speed,
                point.speed_limit,
                c2_reference_acceleration_ * mpc_dt_,
                c2_reference_deceleration_ * mpc_dt_);
        }
        reference_path_speed = clampValue(
            reference_path_speed, 0.0, c2_max_reference_speed_);

        double beta_limit = c3_beta_max_mid_speed_;
        const double mid_speed = 0.5 * (
            c3_beta_low_speed_threshold_ + c3_beta_high_speed_threshold_);
        if (reference_path_speed <= c3_beta_low_speed_threshold_)
        {
            beta_limit = c3_beta_max_low_speed_;
        }
        else if (reference_path_speed < mid_speed)
        {
            beta_limit = interpolateLinear(
                c3_beta_low_speed_threshold_, mid_speed,
                c3_beta_max_low_speed_, c3_beta_max_mid_speed_,
                reference_path_speed);
        }
        else if (reference_path_speed < c3_beta_high_speed_threshold_)
        {
            beta_limit = interpolateLinear(
                mid_speed, c3_beta_high_speed_threshold_,
                c3_beta_max_mid_speed_, c3_beta_max_high_speed_,
                reference_path_speed);
        }
        else
        {
            beta_limit = c3_beta_max_high_speed_;
        }

        const double curvature_strength_value = std::max(
            std::abs(point.curvature_track),
            std::abs(preview_point.curvature_track));
        const double curve_strength = clampValue(
            (curvature_strength_value - c3_yaw_preview_curvature_deadband_)
                / (c3_yaw_preview_full_curvature_
                   - c3_yaw_preview_curvature_deadband_),
            0.0, 1.0);

        double desired_beta = 0.0;
        if (c3_enable_active_drift_)
        {
            const double preview_turn = normalizeAngle(
                preview_point.yaw - point.yaw);
            const double yaw_lead =
                c3_yaw_preview_gain_ * curve_strength * preview_turn;
            desired_beta = clampValue(
                -yaw_lead, -beta_limit, beta_limit);
        }

        const double beta_dt = step == 0 ? first_dt : mpc_dt_;
        const double max_beta_delta = c3_beta_rate_limit_ * beta_dt;
        desired_beta = previous_beta + clampValue(
            normalizeAngle(desired_beta - previous_beta),
            -max_beta_delta, max_beta_delta);
        desired_beta = clampValue(
            desired_beta, -beta_limit, beta_limit);
        previous_beta = desired_beta;

        MpcReferencePoint& ref = reference[static_cast<std::size_t>(step)];
        ref.x = point.x;
        ref.y = point.y;
        ref.motion_yaw = point.yaw;
        ref.curvature = point.curvature_track;
        ref.speed_limit = point.speed_limit;
        ref.path_speed = reference_path_speed;
        ref.planned_beta = desired_beta;
        ref.drift_beta = desired_beta;
        ref.beta_limit = beta_limit;
        ref.curve_strength = curve_strength;
        report.max_abs_planned_beta = std::max(
            report.max_abs_planned_beta, std::abs(desired_beta));
        desired_body_yaw[static_cast<std::size_t>(step)] =
            point.yaw - desired_beta;

        if (step < mpc_horizon_steps_)
        {
            query_s = std::min(
                resampled.back().s,
                query_s + reference_path_speed * mpc_dt_);
        }
    }

    // 10. 角速度参考采用成熟控制器常见的两部分：
    // 车头姿态预瞄差分 + v*kappa曲率前馈，并按建立/制动/反向分别限速。
    double body_previous_yaw = delay_state.yaw;
    double body_previous_omega = clampValue(
        delay_state.omega,
        -c3_reference_omega_limit_, c3_reference_omega_limit_);
    report.initial_omega_seed = body_previous_omega;
    reference.front().yaw = body_previous_yaw;
    reference.front().omega = body_previous_omega;

    for (int step = 1; step < state_count; ++step)
    {
        const MpcReferencePoint& target_ref =
            reference[static_cast<std::size_t>(step)];
        const double target_yaw = body_previous_yaw + normalizeAngle(
            desired_body_yaw[static_cast<std::size_t>(step)]
            - body_previous_yaw);
        const double omega_geometry =
            (target_yaw - body_previous_yaw) / mpc_dt_;
        const double omega_curvature =
            target_ref.path_speed * target_ref.curvature;
        double omega_target =
            (1.0 - c3_omega_curvature_feedforward_gain_) * omega_geometry
            + c3_omega_curvature_feedforward_gain_ * omega_curvature;
        omega_target = clampValue(
            omega_target,
            -c3_reference_omega_limit_, c3_reference_omega_limit_);

        double omega_rate = c3_reference_omega_accel_rate_;
        if (omega_target * body_previous_omega < -1.0e-4)
        {
            omega_rate = c3_reference_omega_reverse_rate_;
        }
        else if (std::abs(omega_target) < std::abs(body_previous_omega))
        {
            omega_rate = c3_reference_omega_decel_rate_;
        }

        const double max_omega_delta = omega_rate * mpc_dt_;
        const double omega = body_previous_omega + clampValue(
            omega_target - body_previous_omega,
            -max_omega_delta, max_omega_delta);

        MpcReferencePoint& ref = reference[static_cast<std::size_t>(step)];
        ref.omega = clampValue(
            omega, -c3_reference_omega_limit_, c3_reference_omega_limit_);
        ref.yaw = normalizeAngle(
            body_previous_yaw + ref.omega * mpc_dt_);
        body_previous_yaw = ref.yaw;
        body_previous_omega = ref.omega;
    }

    // 一阶执行器模型的逆模型，用于生成omega_cmd参考。
    const double angular_a = std::exp(
        -mpc_dt_ / std::max(0.01, c3_angular_response_tau_));
    const double angular_b = std::max(1.0e-4, 1.0 - angular_a);
    for (int step = 0; step < state_count; ++step)
    {
        MpcReferencePoint& ref = reference[static_cast<std::size_t>(step)];
        if (step + 1 < state_count)
        {
            const double next_omega =
                reference[static_cast<std::size_t>(step + 1)].omega;
            ref.omega_cmd = clampValue(
                (next_omega - angular_a * ref.omega) / angular_b,
                mpc_min_omega_, mpc_max_omega_);
        }
        else if (step > 0)
        {
            ref.omega_cmd =
                reference[static_cast<std::size_t>(step - 1)].omega_cmd;
        }
        else
        {
            ref.omega_cmd = ref.omega;
        }
    }

    // 11. 用可实现车头姿态重新计算有效beta，并保留原C3.1可行性保护。
    for (int step = 0; step < state_count; ++step)
    {
        MpcReferencePoint& ref = reference[static_cast<std::size_t>(step)];
        const double effective_beta = normalizeAngle(
            ref.motion_yaw - ref.yaw);
        ref.drift_beta = effective_beta;
        report.max_abs_drift_beta = std::max(
            report.max_abs_drift_beta, std::abs(effective_beta));

        bool feasibility_speed_reduced = false;
        const double beta_limit = std::max(1.0e-3, ref.beta_limit);
        const double abs_beta = std::abs(effective_beta);
        if (abs_beta > beta_limit)
        {
            const double beta_speed_scale = clampValue(
                beta_limit / abs_beta, 0.55, 1.0);
            ref.path_speed *= beta_speed_scale;
            feasibility_speed_reduced = true;
            ++report.beta_speed_guard_steps;
        }

        const double abs_sin_beta = std::abs(std::sin(effective_beta));
        if (abs_sin_beta > 1.0e-3)
        {
            const double max_abs_vy = std::max(
                0.0,
                std::min(std::abs(mpc_min_vy_), std::abs(mpc_max_vy_)));
            if (max_abs_vy > 1.0e-3)
            {
                const double vy_feasible_speed =
                    0.98 * max_abs_vy / abs_sin_beta;
                const double previous_feasible_speed = ref.path_speed;
                ref.path_speed = std::min(
                    ref.path_speed, vy_feasible_speed);
                if (ref.path_speed + 1.0e-9 < previous_feasible_speed)
                    feasibility_speed_reduced = true;
            }
        }

        if (feasibility_speed_reduced)
        {
            // 可行性保护触发时，MPCC进度奖励也不能绕过保护速度。
            ref.speed_limit = std::min(ref.speed_limit, ref.path_speed);
        }

        ref.vx = ref.path_speed * std::cos(effective_beta);
        ref.vy = ref.path_speed * std::sin(effective_beta);

        PathPoint2D& debug_point =
            report.reference_path[static_cast<std::size_t>(step)];
        debug_point.x = ref.x;
        debug_point.y = ref.y;
        debug_point.s = step == 0 ? 0.0
            : report.reference_path[static_cast<std::size_t>(step - 1)].s
                + reference[static_cast<std::size_t>(step - 1)].path_speed
                  * mpc_dt_;
        debug_point.yaw = ref.yaw;
        debug_point.curvature_track = ref.curvature;
        debug_point.curvature_speed = ref.curvature;
        debug_point.speed_limit = ref.path_speed;
    }

    report.projection_s = delayed_projection_s;
    report.remaining_length = std::max(
        0.0, resampled.back().s - delayed_projection_s);
    report.preview_distance = std::max(
        0.0, query_s - delayed_projection_s);
    report.first_reference = reference.front();
    report.first_path_speed = reference.front().path_speed;
    report.first_drift_beta = reference.front().drift_beta;
    report.first_curve_strength = reference.front().curve_strength;
    return true;

}

bool MyPlanner::computeMpcCommand(
    double control_dt,
    geometry_msgs::Twist& desired_cmd,
    MpcSolveReport& report)
{
    const MpcClock::time_point total_begin = MpcClock::now();
    std::vector<MpcReferencePoint> reference;

    if (!buildMpcReferenceTrajectory(control_dt, reference, report))
    {
        report.total_ms = std::chrono::duration<double, std::milli>(
            MpcClock::now() - total_begin).count();
        publishMpcDebugPaths(report);
        return false;
    }

    const bool solved = solveLtvMpcQp(
        reference, control_dt, desired_cmd, report);
    report.total_ms = std::chrono::duration<double, std::milli>(
        MpcClock::now() - total_begin).count();
    publishMpcDebugPaths(report);

    if (!solved)
        return false;

    if (report.total_ms > mpc_max_total_time_ms_)
    {
        std::ostringstream stream;
        stream << "MPC总耗时超限："
               << report.total_ms << "ms > " << mpc_max_total_time_ms_ << "ms";
        report.status = "TOTAL_TIME_LIMIT";
        report.failure_reason = stream.str();
        return false;
    }

    if (!finiteTwist(desired_cmd))
    {
        report.status = "NON_FINITE_OUTPUT";
        report.failure_reason = "MPC输出包含NaN或Inf";
        return false;
    }

    report.success = true;
    return true;
}

bool MyPlanner::solveLtvMpcQp(
    const std::vector<MpcReferencePoint>& reference,
    double control_dt,
    geometry_msgs::Twist& desired_cmd,
    MpcSolveReport& report)
{
    desired_cmd = geometry_msgs::Twist();

    const int horizon = mpc_horizon_steps_;
    const int nx = 4;  // x, y, yaw, measured/predicted actual omega
    const int nu = 3;  // vx_cmd, vy_cmd, omega_cmd
    if (static_cast<int>(reference.size()) != horizon + 1)
    {
        report.status = "REFERENCE_SIZE_ERROR";
        report.failure_reason = "MPC参考状态数与预测步数不一致";
        return false;
    }

    const int state_count = nx * (horizon + 1);
    const int control_count = nu * horizon;
    const int variable_count = state_count + control_count;
    const int equality_count = nx * (horizon + 1);
    const int input_count = nu * horizon;
    const int rate_count = nu * horizon;
    const int velocity_polygon_count =
        mpc_velocity_polygon_sides_ * horizon;
    const int constraint_count =
        equality_count + input_count + rate_count
        + velocity_polygon_count;

    const auto stateIndex = [nx](int step, int component)
    {
        return nx * step + component;
    };
    const auto controlIndex = [state_count, nu](int step, int component)
    {
        return state_count + nu * step + component;
    };

    std::vector<Eigen::Triplet<double> > hessian_triplets;
    hessian_triplets.reserve(
        static_cast<std::size_t>(76 * horizon + 32));
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variable_count);
    const double regularization = 1.0e-7;

    for (int step = 0; step <= horizon; ++step)
    {
        const MpcReferencePoint& point =
            reference[static_cast<std::size_t>(step)];
        const bool terminal = step == horizon;
        const double position_scale = terminal
            ? mpc_terminal_position_weight_scale_ : 1.0;
        const double yaw_scale = terminal
            ? mpc_terminal_yaw_weight_scale_ : 1.0;
        const double omega_scale = terminal
            ? mpc_terminal_omega_weight_scale_ : 1.0;

        const double c = std::cos(point.motion_yaw);
        const double ss = std::sin(point.motion_yaw);
        const double q_long = position_scale * mpc_weight_longitudinal_;
        const double q_lat = position_scale * mpc_weight_lateral_;
        const double yaw_weight =
            mpc_weight_yaw_straight_
            + point.curve_strength
              * (mpc_weight_yaw_curve_ - mpc_weight_yaw_straight_);
        const double omega_state_weight =
            mpc_weight_omega_state_straight_
            + point.curve_strength
              * (mpc_weight_omega_state_curve_
                 - mpc_weight_omega_state_straight_);

        const double q_yaw = yaw_scale * yaw_weight;
        const double q_omega = omega_scale * omega_state_weight;
        const double q_xx = q_long * c * c + q_lat * ss * ss;
        const double q_yy = q_long * ss * ss + q_lat * c * c;
        const double q_xy = (q_long - q_lat) * c * ss;

        const int ix = stateIndex(step, 0);
        const int iy = stateIndex(step, 1);
        const int iyaw = stateIndex(step, 2);
        const int iomega = stateIndex(step, 3);
        hessian_triplets.emplace_back(
            ix, ix, 2.0 * q_xx + regularization);
        hessian_triplets.emplace_back(
            iy, iy, 2.0 * q_yy + regularization);
        hessian_triplets.emplace_back(
            iyaw, iyaw, 2.0 * q_yaw + regularization);
        hessian_triplets.emplace_back(
            iomega, iomega, 2.0 * q_omega + regularization);
        if (std::abs(q_xy) > 1.0e-12)
        {
            hessian_triplets.emplace_back(ix, iy, 2.0 * q_xy);
            hessian_triplets.emplace_back(iy, ix, 2.0 * q_xy);
        }
    }

    const double rate_weight[3] = {
        mpc_weight_delta_vx_,
        mpc_weight_delta_vy_,
        mpc_weight_delta_omega_};
    const double previous_control[3] = {
        clampValue(last_cmd_vel_.linear.x, mpc_min_vx_, mpc_max_vx_),
        clampValue(last_cmd_vel_.linear.y, mpc_min_vy_, mpc_max_vy_),
        clampValue(last_cmd_vel_.angular.z, mpc_min_omega_, mpc_max_omega_)};

    for (int step = 0; step < horizon; ++step)
    {
        const MpcReferencePoint& ref =
            reference[static_cast<std::size_t>(step)];
        const double u_ref[3] = {ref.vx, ref.vy, ref.omega_cmd};

        const double cb = std::cos(ref.drift_beta);
        const double sb = std::sin(ref.drift_beta);
        const double qtt = mpc_weight_tangent_velocity_;
        const double qnn = mpc_weight_path_normal_velocity_;
        const int ivx = controlIndex(step, 0);
        const int ivy = controlIndex(step, 1);
        const double q_vxvx = qtt * cb * cb + qnn * sb * sb;
        const double q_vyvy = qtt * sb * sb + qnn * cb * cb;
        const double q_vxvy = (qtt - qnn) * cb * sb;
        hessian_triplets.emplace_back(ivx, ivx, 2.0 * q_vxvx);
        hessian_triplets.emplace_back(ivy, ivy, 2.0 * q_vyvy);
        if (std::abs(q_vxvy) > 1.0e-12)
        {
            hessian_triplets.emplace_back(ivx, ivy, 2.0 * q_vxvy);
            hessian_triplets.emplace_back(ivy, ivx, 2.0 * q_vxvy);
        }

        // 轻量MPCC式进度奖励：奖励路径切向速度，但仍受曲率速度圆、
        // 输入边界、加速度和轮廓误差共同约束。
        gradient(ivx) -= mpc_weight_progress_ * cb;
        gradient(ivy) -= mpc_weight_progress_ * sb;

        const double omega_control_weight =
            mpc_weight_omega_straight_
            + ref.curve_strength
              * (mpc_weight_omega_curve_ - mpc_weight_omega_straight_);
        const double control_weight[3] = {
            mpc_weight_vx_, mpc_weight_vy_, omega_control_weight};

        for (int component = 0; component < nu; ++component)
        {
            const int index = controlIndex(step, component);
            hessian_triplets.emplace_back(
                index, index,
                2.0 * control_weight[component] + regularization);

            if (step == 0)
            {
                const double constant =
                    u_ref[component] - previous_control[component];
                hessian_triplets.emplace_back(
                    index, index, 2.0 * rate_weight[component]);
                gradient(index) +=
                    2.0 * rate_weight[component] * constant;
            }
            else
            {
                const MpcReferencePoint& prev_ref =
                    reference[static_cast<std::size_t>(step - 1)];
                const double prev_u_ref[3] = {
                    prev_ref.vx, prev_ref.vy, prev_ref.omega_cmd};
                const double constant =
                    u_ref[component] - prev_u_ref[component];
                const int prev_index =
                    controlIndex(step - 1, component);
                hessian_triplets.emplace_back(
                    index, index, 2.0 * rate_weight[component]);
                hessian_triplets.emplace_back(
                    prev_index, prev_index,
                    2.0 * rate_weight[component]);
                hessian_triplets.emplace_back(
                    index, prev_index, -2.0 * rate_weight[component]);
                hessian_triplets.emplace_back(
                    prev_index, index, -2.0 * rate_weight[component]);
                gradient(index) +=
                    2.0 * rate_weight[component] * constant;
                gradient(prev_index) -=
                    2.0 * rate_weight[component] * constant;
            }
        }
    }

    Eigen::SparseMatrix<double> hessian(
        variable_count, variable_count);
    hessian.setFromTriplets(
        hessian_triplets.begin(), hessian_triplets.end());
    hessian.makeCompressed();

    std::vector<Eigen::Triplet<double> > constraint_triplets;
    constraint_triplets.reserve(static_cast<std::size_t>(
        (60 + 2 * mpc_velocity_polygon_sides_) * horizon + 32));
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(constraint_count);
    Eigen::VectorXd upper = Eigen::VectorXd::Zero(constraint_count);

    const Eigen::Vector4d initial_error(
        report.delay_x - reference.front().x,
        report.delay_y - reference.front().y,
        normalizeAngle(report.delay_yaw - reference.front().yaw),
        report.delay_omega - reference.front().omega);
    for (int component = 0; component < nx; ++component)
    {
        constraint_triplets.emplace_back(
            component, stateIndex(0, component), 1.0);
        lower(component) = initial_error(component);
        upper(component) = initial_error(component);
    }

    const double angular_a = std::exp(
        -mpc_dt_ / std::max(0.01, c3_angular_response_tau_));
    const double angular_b = 1.0 - angular_a;

    for (int step = 0; step < horizon; ++step)
    {
        const MpcReferencePoint& current =
            reference[static_cast<std::size_t>(step)];
        const MpcReferencePoint& next =
            reference[static_cast<std::size_t>(step + 1)];
        const double c = std::cos(current.yaw);
        const double ss = std::sin(current.yaw);

        Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
        A(0, 2) = mpc_dt_ * (-ss * current.vx - c * current.vy);
        A(1, 2) = mpc_dt_ * ( c * current.vx - ss * current.vy);
        A(2, 3) = mpc_dt_;
        A(3, 3) = angular_a;

        Eigen::Matrix<double, 4, 3> B;
        B.setZero();
        B(0, 0) = mpc_dt_ * c;
        B(0, 1) = -mpc_dt_ * ss;
        B(1, 0) = mpc_dt_ * ss;
        B(1, 1) = mpc_dt_ * c;
        B(3, 2) = angular_b;

        const double predicted_x = current.x
            + mpc_dt_ * (c * current.vx - ss * current.vy);
        const double predicted_y = current.y
            + mpc_dt_ * (ss * current.vx + c * current.vy);
        const double predicted_yaw =
            current.yaw + mpc_dt_ * current.omega;
        const double predicted_omega =
            angular_a * current.omega
            + angular_b * current.omega_cmd;
        const Eigen::Vector4d residual(
            predicted_x - next.x,
            predicted_y - next.y,
            normalizeAngle(predicted_yaw - next.yaw),
            predicted_omega - next.omega);

        const int row_base = nx * (step + 1);
        for (int row_component = 0;
             row_component < nx; ++row_component)
        {
            const int row = row_base + row_component;
            constraint_triplets.emplace_back(
                row, stateIndex(step + 1, row_component), 1.0);
            for (int column_component = 0;
                 column_component < nx; ++column_component)
            {
                const double value =
                    -A(row_component, column_component);
                if (std::abs(value) > 1.0e-12)
                {
                    constraint_triplets.emplace_back(
                        row,
                        stateIndex(step, column_component),
                        value);
                }
            }
            for (int control_component = 0;
                 control_component < nu; ++control_component)
            {
                const double value =
                    -B(row_component, control_component);
                if (std::abs(value) > 1.0e-12)
                {
                    constraint_triplets.emplace_back(
                        row,
                        controlIndex(step, control_component),
                        value);
                }
            }
            lower(row) = residual(row_component);
            upper(row) = residual(row_component);
        }
    }

    const int input_start = equality_count;
    const int rate_start = equality_count + input_count;
    const int velocity_polygon_start = rate_start + rate_count;
    const double first_dt = clampValue(control_dt, 0.01, 0.20);

    for (int step = 0; step < horizon; ++step)
    {
        const MpcReferencePoint& ref =
            reference[static_cast<std::size_t>(step)];
        const double u_ref[3] = {ref.vx, ref.vy, ref.omega_cmd};

        double actual_min[3] = {
            mpc_min_vx_, mpc_min_vy_, mpc_min_omega_};
        double actual_max[3] = {
            mpc_max_vx_, mpc_max_vy_, mpc_max_omega_};

        for (int component = 0; component < nu; ++component)
        {
            const int variable = controlIndex(step, component);
            const int input_row =
                input_start + nu * step + component;
            constraint_triplets.emplace_back(
                input_row, variable, 1.0);
            lower(input_row) = actual_min[component] - u_ref[component];
            upper(input_row) = actual_max[component] - u_ref[component];

            const int rate_row =
                rate_start + nu * step + component;
            constraint_triplets.emplace_back(rate_row, variable, 1.0);

            double negative_limit = 0.0;
            double positive_limit = 0.0;
            const double step_dt = step == 0 ? first_dt : mpc_dt_;
            if (component == 0)
            {
                negative_limit = mpc_max_decel_x_ * step_dt;
                positive_limit = mpc_max_accel_x_ * step_dt;
            }
            else if (component == 1)
            {
                negative_limit = mpc_max_accel_y_ * step_dt;
                positive_limit = mpc_max_accel_y_ * step_dt;
            }
            else
            {
                negative_limit = mpc_max_accel_theta_ * step_dt;
                positive_limit = mpc_max_accel_theta_ * step_dt;
            }

            if (step == 0)
            {
                lower(rate_row) = previous_control[component]
                    - u_ref[component] - negative_limit;
                upper(rate_row) = previous_control[component]
                    - u_ref[component] + positive_limit;
            }
            else
            {
                const MpcReferencePoint& prev_ref =
                    reference[static_cast<std::size_t>(step - 1)];
                const double prev_u_ref[3] = {
                    prev_ref.vx, prev_ref.vy, prev_ref.omega_cmd};
                constraint_triplets.emplace_back(
                    rate_row,
                    controlIndex(step - 1, component),
                    -1.0);
                const double reference_delta =
                    u_ref[component] - prev_u_ref[component];
                lower(rate_row) =
                    -negative_limit - reference_delta;
                upper(rate_row) =
                     positive_limit - reference_delta;
            }
        }

        // MPCC式优化可以在参考速度爬升前主动增加进度，但不能超过
        // C2曲率速度上限，因此速度圆使用speed_limit而不是path_speed。
        double translational_limit = std::min(
            mpc_max_translational_speed_,
            std::max(0.02, ref.speed_limit));
        if (step == 0)
        {
            const double previous_trans_speed = std::hypot(
                previous_control[0], previous_control[1]);
            const double reachable_min_speed = std::max(
                0.0,
                previous_trans_speed - mpc_max_decel_x_ * first_dt);
            translational_limit = std::max(
                translational_limit, reachable_min_speed);
        }

        for (int side = 0;
             side < mpc_velocity_polygon_sides_; ++side)
        {
            const double angle =
                2.0 * M_PI * static_cast<double>(side)
                / static_cast<double>(mpc_velocity_polygon_sides_);
            const double nx_side = std::cos(angle);
            const double ny_side = std::sin(angle);
            const int row = velocity_polygon_start
                + step * mpc_velocity_polygon_sides_ + side;
            constraint_triplets.emplace_back(
                row, controlIndex(step, 0), nx_side);
            constraint_triplets.emplace_back(
                row, controlIndex(step, 1), ny_side);
            lower(row) = -1.0e20;
            upper(row) = translational_limit
                - nx_side * ref.vx - ny_side * ref.vy;
        }
    }

    Eigen::SparseMatrix<double> constraints(
        constraint_count, variable_count);
    constraints.setFromTriplets(
        constraint_triplets.begin(), constraint_triplets.end());
    constraints.makeCompressed();

    const MpcClock::time_point setup_begin = MpcClock::now();
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(mpc_osqp_verbose_);
    // 当前Noetic环境每周期重建矩阵，为兼容旧版OsqpEigen继续冷启动。
    // 真实状态与历史命令已经解耦，不再用历史命令冒充状态。
    solver.settings()->setWarmStart(false);
    solver.settings()->setPolish(mpc_osqp_polish_);
    solver.settings()->setMaxIteration(mpc_osqp_max_iterations_);
    solver.settings()->setAbsoluteTolerance(mpc_osqp_eps_abs_);
    solver.settings()->setRelativeTolerance(mpc_osqp_eps_rel_);
    solver.settings()->setTimeLimit(
        std::max(0.001, 0.001 * mpc_max_total_time_ms_));

    solver.data()->setNumberOfVariables(variable_count);
    solver.data()->setNumberOfConstraints(constraint_count);
    if (!solver.data()->setHessianMatrix(hessian)
        || !solver.data()->setGradient(gradient)
        || !solver.data()->setLinearConstraintsMatrix(constraints)
        || !solver.data()->setLowerBound(lower)
        || !solver.data()->setUpperBound(upper))
    {
        report.status = "SET_DATA_FAILED";
        report.failure_reason = "OsqpEigen写入QP矩阵或边界失败";
        return false;
    }
    if (!solver.initSolver())
    {
        report.status = "INIT_FAILED";
        report.failure_reason = "OsqpEigen初始化失败";
        return false;
    }
    const MpcClock::time_point setup_end = MpcClock::now();
    const MpcClock::time_point solve_begin = MpcClock::now();
    const OsqpEigen::ErrorExitFlag exit_flag = solver.solveProblem();
    const MpcClock::time_point solve_end = MpcClock::now();

    report.setup_ms = std::chrono::duration<double, std::milli>(
        setup_end - setup_begin).count();
    report.solve_ms = std::chrono::duration<double, std::milli>(
        solve_end - solve_begin).count();
    if (solver.workspace() && solver.workspace()->info != NULL)
    {
        report.iterations =
            static_cast<int>(solver.workspace()->info->iter);
    }

    if (exit_flag != OsqpEigen::ErrorExitFlag::NoError)
    {
        report.status = "SOLVE_EXIT_ERROR";
        report.failure_reason = "OSQP solveProblem返回错误";
        return false;
    }

    const OsqpEigen::Status status = solver.getStatus();
    report.status = osqpStatusName(status);
    report.solved_inaccurate =
        status == OsqpEigen::Status::SolvedInaccurate;
    if (status != OsqpEigen::Status::Solved
        && status != OsqpEigen::Status::SolvedInaccurate)
    {
        report.failure_reason =
            std::string("OSQP未得到可用解：") + report.status;
        return false;
    }

    const Eigen::Matrix<c_float, -1, 1>& solution =
        solver.getSolution();
    if (solution.size() != variable_count)
    {
        report.status = "SOLUTION_SIZE_ERROR";
        report.failure_reason = "OSQP解向量维度异常";
        return false;
    }

    const int first_control = state_count;
    desired_cmd.linear.x = reference.front().vx
        + static_cast<double>(solution(first_control));
    desired_cmd.linear.y = reference.front().vy
        + static_cast<double>(solution(first_control + 1));
    desired_cmd.angular.z = reference.front().omega_cmd
        + static_cast<double>(solution(first_control + 2));

    desired_cmd.linear.x = clampValue(
        desired_cmd.linear.x, mpc_min_vx_, mpc_max_vx_);
    desired_cmd.linear.y = clampValue(
        desired_cmd.linear.y, mpc_min_vy_, mpc_max_vy_);
    desired_cmd.angular.z = clampValue(
        desired_cmd.angular.z, mpc_min_omega_, mpc_max_omega_);

    const double translational_speed = std::hypot(
        desired_cmd.linear.x, desired_cmd.linear.y);
    if (translational_speed > mpc_max_translational_speed_ + 1.0e-9)
    {
        const double scale =
            mpc_max_translational_speed_ / translational_speed;
        desired_cmd.linear.x *= scale;
        desired_cmd.linear.y *= scale;
    }

    report.predicted_path.resize(
        static_cast<std::size_t>(horizon + 1));
    for (int step = 0; step <= horizon; ++step)
    {
        PathPoint2D& predicted =
            report.predicted_path[static_cast<std::size_t>(step)];
        predicted.x = reference[static_cast<std::size_t>(step)].x
            + static_cast<double>(solution(stateIndex(step, 0)));
        predicted.y = reference[static_cast<std::size_t>(step)].y
            + static_cast<double>(solution(stateIndex(step, 1)));
        predicted.yaw = reference[static_cast<std::size_t>(step)].yaw
            + static_cast<double>(solution(stateIndex(step, 2)));
        if (step == 0)
        {
            predicted.s = 0.0;
        }
        else
        {
            predicted.s =
                report.predicted_path[static_cast<std::size_t>(step - 1)].s
                + reference[static_cast<std::size_t>(step - 1)].path_speed
                  * mpc_dt_;
        }
    }

    report.objective = static_cast<double>(solver.getObjValue());
    return true;
}

void MyPlanner::publishMpcDebugPaths(const MpcSolveReport& report)
{
    if (!mpc_publish_debug_paths_)
        return;

    const ros::Time stamp = ros::Time::now();
    const auto publish_path =
        [&](const std::vector<PathPoint2D>& points, ros::Publisher& publisher)
        {
            nav_msgs::Path path;
            path.header.frame_id = base_frame_;
            path.header.stamp = stamp;
            path.poses.reserve(points.size());
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = points[i].x;
                pose.pose.position.y = points[i].y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(points[i].yaw);
                path.poses.push_back(pose);
            }
            publisher.publish(path);
        };

    publish_path(report.reference_path, mpc_reference_path_pub_);
    publish_path(report.predicted_path, mpc_predicted_path_pub_);
}

}  // namespace my_planner