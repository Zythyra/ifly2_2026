#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros_nanodet/detect_result_srv.h>
#include <ros_nanodet/ocr_result_srv.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ucarmain2026/set_speed.h>

#include <algorithm>
#include <cmath>
#include <clocale>
#include <limits>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace {

const double kPi = 3.14159265358979323846;

double clampValue(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double distance2D(double x0, double y0, double x1, double y1) {
    return std::hypot(x1 - x0, y1 - y0);
}

}  // namespace

class TargetPatrolDocking {
public:
    TargetPatrolDocking()
        : nh_(),
          pnh_("~"),
          move_base_("move_base", true),
          tf_listener_(tf_buffer_) {
        pnh_.param("real_target_category", real_target_category_, std::string("food"));
        pnh_.param("simulation_target_category", simulation_target_category_,
                   std::string("daily"));

        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("scan_topic", scan_topic_, std::string("/scan"));
        pnh_.param("room_min_x", room_min_x_, 0.0);
        pnh_.param("room_max_x", room_max_x_, 5.0);
        pnh_.param("room_min_y", room_min_y_, 2.5);
        pnh_.param("room_max_y", room_max_y_, 4.5);
        pnh_.param("start_x", start_x_, 0.25);
        pnh_.param("start_y", start_y_, 4.25);
        pnh_.param("start_yaw_deg", start_yaw_deg_, 0.0);
        pnh_.param("navigation_timeout", navigation_timeout_, 180.0);

        pnh_.param("patrol_forward_speed", patrol_forward_speed_, 0.25);
        pnh_.param("line_hold_kp", line_hold_kp_, 2.0);
        pnh_.param("line_hold_max_speed", line_hold_max_speed_, 0.16);
        pnh_.param("yaw_hold_kp", yaw_hold_kp_, 2.5);
        pnh_.param("yaw_hold_max_speed", yaw_hold_max_speed_, 0.30);
        pnh_.param("yaw_pause_threshold_deg", yaw_pause_threshold_deg_, 10.0);
        pnh_.param("segment_end_tolerance", segment_end_tolerance_, 0.025);
        pnh_.param("control_rate", control_rate_, 15.0);
        pnh_.param("patrol_use_lidar_wall_follow",
                   patrol_use_lidar_wall_follow_, true);
        pnh_.param("wall_target_distance", wall_target_distance_, 0.25);
        pnh_.param("wall_distance_kp", wall_distance_kp_, 1.8);
        pnh_.param("wall_angle_kp", wall_angle_kp_, 2.5);
        pnh_.param("wall_max_lateral_speed",
                   wall_max_lateral_speed_, 0.16);
        pnh_.param("wall_max_angular_speed",
                   wall_max_angular_speed_, 0.30);
        pnh_.param("wall_scan_min_angle_deg",
                   wall_scan_min_angle_deg_, 50.0);
        pnh_.param("wall_scan_max_angle_deg",
                   wall_scan_max_angle_deg_, 130.0);
        pnh_.param("wall_scan_min_range", wall_scan_min_range_, 0.10);
        pnh_.param("wall_scan_max_range", wall_scan_max_range_, 0.70);
        pnh_.param("wall_fit_inlier_threshold",
                   wall_fit_inlier_threshold_, 0.025);
        pnh_.param("wall_fit_min_points", wall_fit_min_points_, 12);
        pnh_.param("wall_fit_min_inliers", wall_fit_min_inliers_, 10);
        pnh_.param("wall_fit_min_span", wall_fit_min_span_, 0.16);
        pnh_.param("wall_fit_max_angle_deg",
                   wall_fit_max_angle_deg_, 30.0);
        pnh_.param("wall_fit_max_rms", wall_fit_max_rms_, 0.030);
        pnh_.param("wall_fit_filter_alpha",
                   wall_fit_filter_alpha_, 0.35);

        pnh_.param("rotate_kp", rotate_kp_, 3.5);
        pnh_.param("rotate_min_speed", rotate_min_speed_, 0.15);
        pnh_.param("rotate_max_speed", rotate_max_speed_, 0.65);
        pnh_.param("rotate_tolerance_deg", rotate_tolerance_deg_, 2.0);
        pnh_.param("rotate_stable_frames", rotate_stable_frames_, 3);
        pnh_.param("rotate_timeout", rotate_timeout_, 12.0);
        pnh_.param("position_hold_kp", position_hold_kp_, 2.0);
        pnh_.param("position_hold_max_speed", position_hold_max_speed_, 0.10);

        pnh_.param("image_width", image_width_, 640);
        pnh_.param("camera_fx", camera_fx_, 554.256);
        pnh_.param("camera_yaw_offset_deg", camera_yaw_offset_deg_, 0.0);
        pnh_.param("docking_standoff", docking_standoff_, 0.50);
        pnh_.param("settle_time", settle_time_, 0.25);
        pnh_.param("ocr_attempts", ocr_attempts_, 3);
        pnh_.param("ocr_retry_interval", ocr_retry_interval_, 0.12);
        pnh_.param("max_detection_duration", max_detection_duration_, 0.50);
        pnh_.param("duplicate_coordinate_distance",
                   duplicate_coordinate_distance_, 0.50);
        pnh_.param("max_track_jump_px", max_track_jump_px_, 140.0);
        pnh_.param("max_lost_frames", max_lost_frames_, 4);

        pnh_.param("retreat_line_tolerance", retreat_line_tolerance_, 0.025);
        pnh_.param("retreat_slow_distance", retreat_slow_distance_, 0.15);
        pnh_.param("retreat_min_speed", retreat_min_speed_, 0.06);
        pnh_.param("retreat_max_speed", retreat_max_speed_, 0.18);
        pnh_.param("retreat_timeout", retreat_timeout_, 8.0);

        pnh_.param("lateral_align_kp", lateral_align_kp_, 0.0010);
        pnh_.param("lateral_align_min_speed", lateral_align_min_speed_, 0.06);
        pnh_.param("lateral_align_max_speed", lateral_align_max_speed_, 0.20);
        pnh_.param("lateral_center_tolerance_px",
                   lateral_center_tolerance_px_, 10.0);
        pnh_.param("lateral_stable_frames", lateral_stable_frames_, 3);
        pnh_.param("lateral_align_timeout", lateral_align_timeout_, 15.0);

        pnh_.param("front_lidar_half_window", front_lidar_half_window_, 3);
        pnh_.param("approach_stop_distance", approach_stop_distance_, 0.20);
        pnh_.param("approach_slow_distance", approach_slow_distance_, 0.35);
        pnh_.param("approach_min_speed", approach_min_speed_, 0.05);
        pnh_.param("approach_max_speed", approach_max_speed_, 0.12);
        pnh_.param("approach_timeout", approach_timeout_, 15.0);
        pnh_.param("lidar_data_timeout", lidar_data_timeout_, 0.50);
        pnh_.param("lidar_loss_abort_timeout", lidar_loss_abort_timeout_, 2.0);
        pnh_.param("docking_yaw_kp", docking_yaw_kp_, 2.0);
        pnh_.param("docking_yaw_max_speed", docking_yaw_max_speed_, 0.25);

        normalizeParameters();
        buildSegments();
        configuration_valid_ = validateConfiguration();

        detect_client_ =
            nh_.serviceClient<ros_nanodet::detect_result_srv>("/nanodet_detect");
        ocr_client_ =
            nh_.serviceClient<ros_nanodet::ocr_result_srv>("/nanodet_ocr");
        set_speed_client_ =
            nh_.serviceClient<ucarmain2026::set_speed>("/set_speed");
        scan_subscriber_ =
            nh_.subscribe(scan_topic_, 1, &TargetPatrolDocking::scanCallback, this);
        ROS_INFO("车头向前左侧三分之二画面找板：现实目标=%s，仿真目标=%s",
                 categoryChinese(real_target_category_),
                 categoryChinese(simulation_target_category_));
        ROS_INFO("找板房间边界=[%.2f, %.2f]×[%.2f, %.2f]；"
                 "巡检检测使用图像左侧三分之二区域；"
                 "停靠导航点距墙%.2fm",
                 room_min_x_, room_max_x_, room_min_y_, room_max_y_,
                 docking_standoff_);
        ROS_INFO("巡检守线模式：%s；左墙目标距离=%.3fm",
                 patrol_use_lidar_wall_follow_
                     ? "左侧雷达直线拟合（拟合失败时回退map巡检线）"
                     : "仅使用map巡检线",
                 wall_target_distance_);
    }

    ~TargetPatrolDocking() {
        stopRobot();
        closeCamera();
    }

    bool run() {
        if (!configuration_valid_) return false;
        if (!waitForDependencies()) return false;

        if (!navigateToPose(start_x_, start_y_,
                            start_yaw_deg_ * kPi / 180.0,
                            "初始巡检点")) {
            return false;
        }
        if (!openCamera()) return false;

        for (std::size_t index = 0; index < segments_.size() && ros::ok();) {
            current_segment_index_ = static_cast<int>(index);
            const Segment& segment = segments_[index];

            if (!rotateToYaw(segment.travel_yaw)) {
                ROS_ERROR("进入第%zu段前无法固定车头朝向", index + 1);
                return false;
            }

            const SegmentResult result = patrolSegment(index);
            if (result == SEGMENT_MISSION_COMPLETE) {
                printSummary(true);
                return true;
            }
            if (result == SEGMENT_ABORTED) {
                printSummary(false);
                return false;
            }
            ++index;
        }

        stopRobot();
        closeCamera();
        const bool success = real_docked_ && simulation_docked_;
        printSummary(success);
        if (!success) {
            ROS_ERROR("四段巡检结束，仍未完成两个目标的停靠");
        }
        return success;
    }

private:
    struct Pose2D {
        double x;
        double y;
        double yaw;

        Pose2D() : x(0.0), y(0.0), yaw(0.0) {}
        Pose2D(double px, double py, double pyaw) : x(px), y(py), yaw(pyaw) {}
    };

    enum WallType {
        WALL_LEFT = 0,
        WALL_RIGHT = 1,
        WALL_BOTTOM = 2,
        WALL_TOP = 3
    };

    struct Segment {
        std::string name;
        double start_x;
        double start_y;
        double end_x;
        double end_y;
        double dir_x;
        double dir_y;
        double length;
        double travel_yaw;
        double docking_yaw;
        WallType wall;
    };

    struct Box {
        int class_id;
        int x0;
        int y0;
        int x1;
        int y1;

        double centerX() const {
            return 0.5 * static_cast<double>(x0 + x1);
        }
        double centerY() const {
            return 0.5 * static_cast<double>(y0 + y1);
        }
        double width() const {
            return std::max(1, x1 - x0);
        }
        double height() const {
            return std::max(1, y1 - y0);
        }
    };

    struct OcrRecord {
        bool success;
        std::string text;
        std::string category;
        double confidence;
        Box box;

        OcrRecord()
            : success(false),
              text("<未识别>"),
              category("unknown"),
              confidence(0.0),
              box{0, 0, 0, 0, 0} {}
    };

    struct TargetObservation {
        bool valid;
        Pose2D pose;
        int segment_index;
        std::string category;

        TargetObservation()
            : valid(false), segment_index(-1), category("unknown") {}
    };

    struct BoardBoundaryEstimate {
        bool valid;
        WallType wall;
        double x;
        double y;

        BoardBoundaryEstimate()
            : valid(false), wall(WALL_LEFT), x(0.0), y(0.0) {}
    };

    struct Point2D {
        double x;
        double y;
    };

    struct WallFit {
        bool valid;
        double distance;
        double angle;
        double rms;
        int point_count;
        int inlier_count;

        WallFit()
            : valid(false),
              distance(0.0),
              angle(0.0),
              rms(std::numeric_limits<double>::infinity()),
              point_count(0),
              inlier_count(0) {}
    };

    enum SegmentResult {
        SEGMENT_COMPLETE,
        SEGMENT_MISSION_COMPLETE,
        SEGMENT_ABORTED
    };

    enum DetectionResult {
        DETECTION_CONTINUE,
        DETECTION_MISSION_COMPLETE,
        DETECTION_ABORT
    };

    static bool isValidCategory(const std::string& category) {
        return category == "food" || category == "daily" ||
               category == "electronic";
    }

    static std::string classifyText(const std::string& text) {
        if (text.find("食品") != std::string::npos) return "food";
        if (text.find("日用品") != std::string::npos) return "daily";
        if (text.find("电子") != std::string::npos ||
            text.find("生产") != std::string::npos) {
            return "electronic";
        }
        return "unknown";
    }

    static const char* categoryChinese(const std::string& category) {
        if (category == "food") return "食品加工车间";
        if (category == "daily") return "日用品加工车间";
        if (category == "electronic") return "电子产品生产车间";
        return "未知";
    }

    void normalizeParameters() {
        patrol_forward_speed_ = std::fabs(patrol_forward_speed_);
        line_hold_kp_ = std::fabs(line_hold_kp_);
        line_hold_max_speed_ = std::fabs(line_hold_max_speed_);
        yaw_hold_kp_ = std::fabs(yaw_hold_kp_);
        yaw_hold_max_speed_ = std::fabs(yaw_hold_max_speed_);
        wall_target_distance_ = std::fabs(wall_target_distance_);
        wall_distance_kp_ = std::fabs(wall_distance_kp_);
        wall_angle_kp_ = std::fabs(wall_angle_kp_);
        wall_max_lateral_speed_ = std::fabs(wall_max_lateral_speed_);
        wall_max_angular_speed_ = std::fabs(wall_max_angular_speed_);
        wall_scan_min_range_ = std::fabs(wall_scan_min_range_);
        wall_scan_max_range_ =
            std::max(std::fabs(wall_scan_max_range_),
                     wall_scan_min_range_ + 0.01);
        wall_fit_inlier_threshold_ =
            std::fabs(wall_fit_inlier_threshold_);
        wall_fit_min_points_ = std::max(2, wall_fit_min_points_);
        wall_fit_min_inliers_ = std::max(2, wall_fit_min_inliers_);
        wall_fit_min_span_ = std::fabs(wall_fit_min_span_);
        wall_fit_max_angle_deg_ = std::fabs(wall_fit_max_angle_deg_);
        wall_fit_max_rms_ = std::fabs(wall_fit_max_rms_);
        wall_fit_filter_alpha_ =
            clampValue(wall_fit_filter_alpha_, 0.01, 1.0);
        rotate_kp_ = std::fabs(rotate_kp_);
        rotate_min_speed_ = std::fabs(rotate_min_speed_);
        rotate_max_speed_ =
            std::max(std::fabs(rotate_max_speed_), rotate_min_speed_);
        position_hold_kp_ = std::fabs(position_hold_kp_);
        position_hold_max_speed_ = std::fabs(position_hold_max_speed_);
        duplicate_coordinate_distance_ =
            std::fabs(duplicate_coordinate_distance_);
        camera_fx_ = std::fabs(camera_fx_);
        docking_standoff_ = std::fabs(docking_standoff_);
        retreat_line_tolerance_ = std::fabs(retreat_line_tolerance_);
        retreat_slow_distance_ = std::fabs(retreat_slow_distance_);
        retreat_min_speed_ = std::fabs(retreat_min_speed_);
        retreat_max_speed_ =
            std::max(std::fabs(retreat_max_speed_), retreat_min_speed_);
        lateral_align_min_speed_ = std::fabs(lateral_align_min_speed_);
        lateral_align_max_speed_ =
            std::max(std::fabs(lateral_align_max_speed_),
                     lateral_align_min_speed_);
        approach_min_speed_ = std::fabs(approach_min_speed_);
        approach_max_speed_ =
            std::max(std::fabs(approach_max_speed_), approach_min_speed_);
        docking_yaw_kp_ = std::fabs(docking_yaw_kp_);
        docking_yaw_max_speed_ = std::fabs(docking_yaw_max_speed_);
        front_lidar_half_window_ = std::max(0, front_lidar_half_window_);
        rotate_stable_frames_ = std::max(1, rotate_stable_frames_);
        lateral_stable_frames_ = std::max(1, lateral_stable_frames_);
    }

    void buildSegments() {
        segments_.clear();
        // 四条巡检基准线保持不变：
        // y=4.25、x=4.75、y=2.75、x=0.25。
        // 5cm仅用于当前巡检段的末端：在到达下一条巡检线前提前停车。
        // 转向后，publishPatrolCommand()现有的linear.y守线修正会让小车
        // 一边沿新方向前进，一边横移回到下一条巡检基准线上。
        addSegment("上墙巡检", 0.25, 4.25, 4.70, 4.25,
                   0.0, 0.5 * kPi, WALL_TOP);
        addSegment("右墙巡检", 4.75, 4.25, 4.75, 2.80,
                   -0.5 * kPi, 0.0, WALL_RIGHT);
        addSegment("下墙巡检", 4.75, 2.75, 0.30, 2.75,
                   kPi, -0.5 * kPi, WALL_BOTTOM);
        addSegment("左墙巡检", 0.25, 2.75, 0.25, 4.20,
                   0.5 * kPi, kPi, WALL_LEFT);
    }

    void addSegment(const std::string& name,
                    double start_x, double start_y,
                    double end_x, double end_y,
                    double travel_yaw, double docking_yaw,
                    WallType wall) {
        Segment segment;
        segment.name = name;
        segment.start_x = start_x;
        segment.start_y = start_y;
        segment.end_x = end_x;
        segment.end_y = end_y;
        segment.length = distance2D(start_x, start_y, end_x, end_y);
        segment.dir_x = (end_x - start_x) / segment.length;
        segment.dir_y = (end_y - start_y) / segment.length;
        segment.travel_yaw = travel_yaw;
        segment.docking_yaw = docking_yaw;
        segment.wall = wall;
        segments_.push_back(segment);
    }

    bool validateConfiguration() {
        if (!isValidCategory(real_target_category_) ||
            !isValidCategory(simulation_target_category_)) {
            ROS_ERROR("real_target_category和simulation_target_category只允许"
                      "food、daily、electronic");
            return false;
        }
        if (real_target_category_ == simulation_target_category_) {
            ROS_ERROR("现实目标和仿真目标不能设置为同一类别");
            return false;
        }
        if (room_max_x_ <= room_min_x_ || room_max_y_ <= room_min_y_) {
            ROS_ERROR("房间坐标边界无效");
            return false;
        }
        if (max_detection_duration_ <= 0.0 ||
            camera_fx_ <= 0.0 ||
            docking_standoff_ <= 0.0 ||
            duplicate_coordinate_distance_ <= 0.0 ||
            approach_stop_distance_ <= 0.0 ||
            approach_slow_distance_ <= approach_stop_distance_ ||
            retreat_slow_distance_ <= retreat_line_tolerance_ ||
            retreat_timeout_ <= 0.0) {
            ROS_ERROR("视觉、重复判定、后退或雷达逼近参数无效");
            return false;
        }
        if (wall_target_distance_ <= 0.0 ||
            wall_scan_min_angle_deg_ >= wall_scan_max_angle_deg_ ||
            wall_fit_min_points_ < 2 ||
            wall_fit_min_inliers_ < 2 ||
            wall_fit_inlier_threshold_ <= 0.0 ||
            wall_fit_min_span_ <= 0.0 ||
            wall_fit_max_rms_ <= 0.0) {
            ROS_ERROR("左墙拟合参数无效");
            return false;
        }
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            if (!isInsideRoom(segments_[i].start_x, segments_[i].start_y) ||
                !isInsideRoom(segments_[i].end_x, segments_[i].end_y)) {
                ROS_ERROR("第%zu段巡检端点超出房间坐标边界", i + 1);
                return false;
            }
        }
        return true;
    }

    bool waitForDependencies() {
        ROS_INFO("等待move_base、NanoDet、OCR和运动控制服务...");
        while (ros::ok() && !move_base_.waitForServer(ros::Duration(3.0))) {
            ROS_INFO("仍在等待move_base");
        }
        if (!ros::ok()) return false;
        if (!ros::service::waitForService("/nanodet_detect",
                                          ros::Duration(20.0))) {
            ROS_ERROR("等待/nanodet_detect超时");
            return false;
        }
        if (!ros::service::waitForService("/nanodet_ocr",
                                          ros::Duration(20.0))) {
            ROS_ERROR("等待/nanodet_ocr超时");
            return false;
        }
        if (!ros::service::waitForService("/set_speed",
                                          ros::Duration(20.0))) {
            ROS_ERROR("等待/set_speed超时");
            return false;
        }
        return true;
    }

    bool getRobotPose(Pose2D& pose) {
        try {
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                           ros::Duration(0.20));
            pose.x = transform.transform.translation.x;
            pose.y = transform.transform.translation.y;
            pose.yaw = tf2::getYaw(transform.transform.rotation);
            return true;
        } catch (const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(1.0, "读取机器人map位姿失败：%s", error.what());
            return false;
        }
    }

    bool isInsideRoom(double x, double y) const {
        const double epsilon = 1e-9;
        return x >= room_min_x_ - epsilon &&
               x <= room_max_x_ + epsilon &&
               y >= room_min_y_ - epsilon &&
               y <= room_max_y_ + epsilon;
    }

    void clampToRoom(double& x, double& y) const {
        x = clampValue(x, room_min_x_, room_max_x_);
        y = clampValue(y, room_min_y_, room_max_y_);
    }

    bool navigateToPose(double x, double y, double yaw,
                        const std::string& purpose) {
        clampToRoom(x, y);
        if (!isInsideRoom(x, y)) {
            ROS_ERROR("%s目标(%.3f, %.3f, %.1f度)超出房间坐标边界",
                      purpose.c_str(), x, y, yaw * 180.0 / kPi);
            return false;
        }

        stopRobot();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(quaternion);

        ROS_INFO("%s：发送move_base目标(%.3f, %.3f, %.1f度)",
                 purpose.c_str(), x, y, yaw * 180.0 / kPi);
        move_base_.sendGoal(goal);
        if (!move_base_.waitForResult(ros::Duration(navigation_timeout_))) {
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("%s超时", purpose.c_str());
            return false;
        }
        if (move_base_.getState() !=
            actionlib::SimpleClientGoalState::SUCCEEDED) {
            const std::string state = move_base_.getState().toString();
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("%s失败：%s", purpose.c_str(), state.c_str());
            return false;
        }

        stopRobot();
        return true;
    }

    bool rotateToYaw(double desired_yaw) {
        Pose2D anchor;
        if (!getRobotPose(anchor)) return false;
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(rotate_timeout_);
        int stable_frames = 0;
        ros::Rate rate(20.0);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }
            const double yaw_error = normalizeAngle(desired_yaw - pose.yaw);
            const double world_error_x = anchor.x - pose.x;
            const double world_error_y = anchor.y - pose.y;
            const double base_error_x =
                std::cos(pose.yaw) * world_error_x +
                std::sin(pose.yaw) * world_error_y;
            const double base_error_y =
                -std::sin(pose.yaw) * world_error_x +
                std::cos(pose.yaw) * world_error_y;

            double angular_z =
                clampValue(rotate_kp_ * yaw_error,
                           -rotate_max_speed_, rotate_max_speed_);
            if (std::fabs(yaw_error) >
                    rotate_tolerance_deg_ * kPi / 180.0 &&
                std::fabs(angular_z) < rotate_min_speed_) {
                angular_z = angular_z >= 0.0
                                ? rotate_min_speed_ : -rotate_min_speed_;
            }
            const double linear_x =
                clampValue(position_hold_kp_ * base_error_x,
                           -position_hold_max_speed_,
                           position_hold_max_speed_);
            const double linear_y =
                clampValue(position_hold_kp_ * base_error_y,
                           -position_hold_max_speed_,
                           position_hold_max_speed_);

            if (std::fabs(yaw_error) <=
                rotate_tolerance_deg_ * kPi / 180.0) {
                ++stable_frames;
                angular_z = 0.0;
                if (stable_frames >= rotate_stable_frames_) {
                    stopRobot();
                    ROS_INFO("车头已固定为%.1f度",
                             desired_yaw * 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
            }
            publishVelocity(linear_x, linear_y, angular_z);
            rate.sleep();
        }
        stopRobot();
        ROS_ERROR("旋转至%.1f度超时", desired_yaw * 180.0 / kPi);
        return false;
    }

    double segmentProgress(const Segment& segment, const Pose2D& pose) const {
        return (pose.x - segment.start_x) * segment.dir_x +
               (pose.y - segment.start_y) * segment.dir_y;
    }

    void resetWallFollower() {
        wall_fit_initialized_ = false;
        filtered_wall_distance_ = 0.0;
        filtered_wall_angle_ = 0.0;
    }

    WallFit fitLeftWall() {
        WallFit result;
        if (!have_laser_scan_ ||
            (ros::WallTime::now() - latest_scan_wall_time_).toSec() >
                lidar_data_timeout_ ||
            latest_scan_.ranges.empty() ||
            std::fabs(latest_scan_.angle_increment) < 1e-12) {
            return result;
        }

        const double min_angle =
            wall_scan_min_angle_deg_ * kPi / 180.0;
        const double max_angle =
            wall_scan_max_angle_deg_ * kPi / 180.0;
        const double valid_min_range =
            std::max(wall_scan_min_range_,
                     static_cast<double>(latest_scan_.range_min));
        const double valid_max_range =
            std::min(wall_scan_max_range_,
                     static_cast<double>(latest_scan_.range_max));

        std::vector<Point2D> points;
        points.reserve(latest_scan_.ranges.size() / 3);
        for (std::size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            const double angle =
                latest_scan_.angle_min +
                static_cast<double>(i) * latest_scan_.angle_increment;
            if (angle < min_angle || angle > max_angle) continue;
            const double range =
                latest_scan_.ranges[i];
            if (!std::isfinite(range) ||
                range < valid_min_range ||
                range > valid_max_range) {
                continue;
            }
            Point2D point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            if (point.y <= 0.0) continue;
            points.push_back(point);
        }
        result.point_count = static_cast<int>(points.size());
        if (result.point_count < wall_fit_min_points_) return result;

        // RANSAC先找左侧占比最大的近似水平直线，避免文字板、路障和
        // 转角处相邻墙面把普通最小二乘拟合拉歪。
        int best_inlier_count = 0;
        double best_residual_sum = std::numeric_limits<double>::infinity();
        double best_nx = 0.0;
        double best_ny = 1.0;
        double best_d = 0.0;
        const std::size_t hypothesis_step =
            std::max<std::size_t>(1, points.size() / 45);
        for (std::size_t i = 0; i + 1 < points.size();
             i += hypothesis_step) {
            for (std::size_t j = i + 1; j < points.size();
                 j += hypothesis_step) {
                const double dx = points[j].x - points[i].x;
                const double dy = points[j].y - points[i].y;
                const double pair_span = std::hypot(dx, dy);
                if (pair_span < wall_fit_min_span_ * 0.55) continue;

                double tangent_angle = std::atan2(dy, dx);
                if (tangent_angle > 0.5 * kPi) tangent_angle -= kPi;
                if (tangent_angle < -0.5 * kPi) tangent_angle += kPi;
                if (std::fabs(tangent_angle) >
                    wall_fit_max_angle_deg_ * kPi / 180.0) {
                    continue;
                }

                double nx = -dy / pair_span;
                double ny = dx / pair_span;
                if (ny < 0.0) {
                    nx = -nx;
                    ny = -ny;
                }
                const double d = nx * points[i].x + ny * points[i].y;
                if (d <= 0.0) continue;

                int inlier_count = 0;
                double residual_sum = 0.0;
                for (std::size_t k = 0; k < points.size(); ++k) {
                    const double residual =
                        std::fabs(nx * points[k].x +
                                  ny * points[k].y - d);
                    if (residual <= wall_fit_inlier_threshold_) {
                        ++inlier_count;
                        residual_sum += residual;
                    }
                }
                if (inlier_count > best_inlier_count ||
                    (inlier_count == best_inlier_count &&
                     residual_sum < best_residual_sum)) {
                    best_inlier_count = inlier_count;
                    best_residual_sum = residual_sum;
                    best_nx = nx;
                    best_ny = ny;
                    best_d = d;
                }
            }
        }
        if (best_inlier_count < wall_fit_min_inliers_) return result;

        std::vector<Point2D> inliers;
        inliers.reserve(static_cast<std::size_t>(best_inlier_count));
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double residual =
                std::fabs(best_nx * points[i].x +
                          best_ny * points[i].y - best_d);
            if (residual <= wall_fit_inlier_threshold_) {
                inliers.push_back(points[i]);
            }
        }

        double mean_x = 0.0;
        double mean_y = 0.0;
        for (std::size_t i = 0; i < inliers.size(); ++i) {
            mean_x += inliers[i].x;
            mean_y += inliers[i].y;
        }
        mean_x /= static_cast<double>(inliers.size());
        mean_y /= static_cast<double>(inliers.size());

        double sxx = 0.0;
        double sxy = 0.0;
        double syy = 0.0;
        for (std::size_t i = 0; i < inliers.size(); ++i) {
            const double x = inliers[i].x - mean_x;
            const double y = inliers[i].y - mean_y;
            sxx += x * x;
            sxy += x * y;
            syy += y * y;
        }
        const double tangent_angle =
            0.5 * std::atan2(2.0 * sxy, sxx - syy);
        if (std::fabs(tangent_angle) >
            wall_fit_max_angle_deg_ * kPi / 180.0) {
            return result;
        }

        const double tangent_x = std::cos(tangent_angle);
        const double tangent_y = std::sin(tangent_angle);
        const double normal_x = -tangent_y;
        const double normal_y = tangent_x;
        const double distance = normal_x * mean_x + normal_y * mean_y;
        if (distance <= 0.0) return result;

        double min_projection = std::numeric_limits<double>::infinity();
        double max_projection = -std::numeric_limits<double>::infinity();
        double squared_error_sum = 0.0;
        for (std::size_t i = 0; i < inliers.size(); ++i) {
            const double projection =
                tangent_x * inliers[i].x +
                tangent_y * inliers[i].y;
            min_projection = std::min(min_projection, projection);
            max_projection = std::max(max_projection, projection);
            const double residual =
                normal_x * inliers[i].x +
                normal_y * inliers[i].y - distance;
            squared_error_sum += residual * residual;
        }
        const double span = max_projection - min_projection;
        const double rms =
            std::sqrt(squared_error_sum /
                      static_cast<double>(inliers.size()));
        if (span < wall_fit_min_span_ || rms > wall_fit_max_rms_) {
            return result;
        }

        result.valid = true;
        result.distance = distance;
        result.angle = tangent_angle;
        result.rms = rms;
        result.inlier_count = static_cast<int>(inliers.size());
        return result;
    }

    void publishMapPatrolCommand(const Segment& segment,
                                 const Pose2D& pose) {
        // 顺时针巡检时墙始终位于左侧，因此左法向就是巡检线法向。
        const double normal_x = -segment.dir_y;
        const double normal_y = segment.dir_x;
        const double line_error =
            (segment.start_x - pose.x) * normal_x +
            (segment.start_y - pose.y) * normal_y;
        const double yaw_error =
            normalizeAngle(segment.travel_yaw - pose.yaw);
        double linear_x = patrol_forward_speed_;
        const double linear_y =
            clampValue(line_hold_kp_ * line_error,
                       -line_hold_max_speed_, line_hold_max_speed_);
        const double angular_z =
            clampValue(yaw_hold_kp_ * yaw_error,
                       -yaw_hold_max_speed_, yaw_hold_max_speed_);
        if (std::fabs(yaw_error) >
            yaw_pause_threshold_deg_ * kPi / 180.0) {
            linear_x = 0.0;
        }
        publishVelocity(linear_x, linear_y, angular_z);
    }

    void publishPatrolCommand(const Segment& segment, const Pose2D& pose) {
        if (!patrol_use_lidar_wall_follow_) {
            publishMapPatrolCommand(segment, pose);
            return;
        }

        const WallFit fit = fitLeftWall();
        if (!fit.valid) {
            ROS_WARN_THROTTLE(
                1.0,
                "左墙拟合无效（候选点=%d），本周期回退map巡检线控制",
                fit.point_count);
            resetWallFollower();
            publishMapPatrolCommand(segment, pose);
            return;
        }

        if (!wall_fit_initialized_) {
            filtered_wall_distance_ = fit.distance;
            filtered_wall_angle_ = fit.angle;
            wall_fit_initialized_ = true;
        } else {
            filtered_wall_distance_ +=
                wall_fit_filter_alpha_ *
                (fit.distance - filtered_wall_distance_);
            filtered_wall_angle_ = normalizeAngle(
                filtered_wall_angle_ +
                wall_fit_filter_alpha_ *
                normalizeAngle(fit.angle - filtered_wall_angle_));
        }

        const double distance_error =
            filtered_wall_distance_ - wall_target_distance_;
        const double linear_y =
            clampValue(wall_distance_kp_ * distance_error,
                       -wall_max_lateral_speed_,
                       wall_max_lateral_speed_);
        const double angular_z =
            clampValue(wall_angle_kp_ * filtered_wall_angle_,
                       -wall_max_angular_speed_,
                       wall_max_angular_speed_);
        double linear_x = patrol_forward_speed_;
        if (std::fabs(filtered_wall_angle_) >
            yaw_pause_threshold_deg_ * kPi / 180.0) {
            linear_x = 0.0;
        }

        ROS_INFO_THROTTLE(
            0.5,
            "左墙拟合巡检：距离=%.3fm，夹角=%.2f度，"
            "内点=%d/%d，RMS=%.3fm，vy=%.3f，wz=%.3f",
            filtered_wall_distance_,
            filtered_wall_angle_ * 180.0 / kPi,
            fit.inlier_count, fit.point_count, fit.rms,
            linear_y, angular_z);
        publishVelocity(linear_x, linear_y, angular_z);
    }

    static const char* wallName(WallType wall) {
        switch (wall) {
            case WALL_LEFT: return "左墙";
            case WALL_RIGHT: return "右墙";
            case WALL_BOTTOM: return "下墙";
            case WALL_TOP: return "上墙";
        }
        return "未知墙面";
    }

    bool estimateBoardBoundary(const Pose2D& robot_pose,
                               const Box& box,
                               BoardBoundaryEstimate& estimate) const {
        estimate = BoardBoundaryEstimate();
        const double image_center = 0.5 * static_cast<double>(image_width_);
        // 图像右侧对应机器人右侧，因此像素向右时相对航向角为负。
        const double relative_yaw =
            std::atan2(image_center - box.centerX(), camera_fx_);
        const double ray_yaw =
            robot_pose.yaw +
            camera_yaw_offset_deg_ * kPi / 180.0 +
            relative_yaw;
        const double ray_x = std::cos(ray_yaw);
        const double ray_y = std::sin(ray_yaw);
        double nearest_t = std::numeric_limits<double>::infinity();
        BoardBoundaryEstimate nearest;
        const double epsilon = 1e-6;

        // 扩大到左侧三分之二画面后，转角处可能同时看到当前左墙和
        // 前方相邻墙。不能再强制把目标投影到segment.wall，而应取
        // 相机射线与矩形场地四条边界的最近正向交点。
        const auto consider =
            [&](WallType wall, double t, double x, double y) {
                if (!std::isfinite(t) || t <= epsilon ||
                    t >= nearest_t) {
                    return;
                }
                if (x < room_min_x_ - epsilon ||
                    x > room_max_x_ + epsilon ||
                    y < room_min_y_ - epsilon ||
                    y > room_max_y_ + epsilon) {
                    return;
                }
                nearest_t = t;
                nearest.valid = true;
                nearest.wall = wall;
                nearest.x = clampValue(x, room_min_x_, room_max_x_);
                nearest.y = clampValue(y, room_min_y_, room_max_y_);
            };

        if (std::fabs(ray_x) >= epsilon) {
            double t = (room_min_x_ - robot_pose.x) / ray_x;
            consider(WALL_LEFT, t, room_min_x_,
                     robot_pose.y + t * ray_y);
            t = (room_max_x_ - robot_pose.x) / ray_x;
            consider(WALL_RIGHT, t, room_max_x_,
                     robot_pose.y + t * ray_y);
        }
        if (std::fabs(ray_y) >= epsilon) {
            double t = (room_min_y_ - robot_pose.y) / ray_y;
            consider(WALL_BOTTOM, t,
                     robot_pose.x + t * ray_x, room_min_y_);
            t = (room_max_y_ - robot_pose.y) / ray_y;
            consider(WALL_TOP, t,
                     robot_pose.x + t * ray_x, room_max_y_);
        }

        if (!nearest.valid) return false;
        estimate = nearest;
        return true;
    }

    int segmentIndexForWall(WallType wall) const {
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            if (segments_[i].wall == wall) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool makeDockingObservation(
            int segment_index,
            const std::string& category,
            const BoardBoundaryEstimate& board_estimate,
            TargetObservation& observation) const {
        observation = TargetObservation();
        if (!board_estimate.valid) return false;

        observation.valid = true;
        observation.pose.x = board_estimate.x;
        observation.pose.y = board_estimate.y;
        switch (board_estimate.wall) {
            case WALL_TOP:
                observation.pose.y = room_max_y_ - docking_standoff_;
                observation.pose.yaw = 0.5 * kPi;
                break;
            case WALL_RIGHT:
                observation.pose.x = room_max_x_ - docking_standoff_;
                observation.pose.yaw = 0.0;
                break;
            case WALL_BOTTOM:
                observation.pose.y = room_min_y_ + docking_standoff_;
                observation.pose.yaw = -0.5 * kPi;
                break;
            case WALL_LEFT:
                observation.pose.x = room_min_x_ + docking_standoff_;
                observation.pose.yaw = kPi;
                break;
        }
        if (!isInsideRoom(observation.pose.x, observation.pose.y)) {
            return false;
        }
        observation.segment_index = segment_index;
        observation.category = category;
        return true;
    }

    bool isDuplicateBoard(const BoardBoundaryEstimate& estimate) const {
        if (!estimate.valid) return false;
        for (std::size_t i = 0; i < seen_board_coordinates_.size(); ++i) {
            const BoardBoundaryEstimate& previous =
                seen_board_coordinates_[i];
            if (!previous.valid || previous.wall != estimate.wall) continue;
            if (distance2D(previous.x, previous.y,
                           estimate.x, estimate.y) <=
                duplicate_coordinate_distance_) {
                return true;
            }
        }
        return false;
    }

    int chooseNewBoardBox(const std::vector<Box>& boxes,
                          const Pose2D& pose) const {
        const double image_center = 0.5 * static_cast<double>(image_width_);
        const double scan_right_limit =
            (2.0 / 3.0) * static_cast<double>(image_width_);
        int selected = -1;
        double best_error = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            // 巡检阶段使用图像左侧三分之二区域。这样能看到转角第一格
            // 和第二格；停靠横移阶段仍使用完整画面跟踪目标。
            if (boxes[i].centerX() >= scan_right_limit) continue;
            BoardBoundaryEstimate estimate;
            if (estimateBoardBoundary(pose, boxes[i], estimate) &&
                isDuplicateBoard(estimate)) {
                continue;
            }
            const double error =
                std::fabs(boxes[i].centerX() - image_center);
            if (error < best_error) {
                best_error = error;
                selected = static_cast<int>(i);
            }
        }
        return selected;
    }

    SegmentResult patrolSegment(std::size_t segment_index) {
        const Segment& segment = segments_[segment_index];
        resetWallFollower();
        ROS_INFO("开始%s：(%.2f, %.2f)→(%.2f, %.2f)，"
                 "前进朝向%.1f度，墙在车体左侧",
                 segment.name.c_str(), segment.start_x, segment.start_y,
                 segment.end_x, segment.end_y,
                 segment.travel_yaw * 180.0 / kPi);
        ros::Rate rate(control_rate_);

        while (ros::ok()) {
            ros::spinOnce();

            // 正常情况下，现实目标停靠成功后会立即处理此前记录的
            // 仿真目标。这里再做一层状态兜底：即使控制流意外返回
            // 巡检循环，也必须先消费待停靠目标，不能继续沿路线探测。
            if (hasPendingSimulationTarget()) {
                ROS_INFO("检测到待停靠的已记录仿真目标，暂停巡检并立即前往");
                if (!dockPendingSimulationTarget()) {
                    return SEGMENT_ABORTED;
                }
                return SEGMENT_MISSION_COMPLETE;
            }

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }
            const double progress = segmentProgress(segment, pose);
            if (progress >= segment.length - segment_end_tolerance_) {
                stopRobot();
                ROS_INFO("%s完成", segment.name.c_str());
                if (hasDeferredRealTargetAfterSegment(segment_index)) {
                    const DetectionResult deferred_result =
                        dockDeferredRealTargetAfterSegment(segment_index);
                    if (deferred_result == DETECTION_MISSION_COMPLETE) {
                        return SEGMENT_MISSION_COMPLETE;
                    }
                    if (deferred_result == DETECTION_ABORT) {
                        return SEGMENT_ABORTED;
                    }
                }
                return SEGMENT_COMPLETE;
            }

            publishPatrolCommand(segment, pose);

            std::vector<Box> boxes;
            const bool detection_ok = detectBoxes(boxes);
            if (!detection_ok || boxes.empty()) {
                rate.sleep();
                continue;
            }

            const int selected =
                chooseNewBoardBox(boxes, pose);
            if (selected < 0) {
                ROS_INFO_THROTTLE(
                    1.0,
                    "当前NanoDet框均在图像右侧三分之一区域，"
                    "或位于已记录板的"
                    "%.2f米重复范围内",
                    duplicate_coordinate_distance_);
                rate.sleep();
                continue;
            }
            const DetectionResult detection_result =
                handleDetectedBoard(segment_index, segment, pose,
                                    boxes[static_cast<std::size_t>(selected)]);
            if (detection_result == DETECTION_MISSION_COMPLETE) {
                return SEGMENT_MISSION_COMPLETE;
            }
            if (detection_result == DETECTION_ABORT) {
                return SEGMENT_ABORTED;
            }

            rate.sleep();
        }
        return SEGMENT_ABORTED;
    }

    DetectionResult handleDetectedBoard(std::size_t segment_index,
                                        const Segment& segment,
                                        const Pose2D& detection_pose,
                                        const Box& trigger_box) {
        stopRobot();
        ROS_INFO("NanoDet发现文字框，已停车；等待%.2f秒后调用OCR",
                 settle_time_);
        ros::Duration(settle_time_).sleep();

        Pose2D stopped_pose = detection_pose;
        getRobotPose(stopped_pose);

        BoardBoundaryEstimate boundary_estimate;
        if (estimateBoardBoundary(stopped_pose,
                                  trigger_box, boundary_estimate)) {
            if (isDuplicateBoard(boundary_estimate)) {
                ROS_INFO("估计坐标%s(%.3f, %.3f)距已记录板不超过%.2fm，"
                         "判定为重复检测并跳过OCR",
                         wallName(boundary_estimate.wall),
                         boundary_estimate.x, boundary_estimate.y,
                         duplicate_coordinate_distance_);
                return DETECTION_CONTINUE;
            }
            seen_board_coordinates_.push_back(boundary_estimate);
            ROS_INFO("记录新文字板边缘估计坐标：%s(%.3f, %.3f)",
                     wallName(boundary_estimate.wall),
                     boundary_estimate.x, boundary_estimate.y);
        } else {
            ROS_WARN("无法由NanoDet框中心估计墙面坐标，"
                     "本次继续OCR但不加入重复坐标表");
        }

        OcrRecord ocr = recognizeStaticTarget(trigger_box);
        // 与已经实测稳定的target_scan_test保持一致：以可分类关键词为准。
        // 某些OCR服务版本即使返回了有效文字，success字段也可能未置true。
        if (ocr.category == "unknown") {
            ROS_WARN("本次文字无法分类，忽略并继续%s", segment.name.c_str());
            return DETECTION_CONTINUE;
        }

        // 停车后的OCR框更接近静止图像，用它重新估计板在场地边界
        // 上的位置；失败时才退回触发本次停车的NanoDet框。
        BoardBoundaryEstimate static_board_estimate;
        bool estimate_ok =
            estimateBoardBoundary(stopped_pose,
                                  ocr.box, static_board_estimate);
        if (!estimate_ok && boundary_estimate.valid) {
            static_board_estimate = boundary_estimate;
            estimate_ok = true;
            ROS_WARN("静止OCR框无法计算板坐标，"
                     "改用NanoDet触发框的墙面交点");
        }

        TargetObservation observation;
        const int target_segment_index =
            estimate_ok ? segmentIndexForWall(static_board_estimate.wall) : -1;
        if (!estimate_ok ||
            target_segment_index < 0 ||
            !makeDockingObservation(
                target_segment_index, ocr.category,
                static_board_estimate, observation)) {
            ROS_WARN("无法由框中心生成距墙%.2fm的停靠导航点，"
                     "本次不使用小车当前位置，继续巡检等待重新检测",
                     docking_standoff_);
            return DETECTION_CONTINUE;
        }

        ROS_INFO("目标%s墙面坐标=%s(%.3f, %.3f)，"
                 "向场内延伸%.2fm后的move_base点=(%.3f, %.3f, %.1f度)",
                 categoryChinese(ocr.category),
                 wallName(static_board_estimate.wall),
                 static_board_estimate.x, static_board_estimate.y,
                 docking_standoff_,
                 observation.pose.x, observation.pose.y,
                 observation.pose.yaw * 180.0 / kPi);

        const bool target_on_current_left_wall =
            static_board_estimate.wall == segment.wall;
        if (!target_on_current_left_wall) {
            ROS_INFO("该目标位于%s，不是当前车体左侧正在扫描的%s",
                     wallName(static_board_estimate.wall),
                     wallName(segment.wall));
        }

        if (ocr.category == simulation_target_category_) {
            // 显式记录“仿真目标待停靠”状态。后续不再只依赖
            // simulation_observation_.valid在单个分支中临时跳转。
            // 保持旧逻辑：仿真目标先于现实目标出现时，只保存第一次
            // 有效记录，避免后续误识别覆盖已经确认的停靠点。
            if (!simulation_observation_.valid) {
                simulation_observation_ = observation;
            }
            simulation_target_pending_ = true;

            if (!real_docked_) {
                ROS_INFO(
                    "已记录仿真目标%s并置为待停靠："
                    "墙面估计坐标%s(%.3f, %.3f)，"
                    "对应停靠导航点(%.3f, %.3f, %.1f度)",
                    categoryChinese(ocr.category),
                    wallName(static_board_estimate.wall),
                    static_board_estimate.x,
                    static_board_estimate.y,
                    simulation_observation_.pose.x,
                    simulation_observation_.pose.y,
                    simulation_observation_.pose.yaw * 180.0 / kPi);
                if (target_on_current_left_wall) {
                    ROS_INFO("现实目标尚未停靠，继续巡检寻找现实目标");
                } else {
                    ROS_INFO("仿真目标不在当前左墙，仅保存坐标；"
                             "完成现实目标停靠后再前往");
                }
                return DETECTION_CONTINUE;
            }

            if (!dockPendingSimulationTarget()) {
                return DETECTION_ABORT;
            }
            return DETECTION_MISSION_COMPLETE;
        }

        if (ocr.category == real_target_category_) {
            if (real_docked_) {
                ROS_INFO("现实目标已经完成停靠，忽略重复识别");
                return DETECTION_CONTINUE;
            }

            if (!target_on_current_left_wall) {
                if (!real_target_pending_) {
                    real_observation_ = observation;
                    real_target_pending_ = true;
                    real_target_defer_segment_index_ =
                        static_cast<int>(segment_index);
                    ROS_INFO(
                        "现实目标位于非当前左墙，已保存停靠点"
                        "(%.3f, %.3f, %.1f度)；"
                        "继续完成%s的整段扫描，段末再前往停靠",
                        observation.pose.x,
                        observation.pose.y,
                        observation.pose.yaw * 180.0 / kPi,
                        segment.name.c_str());
                } else {
                    ROS_INFO("已有一个待处理的非当前左墙现实目标，"
                             "保留首次有效坐标并继续当前边界扫描");
                }
                return DETECTION_CONTINUE;
            }

            if (real_target_pending_) {
                ROS_INFO("现实目标已作为非当前左墙目标记录，"
                         "等待本段扫描完成后统一停靠");
                return DETECTION_CONTINUE;
            }

            real_observation_ = observation;
            if (!navigateToPose(observation.pose.x,
                                observation.pose.y,
                                observation.pose.yaw,
                                "前往现实目标停靠导航点")) {
                return DETECTION_ABORT;
            }
            if (!dockTarget(observation, "现实目标", true)) {
                return DETECTION_ABORT;
            }
            real_docked_ = true;

            if (hasPendingSimulationTarget()) {
                ROS_INFO("仿真目标此前已找到；现实目标停靠完成后"
                         "立即处理待停靠目标，不恢复巡检");
                if (!dockPendingSimulationTarget()) {
                    return DETECTION_ABORT;
                }
                return DETECTION_MISSION_COMPLETE;
            }

            ROS_INFO("尚未记录仿真目标；现实目标停靠后直接直线后退"
                     "至当前平移巡检线");
            if (!retreatToPatrolLine(segment)) {
                return DETECTION_ABORT;
            }
            if (!rotateToYaw(segment.travel_yaw)) {
                ROS_ERROR("回到巡检线后无法恢复前进朝向");
                return DETECTION_ABORT;
            }
            if (!openCamera()) return DETECTION_ABORT;
            ROS_INFO("已回到%s，继续寻找仿真目标",
                     segment.name.c_str());
            return DETECTION_CONTINUE;
        }

        ROS_INFO("OCR结果%s既不是现实目标也不是仿真目标，直接忽略",
                 categoryChinese(ocr.category));
        return DETECTION_CONTINUE;
    }

    bool hasDeferredRealTargetAfterSegment(
            std::size_t segment_index) const {
        return real_target_pending_ &&
               real_observation_.valid &&
               !real_docked_ &&
               real_target_defer_segment_index_ ==
                   static_cast<int>(segment_index);
    }

    DetectionResult dockDeferredRealTargetAfterSegment(
            std::size_t completed_segment_index) {
        if (!hasDeferredRealTargetAfterSegment(
                completed_segment_index)) {
            ROS_ERROR("待处理现实目标状态无效");
            return DETECTION_ABORT;
        }

        ROS_INFO("当前左侧边界已经完整扫描，"
                 "现在前往此前记录的非当前左墙现实目标："
                 "(%.3f, %.3f, %.1f度)",
                 real_observation_.pose.x,
                 real_observation_.pose.y,
                 real_observation_.pose.yaw * 180.0 / kPi);
        if (!navigateToPose(real_observation_.pose.x,
                            real_observation_.pose.y,
                            real_observation_.pose.yaw,
                            "前往延后处理的现实目标")) {
            return DETECTION_ABORT;
        }
        if (!dockTarget(real_observation_,
                        "延后处理的现实目标", true)) {
            return DETECTION_ABORT;
        }

        real_docked_ = true;
        real_target_pending_ = false;
        real_target_defer_segment_index_ = -1;

        if (hasPendingSimulationTarget()) {
            ROS_INFO("现实目标停靠完成，立即前往此前记录的仿真目标");
            if (!dockPendingSimulationTarget()) {
                return DETECTION_ABORT;
            }
            return DETECTION_MISSION_COMPLETE;
        }

        const std::size_t next_segment_index =
            completed_segment_index + 1;
        if (next_segment_index < segments_.size()) {
            const Segment& next_segment =
                segments_[next_segment_index];
            if (real_observation_.segment_index ==
                static_cast<int>(next_segment_index)) {
                ROS_INFO("尚未找到仿真目标；从现实目标处后退至下一条巡检线");
                if (!retreatToPatrolLine(next_segment)) {
                    return DETECTION_ABORT;
                }
                if (!rotateToYaw(next_segment.travel_yaw)) {
                    ROS_ERROR("回到下一条巡检线后无法恢复前进朝向");
                    return DETECTION_ABORT;
                }
            } else {
                ROS_WARN("延后现实目标不在紧邻的下一面墙，"
                         "改用move_base返回下一段起点");
                if (!navigateToPose(next_segment.start_x,
                                    next_segment.start_y,
                                    next_segment.travel_yaw,
                                    "返回下一段巡检起点")) {
                    return DETECTION_ABORT;
                }
            }
            if (!openCamera()) return DETECTION_ABORT;
            ROS_INFO("已准备进入%s，继续寻找仿真目标",
                     next_segment.name.c_str());
        }
        return DETECTION_CONTINUE;
    }

    bool hasPendingSimulationTarget() const {
        return real_docked_ &&
               simulation_target_pending_ &&
               simulation_observation_.valid &&
               !simulation_docked_;
    }

    bool dockPendingSimulationTarget() {
        if (!hasPendingSimulationTarget()) {
            ROS_ERROR("待停靠仿真目标状态无效："
                      "现实停靠=%s，仿真记录=%s，待停靠=%s，仿真停靠=%s",
                      real_docked_ ? "是" : "否",
                      simulation_observation_.valid ? "是" : "否",
                      simulation_target_pending_ ? "是" : "否",
                      simulation_docked_ ? "是" : "否");
            return false;
        }

        ROS_INFO("前往已记录仿真目标：导航点=(%.3f, %.3f, %.1f度)",
                 simulation_observation_.pose.x,
                 simulation_observation_.pose.y,
                 simulation_observation_.pose.yaw * 180.0 / kPi);

        if (!navigateToPose(simulation_observation_.pose.x,
                            simulation_observation_.pose.y,
                            simulation_observation_.pose.yaw,
                            "前往已记录的仿真目标")) {
            return false;
        }

        if (!dockTarget(simulation_observation_,
                        "仿真目标", camera_opened_)) {
            return false;
        }

        simulation_docked_ = true;
        simulation_target_pending_ = false;
        ROS_INFO("已完成待停靠仿真目标，双目标任务完成");
        return true;
    }

    bool dockTarget(const TargetObservation& observation,
                    const std::string& target_name,
                    bool camera_is_already_open) {
        if (!camera_is_already_open && !openCamera()) return false;
        ROS_INFO("%s开始二段式停靠：先横移视觉居中，再雷达直线逼近",
                 target_name.c_str());
        if (!alignBoardLaterally(observation.pose.yaw)) {
            ROS_ERROR("%s横移居中失败", target_name.c_str());
            return false;
        }
        closeCamera();
        if (!approachBoardWithLidar(observation.pose.yaw)) {
            ROS_ERROR("%s雷达逼近失败", target_name.c_str());
            return false;
        }
        ROS_INFO("%s停靠成功", target_name.c_str());
        return true;
    }

    bool retreatToPatrolLine(const Segment& segment) {
        ROS_INFO("开始原地直线后退至%s，目标线=(%.2f, %.2f)→(%.2f, %.2f)",
                 segment.name.c_str(),
                 segment.start_x, segment.start_y,
                 segment.end_x, segment.end_y);
        const double normal_x = std::cos(segment.docking_yaw);
        const double normal_y = std::sin(segment.docking_yaw);
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(retreat_timeout_);
        ros::Rate rate(20.0);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            // 面向墙时，靠近墙的一侧为正；后退直到回到巡检线。
            const double distance_to_line =
                (pose.x - segment.start_x) * normal_x +
                (pose.y - segment.start_y) * normal_y;
            const double yaw_error =
                normalizeAngle(segment.docking_yaw - pose.yaw);
            const double angular_z =
                clampValue(docking_yaw_kp_ * yaw_error,
                           -docking_yaw_max_speed_,
                           docking_yaw_max_speed_);

            if (distance_to_line <= retreat_line_tolerance_) {
                stopRobot();
                ROS_INFO("直线后退完成：距巡检线有向距离=%.3fm",
                         distance_to_line);
                return true;
            }

            double backward_speed = retreat_max_speed_;
            if (distance_to_line < retreat_slow_distance_) {
                const double ratio =
                    clampValue(
                        (distance_to_line - retreat_line_tolerance_) /
                            (retreat_slow_distance_ -
                             retreat_line_tolerance_),
                        0.0, 1.0);
                backward_speed =
                    retreat_min_speed_ +
                    ratio * (retreat_max_speed_ -
                             retreat_min_speed_);
            }
            publishVelocity(-backward_speed, 0.0, angular_z);
            ROS_INFO_THROTTLE(
                0.5,
                "直线后退：距巡检线=%.3fm，linear.x=%.3f，angular.z=%.3f",
                distance_to_line, -backward_speed, angular_z);
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("直线后退至%s超时", segment.name.c_str());
        return false;
    }

    bool openCamera() {
        if (camera_opened_) return true;
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -1;
        if (!detect_client_.call(service)) {
            ROS_ERROR("打开NanoDet摄像头失败");
            return false;
        }
        camera_opened_ = true;
        return true;
    }

    void closeCamera() {
        if (!camera_opened_) return;
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -2;
        detect_client_.call(service);
        camera_opened_ = false;
    }

    bool detectBoxes(std::vector<Box>& boxes) {
        boxes.clear();
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = 1;
        const ros::WallTime begin = ros::WallTime::now();
        if (!detect_client_.call(service)) {
            ROS_WARN_THROTTLE(1.0, "调用/nanodet_detect失败");
            return false;
        }
        const double elapsed =
            (ros::WallTime::now() - begin).toSec();
        if (elapsed > max_detection_duration_) {
            ROS_ERROR("NanoDet耗时%.3f秒，超过%.3f秒，过期结果已丢弃",
                      elapsed, max_detection_duration_);
            return false;
        }

        const std::size_t count = std::min(
            std::min(service.response.x0.size(), service.response.y0.size()),
            std::min(service.response.x1.size(), service.response.y1.size()));
        for (std::size_t i = 0; i < count; ++i) {
            Box box;
            box.class_id = i < service.response.class_name.size()
                               ? service.response.class_name[i] : 0;
            box.x0 = service.response.x0[i];
            box.y0 = service.response.y0[i];
            box.x1 = service.response.x1[i];
            box.y1 = service.response.y1[i];
            if (box.x1 > box.x0 && box.y1 > box.y0) boxes.push_back(box);
        }
        return true;
    }

    int chooseClosestCenterBox(const std::vector<Box>& boxes) const {
        const double center = 0.5 * image_width_;
        int selected = -1;
        double best_error = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double error = std::fabs(boxes[i].centerX() - center);
            if (error < best_error) {
                best_error = error;
                selected = static_cast<int>(i);
            }
        }
        return selected;
    }

    static double intersectionOverUnion(const Box& first, const Box& second) {
        const int left = std::max(first.x0, second.x0);
        const int top = std::max(first.y0, second.y0);
        const int right = std::min(first.x1, second.x1);
        const int bottom = std::min(first.y1, second.y1);
        const double intersection =
            static_cast<double>(std::max(0, right - left) *
                                std::max(0, bottom - top));
        const double union_area =
            first.width() * first.height() +
            second.width() * second.height() - intersection;
        return union_area > 0.0 ? intersection / union_area : 0.0;
    }

    int associateSelectedBox(const std::vector<Box>& boxes,
                             const Box& previous) const {
        int best_iou_index = -1;
        double best_iou = 0.0;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double iou = intersectionOverUnion(boxes[i], previous);
            if (iou > best_iou) {
                best_iou = iou;
                best_iou_index = static_cast<int>(i);
            }
        }
        if (best_iou_index >= 0 && best_iou >= 0.05) {
            return best_iou_index;
        }

        // IoU可能在横移速度较快、检测框尺寸突变或目标跨过画面中线时
        // 瞬间降为0。此时继续按框中心距离寻找同一目标。
        int nearest = -1;
        double nearest_distance =
            std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double dx = boxes[i].centerX() - previous.centerX();
            const double dy = boxes[i].centerY() - previous.centerY();
            const double distance = std::hypot(dx, dy);
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest = static_cast<int>(i);
            }
        }

        // 旧逻辑把max_track_jump_px_作为硬门限：单帧跳动一旦超过
        // 门限就返回-1，而tracked_box仍停留在旧框，导致后续即使
        // NanoDet每帧都检测到目标也无法重新关联，并被累计成连续丢失。
        // 现在该参数只触发重新捕获日志，不再屏蔽全画面（尤其右侧
        // 三分之一）中仍然有效的检测框。
        if (nearest >= 0 && nearest_distance > max_track_jump_px_) {
            ROS_WARN_THROTTLE(
                1.0,
                "横移居中：目标框跳动%.1fpx超过%.1fpx，"
                "已使用全画面最近框重新捕获",
                nearest_distance, max_track_jump_px_);
        }
        return nearest;
    }

    OcrRecord recognizeStaticTarget(const Box& trigger_box) {
        OcrRecord best_any;
        OcrRecord best_keyword;
        bool have_any = false;
        bool have_keyword = false;
        Box reference = trigger_box;

        ros_nanodet::ocr_result_srv clear_service;
        clear_service.request.command = -3;
        if (!ocr_client_.call(clear_service)) {
            ROS_WARN("OCR缓冲帧清理失败，将继续识别");
        }

        for (int attempt = 0; attempt < ocr_attempts_ && ros::ok(); ++attempt) {
            ros_nanodet::ocr_result_srv service;
            service.request.command = 1;
            if (!ocr_client_.call(service)) {
                ROS_WARN("第%d次OCR服务调用失败", attempt + 1);
                ros::Duration(ocr_retry_interval_).sleep();
                continue;
            }
            const std::size_t count = std::min(
                std::min(service.response.text.size(),
                         service.response.confidence.size()),
                std::min(
                    std::min(service.response.x0.size(),
                             service.response.y0.size()),
                    std::min(service.response.x1.size(),
                             service.response.y1.size())));
            int selected = -1;
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < count; ++i) {
                const double center_x =
                    0.5 * (service.response.x0[i] +
                           service.response.x1[i]);
                const double center_y =
                    0.5 * (service.response.y0[i] +
                           service.response.y1[i]);
                const double distance =
                    std::hypot(center_x - reference.centerX(),
                               center_y - reference.centerY());
                if (distance < nearest) {
                    nearest = distance;
                    selected = static_cast<int>(i);
                }
            }
            if (selected >= 0) {
                const std::size_t i = static_cast<std::size_t>(selected);
                OcrRecord candidate;
                candidate.success = service.response.success;
                candidate.text = service.response.text[i];
                candidate.category = classifyText(candidate.text);
                candidate.confidence = service.response.confidence[i];
                candidate.box = Box{0, service.response.x0[i],
                                    service.response.y0[i],
                                    service.response.x1[i],
                                    service.response.y1[i]};
                reference = candidate.box;
                ROS_INFO("OCR第%d/%d次：%s，类别=%s，置信度=%.3f",
                         attempt + 1, ocr_attempts_,
                         candidate.text.c_str(),
                         candidate.category.c_str(),
                         candidate.confidence);
                if (!candidate.text.empty() &&
                    (!have_any ||
                     candidate.confidence > best_any.confidence)) {
                    best_any = candidate;
                    have_any = true;
                }
                if (candidate.category != "unknown" &&
                    (!have_keyword ||
                     candidate.confidence > best_keyword.confidence)) {
                    best_keyword = candidate;
                    have_keyword = true;
                }
            } else {
                ROS_WARN("第%d次OCR没有返回文字框", attempt + 1);
            }
            ros::Duration(ocr_retry_interval_).sleep();
        }

        OcrRecord result;
        if (have_keyword) result = best_keyword;
        else if (have_any) result = best_any;
        else result.box = trigger_box;
        ROS_INFO("OCR最终结果：%s，分类=%s",
                 result.text.c_str(),
                 categoryChinese(result.category));
        return result;
    }

    bool alignBoardLaterally(double desired_yaw) {
        if (!openCamera()) return false;
        ROS_INFO("横移居中：保持朝向%.1f度",
                 desired_yaw * 180.0 / kPi);
        const double image_center = 0.5 * image_width_;
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(lateral_align_timeout_);
        bool have_tracked_box = false;
        Box tracked_box{0, 0, 0, 0, 0};
        int stable_frames = 0;
        int lost_frames = 0;
        ros::Rate rate(15.0);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            std::vector<Box> boxes;
            int selected = -1;
            const bool detection_ok = detectBoxes(boxes);
            if (detection_ok) {
                selected = have_tracked_box
                               ? associateSelectedBox(boxes, tracked_box)
                               : chooseClosestCenterBox(boxes);
            }
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }
            const double yaw_error =
                normalizeAngle(desired_yaw - pose.yaw);
            const double angular_z =
                clampValue(docking_yaw_kp_ * yaw_error,
                           -docking_yaw_max_speed_,
                           docking_yaw_max_speed_);
            if (selected < 0) {
                ++lost_frames;
                stable_frames = 0;
                publishVelocity(0.0, 0.0, angular_z);
                if (detection_ok && boxes.empty()) {
                    ROS_WARN_THROTTLE(
                        0.5,
                        "横移居中：NanoDet本帧未返回目标框，"
                        "连续空帧=%d/%d",
                        lost_frames, max_lost_frames_ + 1);
                } else if (!detection_ok) {
                    ROS_WARN_THROTTLE(
                        0.5,
                        "横移居中：本帧检测服务失败或结果超时，"
                        "连续异常帧=%d/%d",
                        lost_frames, max_lost_frames_ + 1);
                }
                if (lost_frames > max_lost_frames_) {
                    stopRobot();
                    ROS_ERROR("横移居中时连续丢失目标板");
                    return false;
                }
                rate.sleep();
                continue;
            }
            tracked_box = boxes[static_cast<std::size_t>(selected)];
            have_tracked_box = true;
            lost_frames = 0;
            const double pixel_error =
                image_center - tracked_box.centerX();

            if (std::fabs(pixel_error) <=
                lateral_center_tolerance_px_) {
                publishVelocity(0.0, 0.0, angular_z);
                if (std::fabs(yaw_error) <= 2.0 * kPi / 180.0) {
                    ++stable_frames;
                } else {
                    stable_frames = 0;
                }
                if (stable_frames >= lateral_stable_frames_) {
                    stopRobot();
                    ROS_INFO("横移居中完成：目标中心x=%.1f",
                             tracked_box.centerX());
                    return true;
                }
            } else {
                stable_frames = 0;
                double lateral_y =
                    clampValue(lateral_align_kp_ * pixel_error,
                               -lateral_align_max_speed_,
                               lateral_align_max_speed_);
                if (std::fabs(lateral_y) <
                    lateral_align_min_speed_) {
                    lateral_y = lateral_y >= 0.0
                                    ? lateral_align_min_speed_
                                    : -lateral_align_min_speed_;
                }
                publishVelocity(0.0, lateral_y, angular_z);
                ROS_INFO_THROTTLE(
                    0.5,
                    "横移居中：目标x=%.1f，linear.y=%.3f，angular.z=%.3f",
                    tracked_box.centerX(), lateral_y, angular_z);
            }
            rate.sleep();
        }
        stopRobot();
        ROS_ERROR("横移居中超时");
        return false;
    }

    bool getFrontMinimumDistance(double& minimum_distance) {
        if (!have_laser_scan_ ||
            (ros::WallTime::now() - latest_scan_wall_time_).toSec() >
                lidar_data_timeout_ ||
            latest_scan_.ranges.empty() ||
            std::fabs(latest_scan_.angle_increment) < 1e-12) {
            return false;
        }
        int center_index = static_cast<int>(std::lround(
            -latest_scan_.angle_min / latest_scan_.angle_increment));
        center_index =
            std::max(0, std::min(
                center_index,
                static_cast<int>(latest_scan_.ranges.size()) - 1));
        if (!lidar_layout_logged_) {
            ROS_INFO("雷达点数=%zu，正前方索引=%d，使用[%d, %d]",
                     latest_scan_.ranges.size(), center_index,
                     std::max(0, center_index -
                                     front_lidar_half_window_),
                     std::min(
                         static_cast<int>(latest_scan_.ranges.size()) - 1,
                         center_index + front_lidar_half_window_));
            lidar_layout_logged_ = true;
        }
        const int first =
            std::max(0, center_index - front_lidar_half_window_);
        const int last =
            std::min(static_cast<int>(latest_scan_.ranges.size()) - 1,
                     center_index + front_lidar_half_window_);
        minimum_distance = std::numeric_limits<double>::infinity();
        int valid_count = 0;
        for (int i = first; i <= last; ++i) {
            const double range =
                latest_scan_.ranges[static_cast<std::size_t>(i)];
            if (!std::isfinite(range) ||
                range < latest_scan_.range_min ||
                range > latest_scan_.range_max ||
                range <= 0.0) {
                continue;
            }
            minimum_distance = std::min(minimum_distance, range);
            ++valid_count;
        }
        return valid_count > 0;
    }

    bool approachBoardWithLidar(double desired_yaw) {
        ROS_INFO("雷达直线逼近：正前方最小距离达到%.3fm停车",
                 approach_stop_distance_);
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(approach_timeout_);
        ros::WallTime last_valid_lidar = ros::WallTime::now();
        ros::Rate rate(20.0);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();
            double front_distance = 0.0;
            if (!getFrontMinimumDistance(front_distance)) {
                publishVelocity(0.0, 0.0, 0.0);
                if ((ros::WallTime::now() -
                     last_valid_lidar).toSec() >
                    lidar_loss_abort_timeout_) {
                    stopRobot();
                    ROS_ERROR("连续%.2f秒没有有效正前方雷达数据",
                              lidar_loss_abort_timeout_);
                    return false;
                }
                rate.sleep();
                continue;
            }
            last_valid_lidar = ros::WallTime::now();
            if (front_distance <= approach_stop_distance_) {
                stopRobot();
                final_front_distance_ = front_distance;
                ROS_INFO("雷达逼近完成：正前方最小距离=%.3fm",
                         front_distance);
                return true;
            }

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }
            const double yaw_error =
                normalizeAngle(desired_yaw - pose.yaw);
            const double angular_z =
                clampValue(docking_yaw_kp_ * yaw_error,
                           -docking_yaw_max_speed_,
                           docking_yaw_max_speed_);
            double forward_speed = approach_max_speed_;
            if (front_distance < approach_slow_distance_) {
                const double ratio =
                    clampValue(
                        (front_distance - approach_stop_distance_) /
                            (approach_slow_distance_ -
                             approach_stop_distance_),
                        0.0, 1.0);
                forward_speed =
                    approach_min_speed_ +
                    ratio * (approach_max_speed_ -
                             approach_min_speed_);
            }
            publishVelocity(forward_speed, 0.0, angular_z);
            ROS_INFO_THROTTLE(
                0.5,
                "雷达逼近：距离=%.3f，linear.x=%.3f，angular.z=%.3f",
                front_distance, forward_speed, angular_z);
            rate.sleep();
        }
        stopRobot();
        ROS_ERROR("雷达直线逼近超时");
        return false;
    }

    void scanCallback(
        const sensor_msgs::LaserScan::ConstPtr& message) {
        latest_scan_ = *message;
        latest_scan_wall_time_ = ros::WallTime::now();
        have_laser_scan_ = true;
    }

    void publishVelocity(double linear_x, double linear_y,
                         double angular_z) {
        ucarmain2026::set_speed service;
        service.request.target_twist.linear.x = linear_x;
        service.request.target_twist.linear.y = linear_y;
        service.request.target_twist.linear.z = 0.0;
        service.request.target_twist.angular.x = 0.0;
        service.request.target_twist.angular.y = 0.0;
        service.request.target_twist.angular.z = angular_z;
        service.request.work = true;
        service.request.movebase_flag = false;
        service.request.target_x = 0.0;
        service.request.target_y = 0.0;
        service.request.target_yaw = 0.0;
        if (!set_speed_client_.call(service) ||
            !service.response.success) {
            ROS_ERROR_THROTTLE(1.0, "调用/set_speed失败");
        }
    }

    void stopRobot() {
        if (!set_speed_client_.exists()) return;
        ucarmain2026::set_speed service;
        service.request.target_twist = geometry_msgs::Twist();
        service.request.work = true;
        service.request.movebase_flag = false;
        service.request.target_x = 0.0;
        service.request.target_y = 0.0;
        service.request.target_yaw = 0.0;
        set_speed_client_.call(service);
        ros::Duration(0.12).sleep();
        service.request.work = false;
        if (!set_speed_client_.call(service)) {
            ROS_WARN_THROTTLE(1.0, "停止/set_speed控制失败");
        }
    }

    void printSummary(bool success) const {
        ROS_INFO("================ 找板停靠结果 ================");
        ROS_INFO("现实目标：%s；找到=%s；停靠=%s",
                 categoryChinese(real_target_category_),
                 real_observation_.valid ? "是" : "否",
                 real_docked_ ? "成功" : "未完成");
        ROS_INFO("现实目标延后停靠状态：%s",
                 real_target_pending_ ? "是" : "否");
        ROS_INFO("仿真目标：%s；找到=%s；停靠=%s",
                 categoryChinese(simulation_target_category_),
                 simulation_observation_.valid ? "是" : "否",
                 simulation_docked_ ? "成功" : "未完成");
        ROS_INFO("仿真目标待停靠状态：%s",
                 simulation_target_pending_ ? "是" : "否");
        ROS_INFO("任务总结果：%s", success ? "成功" : "失败");
        ROS_INFO("==============================================");
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    MoveBaseClient move_base_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::ServiceClient detect_client_;
    ros::ServiceClient ocr_client_;
    ros::ServiceClient set_speed_client_;
    ros::Subscriber scan_subscriber_;

    std::string real_target_category_;
    std::string simulation_target_category_;
    std::string map_frame_;
    std::string base_frame_;
    std::string scan_topic_;

    double room_min_x_;
    double room_max_x_;
    double room_min_y_;
    double room_max_y_;
    double start_x_;
    double start_y_;
    double start_yaw_deg_;
    double navigation_timeout_;

    double patrol_forward_speed_;
    double line_hold_kp_;
    double line_hold_max_speed_;
    double yaw_hold_kp_;
    double yaw_hold_max_speed_;
    double yaw_pause_threshold_deg_;
    double segment_end_tolerance_;
    double control_rate_;
    bool patrol_use_lidar_wall_follow_;
    double wall_target_distance_;
    double wall_distance_kp_;
    double wall_angle_kp_;
    double wall_max_lateral_speed_;
    double wall_max_angular_speed_;
    double wall_scan_min_angle_deg_;
    double wall_scan_max_angle_deg_;
    double wall_scan_min_range_;
    double wall_scan_max_range_;
    double wall_fit_inlier_threshold_;
    int wall_fit_min_points_;
    int wall_fit_min_inliers_;
    double wall_fit_min_span_;
    double wall_fit_max_angle_deg_;
    double wall_fit_max_rms_;
    double wall_fit_filter_alpha_;
    double rotate_kp_;
    double rotate_min_speed_;
    double rotate_max_speed_;
    double rotate_tolerance_deg_;
    int rotate_stable_frames_;
    double rotate_timeout_;
    double position_hold_kp_;
    double position_hold_max_speed_;

    int image_width_;
    double camera_fx_;
    double camera_yaw_offset_deg_;
    double docking_standoff_;
    double settle_time_;
    int ocr_attempts_;
    double ocr_retry_interval_;
    double max_detection_duration_;
    double duplicate_coordinate_distance_;
    double max_track_jump_px_;
    int max_lost_frames_;
    double retreat_line_tolerance_;
    double retreat_slow_distance_;
    double retreat_min_speed_;
    double retreat_max_speed_;
    double retreat_timeout_;

    double lateral_align_kp_;
    double lateral_align_min_speed_;
    double lateral_align_max_speed_;
    double lateral_center_tolerance_px_;
    int lateral_stable_frames_;
    double lateral_align_timeout_;
    int front_lidar_half_window_;
    double approach_stop_distance_;
    double approach_slow_distance_;
    double approach_min_speed_;
    double approach_max_speed_;
    double approach_timeout_;
    double lidar_data_timeout_;
    double lidar_loss_abort_timeout_;
    double docking_yaw_kp_;
    double docking_yaw_max_speed_;

    std::vector<Segment> segments_;
    TargetObservation real_observation_;
    TargetObservation simulation_observation_;
    bool configuration_valid_ = false;
    bool camera_opened_ = false;
    bool real_docked_ = false;
    bool simulation_docked_ = false;
    bool real_target_pending_ = false;
    int real_target_defer_segment_index_ = -1;
    bool simulation_target_pending_ = false;
    int current_segment_index_ = 0;
    std::vector<BoardBoundaryEstimate> seen_board_coordinates_;

    sensor_msgs::LaserScan latest_scan_;
    ros::WallTime latest_scan_wall_time_;
    bool have_laser_scan_ = false;
    bool lidar_layout_logged_ = false;
    double final_front_distance_ = -1.0;
    bool wall_fit_initialized_ = false;
    double filtered_wall_distance_ = 0.0;
    double filtered_wall_angle_ = 0.0;

};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "target_patrol_docking");
    TargetPatrolDocking node;
    return node.run() ? 0 : 1;
}