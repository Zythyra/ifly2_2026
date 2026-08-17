// 交付构建标识：IFLY2026_FIXED_PATH_PATROL_V14_9_UNKNOWN_CANDIDATE_REVISIT_20260817
// V14.9：基于V14.8，只增加unknown候选点兜底回访：
// 1) 巡检中某块板OCR最终仍为unknown时，记录该板墙面坐标为候选点；
//    候选点独立去重，不改变原seen重复屏蔽和当前巡检流程。
// 2) 四面墙完整巡检结束后，如果现实/仿真目标仍未全部完成，
//    按候选发现顺序逐个导航到该板的docking_standoff临时停靠点，
//    重新NanoDet+OCR；一旦分类为缺失目标，直接沿用现有两段式停靠。
// 3) 候选回访发生在整圈巡检完成之后，因此不再套用“非当前墙等段末”规则。
// V14.8视觉急停直接OCR、V14.7双停车保护、V14.6边缘30度修正和新起点、
// V14.5视觉减速、V14.4三保护、Camera Handoff、两段式停靠、
// V13角点、MyPlanner/C5均不改。


#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <ros_nanodet/detect_result_srv.h>
#include <ros_nanodet/ocr_result_srv.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ucarmain2026/set_speed.h>

#include <algorithm>
#include <cmath>
#include <clocale>
#include <cstdint>
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
        pnh_.param("start_x", start_x_, 1.25);
        pnh_.param("start_y", start_y_, 4.25);
        pnh_.param("start_yaw_deg", start_yaw_deg_, 0.0);
        pnh_.param("navigation_timeout", navigation_timeout_, 180.0);

        pnh_.param("planner_private_namespace", planner_private_namespace_,
                   std::string("/move_base/MyPlanner"));
        pnh_.param("patrol_path_spacing", patrol_path_spacing_, 0.02);
        pnh_.param("patrol_speed_limit", patrol_speed_limit_, 0.60);
        pnh_.param("normal_navigation_speed_limit",
                   normal_navigation_speed_limit_, 1.00);
        pnh_.param("patrol_cancel_timeout", patrol_cancel_timeout_, 3.0);
        pnh_.param("patrol_interface_timeout", patrol_interface_timeout_, 3.0);
        pnh_.param("move_base_reconfigure_service",
                   move_base_reconfigure_service_,
                   std::string("/move_base/set_parameters"));
        pnh_.param("disable_move_base_oscillation_during_patrol",
                   disable_move_base_oscillation_during_patrol_, true);
        pnh_.param("normal_move_base_oscillation_timeout",
                   normal_move_base_oscillation_timeout_, 8.0);
        pnh_.param("patrol_aborted_retry_count",
                   patrol_aborted_retry_count_, 2);

        pnh_.param("segment_end_tolerance", segment_end_tolerance_, 0.015);
        pnh_.param("control_rate", control_rate_, 15.0);

        //  V12：角点安全偏移与同时转向/平移控制参数。
        pnh_.param("patrol_transition_position_kp",
                   patrol_transition_position_kp_, 2.50);
        pnh_.param("patrol_transition_yaw_kp",
                   patrol_transition_yaw_kp_, 3.00);
        pnh_.param("patrol_transition_min_linear_speed",
                   patrol_transition_min_linear_speed_, 0.025);
        pnh_.param("patrol_transition_max_linear_speed",
                   patrol_transition_max_linear_speed_, 0.12);
        pnh_.param("patrol_transition_min_angular_speed",
                   patrol_transition_min_angular_speed_, 0.25);
        pnh_.param("patrol_transition_max_angular_speed",
                   patrol_transition_max_angular_speed_, 1.20);
        pnh_.param("patrol_transition_linear_accel",
                   patrol_transition_linear_accel_, 0.60);
        pnh_.param("patrol_transition_angular_accel",
                   patrol_transition_angular_accel_, 4.00);
        pnh_.param("patrol_transition_yaw_priority_start_deg",
                   patrol_transition_yaw_priority_start_deg_, 55.0);
        pnh_.param("patrol_transition_yaw_priority_release_deg",
                   patrol_transition_yaw_priority_release_deg_, 20.0);
        pnh_.param("patrol_transition_yaw_priority_min_linear_scale",
                   patrol_transition_yaw_priority_min_linear_scale_, 0.15);
        pnh_.param("patrol_transition_position_tolerance",
                   patrol_transition_position_tolerance_, 0.012);
        pnh_.param("patrol_transition_yaw_tolerance_deg",
                   patrol_transition_yaw_tolerance_deg_, 1.5);
        pnh_.param("patrol_transition_stable_frames",
                   patrol_transition_stable_frames_, 3);
        pnh_.param("patrol_transition_timeout",
                   patrol_transition_timeout_, 8.0);

        pnh_.param("image_width", image_width_, 640);
        pnh_.param("camera_fx", camera_fx_, 554.256);
        pnh_.param("camera_yaw_offset_deg", camera_yaw_offset_deg_, 0.0);
        pnh_.param("docking_standoff", docking_standoff_, 0.50);
        pnh_.param("settle_time", settle_time_, 0.25);
        pnh_.param("ocr_attempts", ocr_attempts_, 3);
        pnh_.param("ocr_retry_interval", ocr_retry_interval_, 0.12);
        pnh_.param("ocr_recovery_turn_deg", ocr_recovery_turn_deg_, 30.0);
        pnh_.param("ocr_recovery_turn_kp", ocr_recovery_turn_kp_, 2.0);
        pnh_.param("ocr_recovery_turn_min_speed",
                   ocr_recovery_turn_min_speed_, 0.18);
        pnh_.param("ocr_recovery_turn_max_speed",
                   ocr_recovery_turn_max_speed_, 0.45);
        pnh_.param("ocr_recovery_turn_tolerance_deg",
                   ocr_recovery_turn_tolerance_deg_, 1.5);
        pnh_.param("ocr_recovery_turn_stable_frames",
                   ocr_recovery_turn_stable_frames_, 3);
        pnh_.param("ocr_recovery_turn_timeout",
                   ocr_recovery_turn_timeout_, 5.0);
        pnh_.param("ocr_recovery_settle_time",
                   ocr_recovery_settle_time_, 0.35);
        pnh_.param("max_detection_duration", max_detection_duration_, 0.50);
        // V14.3保护1：仅当巡检NanoDet框中心x严格小于该像素值时，
        // 才允许中断固定巡检Path并停车进入OCR。默认640宽图像下为600。
        pnh_.param("patrol_stop_max_center_x",
                   patrol_stop_max_center_x_, 600.0);

        // V14.5：当前左侧巡检墙视觉目标接近减速。
        // 与line2o挡板减速同样使用线性速度比例：
        // 1.50m -> 1.00倍，0.50m -> 0.20倍，到0.50m取消巡检goal并停车OCR。
        pnh_.param("patrol_target_slowdown_start_distance",
                   patrol_target_slowdown_start_distance_, 1.50);
        pnh_.param("patrol_target_stop_distance",
                   patrol_target_stop_distance_, 0.50);
        pnh_.param("patrol_target_min_speed_ratio",
                   patrol_target_min_speed_ratio_, 0.20);

        // V14.7保护1：
        // 非当前巡检墙只有在机器人距该墙本身小于该阈值时才允许立即停车。
        pnh_.param("patrol_noncurrent_wall_stop_max_distance",
                   patrol_noncurrent_wall_stop_max_distance_, 1.50);

        // V14.7保护2：
        // 当前左侧巡检墙目标框的左边界距图像左缘不足该像素值时立即停车。
        pnh_.param("patrol_target_left_edge_stop_px",
                   patrol_target_left_edge_stop_px_, 100);

        pnh_.param("duplicate_coordinate_distance",
                   duplicate_coordinate_distance_, 0.50);
        pnh_.param("max_track_jump_px", max_track_jump_px_, 140.0);
        pnh_.param("max_lost_frames", max_lost_frames_, 4);
        pnh_.param("docking_recovery_turn_deg",
                   docking_recovery_turn_deg_, 30.0);
        pnh_.param("docking_recovery_detection_attempts",
                   docking_recovery_detection_attempts_, 3);
        pnh_.param("docking_recovery_detection_interval",
                   docking_recovery_detection_interval_, 0.0);
        // 到达第一段预停靠点后，第二次视觉只接受与第一次墙面估计
        // 同墙且坐标足够接近的框，防止把其他文字板当成当前目标。
        pnh_.param("docking_refine_max_board_shift",
                   docking_refine_max_board_shift_, 0.80);

        // V14.2：仅用于±30度恢复旋转后的即时缓存刷新。
        // 正常第一段导航期间摄像头会被关闭，因此到点重开后没有历史缓存。
        pnh_.param("docking_refresh_clear_calls",
                   docking_refresh_clear_calls_, 2);

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
        move_base_reconfigure_client_ =
            nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
                move_base_reconfigure_service_);
        patrol_path_publisher_ = nh_.advertise<nav_msgs::Path>(
            planner_private_namespace_ + "/patrol_path", 1, true);
        patrol_path_lock_client_ = nh_.serviceClient<std_srvs::SetBool>(
            planner_private_namespace_ + "/lock_patrol_path");
        controller_reset_client_ = nh_.serviceClient<std_srvs::Trigger>(
            planner_private_namespace_ + "/reset_controller_state");
        scan_subscriber_ =
            nh_.subscribe(scan_topic_, 1, &TargetPatrolDocking::scanCallback, this);
        ROS_INFO("车头向前巡检找板：现实目标=%s，仿真目标=%s",
                 categoryChinese(real_target_category_),
                 categoryChinese(simulation_target_category_));
        ROS_INFO("找板房间边界=[%.2f, %.2f]×[%.2f, %.2f]；"
                 "巡检视觉门限=centerX<%.1fpx；"
                 "当前左墙目标%.2fm开始减速，%.2fm处降到%.0f%%并停车；"
                 "当前墙框x0<%dpx立即停车；"
                 "非当前墙只有距该墙<%.2fm才立即停车；"
                 "停靠导航点距墙%.2fm",
                 room_min_x_, room_max_x_, room_min_y_, room_max_y_,
                 patrol_stop_max_center_x_,
                 patrol_target_slowdown_start_distance_,
                 patrol_target_stop_distance_,
                 patrol_target_min_speed_ratio_ * 100.0,
                 patrol_target_left_edge_stop_px_,
                 patrol_noncurrent_wall_stop_max_distance_,
                 docking_standoff_);
        ROS_WARN("IFLY2026_FIXED_PATH_PATROL_V14_TWO_STAGE_MOVEBASE_DOCK_20260812："
                 "target_patrol_docking 固定move_base路线巡检版已启动。"
                 "巡检速度=%.2f，普通导航速度=%.2f，路径间距=%.3f。",
                 patrol_speed_limit_, normal_navigation_speed_limit_,
                 patrol_path_spacing_);
        ROS_INFO("OCR单字分类：食→食品，日/用→日用品，"
                 "电/子/产/生→电子产品；"
                 "仅识别到品/加/工/车/间时逆时针转%.1f度复识一次。",
                 ocr_recovery_turn_deg_);
        ROS_INFO("停靠点视觉恢复：第一段预停靠后若当前朝向无法复定位原目标，"
                 "按到点朝向依次扫描左侧+%.1f度和右侧-%.1f度；"
                 "找到后直接计算最终move_base停靠点。",
                 docking_recovery_turn_deg_,
                 docking_recovery_turn_deg_);
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

        // V14.6：初始move_base直接到(1.25,4.25,0deg)，
        // 第一条固定Patrol Path也从该点开始，因此不再执行旧的
        // (0.25,4.25)->(0.25,4.30)额外安全横移。
        setShadowModeActiveOnce();
        if (!openCamera()) return false;

        for (std::size_t index = 0; index < segments_.size() && ros::ok();) {
            current_segment_index_ = static_cast<int>(index);
            const SegmentResult result = patrolSegment(index);
            if (result == SEGMENT_MISSION_COMPLETE) {
                finishPatrolMode();
                printSummary(true);
                return true;
            }
            if (result == SEGMENT_ABORTED) {
                finishPatrolMode();
                printSummary(false);
                return false;
            }
            ++index;
        }

        finishPatrolMode();
        stopRobot();
        closeCamera();

        // V14.9：只有四面墙完整巡检后任务仍未完成，才启动unknown候选回访。
        // 正常巡检过程中候选只记录，不改变任何既有停车/停靠决策。
        if ((!real_docked_ || !simulation_docked_) &&
            !unknown_candidates_.empty()) {
            ROS_WARN(
                "四面墙完整巡检结束但双目标尚未全部完成；"
                "已记录%zu个unknown候选，开始按发现顺序逐个回访。",
                unknown_candidates_.size());

            if (!revisitUnknownCandidates()) {
                finishPatrolMode();
                stopRobot();
                closeCamera();
                printSummary(false);
                return false;
            }
        }

        finishPatrolMode();
        stopRobot();
        closeCamera();
        const bool success = real_docked_ && simulation_docked_;
        printSummary(success);
        if (!success) {
            ROS_ERROR(
                "四段巡检及unknown候选回访结束，仍未完成两个目标的停靠");
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
        // 保存第一次视觉估计出的真实墙面坐标。第二阶段视觉复定位
        // 用它关联“同一块板”，不再按画面中心随意挑框。
        bool board_valid;
        WallType board_wall;
        double board_x;
        double board_y;

        TargetObservation()
            : valid(false),
              segment_index(-1),
              category("unknown"),
              board_valid(false),
              board_wall(WALL_LEFT),
              board_x(0.0),
              board_y(0.0) {}
    };

    struct PatrolCheckpoint {
        bool valid;
        int segment_index;
        double stopped_progress;
        Pose2D stopped_pose;

        PatrolCheckpoint()
            : valid(false), segment_index(-1), stopped_progress(0.0) {}
    };

    struct BoardBoundaryEstimate {
        bool valid;
        WallType wall;
        double x;
        double y;

        BoardBoundaryEstimate()
            : valid(false), wall(WALL_LEFT), x(0.0), y(0.0) {}
    };

    // V14.9：OCR最终仍为unknown的墙面候选。
    // 只用于整圈巡检完成后的兜底回访，不参与正常巡检决策。
    struct UnknownCandidate {
        BoardBoundaryEstimate board;
        int source_segment_index;
        Pose2D source_pose;
        Box source_box;
        bool attempted;
        bool resolved;

        UnknownCandidate()
            : source_segment_index(-1),
              source_box{0, 0, 0, 0, 0},
              attempted(false),
              resolved(false) {}
    };

    // V14.5：当前左侧巡检墙目标的接近状态。
    // 一旦在1.5m减速范围内确认，就锁存墙面map坐标。
    // 即使随后NanoDet偶发丢1~2帧，仍可依据map位姿连续减速到0.5m。
    struct PatrolVisualApproach {
        bool valid;
        int segment_index;
        BoardBoundaryEstimate board;
        Box latest_box;
        bool have_latest_box;

        PatrolVisualApproach()
            : valid(false),
              segment_index(-1),
              latest_box{0, 0, 0, 0, 0},
              have_latest_box(false) {}
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

    enum PatrolStartResult {
        PATROL_STARTED,
        PATROL_ALREADY_COMPLETE,
        PATROL_START_FAILED
    };

    static bool isValidCategory(const std::string& category) {
        return category == "food" || category == "daily" ||
               category == "electronic";
    }

    static std::string classifyText(const std::string& text) {
        // OCR可能因为停车过晚只读到类别名称中的一部分，因此三个类别
        // 都按任意一个关键字命中。优先级保持为食品、日用品、电子产品。
        if (text.find("食") != std::string::npos) return "food";
        if (text.find("日") != std::string::npos ||
            text.find("用") != std::string::npos) {
            return "daily";
        }
        if (text.find("电") != std::string::npos ||
            text.find("子") != std::string::npos ||
            text.find("产") != std::string::npos ||
            text.find("生") != std::string::npos) {
            return "electronic";
        }
        return "unknown";
    }

    static bool hasWorkshopFragment(const std::string& text) {
        return text.find("品") != std::string::npos ||
               text.find("加") != std::string::npos ||
               text.find("工") != std::string::npos ||
               text.find("车") != std::string::npos ||
               text.find("间") != std::string::npos;
    }

    static const char* categoryChinese(const std::string& category) {
        if (category == "food") return "食品加工车间";
        if (category == "daily") return "日用品加工车间";
        if (category == "electronic") return "电子产品生产车间";
        return "未知";
    }

    void normalizeParameters() {
        if (!planner_private_namespace_.empty() &&
            planner_private_namespace_[0] != '/') {
            planner_private_namespace_ = "/" + planner_private_namespace_;
        }
        while (planner_private_namespace_.size() > 1 &&
               planner_private_namespace_.back() == '/') {
            planner_private_namespace_.pop_back();
        }
        patrol_path_spacing_ = std::max(0.005, std::fabs(patrol_path_spacing_));
        patrol_speed_limit_ = std::max(0.05, std::fabs(patrol_speed_limit_));
        normal_navigation_speed_limit_ =
            std::max(0.05, std::fabs(normal_navigation_speed_limit_));
        patrol_cancel_timeout_ = std::max(0.5, patrol_cancel_timeout_);
        patrol_interface_timeout_ = std::max(0.5, patrol_interface_timeout_);
        normal_move_base_oscillation_timeout_ =
            std::max(0.0, normal_move_base_oscillation_timeout_);
        patrol_aborted_retry_count_ =
            std::max(0, patrol_aborted_retry_count_);
        patrol_transition_position_kp_ =
            std::fabs(patrol_transition_position_kp_);
        patrol_transition_yaw_kp_ =
            std::fabs(patrol_transition_yaw_kp_);
        patrol_transition_min_linear_speed_ =
            std::fabs(patrol_transition_min_linear_speed_);
        patrol_transition_max_linear_speed_ = std::max(
            std::fabs(patrol_transition_max_linear_speed_),
            patrol_transition_min_linear_speed_);
        patrol_transition_min_angular_speed_ =
            std::fabs(patrol_transition_min_angular_speed_);
        patrol_transition_max_angular_speed_ = std::max(
            std::fabs(patrol_transition_max_angular_speed_),
            patrol_transition_min_angular_speed_);
        patrol_transition_linear_accel_ =
            std::max(0.05, std::fabs(patrol_transition_linear_accel_));
        patrol_transition_angular_accel_ =
            std::max(0.05, std::fabs(patrol_transition_angular_accel_));
        patrol_transition_yaw_priority_start_deg_ =
            std::max(5.0, std::fabs(patrol_transition_yaw_priority_start_deg_));
        patrol_transition_yaw_priority_release_deg_ = clampValue(
            std::fabs(patrol_transition_yaw_priority_release_deg_),
            0.0, patrol_transition_yaw_priority_start_deg_ - 1.0);
        patrol_transition_yaw_priority_min_linear_scale_ = clampValue(
            patrol_transition_yaw_priority_min_linear_scale_, 0.0, 1.0);
        patrol_transition_position_tolerance_ =
            std::max(0.003, std::fabs(patrol_transition_position_tolerance_));
        patrol_transition_yaw_tolerance_deg_ =
            std::max(0.2, std::fabs(patrol_transition_yaw_tolerance_deg_));
        patrol_transition_stable_frames_ =
            std::max(1, patrol_transition_stable_frames_);
        patrol_transition_timeout_ =
            std::max(1.0, patrol_transition_timeout_);
        patrol_stop_max_center_x_ = clampValue(
            patrol_stop_max_center_x_, 1.0,
            std::max(1.0, static_cast<double>(image_width_)));
        patrol_target_stop_distance_ =
            std::max(0.05, std::fabs(patrol_target_stop_distance_));
        patrol_target_slowdown_start_distance_ =
            std::max(patrol_target_stop_distance_ + 0.05,
                     std::fabs(patrol_target_slowdown_start_distance_));
        patrol_target_min_speed_ratio_ =
            clampValue(patrol_target_min_speed_ratio_, 0.05, 1.0);
        patrol_noncurrent_wall_stop_max_distance_ =
            std::max(0.05,
                     std::fabs(patrol_noncurrent_wall_stop_max_distance_));
        patrol_target_left_edge_stop_px_ =
            std::max(1,
                     std::min(patrol_target_left_edge_stop_px_,
                              std::max(1, image_width_)));
        duplicate_coordinate_distance_ =
            std::fabs(duplicate_coordinate_distance_);
        ocr_recovery_turn_deg_ = clampValue(
            std::fabs(ocr_recovery_turn_deg_), 1.0, 180.0);
        ocr_recovery_turn_kp_ = std::fabs(ocr_recovery_turn_kp_);
        ocr_recovery_turn_min_speed_ =
            std::fabs(ocr_recovery_turn_min_speed_);
        ocr_recovery_turn_max_speed_ = std::max(
            std::fabs(ocr_recovery_turn_max_speed_),
            ocr_recovery_turn_min_speed_);
        ocr_recovery_turn_tolerance_deg_ = std::max(
            0.2, std::fabs(ocr_recovery_turn_tolerance_deg_));
        ocr_recovery_turn_stable_frames_ =
            std::max(1, ocr_recovery_turn_stable_frames_);
        ocr_recovery_turn_timeout_ =
            std::max(1.0, ocr_recovery_turn_timeout_);
        ocr_recovery_settle_time_ =
            std::max(0.0, ocr_recovery_settle_time_);
        docking_recovery_turn_deg_ = clampValue(
            std::fabs(docking_recovery_turn_deg_), 1.0, 90.0);
        docking_recovery_detection_attempts_ =
            std::max(1, docking_recovery_detection_attempts_);
        docking_recovery_detection_interval_ =
            std::max(0.0, docking_recovery_detection_interval_);
        docking_refine_max_board_shift_ =
            std::max(0.10, std::fabs(docking_refine_max_board_shift_));
        docking_refresh_clear_calls_ =
            std::max(1, docking_refresh_clear_calls_);
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
        lateral_stable_frames_ = std::max(1, lateral_stable_frames_);
    }

    void buildSegments() {
        segments_.clear();
        // V14.6：首段直接从(1.25,4.25)发布；首段终点及之后三个
        // V13安全角点全部保持原值。后续段仍由runPatrolPoseTransition()
        // 同时完成安全小平移和90度转向，再启动下一条固定Path。
        // V14.6：第一条固定Patrol Path直接从新的巡检准备点发布。
        // 第一段终点和后续V13安全角点全部保持原值。
        addSegment("上墙巡检", 1.25, 4.25, 4.75, 4.30,
                   0.0, 0.5 * kPi, WALL_TOP);
        addSegment("右墙巡检", 4.80, 4.30, 4.80, 2.75,
                   -0.5 * kPi, 0.0, WALL_RIGHT);
        addSegment("下墙巡检", 4.80, 2.70, 0.25, 2.70,
                   -kPi, -0.5 * kPi, WALL_BOTTOM);
        addSegment("左墙巡检", 0.20, 2.70, 0.20, 4.25,
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
            planner_private_namespace_.empty() ||
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
        ROS_INFO("等待move_base、NanoDet、OCR、运动控制和固定巡检路径接口...");
        while (ros::ok() && !move_base_.waitForServer(ros::Duration(3.0))) {
            ROS_INFO("仍在等待move_base");
        }
        if (!ros::ok()) return false;
        cacheMoveBaseOscillationTimeout();
        if (disable_move_base_oscillation_during_patrol_ &&
            !move_base_reconfigure_client_.waitForExistence(
                ros::Duration(5.0))) {
            ROS_WARN("未找到%s；巡检仍可运行，但无法自动关闭move_base振荡监视。",
                     move_base_reconfigure_service_.c_str());
        }
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
        if (!patrol_path_lock_client_.waitForExistence(ros::Duration(20.0))) {
            ROS_ERROR("等待%s/lock_patrol_path超时",
                      planner_private_namespace_.c_str());
            return false;
        }
        if (!controller_reset_client_.waitForExistence(ros::Duration(20.0))) {
            ROS_ERROR("等待%s/reset_controller_state超时",
                      planner_private_namespace_.c_str());
            return false;
        }
        const ros::WallTime subscriber_deadline =
            ros::WallTime::now() + ros::WallDuration(5.0);
        while (ros::ok() && patrol_path_publisher_.getNumSubscribers() == 0 &&
               ros::WallTime::now() < subscriber_deadline) {
            ros::Duration(0.05).sleep();
        }
        if (patrol_path_publisher_.getNumSubscribers() == 0) {
            ROS_ERROR("固定巡检路径话题%s/patrol_path没有订阅者",
                      planner_private_namespace_.c_str());
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

    std::string plannerParameter(const std::string& name) const {
        return planner_private_namespace_ + "/" + name;
    }

    void cacheMoveBaseOscillationTimeout() {
        if (move_base_oscillation_timeout_cached_) return;

        double configured_timeout = normal_move_base_oscillation_timeout_;
        if (ros::param::get("/move_base/oscillation_timeout",
                            configured_timeout)) {
            normal_move_base_oscillation_timeout_ =
                std::max(0.0, configured_timeout);
        }
        move_base_oscillation_timeout_cached_ = true;
        ROS_INFO("已记录普通导航move_base振荡超时：%.2fs。",
                 normal_move_base_oscillation_timeout_);
    }

    bool setMoveBaseOscillationTimeout(double timeout) {
        dynamic_reconfigure::Reconfigure service;
        dynamic_reconfigure::DoubleParameter parameter;
        parameter.name = "oscillation_timeout";
        parameter.value = std::max(0.0, timeout);
        service.request.config.doubles.push_back(parameter);

        if (!move_base_reconfigure_client_.call(service)) {
            ROS_ERROR("调用%s设置oscillation_timeout=%.2f失败。",
                      move_base_reconfigure_service_.c_str(),
                      parameter.value);
            return false;
        }
        return true;
    }

    void setMoveBasePatrolOscillationGuard(bool patrol_mode) {
        if (!disable_move_base_oscillation_during_patrol_) return;
        if (patrol_mode == move_base_patrol_oscillation_guard_active_) return;

        cacheMoveBaseOscillationTimeout();
        const double timeout = patrol_mode
            ? 0.0
            : normal_move_base_oscillation_timeout_;
        if (!setMoveBaseOscillationTimeout(timeout)) {
            ROS_WARN("未能%smove_base振荡监视；巡检Action仍启用有限自动续跑保护。",
                     patrol_mode ? "关闭" : "恢复");
            return;
        }

        move_base_patrol_oscillation_guard_active_ = patrol_mode;
        if (patrol_mode) {
            ROS_WARN("固定巡检期间已将move_base oscillation_timeout临时设为0："
                     "允许原地换向及连续小幅横移，不再误判振荡。" );
        } else {
            ROS_WARN("已恢复普通导航move_base oscillation_timeout=%.2fs。",
                     normal_move_base_oscillation_timeout_);
        }
    }

    void setPlannerFourSpeedLimits(double speed) {
        ros::param::set(plannerParameter("c2_max_reference_speed"), speed);
        ros::param::set(plannerParameter("mpc_max_vx"), speed);
        ros::param::set(
            plannerParameter("mpc_max_translational_speed"), speed);
        ros::param::set(plannerParameter("max_vel_x"), speed);
    }

    void setPatrolRuntimeSpeedLimit(double speed, bool force = false) {
        const double minimum_speed =
            patrol_speed_limit_ * patrol_target_min_speed_ratio_;
        speed = clampValue(speed, minimum_speed, patrol_speed_limit_);

        if (!force &&
            std::isfinite(current_patrol_runtime_speed_limit_) &&
            std::fabs(speed - current_patrol_runtime_speed_limit_) < 0.01) {
            return;
        }

        setPlannerFourSpeedLimits(speed);
        current_patrol_runtime_speed_limit_ = speed;
    }

    void restorePatrolCruiseSpeedIfNeeded() {
        if (!patrol_goal_active_) return;
        setPatrolRuntimeSpeedLimit(patrol_speed_limit_);
    }

    void setPlannerRuntimeParameters(bool patrol_mode) {
        const double speed = patrol_mode
                                 ? patrol_speed_limit_
                                 : normal_navigation_speed_limit_;

        setPlannerFourSpeedLimits(speed);
        current_patrol_runtime_speed_limit_ =
            patrol_mode
                ? speed
                : std::numeric_limits<double>::quiet_NaN();

        ros::param::set(
            plannerParameter("enable_path_replanning"), !patrol_mode);
        setMoveBasePatrolOscillationGuard(patrol_mode);
        ROS_WARN("局部规划器已切换为%s参数：四项速度=%.2f，"
                 "enable_path_replanning=%s。",
                 patrol_mode ? "巡检" : "普通导航",
                 speed, patrol_mode ? "false" : "true");
    }

    void setShadowModeActiveOnce() {
        if (shadow_mode_has_been_disabled_) return;
        ros::param::set(
            plannerParameter("clearance_optimizer/shadow_mode"), false);
        shadow_mode_has_been_disabled_ = true;
        ROS_WARN("已到达第一条路线起点：shadow_mode=false；"
                 "本次任务后续不再修改该参数。");
    }

    bool requestPatrolPathLock(bool lock_path) {
        std_srvs::SetBool service;
        service.request.data = lock_path;
        if (!patrol_path_lock_client_.call(service)) {
            ROS_ERROR("调用%s/lock_patrol_path失败",
                      planner_private_namespace_.c_str());
            return false;
        }
        if (!service.response.success) {
            ROS_WARN("%s固定巡检路径暂未成功：%s",
                     lock_path ? "锁定" : "解除",
                     service.response.message.c_str());
            return false;
        }
        patrol_path_locked_ = lock_path;
        return true;
    }

    bool resetPlannerControllerState() {
        std_srvs::Trigger service;
        if (!controller_reset_client_.call(service) ||
            !service.response.success) {
            ROS_ERROR("复位局部规划器控制状态失败：%s",
                      service.response.message.c_str());
            return false;
        }
        ROS_INFO("局部规划器控制状态已复位：%s",
                 service.response.message.c_str());
        return true;
    }

    bool cancelPatrolGoalAndWait() {
        if (!patrol_goal_active_) return true;

        move_base_.cancelGoal();
        const bool terminal = move_base_.waitForResult(
            ros::Duration(patrol_cancel_timeout_));
        const std::string state = move_base_.getState().toString();
        patrol_goal_active_ = false;
        stopRobot();

        if (!terminal) {
            ROS_ERROR("取消巡检move_base目标超时，当前状态=%s",
                      state.c_str());
            return false;
        }
        ROS_INFO("巡检move_base目标已停止：%s", state.c_str());
        return true;
    }

    void finishPatrolMode() {
        if (patrol_goal_active_) {
            cancelPatrolGoalAndWait();
        }
        if (patrol_path_locked_) {
            requestPatrolPathLock(false);
        }
        setPlannerRuntimeParameters(false);
    }

    bool prepareNormalNavigation() {
        if (patrol_goal_active_ && !cancelPatrolGoalAndWait()) {
            return false;
        }
        if (patrol_path_locked_ && !requestPatrolPathLock(false)) {
            return false;
        }
        setPlannerRuntimeParameters(false);
        return resetPlannerControllerState();
    }

    nav_msgs::Path buildRemainingPatrolPath(
            const Segment& segment,
            const Pose2D& pose,
            double& start_progress,
            double& remaining_distance) {
        start_progress = clampValue(
            segmentProgress(segment, pose), 0.0, segment.length);
        if (patrol_checkpoint_.valid &&
            patrol_checkpoint_.segment_index == current_segment_index_) {
            // 停靠导航可能使车辆沿巡检线方向产生少量回退。剩余路线的
            // 起点不得退到停车断点之前，避免重复巡检已经扫过的区域。
            start_progress = std::max(
                start_progress,
                clampValue(patrol_checkpoint_.stopped_progress,
                           0.0, segment.length));
        }
        remaining_distance = segment.length - start_progress;

        nav_msgs::Path path;
        path.header.frame_id = map_frame_;
        path.header.stamp = ros::Time::now();
        path.header.seq = ++patrol_path_sequence_;

        if (remaining_distance <= segment_end_tolerance_) {
            return path;
        }

        const int intervals = std::max(
            1, static_cast<int>(
                   std::ceil(remaining_distance / patrol_path_spacing_)));
        path.poses.reserve(static_cast<std::size_t>(intervals + 1));
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, segment.travel_yaw);

        for (int i = 0; i <= intervals; ++i) {
            const double ratio =
                static_cast<double>(i) / static_cast<double>(intervals);
            const double progress =
                start_progress + ratio * remaining_distance;
            geometry_msgs::PoseStamped point;
            point.header = path.header;
            point.pose.position.x =
                segment.start_x + progress * segment.dir_x;
            point.pose.position.y =
                segment.start_y + progress * segment.dir_y;
            point.pose.orientation = tf2::toMsg(quaternion);
            path.poses.push_back(point);
        }
        return path;
    }

    PatrolStartResult startPatrolFromPose(
            const Segment& segment,
            const Pose2D& pose) {
        double start_progress = 0.0;
        double remaining_distance = 0.0;
        nav_msgs::Path path = buildRemainingPatrolPath(
            segment, pose, start_progress, remaining_distance);
        if (remaining_distance <= segment_end_tolerance_) {
            ROS_INFO("%s剩余距离仅%.3fm，直接判定本段完成。",
                     segment.name.c_str(), remaining_distance);
            return PATROL_ALREADY_COMPLETE;
        }

        // 每次开始或断点恢复都先解除旧锁。规划器解除锁时会清空旧的
        // 暂存路线，因此随后SetBool(true)成功必然对应本次新发布的Path。
        if (patrol_path_locked_ && !requestPatrolPathLock(false)) {
            return PATROL_START_FAILED;
        }
        if (!resetPlannerControllerState()) {
            return PATROL_START_FAILED;
        }
        setPlannerRuntimeParameters(true);

        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(patrol_interface_timeout_);
        bool locked = false;
        do {
            path.header.stamp = ros::Time::now();
            for (std::size_t i = 0; i < path.poses.size(); ++i) {
                path.poses[i].header = path.header;
            }
            patrol_path_publisher_.publish(path);
            ros::Duration(0.08).sleep();
            locked = requestPatrolPathLock(true);
            if (!locked) ros::Duration(0.08).sleep();
        } while (ros::ok() && !locked &&
                 ros::WallTime::now() < deadline);

        if (!locked) {
            setPlannerRuntimeParameters(false);
            ROS_ERROR("%s固定路线未能在%.1f秒内被局部规划器确认",
                      segment.name.c_str(), patrol_interface_timeout_);
            return PATROL_START_FAILED;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = path.poses.back();
        goal.target_pose.header.stamp = ros::Time::now();
        move_base_.sendGoal(goal);
        patrol_goal_active_ = true;
        patrol_checkpoint_.valid = false;
        ROS_INFO("%s已异步启动：断点进度=%.3f/%.3fm，剩余=%.3fm，"
                 "固定路径点数=%zu，终点=(%.3f, %.3f, %.1f度)。",
                 segment.name.c_str(), start_progress, segment.length,
                 remaining_distance, path.poses.size(),
                 goal.target_pose.pose.position.x,
                 goal.target_pose.pose.position.y,
                 segment.travel_yaw * 180.0 / kPi);
        return PATROL_STARTED;
    }

    void releaseCameraBeforePredockNavigation(
            const std::string& target_name) {
        if (!camera_opened_) {
            ROS_INFO(
                "[停靠相机] %s准备第一段move_base：摄像头当前已关闭，"
                "无需额外处理。",
                target_name.c_str());
            return;
        }

        // 这是V14.2处理缓存的核心：
        // 与其在第一段到点后等待、grab、丢帧，不如在离开巡检时直接
        // release VideoCapture。第一段行驶期间没有相机会话，自然不会
        // 积累任何旧帧；到点后重新open就是全新的V4L2缓冲。
        ROS_WARN(
            "[停靠相机] %s准备第一段move_base：立即关闭NanoDet摄像头，"
            "彻底释放当前V4L2缓存；第一段到点后再即时重开。",
            target_name.c_str());
        closeCamera();
    }

    bool navigateToPose(double x, double y, double yaw,
                        const std::string& purpose) {
        clampToRoom(x, y);
        if (!isInsideRoom(x, y)) {
            ROS_ERROR("%s目标(%.3f, %.3f, %.1f度)超出房间坐标边界",
                      purpose.c_str(), x, y, yaw * 180.0 / kPi);
            return false;
        }

        if (!prepareNormalNavigation()) return false;
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

    double segmentProgress(const Segment& segment, const Pose2D& pose) const {
        return (pose.x - segment.start_x) * segment.dir_x +
               (pose.y - segment.start_y) * segment.dir_y;
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

    bool makeDockingObservationAtStandoff(
            int segment_index,
            const std::string& category,
            const BoardBoundaryEstimate& board_estimate,
            double standoff,
            TargetObservation& observation) const {
        observation = TargetObservation();
        if (!board_estimate.valid || standoff <= 0.0) return false;

        observation.valid = true;
        observation.board_valid = true;
        observation.board_wall = board_estimate.wall;
        observation.board_x = board_estimate.x;
        observation.board_y = board_estimate.y;
        observation.pose.x = board_estimate.x;
        observation.pose.y = board_estimate.y;
        switch (board_estimate.wall) {
            case WALL_TOP:
                observation.pose.y = room_max_y_ - standoff;
                observation.pose.yaw = 0.5 * kPi;
                break;
            case WALL_RIGHT:
                observation.pose.x = room_max_x_ - standoff;
                observation.pose.yaw = 0.0;
                break;
            case WALL_BOTTOM:
                observation.pose.y = room_min_y_ + standoff;
                observation.pose.yaw = -0.5 * kPi;
                break;
            case WALL_LEFT:
                observation.pose.x = room_min_x_ + standoff;
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

    bool makeDockingObservation(
            int segment_index,
            const std::string& category,
            const BoardBoundaryEstimate& board_estimate,
            TargetObservation& observation) const {
        return makeDockingObservationAtStandoff(
            segment_index, category, board_estimate,
            docking_standoff_, observation);
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

    int findUnknownCandidate(
            const BoardBoundaryEstimate& estimate) const {
        if (!estimate.valid) return -1;

        for (std::size_t i = 0;
             i < unknown_candidates_.size();
             ++i) {
            const UnknownCandidate& candidate =
                unknown_candidates_[i];

            if (!candidate.board.valid ||
                candidate.board.wall != estimate.wall) {
                continue;
            }

            if (distance2D(
                    candidate.board.x,
                    candidate.board.y,
                    estimate.x,
                    estimate.y) <=
                duplicate_coordinate_distance_) {
                return static_cast<int>(i);
            }
        }

        return -1;
    }

    void addUnknownCandidate(
            std::size_t segment_index,
            const Pose2D& pose,
            const Box& reference_box,
            const BoardBoundaryEstimate& fallback_estimate,
            const std::string& reason) {
        BoardBoundaryEstimate estimate;

        // 优先使用OCR最终参考框与“当前最终静止位姿”重新投影。
        // 若该框无法求墙面交点，再退回最初触发停车时的墙面估计。
        if (!estimateBoardBoundary(
                pose,
                reference_box,
                estimate)) {
            estimate = fallback_estimate;
        }

        if (!estimate.valid) {
            ROS_WARN(
                "V14.9候选记录失败：%s；"
                "当前OCR仍unknown，但没有有效墙面坐标，无法生成候选导航点。",
                reason.c_str());
            return;
        }

        const int existing =
            findUnknownCandidate(estimate);
        if (existing >= 0) {
            ROS_INFO(
                "V14.9候选去重：%s(%.3f,%.3f)与candidate[%d]"
                "距离<=%.2fm，不重复加入。",
                wallName(estimate.wall),
                estimate.x,
                estimate.y,
                existing + 1,
                duplicate_coordinate_distance_);
            return;
        }

        UnknownCandidate candidate;
        candidate.board = estimate;
        candidate.source_segment_index =
            static_cast<int>(segment_index);
        candidate.source_pose = pose;
        candidate.source_box = reference_box;

        unknown_candidates_.push_back(candidate);

        ROS_WARN(
            "V14.9新增unknown候选[%zu]：%s(%.3f,%.3f)，"
            "来源巡检段=%zu，原因=%s。"
            "当前不前往候选点；只有整圈巡检结束且任务未完成时才回访。",
            unknown_candidates_.size(),
            wallName(estimate.wall),
            estimate.x,
            estimate.y,
            segment_index + 1,
            reason.c_str());
    }

    void resolveUnknownCandidateNear(
            const BoardBoundaryEstimate& estimate,
            const std::string& reason) {
        const int index =
            findUnknownCandidate(estimate);
        if (index < 0) return;

        UnknownCandidate& candidate =
            unknown_candidates_[
                static_cast<std::size_t>(index)];
        if (candidate.resolved) return;

        candidate.resolved = true;
        ROS_INFO(
            "V14.9候选[%d]已解决：%s(%.3f,%.3f)，%s。",
            index + 1,
            wallName(candidate.board.wall),
            candidate.board.x,
            candidate.board.y,
            reason.c_str());
    }

    bool recognizeUnknownCandidate(
            const TargetObservation& candidate_observation,
            Box& matched_box,
            BoardBoundaryEstimate& matched_board,
            OcrRecord& ocr) {
        TargetObservation unused;

        if (!detectDockingRefinedObservation(
                candidate_observation,
                "unknown候选回访视觉匹配",
                docking_standoff_,
                unused,
                false,
                &matched_box,
                &matched_board,
                true)) {
            return false;
        }

        ocr = recognizeStaticTarget(matched_box);

        // 保留主巡检原有“只读到车间通用残片时逆时针30度再识别一次”。
        // 候选点此时已经在0.5m正面附近，这一步只是最终OCR兜底。
        if (ocr.category == "unknown" &&
            hasWorkshopFragment(ocr.text)) {
            ROS_WARN(
                "V14.9候选回访只识别到通用残片“%s”；"
                "沿用原OCR补偿机制，逆时针转%.1f度后再识别一次。",
                ocr.text.c_str(),
                ocr_recovery_turn_deg_);

            Pose2D recovery_pose;
            if (rotateCounterClockwiseForOcr(
                    recovery_pose)) {
                Box retry_box{0, 0, 0, 0, 0};
                BoardBoundaryEstimate retry_board;
                TargetObservation retry_unused;

                if (detectDockingRefinedObservation(
                        candidate_observation,
                        "unknown候选OCR补偿后匹配",
                        docking_standoff_,
                        retry_unused,
                        true,
                        &retry_box,
                        &retry_board,
                        true)) {
                    matched_box = retry_box;
                    matched_board = retry_board;
                    ocr = recognizeStaticTarget(
                        retry_box);
                }
            }
        }

        return true;
    }

    bool revisitUnknownCandidates() {
        // 已经完成整圈巡检，任何旧的“非当前墙必须等当前段结束”保护
        // 在这里都不再有意义。
        simulation_target_blocked_until_segment_end_ = false;

        for (std::size_t i = 0;
             i < unknown_candidates_.size() &&
             ros::ok();
             ++i) {
            if (real_docked_ &&
                simulation_docked_) {
                ROS_INFO(
                    "V14.9双目标已完成，停止剩余候选回访。");
                return true;
            }

            UnknownCandidate& candidate =
                unknown_candidates_[i];

            if (candidate.resolved) {
                continue;
            }

            candidate.attempted = true;

            const int segment_index =
                segmentIndexForWall(
                    candidate.board.wall);
            TargetObservation predock;

            if (segment_index < 0 ||
                !makeDockingObservation(
                    segment_index,
                    "unknown",
                    candidate.board,
                    predock)) {
                ROS_WARN(
                    "V14.9候选[%zu]无法生成距墙%.2fm临时停靠点，"
                    "跳过该候选。",
                    i + 1,
                    docking_standoff_);
                continue;
            }

            closeCamera();

            ROS_WARN(
                "V14.9回访候选[%zu/%zu]：板=%s(%.3f,%.3f)，"
                "导航到临时停靠点=(%.3f,%.3f,%.1f度)。",
                i + 1,
                unknown_candidates_.size(),
                wallName(candidate.board.wall),
                candidate.board.x,
                candidate.board.y,
                predock.pose.x,
                predock.pose.y,
                predock.pose.yaw * 180.0 / kPi);

            if (!navigateToPose(
                    predock.pose.x,
                    predock.pose.y,
                    predock.pose.yaw,
                    "前往unknown候选临时停靠点")) {
                ROS_WARN(
                    "V14.9候选[%zu]导航失败，继续下一个候选。",
                    i + 1);
                continue;
            }

            if (!openCamera()) {
                return false;
            }

            Box matched_box{0, 0, 0, 0, 0};
            BoardBoundaryEstimate matched_board;
            OcrRecord ocr;

            if (!recognizeUnknownCandidate(
                    predock,
                    matched_box,
                    matched_board,
                    ocr)) {
                ROS_WARN(
                    "V14.9候选[%zu]到点后未重新找到同一目标板，"
                    "继续下一个候选。",
                    i + 1);
                closeCamera();
                continue;
            }

            if (ocr.category == "unknown") {
                ROS_WARN(
                    "V14.9候选[%zu]已到正面临时停靠点，"
                    "但OCR仍为unknown：%s；继续下一个候选。",
                    i + 1,
                    ocr.text.c_str());
                closeCamera();
                continue;
            }

            Pose2D current_pose;
            BoardBoundaryEstimate classified_board =
                matched_board;

            if (getRobotPose(current_pose)) {
                BoardBoundaryEstimate ocr_board;
                if (estimateBoardBoundary(
                        current_pose,
                        ocr.box,
                        ocr_board)) {
                    classified_board =
                        ocr_board;
                }
            }

            candidate.resolved = true;

            // 成功分类后的板加入原seen名单（若尚未存在）。
            if (classified_board.valid &&
                !isDuplicateBoard(
                    classified_board)) {
                seen_board_coordinates_.push_back(
                    classified_board);
            }

            ROS_WARN(
                "V14.9候选[%zu]重新分类成功：文字=%s，类别=%s，"
                "墙面坐标=%s(%.3f,%.3f)。",
                i + 1,
                ocr.text.c_str(),
                categoryChinese(ocr.category),
                wallName(classified_board.wall),
                classified_board.x,
                classified_board.y);

            // 第三类非任务目标：候选已经被证明不是本轮两个任务目标，
            // 标记resolved后继续回访即可。
            if (ocr.category != real_target_category_ &&
                ocr.category != simulation_target_category_) {
                ROS_INFO(
                    "V14.9候选[%zu]分类为非任务类别%s，跳过停靠。",
                    i + 1,
                    categoryChinese(ocr.category));
                closeCamera();
                continue;
            }

            TargetObservation observation;
            const int classified_segment =
                segmentIndexForWall(
                    classified_board.wall);

            if (classified_segment < 0 ||
                !makeDockingObservation(
                    classified_segment,
                    ocr.category,
                    classified_board,
                    observation)) {
                ROS_ERROR(
                    "V14.9候选[%zu]虽然分类成功，但无法生成有效停靠点。",
                    i + 1);
                closeCamera();
                return false;
            }

            if (ocr.category ==
                simulation_target_category_) {
                if (!simulation_docked_) {
                    simulation_observation_ =
                        observation;
                    simulation_target_pending_ = true;
                    simulation_target_blocked_until_segment_end_ =
                        false;
                }

                if (!real_docked_) {
                    ROS_WARN(
                        "V14.9候选[%zu]确认是仿真目标，但现实目标尚未停靠；"
                        "先保存仿真目标，继续回访后续候选寻找现实目标。",
                        i + 1);
                    closeCamera();
                    continue;
                }

                closeCamera();
                if (!dockPendingSimulationTarget()) {
                    return false;
                }
                continue;
            }

            // 现实目标候选：整圈巡检已经结束，不再区分当前/非当前墙，
            // 直接按正常两段式停靠处理。
            if (ocr.category ==
                real_target_category_) {
                if (!real_docked_) {
                    real_observation_ =
                        observation;
                    real_target_pending_ = false;
                    real_target_defer_segment_index_ = -1;

                    releaseCameraBeforePredockNavigation(
                        "候选回访现实目标");

                    if (!navigateToPose(
                            observation.pose.x,
                            observation.pose.y,
                            observation.pose.yaw,
                            "前往候选回访现实目标临时停靠点")) {
                        return false;
                    }

                    if (!dockTarget(
                            real_observation_,
                            "候选回访现实目标",
                            false)) {
                        return false;
                    }

                    real_docked_ = true;
                } else {
                    closeCamera();
                }

                // 若仿真目标此前已经正常巡检识别/候选回访识别并保存，
                // 现实目标一完成就立即补做仿真停靠。
                simulation_target_blocked_until_segment_end_ =
                    false;
                if (hasPendingSimulationTarget()) {
                    ROS_INFO(
                        "V14.9现实候选停靠完成，"
                        "立即处理此前已经保存的仿真目标。");
                    if (!dockPendingSimulationTarget()) {
                        return false;
                    }
                }
            }
        }

        // 防御性：候选循环末尾现实已完成，且仿真已经被保存但还没停。
        simulation_target_blocked_until_segment_end_ = false;
        if (hasPendingSimulationTarget()) {
            if (!dockPendingSimulationTarget()) {
                return false;
            }
        }

        return true;
    }

    double boardProgressOnSegment(
            const Segment& segment,
            const BoardBoundaryEstimate& board) const {
        return (board.x - segment.start_x) * segment.dir_x +
               (board.y - segment.start_y) * segment.dir_y;
    }

    double boardAheadProgress(
            const Segment& segment,
            const Pose2D& pose,
            const BoardBoundaryEstimate& board) const {
        return boardProgressOnSegment(segment, board) -
               segmentProgress(segment, pose);
    }

    double patrolTargetSpeedRatio(double distance_to_board) const {
        if (distance_to_board >=
            patrol_target_slowdown_start_distance_) {
            return 1.0;
        }
        if (distance_to_board <=
            patrol_target_stop_distance_) {
            return patrol_target_min_speed_ratio_;
        }

        const double span =
            patrol_target_slowdown_start_distance_ -
            patrol_target_stop_distance_;
        const double progress = clampValue(
            (distance_to_board - patrol_target_stop_distance_) / span,
            0.0, 1.0);

        // 与line2o障碍物减速同形：
        // stop处=min_ratio，slowdown_start处=1.0。
        return patrol_target_min_speed_ratio_ +
               (1.0 - patrol_target_min_speed_ratio_) * progress;
    }

    void clearPatrolVisualApproach(bool restore_speed) {
        patrol_visual_approach_ = PatrolVisualApproach();
        if (restore_speed) {
            restorePatrolCruiseSpeedIfNeeded();
        }
    }

    double distanceFromPoseToWall(
            const Pose2D& pose,
            WallType wall) const {
        switch (wall) {
            case WALL_LEFT:
                return std::fabs(pose.x - room_min_x_);
            case WALL_RIGHT:
                return std::fabs(room_max_x_ - pose.x);
            case WALL_BOTTOM:
                return std::fabs(pose.y - room_min_y_);
            case WALL_TOP:
                return std::fabs(room_max_y_ - pose.y);
        }
        return std::numeric_limits<double>::infinity();
    }

    // V14.7保护2：
    // 当前左侧正在巡检的墙上，如果框的左边界已经逼近图像左缘，
    // 说明继续依赖map距离有把目标直接开出视野的风险。
    // 该条件优先于1.5m->0.7m距离减速/停车条件。
    int chooseCurrentWallLeftEdgeEmergencyBox(
            const std::vector<Box>& boxes,
            const Pose2D& pose,
            const Segment& segment,
            BoardBoundaryEstimate& selected_estimate) const {
        int selected = -1;
        int best_x0 = std::numeric_limits<int>::max();

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() >=
                patrol_stop_max_center_x_) {
                continue;
            }

            // 严格“不足100px”：默认参数100时x0=99触发，x0=100不触发。
            if (boxes[i].x0 >=
                patrol_target_left_edge_stop_px_) {
                continue;
            }

            BoardBoundaryEstimate estimate;
            if (!estimateBoardBoundary(
                    pose, boxes[i], estimate)) {
                continue;
            }

            if (estimate.wall != segment.wall ||
                isDuplicateBoard(estimate)) {
                continue;
            }

            // 仍要求目标没有明显落到车辆后方。
            const double ahead =
                boardAheadProgress(
                    segment, pose, estimate);
            if (ahead < -0.03) {
                continue;
            }

            if (boxes[i].x0 < best_x0) {
                best_x0 = boxes[i].x0;
                selected = static_cast<int>(i);
                selected_estimate = estimate;
            }
        }

        return selected;
    }

    // V14.7保护1：
    // 非当前巡检墙不再“远远看见就停车”。
    // 已经能判断墙面时，必须机器人到该墙本身的垂直距离<1.5m才停车。
    // 射线无法判断墙面的框无法执行这层几何保护，保留旧安全逻辑立即停车。
    int chooseImmediateNonCurrentBoardBox(
            const std::vector<Box>& boxes,
            const Pose2D& pose,
            const Segment& segment) const {
        const double image_center =
            0.5 * static_cast<double>(image_width_);
        int selected = -1;
        double best_error =
            std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() >= patrol_stop_max_center_x_) {
                continue;
            }

            BoardBoundaryEstimate estimate;
            const bool estimate_ok =
                estimateBoardBoundary(
                    pose, boxes[i], estimate);

            if (estimate_ok && isDuplicateBoard(estimate)) {
                continue;
            }

            bool immediate = false;

            if (!estimate_ok) {
                // 无法判断属于哪面墙，保持V14.6之前的安全行为。
                immediate = true;
            } else if (estimate.wall != segment.wall) {
                const double wall_distance =
                    distanceFromPoseToWall(
                        pose, estimate.wall);

                if (wall_distance >=
                    patrol_noncurrent_wall_stop_max_distance_) {
                    ROS_INFO_THROTTLE(
                        0.8,
                        "V14.7非当前墙远距离保护：检测到%s目标框"
                        "center=(%.1f,%.1f)，但机器人距该墙=%.3fm>=%.3fm；"
                        "本帧只观察，不中断%s。",
                        wallName(estimate.wall),
                        boxes[i].centerX(),
                        boxes[i].centerY(),
                        wall_distance,
                        patrol_noncurrent_wall_stop_max_distance_,
                        segment.name.c_str());
                    continue;
                }

                immediate = true;
                ROS_INFO_THROTTLE(
                    0.8,
                    "V14.7非当前墙允许停车：%s，机器人距该墙=%.3fm<%.3fm。",
                    wallName(estimate.wall),
                    wall_distance,
                    patrol_noncurrent_wall_stop_max_distance_);
            }

            if (!immediate) {
                continue;
            }

            const double error =
                std::fabs(
                    boxes[i].centerX() - image_center);
            if (error < best_error) {
                best_error = error;
                selected = static_cast<int>(i);
            }
        }

        return selected;
    }

    // 从当前帧中寻找“车体左侧正在巡检的这一面墙”上的最近前方新目标。
    // 只在进入1.5m减速范围后锁存，避免远距离框直接打断巡检。
    void updateCurrentWallApproachFromBoxes(
            std::size_t segment_index,
            const Segment& segment,
            const Pose2D& pose,
            const std::vector<Box>& boxes) {
        int selected = -1;
        double best_distance =
            std::numeric_limits<double>::infinity();
        BoardBoundaryEstimate best_board;

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() >=
                patrol_stop_max_center_x_) {
                continue;
            }

            BoardBoundaryEstimate estimate;
            if (!estimateBoardBoundary(
                    pose, boxes[i], estimate)) {
                continue;
            }
            if (estimate.wall != segment.wall ||
                isDuplicateBoard(estimate)) {
                continue;
            }

            // 必须位于当前巡检方向前方；已经驶过的板不参与接近减速。
            const double ahead =
                boardAheadProgress(
                    segment, pose, estimate);
            if (ahead < -0.03) {
                continue;
            }

            const double physical_distance =
                distance2D(
                    pose.x, pose.y,
                    estimate.x, estimate.y);

            if (physical_distance >
                patrol_target_slowdown_start_distance_) {
                continue;
            }

            if (physical_distance < best_distance) {
                best_distance = physical_distance;
                best_board = estimate;
                selected = static_cast<int>(i);
            }
        }

        if (selected < 0) {
            return;
        }

        patrol_visual_approach_.valid = true;
        patrol_visual_approach_.segment_index =
            static_cast<int>(segment_index);
        patrol_visual_approach_.board = best_board;
        patrol_visual_approach_.latest_box =
            boxes[static_cast<std::size_t>(selected)];
        patrol_visual_approach_.have_latest_box = true;

        ROS_INFO_THROTTLE(
            0.6,
            "V14.5锁定当前左墙目标：%s(%.3f,%.3f)，"
            "当前直线距离=%.3fm，开始/继续视觉接近减速。",
            wallName(best_board.wall),
            best_board.x,
            best_board.y,
            best_distance);
    }

    bool updatePatrolVisualApproachSpeed(
            const Segment& segment,
            const Pose2D& pose,
            double& distance_to_board) {
        distance_to_board =
            std::numeric_limits<double>::infinity();

        if (!patrol_visual_approach_.valid ||
            patrol_visual_approach_.segment_index !=
                current_segment_index_) {
            return false;
        }

        const double ahead =
            boardAheadProgress(
                segment,
                pose,
                patrol_visual_approach_.board);

        // 理论上在0.5m处已停车；若因定位瞬跳目标已明显到车后，
        // 清掉状态并恢复巡检速度，防止反向追一个已经驶过的板。
        if (ahead < -0.08) {
            ROS_WARN(
                "V14.5当前左墙目标已位于巡检方向后方%.3fm，"
                "取消本次接近状态并恢复巡检速度。",
                -ahead);
            clearPatrolVisualApproach(true);
            return false;
        }

        distance_to_board =
            distance2D(
                pose.x, pose.y,
                patrol_visual_approach_.board.x,
                patrol_visual_approach_.board.y);

        const double ratio =
            patrolTargetSpeedRatio(
                distance_to_board);
        const double speed_limit =
            patrol_speed_limit_ * ratio;

        setPatrolRuntimeSpeedLimit(speed_limit);

        ROS_INFO_THROTTLE(
            0.35,
            "V14.5巡检视觉减速：当前左墙目标距离=%.3fm，"
            "前向投影剩余=%.3fm，速度比例=%.3f，"
            "MyPlanner四项速度上限=%.3fm/s（巡检基准=%.3f）。",
            distance_to_board,
            ahead,
            ratio,
            speed_limit,
            patrol_speed_limit_);

        return true;
    }

    // 到0.5m停车后再取一帧近距离NanoDet，避免把1.5m处的小框
    // 作为OCR参考框。优先选与锁存墙面坐标最接近的同墙框。
    bool reacquireApproachBoxAfterStop(
            const Segment& segment,
            Pose2D& stopped_pose,
            Box& trigger_box) {
        const BoardBoundaryEstimate locked_board =
            patrol_visual_approach_.board;

        for (int attempt = 0;
             attempt < docking_recovery_detection_attempts_ &&
             ros::ok();
             ++attempt) {
            ros::spinOnce();

            std::vector<Box> boxes;
            if (!detectBoxes(boxes) ||
                boxes.empty()) {
                continue;
            }

            getRobotPose(stopped_pose);

            int selected = -1;
            double best_shift =
                std::numeric_limits<double>::infinity();

            for (std::size_t i = 0;
                 i < boxes.size(); ++i) {
                BoardBoundaryEstimate estimate;
                if (!estimateBoardBoundary(
                        stopped_pose,
                        boxes[i],
                        estimate)) {
                    continue;
                }
                if (estimate.wall != segment.wall ||
                    estimate.wall != locked_board.wall) {
                    continue;
                }

                const double shift =
                    distance2D(
                        estimate.x, estimate.y,
                        locked_board.x, locked_board.y);
                if (shift <=
                        docking_refine_max_board_shift_ &&
                    shift < best_shift) {
                    best_shift = shift;
                    selected = static_cast<int>(i);
                }
            }

            if (selected >= 0) {
                trigger_box =
                    boxes[
                        static_cast<std::size_t>(
                            selected)];

                ROS_WARN(
                    "V14.5近距离停车后重新取框成功："
                    "attempt=%d/%d，center=(%.1f,%.1f)，"
                    "相对减速锁存板坐标偏移=%.3fm。",
                    attempt + 1,
                    docking_recovery_detection_attempts_,
                    trigger_box.centerX(),
                    trigger_box.centerY(),
                    best_shift);
                return true;
            }
        }

        if (patrol_visual_approach_.have_latest_box) {
            trigger_box =
                patrol_visual_approach_.latest_box;
            ROS_WARN(
                "V14.5在0.5m停车后未重新取得匹配框；"
                "回退使用减速过程中最近一次有效NanoDet框作为OCR参考。");
            return true;
        }

        ROS_ERROR(
            "V14.5当前左墙目标到达停车距离，但没有任何可用触发框。");
        return false;
    }

    double limitPatrolTransitionRate(double desired,
                                     double previous,
                                     double max_delta) const {
        return previous + clampValue(
            desired - previous, -max_delta, max_delta);
    }

    bool runPatrolPoseTransition(double target_x,
                                 double target_y,
                                 double target_yaw,
                                 const std::string& label) {
        if (!isInsideRoom(target_x, target_y)) {
            ROS_ERROR("%s目标(%.3f, %.3f)超出房间边界",
                      label.c_str(), target_x, target_y);
            return false;
        }

        // 角点过渡只由/set_speed直接接管。调用前固定巡检Path已经解除，
        // 不让move_base最终姿态调整在墙边原地旋转。
        stopRobot();

        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(patrol_transition_timeout_);
        ros::WallTime last_time = ros::WallTime::now();
        ros::Rate rate(std::max(10.0, control_rate_));

        double command_vx = 0.0;
        double command_vy = 0.0;
        double command_wz = 0.0;
        int stable_frames = 0;

        ROS_INFO("%s开始：目标=(%.3f, %.3f, %.1f度)，"
                 "采用全向XY+yaw同时P闭环，并启用V13旋转优先。",
                 label.c_str(), target_x, target_y,
                 target_yaw * 180.0 / kPi);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            const double world_error_x = target_x - pose.x;
            const double world_error_y = target_y - pose.y;
            const double position_error =
                std::hypot(world_error_x, world_error_y);
            const double yaw_error =
                normalizeAngle(target_yaw - pose.yaw);

            // map误差转到当前车体坐标，因此在旋转过程中仍能正确向绝对目标横移。
            const double c = std::cos(pose.yaw);
            const double s = std::sin(pose.yaw);
            const double base_error_x =
                c * world_error_x + s * world_error_y;
            const double base_error_y =
                -s * world_error_x + c * world_error_y;

            double desired_vx = 0.0;
            double desired_vy = 0.0;
            double desired_wz = 0.0;

            if (position_error > patrol_transition_position_tolerance_) {
                desired_vx = clampValue(
                    patrol_transition_position_kp_ * base_error_x,
                    -patrol_transition_max_linear_speed_,
                    patrol_transition_max_linear_speed_);
                desired_vy = clampValue(
                    patrol_transition_position_kp_ * base_error_y,
                    -patrol_transition_max_linear_speed_,
                    patrol_transition_max_linear_speed_);

                const double linear_norm =
                    std::hypot(desired_vx, desired_vy);
                if (linear_norm > 1e-9 &&
                    linear_norm < patrol_transition_min_linear_speed_) {
                    const double scale =
                        patrol_transition_min_linear_speed_ / linear_norm;
                    desired_vx *= scale;
                    desired_vy *= scale;
                }

                const double limited_norm =
                    std::hypot(desired_vx, desired_vy);
                if (limited_norm > patrol_transition_max_linear_speed_) {
                    const double scale =
                        patrol_transition_max_linear_speed_ / limited_norm;
                    desired_vx *= scale;
                    desired_vy *= scale;
                }
            }

            // V13：角点过渡采用“旋转优先的同步平移”。
            // 当车头仍大幅朝向墙面时，只保留少量平移；随着yaw误差减小，
            // 线速度连续恢复到100%。这样仍然边转边移，但不会先走完5cm。
            double rotation_priority_scale = 1.0;
            const double abs_yaw_error_deg =
                std::fabs(yaw_error) * 180.0 / kPi;
            if (abs_yaw_error_deg >= patrol_transition_yaw_priority_start_deg_) {
                rotation_priority_scale =
                    patrol_transition_yaw_priority_min_linear_scale_;
            } else if (abs_yaw_error_deg >
                       patrol_transition_yaw_priority_release_deg_) {
                const double span = std::max(
                    1.0,
                    patrol_transition_yaw_priority_start_deg_
                    - patrol_transition_yaw_priority_release_deg_);
                const double progress = clampValue(
                    (patrol_transition_yaw_priority_start_deg_
                     - abs_yaw_error_deg) / span,
                    0.0, 1.0);
                rotation_priority_scale =
                    patrol_transition_yaw_priority_min_linear_scale_
                    + (1.0 - patrol_transition_yaw_priority_min_linear_scale_)
                      * progress;
            }
            desired_vx *= rotation_priority_scale;
            desired_vy *= rotation_priority_scale;

            const double yaw_tolerance =
                patrol_transition_yaw_tolerance_deg_ * kPi / 180.0;
            if (std::fabs(yaw_error) > yaw_tolerance) {
                desired_wz = clampValue(
                    patrol_transition_yaw_kp_ * yaw_error,
                    -patrol_transition_max_angular_speed_,
                    patrol_transition_max_angular_speed_);
                if (std::fabs(desired_wz) <
                    patrol_transition_min_angular_speed_) {
                    desired_wz = std::copysign(
                        patrol_transition_min_angular_speed_, yaw_error);
                }
            }

            const ros::WallTime now = ros::WallTime::now();
            double dt = (now - last_time).toSec();
            last_time = now;
            if (!std::isfinite(dt) || dt <= 0.0) dt = 0.05;
            dt = clampValue(dt, 0.01, 0.20);

            command_vx = limitPatrolTransitionRate(
                desired_vx, command_vx,
                patrol_transition_linear_accel_ * dt);
            command_vy = limitPatrolTransitionRate(
                desired_vy, command_vy,
                patrol_transition_linear_accel_ * dt);
            command_wz = limitPatrolTransitionRate(
                desired_wz, command_wz,
                patrol_transition_angular_accel_ * dt);

            const bool position_ok =
                position_error <= patrol_transition_position_tolerance_;
            const bool yaw_ok = std::fabs(yaw_error) <= yaw_tolerance;

            if (position_ok && yaw_ok) {
                ++stable_frames;
                command_vx = 0.0;
                command_vy = 0.0;
                command_wz = 0.0;
                if (stable_frames >= patrol_transition_stable_frames_) {
                    stopRobot();
                    ROS_INFO("%s完成：当前位置=(%.3f, %.3f, %.1f度)。",
                             label.c_str(), pose.x, pose.y,
                             pose.yaw * 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
            }

            publishVelocity(command_vx, command_vy, command_wz);
            ROS_INFO_THROTTLE(
                0.5,
                "%s中：pose=(%.3f,%.3f,%.1f度)，"
                "pos_err=%.3fm，yaw_err=%.1f度，cmd=(%.3f,%.3f,%.3f)",
                label.c_str(), pose.x, pose.y,
                pose.yaw * 180.0 / kPi,
                position_error, yaw_error * 180.0 / kPi,
                command_vx, command_vy, command_wz);
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("%s超时：未能在%.1fs内完成安全位姿过渡。",
                  label.c_str(), patrol_transition_timeout_);
        return false;
    }

    bool runCornerTransitionAfterSegment(std::size_t segment_index) {
        switch (segment_index) {
            case 0:
                // 上墙终点仍是x=4.75；同时右移到x=4.80并转到-90度。
                return runPatrolPoseTransition(
                    4.80, 4.30, -0.5 * kPi,
                    "上墙→右墙安全角点过渡");
            case 1:
                // 右墙终点仍是y=2.75；同时下移到y=2.70并转到-180度。
                return runPatrolPoseTransition(
                    4.80, 2.70, -kPi,
                    "右墙→下墙安全角点过渡");
            case 2:
                // 下墙终点仍是x=0.25；同时左移到x=0.20并转到+90度。
                return runPatrolPoseTransition(
                    0.20, 2.70, 0.5 * kPi,
                    "下墙→左墙安全角点过渡");
            default:
                // 第四条巡检线最终直接停在(0.20,4.25)，不再做额外角点过渡。
                return true;
        }
    }

    SegmentResult completePatrolSegment(std::size_t segment_index) {
        const Segment& segment = segments_[segment_index];
        patrol_checkpoint_.valid = false;
        clearPatrolVisualApproach(false);
        finishPatrolMode();
        ROS_INFO("%s完成；已恢复普通导航速度%.1f和路径重规划。",
                 segment.name.c_str(), normal_navigation_speed_limit_);
        if (!runCornerTransitionAfterSegment(segment_index)) {
            ROS_ERROR("%s完成后安全角点过渡失败。", segment.name.c_str());
            return SEGMENT_ABORTED;
        }

        // V14.3保护2：
        // 非当前巡检墙的仿真目标只阻塞到“发现它时正在巡检的这一整面墙”
        // 完成。到这里才允许解除；在patrolSegment循环、现实目标停靠完成
        // 等任何更早时刻都绝不能绕过该保护。
        if (simulation_target_blocked_until_segment_end_) {
            simulation_target_blocked_until_segment_end_ = false;
            ROS_WARN(
                "%s已经完整巡检结束：此前记录的非当前巡检墙仿真目标"
                "现已解除段内停靠保护。",
                segment.name.c_str());
        }

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

        // 如果现实目标早已停靠，而本段仅延后了一个非当前墙仿真目标，
        // 现在整面墙已扫完，可以在段末正式处理。
        if (hasPendingSimulationTarget()) {
            ROS_INFO(
                "%s整面墙巡检完成，开始处理此前允许在段末执行的仿真目标。",
                segment.name.c_str());
            if (!dockPendingSimulationTarget()) {
                return SEGMENT_ABORTED;
            }
            return SEGMENT_MISSION_COMPLETE;
        }

        return SEGMENT_COMPLETE;
    }

    SegmentResult patrolSegment(std::size_t segment_index) {
        const Segment& segment = segments_[segment_index];

        // 新的一面墙必须从干净的视觉接近状态开始。
        clearPatrolVisualApproach(false);

        ROS_INFO("开始%s：(%.2f, %.2f)→(%.2f, %.2f)，"
                 "固定路径由move_base和MyPlanner跟踪；不使用/set_speed巡检。",
                 segment.name.c_str(), segment.start_x, segment.start_y,
                 segment.end_x, segment.end_y);

        Pose2D start_pose;
        if (!getRobotPose(start_pose)) return SEGMENT_ABORTED;
        PatrolStartResult start_result =
            startPatrolFromPose(segment, start_pose);
        if (start_result == PATROL_ALREADY_COMPLETE) {
            return completePatrolSegment(segment_index);
        }
        if (start_result == PATROL_START_FAILED) {
            finishPatrolMode();
            return SEGMENT_ABORTED;
        }

        int aborted_retry_count = 0;
        ros::Rate rate(control_rate_);
        while (ros::ok()) {
            ros::spinOnce();

            if (hasPendingSimulationTarget() &&
                !simulation_target_blocked_until_segment_end_) {
                ROS_INFO("检测到可立即处理的已记录仿真目标，暂停巡检并前往");
                if (!cancelPatrolGoalAndWait()) {
                    finishPatrolMode();
                    return SEGMENT_ABORTED;
                }
                finishPatrolMode();
                if (!dockPendingSimulationTarget()) {
                    return SEGMENT_ABORTED;
                }
                return SEGMENT_MISSION_COMPLETE;
            }
            if (hasPendingSimulationTarget() &&
                simulation_target_blocked_until_segment_end_) {
                ROS_INFO_THROTTLE(
                    1.0,
                    "V14.4保护：存在待停靠仿真目标，它是在现实目标尚未停靠时"
                    "于非当前巡检墙被识别；必须先完整巡检完%s，当前不允许中断。",
                    segment.name.c_str());
            }

            const actionlib::SimpleClientGoalState state =
                move_base_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                patrol_goal_active_ = false;
                return completePatrolSegment(segment_index);
            }
            if (state.isDone()) {
                patrol_goal_active_ = false;
                const std::string state_text = state.toString();

                if (state == actionlib::SimpleClientGoalState::ABORTED &&
                    aborted_retry_count < patrol_aborted_retry_count_) {
                    Pose2D retry_pose;
                    if (getRobotPose(retry_pose)) {
                        ++aborted_retry_count;
                        ROS_WARN("%s的move_base目标出现ABORTED；不终止任务，"
                                 "从当前位置重建剩余固定Path并续跑（%d/%d）。",
                                 segment.name.c_str(), aborted_retry_count,
                                 patrol_aborted_retry_count_);
                        start_result = startPatrolFromPose(segment, retry_pose);
                        if (start_result == PATROL_ALREADY_COMPLETE) {
                            return completePatrolSegment(segment_index);
                        }
                        if (start_result == PATROL_STARTED) {
                            rate.sleep();
                            continue;
                        }
                    }
                }

                finishPatrolMode();
                ROS_ERROR("%s的move_base目标异常结束：%s",
                          segment.name.c_str(), state_text.c_str());
                return SEGMENT_ABORTED;
            }

            Pose2D pose;
            if (!getRobotPose(pose)) {
                rate.sleep();
                continue;
            }

            std::vector<Box> boxes;
            const bool detection_ok = detectBoxes(boxes);

            // ----------------------------------------------------------
            // V14.7优先级：
            // 1) 当前左墙目标即将从画面左缘消失 -> 立刻停车；
            // 2) 非当前墙且距该墙<1.5m -> 保持原立即停车；
            // 3) 否则当前墙继续按1.5m->0.7m map距离减速。
            // ----------------------------------------------------------
            int current_wall_edge_selected = -1;
            BoardBoundaryEstimate current_wall_edge_estimate;

            int immediate_selected = -1;

            if (detection_ok && !boxes.empty()) {
                current_wall_edge_selected =
                    chooseCurrentWallLeftEdgeEmergencyBox(
                        boxes,
                        pose,
                        segment,
                        current_wall_edge_estimate);

                if (current_wall_edge_selected < 0) {
                    immediate_selected =
                        chooseImmediateNonCurrentBoardBox(
                            boxes,
                            pose,
                            segment);
                }
            }

            bool should_stop_for_ocr = false;

            // 普通map距离停车：仍需要停车后重新取近距离框。
            bool stop_from_current_wall_approach = false;

            // V14.8：x0<100视觉急停已经是合适OCR距离，
            // 直接使用当前触发框，不再二次NanoDet。
            bool direct_ocr_from_edge_guard = false;

            Box trigger_box{0, 0, 0, 0, 0};

            if (current_wall_edge_selected >= 0) {
                trigger_box =
                    boxes[
                        static_cast<std::size_t>(
                            current_wall_edge_selected)];

                should_stop_for_ocr = true;
                direct_ocr_from_edge_guard = true;

                ROS_WARN(
                    "V14.8画面左缘直接OCR急停：当前%s上的目标框"
                    "x0=%d<%dpx，centerX=%.1f；"
                    "该视野已属于实测适合OCR的近距离范围，"
                    "立即取消巡检goal并停车，直接使用当前触发框进入OCR，"
                    "不再执行停车后二次NanoDet取框。",
                    wallName(current_wall_edge_estimate.wall),
                    trigger_box.x0,
                    patrol_target_left_edge_stop_px_,
                    trigger_box.centerX());
            } else if (immediate_selected >= 0) {
                trigger_box =
                    boxes[
                        static_cast<std::size_t>(
                            immediate_selected)];
                should_stop_for_ocr = true;

                ROS_WARN(
                    "V14.7检测到允许立即处理的非当前巡检墙目标/"
                    "墙面估计失败目标；中断%s进入OCR。",
                    segment.name.c_str());
            } else {
                // ------------------------------------------------------
                // 当前车体左侧巡检墙目标：
                // 进入1.5m后锁存，并根据map距离连续限制MyPlanner速度。
                // ------------------------------------------------------
                if (detection_ok && !boxes.empty()) {
                    updateCurrentWallApproachFromBoxes(
                        segment_index,
                        segment,
                        pose,
                        boxes);
                }

                double distance_to_board =
                    std::numeric_limits<double>::infinity();
                if (updatePatrolVisualApproachSpeed(
                        segment,
                        pose,
                        distance_to_board)) {
                    if (distance_to_board <=
                        patrol_target_stop_distance_) {
                        should_stop_for_ocr = true;
                        stop_from_current_wall_approach = true;

                        setPatrolRuntimeSpeedLimit(
                            patrol_speed_limit_ *
                            patrol_target_min_speed_ratio_,
                            true);

                        ROS_WARN(
                            "V14.7当前左墙目标达到map停车距离："
                            "%.3fm<=%.3fm；速度上限已降至巡检速度的%.0f%%，"
                            "现在取消巡检goal并停车。",
                            distance_to_board,
                            patrol_target_stop_distance_,
                            patrol_target_min_speed_ratio_ * 100.0);
                    }
                }
            }

            if (!should_stop_for_ocr) {
                if ((!detection_ok || boxes.empty()) &&
                    !patrol_visual_approach_.valid) {
                    restorePatrolCruiseSpeedIfNeeded();
                }
                rate.sleep();
                continue;
            }

            if (!cancelPatrolGoalAndWait()) {
                clearPatrolVisualApproach(false);
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            Pose2D stopped_pose = pose;
            getRobotPose(stopped_pose);

            // 普通map距离停车仍使用V14.5的二次近距离取框。
            // V14.8的x0<100视觉急停明确跳过这里，直接使用触发急停的当前框。
            if (stop_from_current_wall_approach &&
                !direct_ocr_from_edge_guard) {
                if (!reacquireApproachBoxAfterStop(
                        segment,
                        stopped_pose,
                        trigger_box)) {
                    clearPatrolVisualApproach(false);

                    // 没框时不做错误OCR；从当前断点重新启动剩余固定Path。
                    start_result =
                        startPatrolFromPose(
                            segment,
                            stopped_pose);
                    if (start_result ==
                        PATROL_ALREADY_COMPLETE) {
                        return completePatrolSegment(
                            segment_index);
                    }
                    if (start_result ==
                        PATROL_START_FAILED) {
                        finishPatrolMode();
                        return SEGMENT_ABORTED;
                    }

                    ROS_WARN(
                        "V14.5停车后未能重新取得当前左墙目标框；"
                        "已恢复%s，等待后续重新检测。",
                        segment.name.c_str());
                    rate.sleep();
                    continue;
                }
            }

            patrol_checkpoint_.valid = true;
            patrol_checkpoint_.segment_index =
                static_cast<int>(segment_index);
            patrol_checkpoint_.stopped_progress =
                clampValue(
                    segmentProgress(
                        segment,
                        stopped_pose),
                    0.0,
                    segment.length);
            patrol_checkpoint_.stopped_pose =
                stopped_pose;

            ROS_INFO(
                "已在%s中断点停车：进度=%.3f/%.3fm。",
                segment.name.c_str(),
                patrol_checkpoint_.stopped_progress,
                segment.length);

            // 当前接近目标已经真正停车，后续交给原V14.4 OCR/停靠状态机。
            clearPatrolVisualApproach(false);

            if (direct_ocr_from_edge_guard) {
                ROS_WARN(
                    "V14.8视觉急停已完成：直接使用触发框"
                    "(%d,%d)-(%d,%d)，center=(%.1f,%.1f)"
                    "进入handleDetectedBoard/OCR。",
                    trigger_box.x0,
                    trigger_box.y0,
                    trigger_box.x1,
                    trigger_box.y1,
                    trigger_box.centerX(),
                    trigger_box.centerY());
            }

            const DetectionResult detection_result =
                handleDetectedBoard(
                    segment_index,
                    segment,
                    stopped_pose,
                    trigger_box);

            if (detection_result ==
                DETECTION_MISSION_COMPLETE) {
                finishPatrolMode();
                return SEGMENT_MISSION_COMPLETE;
            }
            if (detection_result ==
                DETECTION_ABORT) {
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            Pose2D resume_pose;
            if (!getRobotPose(resume_pose)) {
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            start_result =
                startPatrolFromPose(
                    segment,
                    resume_pose);
            if (start_result ==
                PATROL_ALREADY_COMPLETE) {
                return completePatrolSegment(
                    segment_index);
            }
            if (start_result ==
                PATROL_START_FAILED) {
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            ROS_INFO(
                "不执行额外朝向恢复；MyPlanner将按剩余固定路径"
                "自行完成初始姿态对准并继续%s。",
                segment.name.c_str());
            rate.sleep();
        }

        finishPatrolMode();
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
        bool boundary_coordinate_recorded = false;
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

            if (boundary_estimate.wall == segment.wall) {
                // 当前巡检墙保持旧行为：停车后立即加入去重名单，
                // 即使OCR失败也不在同一面墙上反复停同一块板。
                seen_board_coordinates_.push_back(boundary_estimate);
                boundary_coordinate_recorded = true;
                ROS_INFO(
                    "记录当前巡检墙新文字板坐标：%s(%.3f, %.3f)",
                    wallName(boundary_estimate.wall),
                    boundary_estimate.x, boundary_estimate.y);
            } else {
                // V14.3保护3：
                // 非当前巡检墙只有OCR最终能分类后才允许进入去重名单。
                // 若本次OCR失败，就保留重新检测机会。
                ROS_WARN(
                    "检测到非当前巡检墙候选：%s(%.3f, %.3f)；"
                    "暂不加入去重名单，等待OCR成功分类后再记录",
                    wallName(boundary_estimate.wall),
                    boundary_estimate.x, boundary_estimate.y);
            }
        } else {
            ROS_WARN("无法由NanoDet框中心估计墙面坐标，"
                     "本次继续OCR但不加入重复坐标表");
        }

        OcrRecord ocr = recognizeStaticTarget(trigger_box);
        // 与已经实测稳定的target_scan_test保持一致：以可分类关键词为准。
        // 某些OCR服务版本即使返回了有效文字，success字段也可能未置true。
        // 若只读到车间名称的通用残片，说明车辆可能因速度较快越过了
        // 类别文字。此时只执行一次逆时针30度补偿旋转和一次完整OCR。
        // 复识成功后必须刷新map位姿，保证后面的相机射线使用新朝向。
        if (ocr.category == "unknown" && hasWorkshopFragment(ocr.text)) {
            ROS_WARN("OCR只识别到车间通用残片“%s”，"
                     "未命中类别关键字；准备原地逆时针转%.1f度复识一次",
                     ocr.text.c_str(), ocr_recovery_turn_deg_);

            Pose2D recovery_pose;
            if (!rotateCounterClockwiseForOcr(recovery_pose)) {
                if (boundary_estimate.valid &&
                    boundary_estimate.wall != segment.wall &&
                    !boundary_coordinate_recorded) {
                    ROS_WARN(
                        "V14.3保护：非当前巡检墙候选OCR补偿旋转失败，"
                        "坐标%s(%.3f, %.3f)不加入去重名单",
                        wallName(boundary_estimate.wall),
                        boundary_estimate.x, boundary_estimate.y);
                }
                addUnknownCandidate(
                    segment_index,
                    stopped_pose,
                    ocr.box,
                    boundary_estimate,
                    "OCR通用残片补偿旋转失败");

                ROS_WARN("OCR补偿旋转失败，本次不再复识，继续%s",
                         segment.name.c_str());
                return DETECTION_CONTINUE;
            }
            stopped_pose = recovery_pose;

            // 旋转后目标在图像中的位置会变化。优先用新一帧NanoDet框
            // 作为OCR关联参考；若该帧未返回框，OCR服务仍用上次框尝试。
            Box retry_reference = ocr.box;
            std::vector<Box> retry_boxes;
            if (detectBoxes(retry_boxes) && !retry_boxes.empty()) {
                const int retry_index = chooseClosestCenterBox(retry_boxes);
                if (retry_index >= 0) {
                    retry_reference = retry_boxes[
                        static_cast<std::size_t>(retry_index)];
                }
            } else {
                ROS_WARN("补偿旋转后NanoDet未返回文字框，"
                         "仍调用OCR完成唯一一次复识");
            }

            ocr = recognizeStaticTarget(retry_reference);
            if (ocr.category == "unknown") {
                if (boundary_estimate.valid &&
                    boundary_estimate.wall != segment.wall &&
                    !boundary_coordinate_recorded) {
                    ROS_WARN(
                        "V14.3保护：非当前巡检墙%s(%.3f, %.3f)"
                        "补偿复识后仍无法分类，不加入去重名单",
                        wallName(boundary_estimate.wall),
                        boundary_estimate.x, boundary_estimate.y);
                }
                addUnknownCandidate(
                    segment_index,
                    stopped_pose,
                    ocr.box,
                    boundary_estimate,
                    "OCR补偿旋转后仍为unknown");

                ROS_WARN("补偿旋转后的唯一一次复识仍无法分类：%s；"
                         "继续%s",
                         ocr.text.c_str(), segment.name.c_str());
                return DETECTION_CONTINUE;
            }
            ROS_INFO("补偿旋转复识成功：%s，分类=%s",
                     ocr.text.c_str(), categoryChinese(ocr.category));
        }
        if (ocr.category == "unknown") {
            if (boundary_estimate.valid &&
                boundary_estimate.wall != segment.wall &&
                !boundary_coordinate_recorded) {
                ROS_WARN(
                    "V14.3保护：非当前巡检墙%s(%.3f, %.3f)本次OCR无法分类，"
                    "不加入去重名单；恢复巡检后允许再次检测并重新OCR",
                    wallName(boundary_estimate.wall),
                    boundary_estimate.x, boundary_estimate.y);
            }
            addUnknownCandidate(
                segment_index,
                stopped_pose,
                ocr.box,
                boundary_estimate,
                "巡检OCR最终分类为unknown");

            ROS_WARN(
                "本次文字无法分类；已按V14.9记录为候选点，继续%s",
                segment.name.c_str());
            return DETECTION_CONTINUE;
        }

        // V14.3保护3：非当前巡检墙只有到这里（OCR最终分类成功）
        // 才加入不重复检测名单。
        if (boundary_estimate.valid &&
            boundary_estimate.wall != segment.wall &&
            !boundary_coordinate_recorded) {
            seen_board_coordinates_.push_back(boundary_estimate);
            boundary_coordinate_recorded = true;
            ROS_INFO(
                "非当前巡检墙候选OCR分类成功为%s；"
                "现在加入去重名单：%s(%.3f, %.3f)",
                categoryChinese(ocr.category),
                wallName(boundary_estimate.wall),
                boundary_estimate.x, boundary_estimate.y);
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

        resolveUnknownCandidateNear(
            static_board_estimate,
            std::string("后续巡检已成功分类为") +
                categoryChinese(ocr.category));

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
                // 只对首次有效记录绑定“是否必须等本段结束”。
                // 后续重复/误识别不得覆盖已经确认的目标与保护状态。
                // V14.4例外：
                // 非当前巡检墙仿真目标通常需要等当前整面墙结束；
                // 但如果现实目标已经完成停靠，则双目标任务只剩仿真目标，
                // 此时允许立即中断当前墙直接前往，不再设置段末锁。
                simulation_target_blocked_until_segment_end_ =
                    !target_on_current_left_wall && !real_docked_;
            }
            simulation_target_pending_ = true;

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

            if (simulation_target_blocked_until_segment_end_) {
                ROS_WARN(
                    "V14.4保护：仿真目标位于非当前巡检墙%s，且现实目标尚未停靠；"
                    "必须先完整巡检完当前%s，段末才允许前往该仿真目标。",
                    wallName(static_board_estimate.wall),
                    segment.name.c_str());
                return DETECTION_CONTINUE;
            }

            if (!real_docked_) {
                ROS_INFO(
                    "仿真目标已记录，但现实目标尚未停靠；"
                    "继续巡检寻找现实目标。");
                return DETECTION_CONTINUE;
            }

            if (!target_on_current_left_wall) {
                ROS_WARN(
                    "V14.4例外生效：现实目标已经完成停靠，"
                    "当前识别到的仿真目标位于非当前巡检墙%s；"
                    "允许立即中断%s并直接前往仿真目标。",
                    wallName(static_board_estimate.wall),
                    segment.name.c_str());
            } else {
                ROS_INFO(
                    "现实目标已经完成停靠，当前巡检墙识别到仿真目标；"
                    "立即前往停靠。");
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
            releaseCameraBeforePredockNavigation("现实目标");
            if (!navigateToPose(observation.pose.x,
                                observation.pose.y,
                                observation.pose.yaw,
                                "前往现实目标停靠导航点")) {
                return DETECTION_ABORT;
            }
            if (!dockTarget(real_observation_, "现实目标", false)) {
                return DETECTION_ABORT;
            }
            real_docked_ = true;

            if (hasPendingSimulationTarget() &&
                !simulation_target_blocked_until_segment_end_) {
                ROS_INFO("仿真目标此前已找到且不受段末保护；"
                         "现实目标停靠完成后立即处理，不恢复巡检");
                if (!dockPendingSimulationTarget()) {
                    return DETECTION_ABORT;
                }
                return DETECTION_MISSION_COMPLETE;
            }

            if (hasPendingSimulationTarget() &&
                simulation_target_blocked_until_segment_end_) {
                ROS_WARN(
                    "V14.3保护：仿真目标此前虽已找到，但属于非当前巡检墙；"
                    "现实目标停靠完成后仍必须返回%s继续巡检，"
                    "直到整面墙完成后才能处理仿真目标。",
                    segment.name.c_str());
            } else {
                ROS_INFO("尚未记录仿真目标；现实目标停靠后直接直线后退"
                         "至当前平移巡检线");
            }
            if (!retreatToPatrolLine(segment)) {
                return DETECTION_ABORT;
            }
            if (!openCamera()) return DETECTION_ABORT;
            ROS_INFO("已回到%s；不额外恢复前进朝向，"
                     "后续由局部规划器根据剩余路径自行对准",
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
        releaseCameraBeforePredockNavigation("延后处理的现实目标");
        if (!navigateToPose(real_observation_.pose.x,
                            real_observation_.pose.y,
                            real_observation_.pose.yaw,
                            "前往延后处理的现实目标")) {
            return DETECTION_ABORT;
        }
        if (!dockTarget(real_observation_,
                        "延后处理的现实目标", false)) {
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
        if (simulation_target_blocked_until_segment_end_) {
            ROS_ERROR(
                "V14.4保护拒绝停靠：该仿真目标是在现实目标尚未停靠时于"
                "非当前巡检墙被识别，当前整面墙尚未完成，禁止提前调用"
                "dockPendingSimulationTarget()");
            return false;
        }

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

        releaseCameraBeforePredockNavigation("仿真目标");
        if (!navigateToPose(simulation_observation_.pose.x,
                            simulation_observation_.pose.y,
                            simulation_observation_.pose.yaw,
                            "前往已记录的仿真目标")) {
            return false;
        }

        if (!dockTarget(simulation_observation_,
                        "仿真目标", false)) {
            return false;
        }

        simulation_docked_ = true;
        simulation_target_pending_ = false;
        simulation_target_blocked_until_segment_end_ = false;
        ROS_INFO("已完成待停靠仿真目标，双目标任务完成");
        return true;
    }

    bool rotateToDockingRecoveryYaw(double target_yaw,
                                    const std::string& action_name) {
        const double tolerance =
            ocr_recovery_turn_tolerance_deg_ * kPi / 180.0;
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(ocr_recovery_turn_timeout_);
        int stable_frames = 0;
        ros::Rate rate(20.0);

        ROS_INFO("%s：目标朝向=%.1f度",
                 action_name.c_str(), target_yaw * 180.0 / kPi);
        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                stable_frames = 0;
                rate.sleep();
                continue;
            }

            const double yaw_error = normalizeAngle(target_yaw - pose.yaw);
            if (std::fabs(yaw_error) <= tolerance) {
                publishVelocity(0.0, 0.0, 0.0);
                ++stable_frames;
                if (stable_frames >= ocr_recovery_turn_stable_frames_) {
                    stopRobot();
                    Pose2D settled_pose = pose;
                    getRobotPose(settled_pose);
                    ROS_INFO("%s完成：最终朝向=%.1f度，误差=%.2f度",
                             action_name.c_str(),
                             settled_pose.yaw * 180.0 / kPi,
                             normalizeAngle(
                                 target_yaw - settled_pose.yaw) *
                                 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
                double angular_z = clampValue(
                    ocr_recovery_turn_kp_ * yaw_error,
                    -ocr_recovery_turn_max_speed_,
                    ocr_recovery_turn_max_speed_);
                if (std::fabs(angular_z) <
                    ocr_recovery_turn_min_speed_) {
                    angular_z = yaw_error >= 0.0
                                    ? ocr_recovery_turn_min_speed_
                                    : -ocr_recovery_turn_min_speed_;
                }
                publishVelocity(0.0, 0.0, angular_z);
                ROS_INFO_THROTTLE(
                    0.5,
                    "%s：剩余角度=%.2f度，angular.z=%.3f",
                    action_name.c_str(),
                    yaw_error * 180.0 / kPi, angular_z);
            }
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("%s在%.1f秒内未完成",
                  action_name.c_str(), ocr_recovery_turn_timeout_);
        return false;
    }

    bool refreshDockingDetectionFrames(const std::string& reason) {
        if (!openCamera()) {
            ROS_ERROR("[停靠诊断][帧刷新] %s：摄像头未能打开",
                      reason.c_str());
            return false;
        }

        // V14.2：这里仅用于±30度恢复旋转后的刷新。
        // NanoDet的detect_start=-3内部连续cap.grab()两帧；
        // 直接连续调用，不额外sleep、不再读取并丢弃正常检测帧。
        ROS_WARN(
            "[停靠诊断][帧刷新] %s：即时清缓存开始，"
            "clear_calls=%d，无固定等待、无额外丢弃帧",
            reason.c_str(), docking_refresh_clear_calls_);

        for (int i = 0; i < docking_refresh_clear_calls_; ++i) {
            ros_nanodet::detect_result_srv clear_service;
            clear_service.request.detect_start = -3;
            if (!detect_client_.call(clear_service)) {
                ROS_ERROR(
                    "[停靠诊断][帧刷新] %s：第%d/%d次-3清缓存调用失败",
                    reason.c_str(), i + 1, docking_refresh_clear_calls_);
                return false;
            }
        }

        ROS_WARN(
            "[停靠诊断][帧刷新] %s：即时清缓存完成；"
            "下一次detect直接作为正式计算帧",
            reason.c_str());
        return true;
    }

    double boardCoordinateDistance(
            const TargetObservation& observation,
            const BoardBoundaryEstimate& estimate) const {
        if (!observation.board_valid ||
            observation.board_wall != estimate.wall) {
            return std::numeric_limits<double>::infinity();
        }
        return distance2D(observation.board_x, observation.board_y,
                          estimate.x, estimate.y);
    }

    bool detectDockingRefinedObservation(
            const TargetObservation& old_observation,
            const std::string& scan_name,
            double final_standoff,
            TargetObservation& refined_observation,
            bool force_refresh,
            Box* matched_box_out = nullptr,
            BoardBoundaryEstimate* matched_board_out = nullptr,
            bool match_only = false) {
        // 正常第二段：第一段导航前已关闭摄像头，到点后刚重新打开，
        // 不存在旧会话缓存，因此立即使用正式检测帧。
        // ±30度恢复：旋转期间相机保持打开但没有持续read，才需要-3即时刷新。
        if (force_refresh &&
            !refreshDockingDetectionFrames(scan_name)) {
            ROS_ERROR("%s刷新NanoDet实时帧失败", scan_name.c_str());
            return false;
        }

        for (int attempt = 0;
             attempt < docking_recovery_detection_attempts_ && ros::ok();
             ++attempt) {
            ros::spinOnce();
            std::vector<Box> boxes;
            const bool detection_ok = detectBoxes(boxes);
            if (!detection_ok || boxes.empty()) {
                ROS_WARN("%s第%d/%d次未检测到文字板",
                         scan_name.c_str(), attempt + 1,
                         docking_recovery_detection_attempts_);
                ros::Duration(
                    docking_recovery_detection_interval_).sleep();
                continue;
            }

            Pose2D pose;
            if (!getRobotPose(pose)) {
                ROS_WARN("%s第%d/%d次检测到框，但无法读取机器人位姿",
                         scan_name.c_str(), attempt + 1,
                         docking_recovery_detection_attempts_);
                ros::Duration(
                    docking_recovery_detection_interval_).sleep();
                continue;
            }

            const double image_center =
                0.5 * static_cast<double>(image_width_);
            ROS_WARN(
                "[停靠诊断][本帧] %s attempt=%d/%d："
                "小车map坐标=(%.4f, %.4f, %.2f度)，"
                "第一段预停靠目标=(%.4f, %.4f, %.2f度)，"
                "第一次板坐标=%s(%.4f, %.4f)，检测框数=%d",
                scan_name.c_str(), attempt + 1,
                docking_recovery_detection_attempts_,
                pose.x, pose.y, pose.yaw * 180.0 / kPi,
                old_observation.pose.x, old_observation.pose.y,
                old_observation.pose.yaw * 180.0 / kPi,
                old_observation.board_valid
                    ? wallName(old_observation.board_wall)
                    : "未知墙",
                old_observation.board_x, old_observation.board_y,
                static_cast<int>(boxes.size()));

            int selected = -1;
            double best_score = std::numeric_limits<double>::infinity();
            BoardBoundaryEstimate best_estimate;
            for (std::size_t i = 0; i < boxes.size(); ++i) {
                const double pixel_error =
                    image_center - boxes[i].centerX();
                const double relative_yaw =
                    std::atan2(pixel_error, camera_fx_);
                const double ray_yaw =
                    pose.yaw +
                    camera_yaw_offset_deg_ * kPi / 180.0 +
                    relative_yaw;

                BoardBoundaryEstimate estimate;
                if (!estimateBoardBoundary(pose, boxes[i], estimate)) {
                    ROS_WARN(
                        "[停靠诊断][候选%d] 框=(%d,%d)-(%d,%d)，"
                        "center=(%.1f,%.1f)，pixel_error=%.1fpx，"
                        "relative_yaw=%.2f度，ray_yaw=%.2f度；"
                        "墙面射线无有效交点",
                        static_cast<int>(i),
                        boxes[i].x0, boxes[i].y0,
                        boxes[i].x1, boxes[i].y1,
                        boxes[i].centerX(), boxes[i].centerY(),
                        pixel_error,
                        relative_yaw * 180.0 / kPi,
                        ray_yaw * 180.0 / kPi);
                    continue;
                }

                const double old_distance =
                    old_observation.board_valid
                        ? boardCoordinateDistance(old_observation, estimate)
                        : std::numeric_limits<double>::infinity();
                if (std::isfinite(old_distance)) {
                    ROS_WARN(
                        "[停靠诊断][候选%d] 框=(%d,%d)-(%d,%d)，"
                        "center=(%.1f,%.1f)，pixel_error=%.1fpx[%s]，"
                        "relative_yaw=%.2f度，ray_yaw=%.2f度；"
                        "计算板坐标=%s(%.4f,%.4f)，"
                        "相对第一次板坐标距离=%.4fm",
                        static_cast<int>(i),
                        boxes[i].x0, boxes[i].y0,
                        boxes[i].x1, boxes[i].y1,
                        boxes[i].centerX(), boxes[i].centerY(),
                        pixel_error,
                        pixel_error < 0.0 ? "板在画面右侧"
                                          : (pixel_error > 0.0
                                                 ? "板在画面左侧"
                                                 : "板在画面中线"),
                        relative_yaw * 180.0 / kPi,
                        ray_yaw * 180.0 / kPi,
                        wallName(estimate.wall),
                        estimate.x, estimate.y, old_distance);
                } else {
                    ROS_WARN(
                        "[停靠诊断][候选%d] 框=(%d,%d)-(%d,%d)，"
                        "center=(%.1f,%.1f)，pixel_error=%.1fpx[%s]，"
                        "relative_yaw=%.2f度，ray_yaw=%.2f度；"
                        "计算板坐标=%s(%.4f,%.4f)，"
                        "与第一次板不在同墙/无可比距离",
                        static_cast<int>(i),
                        boxes[i].x0, boxes[i].y0,
                        boxes[i].x1, boxes[i].y1,
                        boxes[i].centerX(), boxes[i].centerY(),
                        pixel_error,
                        pixel_error < 0.0 ? "板在画面右侧"
                                          : (pixel_error > 0.0
                                                 ? "板在画面左侧"
                                                 : "板在画面中线"),
                        relative_yaw * 180.0 / kPi,
                        ray_yaw * 180.0 / kPi,
                        wallName(estimate.wall),
                        estimate.x, estimate.y);
                }

                double score = 0.0;
                if (old_observation.board_valid) {
                    // 第二阶段只允许关联第一次确认的同一面墙。
                    // 这一步替代旧版“挑画面中心最近框”的不可靠策略。
                    if (estimate.wall != old_observation.board_wall) {
                        continue;
                    }
                    score = boardCoordinateDistance(
                        old_observation, estimate);
                    if (!std::isfinite(score) ||
                        score > docking_refine_max_board_shift_) {
                        continue;
                    }
                } else {
                    // 仅为兼容历史状态；正常新记录一定有board坐标。
                    score = std::fabs(
                        boxes[i].centerX() - 0.5 * image_width_);
                }

                if (score < best_score) {
                    best_score = score;
                    selected = static_cast<int>(i);
                    best_estimate = estimate;
                }
            }

            if (selected >= 0) {
                const Box& box = boxes[
                    static_cast<std::size_t>(selected)];

                if (matched_box_out) {
                    *matched_box_out = box;
                }
                if (matched_board_out) {
                    *matched_board_out = best_estimate;
                }

                // V14.6边缘视野预检查：
                // 此模式只确认“是不是原来那块板”并返回bbox/墙面坐标，
                // 严格不生成最终停靠点。若框位于左右1/4，调用方会先旋转。
                if (match_only) {
                    ROS_WARN(
                        "[停靠诊断][边缘预检查] %s匹配到原目标："
                        "center=(%.1f,%.1f)，板=%s(%.4f,%.4f)，"
                        "相对第一次板坐标修正=%.4fm；"
                        "当前尚未计算最终停靠点。",
                        scan_name.c_str(),
                        box.centerX(), box.centerY(),
                        wallName(best_estimate.wall),
                        best_estimate.x, best_estimate.y,
                        old_observation.board_valid ? best_score : 0.0);
                    return true;
                }

                const int segment_index =
                    segmentIndexForWall(best_estimate.wall);
                if (segment_index >= 0 &&
                    makeDockingObservationAtStandoff(
                        segment_index,
                        old_observation.category,
                        best_estimate,
                        final_standoff,
                        refined_observation)) {
                    const double pixel_error =
                        image_center - box.centerX();
                    const double world_dx =
                        refined_observation.pose.x - pose.x;
                    const double world_dy =
                        refined_observation.pose.y - pose.y;
                    const double c = std::cos(pose.yaw);
                    const double sn = std::sin(pose.yaw);
                    const double base_dx =
                        c * world_dx + sn * world_dy;
                    const double base_dy =
                        -sn * world_dx + c * world_dy;

                    ROS_WARN(
                        "[停靠诊断][最终计算] ===== %s =====",
                        scan_name.c_str());
                    ROS_WARN(
                        "[停靠诊断][最终计算] 小车实际map坐标="
                        "(%.4f, %.4f, %.2f度)",
                        pose.x, pose.y, pose.yaw * 180.0 / kPi);
                    ROS_WARN(
                        "[停靠诊断][最终计算] 第一段预停靠目标="
                        "(%.4f, %.4f, %.2f度)；实际到点误差="
                        "dx=%.4f, dy=%.4f, dist=%.4fm",
                        old_observation.pose.x,
                        old_observation.pose.y,
                        old_observation.pose.yaw * 180.0 / kPi,
                        pose.x - old_observation.pose.x,
                        pose.y - old_observation.pose.y,
                        distance2D(pose.x, pose.y,
                                   old_observation.pose.x,
                                   old_observation.pose.y));
                    ROS_WARN(
                        "[停靠诊断][最终计算] 选中框center=(%.1f,%.1f)，"
                        "image_center=%.1f，pixel_error=%.1fpx[%s]",
                        box.centerX(), box.centerY(),
                        image_center, pixel_error,
                        pixel_error < 0.0 ? "板在画面右侧"
                                          : (pixel_error > 0.0
                                                 ? "板在画面左侧"
                                                 : "板在画面中线"));
                    ROS_WARN(
                        "[停靠诊断][最终计算] 第一次板位置="
                        "%s(%.4f, %.4f)；第二次计算板位置="
                        "%s(%.4f, %.4f)；板坐标修正距离=%.4fm",
                        old_observation.board_valid
                            ? wallName(old_observation.board_wall)
                            : "未知墙",
                        old_observation.board_x,
                        old_observation.board_y,
                        wallName(best_estimate.wall),
                        best_estimate.x, best_estimate.y,
                        old_observation.board_valid ? best_score : 0.0);
                    ROS_WARN(
                        "[停靠诊断][最终计算] approach_stop_distance="
                        "%.4fm；最终move_base停靠坐标="
                        "(%.4f, %.4f, %.2f度)",
                        final_standoff,
                        refined_observation.pose.x,
                        refined_observation.pose.y,
                        refined_observation.pose.yaw * 180.0 / kPi);
                    ROS_WARN(
                        "[停靠诊断][最终计算] 从当前小车到最终点："
                        "map增量=(dx=%.4f,dy=%.4f)，"
                        "车体坐标增量=(forward=%.4f,left=%.4f)，"
                        "距离=%.4fm",
                        world_dx, world_dy, base_dx, base_dy,
                        std::hypot(world_dx, world_dy));
                    ROS_WARN(
                        "[停靠诊断][最终计算] ============================");

                    ROS_INFO(
                        "%s成功：框中心=(%.1f, %.1f)，"
                        "旧墙面坐标=%s(%.3f, %.3f)，"
                        "新墙面坐标=%s(%.3f, %.3f)，修正量=%.3fm",
                        scan_name.c_str(),
                        box.centerX(), box.centerY(),
                        old_observation.board_valid
                            ? wallName(old_observation.board_wall)
                            : "未知墙",
                        old_observation.board_x,
                        old_observation.board_y,
                        wallName(best_estimate.wall),
                        best_estimate.x,
                        best_estimate.y,
                        old_observation.board_valid ? best_score : 0.0);
                    return true;
                }
            }

            ROS_WARN(
                "%s第%d/%d次检测到%d个框，但没有与旧目标同墙且"
                "墙面坐标误差<=%.2fm的有效框",
                scan_name.c_str(), attempt + 1,
                docking_recovery_detection_attempts_,
                static_cast<int>(boxes.size()),
                docking_refine_max_board_shift_);
            ros::Duration(
                docking_recovery_detection_interval_).sleep();
        }
        return false;
    }

    bool recoverDockingObservation(TargetObservation& observation,
                                   const std::string& target_name,
                                   double final_standoff) {
        const TargetObservation old_observation = observation;
        const double center_yaw = old_observation.pose.yaw;
        const double turn = docking_recovery_turn_deg_ * kPi / 180.0;
        const double scan_yaws[2] = {
            normalizeAngle(center_yaw + turn),
            normalizeAngle(center_yaw - turn)};
        const char* scan_names[2] = {
            "左侧恢复识别", "右侧恢复识别"};

        ROS_WARN("%s在预停靠点无法可靠复定位原目标，开始左右视野恢复；"
                 "基准朝向=%.1f度",
                 target_name.c_str(), center_yaw * 180.0 / kPi);
        for (int side = 0; side < 2 && ros::ok(); ++side) {
            if (!rotateToDockingRecoveryYaw(
                    scan_yaws[side], scan_names[side])) {
                ROS_WARN("%s旋转失败，继续尝试下一恢复方向",
                         scan_names[side]);
                continue;
            }

            TargetObservation recovered;
            if (!detectDockingRefinedObservation(
                    old_observation, scan_names[side],
                    final_standoff, recovered, true)) {
                ROS_WARN("%s未找到与旧目标匹配的文字板",
                         scan_names[side]);
                continue;
            }

            observation = recovered;
            ROS_WARN(
                "%s恢复识别成功：最终墙面坐标=%s(%.3f, %.3f)，"
                "最终move_base点=(%.3f, %.3f, %.1f度)",
                target_name.c_str(),
                wallName(observation.board_wall),
                observation.board_x, observation.board_y,
                observation.pose.x, observation.pose.y,
                observation.pose.yaw * 180.0 / kPi);
            return true;
        }

        stopRobot();
        ROS_ERROR("%s左右%.1f度恢复识别均未找到原目标板",
                  target_name.c_str(), docking_recovery_turn_deg_);
        return false;
    }

    bool dockTarget(TargetObservation& observation,
                    const std::string& target_name,
                    bool camera_is_already_open) {
        if (!camera_is_already_open && !openCamera()) return false;

        ROS_INFO(
            "%s开始V14.2两段式停靠：第一段预停靠已完成；"
            "摄像头刚从全新V4L2会话打开，立即进行第二段正式视觉定位，"
            "不再固定等待、不再丢弃正常检测帧。",
            target_name.c_str());

        Pose2D before_refine_pose;
        if (getRobotPose(before_refine_pose)) {
            ROS_WARN(
                "[停靠诊断][第二段入口] 小车实际map坐标="
                "(%.4f, %.4f, %.2f度)；"
                "第一段预停靠目标=(%.4f, %.4f, %.2f度)；"
                "第一次板位置=%s(%.4f, %.4f)",
                before_refine_pose.x, before_refine_pose.y,
                before_refine_pose.yaw * 180.0 / kPi,
                observation.pose.x, observation.pose.y,
                observation.pose.yaw * 180.0 / kPi,
                observation.board_valid
                    ? wallName(observation.board_wall)
                    : "未知墙",
                observation.board_x, observation.board_y);
        } else {
            ROS_WARN("[停靠诊断][第二段入口] 无法读取当前小车map坐标");
        }

        TargetObservation final_observation;

        // --------------------------------------------------------------
        // V14.6边缘视野保护
        //
        // 到达第一段临时停靠点后：
        // 1. 先只匹配同一块板，严格不计算最终停靠点；
        // 2. 若centerX位于左/右1/4，先向对应方向转30度；
        // 3. 转完-3清缓存并重新匹配；
        // 4. 只有此时才根据新帧计算最终approach_stop_distance停靠点。
        // --------------------------------------------------------------
        Box precheck_box{0, 0, 0, 0, 0};
        BoardBoundaryEstimate precheck_board;
        TargetObservation unused_precheck_observation;

        const bool precheck_ok =
            detectDockingRefinedObservation(
                observation,
                target_name + "第二段边缘视野预检查",
                approach_stop_distance_,
                unused_precheck_observation,
                false,
                &precheck_box,
                &precheck_board,
                true);

        bool refined = false;

        if (precheck_ok) {
            const double left_quarter =
                0.25 * static_cast<double>(image_width_);
            const double right_quarter =
                0.75 * static_cast<double>(image_width_);

            const bool in_left_quarter =
                precheck_box.centerX() <= left_quarter;
            const bool in_right_quarter =
                precheck_box.centerX() >= right_quarter;

            if (in_left_quarter || in_right_quarter) {
                Pose2D current_pose;
                if (!getRobotPose(current_pose)) {
                    ROS_ERROR(
                        "%s边缘视野30度修正前无法读取当前map位姿",
                        target_name.c_str());
                    return false;
                }

                const double turn =
                    docking_recovery_turn_deg_ * kPi / 180.0;

                // 当前图像/相机语义已在实车确认：
                // 目标在画面左侧 -> 目标位于车体左侧 -> yaw正方向(CCW)；
                // 目标在画面右侧 -> yaw负方向(CW)。
                const double signed_turn =
                    in_left_quarter ? turn : -turn;
                const double adjusted_yaw =
                    normalizeAngle(
                        current_pose.yaw + signed_turn);

                ROS_WARN(
                    "V14.6临时停靠边缘保护：%s匹配框centerX=%.1f，"
                    "图像宽=%d，左1/4<=%.1f，右1/4>=%.1f；"
                    "框位于%s侧，先向%s旋转%.1f度："
                    "当前yaw=%.1f度 -> 目标yaw=%.1f度。"
                    "此时尚未计算最终停靠点。",
                    target_name.c_str(),
                    precheck_box.centerX(),
                    image_width_,
                    left_quarter,
                    right_quarter,
                    in_left_quarter ? "左" : "右",
                    in_left_quarter
                        ? "左/逆时针"
                        : "右/顺时针",
                    docking_recovery_turn_deg_,
                    current_pose.yaw * 180.0 / kPi,
                    adjusted_yaw * 180.0 / kPi);

                if (rotateToDockingRecoveryYaw(
                        adjusted_yaw,
                        target_name + "临时停靠边缘30度修正")) {
                    // 旋转过程中摄像头保持打开但没有持续read，
                    // force_refresh=true会先执行既有的NanoDet -3缓存刷新。
                    refined =
                        detectDockingRefinedObservation(
                            observation,
                            target_name + "边缘修正后最终视觉复定位",
                            approach_stop_distance_,
                            final_observation,
                            true);

                    if (refined) {
                        ROS_WARN(
                            "V14.6 %s边缘30度修正成功："
                            "已使用转向后的新帧重新计算最终停靠点。",
                            target_name.c_str());
                    }
                } else {
                    ROS_WARN(
                        "%s边缘30度修正旋转失败，"
                        "不使用转向前边缘帧计算最终点。",
                        target_name.c_str());
                }
            } else {
                // 中间二分之一不需要再次取帧：
                // 预检查得到的同墙板坐标就是当前实时新V4L2帧结果，
                // 现在才正式生成最终停靠点。
                const int segment_index =
                    segmentIndexForWall(precheck_board.wall);
                if (segment_index >= 0) {
                    refined =
                        makeDockingObservationAtStandoff(
                            segment_index,
                            observation.category,
                            precheck_board,
                            approach_stop_distance_,
                            final_observation);
                }

                if (refined) {
                    ROS_INFO(
                        "%s临时停靠框centerX=%.1f位于图像中间二分之一"
                        "(%.1f, %.1f)，不额外旋转；"
                        "现在正式计算最终停靠点。",
                        target_name.c_str(),
                        precheck_box.centerX(),
                        left_quarter,
                        right_quarter);
                }
            }
        }

        if (!refined) {
            ROS_WARN(
                "%s当前朝向/边缘30度修正后未能可靠复定位原目标；"
                "启动保留的±%.1f度视觉恢复。",
                target_name.c_str(), docking_recovery_turn_deg_);
            final_observation = observation;
            if (!recoverDockingObservation(
                    final_observation, target_name,
                    approach_stop_distance_)) {
                ROS_ERROR("%s最终视觉复定位失败，不执行盲目前进",
                          target_name.c_str());
                return false;
            }
        }

        ROS_INFO(
            "%s最终停靠目标：板=%s(%.3f, %.3f)，"
            "沿用原approach_stop_distance=%.3fm，"
            "move_base目标=(%.3f, %.3f, %.1f度)",
            target_name.c_str(),
            wallName(final_observation.board_wall),
            final_observation.board_x,
            final_observation.board_y,
            approach_stop_distance_,
            final_observation.pose.x,
            final_observation.pose.y,
            final_observation.pose.yaw * 180.0 / kPi);

        // 保持摄像头开启直到最终目标已计算完成；最终move_base不再依赖
        // 独立linear.y横移或雷达linear.x前进，因此不存在旧版误判居中后盲目前进。
        observation = final_observation;
        if (!navigateToPose(observation.pose.x,
                            observation.pose.y,
                            observation.pose.yaw,
                            target_name + "最终视觉停靠点")) {
            ROS_ERROR("%s最终move_base停靠失败", target_name.c_str());
            return false;
        }

        closeCamera();
        ROS_INFO("%s两段式move_base停靠成功", target_name.c_str());
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

    bool rotateCounterClockwiseForOcr(Pose2D& pose_after_rotation) {
        Pose2D start_pose;
        if (!getRobotPose(start_pose)) {
            ROS_ERROR("OCR补偿旋转前无法读取机器人位姿");
            return false;
        }

        const double target_yaw = normalizeAngle(
            start_pose.yaw + ocr_recovery_turn_deg_ * kPi / 180.0);
        const double tolerance =
            ocr_recovery_turn_tolerance_deg_ * kPi / 180.0;
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(ocr_recovery_turn_timeout_);
        int stable_frames = 0;
        ros::Rate rate(20.0);

        ROS_INFO("OCR补偿旋转开始：当前朝向=%.1f度，"
                 "目标朝向=%.1f度（逆时针%.1f度）",
                 start_pose.yaw * 180.0 / kPi,
                 target_yaw * 180.0 / kPi,
                 ocr_recovery_turn_deg_);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                stable_frames = 0;
                rate.sleep();
                continue;
            }

            const double yaw_error = normalizeAngle(target_yaw - pose.yaw);
            if (std::fabs(yaw_error) <= tolerance) {
                publishVelocity(0.0, 0.0, 0.0);
                ++stable_frames;
                if (stable_frames >= ocr_recovery_turn_stable_frames_) {
                    stopRobot();
                    pose_after_rotation = pose;
                    ros::Duration(ocr_recovery_settle_time_).sleep();
                    // 静止后再读一次最终位姿；短暂TF失败时保留最后一帧。
                    getRobotPose(pose_after_rotation);
                    ROS_INFO("OCR补偿旋转完成：最终朝向=%.1f度，"
                             "目标误差=%.2f度",
                             pose_after_rotation.yaw * 180.0 / kPi,
                             normalizeAngle(
                                 target_yaw - pose_after_rotation.yaw) *
                                 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
                double angular_z = clampValue(
                    ocr_recovery_turn_kp_ * yaw_error,
                    -ocr_recovery_turn_max_speed_,
                    ocr_recovery_turn_max_speed_);
                if (std::fabs(angular_z) <
                    ocr_recovery_turn_min_speed_) {
                    angular_z = yaw_error >= 0.0
                                    ? ocr_recovery_turn_min_speed_
                                    : -ocr_recovery_turn_min_speed_;
                }
                publishVelocity(0.0, 0.0, angular_z);
                ROS_INFO_THROTTLE(
                    0.5,
                    "OCR补偿旋转中：剩余角度=%.2f度，angular.z=%.3f",
                    yaw_error * 180.0 / kPi, angular_z);
            }
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("OCR补偿旋转在%.1f秒内未完成",
                  ocr_recovery_turn_timeout_);
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

    bool alignBoardLaterally(double desired_yaw, bool& board_seen) {
        board_seen = false;
        if (!openCamera()) return false;
        ROS_INFO("横移居中：保持朝向%.1f度",
                 desired_yaw * 180.0 / kPi);
        const double image_center = 0.5 * image_width_;
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(lateral_align_timeout_);
        bool have_tracked_box = false;
        Box tracked_box{0, 0, 0, 0, 0};
        // -1表示首次有效框中心位于左半屏，+1表示位于右半屏。
        // 横移期间一旦跨越屏幕中线，就立即停车并判定已经对准，
        // 防止底盘因最小横移速度和制动距离而在中线两侧来回修正。
        int initial_target_half = 0;
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
            if (selected >= 0) board_seen = true;
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

            const int current_target_half =
                tracked_box.centerX() < image_center
                    ? -1
                    : (tracked_box.centerX() > image_center ? 1 : 0);
            if (initial_target_half == 0 && current_target_half != 0) {
                initial_target_half = current_target_half;
                ROS_INFO("横移居中：首次目标框位于屏幕%s半边，"
                         "中心x=%.1f，屏幕中线x=%.1f",
                         initial_target_half < 0 ? "左" : "右",
                         tracked_box.centerX(), image_center);
            } else if (current_target_half != 0 &&
                       current_target_half != initial_target_half) {
                // 必须在计算并发布下一次反向横移命令之前停车。
                stopRobot();
                ROS_INFO("横移居中完成：目标框已从屏幕%s半边跨越到%s半边，"
                         "立即停车；目标中心x=%.1f，屏幕中线x=%.1f，"
                         "像素误差=%.1fpx",
                         initial_target_half < 0 ? "左" : "右",
                         current_target_half < 0 ? "左" : "右",
                         tracked_box.centerX(), image_center, pixel_error);
                return true;
            }

            if (std::fabs(pixel_error) <=
                lateral_center_tolerance_px_) {
                publishVelocity(0.0, 0.0, angular_z);
                // 横移阶段的任务是让文字框在画面中稳定居中。此前这里
                // 还要求map朝向误差同时小于2度；move_base到点后若残留
                // 几度误差，而小角速度又不足以克服底盘静摩擦，就会出现
                // NanoDet持续有框、像素已经居中，却一直等到15秒超时。
                // 朝向保持仍通过angular_z继续执行，但不再把它作为视觉
                // 居中的成功门槛。后续雷达逼近阶段仍会持续修正朝向。
                ++stable_frames;
                ROS_INFO_THROTTLE(
                    0.5,
                    "横移居中稳定中：目标x=%.1f，像素误差=%.1fpx，"
                    "朝向误差=%.2f度，稳定帧=%d/%d",
                    tracked_box.centerX(), pixel_error,
                    yaw_error * 180.0 / kPi,
                    stable_frames, lateral_stable_frames_);
                if (stable_frames >= lateral_stable_frames_) {
                    stopRobot();
                    ROS_INFO("横移居中完成：目标中心x=%.1f，"
                             "最终像素误差=%.1fpx，朝向误差=%.2f度",
                             tracked_box.centerX(), pixel_error,
                             yaw_error * 180.0 / kPi);
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
        ROS_INFO("仿真目标非当前墙段末保护（仅现实目标未完成时）：%s",
                 simulation_target_blocked_until_segment_end_
                     ? "锁定中" : "未锁定");

        ROS_INFO("unknown候选总数：%zu", unknown_candidates_.size());
        for (std::size_t i = 0;
             i < unknown_candidates_.size();
             ++i) {
            const UnknownCandidate& candidate =
                unknown_candidates_[i];
            ROS_INFO(
                "  候选[%zu]：%s(%.3f,%.3f)，来源段=%d，"
                "已回访=%s，已解决=%s",
                i + 1,
                wallName(candidate.board.wall),
                candidate.board.x,
                candidate.board.y,
                candidate.source_segment_index + 1,
                candidate.attempted ? "是" : "否",
                candidate.resolved ? "是" : "否");
        }

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
    ros::ServiceClient move_base_reconfigure_client_;
    ros::ServiceClient patrol_path_lock_client_;
    ros::ServiceClient controller_reset_client_;
    ros::Publisher patrol_path_publisher_;
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
    std::string planner_private_namespace_;
    double patrol_path_spacing_;
    double patrol_speed_limit_;
    double normal_navigation_speed_limit_;
    double patrol_cancel_timeout_;
    double patrol_interface_timeout_;
    std::string move_base_reconfigure_service_;
    bool disable_move_base_oscillation_during_patrol_;
    double normal_move_base_oscillation_timeout_;
    int patrol_aborted_retry_count_;

    double segment_end_tolerance_;
    double control_rate_;

    // V12角点过渡：短距离全向XY+yaw同时闭环。
    double patrol_transition_position_kp_;
    double patrol_transition_yaw_kp_;
    double patrol_transition_min_linear_speed_;
    double patrol_transition_max_linear_speed_;
    double patrol_transition_min_angular_speed_;
    double patrol_transition_max_angular_speed_;
    double patrol_transition_linear_accel_;
    double patrol_transition_angular_accel_;
    double patrol_transition_yaw_priority_start_deg_;
    double patrol_transition_yaw_priority_release_deg_;
    double patrol_transition_yaw_priority_min_linear_scale_;
    double patrol_transition_position_tolerance_;
    double patrol_transition_yaw_tolerance_deg_;
    int patrol_transition_stable_frames_;
    double patrol_transition_timeout_;

    int image_width_;
    double camera_fx_;
    double camera_yaw_offset_deg_;
    double docking_standoff_;
    double settle_time_;
    int ocr_attempts_;
    double ocr_retry_interval_;
    double ocr_recovery_turn_deg_;
    double ocr_recovery_turn_kp_;
    double ocr_recovery_turn_min_speed_;
    double ocr_recovery_turn_max_speed_;
    double ocr_recovery_turn_tolerance_deg_;
    int ocr_recovery_turn_stable_frames_;
    double ocr_recovery_turn_timeout_;
    double ocr_recovery_settle_time_;
    double max_detection_duration_;
    double patrol_stop_max_center_x_;

    // V14.5当前左墙目标接近减速参数。
    double patrol_target_slowdown_start_distance_;
    double patrol_target_stop_distance_;
    double patrol_target_min_speed_ratio_;

    // V14.7双停车保护参数。
    double patrol_noncurrent_wall_stop_max_distance_;
    int patrol_target_left_edge_stop_px_;

    double duplicate_coordinate_distance_;
    double max_track_jump_px_;
    int max_lost_frames_;
    double docking_recovery_turn_deg_;
    int docking_recovery_detection_attempts_;
    double docking_recovery_detection_interval_;
    double docking_refine_max_board_shift_;
    int docking_refresh_clear_calls_;
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
    // V14.4保护2：
    // 非当前巡检墙仿真目标只有在识别时real_docked_=false才锁到段末；
    // 若real_docked_=true，则不加锁，允许立即停靠。
    bool simulation_target_blocked_until_segment_end_ = false;
    int current_segment_index_ = 0;
    PatrolCheckpoint patrol_checkpoint_;
    bool patrol_goal_active_ = false;
    bool patrol_path_locked_ = false;
    bool move_base_oscillation_timeout_cached_ = false;
    bool move_base_patrol_oscillation_guard_active_ = false;
    bool shadow_mode_has_been_disabled_ = false;
    std::uint32_t patrol_path_sequence_ = 0;
    std::vector<BoardBoundaryEstimate> seen_board_coordinates_;

    // V14.9：巡检OCR=unknown时记录，整圈后任务未完成才依次回访。
    std::vector<UnknownCandidate> unknown_candidates_;

    // V14.5视觉接近减速状态。
    PatrolVisualApproach patrol_visual_approach_;
    double current_patrol_runtime_speed_limit_ =
        std::numeric_limits<double>::quiet_NaN();

    sensor_msgs::LaserScan latest_scan_;
    ros::WallTime latest_scan_wall_time_;
    bool have_laser_scan_ = false;
    bool lidar_layout_logged_ = false;
    double final_front_distance_ = -1.0;

};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "target_patrol_docking");
    TargetPatrolDocking node;
    return node.run() ? 0 : 1;
}