// 交付构建标识：IFLY2026_TARGET_SCAN_COMPETITION_R1_6_CANDIDATE_COOLDOWN_20260817
//
// 正式比赛旋转找板：
// 1) 三个固定扫描点 + 指定绝对map朝向序列；
// 2) 旋转过程NanoDet摄像头始终开启，发现“未屏蔽的新位置目标”立即停车；
// 3) 不再视觉对准，不再OCR后固定追加转角，停车后直接OCR；
// 4) 使用target_patrol_docking同款“相机射线→房间边界→同位置屏蔽”；
// 5) 现实目标：当前扫描点全部角度看完后再停靠；
// 6) 仿真目标：必须等现实目标完成停靠后才允许停靠；
// 7) 无法分类：进入候选位置列表，主流程全扫完仍缺目标时再逐个回访；
// 8) 停靠沿用V14.2两段式move_base + 同一目标视觉复定位 + ±30°恢复；
// 9) 固定角度旋转控制严格复用MyPlanner最终姿态调整的yaw控制形式：
//      P + yaw tolerance + min/max angular speed + acc_lim_theta限加速度。
// 10) R1.2：主扫描中两个目标只要在“同一次固定角度旋转/同一静止视野”
//      内凑齐，无论先识别现实还是先识别仿真，都立即结束当前扫描点。
//      仍保留旧优先级：如果仿真目标是更早轮次已经记录的，后来才找到现实，
//      则不触发该例外，继续完成当前扫描点剩余角度。
// 11) R1.3：同一视野出现多个目标时，先冻结“停车瞬间这一帧”的全部
//      历史未屏蔽框，再逐个OCR。处理第一个框后新增的seen/candidate状态
//      不允许反过来屏蔽同一冻结帧中的第二、第三个框。
//      后续新帧仍按原duplicate_coordinate_distance执行同位置屏蔽。
// 12) R1.4：候选点回访到位后，当前朝向若没有重新找到匹配目标，或虽然
//      找到但OCR仍无法分类，则依次扫描基准朝向+30°、基准朝向-30°。
//      旋转控制、缓存刷新和角度参数直接复用现有停靠恢复逻辑。
// 13) R1.6：unknown候选不进入永久禁止重复识别名单，但加入“视角冷却锁”。
//      主扫描只有当相对上次unknown观察角变化达到阈值，或机器人位置变化
//      达到阈值，才允许同一candidate再次停车OCR，防止在同一残缺画面反复停车。
// 14) R1.6：第一段临时停靠点复定位时，如果目标框中心位于图像左1/4或
//      右1/4，先向对应方向旋转docking_recovery_turn_deg（默认30°），
//      清NanoDet缓存并重新定位原板，然后再计算最终停靠点。

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros_nanodet/detect_result_srv.h>
#include <ros_nanodet/ocr_result_srv.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ucarmain2026/set_speed.h>

#include <algorithm>
#include <cmath>
#include <clocale>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

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

double applyMinimumMagnitude(double value, double minimum_magnitude) {
    if (std::fabs(value) < 1.0e-12) return 0.0;
    if (std::fabs(value) >= minimum_magnitude) return value;
    return std::copysign(minimum_magnitude, value);
}

}  // namespace


class TargetScanCompetition {
public:
    TargetScanCompetition()
        : nh_(),
          pnh_("~"),
          move_base_("move_base", true),
          tf_listener_(tf_buffer_) {
        // ------------------------------------------------------------------
        // 任务类别
        // ------------------------------------------------------------------
        pnh_.param(
            "real_target_category",
            real_target_category_,
            std::string("food"));
        pnh_.param(
            "simulation_target_category",
            simulation_target_category_,
            std::string("daily"));

        // ------------------------------------------------------------------
        // 坐标系 / 房间 / 导航
        // ------------------------------------------------------------------
        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("room_min_x", room_min_x_, 0.0);
        pnh_.param("room_max_x", room_max_x_, 5.0);
        pnh_.param("room_min_y", room_min_y_, 2.5);
        pnh_.param("room_max_y", room_max_y_, 4.5);
        pnh_.param("navigation_timeout", navigation_timeout_, 180.0);

        // ------------------------------------------------------------------
        // 当前单类NanoDet / 相机几何
        // ------------------------------------------------------------------
        pnh_.param("image_width", image_width_, 640);
        pnh_.param("camera_fx", camera_fx_, 554.256);
        pnh_.param("camera_yaw_offset_deg", camera_yaw_offset_deg_, 0.0);
        pnh_.param("max_detection_duration", max_detection_duration_, 0.50);
        pnh_.param(
            "duplicate_coordinate_distance",
            duplicate_coordinate_distance_,
            0.50);
        pnh_.param("static_view_detection_passes",
                   static_view_detection_passes_, 4);

        // ------------------------------------------------------------------
        // OCR
        // ------------------------------------------------------------------
        pnh_.param("ocr_attempts", ocr_attempts_, 3);
        pnh_.param("ocr_retry_interval", ocr_retry_interval_, 0.12);

        // ------------------------------------------------------------------
        // MyPlanner最终姿态控制参数。
        // 默认从 /move_base/MyPlanner 直接读取；若读取失败，使用当前
        // MyPlanner C3.2默认值。
        // ------------------------------------------------------------------
        pnh_.param(
            "planner_private_namespace",
            planner_private_namespace_,
            std::string("/move_base/MyPlanner"));
        pnh_.param("rotation_timeout", rotation_timeout_, 12.0);
        pnh_.param("rotation_control_rate", rotation_control_rate_, 20.0);

        loadMyPlannerFinalYawParameters();

        // ------------------------------------------------------------------
        // 两段式停靠（沿用target_patrol_docking当前稳定逻辑）
        // ------------------------------------------------------------------
        pnh_.param("docking_standoff", docking_standoff_, 0.50);
        pnh_.param("approach_stop_distance", approach_stop_distance_, 0.20);
        pnh_.param(
            "docking_refine_max_board_shift",
            docking_refine_max_board_shift_,
            0.80);
        pnh_.param(
            "docking_recovery_turn_deg",
            docking_recovery_turn_deg_,
            30.0);
        pnh_.param(
            "docking_recovery_detection_attempts",
            docking_recovery_detection_attempts_,
            3);
        pnh_.param(
            "docking_refresh_clear_calls",
            docking_refresh_clear_calls_,
            2);

        // ------------------------------------------------------------------
        // 候选点回访
        // ------------------------------------------------------------------
        pnh_.param(
            "candidate_match_max_shift",
            candidate_match_max_shift_,
            0.80);
        pnh_.param(
            "candidate_detection_attempts",
            candidate_detection_attempts_,
            4);

        // R1.6：候选主扫描重试冷却。
        // 只要观察角或机器人位置发生足够变化，就允许再次OCR。
        pnh_.param(
            "candidate_retry_yaw_delta_deg",
            candidate_retry_yaw_delta_deg_,
            15.0);
        pnh_.param(
            "candidate_retry_position_delta",
            candidate_retry_position_delta_,
            0.20);

        normalizeParameters();
        buildScanPlan();
        configuration_valid_ = validateConfiguration();

        detect_client_ =
            nh_.serviceClient<ros_nanodet::detect_result_srv>(
                "/nanodet_detect");
        ocr_client_ =
            nh_.serviceClient<ros_nanodet::ocr_result_srv>(
                "/nanodet_ocr");
        set_speed_client_ =
            nh_.serviceClient<ucarmain2026::set_speed>("/set_speed");

        ROS_WARN(
            "========== 正式比赛旋转找板 R1 ==========");
        ROS_INFO(
            "现实目标=%s；仿真目标=%s",
            categoryChinese(real_target_category_),
            categoryChinese(simulation_target_category_));
        ROS_INFO(
            "同位置已分类屏蔽阈值=%.2fm；候选回访匹配阈值=%.2fm；"
            "候选主扫描重试条件：yaw变化>=%.1f度 或 位置变化>=%.2fm",
            duplicate_coordinate_distance_,
            candidate_match_max_shift_,
            candidate_retry_yaw_delta_deg_,
            candidate_retry_position_delta_);
        ROS_INFO(
            "固定角度旋转复用MyPlanner最终姿态yaw控制："
            "Kp=%.3f，yaw_tol=%.3frad(%.2f度)，"
            "min_w=%.3f，max_w=%.3f，acc_lim_theta=%.3f",
            rotation_angular_gain_,
            rotation_yaw_tolerance_,
            rotation_yaw_tolerance_ * 180.0 / kPi,
            rotation_min_angular_speed_,
            rotation_max_angular_speed_,
            rotation_acc_lim_theta_);
        printScanPlan();
    }

    ~TargetScanCompetition() {
        stopRobot();
        closeCamera();
    }

    bool run() {
        if (!configuration_valid_) return false;
        if (!waitForDependencies()) return false;

        for (std::size_t point_index = 0;
             point_index < scan_points_.size() && ros::ok();
             ++point_index) {
            current_scan_point_index_ = static_cast<int>(point_index);
            const ScanPoint& point = scan_points_[point_index];

            // 任何move_base导航期间都关闭相机，避免旧帧累计。
            closeCamera();

            if (!navigateToPose(
                    point.x,
                    point.y,
                    point.initial_yaw_deg * kPi / 180.0,
                    point.name + "导航")) {
                printSummary(false);
                return false;
            }

            if (!openCamera()) {
                printSummary(false);
                return false;
            }

            ROS_WARN(
                "到达%s：(%.2f, %.2f, %.1f度)。"
                "本坐标扫描期间摄像头保持开启。",
                point.name.c_str(),
                point.x, point.y, point.initial_yaw_deg);

            // 先识别初始180度视野。
            const ProcessResult initial_result =
                inspectCurrentView(point.name + " 初始视野");
            if (initial_result == PROCESS_ABORT) {
                printSummary(false);
                return false;
            }
            if (initial_result == PROCESS_BOTH_TARGETS_READY) {
                if (!dockBothTargetsAfterEarlyScanStop(
                        point.name + "初始视野")) {
                    printSummary(false);
                    return false;
                }
                printSummary(true);
                return true;
            }
            if (initial_result == PROCESS_MISSION_COMPLETE) {
                printSummary(true);
                return true;
            }

            // 再按用户指定的绝对map yaw依次扫描。
            for (std::size_t yaw_index = 0;
                 yaw_index < point.scan_yaws_deg.size() && ros::ok();
                 ++yaw_index) {
                const double target_deg =
                    point.scan_yaws_deg[yaw_index];

                const ProcessResult rotate_result =
                    rotateToYawWhileScanning(
                        target_deg,
                        point.name);

                if (rotate_result == PROCESS_ABORT) {
                    printSummary(false);
                    return false;
                }
                if (rotate_result == PROCESS_BOTH_TARGETS_READY) {
                    if (!dockBothTargetsAfterEarlyScanStop(
                            point.name + "旋转扫描")) {
                        printSummary(false);
                        return false;
                    }
                    printSummary(true);
                    return true;
                }
                if (rotate_result == PROCESS_MISSION_COMPLETE) {
                    printSummary(true);
                    return true;
                }

                // 到达固定角度后再补一轮静止视野检查，避免目标刚好只在
                // 最终角度进入画面而没有被旋转中那一帧捕获。
                const ProcessResult static_result =
                    inspectCurrentView(
                        point.name + " 固定角度 "
                        + degreeLabel(target_deg));
                if (static_result == PROCESS_ABORT) {
                    printSummary(false);
                    return false;
                }
                if (static_result == PROCESS_BOTH_TARGETS_READY) {
                    if (!dockBothTargetsAfterEarlyScanStop(
                            point.name + "固定角度静止视野")) {
                        printSummary(false);
                        return false;
                    }
                    printSummary(true);
                    return true;
                }
                if (static_result == PROCESS_MISSION_COMPLETE) {
                    printSummary(true);
                    return true;
                }
            }

            stopRobot();
            ROS_WARN(
                "%s全部指定角度已经看完。",
                point.name.c_str());

            // 用户规则：现实目标即便在本点早早发现，也必须等本坐标所有
            // 角度完整看完以后才进行停靠。
            if (real_observation_.valid && !real_docked_) {
                ROS_WARN(
                    "现实目标已在本次/此前扫描中确认；"
                    "%s全部角度已完成，现在才执行现实目标停靠。",
                    point.name.c_str());

                if (!dockStoredTarget(
                        real_observation_,
                        "现实目标")) {
                    printSummary(false);
                    return false;
                }
                real_docked_ = true;

                // 仿真目标只能排在现实目标后面。
                if (simulation_observation_.valid &&
                    !simulation_docked_) {
                    ROS_WARN(
                        "现实目标已经完成停靠，"
                        "立即处理此前记录的仿真目标。");
                    if (!dockStoredTarget(
                            simulation_observation_,
                            "仿真目标")) {
                        printSummary(false);
                        return false;
                    }
                    simulation_docked_ = true;
                }

                if (missionComplete()) {
                    printSummary(true);
                    return true;
                }
            }

            // 当前点结束、还需下一扫描点时释放摄像头。
            closeCamera();
        }

        // 三个正式扫描点全部执行完。
        stopRobot();
        closeCamera();

        if (missionComplete()) {
            printSummary(true);
            return true;
        }

        ROS_WARN(
            "三个固定扫描点全部完成，但两个目标尚未全部停靠；"
            "开始回访无法分类的候选位置。候选数=%zu",
            candidate_locations_.size());

        if (!revisitCandidates()) {
            printSummary(false);
            return false;
        }

        const bool success = missionComplete();
        if (!success) {
            ROS_ERROR(
                "固定扫描 + 候选位置回访全部完成，"
                "仍未完成现实目标和仿真目标。");
        }
        printSummary(success);
        return success;
    }

private:
    // ======================================================================
    // 数据结构
    // ======================================================================
    struct Pose2D {
        double x;
        double y;
        double yaw;

        Pose2D() : x(0.0), y(0.0), yaw(0.0) {}
        Pose2D(double px, double py, double pyaw)
            : x(px), y(py), yaw(pyaw) {}
    };

    enum WallType {
        WALL_LEFT = 0,
        WALL_RIGHT = 1,
        WALL_BOTTOM = 2,
        WALL_TOP = 3
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
    };

    struct OcrRecord {
        bool success;
        std::string text;
        std::string category;
        double confidence;
        double detect_score;
        Box box;

        OcrRecord()
            : success(false),
              text("<未识别>"),
              category("unknown"),
              confidence(0.0),
              detect_score(0.0),
              box{0, 0, 0, 0, 0} {}
    };

    struct BoardBoundaryEstimate {
        bool valid;
        WallType wall;
        double x;
        double y;

        BoardBoundaryEstimate()
            : valid(false),
              wall(WALL_LEFT),
              x(0.0),
              y(0.0) {}
    };

    struct ViewTargetSnapshot {
        Box box;
        BoardBoundaryEstimate estimate;

        ViewTargetSnapshot()
            : box{0, 0, 0, 0, 0} {}
    };

    struct TargetObservation {
        bool valid;
        std::string category;
        BoardBoundaryEstimate board;
        Pose2D pose;

        TargetObservation()
            : valid(false), category("unknown") {}
    };

    struct CandidateLocation {
        bool board_valid;
        BoardBoundaryEstimate board;
        Pose2D detection_pose;
        Box trigger_box;
        int scan_point_index;
        double detected_yaw;
        bool attempted;
        bool resolved;

        // R1.6：最近一次主扫描OCR仍为unknown时的观察状态。
        // candidate不会永久屏蔽，但在视角/位置明显变化前临时锁住。
        bool have_failed_view;
        Pose2D last_failed_pose;
        double last_failed_yaw;

        CandidateLocation()
            : board_valid(false),
              scan_point_index(-1),
              detected_yaw(0.0),
              attempted(false),
              resolved(false),
              have_failed_view(false),
              last_failed_yaw(0.0),
              trigger_box{0, 0, 0, 0, 0} {}
    };

    struct ScanPoint {
        std::string name;
        double x;
        double y;
        double initial_yaw_deg;
        std::vector<double> scan_yaws_deg;
    };

    enum ProcessResult {
        PROCESS_CONTINUE,
        // 主扫描阶段：现实目标已经记录，本次又确认仿真目标。
        // 两个目标位置已经齐全，不再浪费时间扫描当前点剩余角度。
        PROCESS_BOTH_TARGETS_READY,
        PROCESS_MISSION_COMPLETE,
        PROCESS_ABORT
    };

    enum CandidateViewResult {
        CANDIDATE_VIEW_NOT_FOUND,
        CANDIDATE_VIEW_STILL_UNKNOWN,
        CANDIDATE_VIEW_RESOLVED,
        CANDIDATE_VIEW_MISSION_COMPLETE,
        CANDIDATE_VIEW_ABORT
    };

    // ======================================================================
    // 基础工具
    // ======================================================================
    static bool isValidCategory(const std::string& category) {
        return category == "food" ||
               category == "daily" ||
               category == "electronic";
    }

    static std::string classifyText(const std::string& text) {
        // 与当前target_patrol_docking一致：允许只识别到类别名称中的
        // 单个关键字，提高旋转刚停车时的识别容错。
        if (text.find("食") != std::string::npos) {
            return "food";
        }
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

    static const char* categoryChinese(
            const std::string& category) {
        if (category == "food") return "食品加工车间";
        if (category == "daily") return "日用品加工车间";
        if (category == "electronic") return "电子产品生产车间";
        return "未知";
    }

    static const char* wallName(WallType wall) {
        switch (wall) {
            case WALL_LEFT: return "左墙";
            case WALL_RIGHT: return "右墙";
            case WALL_BOTTOM: return "下墙";
            case WALL_TOP: return "上墙";
        }
        return "未知墙";
    }

    static std::string degreeLabel(double degree) {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "%.1f度", degree);
        return std::string(buffer);
    }

    bool missionComplete() const {
        return real_docked_ && simulation_docked_;
    }

    bool dockBothTargetsAfterEarlyScanStop(
            const std::string& trigger_source) {
        stopRobot();

        if (!real_observation_.valid ||
            !simulation_observation_.valid ||
            real_docked_) {
            ROS_ERROR(
                "提前结束扫描状态异常：来源=%s，"
                "现实记录=%s，仿真记录=%s，现实已停靠=%s",
                trigger_source.c_str(),
                real_observation_.valid ? "是" : "否",
                simulation_observation_.valid ? "是" : "否",
                real_docked_ ? "是" : "否");
            return false;
        }

        ROS_WARN(
            "双目标提前完成条件成立：现实目标已经找到，"
            "本次又确认仿真目标；立即结束%s剩余固定角度扫描。",
            trigger_source.c_str());

        ROS_WARN(
            "按照既定优先级先执行现实目标停靠。");
        if (!dockStoredTarget(
                real_observation_,
                "现实目标")) {
            return false;
        }
        real_docked_ = true;

        ROS_WARN(
            "现实目标停靠完成，立即执行已记录仿真目标停靠。");
        if (!dockStoredTarget(
                simulation_observation_,
                "仿真目标")) {
            return false;
        }
        simulation_docked_ = true;

        return missionComplete();
    }

    bool bothTargetsReadyForSameScanPhase(
            bool simulation_known_before_phase) const {
        // R1.2的核心判定：
        //
        // 1. 两个目标现在都已经有有效记录；
        // 2. 现实目标还没有停靠（否则原仿真分支本来就会立即停靠）；
        // 3. 仿真目标不是进入当前扫描阶段之前就已经存在的旧记录。
        //
        // 因此：
        //   - 现实旧记录 + 本阶段新仿真：提前结束；
        //   - 本阶段先现实后仿真：提前结束；
        //   - 本阶段先仿真后现实：提前结束；
        //   - 更早阶段已仿真 + 本阶段才现实：保持旧规则，不提前。
        return !simulation_known_before_phase &&
               real_observation_.valid &&
               simulation_observation_.valid &&
               !real_docked_;
    }

    std::string plannerParameter(
            const std::string& name) const {
        std::string ns = planner_private_namespace_;
        if (ns.empty()) ns = "/move_base/MyPlanner";
        if (ns[0] != '/') ns = "/" + ns;
        while (ns.size() > 1 && ns.back() == '/') {
            ns.pop_back();
        }
        return ns + "/" + name;
    }

    void loadMyPlannerFinalYawParameters() {
        // 当前MyPlanner C3.2默认值。
        rotation_angular_gain_ = 1.50;
        rotation_yaw_tolerance_ = 0.05;
        rotation_min_angular_speed_ = 0.08;
        rotation_max_angular_speed_ = 0.90;
        rotation_acc_lim_theta_ = 8.00;

        double value = 0.0;
        if (ros::param::get(
                plannerParameter("final_angular_gain"),
                value)) {
            rotation_angular_gain_ = value;
        }
        if (ros::param::get(
                plannerParameter("goal_yaw_tolerance"),
                value)) {
            rotation_yaw_tolerance_ = value;
        }
        if (ros::param::get(
                plannerParameter("final_min_angular_speed"),
                value)) {
            rotation_min_angular_speed_ = value;
        }
        if (ros::param::get(
                plannerParameter("final_max_vel_theta"),
                value)) {
            rotation_max_angular_speed_ = value;
        }
        if (ros::param::get(
                plannerParameter("acc_lim_theta"),
                value)) {
            rotation_acc_lim_theta_ = value;
        }

        // 仍允许比赛现场通过本节点私有参数显式覆盖。
        pnh_.param(
            "rotation_angular_gain",
            rotation_angular_gain_,
            rotation_angular_gain_);
        pnh_.param(
            "rotation_yaw_tolerance",
            rotation_yaw_tolerance_,
            rotation_yaw_tolerance_);
        pnh_.param(
            "rotation_min_angular_speed",
            rotation_min_angular_speed_,
            rotation_min_angular_speed_);
        pnh_.param(
            "rotation_max_angular_speed",
            rotation_max_angular_speed_,
            rotation_max_angular_speed_);
        pnh_.param(
            "rotation_acc_lim_theta",
            rotation_acc_lim_theta_,
            rotation_acc_lim_theta_);
    }

    void normalizeParameters() {
        camera_fx_ = std::fabs(camera_fx_);
        max_detection_duration_ =
            std::max(0.05, max_detection_duration_);
        duplicate_coordinate_distance_ =
            std::max(0.05, std::fabs(
                duplicate_coordinate_distance_));
        static_view_detection_passes_ =
            std::max(1, static_view_detection_passes_);
        ocr_attempts_ = std::max(1, ocr_attempts_);
        ocr_retry_interval_ =
            std::max(0.0, ocr_retry_interval_);

        rotation_angular_gain_ =
            std::fabs(rotation_angular_gain_);
        rotation_yaw_tolerance_ =
            std::max(0.002, std::fabs(
                rotation_yaw_tolerance_));
        rotation_min_angular_speed_ =
            std::max(0.0, std::fabs(
                rotation_min_angular_speed_));
        rotation_max_angular_speed_ =
            std::max(
                rotation_min_angular_speed_,
                std::fabs(rotation_max_angular_speed_));
        rotation_acc_lim_theta_ =
            std::max(0.1, std::fabs(
                rotation_acc_lim_theta_));
        rotation_timeout_ =
            std::max(2.0, rotation_timeout_);
        rotation_control_rate_ =
            std::max(5.0, rotation_control_rate_);

        docking_standoff_ =
            std::max(0.05, std::fabs(docking_standoff_));
        approach_stop_distance_ =
            std::max(0.05, std::fabs(
                approach_stop_distance_));
        docking_refine_max_board_shift_ =
            std::max(
                0.10,
                std::fabs(docking_refine_max_board_shift_));
        docking_recovery_turn_deg_ =
            clampValue(
                std::fabs(docking_recovery_turn_deg_),
                1.0, 90.0);
        docking_recovery_detection_attempts_ =
            std::max(
                1,
                docking_recovery_detection_attempts_);
        docking_refresh_clear_calls_ =
            std::max(1, docking_refresh_clear_calls_);

        candidate_match_max_shift_ =
            std::max(
                0.10,
                std::fabs(candidate_match_max_shift_));
        candidate_detection_attempts_ =
            std::max(1, candidate_detection_attempts_);
        candidate_retry_yaw_delta_deg_ =
            clampValue(
                std::fabs(candidate_retry_yaw_delta_deg_),
                1.0, 180.0);
        candidate_retry_position_delta_ =
            std::max(
                0.01,
                std::fabs(candidate_retry_position_delta_));
    }

    bool validateConfiguration() const {
        if (!isValidCategory(real_target_category_) ||
            !isValidCategory(simulation_target_category_)) {
            ROS_ERROR(
                "real_target_category和simulation_target_category"
                "只允许food、daily、electronic");
            return false;
        }
        if (real_target_category_ ==
            simulation_target_category_) {
            ROS_ERROR("现实目标和仿真目标不能是同一类别");
            return false;
        }
        if (room_max_x_ <= room_min_x_ ||
            room_max_y_ <= room_min_y_) {
            ROS_ERROR("房间边界配置无效");
            return false;
        }
        if (camera_fx_ <= 0.0) {
            ROS_ERROR("camera_fx必须大于0");
            return false;
        }
        return true;
    }

    void buildScanPlan() {
        scan_points_.clear();

        ScanPoint point1;
        point1.name = "扫描点1";
        point1.x = 1.25;
        point1.y = 4.25;
        point1.initial_yaw_deg = 180.0;
        point1.scan_yaws_deg = {
            240.0, -90.0, 0.0
        };
        scan_points_.push_back(point1);

        ScanPoint point2;
        point2.name = "扫描点2";
        point2.x = 2.50;
        point2.y = 2.75;
        point2.initial_yaw_deg = 180.0;
        point2.scan_yaws_deg = {
            120.0, 90.0, 60.0, 0.0
        };
        scan_points_.push_back(point2);

        ScanPoint point3;
        point3.name = "扫描点3";
        point3.x = 3.75;
        point3.y = 3.25;
        point3.initial_yaw_deg = 180.0;
        point3.scan_yaws_deg = {
            -90.0, -30.0, 0.0,
            60.0, 90.0, 120.0
        };
        scan_points_.push_back(point3);
    }

    void printScanPlan() const {
        ROS_INFO("固定扫描计划：");
        for (std::size_t i = 0;
             i < scan_points_.size(); ++i) {
            const ScanPoint& point = scan_points_[i];
            std::string yaw_text;
            for (std::size_t j = 0;
                 j < point.scan_yaws_deg.size(); ++j) {
                if (!yaw_text.empty()) yaw_text += " -> ";
                yaw_text += degreeLabel(
                    point.scan_yaws_deg[j]);
            }
            ROS_INFO(
                "  %s：(%.2f, %.2f)，初始=%.1f度，随后%s",
                point.name.c_str(),
                point.x, point.y,
                point.initial_yaw_deg,
                yaw_text.c_str());
        }
    }

    bool waitForDependencies() {
        ROS_INFO(
            "等待move_base、NanoDet、OCR和/set_speed...");
        while (ros::ok() &&
               !move_base_.waitForServer(
                   ros::Duration(3.0))) {
            ROS_INFO("仍在等待move_base");
        }
        if (!ros::ok()) return false;

        if (!ros::service::waitForService(
                "/nanodet_detect",
                ros::Duration(20.0))) {
            ROS_ERROR("等待/nanodet_detect超时");
            return false;
        }
        if (!ros::service::waitForService(
                "/nanodet_ocr",
                ros::Duration(20.0))) {
            ROS_ERROR("等待/nanodet_ocr超时");
            return false;
        }
        if (!ros::service::waitForService(
                "/set_speed",
                ros::Duration(20.0))) {
            ROS_ERROR("等待/set_speed超时");
            return false;
        }
        return true;
    }

    bool getRobotPose(Pose2D& pose) {
        try {
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    map_frame_,
                    base_frame_,
                    ros::Time(0),
                    ros::Duration(0.20));
            pose.x =
                transform.transform.translation.x;
            pose.y =
                transform.transform.translation.y;
            pose.yaw =
                tf2::getYaw(
                    transform.transform.rotation);
            return true;
        } catch (
            const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(
                1.0,
                "读取机器人map位姿失败：%s",
                error.what());
            return false;
        }
    }

    bool isInsideRoom(double x, double y) const {
        const double eps = 1.0e-6;
        return x >= room_min_x_ - eps &&
               x <= room_max_x_ + eps &&
               y >= room_min_y_ - eps &&
               y <= room_max_y_ + eps;
    }

    void clampToRoom(double& x, double& y) const {
        x = clampValue(x, room_min_x_, room_max_x_);
        y = clampValue(y, room_min_y_, room_max_y_);
    }

    // ======================================================================
    // 相机 / NanoDet / OCR
    // ======================================================================
    bool openCamera() {
        if (camera_opened_) return true;

        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -1;
        if (!detect_client_.call(service)) {
            ROS_ERROR("打开NanoDet摄像头失败");
            return false;
        }

        camera_opened_ = true;
        ROS_INFO("NanoDet摄像头已打开");
        return true;
    }

    void closeCamera() {
        if (!camera_opened_) return;

        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -2;
        if (!detect_client_.call(service)) {
            ROS_WARN("释放NanoDet摄像头失败");
        }
        camera_opened_ = false;
    }

    bool clearDetectionBuffer(
            const std::string& reason) {
        if (!openCamera()) return false;

        for (int i = 0;
             i < docking_refresh_clear_calls_;
             ++i) {
            ros_nanodet::detect_result_srv service;
            service.request.detect_start = -3;
            if (!detect_client_.call(service)) {
                ROS_WARN(
                    "%s：第%d/%d次清NanoDet缓存失败",
                    reason.c_str(),
                    i + 1,
                    docking_refresh_clear_calls_);
                return false;
            }
        }
        return true;
    }

    bool detectBoxes(std::vector<Box>& boxes) {
        boxes.clear();

        ros_nanodet::detect_result_srv service;
        service.request.detect_start = 1;

        const ros::WallTime begin =
            ros::WallTime::now();

        if (!detect_client_.call(service)) {
            ROS_WARN_THROTTLE(
                1.0,
                "调用/nanodet_detect失败");
            return false;
        }

        const double elapsed =
            (ros::WallTime::now() - begin).toSec();

        if (elapsed > max_detection_duration_) {
            ROS_ERROR(
                "NanoDet单帧耗时%.3fs超过%.3fs，"
                "本次结果按过期帧丢弃",
                elapsed,
                max_detection_duration_);
            return false;
        }

        const std::size_t count = std::min(
            std::min(
                service.response.x0.size(),
                service.response.y0.size()),
            std::min(
                service.response.x1.size(),
                service.response.y1.size()));

        for (std::size_t i = 0;
             i < count; ++i) {
            Box box;
            box.class_id =
                i < service.response.class_name.size()
                    ? service.response.class_name[i]
                    : 0;
            box.x0 = service.response.x0[i];
            box.y0 = service.response.y0[i];
            box.x1 = service.response.x1[i];
            box.y1 = service.response.y1[i];

            if (box.x1 > box.x0 &&
                box.y1 > box.y0) {
                boxes.push_back(box);
            }
        }
        return true;
    }

    OcrRecord recognizeStaticTarget(
            const Box& trigger_box) {
        OcrRecord best_any;
        OcrRecord best_keyword;
        bool have_any = false;
        bool have_keyword = false;
        Box reference = trigger_box;

        // 停车后直接OCR；先丢掉旋转过程中积压的两帧，保证OCR使用
        // 停车后的新画面。没有任何视觉对准，也没有额外固定转角。
        ros_nanodet::ocr_result_srv clear_service;
        clear_service.request.command = -3;
        if (!ocr_client_.call(clear_service)) {
            ROS_WARN("OCR缓冲清理失败，继续识别");
        }

        for (int attempt = 0;
             attempt < ocr_attempts_ && ros::ok();
             ++attempt) {
            ros_nanodet::ocr_result_srv service;
            service.request.command = 1;

            if (!ocr_client_.call(service)) {
                ROS_WARN(
                    "OCR第%d/%d次服务调用失败",
                    attempt + 1,
                    ocr_attempts_);
                if (ocr_retry_interval_ > 0.0) {
                    ros::Duration(
                        ocr_retry_interval_).sleep();
                }
                continue;
            }

            const std::size_t count = std::min(
                std::min(
                    service.response.text.size(),
                    service.response.confidence.size()),
                std::min(
                    std::min(
                        service.response.x0.size(),
                        service.response.y0.size()),
                    std::min(
                        service.response.x1.size(),
                        service.response.y1.size())));

            int selected = -1;
            double nearest =
                std::numeric_limits<double>::infinity();

            for (std::size_t i = 0;
                 i < count; ++i) {
                const double cx =
                    0.5 * static_cast<double>(
                        service.response.x0[i] +
                        service.response.x1[i]);
                const double cy =
                    0.5 * static_cast<double>(
                        service.response.y0[i] +
                        service.response.y1[i]);

                const double d =
                    std::hypot(
                        cx - reference.centerX(),
                        cy - reference.centerY());

                if (d < nearest) {
                    nearest = d;
                    selected = static_cast<int>(i);
                }
            }

            if (selected >= 0) {
                const std::size_t i =
                    static_cast<std::size_t>(selected);

                OcrRecord candidate;
                candidate.success =
                    service.response.success;
                candidate.text =
                    service.response.text[i];
                candidate.category =
                    classifyText(candidate.text);
                candidate.confidence =
                    service.response.confidence[i];
                candidate.detect_score =
                    i < service.response.detect_score.size()
                        ? service.response.detect_score[i]
                        : 0.0;
                candidate.box = Box{
                    0,
                    service.response.x0[i],
                    service.response.y0[i],
                    service.response.x1[i],
                    service.response.y1[i]};

                reference = candidate.box;

                ROS_INFO(
                    "OCR第%d/%d次：文字='%s'，"
                    "类别=%s，置信度=%.3f，框中心=(%.1f,%.1f)",
                    attempt + 1,
                    ocr_attempts_,
                    candidate.text.c_str(),
                    categoryChinese(candidate.category),
                    candidate.confidence,
                    candidate.box.centerX(),
                    candidate.box.centerY());

                if (!candidate.text.empty() &&
                    (!have_any ||
                     candidate.confidence >
                         best_any.confidence)) {
                    best_any = candidate;
                    have_any = true;
                }

                if (candidate.category != "unknown" &&
                    (!have_keyword ||
                     candidate.confidence >
                         best_keyword.confidence)) {
                    best_keyword = candidate;
                    have_keyword = true;
                }
            } else {
                ROS_WARN(
                    "OCR第%d/%d次没有返回文字框",
                    attempt + 1,
                    ocr_attempts_);
            }

            if (ocr_retry_interval_ > 0.0) {
                ros::Duration(
                    ocr_retry_interval_).sleep();
            }
        }

        OcrRecord result;
        if (have_keyword) {
            result = best_keyword;
        } else if (have_any) {
            result = best_any;
        } else {
            result.box = trigger_box;
        }

        ROS_WARN(
            "OCR最终结果：'%s' -> %s",
            result.text.c_str(),
            categoryChinese(result.category));
        return result;
    }

    // ======================================================================
    // 目标墙面坐标估计 + 同位置屏蔽
    // ======================================================================
    bool estimateBoardBoundary(
            const Pose2D& robot_pose,
            const Box& box,
            BoardBoundaryEstimate& estimate) const {
        estimate = BoardBoundaryEstimate();

        const double image_center =
            0.5 * static_cast<double>(image_width_);

        // 与当前target_patrol_docking一致：
        // 图像向右 -> 机器人相对航向为负。
        const double relative_yaw =
            std::atan2(
                image_center - box.centerX(),
                camera_fx_);

        const double ray_yaw =
            robot_pose.yaw +
            camera_yaw_offset_deg_ *
                kPi / 180.0 +
            relative_yaw;

        const double ray_x =
            std::cos(ray_yaw);
        const double ray_y =
            std::sin(ray_yaw);

        const double epsilon = 1.0e-6;
        double nearest_t =
            std::numeric_limits<double>::infinity();
        BoardBoundaryEstimate nearest;

        const auto consider =
            [&](WallType wall,
                double t,
                double x,
                double y) {
                if (!std::isfinite(t) ||
                    t <= epsilon ||
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
                nearest.x =
                    clampValue(
                        x,
                        room_min_x_,
                        room_max_x_);
                nearest.y =
                    clampValue(
                        y,
                        room_min_y_,
                        room_max_y_);
            };

        if (std::fabs(ray_x) >= epsilon) {
            double t =
                (room_min_x_ - robot_pose.x) /
                ray_x;
            consider(
                WALL_LEFT,
                t,
                room_min_x_,
                robot_pose.y + t * ray_y);

            t =
                (room_max_x_ - robot_pose.x) /
                ray_x;
            consider(
                WALL_RIGHT,
                t,
                room_max_x_,
                robot_pose.y + t * ray_y);
        }

        if (std::fabs(ray_y) >= epsilon) {
            double t =
                (room_min_y_ - robot_pose.y) /
                ray_y;
            consider(
                WALL_BOTTOM,
                t,
                robot_pose.x + t * ray_x,
                room_min_y_);

            t =
                (room_max_y_ - robot_pose.y) /
                ray_y;
            consider(
                WALL_TOP,
                t,
                robot_pose.x + t * ray_x,
                room_max_y_);
        }

        if (!nearest.valid) return false;
        estimate = nearest;
        return true;
    }

    bool sameBoardPosition(
            const BoardBoundaryEstimate& a,
            const BoardBoundaryEstimate& b,
            double threshold) const {
        if (!a.valid || !b.valid ||
            a.wall != b.wall) {
            return false;
        }
        return distance2D(
                   a.x, a.y,
                   b.x, b.y) <= threshold;
    }

    bool isSeenBoard(
            const BoardBoundaryEstimate& estimate) const {
        if (!estimate.valid) return false;

        for (std::size_t i = 0;
             i < seen_board_coordinates_.size();
             ++i) {
            if (sameBoardPosition(
                    estimate,
                    seen_board_coordinates_[i],
                    duplicate_coordinate_distance_)) {
                return true;
            }
        }
        return false;
    }

    int findMatchingCandidateIndex(
            const BoardBoundaryEstimate& board,
            const Pose2D& current_pose) const {
        if (board.valid) {
            for (std::size_t i = 0;
                 i < candidate_locations_.size();
                 ++i) {
                const CandidateLocation& candidate =
                    candidate_locations_[i];

                if (candidate.resolved ||
                    !candidate.board_valid) {
                    continue;
                }

                if (sameBoardPosition(
                        board,
                        candidate.board,
                        duplicate_coordinate_distance_)) {
                    return static_cast<int>(i);
                }
            }
            return -1;
        }

        // 墙面交点失败时，用当时发现候选的机器人位姿做兜底关联。
        for (std::size_t i = 0;
             i < candidate_locations_.size();
             ++i) {
            const CandidateLocation& candidate =
                candidate_locations_[i];

            if (candidate.resolved ||
                candidate.board_valid) {
                continue;
            }

            const double pos_error =
                distance2D(
                    current_pose.x,
                    current_pose.y,
                    candidate.detection_pose.x,
                    candidate.detection_pose.y);

            const double yaw_error =
                std::fabs(normalizeAngle(
                    current_pose.yaw -
                    candidate.detected_yaw));

            if (pos_error <= 0.20 &&
                yaw_error <= 20.0 * kPi / 180.0) {
                return static_cast<int>(i);
            }
        }

        return -1;
    }

    bool isCandidateBoard(
            const BoardBoundaryEstimate& estimate) const {
        if (!estimate.valid) return false;

        Pose2D dummy_pose;
        return findMatchingCandidateIndex(
                   estimate,
                   dummy_pose) >= 0;
    }

    bool candidateRetryAllowed(
            const CandidateLocation& candidate,
            const Pose2D& current_pose,
            double* yaw_delta_deg_out = nullptr,
            double* position_delta_out = nullptr) const {
        if (candidate.resolved ||
            !candidate.have_failed_view) {
            if (yaw_delta_deg_out) {
                *yaw_delta_deg_out =
                    std::numeric_limits<double>::infinity();
            }
            if (position_delta_out) {
                *position_delta_out =
                    std::numeric_limits<double>::infinity();
            }
            return true;
        }

        const double yaw_delta =
            std::fabs(normalizeAngle(
                current_pose.yaw -
                candidate.last_failed_yaw));
        const double yaw_delta_deg =
            yaw_delta * 180.0 / kPi;

        const double position_delta =
            distance2D(
                current_pose.x,
                current_pose.y,
                candidate.last_failed_pose.x,
                candidate.last_failed_pose.y);

        if (yaw_delta_deg_out) {
            *yaw_delta_deg_out = yaw_delta_deg;
        }
        if (position_delta_out) {
            *position_delta_out = position_delta;
        }

        // 满足任意一个条件即可重新尝试：
        // 1) 视角已经明显变化；
        // 2) 小车位置已经明显变化。
        return yaw_delta_deg >=
                   candidate_retry_yaw_delta_deg_ ||
               position_delta >=
                   candidate_retry_position_delta_;
    }

    bool isCandidateRetryLocked(
            const BoardBoundaryEstimate& estimate,
            const Pose2D& current_pose) const {
        const int candidate_index =
            findMatchingCandidateIndex(
                estimate,
                current_pose);

        if (candidate_index < 0) {
            return false;
        }

        const CandidateLocation& candidate =
            candidate_locations_[
                static_cast<std::size_t>(
                    candidate_index)];

        double yaw_delta_deg = 0.0;
        double position_delta = 0.0;

        if (candidateRetryAllowed(
                candidate,
                current_pose,
                &yaw_delta_deg,
                &position_delta)) {
            ROS_INFO_THROTTLE(
                1.0,
                "R1.6候选解锁：同一unknown目标已获得新视角，"
                "yaw变化=%.1f度(阈值%.1f)，位置变化=%.3fm(阈值%.3f)，"
                "本次允许重新停车OCR。",
                yaw_delta_deg,
                candidate_retry_yaw_delta_deg_,
                position_delta,
                candidate_retry_position_delta_);
            return false;
        }

        ROS_INFO_THROTTLE(
            0.8,
            "R1.6候选临时锁：该unknown目标刚在相近视角识别失败，"
            "yaw仅变化=%.1f度<%.1f，位置仅变化=%.3fm<%.3f；"
            "当前帧不停车，让小车继续旋转。",
            yaw_delta_deg,
            candidate_retry_yaw_delta_deg_,
            position_delta,
            candidate_retry_position_delta_);
        return true;
    }

    bool isSuppressedBoard(
            const BoardBoundaryEstimate& estimate,
            const Pose2D& current_pose) const {
        // 永久屏蔽：只有已经成功分类的位置。
        if (isSeenBoard(estimate)) {
            return true;
        }

        // candidate不进入永久屏蔽，只在“观察角还没明显变化”的短时间内
        // 做临时屏蔽，避免同一个残缺画面每一帧都停车OCR。
        return isCandidateRetryLocked(
            estimate,
            current_pose);
    }

    void registerSeenBoard(
            const BoardBoundaryEstimate& estimate,
            const std::string& reason) {
        if (!estimate.valid) return;
        if (isSeenBoard(estimate)) return;

        seen_board_coordinates_.push_back(estimate);
        ROS_INFO(
            "加入同位置屏蔽名单：%s(%.3f, %.3f)，原因=%s",
            wallName(estimate.wall),
            estimate.x,
            estimate.y,
            reason.c_str());
    }

    bool candidateAlreadyExists(
            const BoardBoundaryEstimate& board,
            const Pose2D& detection_pose) const {
        return findMatchingCandidateIndex(
                   board,
                   detection_pose) >= 0;
    }

    int findMutableMatchingCandidateIndex(
            const BoardBoundaryEstimate& board,
            const Pose2D& detection_pose) {
        return findMatchingCandidateIndex(
            board,
            detection_pose);
    }

    void updateCandidateFailedView(
            int candidate_index,
            const Pose2D& failed_pose,
            const Box& failed_box) {
        if (candidate_index < 0 ||
            candidate_index >=
                static_cast<int>(
                    candidate_locations_.size())) {
            return;
        }

        CandidateLocation& candidate =
            candidate_locations_[
                static_cast<std::size_t>(
                    candidate_index)];

        candidate.have_failed_view = true;
        candidate.last_failed_pose = failed_pose;
        candidate.last_failed_yaw = failed_pose.yaw;
        candidate.trigger_box = failed_box;

        ROS_WARN(
            "R1.6更新候选失败视角：candidate[%d]，"
            "pose=(%.3f,%.3f,%.1f度)。"
            "主扫描需yaw变化>=%.1f度或位置变化>=%.2fm才允许再次OCR。",
            candidate_index + 1,
            failed_pose.x,
            failed_pose.y,
            failed_pose.yaw * 180.0 / kPi,
            candidate_retry_yaw_delta_deg_,
            candidate_retry_position_delta_);
    }

    void addCandidateLocation(
            const BoardBoundaryEstimate& board,
            const Pose2D& stopped_pose,
            const Box& box,
            bool frozen_view_member = false) {
        if (!frozen_view_member &&
            candidateAlreadyExists(
                board,
                stopped_pose)) {
            ROS_INFO(
                "无法分类目标已在候选列表中，"
                "不重复加入。");
            return;
        }

        if (frozen_view_member) {
            // frozen_view_member只可能来自当前停车帧提前冻结的独立bbox。
            // 历史候选已经在freezeVisibleNewTargets()之前过滤，因此这里
            // 不再用0.50m墙面距离把同帧相邻的两个unknown候选合并。
            ROS_INFO(
                "R1.3同帧候选：该框已在停车瞬间独立冻结，"
                "允许作为独立候选保存。");
        }

        CandidateLocation candidate;
        candidate.board_valid = board.valid;
        candidate.board = board;
        candidate.detection_pose = stopped_pose;
        candidate.trigger_box = box;
        candidate.scan_point_index =
            current_scan_point_index_;
        candidate.detected_yaw =
            stopped_pose.yaw;
        candidate.have_failed_view = true;
        candidate.last_failed_pose = stopped_pose;
        candidate.last_failed_yaw = stopped_pose.yaw;

        candidate_locations_.push_back(candidate);

        if (board.valid) {
            ROS_WARN(
                "新增无法分类候选位置[%zu]："
                "%s(%.3f, %.3f)，检测位姿=(%.3f, %.3f, %.1f度)，"
                "框=(%d,%d)-(%d,%d)",
                candidate_locations_.size(),
                wallName(board.wall),
                board.x, board.y,
                stopped_pose.x,
                stopped_pose.y,
                stopped_pose.yaw * 180.0 / kPi,
                box.x0, box.y0, box.x1, box.y1);
        } else {
            ROS_WARN(
                "新增无法分类候选位置[%zu]："
                "墙面交点计算失败，保存原检测位姿=(%.3f, %.3f, %.1f度)"
                "和框=(%d,%d)-(%d,%d)，候选回访时回到该位姿。",
                candidate_locations_.size(),
                stopped_pose.x,
                stopped_pose.y,
                stopped_pose.yaw * 180.0 / kPi,
                box.x0, box.y0, box.x1, box.y1);
        }
    }

    static double boxIntersectionOverUnion(
            const Box& first,
            const Box& second) {
        const int left =
            std::max(first.x0, second.x0);
        const int top =
            std::max(first.y0, second.y0);
        const int right =
            std::min(first.x1, second.x1);
        const int bottom =
            std::min(first.y1, second.y1);

        const double intersection =
            static_cast<double>(
                std::max(0, right - left) *
                std::max(0, bottom - top));

        const double first_area =
            static_cast<double>(
                std::max(0, first.x1 - first.x0) *
                std::max(0, first.y1 - first.y0));
        const double second_area =
            static_cast<double>(
                std::max(0, second.x1 - second.x0) *
                std::max(0, second.y1 - second.y0));

        const double union_area =
            first_area + second_area - intersection;

        return union_area > 1.0e-9
                   ? intersection / union_area
                   : 0.0;
    }

    std::vector<ViewTargetSnapshot> freezeVisibleNewTargets(
            const std::vector<Box>& boxes,
            const Pose2D& frame_pose) const {
        std::vector<ViewTargetSnapshot> frozen;
        frozen.reserve(boxes.size());

        // 重要：这一函数在开始处理任何OCR之前一次性执行。
        // 所有筛选只参考“进入当前视野时已经存在”的历史seen/candidate。
        // frozen建立之后，后续第一个目标新增的屏蔽状态不会影响本帧其它框。
        for (std::size_t i = 0;
             i < boxes.size(); ++i) {
            BoardBoundaryEstimate estimate;
            const bool estimate_ok =
                estimateBoardBoundary(
                    frame_pose,
                    boxes[i],
                    estimate);

            if (estimate_ok &&
                isSuppressedBoard(
                    estimate,
                    frame_pose)) {
                continue;
            }

            // 墙面交点失败的候选同样需要临时锁，否则会在原地重复OCR。
            if (!estimate_ok &&
                isCandidateRetryLocked(
                    estimate,
                    frame_pose)) {
                continue;
            }

            // 防止极少数NMS残留的几乎重叠框被当成两个物理目标。
            // 这里只按bbox重叠去重，不使用0.50m墙面坐标阈值，
            // 因而相邻的两块真实板即使距离很近也会分别保留。
            bool duplicate_bbox = false;
            for (std::size_t j = 0;
                 j < frozen.size(); ++j) {
                if (boxIntersectionOverUnion(
                        boxes[i],
                        frozen[j].box) >= 0.70) {
                    duplicate_bbox = true;
                    break;
                }
            }
            if (duplicate_bbox) {
                continue;
            }

            ViewTargetSnapshot item;
            item.box = boxes[i];
            item.estimate = estimate;
            frozen.push_back(item);
        }

        // 固定成从画面左到右依次OCR，日志和现场行为更可预测。
        std::sort(
            frozen.begin(),
            frozen.end(),
            [](const ViewTargetSnapshot& a,
               const ViewTargetSnapshot& b) {
                return a.box.centerX() <
                       b.box.centerX();
            });

        return frozen;
    }

    int chooseNewBoardBox(
            const std::vector<Box>& boxes,
            const Pose2D& frame_pose,
            BoardBoundaryEstimate& selected_estimate) const {
        selected_estimate =
            BoardBoundaryEstimate();

        const double image_center =
            0.5 * static_cast<double>(
                image_width_);

        int selected = -1;
        double best_error =
            std::numeric_limits<double>::infinity();

        for (std::size_t i = 0;
             i < boxes.size(); ++i) {
            BoardBoundaryEstimate estimate;
            const bool estimate_ok =
                estimateBoardBoundary(
                    frame_pose,
                    boxes[i],
                    estimate);

            if (estimate_ok &&
                isSuppressedBoard(
                    estimate,
                    frame_pose)) {
                continue;
            }

            // 墙面交点失败的候选同样需要临时锁，否则会在原地重复OCR。
            if (!estimate_ok &&
                isCandidateRetryLocked(
                    estimate,
                    frame_pose)) {
                continue;
            }

            const double error =
                std::fabs(
                    boxes[i].centerX() -
                    image_center);

            if (error < best_error) {
                best_error = error;
                selected =
                    static_cast<int>(i);
                selected_estimate =
                    estimate;
            }
        }

        return selected;
    }

    // ======================================================================
    // 扫描点固定角度控制：MyPlanner final pose yaw算法
    // ======================================================================
    double computeMyPlannerFinalYawCommand(
            double yaw_error) const {
        if (std::fabs(yaw_error) <=
            rotation_yaw_tolerance_) {
            return 0.0;
        }

        double wz =
            rotation_angular_gain_ *
            yaw_error;

        wz = applyMinimumMagnitude(
            wz,
            rotation_min_angular_speed_);

        return clampValue(
            wz,
            -rotation_max_angular_speed_,
            rotation_max_angular_speed_);
    }

    ProcessResult rotateToYawWhileScanning(
            double requested_target_deg,
            const std::string& point_name) {
        // R1.2：记录“进入这次固定角度旋转之前”是否已经有仿真目标。
        // 用它区分：
        //   A. 仿真是本次旋转中新找到的 -> 两目标凑齐可提前结束；
        //   B. 仿真是更早扫描阶段遗留的 -> 后找到现实仍保持旧规则。
        const bool simulation_known_before_this_rotation =
            simulation_observation_.valid ||
            simulation_docked_;

        const double target_yaw =
            normalizeAngle(
                requested_target_deg *
                kPi / 180.0);

        ROS_WARN(
            "%s开始固定角度扫描旋转：请求角度=%.1f度，"
            "归一化目标=%.1f度。旋转期间相机持续开启。",
            point_name.c_str(),
            requested_target_deg,
            target_yaw * 180.0 / kPi);

        ros::Rate rate(rotation_control_rate_);
        ros::WallTime last_time =
            ros::WallTime::now();
        ros::WallTime deadline =
            ros::WallTime::now() +
            ros::WallDuration(rotation_timeout_);

        double command_wz = 0.0;

        while (ros::ok() &&
               ros::WallTime::now() < deadline) {
            ros::spinOnce();

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishAngular(0.0);
                command_wz = 0.0;
                rate.sleep();
                continue;
            }

            const double yaw_error =
                normalizeAngle(
                    target_yaw - pose.yaw);

            // 与MyPlanner computeFinalPoseCommand一致：
            // yaw进入goal_yaw_tolerance立即认为姿态完成。
            if (std::fabs(yaw_error) <=
                rotation_yaw_tolerance_) {
                stopRobot();
                ROS_INFO(
                    "%s固定角度到达：请求=%.1f度，实际=%.2f度，"
                    "误差=%.2f度",
                    point_name.c_str(),
                    requested_target_deg,
                    pose.yaw * 180.0 / kPi,
                    yaw_error * 180.0 / kPi);
                return PROCESS_CONTINUE;
            }

            const double desired_wz =
                computeMyPlannerFinalYawCommand(
                    yaw_error);

            const ros::WallTime now =
                ros::WallTime::now();
            double dt =
                (now - last_time).toSec();
            last_time = now;
            if (!std::isfinite(dt) ||
                dt <= 0.0) {
                dt = 0.05;
            }
            dt = clampValue(dt, 0.01, 0.30);

            // 与MyPlanner applyVelocityAndAccelerationLimits中的
            // angular.z限加速度形式一致。
            const double max_delta =
                rotation_acc_lim_theta_ * dt;
            command_wz =
                clampValue(
                    desired_wz,
                    command_wz - max_delta,
                    command_wz + max_delta);

            // /set_speed会持续20Hz发布该角速度，所以在检测服务执行期间
            // 小车仍持续转动。
            publishAngular(command_wz);

            // 帧采集发生在detect服务调用刚开始处，因此这里先保存当前
            // map位姿，作为该检测帧最接近的相机位姿，避免“旧图+新pose”。
            Pose2D frame_pose = pose;
            getRobotPose(frame_pose);

            std::vector<Box> boxes;
            if (detectBoxes(boxes) &&
                !boxes.empty()) {
                // R1.3：停车前先一次性冻结当前检测帧内全部历史未屏蔽目标。
                // 后续逐个OCR时绝不重新用刚更新的seen/candidate筛掉同帧其它框。
                const std::vector<ViewTargetSnapshot> visible_targets =
                    freezeVisibleNewTargets(
                        boxes,
                        frame_pose);

                if (!visible_targets.empty()) {
                    stopRobot();
                    command_wz = 0.0;

                    ROS_WARN(
                        "R1.3旋转视野冻结：本帧检测到%zu个框，"
                        "其中%zu个为历史未屏蔽独立目标；"
                        "现在保持停车，从左到右依次OCR全部目标。",
                        boxes.size(),
                        visible_targets.size());

                    for (std::size_t target_i = 0;
                         target_i < visible_targets.size() &&
                         ros::ok();
                         ++target_i) {
                        const ViewTargetSnapshot& item =
                            visible_targets[target_i];

                        ROS_WARN(
                            "R1.3处理当前视野目标%zu/%zu："
                            "center=(%.1f,%.1f)，框=(%d,%d)-(%d,%d)",
                            target_i + 1,
                            visible_targets.size(),
                            item.box.centerX(),
                            item.box.centerY(),
                            item.box.x0,
                            item.box.y0,
                            item.box.x1,
                            item.box.y1);

                        const ProcessResult result =
                            processDetectedBoard(
                                frame_pose,
                                item.box,
                                item.estimate,
                                false,
                                true);

                        if (result != PROCESS_CONTINUE) {
                            return result;
                        }

                        if (bothTargetsReadyForSameScanPhase(
                                simulation_known_before_this_rotation)) {
                            ROS_WARN(
                                "R1.3提前结束扫描：现实目标和仿真目标已在"
                                "本次固定角度旋转视野内凑齐，"
                                "不再继续当前扫描点剩余角度。");
                            return PROCESS_BOTH_TARGETS_READY;
                        }
                    }

                    // OCR全部冻结目标的耗时不计入固定角度旋转超时。
                    deadline =
                        ros::WallTime::now() +
                        ros::WallDuration(
                            rotation_timeout_);
                    last_time =
                        ros::WallTime::now();
                    command_wz = 0.0;
                }
            }

            ROS_INFO_THROTTLE(
                0.5,
                "%s旋转扫描中：当前=%.1f度，目标=%.1f度，"
                "剩余=%.1f度，cmd_wz=%.3f",
                point_name.c_str(),
                pose.yaw * 180.0 / kPi,
                requested_target_deg,
                yaw_error * 180.0 / kPi,
                command_wz);

            rate.sleep();
        }

        stopRobot();
        ROS_ERROR(
            "%s固定角度旋转超时：目标=%.1f度",
            point_name.c_str(),
            requested_target_deg);
        return PROCESS_ABORT;
    }

    bool rotateToYawWithoutScanning(
            double target_yaw,
            const std::string& label) {
        target_yaw =
            normalizeAngle(target_yaw);

        ros::Rate rate(rotation_control_rate_);
        ros::WallTime last_time =
            ros::WallTime::now();
        const ros::WallTime deadline =
            ros::WallTime::now() +
            ros::WallDuration(rotation_timeout_);

        double command_wz = 0.0;

        while (ros::ok() &&
               ros::WallTime::now() < deadline) {
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishAngular(0.0);
                command_wz = 0.0;
                rate.sleep();
                continue;
            }

            const double yaw_error =
                normalizeAngle(
                    target_yaw - pose.yaw);

            if (std::fabs(yaw_error) <=
                rotation_yaw_tolerance_) {
                stopRobot();
                ROS_INFO(
                    "%s完成：最终朝向=%.2f度，误差=%.2f度",
                    label.c_str(),
                    pose.yaw * 180.0 / kPi,
                    yaw_error * 180.0 / kPi);
                return true;
            }

            const double desired_wz =
                computeMyPlannerFinalYawCommand(
                    yaw_error);

            const ros::WallTime now =
                ros::WallTime::now();
            double dt =
                (now - last_time).toSec();
            last_time = now;
            if (!std::isfinite(dt) ||
                dt <= 0.0) {
                dt = 0.05;
            }
            dt = clampValue(dt, 0.01, 0.30);

            const double max_delta =
                rotation_acc_lim_theta_ * dt;
            command_wz =
                clampValue(
                    desired_wz,
                    command_wz - max_delta,
                    command_wz + max_delta);

            publishAngular(command_wz);
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR(
            "%s旋转超时",
            label.c_str());
        return false;
    }

    ProcessResult inspectCurrentView(
            const std::string& label) {
        const bool simulation_known_before_this_view =
            simulation_observation_.valid ||
            simulation_docked_;

        for (int pass = 0;
             pass < static_view_detection_passes_ &&
             ros::ok();
             ++pass) {
            Pose2D frame_pose;
            if (!getRobotPose(frame_pose)) {
                return PROCESS_ABORT;
            }

            std::vector<Box> boxes;
            if (!detectBoxes(boxes) ||
                boxes.empty()) {
                return PROCESS_CONTINUE;
            }

            const std::vector<ViewTargetSnapshot> visible_targets =
                freezeVisibleNewTargets(
                    boxes,
                    frame_pose);

            if (visible_targets.empty()) {
                ROS_INFO_THROTTLE(
                    1.0,
                    "%s当前所有框均属于历史已屏蔽位置/"
                    "历史候选位置，不再重复OCR。",
                    label.c_str());
                return PROCESS_CONTINUE;
            }

            stopRobot();

            ROS_WARN(
                "R1.3 %s视野冻结：本帧检测到%zu个框，"
                "冻结%zu个历史未屏蔽独立目标；"
                "从左到右依次OCR全部目标。",
                label.c_str(),
                boxes.size(),
                visible_targets.size());

            for (std::size_t target_i = 0;
                 target_i < visible_targets.size() &&
                 ros::ok();
                 ++target_i) {
                const ViewTargetSnapshot& item =
                    visible_targets[target_i];

                ROS_WARN(
                    "%s处理视野目标%zu/%zu："
                    "center=(%.1f,%.1f)，框=(%d,%d)-(%d,%d)",
                    label.c_str(),
                    target_i + 1,
                    visible_targets.size(),
                    item.box.centerX(),
                    item.box.centerY(),
                    item.box.x0,
                    item.box.y0,
                    item.box.x1,
                    item.box.y1);

                const ProcessResult result =
                    processDetectedBoard(
                        frame_pose,
                        item.box,
                        item.estimate,
                        false,
                        true);

                if (result != PROCESS_CONTINUE) {
                    return result;
                }

                if (bothTargetsReadyForSameScanPhase(
                        simulation_known_before_this_view)) {
                    ROS_WARN(
                        "R1.3提前结束扫描：现实目标和仿真目标已在"
                        "%s这一视野内凑齐。",
                        label.c_str());
                    return PROCESS_BOTH_TARGETS_READY;
                }
            }

            // 一次检测帧里已经把所有新框处理完。
            // 若还允许更多pass，则下一pass只用于发现刚才帧中没有出现的
            // 新目标；历史列表会正常屏蔽已经处理过的那些板。
        }

        return PROCESS_CONTINUE;
    }

    // ======================================================================
    // 单次目标处理
    // ======================================================================
    ProcessResult processDetectedBoard(
            const Pose2D& frame_pose,
            const Box& trigger_box,
            const BoardBoundaryEstimate& trigger_estimate,
            bool from_candidate_revisit,
            bool frozen_view_member = false) {
        stopRobot();

        Pose2D stopped_pose = frame_pose;
        getRobotPose(stopped_pose);

        OcrRecord ocr =
            recognizeStaticTarget(trigger_box);

        // OCR服务使用的是停车后的新帧，所以用OCR返回框 + 停车后pose
        // 再估算一次墙面坐标；失败时回退到旋转检测帧的交点。
        BoardBoundaryEstimate final_board;
        bool final_board_valid =
            estimateBoardBoundary(
                stopped_pose,
                ocr.box,
                final_board);

        if (!final_board_valid &&
            trigger_estimate.valid) {
            final_board =
                trigger_estimate;
            final_board_valid = true;
            ROS_WARN(
                "OCR静止框无法重新估计墙面坐标，"
                "回退使用触发检测帧的墙面交点。");
        }

        if (final_board_valid &&
            isSeenBoard(final_board) &&
            !frozen_view_member) {
            ROS_INFO(
                "OCR后位置%s(%.3f,%.3f)已在同位置屏蔽名单中，"
                "忽略重复结果。",
                wallName(final_board.wall),
                final_board.x,
                final_board.y);
            return PROCESS_CONTINUE;
        }

        if (final_board_valid &&
            isSeenBoard(final_board) &&
            frozen_view_member) {
            ROS_INFO(
                "R1.3同帧保护：%s(%.3f,%.3f)虽然在前一个同帧目标"
                "处理后进入了0.50m屏蔽范围，但该框在停车瞬间已经作为"
                "独立bbox冻结，因此仍继续本次OCR分类，不被动态屏蔽。",
                wallName(final_board.wall),
                final_board.x,
                final_board.y);
        }

        if (ocr.category == "unknown") {
            if (!from_candidate_revisit) {
                const int existing_candidate =
                    findMutableMatchingCandidateIndex(
                        final_board,
                        stopped_pose);

                if (existing_candidate >= 0) {
                    // candidate不永久屏蔽，因此当视角满足解锁条件后，
                    // 同一目标会再次进入OCR。若仍unknown，就把“最近失败视角”
                    // 更新到这次，而不是重复push候选。
                    updateCandidateFailedView(
                        existing_candidate,
                        stopped_pose,
                        ocr.box);
                } else {
                    addCandidateLocation(
                        final_board,
                        stopped_pose,
                        ocr.box,
                        frozen_view_member);
                }
            } else {
                // 正式candidate回访阶段无条件绕过主扫描冷却锁。
                // 基准/+30/-30三视角由revisitCandidates()自己控制。
                ROS_WARN(
                    "候选位置回访后仍无法分类：'%s'。"
                    "继续按候选回访既定的基准/+30/-30流程处理。",
                    ocr.text.c_str());
            }
            return PROCESS_CONTINUE;
        }

        if (final_board_valid) {
            registerSeenBoard(
                final_board,
                std::string("OCR分类=") +
                    categoryChinese(ocr.category));
        }

        ROS_WARN(
            "目标分类成功：%s；当前位置=%s",
            categoryChinese(ocr.category),
            final_board_valid
                ? wallName(final_board.wall)
                : "墙面坐标未知");

        // 第三个无关类别也进入同位置屏蔽名单，但不参与任务。
        if (ocr.category != real_target_category_ &&
            ocr.category != simulation_target_category_) {
            ROS_INFO(
                "该类别既不是现实目标也不是仿真目标，"
                "已屏蔽该位置并继续搜索。");
            return PROCESS_CONTINUE;
        }

        if (!final_board_valid) {
            ROS_ERROR(
                "类别已识别为%s，但无法获得可靠墙面坐标，"
                "不能生成安全停靠点；本次不覆盖已有目标。",
                categoryChinese(ocr.category));
            return PROCESS_CONTINUE;
        }

        TargetObservation observation;
        if (!makeDockingObservation(
                ocr.category,
                final_board,
                observation)) {
            ROS_ERROR(
                "无法为%s生成第一段预停靠点",
                categoryChinese(ocr.category));
            return PROCESS_CONTINUE;
        }

        if (ocr.category ==
            real_target_category_) {
            if (!real_observation_.valid &&
                !real_docked_) {
                real_observation_ = observation;
                ROS_WARN(
                    "现实目标已记录：板=%s(%.3f,%.3f)，"
                    "第一段停靠点=(%.3f,%.3f,%.1f度)。"
                    "按照规则，主扫描阶段必须等当前坐标所有角度看完"
                    "以后才能停靠。",
                    wallName(observation.board.wall),
                    observation.board.x,
                    observation.board.y,
                    observation.pose.x,
                    observation.pose.y,
                    observation.pose.yaw *
                        180.0 / kPi);
            } else {
                ROS_INFO(
                    "现实目标已经记录/完成，不覆盖首次有效位置。");
            }

            // 主扫描阶段绝不在这里停靠现实目标。
            // 候选回访阶段主扫描早已结束，因此可以立即停靠。
            if (from_candidate_revisit &&
                real_observation_.valid &&
                !real_docked_) {
                if (!dockStoredTarget(
                        real_observation_,
                        "候选回访现实目标")) {
                    return PROCESS_ABORT;
                }
                real_docked_ = true;

                if (simulation_observation_.valid &&
                    !simulation_docked_) {
                    if (!dockStoredTarget(
                            simulation_observation_,
                            "已记录仿真目标")) {
                        return PROCESS_ABORT;
                    }
                    simulation_docked_ = true;
                }

                return missionComplete()
                    ? PROCESS_MISSION_COMPLETE
                    : PROCESS_CONTINUE;
            }

            return PROCESS_CONTINUE;
        }

        // 仿真目标：必须等现实目标完成停靠。
        if (ocr.category ==
            simulation_target_category_) {
            if (!simulation_observation_.valid &&
                !simulation_docked_) {
                simulation_observation_ =
                    observation;
                ROS_WARN(
                    "仿真目标已记录：板=%s(%.3f,%.3f)，"
                    "第一段停靠点=(%.3f,%.3f,%.1f度)。",
                    wallName(observation.board.wall),
                    observation.board.x,
                    observation.board.y,
                    observation.pose.x,
                    observation.pose.y,
                    observation.pose.yaw *
                        180.0 / kPi);
            } else {
                ROS_INFO(
                    "仿真目标已经记录/完成，不覆盖首次有效位置。");
            }

            if (!real_docked_) {
                // R1.1新增例外，只针对“现实先找到、本次又找到仿真”。
                // 此时两个任务目标的位置已经都确定，继续转当前扫描点
                // 不再产生任务收益，所以立即结束剩余角度，先停现实再停仿真。
                //
                // 反方向“仿真先找到、这次才找到现实”不在这里触发，
                // 现实目标分支仍保持原规则：继续把当前坐标所有角度看完。
                if (!from_candidate_revisit &&
                    real_observation_.valid &&
                    simulation_observation_.valid) {
                    ROS_WARN(
                        "R1.3提前结束扫描：现实目标已经记录，"
                        "本次又确认仿真目标；两个目标位置均已确定，"
                        "不再继续当前扫描点剩余角度。");
                    return PROCESS_BOTH_TARGETS_READY;
                }

                ROS_INFO(
                    "现实目标尚未完成停靠，"
                    "仿真目标只记录位置，继续扫描。");
                return PROCESS_CONTINUE;
            }

            // 现实目标已经完成，则仿真目标允许立即停靠。
            if (simulation_observation_.valid &&
                !simulation_docked_) {
                ROS_WARN(
                    "现实目标已经完成，"
                    "现在允许立即停靠仿真目标。");
                if (!dockStoredTarget(
                        simulation_observation_,
                        from_candidate_revisit
                            ? "候选回访仿真目标"
                            : "仿真目标")) {
                    return PROCESS_ABORT;
                }
                simulation_docked_ = true;
            }

            return missionComplete()
                ? PROCESS_MISSION_COMPLETE
                : PROCESS_CONTINUE;
        }

        return PROCESS_CONTINUE;
    }

    // ======================================================================
    // move_base导航
    // ======================================================================
    bool navigateToPose(
            double x,
            double y,
            double yaw,
            const std::string& purpose) {
        clampToRoom(x, y);

        if (!isInsideRoom(x, y)) {
            ROS_ERROR(
                "%s目标超出房间："
                "(%.3f,%.3f,%.1f度)",
                purpose.c_str(),
                x, y,
                yaw * 180.0 / kPi);
            return false;
        }

        stopRobot();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id =
            map_frame_;
        goal.target_pose.header.stamp =
            ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation =
            tf2::toMsg(quaternion);

        ROS_INFO(
            "%s：发送move_base目标"
            "(%.3f, %.3f, %.1f度)",
            purpose.c_str(),
            x, y,
            yaw * 180.0 / kPi);

        move_base_.sendGoal(goal);

        if (!move_base_.waitForResult(
                ros::Duration(
                    navigation_timeout_))) {
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("%s超时", purpose.c_str());
            return false;
        }

        if (move_base_.getState() !=
            actionlib::SimpleClientGoalState::
                SUCCEEDED) {
            const std::string state =
                move_base_.getState().toString();
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR(
                "%s失败：%s",
                purpose.c_str(),
                state.c_str());
            return false;
        }

        stopRobot();
        ROS_INFO("%s完成", purpose.c_str());
        return true;
    }

    // ======================================================================
    // 两段式停靠
    // ======================================================================
    bool makeDockingObservationAtStandoff(
            const std::string& category,
            const BoardBoundaryEstimate& board,
            double standoff,
            TargetObservation& observation) const {
        observation = TargetObservation();

        if (!board.valid ||
            standoff <= 0.0) {
            return false;
        }

        observation.valid = true;
        observation.category = category;
        observation.board = board;
        observation.pose.x = board.x;
        observation.pose.y = board.y;

        switch (board.wall) {
            case WALL_TOP:
                observation.pose.y =
                    room_max_y_ - standoff;
                observation.pose.yaw =
                    0.5 * kPi;
                break;
            case WALL_RIGHT:
                observation.pose.x =
                    room_max_x_ - standoff;
                observation.pose.yaw =
                    0.0;
                break;
            case WALL_BOTTOM:
                observation.pose.y =
                    room_min_y_ + standoff;
                observation.pose.yaw =
                    -0.5 * kPi;
                break;
            case WALL_LEFT:
                observation.pose.x =
                    room_min_x_ + standoff;
                observation.pose.yaw =
                    kPi;
                break;
        }

        return isInsideRoom(
            observation.pose.x,
            observation.pose.y);
    }

    bool makeDockingObservation(
            const std::string& category,
            const BoardBoundaryEstimate& board,
            TargetObservation& observation) const {
        return makeDockingObservationAtStandoff(
            category,
            board,
            docking_standoff_,
            observation);
    }

    double boardCoordinateDistance(
            const TargetObservation& old_observation,
            const BoardBoundaryEstimate& estimate) const {
        if (!old_observation.valid ||
            !old_observation.board.valid ||
            old_observation.board.wall !=
                estimate.wall) {
            return
                std::numeric_limits<double>::infinity();
        }

        return distance2D(
            old_observation.board.x,
            old_observation.board.y,
            estimate.x,
            estimate.y);
    }

    bool detectDockingRefinedObservation(
            const TargetObservation& old_observation,
            const std::string& scan_name,
            double final_standoff,
            TargetObservation& refined_observation,
            bool force_refresh,
            Box* matched_box_out = nullptr) {
        if (force_refresh &&
            !clearDetectionBuffer(scan_name)) {
            return false;
        }

        for (int attempt = 0;
             attempt <
                 docking_recovery_detection_attempts_ &&
             ros::ok();
             ++attempt) {
            Pose2D frame_pose;
            if (!getRobotPose(frame_pose)) {
                continue;
            }

            std::vector<Box> boxes;
            if (!detectBoxes(boxes) ||
                boxes.empty()) {
                ROS_WARN(
                    "%s第%d/%d次没有检测到文字板",
                    scan_name.c_str(),
                    attempt + 1,
                    docking_recovery_detection_attempts_);
                continue;
            }

            int selected = -1;
            double best_distance =
                std::numeric_limits<double>::infinity();
            BoardBoundaryEstimate best_board;

            for (std::size_t i = 0;
                 i < boxes.size(); ++i) {
                BoardBoundaryEstimate estimate;
                if (!estimateBoardBoundary(
                        frame_pose,
                        boxes[i],
                        estimate)) {
                    continue;
                }

                if (estimate.wall !=
                    old_observation.board.wall) {
                    continue;
                }

                const double distance =
                    boardCoordinateDistance(
                        old_observation,
                        estimate);

                if (!std::isfinite(distance) ||
                    distance >
                        docking_refine_max_board_shift_) {
                    continue;
                }

                if (distance < best_distance) {
                    best_distance = distance;
                    selected =
                        static_cast<int>(i);
                    best_board = estimate;
                }
            }

            if (selected >= 0 &&
                makeDockingObservationAtStandoff(
                    old_observation.category,
                    best_board,
                    final_standoff,
                    refined_observation)) {
                const Box& box =
                    boxes[
                        static_cast<std::size_t>(
                            selected)];

                if (matched_box_out) {
                    *matched_box_out = box;
                }

                ROS_WARN(
                    "%s成功：同一目标框center=(%.1f,%.1f)，"
                    "旧板=%s(%.3f,%.3f)，"
                    "新板=%s(%.3f,%.3f)，"
                    "修正=%.3fm，最终点=(%.3f,%.3f,%.1f度)",
                    scan_name.c_str(),
                    box.centerX(),
                    box.centerY(),
                    wallName(
                        old_observation.board.wall),
                    old_observation.board.x,
                    old_observation.board.y,
                    wallName(best_board.wall),
                    best_board.x,
                    best_board.y,
                    best_distance,
                    refined_observation.pose.x,
                    refined_observation.pose.y,
                    refined_observation.pose.yaw *
                        180.0 / kPi);
                return true;
            }
        }

        return false;
    }

    bool recoverDockingObservation(
            TargetObservation& observation,
            const std::string& target_name,
            double final_standoff) {
        const TargetObservation old_observation =
            observation;

        const double center_yaw =
            old_observation.pose.yaw;
        const double turn =
            docking_recovery_turn_deg_ *
            kPi / 180.0;

        const double scan_yaws[2] = {
            normalizeAngle(center_yaw + turn),
            normalizeAngle(center_yaw - turn)
        };

        const char* names[2] = {
            "停靠左侧恢复",
            "停靠右侧恢复"
        };

        for (int i = 0;
             i < 2 && ros::ok();
             ++i) {
            if (!rotateToYawWithoutScanning(
                    scan_yaws[i],
                    names[i])) {
                continue;
            }

            TargetObservation recovered;
            if (!detectDockingRefinedObservation(
                    old_observation,
                    names[i],
                    final_standoff,
                    recovered,
                    true)) {
                continue;
            }

            observation = recovered;
            ROS_WARN(
                "%s恢复定位成功",
                target_name.c_str());
            return true;
        }

        ROS_ERROR(
            "%s在±%.1f度范围内均未重新找到同一块板",
            target_name.c_str(),
            docking_recovery_turn_deg_);
        return false;
    }

    bool dockStoredTarget(
            TargetObservation& observation,
            const std::string& target_name) {
        if (!observation.valid ||
            !observation.board.valid) {
            ROS_ERROR(
                "%s停靠状态无效",
                target_name.c_str());
            return false;
        }

        // V14.2 Camera Handoff：
        // 第一段move_base之前关闭相机，导航期间根本不积压旧帧。
        closeCamera();

        if (!navigateToPose(
                observation.pose.x,
                observation.pose.y,
                observation.pose.yaw,
                target_name + "第一段预停靠")) {
            return false;
        }

        if (!openCamera()) return false;

        ROS_WARN(
            "%s第一段完成：板=%s(%.3f,%.3f)，"
            "开始实时视觉第二段复定位。",
            target_name.c_str(),
            wallName(observation.board.wall),
            observation.board.x,
            observation.board.y);

        TargetObservation final_observation;
        Box refined_box{0, 0, 0, 0, 0};

        bool refined =
            detectDockingRefinedObservation(
                observation,
                target_name + "第二段视觉复定位",
                approach_stop_distance_,
                final_observation,
                false,
                &refined_box);

        // R1.6：临时停靠点的边缘视野保护。
        // 第一帧已经成功关联到原目标后，如果框中心落在图像最左/最右1/4，
        // 不直接使用该斜视角的最终坐标，而是先向目标方向转30°再重新计算。
        if (refined) {
            const double left_quarter =
                0.25 * static_cast<double>(
                    image_width_);
            const double right_quarter =
                0.75 * static_cast<double>(
                    image_width_);

            const bool in_left_quarter =
                refined_box.centerX() <=
                left_quarter;
            const bool in_right_quarter =
                refined_box.centerX() >=
                right_quarter;

            if (in_left_quarter ||
                in_right_quarter) {
                Pose2D current_pose;
                if (!getRobotPose(current_pose)) {
                    ROS_ERROR(
                        "%s边缘视野调整前无法读取当前map位姿",
                        target_name.c_str());
                    return false;
                }

                const double turn =
                    docking_recovery_turn_deg_ *
                    kPi / 180.0;
                const double signed_turn =
                    in_left_quarter
                        ? turn
                        : -turn;

                const double adjusted_yaw =
                    normalizeAngle(
                        current_pose.yaw +
                        signed_turn);

                ROS_WARN(
                    "R1.6临时停靠边缘保护：%s目标框centerX=%.1f，"
                    "图像左1/4阈值=%.1f，右1/4阈值=%.1f；"
                    "目标位于%s侧，先向%s旋转%.1f度，"
                    "当前yaw=%.1f度 -> 新yaw=%.1f度，"
                    "转完清缓存并重新计算最终停靠点。",
                    target_name.c_str(),
                    refined_box.centerX(),
                    left_quarter,
                    right_quarter,
                    in_left_quarter ? "左" : "右",
                    in_left_quarter
                        ? "左/逆时针"
                        : "右/顺时针",
                    docking_recovery_turn_deg_,
                    current_pose.yaw *
                        180.0 / kPi,
                    adjusted_yaw *
                        180.0 / kPi);

                if (!rotateToYawWithoutScanning(
                        adjusted_yaw,
                        target_name +
                            "临时停靠边缘30度修正")) {
                    ROS_WARN(
                        "%s边缘30度修正失败，"
                        "转入原有±恢复逻辑。",
                        target_name.c_str());
                    refined = false;
                } else {
                    TargetObservation edge_refined;
                    Box edge_box{0, 0, 0, 0, 0};

                    // force_refresh=true：
                    // 旋转过程中相机一直开着，转完先-3清旧帧再正式定位。
                    if (detectDockingRefinedObservation(
                            observation,
                            target_name +
                                "边缘修正后第二段视觉复定位",
                            approach_stop_distance_,
                            edge_refined,
                            true,
                            &edge_box)) {
                        final_observation =
                            edge_refined;
                        refined_box =
                            edge_box;

                        ROS_WARN(
                            "R1.6边缘修正成功：转向后框centerX=%.1f，"
                            "现在使用新视角重新计算的最终停靠点。",
                            refined_box.centerX());
                    } else {
                        ROS_WARN(
                            "%s边缘30度修正后仍无法可靠关联原目标，"
                            "转入原有±恢复逻辑。",
                            target_name.c_str());
                        refined = false;
                    }
                }
            } else {
                ROS_INFO(
                    "%s临时停靠复定位框centerX=%.1f，"
                    "位于图像中间二分之一[%.1f, %.1f]，"
                    "不做额外30度转向。",
                    target_name.c_str(),
                    refined_box.centerX(),
                    left_quarter,
                    right_quarter);
            }
        }

        if (!refined) {
            final_observation = observation;
            ROS_WARN(
                "%s当前朝向/边缘修正后无法复定位原板，"
                "启动原有±%.1f度恢复。",
                target_name.c_str(),
                docking_recovery_turn_deg_);

            if (!recoverDockingObservation(
                    final_observation,
                    target_name,
                    approach_stop_distance_)) {
                return false;
            }
        }

        observation = final_observation;

        ROS_WARN(
            "%s最终move_base停靠点："
            "(%.3f,%.3f,%.1f度)，"
            "距墙=%.3fm",
            target_name.c_str(),
            observation.pose.x,
            observation.pose.y,
            observation.pose.yaw *
                180.0 / kPi,
            approach_stop_distance_);

        if (!navigateToPose(
                observation.pose.x,
                observation.pose.y,
                observation.pose.yaw,
                target_name + "最终停靠")) {
            return false;
        }

        closeCamera();
        ROS_WARN(
            "%s两段式停靠完成",
            target_name.c_str());
        return true;
    }

    // ======================================================================
    // 候选位置回访
    // ======================================================================
    bool chooseCandidateMatchingBox(
            const CandidateLocation& candidate,
            const std::vector<Box>& boxes,
            const Pose2D& frame_pose,
            Box& selected_box,
            BoardBoundaryEstimate& selected_board) const {
        int selected = -1;
        double best_score =
            std::numeric_limits<double>::infinity();

        for (std::size_t i = 0;
             i < boxes.size(); ++i) {
            if (candidate.board_valid) {
                BoardBoundaryEstimate estimate;
                if (!estimateBoardBoundary(
                        frame_pose,
                        boxes[i],
                        estimate)) {
                    continue;
                }

                if (estimate.wall !=
                    candidate.board.wall) {
                    continue;
                }

                const double distance =
                    distance2D(
                        estimate.x,
                        estimate.y,
                        candidate.board.x,
                        candidate.board.y);

                if (distance >
                    candidate_match_max_shift_) {
                    continue;
                }

                if (distance < best_score) {
                    best_score = distance;
                    selected =
                        static_cast<int>(i);
                    selected_board = estimate;
                }
            } else {
                const double dx =
                    boxes[i].centerX() -
                    candidate.trigger_box.centerX();
                const double dy =
                    boxes[i].centerY() -
                    candidate.trigger_box.centerY();
                const double distance =
                    std::hypot(dx, dy);

                if (distance < best_score) {
                    best_score = distance;
                    selected =
                        static_cast<int>(i);
                }
            }
        }

        if (selected < 0) return false;

        selected_box =
            boxes[
                static_cast<std::size_t>(
                    selected)];

        if (!candidate.board_valid) {
            estimateBoardBoundary(
                frame_pose,
                selected_box,
                selected_board);
        }

        return true;
    }

    CandidateViewResult tryCandidateAtCurrentView(
            CandidateLocation& candidate,
            const std::string& view_name,
            bool refresh_after_rotation) {
        if (refresh_after_rotation) {
            // 候选恢复旋转期间摄像头始终开启，但没有持续read。
            // 与停靠±30°恢复完全相同：转完先-3清掉旧缓存，再正式检测。
            if (!clearDetectionBuffer(view_name)) {
                ROS_WARN(
                    "%s清NanoDet缓存失败，本朝向仍继续尝试检测。",
                    view_name.c_str());
            }
        }

        bool matched = false;
        Box matched_box{0, 0, 0, 0, 0};
        BoardBoundaryEstimate matched_board;
        Pose2D matched_frame_pose;

        for (int attempt = 0;
             attempt < candidate_detection_attempts_ &&
             ros::ok();
             ++attempt) {
            Pose2D frame_pose;
            if (!getRobotPose(frame_pose)) {
                continue;
            }

            std::vector<Box> boxes;
            if (!detectBoxes(boxes) ||
                boxes.empty()) {
                ROS_WARN(
                    "%s第%d/%d次未检测到文字框",
                    view_name.c_str(),
                    attempt + 1,
                    candidate_detection_attempts_);
                continue;
            }

            if (chooseCandidateMatchingBox(
                    candidate,
                    boxes,
                    frame_pose,
                    matched_box,
                    matched_board)) {
                matched = true;
                matched_frame_pose = frame_pose;
                ROS_WARN(
                    "%s重新找到候选目标："
                    "center=(%.1f,%.1f)，框=(%d,%d)-(%d,%d)",
                    view_name.c_str(),
                    matched_box.centerX(),
                    matched_box.centerY(),
                    matched_box.x0,
                    matched_box.y0,
                    matched_box.x1,
                    matched_box.y1);
                break;
            }

            ROS_WARN(
                "%s第%d/%d次检测到%d个框，但没有与该候选匹配的目标",
                view_name.c_str(),
                attempt + 1,
                candidate_detection_attempts_,
                static_cast<int>(boxes.size()));
        }

        if (!matched) {
            return CANDIDATE_VIEW_NOT_FOUND;
        }

        stopRobot();

        const ProcessResult result =
            processDetectedBoard(
                matched_frame_pose,
                matched_box,
                matched_board,
                true);

        if (result == PROCESS_ABORT) {
            return CANDIDATE_VIEW_ABORT;
        }
        if (result == PROCESS_MISSION_COMPLETE ||
            result == PROCESS_BOTH_TARGETS_READY) {
            return CANDIDATE_VIEW_MISSION_COMPLETE;
        }

        // 成功分类后processDetectedBoard会把墙面位置写入seen名单。
        // 用“本次重新匹配出的墙面坐标”判断，避免candidate原始坐标
        // 与重新检测坐标相差0.5~0.8m时误判为仍未解决。
        if (matched_board.valid &&
            isSeenBoard(matched_board)) {
            candidate.resolved = true;
            ROS_INFO(
                "%s候选已经成功分类，候选状态标记为已解决。",
                view_name.c_str());
            return CANDIDATE_VIEW_RESOLVED;
        }

        // 对于极少数原候选没有有效墙面坐标的情况，如果这次分类后
        // real/simulation记录已经形成，也视为本次候选得到有效结果。
        if ((!candidate.board_valid) &&
            (real_observation_.valid ||
             simulation_observation_.valid)) {
            candidate.resolved = true;
            ROS_INFO(
                "%s候选已形成有效任务目标记录，标记为已解决。",
                view_name.c_str());
            return CANDIDATE_VIEW_RESOLVED;
        }

        ROS_WARN(
            "%s虽然重新检测到候选目标，但OCR后仍无法可靠分类；"
            "允许继续尝试候选恢复朝向。",
            view_name.c_str());
        return CANDIDATE_VIEW_STILL_UNKNOWN;
    }

    bool revisitCandidates() {
        for (std::size_t i = 0;
             i < candidate_locations_.size() &&
             ros::ok();
             ++i) {
            CandidateLocation& candidate =
                candidate_locations_[i];

            if (missionComplete()) return true;
            if (candidate.resolved) continue;

            // 如果主扫描后续已经从别的角度成功分类了同一位置，
            // 该候选直接跳过回访。
            if (candidate.board_valid &&
                isSeenBoard(candidate.board)) {
                candidate.resolved = true;
                ROS_INFO(
                    "候选[%zu]已经由其它视角成功分类，跳过回访。",
                    i + 1);
                continue;
            }

            candidate.attempted = true;

            Pose2D revisit_pose;
            if (candidate.board_valid) {
                TargetObservation candidate_view;
                if (!makeDockingObservation(
                        "unknown",
                        candidate.board,
                        candidate_view)) {
                    ROS_WARN(
                        "候选[%zu]无法生成回访点，跳过。",
                        i + 1);
                    continue;
                }
                revisit_pose =
                    candidate_view.pose;
            } else {
                // 射线交点失败的兜底候选：直接回到当时发现该框的位姿。
                revisit_pose =
                    candidate.detection_pose;
            }

            closeCamera();

            ROS_WARN(
                "回访候选[%zu/%zu]：导航到"
                "(%.3f,%.3f,%.1f度)",
                i + 1,
                candidate_locations_.size(),
                revisit_pose.x,
                revisit_pose.y,
                revisit_pose.yaw *
                    180.0 / kPi);

            if (!navigateToPose(
                    revisit_pose.x,
                    revisit_pose.y,
                    revisit_pose.yaw,
                    "候选位置回访")) {
                ROS_WARN(
                    "候选[%zu]导航失败，继续下一个候选。",
                    i + 1);
                continue;
            }

            if (!openCamera()) return false;

            // ----------------------------------------------------------
            // R1.4候选回访扫描：
            //   0. 基准朝向
            //   1. 基准 + recovery_turn
            //   2. 基准 - recovery_turn
            //
            // 当前朝向如果：
            //   - 没有匹配框；
            //   - 或匹配到了，但OCR仍unknown；
            // 都继续尝试下一恢复方向。
            // ----------------------------------------------------------
            const double base_yaw =
                normalizeAngle(revisit_pose.yaw);
            const double recovery_turn =
                docking_recovery_turn_deg_ *
                kPi / 180.0;

            CandidateViewResult view_result =
                tryCandidateAtCurrentView(
                    candidate,
                    "候选基准朝向",
                    false);

            if (view_result == CANDIDATE_VIEW_ABORT) {
                closeCamera();
                return false;
            }
            if (view_result ==
                CANDIDATE_VIEW_MISSION_COMPLETE) {
                candidate.resolved = true;
                closeCamera();
                return true;
            }

            if (view_result ==
                CANDIDATE_VIEW_RESOLVED) {
                closeCamera();
                continue;
            }

            ROS_WARN(
                "候选[%zu]在基准朝向没有得到可分类结果，"
                "启动与停靠恢复相同的左右±%.1f度扫描。",
                i + 1,
                docking_recovery_turn_deg_);

            const double recovery_yaws[2] = {
                normalizeAngle(
                    base_yaw + recovery_turn),
                normalizeAngle(
                    base_yaw - recovery_turn)
            };
            const char* recovery_names[2] = {
                "候选左侧恢复",
                "候选右侧恢复"
            };

            bool candidate_finished = false;

            for (int side = 0;
                 side < 2 && ros::ok();
                 ++side) {
                std::string action_name =
                    recovery_names[side];

                // 日志中的±30只用于默认值可读性，实际目标角严格使用
                // docking_recovery_turn_deg_，因此launch若改该参数仍生效。
                ROS_WARN(
                    "候选[%zu]开始%s："
                    "基准=%.1f度，目标=%.1f度，恢复偏角=%.1f度",
                    i + 1,
                    action_name.c_str(),
                    base_yaw * 180.0 / kPi,
                    recovery_yaws[side] *
                        180.0 / kPi,
                    side == 0
                        ? docking_recovery_turn_deg_
                        : -docking_recovery_turn_deg_);

                if (!rotateToYawWithoutScanning(
                        recovery_yaws[side],
                        action_name)) {
                    ROS_WARN(
                        "候选[%zu]的%s旋转失败，继续下一方向。",
                        i + 1,
                        action_name.c_str());
                    continue;
                }

                view_result =
                    tryCandidateAtCurrentView(
                        candidate,
                        action_name,
                        true);

                if (view_result ==
                    CANDIDATE_VIEW_ABORT) {
                    closeCamera();
                    return false;
                }

                if (view_result ==
                    CANDIDATE_VIEW_MISSION_COMPLETE) {
                    candidate.resolved = true;
                    closeCamera();
                    return true;
                }

                if (view_result ==
                    CANDIDATE_VIEW_RESOLVED) {
                    candidate_finished = true;
                    break;
                }

                if (view_result ==
                    CANDIDATE_VIEW_NOT_FOUND) {
                    ROS_WARN(
                        "候选[%zu]在%s仍未重新找到匹配目标。",
                        i + 1,
                        action_name.c_str());
                } else {
                    ROS_WARN(
                        "候选[%zu]在%s检测到了目标，"
                        "但OCR仍无法分类。",
                        i + 1,
                        action_name.c_str());
                }
            }

            if (!candidate_finished &&
                !candidate.resolved) {
                ROS_WARN(
                    "候选[%zu]基准、左侧+%.1f度、右侧-%.1f度"
                    "均未得到可分类结果，本候选本轮回访结束。",
                    i + 1,
                    docking_recovery_turn_deg_,
                    docking_recovery_turn_deg_);
            }

            closeCamera();
        }

        // 防御性：候选阶段结束时，如果现实已停靠而仿真已经记录但
        // 尚未停靠，则立即补做。
        if (real_docked_ &&
            simulation_observation_.valid &&
            !simulation_docked_) {
            if (!dockStoredTarget(
                    simulation_observation_,
                    "候选阶段已记录仿真目标")) {
                return false;
            }
            simulation_docked_ = true;
        }

        return true;
    }

    // ======================================================================
    // 底盘速度
    // ======================================================================
    void publishVelocity(
            double linear_x,
            double linear_y,
            double angular_z) {
        ucarmain2026::set_speed service;
        service.request.target_twist.linear.x =
            linear_x;
        service.request.target_twist.linear.y =
            linear_y;
        service.request.target_twist.linear.z =
            0.0;
        service.request.target_twist.angular.x =
            0.0;
        service.request.target_twist.angular.y =
            0.0;
        service.request.target_twist.angular.z =
            angular_z;
        service.request.work = true;
        service.request.movebase_flag = false;
        service.request.target_x = 0.0;
        service.request.target_y = 0.0;
        service.request.target_yaw = 0.0;

        if (!set_speed_client_.call(service) ||
            !service.response.success) {
            ROS_ERROR_THROTTLE(
                1.0,
                "调用/set_speed失败");
        }
    }

    void publishAngular(double angular_z) {
        publishVelocity(
            0.0, 0.0, angular_z);
    }

    void stopRobot() {
        if (!set_speed_client_.exists()) {
            return;
        }

        ucarmain2026::set_speed service;
        service.request.target_twist =
            geometry_msgs::Twist();
        service.request.work = true;
        service.request.movebase_flag = false;
        service.request.target_x = 0.0;
        service.request.target_y = 0.0;
        service.request.target_yaw = 0.0;

        set_speed_client_.call(service);

        // 保留现有simple_move_control已验证的三帧零速度机制。
        ros::Duration(0.12).sleep();

        service.request.work = false;
        if (!set_speed_client_.call(service)) {
            ROS_WARN_THROTTLE(
                1.0,
                "停止/set_speed控制失败");
        }
    }

    void printSummary(bool success) const {
        ROS_INFO(
            "================ 旋转找板结果 ================");
        ROS_INFO(
            "现实目标：%s；记录=%s；停靠=%s",
            categoryChinese(real_target_category_),
            real_observation_.valid ? "是" : "否",
            real_docked_ ? "成功" : "未完成");
        if (real_observation_.valid) {
            ROS_INFO(
                "  现实板=%s(%.3f,%.3f)",
                wallName(
                    real_observation_.board.wall),
                real_observation_.board.x,
                real_observation_.board.y);
        }

        ROS_INFO(
            "仿真目标：%s；记录=%s；停靠=%s",
            categoryChinese(
                simulation_target_category_),
            simulation_observation_.valid
                ? "是" : "否",
            simulation_docked_
                ? "成功" : "未完成");
        if (simulation_observation_.valid) {
            ROS_INFO(
                "  仿真板=%s(%.3f,%.3f)",
                wallName(
                    simulation_observation_.board.wall),
                simulation_observation_.board.x,
                simulation_observation_.board.y);
        }

        ROS_INFO(
            "已分类位置屏蔽数=%zu；"
            "无法分类候选数=%zu",
            seen_board_coordinates_.size(),
            candidate_locations_.size());

        for (std::size_t i = 0;
             i < candidate_locations_.size();
             ++i) {
            const CandidateLocation& c =
                candidate_locations_[i];
            if (c.board_valid) {
                ROS_INFO(
                    "  候选[%zu] %s(%.3f,%.3f)，"
                    "最近unknown视角=%.1f度，已回访=%s，已解决=%s",
                    i + 1,
                    wallName(c.board.wall),
                    c.board.x,
                    c.board.y,
                    c.have_failed_view
                        ? c.last_failed_yaw *
                              180.0 / kPi
                        : c.detected_yaw *
                              180.0 / kPi,
                    c.attempted ? "是" : "否",
                    c.resolved ? "是" : "否");
            } else {
                ROS_INFO(
                    "  候选[%zu] 原检测位姿=(%.3f,%.3f,%.1f度)，"
                    "已回访=%s，已解决=%s",
                    i + 1,
                    c.detection_pose.x,
                    c.detection_pose.y,
                    c.detection_pose.yaw *
                        180.0 / kPi,
                    c.attempted ? "是" : "否",
                    c.resolved ? "是" : "否");
            }
        }

        ROS_INFO(
            "任务总结果：%s",
            success ? "成功" : "失败");
        ROS_INFO(
            "===============================================");
    }

    // ======================================================================
    // ROS
    // ======================================================================
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    MoveBaseClient move_base_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::ServiceClient detect_client_;
    ros::ServiceClient ocr_client_;
    ros::ServiceClient set_speed_client_;

    // ======================================================================
    // 配置
    // ======================================================================
    std::string real_target_category_;
    std::string simulation_target_category_;

    std::string map_frame_;
    std::string base_frame_;

    double room_min_x_;
    double room_max_x_;
    double room_min_y_;
    double room_max_y_;
    double navigation_timeout_;

    int image_width_;
    double camera_fx_;
    double camera_yaw_offset_deg_;
    double max_detection_duration_;
    double duplicate_coordinate_distance_;
    int static_view_detection_passes_;

    int ocr_attempts_;
    double ocr_retry_interval_;

    std::string planner_private_namespace_;
    double rotation_timeout_;
    double rotation_control_rate_;

    double rotation_angular_gain_;
    double rotation_yaw_tolerance_;
    double rotation_min_angular_speed_;
    double rotation_max_angular_speed_;
    double rotation_acc_lim_theta_;

    double docking_standoff_;
    double approach_stop_distance_;
    double docking_refine_max_board_shift_;
    double docking_recovery_turn_deg_;
    int docking_recovery_detection_attempts_;
    int docking_refresh_clear_calls_;

    double candidate_match_max_shift_;
    int candidate_detection_attempts_;
    double candidate_retry_yaw_delta_deg_;
    double candidate_retry_position_delta_;

    // ======================================================================
    // 状态
    // ======================================================================
    bool configuration_valid_ = false;
    bool camera_opened_ = false;

    std::vector<ScanPoint> scan_points_;
    int current_scan_point_index_ = -1;

    std::vector<BoardBoundaryEstimate>
        seen_board_coordinates_;
    std::vector<CandidateLocation>
        candidate_locations_;

    TargetObservation real_observation_;
    TargetObservation simulation_observation_;

    bool real_docked_ = false;
    bool simulation_docked_ = false;
};


int main(int argc, char** argv) {
    setlocale(LC_ALL, "");

    ros::init(
        argc,
        argv,
        "target_scan_competition");

    TargetScanCompetition node;
    return node.run() ? 0 : 1;
}