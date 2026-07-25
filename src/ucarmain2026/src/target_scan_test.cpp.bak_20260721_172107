#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <ros_nanodet/detect_result_srv.h>
#include <ros_nanodet/ocr_result_srv.h>
#include <ucarmain2026/set_speed.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <clocale>
#include <fstream>
#include <iomanip>
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

std::string csvEscape(const std::string& value) {
    std::string escaped = "\"";
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '"') escaped.push_back('"');
        escaped.push_back(value[i]);
    }
    escaped.push_back('"');
    return escaped;
}

}  // namespace

class TargetScanTest {
public:
    TargetScanTest()
        : nh_(),
          pnh_("~"),
          move_base_("move_base", true),
          tf_listener_(tf_buffer_) {
        pnh_.param("goal_x", goal_x_, 3.75);
        pnh_.param("goal_y", goal_y_, 3.75);
        pnh_.param("goal_yaw", goal_yaw_, 0.0);
        pnh_.param("second_scan_x", second_scan_x_, 1.75);
        pnh_.param("second_scan_y", second_scan_y_, 3.25);
        pnh_.param("second_scan_yaw", second_scan_yaw_, 0.0);
        pnh_.param("navigation_timeout", navigation_timeout_, 180.0);

        pnh_.param("image_width", image_width_, 640);
        pnh_.param("scan_speed", scan_speed_, 0.80);
        pnh_.param("scan_angle_deg", scan_angle_deg_, 360.0);
        pnh_.param("post_ocr_rotate_deg", post_ocr_rotate_deg_, 20.0);
        pnh_.param("post_ocr_rotate_speed", post_ocr_rotate_speed_, 0.80);
        pnh_.param("post_ocr_rotate_min_speed", post_ocr_rotate_min_speed_, 0.15);
        pnh_.param("post_ocr_slow_angle_deg", post_ocr_slow_angle_deg_, 5.0);
        pnh_.param("post_ocr_rotate_kp", post_ocr_rotate_kp_, 5.0);
        pnh_.param("target_category", target_category_, std::string("food"));

        pnh_.param("align_kp", align_kp_, 0.0030);
        // 二代车实测 0.055 rad/s 无法克服原地旋转死区，最低提高到 0.10。
        pnh_.param("align_min_speed", align_min_speed_, 0.10);
        pnh_.param("align_max_speed", align_max_speed_, 0.35);
        // x≈301 时目测已经对准，因此中心允许误差改为 ±20 像素。
        pnh_.param("center_tolerance_px", center_tolerance_px_, 20.0);
        pnh_.param("center_stable_frames", center_stable_frames_, 3);
        pnh_.param("max_track_jump_px", max_track_jump_px_, 140.0);
        pnh_.param("max_lost_frames", max_lost_frames_, 4);
        pnh_.param("align_timeout", align_timeout_, 12.0);

        pnh_.param("ocr_attempts", ocr_attempts_, 3);
        pnh_.param("ocr_retry_interval", ocr_retry_interval_, 0.12);
        pnh_.param("max_detection_duration", max_detection_duration_, 0.50);
        pnh_.param("result_file", result_file_,
                   std::string("/home/ucar/ucar_ws_copy/src/ucarmain2026/ocr_scan_results.csv"));
        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));

        // 找板房间及栅格的map坐标。目标板贴在四条边界之一，并占据一个0.5米边长。
        pnh_.param("room_min_x", room_min_x_, 0.0);
        pnh_.param("room_max_x", room_max_x_, 5.0);
        pnh_.param("room_min_y", room_min_y_, 2.5);
        pnh_.param("room_max_y", room_max_y_, 4.5);
        pnh_.param("grid_size", grid_size_, 0.5);
        // 摄像头光轴相对base_link正前方的偏角；正值表示逆时针。
        pnh_.param("camera_yaw_offset", camera_yaw_offset_, 0.0);

        // 第二段：车头保持正对墙面，仅通过全向底盘左右平移使板位于画面中心。
        pnh_.param("lateral_align_kp", lateral_align_kp_, 0.0010);
        pnh_.param("lateral_align_min_speed", lateral_align_min_speed_, 0.06);
        pnh_.param("lateral_align_max_speed", lateral_align_max_speed_, 0.20);
        pnh_.param("lateral_center_tolerance_px", lateral_center_tolerance_px_, 10.0);
        pnh_.param("lateral_stable_frames", lateral_stable_frames_, 3);
        pnh_.param("lateral_align_timeout", lateral_align_timeout_, 15.0);

        // 第三段：取雷达正前方若干点的最小距离，直行至板前20厘米。
        pnh_.param("scan_topic", scan_topic_, std::string("/scan"));
        pnh_.param("front_lidar_half_window", front_lidar_half_window_, 3);
        pnh_.param("approach_stop_distance", approach_stop_distance_, 0.20);
        pnh_.param("approach_slow_distance", approach_slow_distance_, 0.35);
        pnh_.param("approach_min_speed", approach_min_speed_, 0.05);
        pnh_.param("approach_max_speed", approach_max_speed_, 0.12);
        pnh_.param("approach_timeout", approach_timeout_, 15.0);
        pnh_.param("lidar_data_timeout", lidar_data_timeout_, 0.50);
        pnh_.param("lidar_loss_abort_timeout", lidar_loss_abort_timeout_, 2.0);

        // 横移和前进时持续修正车头，保证朝向固定为0/90/180/-90度。
        pnh_.param("docking_yaw_kp", docking_yaw_kp_, 2.0);
        pnh_.param("docking_yaw_max_speed", docking_yaw_max_speed_, 0.25);

        scan_speed_ = std::fabs(scan_speed_);
        scan_target_rad_ = std::fabs(scan_angle_deg_) * kPi / 180.0;
        post_ocr_rotate_speed_ = std::fabs(post_ocr_rotate_speed_);
        post_ocr_rotate_min_speed_ = std::fabs(post_ocr_rotate_min_speed_);
        post_ocr_rotate_speed_ = std::max(post_ocr_rotate_speed_, post_ocr_rotate_min_speed_);
        if (max_detection_duration_ <= 0.0) {
            ROS_WARN("max_detection_duration必须大于0，恢复为0.50秒");
            max_detection_duration_ = 0.50;
        }
        post_ocr_slow_angle_deg_ = std::fabs(post_ocr_slow_angle_deg_);
        post_ocr_rotate_kp_ = std::fabs(post_ocr_rotate_kp_);
        lateral_align_min_speed_ = std::fabs(lateral_align_min_speed_);
        lateral_align_max_speed_ = std::max(
            std::fabs(lateral_align_max_speed_), lateral_align_min_speed_);
        approach_min_speed_ = std::fabs(approach_min_speed_);
        approach_max_speed_ = std::max(
            std::fabs(approach_max_speed_), approach_min_speed_);
        front_lidar_half_window_ = std::max(0, front_lidar_half_window_);
        docking_yaw_kp_ = std::fabs(docking_yaw_kp_);
        docking_yaw_max_speed_ = std::fabs(docking_yaw_max_speed_);
        configuration_valid_ = isValidCategory(target_category_) &&
                               room_max_x_ > room_min_x_ &&
                               room_max_y_ > room_min_y_ && grid_size_ > 0.0 &&
                               approach_stop_distance_ > 0.0 &&
                               approach_slow_distance_ > approach_stop_distance_ &&
                               lidar_data_timeout_ > 0.0;

        ROS_INFO("环扫参数：搜索速度=%.3f，对准速度范围=[%.3f, %.3f]，中心容差=±%.1f像素",
                 scan_speed_, align_min_speed_, align_max_speed_, center_tolerance_px_);
        ROS_INFO("识别后转动：%.1f度，最高速度=%.3f；检测结果最大允许延迟=%.2f秒；"
                 "本轮目标类别=%s",
                 post_ocr_rotate_deg_, post_ocr_rotate_speed_, max_detection_duration_,
                 target_category_.c_str());
        if (!configuration_valid_) {
            ROS_ERROR("配置无效：target_category只允许food、daily、electronic，"
                      "且房间边界和grid_size必须有效");
        }

        detect_client_ = nh_.serviceClient<ros_nanodet::detect_result_srv>("/nanodet_detect");
        ocr_client_ = nh_.serviceClient<ros_nanodet::ocr_result_srv>("/nanodet_ocr");
        set_speed_client_ = nh_.serviceClient<ucarmain2026::set_speed>("/set_speed");
        scan_subscriber_ = nh_.subscribe(scan_topic_, 1,
                                         &TargetScanTest::scanCallback, this);
    }

    ~TargetScanTest() {
        stopRobot();
        closeCamera();
        if (result_stream_.is_open()) result_stream_.close();
    }

    bool run() {
        if (!configuration_valid_) return false;
        if (!openResultFile()) return false;
        if (!waitForDependencies()) return false;
        if (!navigateToScanPoint()) return false;
        if (!openCamera()) return false;

        for (int scan_point_index = 0; scan_point_index < 2; ++scan_point_index) {
            current_scan_point_index_ = scan_point_index;
            if (scan_point_index == 1) {
                stopRobot();
                goal_x_ = second_scan_x_;
                goal_y_ = second_scan_y_;
                goal_yaw_ = second_scan_yaw_;
                ROS_INFO("第一个探测点未找到目标，前往第二个探测点");
                if (!navigateToScanPoint()) return false;
            }

            ros::Duration(0.4).sleep();
            if (!initializeAngleProgress()) return false;

            ROS_INFO("在第%d个探测点(%.2f, %.2f)开始逆时针旋转，"
                     "只处理中心点位于左半屏的文字框",
                     scan_point_index + 1, goal_x_, goal_y_);
            ros::Rate rate(15.0);

            while (ros::ok() && total_ccw_angle_ < scan_target_rad_) {
            updateAngleProgress();
            if (total_ccw_angle_ >= scan_target_rad_) break;

            // 搜索阶段始终逆时针旋转。
            publishAngular(scan_speed_);

            std::vector<Box> boxes;
            if (!detectBoxes(boxes)) {
                rate.sleep();
                continue;
            }
            updateAngleProgress();
            if (total_ccw_angle_ >= scan_target_rad_) break;

            const int target_index = chooseClosestLeftBox(boxes);
            if (target_index < 0) {
                ROS_INFO_THROTTLE(2.0, "扫描进度：%.1f / %.1f 度",
                                  total_ccw_angle_ * 180.0 / kPi, scan_angle_deg_);
                rate.sleep();
                continue;
            }

            stopRobot();
            const Box selected = boxes[static_cast<std::size_t>(target_index)];
            ROS_INFO("左半屏发现 %zu 个有效框，选择中心 x=%.1f 的目标",
                     countLeftBoxes(boxes), selected.centerX());

            Box centered_box;
            double aligned_yaw = 0.0;
            if (!alignTarget(selected, centered_box, aligned_yaw)) {
                ROS_WARN("目标在对准过程中丢失，继续逆时针搜索");
                rate.sleep();
                continue;
            }

            const OcrRecord record = recognizeCenteredTarget(centered_box);
            saveRecord(aligned_yaw, record);

            if (record.category == target_category_) {
                target_found_ = true;
                stopRobot();
                ROS_INFO("已经确认目标板，立即结束环扫并计算停靠格");

                DockingGoal docking_goal;
                if (!computeDockingGoal(docking_goal)) {
                    closeCamera();
                    printFinalSummary();
                    return false;
                }

                docking_goal_ = docking_goal;
                docking_goal_valid_ = true;
                if (!navigateToPredockingEdge(docking_goal_)) {
                    closeCamera();
                    printFinalSummary();
                    return false;
                }
                predocking_succeeded_ = true;

                if (!alignBoardLaterally(docking_goal_.goal_yaw)) {
                    closeCamera();
                    printFinalSummary();
                    return false;
                }
                lateral_alignment_succeeded_ = true;

                // 横移居中后不再使用视觉，释放摄像头再进行雷达直线逼近。
                closeCamera();
                docking_succeeded_ = approachBoardWithLidar(docking_goal_.goal_yaw);
                printFinalSummary();
                return docking_succeeded_;
            }

            // 每次读取参数，因此运行过程中 rosparam set 也能影响下一次转动。
            pnh_.getParam("post_ocr_rotate_deg", post_ocr_rotate_deg_);
            pnh_.getParam("post_ocr_rotate_speed", post_ocr_rotate_speed_);
            post_ocr_rotate_deg_ = std::max(0.0, post_ocr_rotate_deg_);
            post_ocr_rotate_speed_ = std::fabs(post_ocr_rotate_speed_);
            post_ocr_rotate_speed_ = std::max(post_ocr_rotate_speed_,
                                              post_ocr_rotate_min_speed_);
            ROS_INFO("识别完成，逆时针转动 %.2f 度，最高速度 %.2f rad/s",
                     post_ocr_rotate_deg_, post_ocr_rotate_speed_);
            rotateCounterClockwise(post_ocr_rotate_deg_ * kPi / 180.0);

            ROS_INFO("当前累计逆时针转角：%.2f 度",
                     total_ccw_angle_ * 180.0 / kPi);
                rate.sleep();
            }

            stopRobot();
            completed_scan_points_ = scan_point_index + 1;
            ROS_INFO("第%d个探测点已经完成%.1f度环扫，未找到指定目标",
                     scan_point_index + 1, scan_angle_deg_);
        }

        stopRobot();
        closeCamera();
        printFinalSummary();
        return true;
    }

private:
    struct Box {
        int class_id;
        int x0;
        int y0;
        int x1;
        int y1;

        double centerX() const { return 0.5 * static_cast<double>(x0 + x1); }
        double centerY() const { return 0.5 * static_cast<double>(y0 + y1); }
        double width() const { return std::max(1, x1 - x0); }
        double height() const { return std::max(1, y1 - y0); }
    };

    struct OcrRecord {
        bool success;
        std::string text;
        std::string category;
        double confidence;
        double detect_score;
        Box box;

        OcrRecord()
            : success(false), text("<未识别>"), category("unknown"),
              confidence(0.0), detect_score(0.0),
              box{0, 0, 0, 0, 0} {}
    };

    struct ScanRecord {
        double map_yaw;
        double accumulated_angle;
        OcrRecord ocr;
    };

    enum Wall {
        WALL_LEFT,
        WALL_RIGHT,
        WALL_BOTTOM,
        WALL_TOP
    };

    struct DockingGoal {
        Wall wall;
        double intersection_x;
        double intersection_y;
        double goal_x;
        double goal_y;
        double goal_yaw;
        int column;
        int row;

        DockingGoal()
            : wall(WALL_LEFT), intersection_x(0.0), intersection_y(0.0),
              goal_x(0.0), goal_y(0.0), goal_yaw(0.0), column(-1), row(-1) {}
    };

    static bool isValidCategory(const std::string& category) {
        return category == "food" || category == "daily" || category == "electronic";
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
        if (category == "food") return "食品";
        if (category == "daily") return "日用品";
        if (category == "electronic") return "电子产品";
        return "未知";
    }

    std::string targetStatus(const std::string& category) const {
        if (category == "unknown") return "无法判断";
        return category == target_category_ ? "是" : "否";
    }

    static const char* wallName(Wall wall) {
        if (wall == WALL_LEFT) return "左墙";
        if (wall == WALL_RIGHT) return "右墙";
        if (wall == WALL_BOTTOM) return "下墙";
        return "上墙";
    }

    bool waitForDependencies() {
        ROS_INFO("等待 move_base、NanoDet、OCR 和运动控制服务...");
        while (ros::ok() && !move_base_.waitForServer(ros::Duration(3.0))) {
            ROS_INFO("仍在等待 move_base");
        }
        if (!ros::ok()) return false;

        if (!ros::service::waitForService("/nanodet_detect", ros::Duration(20.0))) {
            ROS_ERROR("等待 /nanodet_detect 超时");
            return false;
        }
        if (!ros::service::waitForService("/nanodet_ocr", ros::Duration(20.0))) {
            ROS_ERROR("等待 /nanodet_ocr 超时");
            return false;
        }
        if (!ros::service::waitForService("/set_speed", ros::Duration(20.0))) {
            ROS_ERROR("等待 /set_speed 超时，请确认 simple_move_control 节点已经启动");
            return false;
        }
        return true;
    }

    bool navigateToScanPoint() {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_x_;
        goal.target_pose.pose.position.y = goal_y_;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, goal_yaw_);
        goal.target_pose.pose.orientation.x = quaternion.x();
        goal.target_pose.pose.orientation.y = quaternion.y();
        goal.target_pose.pose.orientation.z = quaternion.z();
        goal.target_pose.pose.orientation.w = quaternion.w();

        ROS_INFO("前往扫描点 (%.2f, %.2f)，目标朝向 %.2f 度",
                 goal_x_, goal_y_, goal_yaw_ * 180.0 / kPi);
        move_base_.sendGoal(goal);

        if (!move_base_.waitForResult(ros::Duration(navigation_timeout_))) {
            move_base_.cancelGoal();
            ROS_ERROR("前往扫描点超时");
            return false;
        }
        if (move_base_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ERROR("未能到达扫描点：%s", move_base_.getState().toString().c_str());
            return false;
        }

        move_base_.cancelAllGoals();
        stopRobot();
        ros::Duration(0.5).sleep();
        ROS_INFO("已经到达扫描点");
        return true;
    }

    bool getRobotYaw(double& yaw) {
        try {
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                           ros::Duration(0.25));
            yaw = tf2::getYaw(transform.transform.rotation);
            return true;
        } catch (const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(1.0, "读取机器人朝向失败：%s", error.what());
            return false;
        }
    }

    bool getRobotPose(double& x, double& y, double& yaw) {
        try {
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                           ros::Duration(0.25));
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            yaw = tf2::getYaw(transform.transform.rotation);
            return true;
        } catch (const tf2::TransformException& error) {
            ROS_ERROR("读取机器人map位姿失败：%s", error.what());
            return false;
        }
    }

    bool computeDockingGoal(DockingGoal& goal) {
        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;
        if (!getRobotPose(robot_x, robot_y, robot_yaw)) return false;

        const double position_tolerance = 0.10;
        if (robot_x < room_min_x_ - position_tolerance ||
            robot_x > room_max_x_ + position_tolerance ||
            robot_y < room_min_y_ - position_tolerance ||
            robot_y > room_max_y_ + position_tolerance) {
            ROS_ERROR("当前位姿(%.3f, %.3f)不在找板房间[%.2f, %.2f]x[%.2f, %.2f]内",
                      robot_x, robot_y, room_min_x_, room_max_x_,
                      room_min_y_, room_max_y_);
            return false;
        }

        const double ray_yaw = normalizeAngle(robot_yaw + camera_yaw_offset_);
        const double direction_x = std::cos(ray_yaw);
        const double direction_y = std::sin(ray_yaw);
        const double epsilon = 1e-8;
        const double boundary_tolerance = 1e-6;
        double best_t = std::numeric_limits<double>::infinity();
        Wall best_wall = WALL_LEFT;
        double best_x = 0.0;
        double best_y = 0.0;

        // 只接受射线正方向上的交点，并选择距离机器人最近的一条房间边界。
        if (std::fabs(direction_x) > epsilon) {
            const double left_t = (room_min_x_ - robot_x) / direction_x;
            const double left_y = robot_y + left_t * direction_y;
            if (left_t > epsilon &&
                left_y >= room_min_y_ - boundary_tolerance &&
                left_y <= room_max_y_ + boundary_tolerance && left_t < best_t) {
                best_t = left_t;
                best_wall = WALL_LEFT;
                best_x = room_min_x_;
                best_y = clampValue(left_y, room_min_y_, room_max_y_);
            }

            const double right_t = (room_max_x_ - robot_x) / direction_x;
            const double right_y = robot_y + right_t * direction_y;
            if (right_t > epsilon &&
                right_y >= room_min_y_ - boundary_tolerance &&
                right_y <= room_max_y_ + boundary_tolerance && right_t < best_t) {
                best_t = right_t;
                best_wall = WALL_RIGHT;
                best_x = room_max_x_;
                best_y = clampValue(right_y, room_min_y_, room_max_y_);
            }
        }

        if (std::fabs(direction_y) > epsilon) {
            const double bottom_t = (room_min_y_ - robot_y) / direction_y;
            const double bottom_x = robot_x + bottom_t * direction_x;
            if (bottom_t > epsilon &&
                bottom_x >= room_min_x_ - boundary_tolerance &&
                bottom_x <= room_max_x_ + boundary_tolerance && bottom_t < best_t) {
                best_t = bottom_t;
                best_wall = WALL_BOTTOM;
                best_x = clampValue(bottom_x, room_min_x_, room_max_x_);
                best_y = room_min_y_;
            }

            const double top_t = (room_max_y_ - robot_y) / direction_y;
            const double top_x = robot_x + top_t * direction_x;
            if (top_t > epsilon &&
                top_x >= room_min_x_ - boundary_tolerance &&
                top_x <= room_max_x_ + boundary_tolerance && top_t < best_t) {
                best_t = top_t;
                best_wall = WALL_TOP;
                best_x = clampValue(top_x, room_min_x_, room_max_x_);
                best_y = room_max_y_;
            }
        }

        if (!std::isfinite(best_t)) {
            ROS_ERROR("无法计算当前朝向与找板房间边界的交点");
            return false;
        }

        const int column_count = std::max(
            1, static_cast<int>(std::lround((room_max_x_ - room_min_x_) / grid_size_)));
        const int row_count = std::max(
            1, static_cast<int>(std::lround((room_max_y_ - room_min_y_) / grid_size_)));

        goal.wall = best_wall;
        goal.intersection_x = best_x;
        goal.intersection_y = best_y;

        if (best_wall == WALL_LEFT || best_wall == WALL_RIGHT) {
            int row = static_cast<int>(std::floor((best_y - room_min_y_) / grid_size_));
            row = std::max(0, std::min(row, row_count - 1));
            goal.row = row;
            goal.column = best_wall == WALL_LEFT ? 0 : column_count - 1;
            goal.goal_x = best_wall == WALL_LEFT
                              ? room_min_x_ + grid_size_
                              : room_max_x_ - grid_size_;
            goal.goal_y = room_min_y_ + (static_cast<double>(row) + 0.5) * grid_size_;
            goal.goal_yaw = best_wall == WALL_LEFT ? kPi : 0.0;
        } else {
            int column = static_cast<int>(std::floor((best_x - room_min_x_) / grid_size_));
            column = std::max(0, std::min(column, column_count - 1));
            goal.column = column;
            goal.row = best_wall == WALL_BOTTOM ? 0 : row_count - 1;
            goal.goal_x = room_min_x_ + (static_cast<double>(column) + 0.5) * grid_size_;
            goal.goal_y = best_wall == WALL_BOTTOM
                              ? room_min_y_ + grid_size_
                              : room_max_y_ - grid_size_;
            goal.goal_yaw = best_wall == WALL_BOTTOM ? -0.5 * kPi : 0.5 * kPi;
        }

        ROS_INFO("目标射线：机器人(%.3f, %.3f, %.2f度)，交于%s(%.3f, %.3f)",
                 robot_x, robot_y, ray_yaw * 180.0 / kPi,
                 wallName(goal.wall), goal.intersection_x, goal.intersection_y);
        ROS_INFO("目标板归入第%d列、第%d行（均从左下角开始计数），"
                 "预停靠位姿=(%.3f, %.3f, %.2f度)",
                 goal.column + 1, goal.row + 1, goal.goal_x, goal.goal_y,
                 goal.goal_yaw * 180.0 / kPi);
        return true;
    }

    bool navigateToPredockingEdge(const DockingGoal& docking_goal) {
        stopRobot();
        // 等simple_move_control完成最后三帧零速度，避免与move_base最初的速度争用。
        ros::Duration(0.20).sleep();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = docking_goal.goal_x;
        goal.target_pose.pose.position.y = docking_goal.goal_y;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, docking_goal.goal_yaw);
        goal.target_pose.pose.orientation.x = quaternion.x();
        goal.target_pose.pose.orientation.y = quaternion.y();
        goal.target_pose.pose.orientation.z = quaternion.z();
        goal.target_pose.pose.orientation.w = quaternion.w();

        ROS_INFO("第一段预停靠：前往%s前方0.5米处(%.3f, %.3f)，朝向固定%.2f度",
                 wallName(docking_goal.wall), docking_goal.goal_x,
                 docking_goal.goal_y, docking_goal.goal_yaw * 180.0 / kPi);
        move_base_.sendGoal(goal);

        if (!move_base_.waitForResult(ros::Duration(navigation_timeout_))) {
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("前往目标板预停靠点超时");
            return false;
        }
        if (move_base_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            const std::string state = move_base_.getState().toString();
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("未能到达目标板预停靠点：%s", state.c_str());
            return false;
        }

        move_base_.cancelAllGoals();
        stopRobot();
        ROS_INFO("第一段完成：已到达第%d列、第%d行目标格的外侧边中点",
                 docking_goal.column + 1, docking_goal.row + 1);
        return true;
    }

    double yawHoldCommand(double desired_yaw, bool& pose_valid) {
        double current_yaw = 0.0;
        pose_valid = getRobotYaw(current_yaw);
        if (!pose_valid) return 0.0;
        const double error = normalizeAngle(desired_yaw - current_yaw);
        return clampValue(docking_yaw_kp_ * error,
                          -docking_yaw_max_speed_, docking_yaw_max_speed_);
    }

    int chooseClosestCenterBox(const std::vector<Box>& boxes) const {
        const double image_center = 0.5 * static_cast<double>(image_width_);
        int selected = -1;
        double smallest_error = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double error = std::fabs(image_center - boxes[i].centerX());
            if (error < smallest_error) {
                smallest_error = error;
                selected = static_cast<int>(i);
            }
        }
        return selected;
    }

    bool alignBoardLaterally(double desired_yaw) {
        ROS_INFO("第二段横移居中：保持朝向%.2f度，使目标板进入画面正中心",
                 desired_yaw * 180.0 / kPi);
        const double image_center = 0.5 * static_cast<double>(image_width_);
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(lateral_align_timeout_);
        int stable_frames = 0;
        int lost_frames = 0;
        bool have_tracked_box = false;
        Box tracked_box{0, 0, 0, 0, 0};
        ros::Rate rate(15.0);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            std::vector<Box> boxes;
            int selected = -1;
            if (detectBoxes(boxes)) {
                selected = have_tracked_box
                               ? associateSelectedBox(boxes, tracked_box)
                               : chooseClosestCenterBox(boxes);
            }

            bool yaw_valid = false;
            const double angular_z = yawHoldCommand(desired_yaw, yaw_valid);
            if (!yaw_valid) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            if (selected < 0) {
                ++lost_frames;
                stable_frames = 0;
                // 丢失视觉时禁止盲目横移，只允许小角度朝向修正。
                publishVelocity(0.0, 0.0, angular_z);
                if (lost_frames > max_lost_frames_) {
                    stopRobot();
                    ROS_ERROR("横移居中时目标板连续丢失");
                    return false;
                }
                rate.sleep();
                continue;
            }

            tracked_box = boxes[static_cast<std::size_t>(selected)];
            have_tracked_box = true;
            lost_frames = 0;
            const double pixel_error = image_center - tracked_box.centerX();
            const double yaw_error = std::fabs(angular_z) /
                                     std::max(1e-6, docking_yaw_kp_);

            if (std::fabs(pixel_error) <= lateral_center_tolerance_px_) {
                publishVelocity(0.0, 0.0, angular_z);
                if (yaw_error <= 2.0 * kPi / 180.0) {
                    ++stable_frames;
                } else {
                    stable_frames = 0;
                }
                if (stable_frames >= lateral_stable_frames_) {
                    stopRobot();
                    ROS_INFO("第二段完成：目标中心x=%.1f，连续%d帧位于中心容差内",
                             tracked_box.centerX(), stable_frames);
                    return true;
                }
            } else {
                stable_frames = 0;
                double lateral_y = clampValue(
                    lateral_align_kp_ * pixel_error,
                    -lateral_align_max_speed_, lateral_align_max_speed_);
                if (std::fabs(lateral_y) < lateral_align_min_speed_) {
                    lateral_y = lateral_y >= 0.0
                                    ? lateral_align_min_speed_
                                    : -lateral_align_min_speed_;
                }
                // base_link中linear.y正方向为车体左侧；目标在画面左侧时向左平移。
                publishVelocity(0.0, lateral_y, angular_z);
                ROS_INFO_THROTTLE(0.5,
                                  "横移居中：目标x=%.1f，像素误差=%.1f，"
                                  "linear.y=%.3f，angular.z=%.3f",
                                  tracked_box.centerX(), pixel_error,
                                  lateral_y, angular_z);
            }
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("横移居中超时");
        return false;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message) {
        latest_scan_ = *message;
        have_laser_scan_ = true;
        latest_scan_wall_time_ = ros::WallTime::now();
    }

    bool getFrontMinimumDistance(double& minimum_distance) {
        if (!have_laser_scan_) return false;
        if ((ros::WallTime::now() - latest_scan_wall_time_).toSec() >
            lidar_data_timeout_) {
            return false;
        }
        if (latest_scan_.ranges.empty() ||
            std::fabs(latest_scan_.angle_increment) < 1e-12) {
            return false;
        }

        int center_index = static_cast<int>(std::lround(
            (0.0 - latest_scan_.angle_min) / latest_scan_.angle_increment));
        center_index = std::max(
            0, std::min(center_index,
                        static_cast<int>(latest_scan_.ranges.size()) - 1));

        if (!lidar_layout_logged_) {
            ROS_INFO("雷达ranges数量=%zu，理论正前方索引=%d，使用索引[%d, %d]",
                     latest_scan_.ranges.size(), center_index,
                     std::max(0, center_index - front_lidar_half_window_),
                     std::min(static_cast<int>(latest_scan_.ranges.size()) - 1,
                              center_index + front_lidar_half_window_));
            if (latest_scan_.ranges.size() != 909) {
                ROS_WARN("当前雷达点数不是预期的909，将继续根据angle_min和"
                         "angle_increment自动计算正前方索引");
            }
            lidar_layout_logged_ = true;
        }

        const int first = std::max(0, center_index - front_lidar_half_window_);
        const int last = std::min(
            static_cast<int>(latest_scan_.ranges.size()) - 1,
            center_index + front_lidar_half_window_);
        minimum_distance = std::numeric_limits<double>::infinity();
        int valid_count = 0;
        for (int i = first; i <= last; ++i) {
            const double range = latest_scan_.ranges[static_cast<std::size_t>(i)];
            if (!std::isfinite(range) || range < latest_scan_.range_min ||
                range > latest_scan_.range_max || range <= 0.0) {
                continue;
            }
            minimum_distance = std::min(minimum_distance, range);
            ++valid_count;
        }
        return valid_count > 0;
    }

    bool approachBoardWithLidar(double desired_yaw) {
        ROS_INFO("第三段雷达逼近：正前方最小距离达到%.3f米时停车",
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
                if ((ros::WallTime::now() - last_valid_lidar).toSec() >
                    lidar_loss_abort_timeout_) {
                    stopRobot();
                    ROS_ERROR("连续%.2f秒没有有效的正前方雷达数据，终止逼近",
                              lidar_loss_abort_timeout_);
                    return false;
                }
                ROS_WARN_THROTTLE(1.0, "等待有效且新鲜的正前方雷达数据");
                rate.sleep();
                continue;
            }
            last_valid_lidar = ros::WallTime::now();

            if (front_distance <= approach_stop_distance_) {
                stopRobot();
                final_front_distance_ = front_distance;
                ROS_INFO("第三段完成：正前方最小距离=%.3f米，停靠成功",
                         front_distance);
                return true;
            }

            bool yaw_valid = false;
            const double angular_z = yawHoldCommand(desired_yaw, yaw_valid);
            if (!yaw_valid) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            double forward_speed = approach_max_speed_;
            if (front_distance < approach_slow_distance_) {
                const double ratio = clampValue(
                    (front_distance - approach_stop_distance_) /
                        (approach_slow_distance_ - approach_stop_distance_),
                    0.0, 1.0);
                forward_speed = approach_min_speed_ +
                    ratio * (approach_max_speed_ - approach_min_speed_);
            }
            publishVelocity(forward_speed, 0.0, angular_z);
            ROS_INFO_THROTTLE(0.5,
                              "雷达逼近：正前方最小距离=%.3f米，"
                              "linear.x=%.3f，angular.z=%.3f",
                              front_distance, forward_speed, angular_z);
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("雷达直线逼近超时");
        return false;
    }

    bool initializeAngleProgress() {
        double yaw = 0.0;
        if (!getRobotYaw(yaw)) {
            ROS_ERROR("无法取得扫描起始角度");
            return false;
        }
        last_yaw_ = yaw;
        start_yaw_ = yaw;
        total_ccw_angle_ = 0.0;
        have_last_yaw_ = true;
        ROS_INFO("扫描起始 map yaw：%.2f 度", start_yaw_ * 180.0 / kPi);
        return true;
    }

    bool updateAngleProgress() {
        double yaw = 0.0;
        if (!getRobotYaw(yaw)) return false;
        if (!have_last_yaw_) {
            last_yaw_ = yaw;
            have_last_yaw_ = true;
            return true;
        }

        const double delta = normalizeAngle(yaw - last_yaw_);
        // 只累计逆时针的正增量。对准过冲后的少量顺时针修正不倒扣扫描进度。
        if (delta > 0.0 && delta < 0.5) total_ccw_angle_ += delta;
        last_yaw_ = yaw;
        return true;
    }

    bool openCamera() {
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -1;
        if (!detect_client_.call(service)) {
            ROS_ERROR("打开 NanoDet 摄像头失败");
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
        // 用墙上时钟判断真实阻塞时长，不受 /use_sim_time 或ROS时间跳变影响。
        const ros::WallTime begin = ros::WallTime::now();
        if (!detect_client_.call(service)) {
            ROS_WARN_THROTTLE(1.0, "调用 /nanodet_detect 失败");
            return false;
        }

        const std::size_t count = std::min(
            std::min(service.response.x0.size(), service.response.y0.size()),
            std::min(service.response.x1.size(), service.response.y1.size()));

        for (std::size_t i = 0; i < count; ++i) {
            Box box;
            box.class_id = i < service.response.class_name.size()
                               ? service.response.class_name[i]
                               : 0;
            box.x0 = service.response.x0[i];
            box.y0 = service.response.y0[i];
            box.x1 = service.response.x1[i];
            box.y1 = service.response.y1[i];
            if (box.x1 > box.x0 && box.y1 > box.y0) boxes.push_back(box);
        }

        const double elapsed = (ros::WallTime::now() - begin).toSec();
        if (elapsed > max_detection_duration_) {
            boxes.clear();
            ROS_ERROR("NanoDet 单帧耗时 %.3f 秒，超过 %.3f 秒，结果可能已经过期，已丢弃",
                      elapsed, max_detection_duration_);
            return false;
        }
        if (elapsed > 0.20) ROS_WARN("NanoDet 单帧耗时 %.3f 秒", elapsed);
        return true;
    }

    std::size_t countLeftBoxes(const std::vector<Box>& boxes) const {
        const double image_center = 0.5 * static_cast<double>(image_width_);
        std::size_t count = 0;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() < image_center) ++count;
        }
        return count;
    }

    int chooseClosestLeftBox(const std::vector<Box>& boxes) const {
        const double image_center = 0.5 * static_cast<double>(image_width_);
        int selected = -1;
        double largest_center_x = -std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double center_x = boxes[i].centerX();
            // 右半屏及恰好位于中心线右侧的框全部忽略。
            if (center_x >= image_center) continue;
            // 左半屏多个目标时，中心 x 最大的就是离画面中心最近的目标。
            if (center_x > largest_center_x) {
                largest_center_x = center_x;
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
            static_cast<double>(std::max(0, right - left) * std::max(0, bottom - top));
        const double first_area = first.width() * first.height();
        const double second_area = second.width() * second.height();
        const double union_area = first_area + second_area - intersection;
        return union_area > 0.0 ? intersection / union_area : 0.0;
    }

    int associateSelectedBox(const std::vector<Box>& boxes, const Box& previous) const {
        int best_iou_index = -1;
        double best_iou = 0.0;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double iou = intersectionOverUnion(boxes[i], previous);
            if (iou > best_iou) {
                best_iou = iou;
                best_iou_index = static_cast<int>(i);
            }
        }
        if (best_iou_index >= 0 && best_iou >= 0.05) return best_iou_index;

        int nearest_index = -1;
        double nearest_distance = max_track_jump_px_;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double dx = boxes[i].centerX() - previous.centerX();
            const double dy = boxes[i].centerY() - previous.centerY();
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_index = static_cast<int>(i);
            }
        }
        return nearest_index;
    }

    bool alignTarget(Box tracked_box, Box& centered_box, double& aligned_yaw) {
        const ros::Time deadline = ros::Time::now() + ros::Duration(align_timeout_);
        const double image_center = 0.5 * static_cast<double>(image_width_);
        int stable_frames = 0;
        int lost_frames = 0;
        ros::Rate rate(20.0);

        while (ros::ok() && ros::Time::now() < deadline) {
            updateAngleProgress();

            std::vector<Box> boxes;
            if (!detectBoxes(boxes)) {
                ++lost_frames;
            } else {
                const int index = associateSelectedBox(boxes, tracked_box);
                if (index < 0) {
                    ++lost_frames;
                } else {
                    tracked_box = boxes[static_cast<std::size_t>(index)];
                    lost_frames = 0;

                    const double error = image_center - tracked_box.centerX();
                    if (std::fabs(error) <= center_tolerance_px_) {
                        stopRobot();
                        ++stable_frames;
                        if (stable_frames >= center_stable_frames_) {
                            centered_box = tracked_box;
                            if (!getRobotYaw(aligned_yaw)) return false;
                            updateAngleProgress();
                            ROS_INFO("目标已经对准：中心 x=%.1f，map yaw=%.2f 度",
                                     tracked_box.centerX(), aligned_yaw * 180.0 / kPi);
                            return true;
                        }
                    } else {
                        stable_frames = 0;
                        double angular_speed = clampValue(
                            align_kp_ * error, -align_max_speed_, align_max_speed_);
                        if (std::fabs(angular_speed) < align_min_speed_) {
                            angular_speed = angular_speed >= 0.0
                                                ? align_min_speed_
                                                : -align_min_speed_;
                        }
                        ROS_INFO("对准中：中心 x=%.1f，误差=%.1f，持续角速度=%.3f",
                                 tracked_box.centerX(), error, angular_speed);
                        publishAngular(angular_speed);
                    }
                }
            }

            if (lost_frames > max_lost_frames_) {
                stopRobot();
                return false;
            }
            rate.sleep();
        }

        stopRobot();
        ROS_WARN("目标对准超时");
        return false;
    }

    OcrRecord recognizeCenteredTarget(const Box& centered_box) {
        OcrRecord best_any;
        OcrRecord best_with_keyword;
        best_any.box = centered_box;
        best_with_keyword.box = centered_box;
        bool have_any_text = false;
        bool have_keyword_text = false;
        const double image_center = 0.5 * static_cast<double>(image_width_);

        // 清掉旋转对准时积压的旧帧。
        ros_nanodet::ocr_result_srv clear_service;
        clear_service.request.command = -3;
        if (!ocr_client_.call(clear_service)) {
            ROS_WARN("OCR 缓冲帧清理失败，将继续识别");
        }

        for (int attempt = 0; attempt < ocr_attempts_ && ros::ok(); ++attempt) {
            ros_nanodet::ocr_result_srv service;
            service.request.command = 1;
            const ros::Time begin = ros::Time::now();
            if (!ocr_client_.call(service)) {
                ROS_WARN("第 %d 次 OCR 服务调用失败", attempt + 1);
                ros::Duration(ocr_retry_interval_).sleep();
                continue;
            }

            const std::size_t count = std::min(
                std::min(service.response.text.size(), service.response.confidence.size()),
                std::min(
                    std::min(service.response.x0.size(), service.response.y0.size()),
                    std::min(service.response.x1.size(), service.response.y1.size())));

            int selected = -1;
            double smallest_center_error = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < count; ++i) {
                const double center_x = 0.5 *
                    static_cast<double>(service.response.x0[i] + service.response.x1[i]);
                const double error = std::fabs(center_x - image_center);
                if (error < smallest_center_error) {
                    smallest_center_error = error;
                    selected = static_cast<int>(i);
                }
            }

            if (selected >= 0) {
                const std::size_t i = static_cast<std::size_t>(selected);
                const std::string& text = service.response.text[i];
                const double confidence = service.response.confidence[i];
                ROS_INFO("OCR 第 %d/%d 帧：%s，置信度 %.3f，耗时 %.3f 秒",
                         attempt + 1, ocr_attempts_, text.c_str(), confidence,
                         (ros::Time::now() - begin).toSec());

                if (!text.empty()) {
                    OcrRecord candidate;
                    candidate.success = service.response.success;
                    candidate.text = text;
                    candidate.category = classifyText(text);
                    candidate.confidence = confidence;
                    candidate.detect_score = i < service.response.detect_score.size()
                                                 ? service.response.detect_score[i]
                                                 : 0.0;
                    candidate.box.class_id = 0;
                    candidate.box.x0 = service.response.x0[i];
                    candidate.box.y0 = service.response.y0[i];
                    candidate.box.x1 = service.response.x1[i];
                    candidate.box.y1 = service.response.y1[i];

                    if (!have_any_text || confidence > best_any.confidence) {
                        have_any_text = true;
                        best_any = candidate;
                    }
                    // 多帧中只要出现可分类关键词，就优先采用关键词完整的一帧；
                    // 多个有效结果之间再选择置信度最高者。
                    if (candidate.category != "unknown" &&
                        (!have_keyword_text || confidence > best_with_keyword.confidence)) {
                        have_keyword_text = true;
                        best_with_keyword = candidate;
                    }
                }
            } else {
                ROS_WARN("第 %d 次 OCR 没有返回文字框", attempt + 1);
            }

            ros::Duration(ocr_retry_interval_).sleep();
        }

        OcrRecord result;
        if (have_keyword_text) {
            result = best_with_keyword;
        } else if (have_any_text) {
            result = best_any;
        } else {
            result.success = false;
            result.text = "<未识别>";
            result.category = "unknown";
            result.box = centered_box;
        }

        ROS_INFO("OCR最终结果：%s；识别类别=%s（%s）；是否为本轮目标=%s",
                 result.text.c_str(), result.category.c_str(),
                 categoryChinese(result.category), targetStatus(result.category).c_str());
        return result;
    }

    bool rotateCounterClockwise(double requested_angle) {
        if (requested_angle <= 0.0) return true;

        double previous_yaw = 0.0;
        if (!getRobotYaw(previous_yaw)) return false;
        updateAngleProgress();

        double turned = 0.0;
        const ros::Time deadline = ros::Time::now() + ros::Duration(15.0);
        ros::Rate rate(30.0);

        while (ros::ok() && turned < requested_angle && ros::Time::now() < deadline) {
            double current_yaw = 0.0;
            if (!getRobotYaw(current_yaw)) {
                rate.sleep();
                continue;
            }

            const double delta = normalizeAngle(current_yaw - previous_yaw);
            if (delta > 0.0 && delta < 0.5) turned += delta;
            previous_yaw = current_yaw;
            updateAngleProgress();

            const double remaining = requested_angle - turned;
            if (remaining <= 0.0) break;
            const double slow_angle = post_ocr_slow_angle_deg_ * kPi / 180.0;
            double speed = post_ocr_rotate_speed_;
            if (remaining <= slow_angle) {
                speed = clampValue(post_ocr_rotate_kp_ * remaining,
                                   post_ocr_rotate_min_speed_,
                                   post_ocr_rotate_speed_);
            }
            publishAngular(speed);
            rate.sleep();
        }

        stopRobot();
        updateAngleProgress();
        if (turned + 0.015 < requested_angle) {
            ROS_WARN("固定转角未完全达到：要求 %.2f 度，实际 %.2f 度",
                     requested_angle * 180.0 / kPi, turned * 180.0 / kPi);
            return false;
        }
        return true;
    }

    bool openResultFile() {
        result_stream_.open(result_file_.c_str(), std::ios::out | std::ios::trunc);
        if (!result_stream_.is_open()) {
            ROS_ERROR("无法创建结果文件：%s", result_file_.c_str());
            return false;
        }
        result_stream_ << "index,map_yaw_deg,map_yaw_rad,accumulated_ccw_deg,text,"
                          "recognized_category,target_category,target_status,"
                          "ocr_confidence,detect_score,x0,y0,x1,y1\n";
        result_stream_.flush();
        return true;
    }

    void saveRecord(double aligned_yaw, const OcrRecord& ocr) {
        ScanRecord record;
        record.map_yaw = normalizeAngle(aligned_yaw);
        record.accumulated_angle = total_ccw_angle_;
        record.ocr = ocr;
        records_.push_back(record);

        result_stream_ << std::fixed << std::setprecision(6)
                       << records_.size() << ','
                       << record.map_yaw * 180.0 / kPi << ','
                       << record.map_yaw << ','
                       << record.accumulated_angle * 180.0 / kPi << ','
                       << csvEscape(record.ocr.text) << ','
                       << record.ocr.category << ','
                       << target_category_ << ','
                       << csvEscape(targetStatus(record.ocr.category)) << ','
                       << record.ocr.confidence << ','
                       << record.ocr.detect_score << ','
                       << record.ocr.box.x0 << ',' << record.ocr.box.y0 << ','
                       << record.ocr.box.x1 << ',' << record.ocr.box.y1 << '\n';
        result_stream_.flush();

        ROS_INFO("记录完成：map yaw=%.2f 度，文字=%s，类别=%s，是否目标=%s",
                 record.map_yaw * 180.0 / kPi, record.ocr.text.c_str(),
                 record.ocr.category.c_str(), targetStatus(record.ocr.category).c_str());
    }

    void printFinalSummary() const {
        ROS_INFO("================ 环扫识别结果 ================");
        ROS_INFO("当前探测点起始 map yaw：%.2f 度；累计逆时针转角：%.2f 度",
                 start_yaw_ * 180.0 / kPi, total_ccw_angle_ * 180.0 / kPi);
        ROS_INFO("本轮指定目标：%s（%s）",
                 target_category_.c_str(), categoryChinese(target_category_));
        if (records_.empty()) {
            ROS_WARN("本轮没有成功对准任何文字框");
        }
        for (std::size_t i = 0; i < records_.size(); ++i) {
            const ScanRecord& record = records_[i];
            ROS_INFO("[%zu] 对准角度：%.2f 度（%.5f rad）；文字：%s；"
                     "类别：%s（%s）；是否目标：%s；OCR置信度：%.3f",
                     i + 1, record.map_yaw * 180.0 / kPi, record.map_yaw,
                     record.ocr.text.c_str(), record.ocr.category.c_str(),
                     categoryChinese(record.ocr.category),
                     targetStatus(record.ocr.category).c_str(), record.ocr.confidence);
        }
        if (target_found_) {
            ROS_INFO("已经在第%d个探测点找到指定目标板，环扫提前结束",
                     current_scan_point_index_ + 1);
        } else {
            ROS_INFO("已完成%d个探测点的环扫", completed_scan_points_);
        }
        if (docking_goal_valid_) {
            ROS_INFO("目标边界：%s，射线交点=(%.3f, %.3f)",
                     wallName(docking_goal_.wall), docking_goal_.intersection_x,
                     docking_goal_.intersection_y);
            ROS_INFO("目标格：第%d列、第%d行；预停靠点=(%.3f, %.3f)",
                     docking_goal_.column + 1, docking_goal_.row + 1,
                     docking_goal_.goal_x, docking_goal_.goal_y);
            ROS_INFO("三段式停靠：预停靠导航=%s，横移居中=%s，雷达逼近=%s",
                     predocking_succeeded_ ? "成功" : "失败",
                     lateral_alignment_succeeded_ ? "成功" : "失败",
                     docking_succeeded_ ? "成功" : "失败");
            if (docking_succeeded_) {
                ROS_INFO("最终正前方最小雷达距离：%.3f米", final_front_distance_);
            }
        }
        ROS_INFO("结果文件：%s", result_file_.c_str());
        ROS_INFO("================================================");
    }

    void publishVelocity(double linear_x, double linear_y, double angular_z) {
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
        if (!set_speed_client_.call(service) || !service.response.success) {
            ROS_ERROR_THROTTLE(1.0, "调用 /set_speed 设置底盘速度失败");
        }
    }

    void publishAngular(double angular_z) {
        publishVelocity(0.0, 0.0, angular_z);
    }

    void stopRobot() {
        if (!set_speed_client_.exists()) return;

        // 先让 simple_move_control 以 20 Hz 连续发布零速度，再停止其控制循环。
        ucarmain2026::set_speed zero_service;
        zero_service.request.target_twist = geometry_msgs::Twist();
        zero_service.request.work = true;
        zero_service.request.movebase_flag = false;
        zero_service.request.target_x = 0.0;
        zero_service.request.target_y = 0.0;
        zero_service.request.target_yaw = 0.0;
        set_speed_client_.call(zero_service);
        ros::Duration(0.12).sleep();

        zero_service.request.work = false;
        if (!set_speed_client_.call(zero_service)) {
            ROS_WARN_THROTTLE(1.0, "停止 /set_speed 运动控制失败");
        }
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

    std::ofstream result_stream_;
    std::vector<ScanRecord> records_;
    bool camera_opened_ = false;
    bool have_last_yaw_ = false;

    double goal_x_;
    double goal_y_;
    double goal_yaw_;
    double second_scan_x_;
    double second_scan_y_;
    double second_scan_yaw_;
    double navigation_timeout_;
    int image_width_;
    double scan_speed_;
    double scan_angle_deg_;
    double scan_target_rad_;
    double post_ocr_rotate_deg_;
    double post_ocr_rotate_speed_;
    double post_ocr_rotate_min_speed_;
    double post_ocr_slow_angle_deg_;
    double post_ocr_rotate_kp_;
    double align_kp_;
    double align_min_speed_;
    double align_max_speed_;
    double center_tolerance_px_;
    int center_stable_frames_;
    double max_track_jump_px_;
    int max_lost_frames_;
    double align_timeout_;
    int ocr_attempts_;
    double ocr_retry_interval_;
    double max_detection_duration_;
    std::string result_file_;
    std::string map_frame_;
    std::string base_frame_;
    std::string target_category_;
    double room_min_x_;
    double room_max_x_;
    double room_min_y_;
    double room_max_y_;
    double grid_size_;
    double camera_yaw_offset_;
    double lateral_align_kp_;
    double lateral_align_min_speed_;
    double lateral_align_max_speed_;
    double lateral_center_tolerance_px_;
    int lateral_stable_frames_;
    double lateral_align_timeout_;
    std::string scan_topic_;
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
    bool configuration_valid_ = false;
    bool target_found_ = false;
    bool docking_goal_valid_ = false;
    bool predocking_succeeded_ = false;
    bool lateral_alignment_succeeded_ = false;
    bool docking_succeeded_ = false;
    DockingGoal docking_goal_;
    sensor_msgs::LaserScan latest_scan_;
    ros::WallTime latest_scan_wall_time_;
    bool have_laser_scan_ = false;
    bool lidar_layout_logged_ = false;
    double final_front_distance_ = -1.0;
    int current_scan_point_index_ = 0;
    int completed_scan_points_ = 0;

    double start_yaw_ = 0.0;
    double last_yaw_ = 0.0;
    double total_ccw_angle_ = 0.0;
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "target_scan_test");
    TargetScanTest test;
    return test.run() ? 0 : 1;
}