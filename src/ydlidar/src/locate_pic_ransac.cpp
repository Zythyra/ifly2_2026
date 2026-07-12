#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <deque>
#include <numeric>
#include <algorithm>

// 这个才是我们真正的比赛代码！！！！！！


class LaserScanPr {
public:
    LaserScanPr();
    void laserFilter(const sensor_msgs::LaserScanConstPtr &laserRaw);
    void laserDiffusion(sensor_msgs::LaserScan &scanIn);
    void publishGoal();

private:
    void toCartesian(float range, float angle, float& x, float& y) {
        x = range * std::cos(angle);
        y = range * std::sin(angle);
    }
    
    bool isSegmentLinear(const sensor_msgs::LaserScan &scan, int startIndex, int endIndex);

    ros::NodeHandle nh;                  
    ros::NodeHandle private_nh;          
    ros::Subscriber laser_sub;
    ros::Publisher goal_pub;
    ros::Publisher filtered_scan_pub;
    ros::Publisher processed_scan_pub;
    tf::TransformListener tf_listener;
    sensor_msgs::LaserScan scan;
    std::vector<geometry_msgs::PoseStamped> map_goal_ptr;
    std::vector<std::vector<float>> obstacleCenterPoint;
    std::vector<std::vector<float>> obstacleFrontPoint;
    std::vector<std::vector<float>> obstacleBackPoint;

    int control;
    
    int window_size;
    float min_valid_range;
    float max_valid_range;
    std::vector<std::deque<float>> range_windows;

    double continuity_threshold;
    double target_width;
    double linearity_threshold;

    int detected_segment_start_index_;
    int detected_segment_end_index_;
};

LaserScanPr::LaserScanPr() : 
    nh(),                  
    private_nh("~"),       
    laser_sub(nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserScanPr::laserFilter, this)),
    goal_pub(nh.advertise<geometry_msgs::PoseArray>("yd_msg", 1)), 
    tf_listener(ros::Duration(10.0)),
    control(0),
    detected_segment_start_index_(-1),
    detected_segment_end_index_(-1)
{
    private_nh.param("window_size", window_size, 2);
    private_nh.param("min_valid_range", min_valid_range, 0.1f);
    private_nh.param("max_valid_range", max_valid_range, 10.0f);
    private_nh.param("continuity_threshold", continuity_threshold, 0.1);
    private_nh.param("target_width", target_width, 0.5);
    private_nh.param("linearity_threshold", linearity_threshold, 0.02);

    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/filtered_scan", 1);
    processed_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/processed_scan", 1);
}

bool LaserScanPr::isSegmentLinear(const sensor_msgs::LaserScan &scan, int startIndex, int endIndex) {
    if (endIndex - startIndex < 2) {
        return true;
    }
    float start_x, start_y, end_x, end_y;
    toCartesian(scan.ranges[startIndex], scan.angle_min + startIndex * scan.angle_increment, start_x, start_y);
    toCartesian(scan.ranges[endIndex], scan.angle_min + endIndex * scan.angle_increment, end_x, end_y);
    double A = start_y - end_y;
    double B = end_x - start_x;
    double C = -A * start_x - B * start_y;
    double denominator = std::sqrt(A * A + B * B);
    if (denominator < 1e-6) {
        return true;
    }
    for (int i = startIndex + 1; i < endIndex; ++i) {
        float point_x, point_y;
        toCartesian(scan.ranges[i], scan.angle_min + i * scan.angle_increment, point_x, point_y);
        double distance = std::abs(A * point_x + B * point_y + C) / denominator;
        if (distance > linearity_threshold) {
            return false;
        }
    }
    return true;
}

void LaserScanPr::laserFilter(const sensor_msgs::LaserScanConstPtr &laserRaw) {
    if (range_windows.empty() && !laserRaw->ranges.empty()) {
        range_windows.resize(laserRaw->ranges.size());
    }
    if (laserRaw->ranges.empty()) {
        ROS_WARN_THROTTLE(5.0, "Received an empty laser scan.");
        return;
    }
    scan = *laserRaw; 
    scan.ranges.assign(laserRaw->ranges.size(), laserRaw->range_max);

    for (size_t i = 0; i < laserRaw->ranges.size(); ++i) {
        float raw_range = laserRaw->ranges[i];
        bool is_valid = !(std::isinf(raw_range) || std::isnan(raw_range) || raw_range < min_valid_range || raw_range > max_valid_range);
        if (is_valid) {
            range_windows[i].push_back(raw_range);
            if (range_windows[i].size() > window_size) {
                range_windows[i].pop_front();
            }
        }
        if (!range_windows[i].empty()) {
            std::vector<float> temp_window;
            for (const auto& val : range_windows[i]) temp_window.push_back(val);
            std::sort(temp_window.begin(), temp_window.end());
            scan.ranges[i] = (temp_window.size() % 2 == 1)
                           ? temp_window[temp_window.size() / 2]
                           : (temp_window[temp_window.size() / 2 - 1] + temp_window[temp_window.size() / 2]) / 2.0f;
        } else {
            scan.ranges[i] = laserRaw->range_max;
        }
    }

    laserDiffusion(scan);
    publishGoal();
    filtered_scan_pub.publish(scan);
}

void LaserScanPr::laserDiffusion(sensor_msgs::LaserScan &scanIn) {
    detected_segment_start_index_ = -1;
    detected_segment_end_index_ = -1;

    if (scanIn.ranges.empty()) return;

    int center_index = static_cast<int>(round((0.0 - scanIn.angle_min) / scanIn.angle_increment));
    if (center_index < 0 || center_index >= scanIn.ranges.size()) {
        ROS_WARN_THROTTLE(5.0, "Center index is out of bounds. Cannot perform diffusion.");
        return;
    }
    if (scanIn.ranges[center_index] >= scanIn.range_max) return;

    int left_index = center_index;
    int right_index = center_index;
    bool left_expandable = true;
    bool right_expandable = true;

    while (true) {
        bool expanded_this_iteration = false;

        if (right_expandable) {
            int next_right_index = right_index + 1;
            if (next_right_index < scanIn.ranges.size() && scanIn.ranges[next_right_index] < scanIn.range_max) {
                float x1, y1, x2, y2;
                toCartesian(scanIn.ranges[right_index], scanIn.angle_min + right_index * scanIn.angle_increment, x1, y1);
                toCartesian(scanIn.ranges[next_right_index], scanIn.angle_min + next_right_index * scanIn.angle_increment, x2, y2);
                float dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
                if (dist < continuity_threshold) {
                    if (isSegmentLinear(scanIn, left_index, next_right_index)) {
                        right_index = next_right_index;
                        expanded_this_iteration = true;
                    } else { right_expandable = false; }
                } else { right_expandable = false; }
            } else { right_expandable = false; }
        }

        if (left_expandable) {
            int next_left_index = left_index - 1;
            if (next_left_index >= 0 && scanIn.ranges[next_left_index] < scanIn.range_max) {
                float x1, y1, x2, y2;
                toCartesian(scanIn.ranges[left_index], scanIn.angle_min + left_index * scanIn.angle_increment, x1, y1);
                toCartesian(scanIn.ranges[next_left_index], scanIn.angle_min + next_left_index * scanIn.angle_increment, x2, y2);
                float dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
                if (dist < continuity_threshold) {
                    if (isSegmentLinear(scanIn, next_left_index, right_index)) {
                        left_index = next_left_index;
                        expanded_this_iteration = true;
                    } else { left_expandable = false; }
                } else { left_expandable = false; }
            } else { left_expandable = false; }
        }
        
        float final_lx, final_ly, final_rx, final_ry;
        toCartesian(scanIn.ranges[left_index], scanIn.angle_min + left_index * scanIn.angle_increment, final_lx, final_ly);
        toCartesian(scanIn.ranges[right_index], scanIn.angle_min + right_index * scanIn.angle_increment, final_rx, final_ry);
        float total_width = std::sqrt(std::pow(final_rx - final_lx, 2) + std::pow(final_ry - final_ly, 2));

        if (total_width >= target_width || (!expanded_this_iteration && (!left_expandable && !right_expandable))) {
            break;
        }
    }

    if (left_index < right_index) {
        detected_segment_start_index_ = left_index;
        detected_segment_end_index_ = right_index;

        sensor_msgs::LaserScan output_scan;
        output_scan.header = scanIn.header;
        output_scan.angle_increment = scanIn.angle_increment;
        output_scan.time_increment = scanIn.time_increment;
        output_scan.scan_time = scanIn.scan_time;
        output_scan.range_min = scanIn.range_min;
        output_scan.range_max = scanIn.range_max;
        output_scan.angle_min = scanIn.angle_min + left_index * scanIn.angle_increment;
        output_scan.angle_max = scanIn.angle_min + right_index * scanIn.angle_increment;
        output_scan.ranges.assign(scanIn.ranges.begin() + left_index, scanIn.ranges.begin() + right_index + 1);
        if (!scanIn.intensities.empty() && scanIn.intensities.size() == scanIn.ranges.size()) {
            output_scan.intensities.assign(scanIn.intensities.begin() + left_index, scanIn.intensities.begin() + right_index + 1);
        }
        processed_scan_pub.publish(output_scan);
    }
}

void LaserScanPr::publishGoal() {
    float distance = 0.35;   // 默认是找前方的板
    if (private_nh.getParam("control", control)) {} 
    else {
        ROS_WARN_THROTTLE(5.0, "Could not get control parameter from server, using default %d.", control);
    }
    // control == 0， 表示不进行发点的逻辑
    if (control == 0) {
        return;
    }
    // distance 为正值，表示跑到点前
    if (control == 1) {
        if (private_nh.getParam("distance_front", distance)) {} 
        else {
            ROS_WARN_THROTTLE(5.0, "Could not get control parameter from server, using default %d.", distance);
        }
    }
    // distance 为负值，表示跑到点后
    if (control == 2) {
        if (private_nh.getParam("distance_back", distance)) {} 
        else {
            ROS_WARN_THROTTLE(5.0, "Could not get control parameter from server, using default %d.", distance);
        }  
    }
    if (scan.ranges.empty()) {
        ROS_WARN_THROTTLE(1.0, "Scan data is empty, cannot publish goal.");
        return;
    }
    
    // 检查 laserDiffusion 是否找到了一个有效的线段
    if (detected_segment_start_index_ < 0 || detected_segment_end_index_ < 0 || detected_segment_start_index_ >= detected_segment_end_index_) {
        ROS_WARN_THROTTLE(1.0, "No valid linear segment detected in front, cannot publish goal.");
        return;
    }
    
    // 获取线段的左右端点索引
    int left_idx = detected_segment_start_index_;
    int right_idx = detected_segment_end_index_;

    float left_x, left_y, right_x, right_y, center_x, center_y;
    
    // 计算线段左右端点的笛卡尔坐标
    toCartesian(scan.ranges[left_idx], scan.angle_min + left_idx * scan.angle_increment, left_x, left_y);
    toCartesian(scan.ranges[right_idx], scan.angle_min + right_idx * scan.angle_increment, right_x, right_y);
    
    // 新增：计算线段的中点作为基准中心点
    center_x = (left_x + right_x) / 2.0f;
    center_y = (left_y + right_y) / 2.0f;

    // 计算线段的法线方向 (yaw)
    float yaw = atan2(right_y - left_y, right_x - left_x) - 0.5 * M_PI;

    // 沿着法线方向，从新的中心点向外偏移
    float target_x = center_x - distance * cos(yaw);
    float target_y = center_y - distance * sin(yaw);

    geometry_msgs::PoseStamped laser_pose_front;
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    laser_pose_front.header.frame_id = scan.header.frame_id;
    laser_pose_front.header.stamp = ros::Time::now();
    laser_pose_front.pose.position.x = target_x;
    laser_pose_front.pose.position.y = target_y;
    laser_pose_front.pose.position.z = 0.0;
    laser_pose_front.pose.orientation = quaternion;

    geometry_msgs::PoseStamped map_point_front;
    try {
        tf_listener.waitForTransform("map", laser_pose_front.header.frame_id, 
                                     laser_pose_front.header.stamp, ros::Duration(0.5));
        tf_listener.transformPose("map", laser_pose_front, map_point_front);
    } catch (tf::TransformException &ex) {
        ROS_ERROR_THROTTLE(1.0, "Failed to transform pose from %s to map: %s", 
                           laser_pose_front.header.frame_id.c_str(), ex.what());
        return;
    }
    if (control == 2) {
        // Todo: 添加set"zhang"逻辑, 当障碍物距离车子50cm内并且在框定的范围内认为该避障了
        float dis = sqrt(pow(center_x, 2) + pow(center_y, 2));
        if (dis <= 0.55) {
            // 这里是点后的位置的 map 坐标
            // if (map_point_front.pose.position.x <= 4.5 && map_point_front.pose.position.x >= 3.0
            //     && map_point_front.pose.position.y >= 0.2 && map_point_front.pose.position.y <= 1.5) {
                ros::param::set("/zhang", 1);
            // }
        }
    }
    geometry_msgs::PoseArray goal_array_msg;
    goal_array_msg.header.frame_id = "map";
    goal_array_msg.header.stamp = map_point_front.header.stamp;
    goal_array_msg.poses.push_back(map_point_front.pose);
    goal_pub.publish(goal_array_msg);
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "laser_scan_pr");
    LaserScanPr processor;
    ROS_INFO("Laser Scan Processor node started.");
    ros::spin();
    return 0;
}
