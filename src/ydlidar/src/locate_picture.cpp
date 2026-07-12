// 这个代码是实际比赛使用的板子定位以及发布航点的程序
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <stdlib.h>

class LaserScan {
public:
    LaserScan();
    void laserCloudHandler(const sensor_msgs::LaserScanConstPtr &laserScan);
    void removeClosedPointCloud(const sensor_msgs::LaserScan &scan_in,
                                sensor_msgs::LaserScan &scan_out, float min); // 这个函数就没有使用
    void resetRange(const std::vector<std::vector<float>> &obstacleMaycoord_in,
                    std::vector<std::vector<float>> &obstacleMaycoord_out,
                    float max_x_f, float max_x_b, float max_y_l, float max_y_r);

private:
    ros::NodeHandle nh;
    ros::Subscriber laser_sub;
    ros::Subscriber control_sub;
    ros::Publisher goal_pub;
    tf::TransformListener tf_listener;
    std::vector<geometry_msgs::PoseStamped> map_goal_ptr;
    std::vector<std::vector<float>> obstacleCenterPoint;
    std::vector<std::vector<float>> obstacleFrontPoint;
    std::vector<std::vector<float>> obstacleBackPoint;

    int obstacle_counter = 0;
    bool systemInited = true;
    int systemInitCount = 0;
    const int systemDelay = 5;
    const float MINIMUM_RANGE = 0.1;
    const float MAXIMUM_RANGE = 3;
    float MAX_X_FRONT = 4;
    float MAX_X_BACK = -0.2;
    float MAX_Y_LEFT = 0.3;    // 0.3 -> 0.5 17 xu
    float MAX_Y_RIGHT = 0.3;
    bool debug = false;
    bool enabled = true;
    int xflag=0,zhang=0;
    ros::Rate rate = 10.0;

    void TransformCoord(const sensor_msgs::LaserScan &laserCloudIn, std::vector<std::vector<float>> &obstacleCoord, float min, float max);
    void controlHandler(const std_msgs::Int8::ConstPtr& msg);
    void publishGoals();
};

// 构造函数，两个订阅者的初始化、一个目标发送者的初始化喵 > ~ < !!
// 然后主要的逻辑就在laserCloudHandler里
LaserScan::LaserScan() {
    control_sub = nh.subscribe<std_msgs::Int8>("start_yd", 1, &LaserScan::controlHandler, this);
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserScan::laserCloudHandler, this);
    goal_pub = nh.advertise<geometry_msgs::PoseArray>("yd_msg", 1);
}

// 在 LaserScan::controlHandler 函数中

void LaserScan::controlHandler(const std_msgs::Int8::ConstPtr& msg) {
    if (msg->data == 0) {
        // --- 开启处理逻辑 ---
        enabled = true;
        ROS_INFO("LD YES - Processing is ON.");
    } else if (msg->data == 1) { // 明确使用 1 作为“关闭”指令
        // --- 关闭处理逻辑 ---
        enabled = false;
        ROS_INFO("LD NO - Processing is OFF.");
    } else if (msg->data == 2) {
        // --- 永久终止节点 ---
        ROS_INFO("LD DONE - Shutting down node permanently.");
        nh.shutdown();
    } else {
        // 对其他未定义的值给出警告，可以默认为关闭状态
        ROS_WARN("Received an undefined control command: %d. Turning processing OFF.", msg->data);
        enabled = false;
    }
}

// 在把 scan_in 的激光信息 >> scan_out 中，用于更新
void LaserScan::removeClosedPointCloud(const sensor_msgs::LaserScan &scan_in,
                                       sensor_msgs::LaserScan &scan_out, float min) {
    
    // 把header字段更新，为了让scan_out能装下scan_in的内容
    if (&scan_in != &scan_out) {
        scan_out.header = scan_in.header;
        scan_out.ranges.resize(scan_in.ranges.size());
    }
    size_t j = 0;
    for (size_t i = 0; i < scan_in.ranges.size(); ++i) {
        if (scan_in.ranges[i] < min)  // 似乎认为是机器的天线反射的雷达信号，要忽略喵~
            continue;
        scan_out.ranges[j] = scan_in.ranges[i];
        scan_out.intensities[j] = scan_in.intensities[j];
        j++;
    }

    // 组织scan_out的长度为实际长度
    if (j != scan_in.ranges.size()) {
        scan_out.ranges.resize(j);
        scan_out.intensities.resize(j);
    }
}

// 计算出每个有效扫描点的坐标(相对于雷达)并放入 obstacleCoord, 距离范围是 min ~ max（会把从基于雷达的极坐标系转变成欧拉坐标）
// x就是相对于原来极坐标上的x的距离值，y同理，x的正值表现为车子的正前方，y的正值表现为车子的正左方
void LaserScan::TransformCoord(const sensor_msgs::LaserScan &laserCloudIn, std::vector<std::vector<float>> &obstacleCoord, float min, float max) {
    float startOri = laserCloudIn.angle_min;
    float angleIncrement = laserCloudIn.angle_increment;

    for (size_t i = 0; i < laserCloudIn.ranges.size(); ++i) {
        // 把噪声去除
        if (std::isnan(laserCloudIn.ranges[i]) || std::isinf(laserCloudIn.ranges[i]) || laserCloudIn.ranges[i] < min || laserCloudIn.ranges[i] > max) {
            continue;
        }

        std::vector<float> coord;
        float obstacleMayangle = startOri + i * angleIncrement;

        float x = cos(obstacleMayangle) * laserCloudIn.ranges[i];
        float y = sin(obstacleMayangle) * laserCloudIn.ranges[i];

        coord.push_back(x);
        coord.push_back(y);
        coord.push_back(i);

        obstacleCoord.push_back(coord);
    }
}


// 选择检测的范围 x,y的含义依照于LaserScan::TransformCoord的注释
void LaserScan::resetRange(const std::vector<std::vector<float>> &obstacleMaycoord_in,
                           std::vector<std::vector<float>> &obstacleMaycoord_out,
                           float max_x_f, float max_x_b, float max_y_l, float max_y_r) {
    size_t j = 0;
    for (size_t i = 0; i < obstacleMaycoord_in.size(); i++) {
        float x = obstacleMaycoord_in[i][0];
        float y = obstacleMaycoord_in[i][1];

        if (x >= -max_x_b && x <= max_x_f && y >= -max_y_l && y <= max_y_r) {
            obstacleMaycoord_out[j] = obstacleMaycoord_in[i];
            // std::cout << "x:" << x << " , y: "<< y << ", i:" << obstacleMaycoord_in[i][2] << std::endl;
            j++;
        }
    }
    if (j != obstacleMaycoord_in.size()) {
        obstacleMaycoord_out.resize(j);
    }
}

// 提取板子中点
void LaserScan::laserCloudHandler(const sensor_msgs::LaserScanConstPtr &laserScan) {
    static int first_xflag = 0;
    if (!ros::param::get("/xflag", xflag)) {
        ROS_WARN("Parameter /xflag not found, using default value: %d", xflag);
    }
    if(xflag == 1 && first_xflag == 0) {
        first_xflag = 1;
        for(int i=0; i<60; i++){
            ros::Duration(0.05).sleep();
        }
    }
    if (xflag) {
        ROS_WARN("This is back_point");
        MAX_X_FRONT = 1;   // 设置在巡线区识别障碍板的范围
        MAX_X_BACK = 0;
        MAX_Y_LEFT = 0.4;
        MAX_Y_RIGHT = 0.4;
    }
    else 
        // ROS_WARN("This is front_point");
    if (!enabled) {
        return;
    }

    if (!systemInited) {
        systemInitCount++;
        if (systemInitCount >= systemDelay) {
            systemInited = true;
        } else {
            return;
        }
    }
    // 把原始数据 >> laserCloudIn
    sensor_msgs::LaserScan laserCloudIn = *laserScan;

    // 滤波
    std::vector<std::vector<float>> obstacleCoord;
    TransformCoord(laserCloudIn, obstacleCoord, MINIMUM_RANGE, MAXIMUM_RANGE);
    // 第一个和第二个参数是同个值，是为了在函数定义中方便梳理逻辑么？
    resetRange(obstacleCoord, obstacleCoord, MAX_X_FRONT, MAX_X_BACK, MAX_Y_LEFT, MAX_Y_RIGHT);
    
    // @@@@@@@  4月13日看到了这里
    // @@@@@@@  明天继续喵！！
    std::vector<std::vector<float>> obstacleCornerPoint;
    for (size_t i = 0; i < obstacleCoord.size(); i++) {
        if (i == obstacleCoord.size() - 1) break;

        // first
        if (i == 0) {
            obstacleCornerPoint.push_back(obstacleCoord[i]);
            continue;
        }

        float dx = obstacleCoord[i + 1][0] - obstacleCoord[i][0];
        float dy = obstacleCoord[i + 1][1] - obstacleCoord[i][1];
        float dist = sqrt(dx * dx + dy * dy); // 这里算出的相邻两个点的欧拉距离

        // 如果相邻两个点的距离大于0.04则认为 i、i+1 为角点的索引 （不过这样做可能会导致一些点被重复添加）
        // ------ 17号被 徐 dist > 0.04 -> dist > 0.02
        if (dist > 0.02) { // || (angleChange >= 0.45)
            obstacleCornerPoint.push_back(obstacleCoord[i]);
            obstacleCornerPoint.push_back(obstacleCoord[i + 1]);
        }

        // last
        if (i == obstacleCoord.size() - 2) {
            if (obstacleCornerPoint.back()[2] != obstacleCoord[i + 1][2]) {
                obstacleCornerPoint.push_back(obstacleCoord[i]);
            }
        }
    }

    // 调试模式
    if(debug) {
        ROS_INFO("--------------------------------------"); // 循环打印出角点的位置喵！
        for(size_t i = 0; i < obstacleCornerPoint.size(); i++) {
            std::cout <<  " x is " << obstacleCornerPoint[i][0] << " y is " << obstacleCornerPoint[i][1] << std::endl;
        }
        std::cout <<  " this " << obstacleCornerPoint.size() << " obstacle " << std::endl;
    }
    
    if (obstacleCornerPoint.size() > 0) {
        obstacleCenterPoint.clear();
        obstacleFrontPoint.clear();
        obstacleBackPoint.clear();
        // 注意这里的步长为 2 ，Xu认为是为了和第 197 行的重复添加有关
        for (size_t i = 0; i < obstacleCornerPoint.size(); i += 2) {
            if (i == obstacleCornerPoint.size() - 1) break;
            float x1 = obstacleCornerPoint[i][0];
            float y1 = obstacleCornerPoint[i][1];
            float x2 = obstacleCornerPoint[i + 1][0];
            float y2 = obstacleCornerPoint[i + 1][1];

            float size = obstacleCornerPoint[i + 1][2] - obstacleCornerPoint[i][2];
            float distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

            if(debug) {
                std::cout <<  "dist:" << distance << "  size:" << size << std::endl;
            }

            // 这里认为两个角点之间的距离在 0.35 ~ 0.54 之间时，两点之间的内容是目标板子
            if (distance >= 0.40 && distance <= 0.6) {
                float center_x = (x1 + x2) * 0.5;
                float center_y = (y1 + y2) * 0.5;
                float yaw = atan2(y2 - y1, x2 - x1) - 0.5 * M_PI;  // 这里计算出的方向角为两点连线的法线方向

                float front_x = center_x - 0.35 * cos(yaw);  // 这个FrontPoint好像是从当前车到目标板中线的路径点
                float front_y = center_y - 0.35 * sin(yaw);

                float back_x = center_x + 0.5 * cos(yaw);
                float back_y = center_y + 0.5 * sin(yaw);

                std::vector<float> CenterPoint{center_x, center_y, yaw};
                std::vector<float> FrontPoint{front_x, front_y, yaw};
                std::vector<float> BackPoint{back_x, back_y, yaw};
                obstacleCenterPoint.push_back(CenterPoint);
                obstacleFrontPoint.push_back(FrontPoint);
                obstacleBackPoint.push_back(BackPoint);
            }
        }

        auto compare = [](const std::vector<float>& a, const std::vector<float>& b) {
            float weight_x = 0.1;
            float weight_y = 0.9;

            float weighted_a = weight_x * a[0] + weight_y * std::abs(a[1]);
            float weighted_b = weight_x * b[0] + weight_y * std::abs(b[1]);

            return weighted_a < weighted_b;
        };

        std::sort(obstacleCenterPoint.begin(), obstacleCenterPoint.end(), compare);
        std::sort(obstacleFrontPoint.begin(), obstacleFrontPoint.end(), compare);
        std::sort(obstacleBackPoint.begin(), obstacleBackPoint.end(), compare);
        publishGoals();
    }

    rate.sleep();
}

// 编写发布者
void LaserScan::publishGoals() {
    map_goal_ptr.clear();
    for (size_t i = 0; i < obstacleCenterPoint.size(); i++) {
        float yaw = obstacleCenterPoint[i][2];
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
        geometry_msgs::PoseStamped front_goal , center_goal, back_goal;

        center_goal.header.frame_id = "laser_frame";
        center_goal.header.stamp = ros::Time::now();
        center_goal.pose.position.x = obstacleCenterPoint[i][0];
        center_goal.pose.position.y = obstacleCenterPoint[i][1];
        center_goal.pose.position.z = 0.0;
        center_goal.pose.orientation = quaternion;

        front_goal.header.frame_id = "laser_frame";
        front_goal.header.stamp = ros::Time::now();
        front_goal.pose.position.x = obstacleFrontPoint[i][0];
        front_goal.pose.position.y = obstacleFrontPoint[i][1];
        front_goal.pose.position.z = 0.0;
        front_goal.pose.orientation = quaternion;
 
        back_goal.header.frame_id = "laser_frame";
        back_goal.header.stamp = ros::Time::now();
        back_goal.pose.position.x = obstacleBackPoint[i][0] - 0.3;   // 21 by xu
        back_goal.pose.position.y = obstacleBackPoint[i][1];
        back_goal.pose.position.z = 0.0;
        back_goal.pose.orientation = quaternion;    

        // ROS_INFO("yaw1:%f",yaw);
        geometry_msgs::PoseStamped front_map_goal, center_map_goal, back_map_goal;

        try {
            tf_listener.waitForTransform("map", "laser_frame", ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose("map", ros::Time(0), front_goal, "laser_frame", front_map_goal);
            tf_listener.transformPose("map", ros::Time(0), center_goal, "laser_frame", center_map_goal);
            tf_listener.transformPose("map", ros::Time(0), back_goal, "laser_frame", back_map_goal);
            // std::cout << front_map_goal.header.frame_id << std::endl;
            // if (xflag)
                // ROS_INFO("Transformed goal of back: (%f, %f)", back_map_goal.pose.position.x, back_map_goal.pose.position.y);
            // else 
                // ROS_INFO("Transformed goal of front: (%f, %f)", front_map_goal.pose.position.x, front_map_goal.pose.position.y);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Failed to transform point: %s", ex.what());
            continue;
        }
        // // 添加这行日志进行调试
        //  ROS_INFO("Transformed center_map_goal: x=%.2f, y=%.2f",center_map_goal.pose.position.x,
                    // center_map_goal.pose.position.y);
        // 如果目标板以下矩形范围内，则认为检测正确------------- MATTER
        if (center_map_goal.pose.position.x > 0.0 && center_map_goal.pose.position.x < 5.0 &&
            center_map_goal.pose.position.y > 0.0 && center_map_goal.pose.position.y < 5.0) {
            if (center_map_goal.pose.position.x < -2.2 && center_map_goal.pose.position.y < -1.9)  // 这个判断可以用于某些不可能点的判断
                continue;
            
            if (xflag && center_map_goal.pose.position.x > 2.5 && center_map_goal.pose.position.x < 5.0 &&
                center_map_goal.pose.position.y > 0.0 && center_map_goal.pose.position.y < 3.3) {
                ROS_INFO("maybe ban is here");
                yaw = tf::getYaw(back_map_goal.pose.orientation);
                quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
                // geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
                back_map_goal.pose.orientation = quaternion;
                if(debug) {
                    ROS_INFO("map goal after: (%f, %f)", back_map_goal.pose.position.x, back_map_goal.pose.position.y);
                }
                ros::param::set("/pointx",back_map_goal.pose.position.x);
                ros::param::set("/pointx",back_map_goal.pose.position.y);
                map_goal_ptr.push_back(back_map_goal);
            }
            else {
                yaw = tf::getYaw(front_map_goal.pose.orientation);
                quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
                // geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
                front_map_goal.pose.orientation = quaternion;
                if(debug) {
                    ROS_INFO("map goal after: (%f, %f)", front_map_goal.pose.position.x, front_map_goal.pose.position.y);
                }
                map_goal_ptr.push_back(front_map_goal);
            }
        }

    }
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = ros::Time::now();
    pose_array_msg.header.frame_id = "map";


    for (size_t i = 0; i < map_goal_ptr.size(); ++i) {
        pose_array_msg.poses.push_back(map_goal_ptr[i].pose);
    }

    goal_pub.publish(pose_array_msg); // cl
    if (xflag == 1 && map_goal_ptr.size() > 0) {
        bool obstacle_found = false; // 标志在当前这帧数据里是否找到了障碍物
        for (size_t i = 0; i < obstacleCenterPoint.size(); i++) {
            float x_center = obstacleCenterPoint[i][0];
            float y_center = obstacleCenterPoint[i][1];

            tf::StampedTransform transform;
            bool transform_success = false;
            for (int attempt = 0; attempt < 3; attempt++) {  // 重试3次
                try {
                    tf_listener.lookupTransform("base_link", "laser_frame", ros::Time(0), transform);
                    transform_success = true;
                    break;
                } catch (tf::TransformException &ex) {
                    ROS_WARN("Transform attempt %d failed: %s", attempt + 1, ex.what());
                    ros::Duration(0.1).sleep();  // 等待0.1秒重试
                }
            }
            if (!transform_success) {
                ROS_ERROR("Failed to get transform after retries.");
                continue;
            }

            float distance_to_car = sqrt(pow(x_center - transform.getOrigin().x(), 2) +
                                        pow(y_center - transform.getOrigin().y(), 2));
            int current_zhang_val = 0;
            ros::param::get("/zhang", current_zhang_val);
            // 只在前方(x > 0)且距离 <= 1米时触发
            if (x_center > 0 && distance_to_car <= 0.8) {  // 0.8 -> 1
                obstacle_found = true;
                break;  // 检测到前方障碍物后立即退出循环
            }
        }
        if (obstacle_found) {
            obstacle_counter++;
        }
        else {
            obstacle_counter = 0;
        }
        if (obstacle_counter >= 5) {
            ros::param::set("/zhang", 1);
            ROS_INFO("Obstacle detected for %d consecutive frames. /zhang set to 1.", obstacle_counter);
        }
        else {
            ros::param::set("/zhang", 0);
            if (obstacle_counter > 0) {
                ROS_INFO("count %d is below 5", obstacle_counter);
            }
        }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ld_loc_node");
    LaserScan ls;
    ros::spin();
    return 0;
}