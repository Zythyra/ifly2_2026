#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

#include <clocale>

#include "ucarmain2026/set_speed.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace {

geometry_msgs::Twist current_twist;
bool manual_control_active = false;
int zero_publish_cycles = 0;
double command_timeout = 0.50;
ros::WallTime last_command_time;

void requestZeroPublish() {
    current_twist = geometry_msgs::Twist();
    manual_control_active = false;
    // 连续发布三帧零速度，避免底盘漏掉单帧停车指令。
    zero_publish_cycles = 3;
}

bool setSpeedCallback(ucarmain2026::set_speed::Request& request,
                      ucarmain2026::set_speed::Response& response) {
    if (!request.work) {
        requestZeroPublish();
        ROS_INFO("运动控制节点停止");
    } else {
        if (request.target_twist.angular.x != 0.0 ||
            request.target_twist.angular.y != 0.0 ||
            request.target_twist.linear.z != 0.0) {
            ROS_ERROR("请勿输入底盘不支持的速度分量");
            response.success = false;
            return true;
        }

        current_twist = request.target_twist;
        manual_control_active = true;
        zero_publish_cycles = 0;
        last_command_time = ros::WallTime::now();
    }

    if (request.movebase_flag) {
        ROS_INFO("开始move_base导航");
        MoveBaseClient move_base("move_base", true);
        while (ros::ok() && !move_base.waitForServer(ros::Duration(2.0))) {
            ROS_INFO("等待move_base服务中...");
        }
        if (!ros::ok()) {
            response.success = false;
            return true;
        }

        move_base_msgs::MoveBaseGoal goal;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, request.target_yaw);
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = request.target_x;
        goal.target_pose.pose.position.y = request.target_y;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.x = quaternion.x();
        goal.target_pose.pose.orientation.y = quaternion.y();
        goal.target_pose.pose.orientation.z = quaternion.z();
        goal.target_pose.pose.orientation.w = quaternion.w();

        move_base.sendGoal(goal);
        move_base.waitForResult();
        if (move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("到达目标");
        } else {
            ROS_WARN("无法到达目标，状态：%s",
                     move_base.getState().toString().c_str());
        }
    }

    response.success = true;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "simple_move_control");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    private_node.param("command_timeout", command_timeout, 0.50);
    if (command_timeout <= 0.0) {
        ROS_WARN("command_timeout必须大于0，恢复为0.50秒");
        command_timeout = 0.50;
    }

    ros::ServiceServer speed_service =
        node.advertiseService("set_speed", setSpeedCallback);
    ros::Publisher command_publisher =
        node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate control_rate(20.0);
    ROS_INFO("运动控制节点已启动（20Hz），速度指令看门狗=%.2f秒",
             command_timeout);

    while (ros::ok()) {
        ros::spinOnce();

        if (manual_control_active) {
            const double command_age =
                (ros::WallTime::now() - last_command_time).toSec();
            if (command_age > command_timeout) {
                ROS_ERROR("速度指令已有%.3f秒未刷新，超过看门狗%.3f秒，自动停车",
                          command_age, command_timeout);
                requestZeroPublish();
            }
        }

        if (manual_control_active) {
            command_publisher.publish(current_twist);
        } else if (zero_publish_cycles > 0) {
            command_publisher.publish(geometry_msgs::Twist());
            --zero_publish_cycles;
        }

        control_rate.sleep();
    }

    ROS_INFO("运动控制节点关闭，发送零速度");
    geometry_msgs::Twist zero_twist;
    for (int i = 0; i < 3; ++i) {
        command_publisher.publish(zero_twist);
        ros::WallDuration(0.05).sleep();
    }
    return 0;
}
