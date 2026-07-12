#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Float64MultiArray.h"

const double pic_wid = 1280, pic_hit = 720;
const std::vector<std::string> class_names = {"Watermelon", "Cake", "Apple", "Banana",
     "Chili", "Tomato", "Milk", "Cola", "Potato"};
int ob_num; // 用于接收雷达定位点数量
bool target_found;  // 用于判断目标板是否在视野中
std::string thing = "Dessert";
std::string rething;


void ydCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    ob_num = msg->poses.size();
    // ROS_INFO("ob_num = %d", ob_num);
    
}

void viCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    rething = class_names[msg->data[0]];
    double Px1, Py1, Px2, Py2;
    double midPx;
    Px1 = msg->data[1];
    Py1 = msg->data[2];
    Px2 = msg->data[3];
    Py2 = msg->data[4];
    midPx = (Px1 + Px2) / 2;
    target_found = (
        (thing == "Dessert"   && (rething == "Cake" || rething == "Milk" || rething == "Cola")) ||
        (thing == "Fruit"     && (rething == "Watermelon" || rething == "Apple" || rething == "Banana")) ||
        (thing == "Vegetable" && (rething == "Chili" || rething == "Tomato" || rething == "Potato"))
    );
    ROS_INFO("target: %d, rething: %s, midPx: %lf", target_found, rething.c_str(), midPx);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "collectPic");
    ros::NodeHandle nh;
    ros::Subscriber yd_sub = nh.subscribe<geometry_msgs::PoseArray>("/yd_msg", 1, ydCallback);
    ros::Subscriber vi_sub = nh.subscribe<std_msgs::Float64MultiArray>("/nanodet/detect", 1, viCallback);
    double mid_goal[4]; // x,y,z,w
    ros::spin();
}