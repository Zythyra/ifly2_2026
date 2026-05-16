/**
 * @file voice_wakeup_test.cpp
 * @brief 2026 智能车比赛 - 极简语音唤醒独立测试节点 (修正 Int8 匹配)
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h> // 【核心修正】：必须用 Int8 匹配底层的 md5sum

// 唤醒回调函数，参数也必须是 Int8
void awakeCallback(const std_msgs::Int8::ConstPtr& msg) {
    // 只要收到数据就打印出来
    ROS_INFO("=================================================");
    ROS_INFO("🔔 [语音测试成功]：检测到小车被喊醒！信号码: %d", msg->data);
    ROS_INFO("=================================================");
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); // 解决中文乱码
    
    ros::init(argc, argv, "voice_wakeup_test_node");
    ros::NodeHandle nh;

    ROS_INFO("🎙️ 正在监听语音唤醒信号...");
    ROS_INFO("-> 请对麦克风大喊：‘小飞小飞’ ");

    // 【核心修正】：订阅带斜杠的 "/awake_flag" 话题，队列长度设为 10
    ros::Subscriber awake_sub = nh.subscribe("/awake_flag", 10, awakeCallback);

    ros::spin(); // 挂起，死循环等待接收话题消息

    return 0;
}