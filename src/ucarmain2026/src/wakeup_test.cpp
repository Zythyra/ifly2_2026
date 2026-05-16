/**
 * @file voice_wakeup_test.cpp
 * @brief 语音唤醒功能独立测试节点
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>

// 唤醒回调函数
void awakeCallback(const std_msgs::Int32::ConstPtr& msg) {
    // 收到唤醒信号！
    // 讯飞或 CSK5062 底层包通常在唤醒时发布 1，或者唤醒声源的角度 (0-360)
    ROS_INFO("===========================================");
    ROS_INFO("触发唤醒,收到底层驱动发来的指令码: %d", msg->data);
    ROS_INFO("===========================================");
}

int main(int argc, char** argv) {
    // 必须加这句，否则终端里的中文全是乱码问号
    setlocale(LC_ALL, ""); 
    
    ros::init(argc, argv, "wakeup_test_node");
    ros::NodeHandle nh;

    ROS_INFO("语音唤醒测试节点已启动...");
    ROS_INFO("-> 请对麦克风喊出唤醒词");

    // 订阅语音驱动包发出的唤醒话题
    // 注意：请确保底层的包发布的话题名确实是 /awake_flag，并且类型是 Int32
    ros::Subscriber awake_sub = nh.subscribe("/awake_flag", 10, awakeCallback);

    // 挂起程序，死循环等待接收话题消息
    ros::spin();

    return 0;
}