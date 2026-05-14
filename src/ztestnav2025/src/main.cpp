#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

// 定义 move_base 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief 向 AMCL 发送初始位姿，用于开机或重定位时的粒子滤波初始化
 * 这是 SLAM 定位系统中非常关键的一步，包含了位姿估计和位姿的协方差矩阵（置信度）
 */
void publishInitialPose(double x, double y, double yaw, tf2::Quaternion &q, ros::Publisher& initial_pose_pub_) {
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    
    // 设置 header
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.header.frame_id = "map";  // 强依赖于 SLAM 建出的 map 坐标系
    
    // 设置平移位置
    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = 0.0;
    
    // 设置旋转姿态
    q.setRPY(0, 0, yaw);
    initial_pose.pose.pose.orientation.x = q.x();
    initial_pose.pose.pose.orientation.y = q.y();
    initial_pose.pose.pose.orientation.z = q.z();
    initial_pose.pose.pose.orientation.w = q.w();
    
    // 设置协方差矩阵 (表示对初始定位的置信度，值越小代表越确信)
    boost::array<double, 36> covariance = {{
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0076
    }};
    initial_pose.pose.covariance = covariance;
    
    // 发布初始位置，唤醒 AMCL 粒子云
    initial_pose_pub_.publish(initial_pose);
    ROS_INFO("AMCL 初始化完成: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
}

/**
 * @brief 封装目标点格式
 */
void goal_set(move_base_msgs::MoveBaseGoal &goal, double x, double y, double yaw, tf2::Quaternion q) {
    q.setRPY(0, 0, yaw);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
}

/**
 * @brief 向底层的全局/局部路径规划器发送导航请求
 */
void go_destination(move_base_msgs::MoveBaseGoal &goal, double x, double y, double yaw, tf2::Quaternion &q, MoveBaseClient &ac) {
    goal.target_pose.header.stamp = ros::Time::now();
    goal_set(goal, x, y, yaw, q);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("到达目标点");
    else
        ROS_WARN("路径规划或执行失败");
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "slam_nav_controller");
    ros::NodeHandle nh;

    // 初始化导航 Action 客户端
    MoveBaseClient ac("move_base", true); 
    tf2::Quaternion q;  
    
    // 等待底层导航与建图节点启动
    ROS_INFO("等待底层规划与定位服务启动...");
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("等待 move_base 服务中...");
    } 

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // 必须基于全局地图坐标系

    // 初始化定位发布器与代价地图清理客户端
    ros::Publisher initial_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::ServiceClient clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;

    // 1. 初始化 AMCL 定位 (假设起点已知)
    // 注入先验位姿信息，收敛粒子滤波
    publishInitialPose(0.25, 0.25, 0.0, q, initial_pose_pub_);

    // 2. 发送首个导航目标，触发全局规划器 (Dijkstra/A*) 和局部规划器
    ROS_INFO("开始环境自主导航...");
    go_destination(goal, 1.25, 0.75, 3.14, q, ac);

    // 3. 动态地图处理演示：清理代价地图
    // 在复杂环境中导航一段时间后，清除可能残留的噪点或动态障碍物假象
    if (clear_costmaps_client.call(srv)) { 
        ROS_INFO("局部/全局代价地图清理成功，准备重新规划");
    } else {
        ROS_WARN("代价地图清理服务调用失败");
    }

    // 4. 前往下一个目标点
    go_destination(goal, 1.25, 3.75, 0.0, q, ac);
    
    return 0;
}