#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

// 定义 actionlib 客户端类型，直接与 move_base 通信
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 封装位姿设置
void goal_set(move_base_msgs::MoveBaseGoal &goal, double x, double y, double yaw, tf2::Quaternion &q) {
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

// 封装发点与等待反馈逻辑
void go_destination(move_base_msgs::MoveBaseGoal &goal, double x, double y, double yaw, tf2::Quaternion &q, MoveBaseClient &ac) {
    goal.target_pose.header.frame_id = "map"; // 必须指定参考坐标系为 map
    goal_set(goal, x, y, yaw, q);
    
    ROS_INFO("正在前往目标点: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    ac.sendGoal(goal);
    ac.waitForResult(); // 阻塞等待，直到到达目标或失败
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("成功到达目标!");
    else
        ROS_WARN("无法到达目标，状态: %s", ac.getState().toString().c_str());
}

int main(int argc, char *argv[]) {
    // 解决中文乱码问题
    setlocale(LC_ALL, "");
    
    // 初始化 ROS 节点
    ros::init(argc, argv, "nav_test_node");
    ros::NodeHandle nh;

    // 实例化 MoveBase 客户端，true 表示不需要额外调用 ros::spin()
    MoveBaseClient ac("move_base", true); 
    tf2::Quaternion q;  
    move_base_msgs::MoveBaseGoal goal;

    // 等待 move_base 服务启动
    ROS_INFO("等待 move_base 服务中...");
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("仍在等待 move_base 服务...");
    } 
    ROS_INFO("成功连接到 move_base 服务！准备发车。");

    // ================= 开始依次发送测试点 ================= //
    
    // 目标点 1 (修改为你的实际地图坐标)
    go_destination(goal, 0.75, 5.25, 3.14, q, ac);
    ros::Duration(1.0).sleep(); // 到达后停顿1秒

    // 目标点 2
    go_destination(goal, 0.3, 3.25, 3.14, q, ac);
    ros::Duration(1.0).sleep();

    // 目标点 3
    go_destination(goal, 1.75, 4.15, 1.57, q, ac);
    ros::Duration(1.0).sleep();

    go_destination(goal, 3.75, 2.75, -1.57, q, ac);
    ros::Duration(1.0).sleep();

    // 目标点 4 (回到起点附近)
    go_destination(goal, 0.25, 0.25, -1.57, q, ac);
    ros::Duration(1.0).sleep();


    ROS_INFO("所有测试点已跑完，测试程序结束！");
    return 0;
}