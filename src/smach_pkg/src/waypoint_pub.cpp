#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_array_example");
    ros::NodeHandle nh;

    // 修正：发布 PoseArray 而不是 std_msgs::String
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("pose_array_topic", 10);

    // 创建 PoseArray 消息
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";  // 坐标系

    // 创建多个 Pose 并添加到 PoseArray
    for (int i = 0; i < 3; i++) {
        geometry_msgs::Pose pose;
        pose.position.x = i * 1.0;
        pose.position.y = i * 2.0;
        pose.position.z = 0.0;  // 2D 平面
        pose.orientation.w = 1.0;  // 无旋转
        pose_array.poses.push_back(pose);
    }

    ros::Rate rate(1); // 1Hz 频率
    while (ros::ok()) {
        pose_array.header.stamp = ros::Time::now();  // 更新时间戳
        pub.publish(pose_array);
        ros::spinOnce();  // 允许处理回调
        rate.sleep();
    }

    return 0;
}
