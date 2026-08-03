#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

#include <clocale>
#include <cmath>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

struct NavPoint
{
    double x;
    double y;
    double yaw;
    std::string name;
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "back_and_forth_nav");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    NavPoint point_a;
    NavPoint point_b;

    // A点
    pnh.param("point_a_x", point_a.x, 0.75);
    pnh.param("point_a_y", point_a.y, 5.25);
    pnh.param("point_a_yaw", point_a.yaw, 0.0);
    point_a.name = "A点";

    // B点
    pnh.param("point_b_x", point_b.x, 0.0);
    pnh.param("point_b_y", point_b.y, 0.0);
    pnh.param("point_b_yaw", point_b.yaw, 0.0);
    point_b.name = "B点";

    double wait_after_arrival;
    double retry_delay;

    pnh.param("wait_after_arrival", wait_after_arrival, 0.2);
    pnh.param("retry_delay", retry_delay, 1.0);

    // 默认先去A点
    bool go_to_a = true;

    MoveBaseClient ac("move_base", true);

    ROS_INFO("等待 move_base action server...");

    while (ros::ok() &&
           !ac.waitForServer(ros::Duration(1.0)))
    {
        ROS_WARN_THROTTLE(
            5.0,
            "move_base 尚未启动，继续等待...");
    }

    if (!ros::ok())
    {
        return 0;
    }

    ROS_INFO("move_base 已连接。");
    ROS_INFO(
        "开始自动往返：A(%.3f, %.3f) <-> B(%.3f, %.3f)",
        point_a.x,
        point_a.y,
        point_b.x,
        point_b.y);

    while (ros::ok())
    {
        /*
         * 根据当前状态选择目标。
         */
        const NavPoint& target =
            go_to_a ? point_a : point_b;

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = target.x;
        goal.target_pose.pose.position.y = target.y;
        goal.target_pose.pose.position.z = 0.0;

        goal.target_pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(target.yaw);

        ROS_INFO(
            "前往%s：(%.3f, %.3f)，目标角度 %.1f°",
            target.name.c_str(),
            target.x,
            target.y,
            target.yaw * 180.0 / M_PI);

        /*
         * 发送目标。
         */
        ac.sendGoal(goal);

        /*
         * 等待当前目标结束。
         */
        ac.waitForResult();

        if (!ros::ok())
        {
            break;
        }

        actionlib::SimpleClientGoalState state =
            ac.getState();

        /*
         * 到达成功。
         */
        if (state ==
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO(
                "已到达%s：(%.3f, %.3f)",
                target.name.c_str(),
                target.x,
                target.y);

            /*
             * 关键：
             * 到A后下一次去B；
             * 到B后下一次去A。
             */
            go_to_a = !go_to_a;

            ROS_INFO(
                "下一个目标：%s",
                go_to_a ? "A点" : "B点");

            /*
             * 到达以后短暂停留。
             */
            if (wait_after_arrival > 0.0)
            {
                ros::Duration(
                    wait_after_arrival).sleep();
            }

            /*
             * 直接进入下一轮while。
             * 防止误入失败逻辑。
             */
            continue;
        }

        /*
         * 到这里说明绝对不是SUCCEEDED。
         */
        ROS_WARN(
            "前往%s失败，move_base状态=%s",
            target.name.c_str(),
            state.toString().c_str());

        ROS_WARN(
            "%.2f 秒后重新尝试%s。",
            retry_delay,
            target.name.c_str());

        /*
         * 失败时不切换A/B，
         * 继续尝试当前目标。
         */
        ros::Duration(retry_delay).sleep();
    }

    ac.cancelAllGoals();

    ROS_INFO("自动往返导航节点退出。");

    return 0;
}