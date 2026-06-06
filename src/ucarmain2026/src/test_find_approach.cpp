/**
 * @file test_find_approach.cpp
 * @brief 基于几何截距与 MoveBase 的绝对坐标停靠节点
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>

// 引入你的控制头文件
#include "ucarmain2026/turn_detect.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// ================== 辅助函数：发送 MoveBase 目标 ==================
void go_destination(move_base_msgs::MoveBaseGoal &goal, double x, double y, double yaw, tf2::Quaternion &q, MoveBaseClient &ac) {
    q.setRPY(0, 0, yaw);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
    
    ROS_INFO("----> 正在请求 move_base 前往坐标: [X: %.2f, Y: %.2f, Yaw: %.2f]", x, y, yaw);
    ac.sendGoal(goal);
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("✔️ 成功到达目标点");
    else
        ROS_WARN(" move_base 无法到达目标点");
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    ros::init(argc, argv, "test_find_approach_node");
    ros::NodeHandle nh;

    ROS_INFO("========== 基于几何截距的找板停靠测试启动 ==========");

    // 1. 初始化 MoveBase 客户端
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("等待 move_base 服务中...");
    }
    tf2::Quaternion q;
    move_base_msgs::MoveBaseGoal goal;

    // 2. 初始化底层控制器
    MecanumController mecanumController(nh);

    // ================= 配置测试参数 =================
    int target_board_class = 2; // 测试寻找的目标类别
    
    // 更新为你的新版预设点
    double preset_x[3]   = {1.25, 2.75, 3.75};
    double preset_y[3]   = {3.5, 3.5, 3.5};
    double preset_yaw[3] = {0.0,  0.0,  0.0};
    
    // 场地四边边界定义 (你的要求)
    double bound_x_min = 0.0;
    double bound_x_max = 5.0;
    double bound_y_min = 2.5;
    double bound_y_max = 4.5;
    // ==============================================

    bool task_success = false;

    // 唤醒摄像头
    std::vector<std::vector<int>> dummy_result = {{-1},{-1},{-1},{-1},{-1},{-1}};
    mecanumController.detect(dummy_result, -1);
    ros::Duration(2.0).sleep();

    // 3. 开始遍历预设点
    for (int i = 0; i < 3; i++) {
        ROS_INFO("--------------------------------------------------");
        ROS_INFO("前往第 %d 个预设点...", i + 1);
        
        go_destination(goal, preset_x[i], preset_y[i], preset_yaw[i], q, ac);

        mecanumController.cap_buffer_clear();
        ROS_INFO("到达预设点，开始原地旋转对准目标板...");

        double targetx, targety, targetz, targetx2, targety2, targetz2;
        bool targetflag = false, target2flag = false, use_forward = false;

        // 调用旋转找板，成功对准后车头会正对目标板
        bool found = mecanumController.turn_and_find_plus(
            14.0, target_board_class, 0.4, 
            targetx, targety, targetz, targetflag, 
            targetx2, targety2, targetz2, target2flag, 
            use_forward, 1
        );

        if (found) {
            ROS_INFO("成功锁定目标！开始计算绝对停靠坐标...");

            // 获取当前车体在地图中的绝对坐标和朝向
            std::vector<float> cur_pose = mecanumController.getCurrentPose();
            double cur_x = cur_pose[0];
            double cur_y = cur_pose[1];
            double cur_yaw = cur_pose[2]; // 这个朝向现在正是对准墙壁目标板的方向

            ROS_INFO("当前车体坐标: [X: %.3f, Y: %.3f, Yaw: %.3f rad]", cur_x, cur_y, cur_yaw);

            // ---------------------------------------------------------
            // 【核心算法】：射线与矩形场地边界的交点计算
            // ---------------------------------------------------------
            double t_min = 9999.0; // 射线参数 t
            int hit_wall = -1;     // 记录打中了哪一面墙：0=右, 1=左, 2=上, 3=下
            
            // 1. 尝试与 x = bound_x_max (右墙 x=5) 相交
            if (cos(cur_yaw) > 1e-4) {
                double t = (bound_x_max - cur_x) / cos(cur_yaw);
                if (t > 0 && t < t_min) { t_min = t; hit_wall = 0; }
            }
            // 2. 尝试与 x = bound_x_min (左墙 x=0) 相交
            else if (cos(cur_yaw) < -1e-4) {
                double t = (bound_x_min - cur_x) / cos(cur_yaw);
                if (t > 0 && t < t_min) { t_min = t; hit_wall = 1; }
            }

            // 3. 尝试与 y = bound_y_max (上墙 y=4.5) 相交
            if (sin(cur_yaw) > 1e-4) {
                double t = (bound_y_max - cur_y) / sin(cur_yaw);
                if (t > 0 && t < t_min) { t_min = t; hit_wall = 2; }
            }
            // 4. 尝试与 y = bound_y_min (下墙 y=2.5) 相交
            else if (sin(cur_yaw) < -1e-4) {
                double t = (bound_y_min - cur_y) / sin(cur_yaw);
                if (t > 0 && t < t_min) { t_min = t; hit_wall = 3; }
            }

            if (t_min == 9999.0 || hit_wall == -1) {
                ROS_ERROR("计算交点失败：射线未能打中边界！(这在数学上基本不可能)");
                continue; // 跳过当前点
            }

            // 计算目标板在墙上的中心坐标 (交点)
            double board_x = cur_x + t_min * cos(cur_yaw);
            double board_y = cur_y + t_min * sin(cur_yaw);
            ROS_INFO("=====> 📐 计算得出墙上目标板坐标为: [X: %.3f, Y: %.3f]", board_x, board_y);

            // ---------------------------------------------------------
            // 【终极停靠点】：强制正交贴靠 (垂直于墙面)
            // ---------------------------------------------------------
            double safe_distance = 0.3; // 距离墙面的安全停车距离 (可根据车体长度微调)
            double dock_x = board_x;
            double dock_y = board_y;
            double dock_yaw = 0.0;

            if (hit_wall == 0) { 
                // 打中右墙 (x = 5.0)，小车应该向右停靠
                dock_x = bound_x_max - safe_distance;
                dock_yaw = 0.0;
                ROS_INFO("👉 目标在右墙 (X=5.0)，小车将正向朝右贴靠");
            } else if (hit_wall == 1) { 
                // 打中左墙 (x = 0.0)，小车应该向左停靠
                dock_x = bound_x_min + safe_distance;
                dock_yaw = 3.1415926; // 180度，朝左
                ROS_INFO("👈 目标在左墙 (X=0.0)，小车将正向朝左贴靠");
            } else if (hit_wall == 2) { 
                // 打中上墙 (y = 4.5)，小车应该向上停靠
                dock_y = bound_y_max - safe_distance;
                dock_yaw = 1.5707963; // 90度，朝上
                ROS_INFO("👆 目标在上墙 (Y=4.5)，小车将正向朝上贴靠");
            } else if (hit_wall == 3) { 
                // 打中下墙 (y = 2.5)，小车应该向下停靠
                dock_y = bound_y_min + safe_distance;
                dock_yaw = -1.5707963; // -90度，朝下
                ROS_INFO("👇 目标在下墙 (Y=2.5)，小车将正向朝下贴靠");
            }
            
            ROS_INFO("=====> 🅿️ 最终垂直安全停靠点为: [X: %.3f, Y: %.3f, Yaw: %.2f]", dock_x, dock_y, dock_yaw);
            ROS_INFO("--------------------------------------------------");

            // 执行精准停靠！(使用强制矫正后的 Yaw)
            ROS_INFO("🚀 发送停靠坐标至 move_base，执行最终平滑贴靠...");
            go_destination(goal, dock_x, dock_y, dock_yaw, q, ac);

            task_success = true;
            break; // 找到了且停靠完毕，跳出循环
            
        } else {
            ROS_INFO("此预设点未发现目标，准备前往下一个...");
        }
    }

    if (!task_success) {
        ROS_ERROR("任务失败：遍历所有点均未找到目标板。");
    }

    mecanumController.cap_close();
    return 0;
}