/**
 * @file test_find_approach.cpp
 * @brief 旋转找板与逼近专属测试节点
 * @details 依次前往3个预设点进行旋转找板，找到后调用 forward_and_adjust 逼近
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

// 引入你原有的控制和定位头文件
#include "ucarmain2026/turn_detect.h"

// 定义 move_base 客户端
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// ---------------- 辅助函数：设置并发送 move_base 目标 ----------------
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
    
    ROS_INFO("----> 正在前往坐标: X=%.2f, Y=%.2f, Yaw=%.2f", x, y, yaw);
    ac.sendGoal(goal);
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("✔️ 成功到达目标点");
    else
        ROS_WARN("❌ 无法到达目标点");
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    ros::init(argc, argv, "test_find_approach_node");
    ros::NodeHandle nh;

    ROS_INFO("========== 找板与逼近测试节点启动 ==========");

    // 1. 初始化 MoveBase 客户端
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("等待 move_base 服务中...");
    }
    tf2::Quaternion q;
    move_base_msgs::MoveBaseGoal goal;

    // 2. 初始化底层控制器 (内部会等待视觉和雷达服务上线)
    ROS_INFO("正在初始化 MecanumController...");
    MecanumController mecanumController(nh);

    // ================= 配置测试参数 =================
    // 假设寻找 1 号板 (对应 yaml 里的第一个类别)
    int target_board_class = 1; 
    
    // 预设的 3 个找板坐标 (根据你的地图实际情况修改)
    double preset_x[3]   = {1.25, 1.25, 2.25};
    double preset_y[3]   = {2.75, 3.75, 4.25};
    double preset_yaw[3] = {0.0,  0.0,  0.0};
    // ==============================================

    bool task_success = false;

    // 唤醒摄像头预热 (-1 指令)
    std::vector<std::vector<int>> dummy_result = {{-1},{-1},{-1},{-1},{-1},{-1}};
    mecanumController.detect(dummy_result, -1);
    ros::Duration(2.0).sleep();

    // 3. 开始遍历预设点
    for (int i = 0; i < 3; i++) {
        ROS_INFO("--------------------------------------------------");
        ROS_INFO("🚗 准备前往第 %d 个预设找板点...", i + 1);
        
        // 导航过去
        go_destination(goal, preset_x[i], preset_y[i], preset_yaw[i], q, ac);

        // 清理摄像头缓存
        mecanumController.cap_buffer_clear();
        ROS_INFO("👀 开始旋转找板...");

        // turn_and_find_plus 引用传参所需的变量
        double targetx, targety, targetz, targetx2, targety2, targetz2;
        bool targetflag = false, target2flag = false, use_forward = false;

        // 调用旋转找板核心函数 (14秒超时时间, 旋转速度 0.4)
        bool found = mecanumController.turn_and_find_plus(
            14.0, target_board_class, 0.4, 
            targetx, targety, targetz, targetflag, 
            targetx2, targety2, targetz2, target2flag, 
            use_forward, 1
        );

        if (found) {
            ROS_INFO("🎯 成功在画面中锁定 %d 号目标板！", target_board_class);

            // 【遇障绕行逻辑】：如果雷达检测到前方有遮挡物不能直走
            if (!use_forward) {
                // 如果终点距离障碍物有一定的安全余量，先 move_base 绕行过去
                if (std::min(abs(targetx2 - 0), abs(targetx2 - 2.5)) > 0.4 && 
                    std::min(abs(targety2 - 2), abs(targety2 - 5)) > 0.4) {
                    
                    ROS_WARN("⚠️ 前方有障碍物遮挡，启动 move_base 绕行策略...");
                    go_destination(goal, targetx2, targety2, targetz2, q, ac);
                    
                    // 绕过去之后，需要重新对准一下板子
                    ROS_INFO("重新进行视觉对准...");
                    mecanumController.turn_and_find_plus(
                        8.0, target_board_class, 0.4, 
                        targetx, targety, targetz, targetflag, 
                        targetx2, targety2, targetz2, target2flag, 
                        use_forward, 1
                    );
                } else {
                    ROS_INFO("障碍物较远或无空间绕行，尝试直接视觉逼近。");
                }
            }

            // 【终极逼近逻辑】：调用去年被注释掉的 forward_and_adjust 贴脸停靠
            ROS_INFO("🚀 启动视觉雷达融合逼近...");
            // 参数 0.35 是逼近时的初始前进速度
            int final_board = mecanumController.forward_and_adjust(target_board_class, 0.35);

            if (final_board >= 0) {
                ROS_INFO("✅ 逼近完成！稳稳停在板前。最终确认类别 ID: %d", final_board);
            } else {
                ROS_WARN("⚠️ 逼近过程中目标丢失或道路彻底封死。");
            }
            
            task_success = true;
            break; // 只要找到了，就不再去后面的预设点了，直接跳出循环
            
        } else {
            ROS_INFO("❌ 第 %d 个点未发现目标，准备前往下一个预设点...", i + 1);
        }
    }

    // 4. 结算任务状态
    ROS_INFO("==================================================");
    if (!task_success) {
        // 如果 3 个点都跑完了还是没把 task_success 置为 true
        ROS_ERROR("任务失败");
    }

    // 关闭相机释放性能
    mecanumController.cap_close();

    return 0;
}