#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

namespace my_recovery_plugins {

class AmclRelocRecovery : public nav_core::RecoveryBehavior {
public:
    AmclRelocRecovery() : global_costmap_(NULL), local_costmap_(NULL), initialized_(false) {}

    void initialize(std::string name, tf2_ros::Buffer*,
                    costmap_2d::Costmap2DROS* global_costmap,
                    costmap_2d::Costmap2DROS* local_costmap) {
        if (!initialized_) {
            global_costmap_ = global_costmap;
            local_costmap_ = local_costmap;

            // 创建节点句柄
            ros::NodeHandle nh("/move_base/amcl_reloc");

            // 初始化 /initialpose 话题的发布者
            initial_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

            // 初始化清除代价地图服务的客户端
            clear_costmaps_client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

            initialized_ = true;
            ROS_INFO("自定义恢复行为 'AmclRelocRecovery' 已初始化。");
        } else {
            ROS_ERROR("自定义恢复行为 'AmclRelocRecovery' 已经初始化过了，请勿重复操作。");
        }
    }

    void runBehavior() {
        if (!initialized_) {
            ROS_ERROR("自定义恢复行为 'AmclRelocRecovery' 尚未初始化，无法运行。");
            return;
        }

        ROS_WARN("执行恢复行为: AMCL重定位并清除代价地图。");

        // 1. 获取机器人当前的位姿和方差
        ROS_INFO("步骤1: 等待获取 /amcl_pose...");
        boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> current_pose_msg;
        try {
            current_pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
                "/amcl_pose", ros::Duration(2.0)
            );
            if (!current_pose_msg) {
                ROS_ERROR("获取 /amcl_pose 超时，无法执行重定位。");
                return;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("获取 /amcl_pose 时发生异常: %s", e.what());
            return;
        }
        
        ROS_INFO("成功获取当前AMCL位姿。");

        // 2. 调用AMCL的重新定位服务 (通过发布到 /initialpose)
        ROS_INFO("步骤2: 将当前位姿作为初始位姿发布，触发AMCL重定位...");
        initial_pose_pub_.publish(*current_pose_msg);

        // 等待一小段时间，给AMCL一些时间来处理新的初始位姿并更新其粒子云
        ros::Duration(1.0).sleep();
        ROS_INFO("已发布初始位姿，AMCL应该正在重新收敛。");

        // 3. 调用清除代价地图的服务
        ROS_INFO("步骤3: 请求清除全局和局部代价地图...");
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("成功清除了 move_base 的代价地图。");
        } else {
            ROS_ERROR("清除 move_base 的代价地图失败。请检查 /move_base/clear_costmaps 服务是否可用。");
        }
    }

    ~AmclRelocRecovery() {}

private:
    costmap_2d::Costmap2DROS* global_costmap_;
    costmap_2d::Costmap2DROS* local_costmap_;
    bool initialized_;
    ros::Publisher initial_pose_pub_;
    ros::ServiceClient clear_costmaps_client_;
};

} // namespace my_recovery_plugins

// 使用pluginlib宏将我们的类注册为 nav_core::RecoveryBehavior 插件
PLUGINLIB_EXPORT_CLASS(my_recovery_plugins::AmclRelocRecovery, nav_core::RecoveryBehavior)