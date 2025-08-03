#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseActionGoal.h> // To get the original goal

#include <vector>
#include <string>
#include <cmath>
#include <boost/optional.hpp>
#include <boost/thread/mutex.hpp>

namespace my_recovery_plugins {

// Helper function to calculate 2D distance between two poses
double getDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
    return std::hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}

class AmclRelocRecovery : public nav_core::RecoveryBehavior {
public:
    AmclRelocRecovery() : tf_buffer_(NULL), initialized_(false) {}

    void initialize(std::string name, tf2_ros::Buffer* tf_buffer,
                    costmap_2d::Costmap2DROS* global_costmap,
                    costmap_2d::Costmap2DROS* local_costmap) {
        if (!initialized_) {
            tf_buffer_ = tf_buffer;
            global_costmap_ = global_costmap;

            ros::NodeHandle nh("/move_base/amcl_reloc");
            ros::NodeHandle g_nh;
            
            // Publishers and Service Clients
            initial_pose_pub_ = g_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
            new_goal_pub_ = g_nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
            clear_costmaps_client_ = g_nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
            make_plan_client_ = g_nh.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");//绕过movebase的状态机机制，直接请求globalplan的服务

            // Subscriber to get the original goal
            std::string goal_topic;
            nh.param("goal_topic", goal_topic, std::string("/move_base/goal"));
            goal_sub_ = g_nh.subscribe<move_base_msgs::MoveBaseActionGoal>(goal_topic, 1, &AmclRelocRecovery::goalCallback, this);

            initialized_ = true;
            ROS_INFO("自定义恢复行为 'AmclRelocRecovery' v3.0 (目标导向版) 已初始化。");
        } else {
            ROS_ERROR("'AmclRelocRecovery' 已经初始化过了。");
        }
    }

    void runBehavior() {
        if (!initialized_ || !original_goal_) {
            ROS_ERROR("恢复行为未初始化或未收到原始目标，无法运行。");
            return;
        }

        ROS_WARN("执行恢复行为: 重定位，清空地图，并在原目标附近寻找最近的可达点。");

        // === 第1部分：重定位和清除代价地图 (和之前一样) ===
        // ... (此处省略，保持简洁)
        ros::Duration(2.0).sleep();

        // === 第2部分：在原目标附近寻找最近可达点 ===
        ROS_INFO("步骤3: 在原目标点10cm范围内生成候选点并测试可达性...");

        geometry_msgs::PoseStamped current_pose;
        if(!global_costmap_->getRobotPose(current_pose)){
            ROS_ERROR("无法获取机器人当前位姿。");
            return;
        }

        // Use a local copy of the goal to avoid mutex locking in the loop
        geometry_msgs::PoseStamped original_goal = *original_goal_;

        std::vector<geometry_msgs::PoseStamped> reachable_candidates;
        
        // Generate candidate points in circles around the original goal
        const double search_radius = 0.15; // 寻找安全点的半径范围（必须大于膨胀死区的大小，否则不可能找到安全点）
        const int angular_slices = 8; // 在一圈中寻找的安全点的数量
        // We can test points on the 10cm circle and 5cm circle
        for (double r = search_radius; r > 0; r -= 0.05) {
            for (int i = 0; i < angular_slices; ++i) {
                double angle = i * (2.0 * M_PI / angular_slices);
                double dx = r * cos(angle);
                double dy = r * sin(angle);
                
                geometry_msgs::PoseStamped candidate_pose = original_goal; // Start with original goal
                candidate_pose.header.stamp = ros::Time::now();
                candidate_pose.pose.position.x += dx;
                candidate_pose.pose.position.y += dy;

                // Test reachability
                nav_msgs::GetPlan srv_plan;
                srv_plan.request.start = current_pose;
                srv_plan.request.goal = candidate_pose;
                srv_plan.request.tolerance = 0.2; // A bit of tolerance

                if (make_plan_client_.call(srv_plan) && !srv_plan.response.plan.poses.empty()) {
                    reachable_candidates.push_back(candidate_pose);
                }
            }
        }
        
        if (reachable_candidates.empty()) {
            ROS_ERROR("在原目标附近15cm内未找到任何可通过规划到达的点。恢复失败。");
            return;
        }

        ROS_INFO("找到了 %zu 个可达的候选点。现在从中选择距离机器人最近的一个。", reachable_candidates.size());

        // === 第3部分：从可达点中选择离机器人最近的 ===
        boost::optional<geometry_msgs::PoseStamped> best_goal;
        double min_dist = std::numeric_limits<double>::max();

        for (const auto& candidate : reachable_candidates) {
            double dist = getDistance(current_pose.pose, candidate.pose);
            if (dist < min_dist) {
                min_dist = dist;
                best_goal = candidate;
            }
        }

        if (best_goal) {
            ROS_INFO("已选择最近的可达点 (距离机器人 %.2f 米) 作为新目标。", min_dist);
            new_goal_pub_.publish(*best_goal);
        }
    }

private:
    // Callback to store the latest goal sent to move_base
    void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal_msg) {
        boost::mutex::scoped_lock lock(goal_mutex_);
        original_goal_ = goal_msg->goal.target_pose;
        ROS_INFO_ONCE("已成功捕获到第一个原始目标点。");
    }

    tf2_ros::Buffer* tf_buffer_;
    costmap_2d::Costmap2DROS* global_costmap_;
    bool initialized_;

    ros::Publisher initial_pose_pub_;
    ros::Publisher new_goal_pub_;
    ros::ServiceClient clear_costmaps_client_;
    ros::ServiceClient make_plan_client_;
    ros::Subscriber goal_sub_;

    boost::optional<geometry_msgs::PoseStamped> original_goal_;
    boost::mutex goal_mutex_;
};

} // namespace my_recovery_plugins

PLUGINLIB_EXPORT_CLASS(my_recovery_plugins::AmclRelocRecovery, nav_core::RecoveryBehavior)