#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <vector>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>

namespace my_planner 
{
    class MyPlanner : public nav_core::BaseLocalPlanner 
    {
        public:
            MyPlanner();
            ~MyPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();
        private:
            tf::TransformListener* tf_listener_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
    
            int target_index_;
            bool pose_adjusting_;
            bool goal_reached_;
            bool initial_rotation_done_;//用于判断初始旋转是否完成的标志
            
            // 存储从参数服务器读取的值的成员变量
            double path_linear_x_gain_, path_linear_y_gain_, path_angular_gain_, lookahead_dist_;
            double goal_dist_threshold_, final_pose_linear_gain_, final_pose_angular_gain_, goal_yaw_tolerance_,final_vel_min_;
            int collision_check_lookahead_points_;
            int visualization_scale_factor_;
            bool visualize_costmap_;
            
            double smoothed_angular_vel_ = 0.0;

            double a_;
            double k_;
            double angular_limit_;

            double z_pre_error;
            double diff_gain_;
            double alpha_; 
            double y_pre_error;
            double y_diff_gain_;

            double last_cmdvel_x_ = 0.5;
    };
} // namespace my_planner
 
#endif // MY_PLANNER_H_