#include "my_planner.h"
#include <tf/tf.h>//沟槽的move_base源码用的TF1，这里用TF2就会导致重启时odom,amcl,globalplan出现一堆莫名其妙的错误，甚至与启动位置有关，原理不详，疑似机魂不悦
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

namespace my_planner 
{
    // 构造函数：初始化指针成员为nullptr
    MyPlanner::MyPlanner() : tf_listener_(nullptr), costmap_ros_(nullptr), target_index_(0), pose_adjusting_(false), goal_reached_(false), initial_rotation_done_(false)//初始化指针和变量，防止在某些情况下触发exit code = -11的段错误
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {
        ROS_WARN(">>> 析构函数被调用 <<<");//
        // 检查指针是否有效，然后释放它
        if(tf_listener_ != nullptr)
        {
            delete tf_listener_;
            tf_listener_ = nullptr; // 释放后置空，防止悬挂指针
        }
    }
    
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("本地规划器，启动！");
        if (!tf || !costmap_ros) {
            ROS_FATAL("致命错误: TF Buffer 或 CostmapROS 的指针为空!");
            throw std::runtime_error("TF Buffer 或 CostmapROS 的指针为空!");
        }
        
        if(tf_listener_ == nullptr)
        {
            tf_listener_ = new tf::TransformListener();
        }

        costmap_ros_ = costmap_ros;
        ROS_INFO("指针检查通过，tf_buffer 和 costmap_ros 已接收。");

        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        if (!costmap) {
            ROS_FATAL("致命错误: 代价地图尚未初始化!");
            throw std::runtime_error("代价地图尚未初始化!");
        }
        ROS_INFO("代价地图功能检查通过，尺寸: %d x %d。", costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
        
        ros::NodeHandle private_nh("/move_base/MyPlanner" );

        ROS_INFO("为 %s 加载参数...", name.c_str());

        private_nh.param("path_linear_x_gain", path_linear_x_gain_, 2.0);
        private_nh.param("path_linear_y_gain", path_linear_y_gain_, 0.5);
        private_nh.param("path_angular_gain", path_angular_gain_, 6.8);
        private_nh.param("angular_limit", angular_limit_, 0.08); 
        private_nh.param("lookahead_dist", lookahead_dist_, 0.2);
        
        private_nh.param("goal_dist_threshold", goal_dist_threshold_, 0.10);
        private_nh.param("final_pose_linear_gain", final_pose_linear_gain_, 2.5);
        private_nh.param("final_pose_angular_gain", final_pose_angular_gain_, 1.5);
        private_nh.param("final_vel_min", final_vel_min_, 0.2);
        private_nh.param("goal_yaw_tolerance", goal_yaw_tolerance_, 0.1);

        private_nh.param("collision_check_lookahead_points", collision_check_lookahead_points_, 10);
        private_nh.param("visualization_scale_factor", visualization_scale_factor_, 5);
        private_nh.param("visualize_costmap", visualize_costmap_, false);

        //------------------------------------用于动态速度控制的参数-----------------------------
        private_nh.param("a", a_, 7.0);
        private_nh.param("k", k_, -25.0); 
        private_nh.param("diff_gain", diff_gain_, 0.0);
        private_nh.param("alpha", alpha_, 0.7);
        private_nh.param("y_diff_gain", y_diff_gain_, 1.0);
        initial_rotation_done_ = false;
    }

    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        //每次设置新路径时，都将“初始旋转”标志重置为 false,确保下一次导航会执行初始旋转调整朝向
        initial_rotation_done_ = false;
        ROS_INFO("查看全局路径规划,一共%zu个点",global_plan_.size());
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(global_plan_.empty()) return false;

        // =====================================================================
        // [新增终极补丁]：基于局部代价地图的“参考线梯度治愈” (Gradient-based Path Healing)
        // 在所有计算开始前，先把前方即将查阅的全局路径点强行推到走廊正中心！
        // =====================================================================
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        
        // 为了节省算力，只治愈小车前方 target_index 附近的点
        int heal_start = std::max(0, target_index_ - 5);
        int heal_end = std::min((int)global_plan_.size(), target_index_ + 60);

        for(int i = heal_start; i < heal_end; i++) {
            geometry_msgs::PoseStamped pt_map;
            global_plan_[i].header.stamp = ros::Time(0);
            try {
                // 将路径点统一转换到 map 坐标系去读取代价地图
                tf_listener_->transformPose("map", global_plan_[i], pt_map);
                double wx = pt_map.pose.position.x;
                double wy = pt_map.pose.position.y;

                for(int iter = 0; iter < 3; iter++) { // 迭代3次，弹性拉伸
                    unsigned int mx, my;
                    if(costmap->worldToMap(wx, wy, mx, my)) {
                        int cost = costmap->getCost(mx, my);
                        // 只对靠近障碍物的点进行推挤 (防止在空旷区域漂移)
                        if(cost > 0 && cost < 253) {
                            unsigned int size_x = costmap->getSizeInCellsX();
                            unsigned int size_y = costmap->getSizeInCellsY();
                            if (mx > 0 && mx < size_x - 1 && my > 0 && my < size_y - 1) {
                                // 探针：获取周围十字区域的代价值
                                int cost_up = costmap->getCost(mx, my + 1);
                                int cost_down = costmap->getCost(mx, my - 1);
                                int cost_left = costmap->getCost(mx - 1, my);
                                int cost_right = costmap->getCost(mx + 1, my);

                                // 梯度下降：寻找代价值下降最快的方向
                                double grad_x = (cost_left - cost_right);
                                double grad_y = (cost_down - cost_up);
                                double norm = std::hypot(grad_x, grad_y);
                                
                                if (norm > 0.01) {
                                    double push_step = 0.02; // 每次推动 2 厘米
                                    wx += (grad_x / norm) * push_step;
                                    wy += (grad_y / norm) * push_step;
                                }
                            }
                        }
                    }
                }
                pt_map.pose.position.x = wx;
                pt_map.pose.position.y = wy;
                
                // 将治愈后的完美坐标覆盖回 global_plan_ (后续所有纯跟踪逻辑都会吃到红利)
                tf_listener_->transformPose(global_plan_[i].header.frame_id, pt_map, global_plan_[i]);
            } catch (tf::TransformException &ex) {
                // 如果TF不可用，忽略该点保留原样
                continue; 
            }
        }
        // =====================================================================


        int final_index = global_plan_.size()-1;
        if(visualize_costmap_)//可视化分支：opencv绘制代价地图
        {
            unsigned char* map_data = costmap->getCharMap();
            unsigned int size_x = costmap->getSizeInCellsX();
            unsigned int size_y = costmap->getSizeInCellsY();

            cv::Mat map_image;
            map_image.create(size_y, size_x, CV_8UC3);
            for (unsigned int y = 0; y < size_y; y++)
            {
                for (unsigned int x = 0; x < size_x; x++)
                {
                    int map_index = y * size_x + x;
                    unsigned char cost = map_data[map_index];               
                    cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);  
                
                    if (cost == 0)          // 可通行区域
                        pixel = cv::Vec3b(128, 128, 128); 
                    else if (cost == 254)   // 障碍物
                        pixel = cv::Vec3b(0, 0, 0);       
                    else if (cost == 253)   // 禁行区域 
                        pixel = cv::Vec3b(255, 255, 0);   
                    else
                    {
                        unsigned char blue = 255 - cost;
                        unsigned char red = cost;
                        pixel = cv::Vec3b(blue, 0, red);
                    }
                }
            }
            
            cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
            for(int i=0;i<global_plan_.size();i++)
            {
                geometry_msgs::PoseStamped pose_in_map;
                global_plan_[i].header.stamp = ros::Time(0);
                tf_listener_->transformPose("map",global_plan_[i],pose_in_map);
                double map_x = pose_in_map.pose.position.x;
                double map_y = pose_in_map.pose.position.y;

                double origin_x = costmap->getOriginX();
                double origin_y = costmap->getOriginY();
                double local_x = map_x - origin_x;
                double local_y = map_y - origin_y;
                int x = local_x / costmap->getResolution();
                int y = local_y / costmap->getResolution();
                cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(255,0,255));

                if(i >= target_index_ && i < target_index_ + collision_check_lookahead_points_)
                {
                    cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(0,255,255));
                    int map_index = y * size_x + x;
                    unsigned char cost = map_data[map_index];
                    if(cost >= 253)
                    {
                        ROS_INFO("重新规划路径");
                        initial_rotation_done_ = false;
                        return false;
                    }
                }
            }

            map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0); 
            
            flipped_image = cv::Mat(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
            cv::flip(map_image, map_image, 0);
            if(visualize_costmap_)
            {
                cv::namedWindow("Map");
                cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST);
                cv::resizeWindow("Map", size_y*5, size_x*5);
                cv::imshow("Map", map_image);
            }
        }
        else
        {
            unsigned char* map_data = costmap->getCharMap();
            unsigned int size_x = costmap->getSizeInCellsX();
            unsigned int size_y = costmap->getSizeInCellsY();
            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double resolution = costmap->getResolution();

            int check_end_index = std::min((int)(target_index_ + collision_check_lookahead_points_), (int)global_plan_.size());
            for(int i = target_index_; i < check_end_index; i++)
            {
                geometry_msgs::PoseStamped pose_in_map;
                global_plan_[i].header.stamp = ros::Time(0);
                tf_listener_->transformPose("map", global_plan_[i], pose_in_map);
                double map_x = pose_in_map.pose.position.x;
                double map_y = pose_in_map.pose.position.y;

                unsigned int cell_x, cell_y;
                if (!costmap->worldToMap(map_x, map_y, cell_x, cell_y))
                {
                    ROS_WARN("Path point (%.2f, %.2f) is outside the costmap.", map_x, map_y);
                    continue; 
                }

                unsigned int map_index = costmap->getIndex(cell_x, cell_y);
                unsigned char cost = map_data[map_index];

                if(cost >= 253)
                {
                    ROS_WARN("路径上有障碍,开始检查最终目标点状态");
                    
                    geometry_msgs::PoseStamped final_goal_pose_in_map;
                    global_plan_.back().header.stamp = ros::Time(0);
                    try {
                        tf_listener_->transformPose("map", global_plan_.back(), final_goal_pose_in_map);
                    } catch (tf::TransformException &ex) {
                        ROS_ERROR("TF transform failed for final goal: %s", ex.what());
                        return false; 
                    }

                    unsigned int final_cell_x, final_cell_y;
                    if (costmap->worldToMap(final_goal_pose_in_map.pose.position.x, final_goal_pose_in_map.pose.position.y, final_cell_x, final_cell_y))
                    {
                        unsigned int final_map_index = costmap->getIndex(final_cell_x, final_cell_y);
                        unsigned char final_goal_cost = map_data[final_map_index];

                        if (final_goal_cost >= 253)
                        {
                            ROS_WARN("最终目标点 (cost=%d) 本身不可达. 从后向前搜索新的有效终点.", final_goal_cost);
                            geometry_msgs::Quaternion original_final_orientation = global_plan_.back().pose.orientation;

                            for (int j = global_plan_.size() - 2; j >= 0; --j)
                            {
                                geometry_msgs::PoseStamped backtrack_pose_in_map;
                                global_plan_[j].header.stamp = ros::Time(0);
                                try {
                                    tf_listener_->transformPose("map", global_plan_[j], backtrack_pose_in_map);
                                } catch (tf::TransformException &ex) {
                                    continue;
                                }

                                unsigned int backtrack_cell_x, backtrack_cell_y;
                                if (costmap->worldToMap(backtrack_pose_in_map.pose.position.x, backtrack_pose_in_map.pose.position.y, backtrack_cell_x, backtrack_cell_y))
                                {
                                    unsigned int backtrack_map_index = costmap->getIndex(backtrack_cell_x, backtrack_cell_y);
                                    unsigned char backtrack_cost = map_data[backtrack_map_index];
                                    if (backtrack_cost < 253)
                                    {
                                        ROS_INFO("找到新的可行终点，索引为 %d. 截断全局路径并继续执行.", j);
                                        global_plan_.resize(j + 1);
                                        global_plan_.back().pose.orientation = original_final_orientation;
                                        goal_reached_ = false;
                                        pose_adjusting_ = false;
                                        target_index_ = std::min(target_index_, (int)global_plan_.size() - 1);
                                        goto continue_execution; 
                                    }
                                }
                            }
                            ROS_ERROR("在整个路径中都未找到代价值小于253的有效终点. 放弃并请求重规划.");
                            return false;
                        }
                        else
                        {
                            ROS_INFO("路径中有障碍，但终点可达。请求全局重规划以绕行。");
                            return false;
                        }
                    }
                    else
                    {
                        ROS_WARN("最终目标点位于代价地图之外. 请求重规划.");
                        return false;
                    }
                }
            }
        continue_execution:;
        final_index = global_plan_.size()-1;
        }

        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);

        if(pose_adjusting_ == false)
        {
            if(final_index-target_index_<50)
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);

            if(pose_final.pose.position.x>0.03) cmd_vel.linear.x = std::max(cmd_vel.linear.x-0.1,0.2);
            else if(pose_final.pose.position.x<-0.03) cmd_vel.linear.x = std::min(cmd_vel.linear.x+0.1,-0.2);
            else cmd_vel.linear.x = 0;
            
            if(pose_final.pose.position.y>0.03) cmd_vel.linear.y = std::max(cmd_vel.linear.y-0.15,0.2);
            else if(pose_final.pose.position.y<-0.03) cmd_vel.linear.y = std::min(cmd_vel.linear.y-0.15,-0.2);
            else cmd_vel.linear.y = 0;

            if(final_yaw>0.04) cmd_vel.angular.z = std::max(std::min(final_yaw * final_pose_angular_gain_,2.5),0.45);
            else if(final_yaw<-0.04) cmd_vel.angular.z = std::min(std::max(final_yaw * final_pose_angular_gain_,-2.5),-0.45);
            else cmd_vel.angular.z = 0;

            if(cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0 && cmd_vel.angular.z == 0)
            {
                goal_reached_ = true;
                ROS_WARN("到达终点！");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                initial_rotation_done_ = false;
            }
            return true;
        }

        geometry_msgs::PoseStamped target_pose;
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist > lookahead_dist_) 
            {
                target_pose = pose_base;
                target_index_ = i;
                break;
            }

            if(i == global_plan_.size()-1){
                target_pose = pose_base; 
            }
        }
        
        if(global_plan_[0].pose.position.x < 2.4 &&global_plan_[0].pose.position.y <1.1) initial_rotation_done_ = true;
        if (!initial_rotation_done_) 
        {
            double angle_to_target = atan2(target_pose.pose.position.y, target_pose.pose.position.x);
            
            if (std::abs(angle_to_target) < goal_yaw_tolerance_) {
                ROS_INFO("初始姿态已对准，设置标志位并开始正常行驶。");
                initial_rotation_done_ = true;
            } else {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                if(angle_to_target>0) cmd_vel.angular.z = std::min(std::max(angle_to_target * final_pose_angular_gain_,0.8),2.5); 
                else cmd_vel.angular.z = std::max(std::min(angle_to_target * final_pose_angular_gain_,-0.8),-2.5);
                return true; 
            }
        }

        double avrage_curvature = 0.0001;
        double dynamic_x_gain = path_linear_x_gain_; 

        int cur_final = std::min(target_index_+15,final_index);
        int cur_start = std::max(target_index_ - 15 , 0 );
        if (target_index_ >= 5) 
        {
            for (int i = cur_start; i < cur_final;i+=5)
            {
                const auto& p0 = global_plan_[i-5].pose.position;
                const auto& p1 = global_plan_[i].pose.position;
                const auto& p2 = global_plan_[i+5].pose.position;
                
                double ux = p1.x - p0.x, uy = p1.y - p0.y;
                double vx = p2.x - p1.x, vy = p2.y - p1.y;

                double norm_u = std::sqrt(ux*ux + uy*uy);
                double norm_v = std::sqrt(vx*vx + vy*vy);
                double w_norm = std::sqrt(std::pow(p2.x - p0.x, 2) + std::pow(p2.y - p0.y, 2));

                double cross_product_mag = std::abs(ux * vy - uy * vx);

                if (norm_u > 1e-6 && norm_v > 1e-6 && w_norm > 1e-6) {
                    double curvature = (2.0 * cross_product_mag) / (norm_u * norm_v * w_norm);
                    avrage_curvature += curvature*(1.5-(i-cur_start)/(cur_final-cur_start));
                }
            }
        }
        avrage_curvature = avrage_curvature/(cur_final - cur_start)*5;
        
        dynamic_x_gain = path_linear_x_gain_  * exp((avrage_curvature-a_)/k_);
        dynamic_x_gain = std::max(3.5, dynamic_x_gain); 

        std::vector<cv::Point2f> slope_points;
        for (int j = cur_start; j < cur_final; j+=3)
        {
            geometry_msgs::PoseStamped point_in_base;
            global_plan_[j].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link", global_plan_[j], point_in_base);
            slope_points.push_back(cv::Point2f(point_in_base.pose.position.x,point_in_base.pose.position.y));
        }
        cv::Vec4f line_params;
        cv::fitLine(slope_points, line_params, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line_params[0], vy = line_params[1];
        if(line_params[0]<0){
            vx *= -1;
            vy *= -1;
        }
        float angle_rad = std::atan2(vy, vx);

        cur_final = std::min(target_index_+(int)(last_cmdvel_x_*100),final_index);
        cur_start = std::max(target_index_ - 15 , 0 );
        std::vector<cv::Point2f> sampled_points;
        double line_error = 1.0;

        if (target_index_ >= 5) 
        {
            for (int i = cur_start; i < cur_final;i+=5)
            {
                cv::Point2f point(global_plan_[i].pose.position.x,global_plan_[i].pose.position.y);
                sampled_points.push_back(point);
            }
            if (sampled_points.size() > 5) {
                cv::Vec4f line_params;
                cv::fitLine(sampled_points, line_params, cv::DIST_L2, 0, 0.01, 0.01);
                float vx = line_params[0];
                float vy = line_params[1];
                float x0 = line_params[2];
                float y0 = line_params[3];
                int total_point = (int)sampled_points.size();
                float sum_residual = 0.0f,max_residual = 0.0f;
                int valid_points = 0;
                
                for(int j=0;j<total_point;j++){
                    cv::Point2f point = sampled_points[j];
                    float dist = std::abs((point.y - y0) * vx - (point.x - x0) * vy);
                    
                    if (dist < 10.0f) { 
                        if(dist>max_residual){
                            max_residual = dist;
                        }
                        sum_residual += dist*(1.5-j/(float)total_point);
                        valid_points++;
                    }
                }
                float avg_residual = sum_residual / valid_points;
                line_error = exp(-40*avg_residual);
            }
        }
        dynamic_x_gain = dynamic_x_gain*line_error;

        double min_y_deviation = target_pose.pose.position.y; 
        int search_start_index = std::max(0, (int)target_index_ - 10);
        
        for (int j = search_start_index; j < target_index_; ++j)
        {
            geometry_msgs::PoseStamped point_in_base;
            geometry_msgs::PoseStamped plan_point = global_plan_[j];
            plan_point.header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link", plan_point, point_in_base);
            if (std::abs(point_in_base.pose.position.y) < std::abs(min_y_deviation))
            {
                min_y_deviation = point_in_base.pose.position.y;
            }
        }
        
        cmd_vel.linear.x = target_pose.pose.position.x * dynamic_x_gain;
        // ROS_INFO("前进速度是%f",cmd_vel.linear.x);
        cmd_vel.linear.y = min_y_deviation * path_linear_y_gain_ + (min_y_deviation - y_pre_error) * y_diff_gain_;
        
        last_cmdvel_x_ = cmd_vel.linear.x;
        
        double raw_angular_vel = angle_rad * path_angular_gain_ + (angle_rad-z_pre_error)*diff_gain_;
        z_pre_error = angle_rad;
        smoothed_angular_vel_ = alpha_ * raw_angular_vel + (1.0 - alpha_) * smoothed_angular_vel_;
        cmd_vel.angular.z = smoothed_angular_vel_;
        
        return true;
    }

    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
}
