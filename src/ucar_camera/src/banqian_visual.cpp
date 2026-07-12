#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

// 全局变量存储速度指令
double twist_linear_x;
double twist_angular_z;

void detectionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 参数配置
    const double img_width = 1280.0;
    const double img_height = 720.0;
    const double MAX_LINEAR = 0.5;   // 最大线速度(m/s)
    const double MAX_ANGULAR = 0.5;  // 最大角速度(rad/s)
    
    // 转向控制参数
    const double SMALL_ERROR_THRESHOLD = 50.0;  // 小误差阈值（像素）
    const double LARGE_ERROR_THRESHOLD = 200.0; // 大误差阈值（像素）
    const double SMALL_ERROR_GAIN = 0.001;      // 小误差增益
    const double LARGE_ERROR_GAIN = 0.02;       // 大误差增益

    // 数据校验
    if (msg->data.size() < 5) {
        ROS_WARN_THROTTLE(1.0, "Invalid detection data! Size=%zu", msg->data.size());
        return;
    }

    double x1 = msg->data[1], y1 = msg->data[2];
    double x2 = msg->data[3], y2 = msg->data[4];

    // 计算控制参数
    double center_error = (x1 + x2)/2.0 - img_width/2.0;
    double area_ratio = (x2-x1)*(y2-y1)/(img_width*img_height);
    double center_error_percent = (center_error / img_width) * 100.0;
    
    // 动态调整转向增益（误差越大转向越强）
    double error_abs = fabs(center_error);
    double angular_gain = SMALL_ERROR_GAIN;  // 默认小误差增益
    
    if (error_abs > LARGE_ERROR_THRESHOLD) {
        angular_gain = LARGE_ERROR_GAIN;
    } 
    else if (error_abs > SMALL_ERROR_THRESHOLD) {
        // 线性插值
        double t = (error_abs - SMALL_ERROR_THRESHOLD) / 
                  (LARGE_ERROR_THRESHOLD - SMALL_ERROR_THRESHOLD);
        angular_gain = SMALL_ERROR_GAIN + t * (LARGE_ERROR_GAIN - SMALL_ERROR_GAIN);
    }
    
    // 生成控制指令
    twist_angular_z = -angular_gain * center_error;  // 动态增益转向控制
    twist_linear_x = 0.1 * (1.0 - area_ratio);      // 前进控制
    
    // ========== 调试信息输出 ==========
    // 1. 目标框坐标信息
    // ROS_INFO("目标框坐标: 左上(%.1f,%.1f) 右下(%.1f,%.1f)", x1, y1, x2, y2);
    
    // 2. 面积和中心误差信息（保留面积占比打印）
    ROS_INFO("目标信息: 面积占比=%.1f%%, 中心偏移=%.1fpx (%.1f%%)", 
             area_ratio * 100.0, 
             center_error,
             center_error_percent);
    
    // 3. 控制参数信息
    ROS_INFO("控制参数: 动态增益=%.4f", angular_gain);

    // 速度限制
    twist_linear_x = std::max(0.0, std::min(twist_linear_x, MAX_LINEAR));
    twist_angular_z = std::max(-MAX_ANGULAR, std::min(twist_angular_z, MAX_ANGULAR));

    // 接近目标时减速
    if (area_ratio > 0.05) {
        twist_linear_x *= 0.5;
        twist_angular_z *= 0.5;
    }

    // 完全停止条件
    if (area_ratio > 0.15) {
        twist_linear_x = 0;
        twist_angular_z = 0;
        ROS_WARN_THROTTLE(1.0, "Reached target! Area: %.1f%%", area_ratio*100);
    }

    // 4. 最终控制指令输出
    ROS_INFO_THROTTLE(0.4, "控制指令: 线速度=%.3f m/s, 角速度=%.3f rad/s", 
                     twist_linear_x, twist_angular_z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_follower");
    ros::NodeHandle nh;

    // 订阅检测结果
    ros::Subscriber detect_sub = nh.subscribe("/nanodet/detect", 10, detectionCallback);
    
    // 发布控制指令
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("Simple follower initialized");

    while (ros::ok()) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = twist_linear_x;
        cmd.angular.z = twist_angular_z;
        cmd_pub.publish(cmd);
        ros::spinOnce();
    }
    return 0;
}

// 带雷达
// 到中央白框，navTogoal意思是导航到这个白框目标点，然后Point_number++，指向下一个坐标点
// void Robot::toMidFrame() {
//      ROS_INFO("------ 开始toMidFrame ------");
//      navToGoal(Point_number);
//      Point_number++;
// }

// //找版，储存，通常用于发布多个物体的位姿
// void Robot::ydCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
//     ob_num = msg->poses.size();
//     ROS_INFO("ob_num = %d", ob_num);
//     for (int j = 0; j < ob_num; j++) {
//         goal_p_list[j][0] = msg->poses[j].position.x;
//         goal_p_list[j][1] = msg->poses[j].position.y;
//         goal_p_list[j][2] = msg->poses[j].orientation.z;
//         goal_p_list[j][3] = msg->poses[j].orientation.w;
//     }
//     callback_triggered = true;  // 设置回调触发标志
//     ROS_INFO("race_all get a goal_p_list 0 is: (%lf, %lf)", goal_p_list[0][0], goal_p_list[0][1]);
//     cond_var_.notify_all();    // 通知主线程数据已更新
// }



// void Robot::viCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(mutex_); 
//     // 更新识别到的物品
//     std::vector<double> vi_msg = msg->data;
//     int lenofMsg = msg->data.size(); 
//     for(int i = 0; i < (lenofMsg - 1)/5; ++i) {
//         rething = class_names[static_cast<int>(msg->data[5*i])];
//         target_found = (
//             (thing == "Dessert"   && (rething == "Cake" || rething == "Milk" || rething == "Cola")) ||
//             (thing == "Fruit"     && (rething == "Watermelon" || rething == "Apple" || rething == "Banana")) ||
//             (thing == "Vegetable" && (rething == "Chili" || rething == "Tomato" || rething == "Potato"))
//         );
//         if (target_found) {
//             double Px1 = msg->data[5*i+1];
//             double Px2 = msg->data[5*i+3];
//             target_pixel_x = (Px1 + Px2) / 2;  
//         }
//     }
// }


// void Robot::collectPic() {
//      ROS_INFO("-------- 开始 collectPic 任务, 目标类别: '%s' --------", thing.c_str());
//      // 1. 初始化

//      ros::param::set("/detect", 1);
//      yd_control.data = 0;
//      yd_loc_control.publish(yd_control);

//      yd_sub = nh_.subscribe<geometry_msgs::PoseArray>("/yd_msg", 1, &Robot::ydCallback, this);
//      vi_sub = nh_.subscribe<std_msgs::Float64MultiArray>("/nanodet/detect", 1, &Robot::viCallback, this);

//      tf::TransformListener listener;
//      bool parked_successfully = false;
//      ros::Rate rate(30); // 提高频率以获得更及时的响应
//      std::set<std::string> class_thing;

//      // 2. 主循环
//      while (ros::ok() && !parked_successfully) {
        
//         // --- 阶段一：粗略旋转搜索 ---
//          ROS_INFO("阶段1:正在旋转以寻找 '%s' 类的物品...", thing.c_str());
//         { 
//             std::lock_guard<std::mutex> lock(mutex_);
//             target_found = false; 
//             target_pixel_x = -1.0; // 重置像素位置
//         }
//         while (ros::ok()) {
//             ros::spinOnce(); 
//             bool is_target_found;
//             {
//                 std::lock_guard<std::mutex> lock(mutex_);
//                 is_target_found = target_found;
//             }

//             if (is_target_found) {
//                 ROS_INFO("初步发现目标！准备进入对准阶段。");
//                 class_thing.insert(rething);
//                 break; 
//             }
//             twist.linear.x = 0.0;
//             twist.angular.z = 0.3;
//             ro_pub.publish(twist);
//             rate.sleep();
//         }
//         // ... 在阶段一的while循环之后，阶段二微调之前 ...

//         // --- 关键修复步骤：停止并等待数据刷新 ---
//         // 1. 停止机器人
//         twist.angular.z = 0.0;
//         ro_pub.publish(twist);
//         ROS_INFO("初步发现目标，正在停止并稳定数据...");

//         // 2. 短暂延时，让旧的回调跑完，同时让机器人物理上停稳
//         ros::Duration(0.5).sleep(); // 0.5秒，可以根据实际情况调整

//         // 3. 清空关键状态标志位，准备接收新数据
//         {
//             std::lock_guard<std::mutex> lock(mutex_);
//             target_found = false; // 强制重置，丢弃所有旧的检测结果
//             target_pixel_x = -1.0;
//         }

//         // 4. 让ROS回调队列处理一下，确保上面的重置生效
//         ros::spinOnce(); 

//         // --- 现在可以安全进入阶段二：精确对准微调 (Servoing) ---
//         ROS_INFO("阶段2:正在微调以对准目标...");
//         // ... 后续的微调代码 ...

//         // --- 阶段二：精确对准微调 (Servoing) ---
//         ROS_INFO("阶段2:正在微调以对准目标...");
//         const double CENTER_DEAD_ZONE_LEFT = 440.0;
//         const double CENTER_DEAD_ZONE_RIGHT = 840.0;
//         ros::Time align_start_time = ros::Time::now();

//         while(ros::ok() && (ros::Time::now() - align_start_time < ros::Duration(5.0))) { 
//             ros::spinOnce();
//             double current_pixel_x;
//             bool is_still_found;
//             {
//                 std::lock_guard<std::mutex> lock(mutex_);
//                 current_pixel_x = target_pixel_x;
//                 is_still_found = target_found; 
//             }

//             if (!is_still_found) {
//                 ROS_WARN("在微调过程中丢失目标...");
//                 break; 
//             }

//             if (current_pixel_x > CENTER_DEAD_ZONE_LEFT && current_pixel_x < CENTER_DEAD_ZONE_RIGHT) {
//                 ROS_INFO("已对准目标！(Pixel X: %f)", current_pixel_x);
//                 break; 
//             }
            
//             if (current_pixel_x <= CENTER_DEAD_ZONE_LEFT) {
//                 twist.angular.z = -0.30;
//             } else { 
//                 twist.angular.z = +0.30;
//             }
//             twist.linear.x = 0.0;
//             ro_pub.publish(twist);
//             rate.sleep();
//         }

//         // 检查微调循环的结果
//         bool aligned_successfully = false;
//         {
//             std::lock_guard<std::mutex> lock(mutex_);
//             // 重新获取一次最新的像素位置来判断
//             if (target_found && target_pixel_x > CENTER_DEAD_ZONE_LEFT && target_pixel_x < CENTER_DEAD_ZONE_RIGHT) {
//                 aligned_successfully = true;
//             }
//         }

//         // 如果没有对准成功 (因为超时或目标丢失)
//         if (!aligned_successfully) {
//             ROS_WARN("微调失败，将重新开始搜索...");
//             // 使用 continue 跳到外层主循环的开头，重新从阶段一开始
//             continue;
//         }

//         // 只有在对准成功后，才执行后续步骤
//         twist.angular.z = 0.0;
//         ro_pub.publish(twist);
//         ROS_INFO("机器人已停止并对准。");
//         ros::Duration(0.5).sleep(); 

//         // 5. --- 阶段三：分析并导航 ---
//         // (这部分逻辑与之前相同，但在机器人完全对准后执行，效果会好很多)
//         ROS_INFO("阶段3:正在分析雷达数据并导航...");
//         ros::spinOnce();

//         int local_ob_num;
//         double local_goal_p_list[8][4];
//         {
//             std::lock_guard<std::mutex> lock(mutex_);
//             local_ob_num = ob_num;
//             for(int i=0; i < ob_num; ++i) {
//                 for(int j=0; j<4; ++j) {
//                     local_goal_p_list[i][j] = goal_p_list[i][j];
//                 }
//             }
//         }
        
//         if (local_ob_num == 0) {
//             ROS_WARN("已对准但雷达未检测到物体。重新开始搜索...");
//             continue; 
//         }

//         int best_target_index = -1;
//         double min_y_abs = 1e9;
//         for (int i = 0; i < local_ob_num; i++) {
//             geometry_msgs::PointStamped map_point;
//             map_point.header.frame_id = "map";
//             map_point.header.stamp = ros::Time(0);
//             map_point.point.x = local_goal_p_list[i][0];
//             map_point.point.y = local_goal_p_list[i][1];
//             map_point.point.z = 0;
//             try {
//                 geometry_msgs::PointStamped base_link_point;
//                 listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
//                 listener.transformPoint("base_link", map_point, base_link_point);
//                 if (base_link_point.point.x > 0.1) {
//                     if (std::abs(base_link_point.point.y) < min_y_abs) {
//                         min_y_abs = std::abs(base_link_point.point.y);
//                         best_target_index = i;
//                     }
//                 }
//             } catch (tf::TransformException &ex) {
//                 ROS_ERROR("TF坐标变换失败: %s. 跳过此物体。", ex.what());
//                 continue;
//             }
//         }

//         if (best_target_index == -1) {
//             ROS_WARN("未能在雷达数据中定位到明确的车前目标。重新开始搜索...");
//             continue;
//         }

//         ROS_INFO("目标已关联！准备导航至物体索引 %d。", best_target_index);
//         goal.target_pose.header.frame_id = "map";
//         goal.target_pose.header.stamp = ros::Time::now();
//         goal.target_pose.pose.position.x = local_goal_p_list[best_target_index][0];
//         goal.target_pose.pose.position.y = local_goal_p_list[best_target_index][1];
//         goal.target_pose.pose.orientation.z = local_goal_p_list[best_target_index][2];
//         goal.target_pose.pose.orientation.w = local_goal_p_list[best_target_index][3];
//         move_base_client_.sendGoal(goal);
//         bool nav_success = move_base_client_.waitForResult(ros::Duration(15.0));

//         if (nav_success && move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//             ROS_INFO("已成功停靠在目标前方。");
//             parked_successfully = true; 
//         } else {
//             ROS_WARN("导航至目标失败或超时。将重新开始搜索...");
//             move_base_client_.cancelGoal();
//         }
//     }

//     // 7. 任务收尾 (与之前相同)
//     ROS_INFO("任务完成，已到达 '%s' 前方。", rething.c_str());
//     std::string wav_thing;
//     for (auto it = class_thing.begin(); it != class_thing.end(); ++it) {
//         if ((thing == "Dessert"   && (*it == "Cake" || *it == "Milk" || *it == "Cola")) ||
//             (thing == "Fruit"     && (*it == "Watermelon" || *it == "Apple" || *it == "Banana")) ||
//             (thing == "Vegetable" && (*it == "Chili" || *it == "Tomato" || *it == "Potato"))) {
//                 wav_thing = *it;
//                 break;
//             }
//     }
//     std::string wav_path = "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/get_" + toLower(wav_thing) + "_x.wav";
//     system(("aplay " + wav_path).c_str());
//     navToGoal(Point_number++);
//     ros::param::set("/detect", -1);
//     ros::param::set("/thing1",wav_thing);
//     yd_control.data = 1;
//     yd_loc_control.publish(yd_control);
//     yd_sub.shutdown();
//     vi_sub.shutdown();
// }
