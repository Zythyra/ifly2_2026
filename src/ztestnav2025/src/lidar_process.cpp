#include <ros/ros.h>  
#include <sensor_msgs/LaserScan.h>  
#include "ztestnav2025/lidar_process.h"
#include "ztestnav2025/traffic_light.h"

#include <vector>  
#include <algorithm>
#include <cmath>  

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

#include "ztestnav2025/getpose_server.h"//获取yaw以便在== 3的分支中筛选雷达点

class LidarProcessor {
private:
    ros::NodeHandle nh_; 
    // 为服务客户端创建一个全局NodeHandle
    ros::NodeHandle nh_global_;
    ros::Subscriber sub_;
    ros::ServiceServer server;
    sensor_msgs::LaserScan lasar_scan_;

    // 新增的服务客户端，用于获取机器人位姿
    ros::ServiceClient getpose_client_;

    std::vector<float> ranges_;
    int num_points_ = 337;
    float angle_step = M_PI * 2.0 / (num_points_ - 1);

    double lidar_board_backdisdance;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        lasar_scan_ = *scan_msg;  // 更新最新扫描数据
    }

    bool lidar_process(ztestnav2025::lidar_process::Request& req,ztestnav2025::lidar_process::Response& resp){//处理雷达数据，获取板子坐标
        ranges_ = lasar_scan_.ranges;
        num_points_ = ranges_.size();
        std::vector<std::vector<float>> result;
        float theta = 0;
        if(req.lidar_process_start==1){//视觉发现板子，雷达查看正前方数据
            int effective_point = 0;
            std::vector<float> disdance;
            for(int i=158;i<=178;i++){//只看正前方的点
                if(std::isinf(ranges_[i]) || ranges_[i] == 0.0f) continue;
                effective_point++;
                disdance.push_back(ranges_[i]);
            }
            // ROS_INFO("有效点数%d",effective_point);
            std::sort(disdance.begin(), disdance.end());
            ROS_INFO("距离%f",disdance[effective_point/2]);//中位数
            resp.lidar_results.push_back(disdance[effective_point/2]);
            return true;
        }

        if(req.lidar_process_start==2){//到达板前，雷达对准
            int effective_point = 0;
            float shortest = 100;
            for (int i=158;i<=178;i++) {// 将雷达数据转化为xy坐标系
                if (std::isinf(ranges_[i]) || ranges_[i] == 0.0f) {
                    continue;
                }
                theta = i * angle_step;
                effective_point++;
                result.push_back({ranges_[i] * cos(theta) * -1,ranges_[i] * sin(theta) * -1});
                if(ranges_[i]<shortest){
                    shortest = ranges_[i];
                }
            }
            std::vector<double> slope;
            for (int i=0;i<effective_point-1;i++){
                slope.push_back((result[i+1][0]-result[i][0])/(result[i+1][1]-result[i][1])*-1);//这里是x/y，免得斜率变成无穷大了
            }
            std::sort(slope.begin(), slope.end());
            // ROS_INFO("板子斜率%f",slope[effective_point/2]);
            resp.lidar_results.push_back(slope[effective_point/2]);
            resp.lidar_results.push_back(shortest);//最短距离
            return true;
        }
        if(req.lidar_process_start == 3)
        { // 拣货区对准后绕行用

            ztestnav2025::getpose_server pose_srv;
            pose_srv.request.getpose_start = 1;
            double robot_yaw;
            if (getpose_client_.call(pose_srv) && !pose_srv.response.pose_at.empty()) {
                robot_yaw = pose_srv.response.pose_at[2]; // 获取yaw
                
            } else {
                ROS_ERROR("调用/getpose_server服务失败或响应为空");
                resp.lidar_results.push_back(-1);
                return true;
            }

            int effective_point = 0;
            double average_x = 0;
            double average_y = 0;
            std::vector<cv::Point2f> points; // 准备拟合直线
            std::vector<float> distance;
            bool flag = 1;
            int count = 168;
            int failed_count = 0;

            double leftestx,leftesty,rightestx,rightesty;
            // 从中心向右搜索
            while(flag){
                count++;
                if (count > 332 || failed_count > 6) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    failed_count++;
                    continue;
                }

                float current_distance = ranges_[count];
                // 计算每个雷达点在世界坐标系下的绝对角度
                float relative_angle = lasar_scan_.angle_min + count * lasar_scan_.angle_increment;
                double world_angle = robot_yaw + relative_angle;
                world_angle = atan2(sin(world_angle), cos(world_angle)); // 归一化到[-pi, pi]
                // 使用test_point的逻辑进行筛选
                bool is_wall = false;
                if (world_angle > -0.95 && world_angle <= 0.785) { // 右墙
                    if (current_distance > 1.25 / cos(world_angle) - 0.5) { is_wall = true; }
                } else if (world_angle > 0.785 && world_angle <= 2.355) { // 上墙
                    if (current_distance > 1.25 / sin(world_angle) - 0.5) { is_wall = true; }
                } else if (world_angle > -2.19 && world_angle <= -0.95) { // 下墙
                    if (current_distance > 1.75 / std::abs(sin(world_angle)) - 0.5) { is_wall = true; }
                } else { // 左墙
                    if (current_distance > 1.25 / std::abs(cos(world_angle)) - 0.5) { is_wall = true; }
                }
                    
                if(is_wall) {
                    continue; // 如果是墙壁，则跳过该点
                }

                // 放宽距离阈值以适应远距离板，同时过滤墙面
                if ((fabs(ranges_[count] - ranges_[count - failed_count - 1]) > 0.2)) {
                    failed_count++;
                    continue;
                }
                failed_count = 0;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);
                points.push_back(pt);
                effective_point++;
                average_x += pt.x;
                average_y += pt.y;
                distance.push_back(ranges_[count]);
            }
            ROS_INFO("完成一轮搜索%zu",points.size());
            if(points.size()>5){
                rightestx = points[points.size()-1].x;
                rightesty = points[points.size()-1].y;
            }
            

            failed_count = 0;
            count = 168;
            flag = true;

            // 从中心向左搜索
            while(flag){
                count--;
                if (count == 3 || failed_count > 6) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    failed_count++;
                    continue;
                }

                float current_distance = ranges_[count];
                float relative_angle = lasar_scan_.angle_min + count * lasar_scan_.angle_increment;
                double world_angle = robot_yaw + relative_angle;
                world_angle = atan2(sin(world_angle), cos(world_angle));

                bool is_wall = false;
                if (world_angle > -0.95 && world_angle <= 0.785) {
                    if (current_distance > 1.25 / cos(world_angle) - 0.5) { is_wall = true; }
                } else if (world_angle > 0.785 && world_angle <= 2.355) {
                    if (current_distance > 1.25 / sin(world_angle) - 0.5) { is_wall = true; }
                } else if (world_angle > -2.19 && world_angle <= -0.95) {
                    if (current_distance > 1.75 / std::abs(sin(world_angle)) - 0.5) { is_wall = true; }
                } else {
                    if (current_distance > 1.25 / std::abs(cos(world_angle)) - 0.5) { is_wall = true; }
                }
                if(is_wall) 
                {
                    continue;
                }


                 // 同样放宽距离阈值
                if (fabs(ranges_[count] - ranges_[count + failed_count - 1]) > 0.2) {
                    failed_count++;
                    continue;
                }
                failed_count = 0;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);
                points.push_back(pt);
                effective_point++;
                average_x += pt.x;
                average_y += pt.y;
                distance.push_back(ranges_[count]);
            }
            ROS_INFO("完成二轮搜索%zu",points.size());
            if(points.size()>4){
                leftestx = points[points.size()-1].x;
                leftesty = points[points.size()-1].y;
            }
            // 去除 effective_point > 5 && distance[0] < 0.45 的判断
            // 只要找到足够的点，就进行拟合
            if (effective_point > 7){ 
                ROS_INFO("雷达绕障模式(3): 找到 %d 个有效点, 进行直线拟合。", effective_point);
                std::sort(distance.begin(), distance.end());
                cv::Vec4f lineParams;
                cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);
                //标准化方向向量 
                float vx = lineParams[0];
                float vy = lineParams[1];

                // 强制让向量的y分量为负, 这样其法向量（-vy,vx）必定会指向障碍物后方
                if (vy > 0) 
                {
                    vx = -vx;
                    vy = -vy;
                }
                // 确保法向量单位化（已知原始方向向量已单位化）
                float norm_length = std::sqrt(vy * vy + vx * vx);  // 实际应为1，安全起见保留
                float nx = -vy / norm_length;  // 法向量x分量
                float ny = vx / norm_length;   // 法向量y分量
                // 计算后方点坐标（从中点沿法向量方向移动lidar_board_backdisdance米）
                // float mid_x = average_x / effective_point;
                // float mid_y = average_y / effective_point;
                float mid_x = (leftestx+rightestx)/2;
                float mid_y = (leftesty+rightesty)/2;
                float back_x = mid_x + nx * lidar_board_backdisdance;
                float back_y = mid_y + ny * lidar_board_backdisdance;
                float angle = std::atan2(ny, nx);

                resp.lidar_results.push_back(distance[0]);//最短距离
                // resp.lidar_results.push_back(average_x / effective_point);//中点x坐标
                // resp.lidar_results.push_back(average_y / effective_point);//中点y坐标
                // resp.lidar_results.push_back(lineParams[0] / lineParams[1]);//板子斜率
                // resp.lidar_results.push_back(vx); // 新增返回拟合直线的xy分量
                // resp.lidar_results.push_back(vy); 
                resp.lidar_results.push_back(back_x);//直接返回目的地就可以了
                resp.lidar_results.push_back(back_y);//
                resp.lidar_results.push_back(angle);//目的地角度（雷达坐标系）
                ROS_INFO("x%f,y%f",average_x/effective_point,average_y/effective_point);
                ROS_INFO("BACK_X%f,back_y%f,angle%f",back_x,back_y,angle);

                return true;
            }
            else { // 如果点不够，则认为没有找到有效障碍物
                ROS_WARN("雷未找到足够的有效点来拟合障碍物。");
                resp.lidar_results.push_back(-1);
                return true;
            }
        }
        if(req.lidar_process_start == 4){//不仅仅要知道前方障碍板在哪里，还要看路上有没有其他障碍物
            int effective_point = 0,dangerous_point = 0;
            std::vector<float> disdance;
            float center_disdance = -1;
            for(int i=0;i<20;i++){//认为最中心的点属于板子
                if(!(std::isinf(ranges_[168+i]) || ranges_[168+i] == 0.0f)){
                    center_disdance = ranges_[168+i];
                    break;
                }
                if(!(std::isinf(ranges_[168-i]) || ranges_[168-i] == 0.0f)){
                    center_disdance = ranges_[168-i];
                    break;
                }
            }
            if(center_disdance!=-1){//找到中央点就好处理，没找到的话直接前进到最近一个障碍物前方一般不会看不见
                for(int i=138;i<=198;i++){//只看正前方的点
                    if(std::isinf(ranges_[i]) || ranges_[i] == 0.0f) continue;
                    theta = (168-i) * angle_step;
                    if(abs(ranges_[i] * sin(theta))<0.25){
                        if(ranges_[i] * cos(theta)<center_disdance){//前方有障碍点,启用movebase
                            dangerous_point++;
                            if(dangerous_point>3){
                                resp.lidar_results.push_back(center_disdance);
                                resp.lidar_results.push_back(-1);//-1表示启用movebase
                                return true;
                            }
                        }
                    }
                }
                resp.lidar_results.push_back(center_disdance);
                resp.lidar_results.push_back(1);//1表示直线前进
                return true;
            }
            ROS_INFO("正前方没点，另外处理");
            return true;
        }
    //---------------------------新增的模式：为避障获取精确前方距离和角度---------------------
        if(req.lidar_process_start == 0){ 
            if (lasar_scan_.ranges.empty()) { // 检查是否有雷达数据
                ROS_INFO("无雷达数据");
                resp.lidar_results.push_back(-1); // 返回-1表示无数据
                return true;
            }
            float min_dist = std::numeric_limits<float>::infinity();
            int min_index = -1;
    
            // 根据要求，检查索引150到186的范围
            for(int i = 120; i <= 216; i++){
                // 安全检查，防止索引越界
                if(i >= lasar_scan_.ranges.size()) break; 
        
                float current_range = lasar_scan_.ranges[i];
        
                // 忽略无效数据
                if(std::isinf(current_range) || std::isnan(current_range) || current_range <= 0.0f) continue;
        
                // 寻找最小值
                if(current_range < min_dist){
                    min_dist = current_range;
                    min_index = i;
                }
            }

            if(min_index != -1){ // 如果找到了有效点
                // 使用LaserScan消息中的元数据计算精确角度
                float angle = lasar_scan_.angle_min + min_index * lasar_scan_.angle_increment;
        
                // 返回最小距离和对应的精确角度
                resp.lidar_results.push_back(min_dist);
                resp.lidar_results.push_back(angle);
                ROS_INFO("找到有效点");
            } else {
                // 如果在该范围内没有找到有效点，返回-1
                resp.lidar_results.push_back(-1); 
                ROS_INFO("未找到有效点");
            }
            return true;
        }
        
        // else if(req.lidar_process_start==-1){//视觉巡线区的雷达处理代码
        //     int effective_point = 0;
        //     double average_x = 0;
        //     double average_y = 0;
        //     std::vector<cv::Point2f> points;//准备拟合直线
        //     std::vector<float> distance;
        //     bool flag = 1;
        //     int count = 168;
        //     while(flag){
        //         count++;
        //         if (count>332) break;
        //         if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
        //             continue;
        //         }
        //         if (fabs(ranges_[count+1]-ranges_[count])>0.2) break;
        //         if (ranges_[count] > 0.6) continue;
        //         theta = count * angle_step;
        //         cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
        //         points.push_back(pt);
        //         effective_point++;
        //         average_x += ranges_[count] * cos(theta)*-1;
        //         average_y += ranges_[count] * sin(theta)*-1;
        //         distance.push_back(ranges_[count]);
        //     }
        //     if (effective_point==0){
        //         resp.lidar_results.push_back(-1);
        //         return true;
        //     }
        //     count = 168;
        //     while(flag){
        //         count--;
        //         if (count==3) break;
        //         if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
        //             continue;
        //         }
        //         if (fabs(ranges_[count+1]-ranges_[count])>0.2) break;
        //         if (ranges_[count] > 0.6) continue;
        //         theta = count * angle_step;
        //         cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
        //         points.push_back(pt);
        //         effective_point++;
        //         average_x += ranges_[count] * cos(theta)*-1;
        //         average_y += ranges_[count] * sin(theta)*-1;
        //         distance.push_back(ranges_[count]);
        //     }
        //     std::sort(distance.begin(), distance.end());
        //     ROS_INFO("最小距离%f",distance[0]);
        //     ROS_INFO("有效点数%d",effective_point);
        //     if (effective_point > 5 && distance[0] < 0.45){ //满足条件就说明前方有障碍物
        //         // waitForContinue();
        //         cv::Vec4f lineParams;
        //         cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);
        //         resp.lidar_results.push_back(distance[0]);//最短距离
        //         resp.lidar_results.push_back(average_x/effective_point);//中点x坐标
        //         resp.lidar_results.push_back(average_y/effective_point);//中点y坐标
        //         resp.lidar_results.push_back(lineParams[0]/lineParams[1]);//板子斜率
        //         ROS_INFO("已返回障碍物斜率%f",lineParams[0]/lineParams[1]);
        //         return true;
        //     }
        //     else {//从else回来的第一项=-1就是没有障碍物
        //         resp.lidar_results.push_back(-1);
        //         return true;
        //     }
        // }
        if(req.lidar_process_start==-2){//视觉巡线区的雷达处理代码第二版，考虑到雷达丢数据
            // ROS_INFO("雷达参数-2");
            int effective_point = 0;
            double average_x = 0;
            double average_y = 0;
            std::vector<cv::Point2f> points;//准备拟合直线
            std::vector<float> distance;
            bool flag = 1;
            int count = 168;
            int failed_count = 0;

            while(flag){
                count++;
                if (count>332 || failed_count > 6) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    failed_count++;
                    continue;
                }
                if ((fabs(ranges_[count]-ranges_[count-failed_count-1])>0.2) || ranges_[count]>0.8) {
                    failed_count++;
                    continue;
                }
                failed_count = 0;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
                points.push_back(pt);
                effective_point++;
                average_x += ranges_[count] * cos(theta)*-1;
                average_y += ranges_[count] * sin(theta)*-1;
                // ROS_INFO("左x:%f,y:%f",ranges_[count] * cos(theta)*-1,ranges_[count] * sin(theta)*-1);
                // ROS_INFO("距离%f",ranges_[count]);
                distance.push_back(ranges_[count]);
            }
            if (effective_point==0){
                resp.lidar_results.push_back(-1);
                // ROS_INFO("没有点");
                return true;
            }
            failed_count = 0;
            count = 168;
            flag = true;

            while(flag){
                count--;
                if (count==3 || failed_count > 6) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    failed_count++;
                    continue;
                }
                if (fabs(ranges_[count]-ranges_[count+failed_count-1])>0.2|| ranges_[count]>0.8) {
                    failed_count++;
                    continue;
                }
                failed_count = 0;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
                points.push_back(pt);
                effective_point++;
                average_x += ranges_[count] * cos(theta)*-1;
                average_y += ranges_[count] * sin(theta)*-1;
                // ROS_INFO("右x:%f,y:%f",ranges_[count] * cos(theta)*-1,ranges_[count] * sin(theta)*-1);
                // ROS_INFO("距离%f",ranges_[count]);
                distance.push_back(ranges_[count]);
            }
            std::sort(distance.begin(), distance.end());
            // ROS_INFO("最小距离%f",distance[0]);
            // ROS_INFO("有效点数%d",effective_point);
            if (effective_point > 5 && distance[0] < 0.6){ //满足条件就说明前方有障碍物
                // waitForContinue();
                cv::Vec4f lineParams;
                cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);

                //标准化方向向量 
                float vx = lineParams[0];
                float vy = lineParams[1];

                // 强制让向量的y分量为负，这样其法向量（-vy,vx）必定会指向障碍物后方
                if (vy > 0) 
                {
                    vx = -vx;
                    vy = -vy;
                }
                resp.lidar_results.push_back(distance[0]);//最短距离
                resp.lidar_results.push_back(average_x/effective_point);//中点x坐标
                resp.lidar_results.push_back(average_y/effective_point);//中点y坐标
                resp.lidar_results.push_back(lineParams[0]/lineParams[1]);//板子斜率

                resp.lidar_results.push_back(vx); // 新增返回拟合直线的xy分量
                resp.lidar_results.push_back(vy); 
                ROS_INFO("x%f,y%f",average_x/effective_point,average_y/effective_point);
                return true;
            }
            else {//从else回来的第一项=-1就是没有障碍物
                resp.lidar_results.push_back(-1);
                return true;
            }
        }
        
    }
    
public:
    LidarProcessor() : nh_("~") {
        // 订阅雷达数据（队列大小10
        sub_ = nh_.subscribe("/scan", 10, &LidarProcessor::scanCallback, this);
        server = nh_.advertiseService("lidar_process", &LidarProcessor::lidar_process, this);
        ROS_INFO("雷达初始化");
        // 使用全局NodeHandle初始化服务客户端
        getpose_client_ = nh_global_.serviceClient<ztestnav2025::getpose_server>("/getpose_server");
        ROS_INFO("等待坐标获取服务 /getpose_server ...");
        getpose_client_.waitForExistence();
        ROS_INFO("雷达处理器初始化完成，坐标获取服务已连接。");
        nh_.getParam("/myplanernav/lidar_board_backdisdance",lidar_board_backdisdance);
        ROS_INFO("获取参数%f",lidar_board_backdisdance);
    }

    // 实时查看数据（新增方法）
    void printCurrentData() {
        // ROS_INFO("打印雷达数据");        //调试信息，打印雷达数据
        // std::cout << "[";
        // for (size_t i = 0; i < num_points_; ++i) {
        //     std::cout << "[";
        //     std::cout << result[i][0] << ",";
        //     std::cout << result[i][1];
        //     std::cout << "]";
        //     std::cout << ",";
        // } 
        // std::cout << "]";
        // std::cout << "\n";
        // for (int j=0;j<mask.size();j++){
        //     std::cout << mask[j] << ",";
        // }
        // std::cout << "\n";

        //获取板子坐标
    }
};

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "lidar_process");
    
    // 初始化存储对象
    LidarProcessor lidar_processor;
    
    ros::spin();
    return 0;
}