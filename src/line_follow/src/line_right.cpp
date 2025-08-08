#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <random>
#include <string>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>
#include "line_follow/line_follow.h"
#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/lidar_process.h"
// #include "ztestnav2025/set_speed.h"
//停车的时候画面大部分会超出赛道，找赛道的算法需要考虑同一个画面亮度不均匀
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv;
using namespace std;

string output_file = "/home/ucar/ucar_car/src/line_follow/image/line_right.avi";//录制视频避免网络传输卡顿
VideoWriter out;
int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D'); // MP4V编码
ostringstream displayStream;

struct RaceTrack {
    double slope;
    vector<Point> points;
    int direction_change;
    int slope_change_count;
}; 

void drawLineFromEquation(cv::Mat& img, double a, double b, double c, const cv::Scalar& color, int thickness) {
    int width = img.cols;
    int height = img.rows;
    
    // 特殊情况处理
    if (fabs(a) < 1e-6 && fabs(b) < 1e-6) {
        return; // 无效直线
    }
    
    cv::Point pt1, pt2;
    
    if (fabs(b) > fabs(a)) {
        // 更接近水平线，使用左右边界
        pt1.x = 0;
        pt1.y = static_cast<int>(-c / b); // x=0 时的 y 值
        
        pt2.x = width - 1;
        pt2.y = static_cast<int>((-a * (width - 1) - c) / b); // x=width-1 时的 y 值
    } else {
        // 更接近垂直线，使用上下边界
        pt1.y = 0;
        pt1.x = static_cast<int>(-c / a); // y=0 时的 x 值
        
        pt2.y = height - 1;
        pt2.x = static_cast<int>((-b * (height - 1) - c) / a); // y=height-1 时的 x 值
    }
    
    // 裁剪直线到图像边界
    cv::Rect rect(0, 0, width, height);
    if (cv::clipLine(rect, pt1, pt2)) {
        cv::line(img, pt1, pt2, color, thickness);
    }
}

// RANSAC直线拟合函数
// 输入：点集 points，距离阈值 distThreshold，最大迭代次数 maxIterations
// 输出：直线参数 (a, b, c) 满足 ax+by+c=0，以及内点索引
std::pair<std::vector<double>, std::vector<int>> fitLineRANSAC(
    std::vector<cv::Point>& points, 
    float distThreshold, 
    int maxIterations) 
{
    // 随机数生成器初始化
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> uni(0, points.size()-1);

    // 最佳模型和内点
    std::vector<double> bestModel(3);
    std::vector<int> bestInliers;
    int bestInlierCount = 0;

    // RANSAC主循环
    for (int i = 0; i < maxIterations; ++i) {
        // 1. 随机选两个点
        int idx1 = uni(rng);
        int idx2 = uni(rng);
        while (idx2 == idx1) idx2 = uni(rng);  // 确保不同点
        
        const auto& p1 = points[idx1];
        const auto& p2 = points[idx2];
        
        // 2. 计算直线参数 (a, b, c)
        double a = p2.y - p1.y;
        double b = p1.x - p2.x;
        double c = p2.x * p1.y - p1.x * p2.y;
        
        // 处理重合点 (分母接近0)
        double denom = std::sqrt(a*a + b*b);
        if (denom < 1e-5) continue;  // 跳过无效直线
        
        // 3. 计算内点
        std::vector<int> inliers;
        for (int j = 0; j < points.size(); ++j) {
            const auto& pt = points[j];
            double dist = std::abs(a*pt.x + b*pt.y + c) / denom;
            if (dist < distThreshold) inliers.push_back(j);
        }
        
        // 4. 更新最佳模型
        if (inliers.size() > bestInlierCount) {
            bestInlierCount = inliers.size();
            bestModel = {a, b, c};
            bestInliers = std::move(inliers);
        }
    }
    
    return {bestModel, bestInliers};
}


void threshold_image(Mat& gray){
    int adaptive_block = 45;    // 自适应邻域大小（基数）
    int adaptive_c = -15;       // 阈值偏移量（关键参数）
    int min_contour_area = 250; // 最小轮廓面积阈值

    cv::Mat binary;
    cv::adaptiveThreshold(
        gray, binary, 255, 
        cv::ADAPTIVE_THRESH_MEAN_C, // 使用局部均值
        cv::THRESH_BINARY,           // 二值化类型
        adaptive_block,             // 邻域大小（必须奇数）
        adaptive_c                   // 关键偏移量
    );
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat denoised = cv::Mat::zeros(binary.size(), CV_8UC1);
    // 过滤小面积轮廓（可能是噪声）
    for(size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if(area > min_contour_area) {
            cv::drawContours(denoised, contours, static_cast<int>(i), 
                            cv::Scalar(255), cv::FILLED);
        }
    }
    gray = denoised.clone();
}

bool stop_car(Mat& gray,int brightness_threshold,int& point,Mat& visual_img){
    int white_count = 0;
    for (int y = 227; y >= 200; y--) {//
        for (int x = 1; x < 639; x++) {
            if (gray.at<uchar>(y, x)>=brightness_threshold){ //停车逻辑要换算法
                white_count++;
                circle(visual_img, Point(x,y), 2, Scalar(0, 0, 0), -1);
            }
        }
    }
    point = white_count;
    ROS_INFO("停车白点数量%d",white_count);
    if (white_count>2058){
        return true;
    }
    return false;
}

// 寻找赛道第一点，这个点的特征是，与赛道边缘接壤,右巡线时，赛道起点应该在图像的右下方
std::vector<Point> find_track_edge(Mat& gray_img,Mat& visual_img) {
    int bottom_trace_end_point_index = 200,right_trace_end_point_index = 70;
    bool is_now_white = false;//这个变量的意思是，接壤的白点连续时，最左边一个是赛道
    std::vector<Point> maybe_start_point;
    Point start_point;
    for(int i = 639;i>bottom_trace_end_point_index;i--){//在图片底部寻找接壤的赛道
        if(!is_now_white){
            if(gray_img.at<uchar>(269, i)==255){//发现疑似赛道起点
                is_now_white = true;
            }
        }
        if(is_now_white){
            if(gray_img.at<uchar>(269, i)==0){//如果到了6还没有break，说明断线了，找到赛道起点
                maybe_start_point.push_back(Point(i-1,269));
                // ROS_INFO("找到起点%d,%d",start_point.x,start_point.y);
                circle(visual_img, Point(i-1,269), 5, Scalar(0, 0, 255), -1);
                is_now_white = false;
            }
        }
    }
    is_now_white = true;
    for(int i = 269;i>right_trace_end_point_index;i--){//在图片右部寻找接壤的赛道
        if(is_now_white){
                if(gray_img.at<uchar>(i, 639)==0){//发现疑似赛道起点
                    is_now_white = false;
                }
        }
        if(!is_now_white){
            if(gray_img.at<uchar>(i,639)==255){//有白点就跳过
                circle(visual_img, Point(639,i), 5, Scalar(0, 0, 255), -1);
                // ROS_INFO("找到起点%d,%d",start_point.x,start_point.y);
                maybe_start_point.push_back(Point(639,i));
                is_now_white = true;
            }
        }
    }
    // imshow("test",visual_img);
    // waitKey(1);
    return maybe_start_point;
}

// 从起始点开始追踪赛道边线（添加断裂检测机制）gray_img,cropped
bool trace_edge(Mat& gray_img, vector<Point> start_points, RaceTrack& racetrack,Mat& visual_img) {
    int point_number = (int)start_points.size();
    vector<RaceTrack> racetracks(point_number);
    int height = gray_img.rows,width = gray_img.cols,search_range = 60;

    for(int point_index = 0;point_index<point_number;point_index++){
        bool broken = false,last_left = false,last_right = false;//赛道是平滑的，不能一会往左一会往右
        // 计数器：记录连续未找到点的行数
        int fail_count = 0;
        Point start_point = start_points[point_index];
        // 初始化搜索中心
        int center_x = start_point.x,center_y = start_point.y - 1,number = 1;// 从起始点上方开始搜索没必要找太多，60个点够了

        while (center_y > start_point.y-100) {  // 只看往上130行
            bool found = false;
            // 在当前行搜索范围内检查所有可能点
            for (int dx = 0; dx <= search_range/2; dx++) {
                // 计算候选点位置
                // ROS_INFO("进循环");
                int cand_x = center_x + dx;
                int cand_x2 = center_x - dx;
                bool left_check = 1;
                bool right_check = 1;
                // 边界检查
                if (cand_x >= width - 1) {
                    right_check = 0;
                }
                if (cand_x2 <= 1) {
                    left_check = 0;
                }

                if (left_check){
                    // ROS_INFO("过检查%d",gray_img.at<uchar>(center_y, cand_x2));
                    if (gray_img.at<uchar>(center_y, cand_x2)==255 && gray_img.at<uchar>(center_y, cand_x2 - 1)==0 ){
                        // ROS_INFO("可行点");
                        racetracks[point_index].points.push_back(Point(cand_x2,center_y));
                        found = true;
                        center_x = cand_x2;
                        if(last_right){
                            racetracks[point_index].direction_change ++;//发现赛道方向突变，右变左
                        }
                        last_left = true;
                        last_right = false;
                    }
                }
                if(!found&&right_check){
                    if (gray_img.at<uchar>(center_y, cand_x)==0 && gray_img.at<uchar>(center_y, cand_x + 1)==255) {
                        // racetracks[point_index].points.push_back(Point(cand_x + 1,center_y));
                        found = true;
                        center_x = cand_x + 1;
                        if(last_left){
                            racetracks[point_index].direction_change ++;//发现赛道方向突变，左变右
                        }
                        last_left = false;
                        last_right = true;
                    }
                }
            }

            if (found) {
                fail_count = 0;
                number++;
                center_y --;
            } else {
                fail_count++;
                center_y--;
                if (fail_count >= 20) {
                    broken = true;
                    break;
                }
            }
            if (number>100|| center_y <= 0) {
                break;
            }
        }
        // ROS_INFO("有效点数%zu",racetracks[point_index].points.size());
        if(racetracks[point_index].points.size()>15){
            Vec4f lineParams; // 存放结果的 Vec4f
            fitLine(racetracks[point_index].points, lineParams, DIST_L2, 0, 0.01, 0.01);
            racetracks[point_index].slope = lineParams[1]/lineParams[0];
        }
        else{
            racetracks[point_index].slope = -2.0;
        }
    }
    int maybe_race_index = -1;
    float min_dangerous_point_weight = 2.1;//跳变点占所有点数量百分比
    for(int i=0;i<point_number;i++){//遍历所有赛道，选出最好的一条
        // ROS_INFO("斜率%f,if条件%d,%d",racetracks[i].slope,racetracks[i].slope<0.05,racetracks[i].slope>-10);
        if(!(racetracks[i].slope<0.05&&racetracks[i].slope>-10)){//不满足条件直接不要
            if(racetracks[i].direction_change/(float)racetracks[i].points.size()<min_dangerous_point_weight){
                maybe_race_index = i;//在斜率正确的前提下，选择跳变最少的那个
            }
        }
    }
    if(maybe_race_index != -1) {
        racetrack = racetracks[maybe_race_index];
        // 可视化追踪过程
        Scalar color = Scalar(0, 255, 0);  // 红色:左, 绿色:右
        for (const auto& point : racetrack.points) {
            circle(visual_img, point, 2, color, -1);
        }
        ostringstream displayStream1;
        displayStream1 << fixed << setprecision(2);
        displayStream1 << "line_slope:  " << racetrack.slope;
        string displayText1 = displayStream1.str();
        putText(visual_img, displayText1, Point(50, 100),
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        return true;
    }
    return false;
}
//如果右边丢线，就只看左边，因为右边丢线了，所以直接从最右边开始找，找到就是左线，然后把左线拟合成直线，如果左线碰到图片底端，就开始旋转，通过定位来判断是否到达终点，到达终点前不启用停车逻辑
bool find_left_edge(Mat gray_img,Mat& visualizeImg){
    bool is_now_white = false;
    vector<Point> maybe_left_point;
    for(int i = 269;i>50;i--){//在图片中部寻找角点所在赛道线
        if(is_now_white){
                if(gray_img.at<uchar>(i, 639)==0){//发现疑似赛道起点
                    is_now_white = false;
                }
        }
        if(!is_now_white){
            if(gray_img.at<uchar>(i,639)==255){
                circle(visual_img, Point(639,i), 9, Scalar(0, 0, 255), -1);
                maybe_left_point.push_back(Point(639,i));
                is_now_white = true;
            }
        }
    }
    int point_number = (int)maybe_left_point.size();
    vector<RaceTrack> racetracks(point_number);
    for(size_t i=0;i<maybe_left_point.size();i++){
        Point traced_point = maybe_left_point[i];
        racetracks.points.push_back(Point(traced_point.x,traced_point.y));
        while(ros::ok()){
            if(gray_img.at<uchar>(traced_point.y+1, traced_point.x-1)==255){//首先待追踪点自己是赛道1
                if(gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y+1;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }
            else if(gray_img.at<uchar>(traced_point.y, traced_point.x-1)==255){//首先待追踪点自己是赛道2
                if(gray_img.at<uchar>(traced_point.y-1, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y-1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }
            else if(gray_img.at<uchar>(traced_point.y-1, traced_point.x-1)==255){//首先待追踪点自己是赛道3
                if(gray_img.at<uchar>(traced_point.y-2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y-1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y-2, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y-1, traced_point.x)==0||gray_img.at<uchar>(traced_point.y, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y-1;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }


            else if(gray_img.at<uchar>(traced_point.y+1, traced_point.x)==255){//首先待追踪点自己是赛道4
                if(gray_img.at<uchar>(traced_point.y, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x+1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x+1)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y+1;traced_point.x = traced_point.x;
                    racetracks.points.push_back(traced_point);
                }
            }
            else if(gray_img.at<uchar>(traced_point.y+1, traced_point.x)==255){//首先待追踪点自己是赛道5
                if(gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y-1, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y+1;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }
            else if(gray_img.at<uchar>(traced_point.y+1, traced_point.x+1)==255){//首先待追踪点自己是赛道6
                if(gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y+1;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }
            else if(gray_img.at<uchar>(traced_point.y+1, traced_point.x+1)==255){//首先待追踪点自己是赛道7
                if(gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y+1;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }
            else if(gray_img.at<uchar>(traced_point.y+1, traced_point.x+1)==255){//首先待追踪点自己是赛道8
                if(gray_img.at<uchar>(traced_point.y, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-1)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y, traced_point.x-2)==0||gray_img.at<uchar>(traced_point.y+1, traced_point.x)==0||gray_img.at<uchar>(traced_point.y+2, traced_point.x)){//其次他周围要有非赛道
                    traced_point.y = traced_point.y+1;traced_point.x = traced_point.x-1;
                    racetracks.points.push_back(traced_point);
                }
            }

        }
    }
    
}

bool find_left_line(Mat gray_img,vector<Point>& left_edge_points,int brightness_threshold,Mat visualizeImg = Mat()){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = 1;
    // ROS_INFO("进入左边巡线");
    for (int y = height - 1; y >= 69; y--) {
        for (int x = width -1; x > 60; x--) {
            if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                left_edge_points.push_back(Point(x, y));
                break;
            }
        }
    }
    if(left_edge_points.empty()){
        ROS_INFO("没找到左线");
        return false;
    } 
    else {
        std::pair<std::vector<double>, std::vector<int>> result;
        result = fitLineRANSAC(left_edge_points,7,1000);
        drawLineFromEquation(visualizeImg,result.first[0],result.first[1],result.first[2],cv::Scalar(0, 0, 255),2);
        for (int i=0;i<left_edge_points.size();i++) {
            circle(visualizeImg, left_edge_points[i], 3, Scalar(255, 0, 0), -1);
        }
        out.write(visualizeImg);
        if((-1*result.first[2]-(270*result.first[1]))/result.first[0]>320){//直线和图像底部的交点
            return true;
        }else{
            return false;
        }
    }
}

double double_find(Mat gray_img,int brightness_threshold, Mat& visual_img)//最后阶段采用双边巡线，避免单边巡线导致的偏离从而无法停车
{
    vector<int> left_total;
    vector<int> right_total;
    double error = 0.0;

    vector<Point> midPoints; // 存储所有中线的点坐标

    bool falg = false;//赛道先有后无，就是没线了
    int failed = 0;//连续几行都没找到才算丢线
    for (int y = 269; y >= 50; y--) {//计算每一行的误差
        int left = 0;
        bool find = false;
        for (int x = 319; x > 1; x--) {
            if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                left = x;
                find = true;
                falg = true;
                failed = 0;
                break;
            }
        }
        if (falg && !find){
            failed++;
            if (failed>10){
                break;
            }
        }
        left_total.push_back(left);
    }
    falg = false;//赛道先有后无，就是没线了
    failed = 0;//连续几行都没找到才算丢线
    for (int y = 269; y >= 50; y--) {//计算每一行的误差
        int right = 639;
        bool find = false;
        for (int x = 319; x < 639; x++) {
            if ((int)gray_img.at<uchar>(y, x) >= brightness_threshold) {
                right = x;
                find = true;
                falg = true;
                failed = 0;
                break;
            }
        }
        if (falg && !find){
            failed++;
            if (failed>10){
                break;
            }
        }
        right_total.push_back(right);
    }
    float row = min(left_total.size(),right_total.size())/1.0;
    for(int i=0;i<row;i++){
        error += (640-(left_total[i]+right_total[i]))*(1-i/row);//error += (left_total+right_total-640)/2.0*(1-i/row) error = error/(row/2)归一化的除以2整理到前面
    }

    int minSize = min(left_total.size(), right_total.size());
    for (int i = 0; i < minSize; i++) {
        // 计算当前行的中线x坐标 (左右边界的中点)
        int midX = (left_total[i] + right_total[i]) / 2;
        // 计算当前行的y坐标 (从底部向上递减)
        int y = 269 - i;
        // 存储中点坐标
        midPoints.push_back(Point(midX, y));
    }

    // 在可视化图像上绘制黄色中线点
    for (const Point& p : midPoints) {
        circle(visual_img, p, 1, Scalar(0, 255, 255), -1); // 绘制1像素的实心黄点
    }

    return error/row;
}


double error_calculater(vector<Point>& traced_points,Mat& visualizeImg){
    double total_error = 0;
    // std::vector<Point2f> line;//拟合直线
    for (size_t i=0;i<traced_points.size();i++){
        int y = traced_points[i].y;
        if (i <= 30.0) {
            double mid_error = (traced_points[i].x - (280 - (188-y)*1.34)-320)*(1-i/100);
            total_error += mid_error;
        }
        else {
            double mid_error = (traced_points[i].x - (280 - (188-y)*1.34)-320)*0.7 * exp(-0.064 * (i - 30.0));
            total_error += mid_error;
        }
        // line.push_back(Point2f(traced_points[i].x - (320 - (188-y)*1.34)-320,269-y));
    }

    // 可视化代码（例如在图像上绘制轨迹）
    for (int i=0;i<traced_points.size();i++) {
        int y = traced_points[i].y;
        Point pt = Point(traced_points[i].x - (280 - (188-y)*1.34),traced_points[i].y);
        circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
    }
    if (traced_points.size()==0){
        ROS_INFO("没有点，传了个空的点集来算误差");
        return 100.0;
    }
    else{
        return total_error/traced_points.size()*-1;
    }
}

bool line_server_callback(line_follow::line_follow::Request& req,line_follow::line_follow::Response& resp){
    FileStorage fs("/home/ucar/ucar_car/src/line_follow/camera_info/pinhole.yaml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "无法打开标定文件" << endl;
        return -1;
    }
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist twist;
    ROS_INFO("等待lidar_process服务中---");
    ros::ServiceClient client_line_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
    ztestnav2025::lidar_process board;
    board.request.lidar_process_start = -2;
    client_line_board.waitForExistence();
    ROS_INFO("等待坐标获取服务中---");
    ros::ServiceClient pose_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
    ztestnav2025::getpose_server pose;
    pose.request.getpose_start = 1;
    pose_client.waitForExistence();
    
    MoveBaseClient ac("move_base", true);
    ROS_INFO("等待movebase服务中---");
    // 等待服务器连接成功，可以设置一个超时时间，或者一直等待
    ac.waitForServer(); 
    ROS_INFO("move_base action server 已连接.");

    ros::ServiceClient reconfigure_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
    reconfigure_client.waitForExistence();
    dynamic_reconfigure::ReconfigureRequest request;
    dynamic_reconfigure::ReconfigureResponse response;
    dynamic_reconfigure::DoubleParameter planner_frequency;
    planner_frequency.name = "planner_frequency";
    planner_frequency.value = 0.0;
    request.config.doubles.push_back(planner_frequency);
    if (reconfigure_client.call(request, response)) {
        ROS_INFO("参数更新成功");
        double new_value;
        if (ros::param::get("/move_base/planner_frequency", new_value)) {
            ROS_INFO("Current planner_frequency: %.2f", new_value);
        }
    } else {
        ROS_ERROR("参数更新失败");
    }
    
    ROS_INFO("tf变换");
    tf::TransformListener* tf_listener_;
    tf_listener_ = new tf::TransformListener();

//-------------------------------------------标定去畸变----------------------------//
    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    // 1. 读取图像并转换为灰度图
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 5);
    Mat map1, map2;
    Mat optimalMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640, 480), 1,Size(640, 480));
    initUndistortRectifyMap(
        cameraMatrix, 
        distCoeffs, 
        Mat(), // 无旋转
        optimalMatrix, 
        Size(640, 480), 
        CV_32FC1, // 32位浮点类型（速度优化）
        map1, 
        map2
    );
    Mat image,undistorted;
    Rect roi(0, 210, 640, 270);

    double p,i,d,integrationy,integration,pre_error,leftpoint_p,leftpoint_I,leftpoint_D,x_max,integration_limit,out_turn,out_forward,yp,yi,yd;
    nh.getParam("/line_right/right_P", p);
    nh.getParam("/line_right/right_I", i);
    nh.getParam("/line_right/right_D", d);
    nh.getParam("/line_right/righty_P", yp);
    nh.getParam("/line_right/righty_I", yi);
    nh.getParam("/line_right/righty_D", yd);
    nh.getParam("/line_right/leftpoint_p", leftpoint_p);
    nh.getParam("/line_right/leftpoint_I", leftpoint_I);
    nh.getParam("/line_right/leftpoint_D", leftpoint_D);
    nh.getParam("/line_right/x_max_", x_max);
    nh.getParam("/line_right/integration_limit", integration_limit);
    nh.getParam("/line_right/out_forward", out_forward);
    nh.getParam("/line_right/out_turn", out_turn);
    ROS_INFO("参数加载P: %f", p);
    integration = 0;
    integrationy = 0;
    pre_error = 0;
    double pointx_integration = 0;
    double pointx_pre_error = 0;
    double pointy_integration = 0;
    double pointy_pre_error = 0;
    bool right = true;//判断现在是右边线还是左边线
    int first_point_x_last = 320;//上一帧的赛道起点，发生突变就说明右赛道变成左赛道了
    bool left_forward = true;
    bool point_forward = true;
    bool avoid_done = false;//避障结束才会停车

    out.open(output_file, fourcc, 5, Size(640, 270));
    displayStream << fixed << setprecision(2);

    ros::Time start_time = ros::Time::now();
    ros::Time frame_start = ros::Time::now();
    bool double_line = false;//最后进入两边巡线逻辑
    int point_confirm = 0;//要连续看到那个点20帧才行，不然会超调
            
    int height = 270;
    int width = 640;
    int scan_rows = 180;  // 向上搜索的行数

    while(ros::ok()){
        // ROS_INFO("耗时:%f",(ros::Time::now()-frame_start).toSec());
        // frame_start = ros::Time::now();
        //----------------------------------避障逻辑----------------------------//
        client_line_board.call(board);
        pose_client.call(pose);
        if(!avoid_done){
            if(board.response.lidar_results[0] != -1){
                if(board.response.lidar_results[0]>0.41){//如果还比较远先减速
                    x_max = 0.22;
                }
                else{
                    ROS_INFO("最短距离%f",board.response.lidar_results[0]);
                    // 立即停止当前的巡线控制
                    twist.linear.x = 0;
                    twist.linear.y = 0;
                    twist.angular.z = 0;
                    cmd_pub.publish(twist);
                    ros::Duration(0.1).sleep(); // 等待机器人停止，不加在平移前方向会偏转

                    // 获取避障起始Y坐标
                    pose_client.call(pose);
                    double initial_y = pose.response.pose_at[1];

                    // 定义避障过程中的目标点和姿态
                    double target_yaw = -1.57;      // 目标朝向，保持前进方向
                    double side_step_x = 3.25;      // 避障时横向平移到的X坐标
                    double track_x = 3.75;          // 原始赛道的X坐标
                    double forward_target_y = initial_y - board.response.lidar_results[0] - 0.19;

                    // P控制器参数
                    const double Kp_x = 1.5;      // X方向 (横向) P-gain
                    const double Kp_y = 1.5;      // Y方向 (前进) P-gain
                    const double Kp_yaw = 1.0;    // 角度 P-gain
                    const double max_vel_lateral = 0.35; // 最大横向速度
                    const double max_vel_forward = 0.35; // 最大前进速度
                    const double max_vel_yaw = 0.4;      // 最大角速度
                    const double tolerance_x = 0.02;     // X方向容忍误差
                    const double tolerance_y = 0.03;     // Y方向容忍误差
                    ros::Rate rate(20.0);                // 控制频率

                    // --- 第1步: 横向平移至 x = 3.25 ---
                    ROS_INFO("避障第1步: 横向平移至 x=%.2f", side_step_x);
                    while (ros::ok()) {
                        pose_client.call(pose);
                        double current_x = pose.response.pose_at[0];
                        double current_yaw = pose.response.pose_at[2];
                        
                        double error_x = side_step_x - current_x;
                        if (std::abs(error_x) < tolerance_x) {
                            break; // 到达目标点，完成第1步
                        }

                        // 保持朝向
                        double error_yaw = target_yaw - current_yaw;
                        error_yaw = atan2(sin(error_yaw), cos(error_yaw));

                        // 给机器人一个负的本地Y轴速度
                        twist.linear.y = Kp_x * error_x;

                        // 限制速度
                        twist.linear.y = std::max(-max_vel_lateral, std::min(max_vel_lateral, twist.linear.y));
                        twist.linear.x = 0; // 此阶段不前进
                        twist.angular.z = Kp_yaw * error_yaw;
                        twist.angular.z = std::max(-max_vel_yaw, std::min(max_vel_yaw, twist.angular.z));

                        cmd_pub.publish(twist);
                        rate.sleep();
                    }
                    // 停止运动
                    twist.linear.x = 0; twist.linear.y = 0; twist.angular.z = 0; cmd_pub.publish(twist);

                    // --- 第2步: 前进至目标Y点 ---
                    ROS_INFO("避障第2步: 前进至 y=%.2f", forward_target_y);
                    while (ros::ok()) {
                        pose_client.call(pose);
                        double current_y = pose.response.pose_at[1];
                        double current_x = pose.response.pose_at[0];
                        double current_yaw = pose.response.pose_at[2];

                        double error_y = forward_target_y - current_y;
                        if (std::abs(error_y) < tolerance_y) {
                            break; // 到达目标点，完成第2步
                        }

                        // 机器人本地X轴为前进方向, 对应地图-Y轴
                        // error_y为负, 需要正的本地X轴速度
                        twist.linear.x = -Kp_x * error_y;
                        
                        // 同时修正横向和角度偏差
                        double error_x = side_step_x - current_x;
                        twist.linear.y = -Kp_y * error_x;
                        double error_yaw = target_yaw - current_yaw;
                        error_yaw = atan2(sin(error_yaw), cos(error_yaw));
                        twist.angular.z = Kp_yaw * error_yaw;

                        // 限制速度
                        twist.linear.x = std::max(0.0, std::min(max_vel_forward, twist.linear.x)); //只准前进
                        twist.linear.y = std::max(-max_vel_lateral, std::min(max_vel_lateral, twist.linear.y));
                        twist.angular.z = std::max(-max_vel_yaw, std::min(max_vel_yaw, twist.angular.z));

                        cmd_pub.publish(twist);
                        rate.sleep();
                    }
                    // 停止运动
                    twist.linear.x = 0; twist.linear.y = 0; twist.angular.z = 0; cmd_pub.publish(twist);

                    // --- 第3步: 横向平移回 x = 3.75 ---
                    ROS_INFO("避障第3步: 横向平移回 x=%.2f", track_x);
                    while (ros::ok()) {
                        pose_client.call(pose);
                        double current_x = pose.response.pose_at[0];
                        double current_yaw = pose.response.pose_at[2];
                        
                        double error_x = track_x - current_x;
                        if (std::abs(error_x) < tolerance_x) {
                            break; // 到达目标点，完成第3步
                        }

                        double error_yaw = target_yaw - current_yaw;
                        error_yaw = atan2(sin(error_yaw), cos(error_yaw));

                        // 给机器人一个正的本地Y轴速度
                        twist.linear.y = Kp_x * error_x;

                        // 限制速度
                        twist.linear.y = std::max(-max_vel_lateral, std::min(max_vel_lateral, twist.linear.y));
                        twist.linear.x = 0;
                        twist.angular.z = Kp_yaw * error_yaw;
                        twist.angular.z = std::max(-max_vel_yaw, std::min(max_vel_yaw, twist.angular.z));

                        cmd_pub.publish(twist);
                        rate.sleep();
                    }
                    // 避障动作完成，彻底停止机器人
                    twist.linear.x = 0; twist.linear.y = 0; twist.angular.z = 0; cmd_pub.publish(twist);

                    // --- 清理并切换回巡线模式 ---
                    cap.grab(); cap.grab(); cap.grab(); cap.grab(); cap.grab();
                    ROS_INFO("避障结束");
                    avoid_done = true;
                    nh.getParam("/line_right/x_max_", x_max);
                    double_line = true;
                    nh.getParam("/line_right/double_P", p);
                    nh.getParam("/line_right/double_I", i);
                    nh.getParam("/line_right/double_D", d);
                    ROS_INFO("p %f",p);
                    ROS_INFO("切换为双边巡线");
                }
            }
        }


        //----------------------------------巡线逻辑----------------------------//
        displayStream.str("");
        cap.read(image);
        if (image.empty()) continue;
        Mat cropped = image(roi);
        flip(cropped, cropped, 1);
        Mat gray_img;
        vector<Mat> channels;
        split(cropped, channels);
        gray_img = channels[2];//红色通道代替灰度图
        threshold_image(gray_img);
        // imshow("binary",gray_img);

//-------------------------------预处理完成，开始找赛道-----------------//
        vector<Point> start_points;
        // ros::Time test = ros::Time::now();
        start_points = find_track_edge(gray_img,cropped);
        RaceTrack racetrack;
        if(trace_edge(gray_img,start_points,racetrack,cropped)){//成功找到右线，正常巡线
            double line_error = error_calculater(racetrack.points,cropped);//有调试图片输出
            integration += line_error*0.03;
            integration = std::max(std::min(integration,abs(line_error)/integration_limit+1),-1*abs(line_error)/integration_limit-1);
            double diff = line_error - pre_error;
            diff = std::max(std::min(diff,50.0),-50.0);
            twist.linear.x = x_max / exp(abs(line_error) / 100.0);
            twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
            pre_error = line_error;
            displayStream << "error: " << line_error << "p: " << line_error*p << "i: " << integration*i << "d: " << diff*d<<"z: "<< twist.angular.z;
            string displayText = displayStream.str();
            putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            cmd_pub.publish(twist);
            out.write(cropped);
        }
        else{//丢线了，找左点去

        }
        // ROS_INFO("耗时%f",(ros::Time::now()-test).toSec());
        
        continue;

        // if ((right && (first_point_x_last - right_edge_point.x>250) &&pose.response.pose_at[0]>3.0)&&!double_line){//如果右边线丢了或者右边界首个点发生剧烈左移动
        //     right = false;
        //     ROS_INFO("找左侧点%d,%d",first_point_x_last,right_edge_point.x);
        // }
        // else if((!right && (right_edge_point.x-first_point_x_last>250||right_edge_point.x>500 )&&!double_line)){//左线发生剧烈偏移说明又看到右线了左跳右的幅度一般很剧烈|| (right_edge_point.y>170&&right_edge_point.x>300)
        //     right = true;
        //     ROS_INFO("找右侧线:%d,%d",right_edge_point.x,first_point_x_last);
        // }
        // vector<Point> traced_right,left_edge_points;
        // Point left_edge_point;
        // // 追踪右侧边线
        // // imshow("test",cropped);
        // // waitKey(1);
        // // continue;
        // // if(pose.response.pose_at[0]>3.72){
        // //     std::cout << "Press [Enter] to continue...";
        // //     std::cin.ignore(); // 清除缓冲区
        // //     std::cin.get();    // 等待回车
        // // }
        // bool right_checker = true;//右线不一定真的是右线，可能是太偏的左线，不接受右线向右倾斜，不满足条件切换逻辑
        // if (right) {
        //     first_point_x_last = right_edge_point.x;
        //     double line_error = 0,z_error = 0;
        //     if(!double_line){
        //         trace_edge(right_edge_point, gray_img, traced_right, right_checker, brightness_threshold, &cropped);
        //         line_error = error_calculater(traced_right,right_edge_point.y,z_error,cropped);//有调试图片输出
        //     }
        //     else{
        //         line_error = double_find(gray_img,brightness_threshold,cropped);
        //     }

        //     if(!right_checker){
        //         ROS_INFO("右线斜率出错，舍弃%f",twist.angular.z);//
        //         displayStream << "error_rightline: ";
        //         string displayText = displayStream.str();
        //         putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        //         if(!left_forward && !point_forward){
        //             twist.linear.x = 0;
        //             twist.angular.z = out_turn;
        //         }
        //         else{
        //             twist.angular.z = std::max(twist.angular.z-0.05,-0.3);
        //         }
        //     }
        //     else{
        //         // if (!left_forward && !point_forward && (ros::Time::now()-start_time).toSec()>1.0){
        //         if (!left_forward && !point_forward){
        //             double_line = true;
        //             nh.getParam("/line_right/double_P", p);
        //             nh.getParam("/line_right/double_I", i);
        //             nh.getParam("/line_right/double_D", d);
        //             ROS_INFO("p%f",p);
        //             ROS_INFO("双边巡线");
        //         }
        //         point_confirm = 0;
        //         left_forward = true;
        //         point_forward = true;

        //         integration += z_error*0.03;
        //         integration = std::max(std::min(integration,abs(line_error)/integration_limit+1),-1*abs(line_error)/integration_limit-1);
        //         double diff = line_error - pre_error;
        //         diff = std::max(std::min(diff,50.0),-50.0);
        //         twist.linear.x = x_max / exp(abs(line_error) / 100.0);
        //         twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
        //         pre_error = line_error;
        //         displayStream << "error: " << line_error << "p: " << line_error*p << "i: " << integration*i << "d: " << diff*d<<"z: "<< twist.angular.z;
        //         string displayText = displayStream.str();
        //         putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        //     }
        //     out.write(cropped);
        //     // ROS_INFO("积分项%f",integration);
            
        // } 
        // else {
        //     if(left_forward){
        //         if(point_forward){
        //             // ROS_INFO("左点");
        //             // start_time = ros::Time::now();
        //             find_left_edge(gray_img, left_edge_point,brightness_threshold,cropped);
        //             first_point_x_last = left_edge_point.x;

        //             double error_x = 320-left_edge_point.x;
        //             pointx_integration += error_x*0.02;
        //             ROS_INFO("左点位置%d",left_edge_point.y);
        //             if(left_edge_point.y > 200){
        //                 point_forward = false;
        //             }
        //             pointx_integration = std::max(std::min(pointx_integration,1.0),-1.0);
        //             double point_diff = error_x-pointx_pre_error;
        //             // twist.linear.x = std::max(twist.linear.x-0.2,0.0);
        //             twist.linear.x = 0.23;
        //             twist.angular.z = error_x*leftpoint_p + pointx_integration*leftpoint_I+point_diff*leftpoint_D;

        //             pointx_pre_error = error_x;
        //             displayStream <<"errorx:  "<<error_x<<"p:"<<error_x*leftpoint_p<<"i:"<<pointx_integration * leftpoint_I<<"d:"<<error_x*leftpoint_p;
        //             string displayText = displayStream.str();
        //             putText(cropped, displayText, Point(50, 50),
        //             FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        //             out.write(cropped);
        //         }
        //         else{
        //             // // ROS_INFO("左线");
        //             // if(find_left_line(gray_img,left_edge_points,brightness_threshold,cropped)){
        //                 left_forward = false;
        //             // }
        //             // else{
        //                 // twist.linear.x = 0.15;
        //                 // twist.angular.z = -0.05;
        //             // }
        //         }
        //     }
        //     else{
        //         // twist.linear.x = 0;
        //         // twist.angular.z = -0.8;
        //         ROS_INFO("转弯");
        //         twist.linear.x = out_forward;
        //         twist.angular.z = out_turn;
        //         out.write(cropped);
        //     }
        // }
        // int test;
        // if(avoid_done && pose.response.pose_at[2]>-1.8&&pose.response.pose_at[2]<-1.3&&pose.response.pose_at[0]>3.3&&pose.response.pose_at[0]<4.2&&pose.response.pose_at[1]<1){
        //     if(stop_car(gray_img,brightness_threshold,test,cropped)){
        //         ROS_INFO("巡线结束");
        //         twist.linear.x = 0;
        //         twist.angular.z = 0;
        //         cmd_pub.publish(twist);
        //         imshow("stop",cropped);
        //         waitKey(0);
        //         break;
        //     }
        // }
        
        // cmd_pub.publish(twist);
    }
    cap.release();
    out.release();
    return true;
}
int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "line_right");
    ros::NodeHandle nh_;
    ros::ServiceServer line_server = nh_.advertiseService("line_right", line_server_callback);
    ROS_INFO("视觉巡线初始化");
    ros::spin();
    return 0;
}