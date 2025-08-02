#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include <random>
#include <string>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>
#include "line_follow/line_follow.h"
#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/lidar_process.h"

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
cv::VideoCapture cap;
// ------------------------------ 新增：弧线检测相关结构体与函数 ------------------------------
// 弧线信息结构体（存储检测结果）
struct ArcInfo {
    bool detected;       // 是否检测到弧线
    double curvature;    // 曲率（正值为左弧，负值为右弧）
    Point center;        // 弧线中心在图像中的位置
};

// 计算曲线曲率（判断是否为弧线）
double calculateCurvature(const vector<Point>& curve) {
    if (curve.size() < 5) return 0; // 至少需要5个点
    
    // 取曲线中间段计算曲率（更稳定）
    int mid = curve.size() / 2;
    Point p1 = curve[mid - 2];
    Point p2 = curve[mid];
    Point p3 = curve[mid + 2];
    
    // 计算三角形面积
    double area = 0.5 * abs(
        (p2.x - p1.x) * (p3.y - p1.y) - 
        (p2.y - p1.y) * (p3.x - p1.x)
    );
    
    // 计算三边长
    double a = norm(Point2f(p2) - Point2f(p3));
    double b = norm(Point2f(p1) - Point2f(p3));
    double c = norm(Point2f(p1) - Point2f(p2));
    
    // 曲率 = 4*面积/(a*b*c)（值越大曲线越弯曲）
    if (a * b * c < 1e-6) return 0;
    return (4 * area) / (a * b * c);
}


// 环岛内持续弧线检测函数（聚焦左下角特定区域）
ArcInfo detectArcContinuous(Mat& gray_img, Mat& visualizeImg, int brightness_threshold, bool in_roundabout) {
    ArcInfo info = {false, 0, Point(-1, -1)};
    if (!in_roundabout) return info;  // 非环岛内不执行检测

    // 计算图像尺寸
    int img_width = gray_img.cols;
    int img_height = gray_img.rows;
    
    // 定义左下角检测区域：宽度0-1/6，高度0-1/3
    int start_x = 0;
    int start_y = 0;
    int area_width = img_width / 6;   // 宽度占比1/6
    int area_height = img_height / 3; // 高度占比1/3
    
    // 确保区域不超出图像边界
    area_width = min(area_width, img_width - start_x);
    area_height = min(area_height, img_height - start_y);
    
    Rect searchArea(start_x, start_y, area_width, area_height);
    Mat roi = gray_img(searchArea);
    
    // 图像预处理（针对小区域优化）
    Mat blurred, edges, binary;
    GaussianBlur(roi, blurred, Size(3, 3), 1);  // 小核尺寸适配小区域
    threshold(blurred, binary, brightness_threshold - 15, 255, THRESH_BINARY);
    Canny(binary, edges, 30, 100);  // 降低阈值提高小区域边缘检测灵敏度
    
    // 提取轮廓并筛选弧线特征
    vector<vector<Point>> contours;
    findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    double max_curvature = 0;
    vector<Point> best_arc;

    for (auto& contour : contours) {
        if (contour.size() < 10) continue;  // 小区域接受更短轮廓

        // 拟合多边形（更高精度）
        vector<Point> approx;
        approxPolyDP(contour, approx, 1.5, false);
        
        // 计算曲率（小区域弧线特征更明显）
        double curvature = calculateCurvature(approx);
        if (curvature < 0.004) continue;  // 阈值适配小区域

        if (curvature > max_curvature) {
            max_curvature = curvature;
            best_arc = approx;
            info.detected = true;
        }
    }

    // 处理检测结果并映射到原图坐标
    if (info.detected) {
        vector<Point> arc_points;
        for (auto p : best_arc) {
            arc_points.push_back(Point(p.x + searchArea.x, p.y + searchArea.y));
        }
        
        // 计算弧线中心
        Moments m = moments(arc_points);
        info.center = Point(m.m10/m.m00, m.m01/m.m00);
        
        // 确定曲率方向
        Rect bbox = boundingRect(arc_points);
        info.curvature = max_curvature * (bbox.x + bbox.width/2 < area_width/2 ? 1 : -1);
        
        // 可视化标记
        polylines(visualizeImg, arc_points, false, Scalar(0, 255, 255), 2);
        circle(visualizeImg, info.center, 4, Scalar(0, 165, 255), -1);
        putText(visualizeImg, "Arc: " + to_string(info.curvature).substr(0,5), 
                Point(info.center.x+5, info.center.y), 
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 255), 1);
    }

    // 标记检测区域
    rectangle(visualizeImg, searchArea, Scalar(255, 0, 0), 2);
    return info;
}
    
    

// 控制小车轻微逆时针转动（寻找弧线时使用）
void rotateCounterClockwise(ros::Publisher& cmd_pub, double angle = 0.1) {
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = angle; // 逆时针转动（弧度）
    cmd_pub.publish(twist);
    ros::Duration(0.3).sleep();
    twist.angular.z = 0;
    cmd_pub.publish(twist);
}

// ------------------------------ 原有函数 ------------------------------
string output_file = "/home/ucar/ucar_car/src/line_follow/image/right2.avi";//录制视频避免网络传输卡顿
VideoWriter out;
int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D'); // MP4V编码
ostringstream displayStream;


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

bool checkNearbyLine(Mat& gray_img, Point candidate, int brightness_threshold) {
    int x = candidate.x;
    int y = candidate.y;
    int width = gray_img.cols;
    int height = gray_img.rows;

    // 1. 进一步限制位置：内边界不会太靠近边缘
    if (x < 100 || x > 540) { // 与亮度计算的x范围一致
        return false;
    }
    if (y < 100 || y > 220) { // 仅关注中下部有效赛道区域
        return false;
    }

    // 2. 同一行检查：内边界跳变方向应为“左暗右亮”（current < next）
    int consecutiveCount = 0;
    for (int dx = -3; dx <= 3; dx++) {
        int cx = x + dx;
        if (cx < 1 || cx >= width - 1) continue;

        int current = gray_img.at<uchar>(y, cx);
        int next = gray_img.at<uchar>(y, cx + 1);
        // 强调跳变方向：current < next（左暗右亮），且亮度差足够
        if (next >= brightness_threshold - 20 && current > 80 && current < next && (next - current) > 30) {
            consecutiveCount++;
        }
    }
    if (consecutiveCount < 2) { // 至少2个相邻点符合内边界方向
        return false;
    }

    // 3. 相邻行检查：内边界直线斜率应平缓（接近水平，避免外边界的陡峭边缘）
    vector<Point> edgePoints;
    edgePoints.push_back(candidate); // 加入当前点
    for (int dy = -2; dy <= 2; dy++) {
        if (dy == 0) continue;
        int cy = y + dy;
        if (cy < 100 || cy > 220) continue; // 限制在中下部

        for (int dx = -2; dx <= 2; dx++) {
            int cx = x + dx;
            if (cx < 100 || cx > 540) continue;

            int current = gray_img.at<uchar>(cy, cx);
            int next = gray_img.at<uchar>(cy, cx + 1);
            if (next >= brightness_threshold - 20 && current > 80 && current < next && (next - current) > 20) {
                edgePoints.push_back(Point(cx, cy));
                break;
            }
        }
    }

    // 计算直线斜率：内边界斜率应接近0（平缓），外边界可能更陡峭
    if (edgePoints.size() < 3) return false; // 至少3个点才能拟合直线
    Vec4f line;
    fitLine(edgePoints, line, DIST_L2, 0, 0.01, 0.01);
    double slope = fabs(line[1] / line[0]); // 斜率绝对值
    return slope < 0.5; // 过滤斜率陡峭的外边界（假设内边界平缓）
}

int brightness_threshold_calculator(Mat& gray_img, Mat& visualizeImg) {
    int max_brightness_change = 0;
    int best_binary_brightness = 180; // 默认阈值
    Point threshold_keypoint;

    // 有效区域约束：排除图像边缘（假设内边界在x=100~540之间）
    const int MIN_X = 100;
    const int MAX_X = 540;
    // 优先选择中下部区域（y越大越靠近小车，权重越高）
    const int MIN_Y = 100; // 仅关注图像中下部

    for (int y = 220; y > MIN_Y; y--) { // 从底部向上搜索，优先保留下方的点
        for (int x = MIN_X; x < MAX_X; x++) { // 限制x在中间区域，排除边缘外边界
            int current = (int)gray_img.at<uchar>(y, x);
            int next = (int)gray_img.at<uchar>(y, x + 1);

            // 内边界特征：左侧暗（current适中）、右侧亮（next高）、跳变方向为暗→亮
            if (next >= 150 && current > 80 && current < next) { // 关键：current < next（暗→亮）
                int brightness_change = next - current;
                // 下方的点权重更高（乘以y相关系数，y越大权重越高）
                double weight = 1.0 + (y - MIN_Y) / 100.0; // 底部点权重提升
                int weighted_change = brightness_change * weight;

                if (weighted_change > max_brightness_change) {
                    Point candidate(x, y);
                    // 验证是否属于连续直线（排除孤立外边界）
                    if (checkNearbyLine(gray_img, candidate, next)) {
                        max_brightness_change = weighted_change;
                        best_binary_brightness = next - 20;
                        threshold_keypoint = candidate;
                    }
                }
            }
        }
    }

    circle(visualizeImg, threshold_keypoint, 7, Scalar(0, 255, 255), -1);
    return best_binary_brightness;
}

bool stop_car(Mat& gray,int brightness_threshold,int& point,Mat& visual_img){
    int white_count = 0;
    for (int y = 227; y >= 200; y--) {//
        for (int x = 1; x < 639; x++) {
            if (gray.at<uchar>(y, x)>=brightness_threshold){ 
                white_count++;
                circle(visual_img, Point(x,y), 2, Scalar(0, 0, 0), -1);
            }
        }
    }
    point = white_count;
    ROS_INFO("停车白点数量%d",white_count);
    if (white_count>4058){
        return true;
    }
    return false;
}

// 从图像底部向上搜索指定行数，分别独立寻找左右两侧的赛道边缘起始点
void find_righttrack_edge(Mat& gray_img, Point& right_point, int scan_rows, int brightness_threshold,Mat visualizeImg) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int middle_x = width / 2;
    bool flag = 1;
    int last_scanned_y = height - scan_rows;

    for (int y = height - 1; y >= last_scanned_y; y--) {
        // 向右搜索边界
        if (right_point.x == -1) {
            for (int x = middle_x + 1; x < width - 2; x++) {
                if ((int)gray_img.at<uchar>(y, x)>=brightness_threshold){ 
                    right_point = Point(x, y);
                    circle(visualizeImg, right_point, 7, Scalar(0, 0, 0), -1);
                    break;
                }
            }
        }
        else break;
    }
}

// 从起始点开始追踪赛道边线（添加断裂检测机制）
void trace_rightedge(Point start_point, Mat& gray_img, vector<Point>& traced_points, bool& right,  
                int brightness_threshold,Mat* visual_img = nullptr) {
    int height = gray_img.rows;
    int width = gray_img.cols;
    int search_range = 60;
    traced_points.clear();
    traced_points.push_back(start_point);
    bool broken = false;
    // 计数器：记录连续未找到点的行数
    int fail_count = 0;
    
    // 初始化搜索中心
    int center_x = start_point.x;
    int center_y = start_point.y - 1;  // 从起始点上方开始搜索
    int number = 1;//没必要找太多，60个点够了

    while (center_y > start_point.y-100) {  // 只看往上130行
        bool found = false;
        Point best_point;
        int max_brightness_change = 0;

        // 在当前行搜索范围内检查所有可能点
        for (int dx = 0; dx <= search_range/2; dx++) {
            // 计算候选点位置
            int cand_x = center_x + dx;
            int cand_x2 = center_x - dx;
            bool left_check = 1;
            bool right_check = 1;
            // 边界检查
            if (cand_x >= width - 1) {
                right_check = 0;
            }
            if (cand_x2 < 1) {
                left_check = 0;
            }

            //根据阈值
            int brightness_change;
            //右减左
            int current = 0;
            if (left_check){
                current = gray_img.at<uchar>(center_y, cand_x2);
                int prev = gray_img.at<uchar>(center_y, cand_x2 - 1);
                brightness_change = current - prev;
            }
            if (current >= brightness_threshold) {
                if (brightness_change > max_brightness_change) {
                    max_brightness_change = brightness_change;
                    best_point = Point(cand_x2, center_y);
                    found = true;
                }
            }
            else{
                if (right_check) {
                    current = gray_img.at<uchar>(center_y, cand_x);
                    int next = gray_img.at<uchar>(center_y, cand_x + 1);
                    brightness_change = next-current;
                } 
                if (current >= brightness_threshold) {
                    if (brightness_change > max_brightness_change) {
                        max_brightness_change = brightness_change;
                        best_point = Point(cand_x, center_y);
                        found = true;
                    }
                }
            }
        }

        if (found) {
            traced_points.push_back(best_point);
            // 重置失败计数器
            fail_count = 0;
            number++;
            // 更新搜索中心（继续向上移动）
            center_x = best_point.x;
            center_y = best_point.y - 1;

        } else {
            // 没有找到符合条件的点
            fail_count++;
            // 向上移动一行继续搜索
            center_y--;
            // 如果连续40行找不到点，判定为赛道断裂
            if (fail_count >= 20) {
                broken = true;
                break;
            }
        }
        // 如果已经到达图像顶部，结束追踪
        if (number>60|| center_y <= 0) {
            break;
        }
    }
    
    Vec4f lineParams; // 存放结果的 Vec4f
    fitLine(traced_points, lineParams, DIST_L2, 0, 0.01, 0.01);
    // ROS_INFO("有效点数%zu",traced_points.size());
    if((lineParams[1]/lineParams[0]<0.05&&lineParams[1]/lineParams[0]>-10)||traced_points.size()<15){//不接受右线向右倾斜数量太少不要
        right = false;
    }
    // 可视化追踪过程
    if (visual_img != nullptr) {
        Scalar color = Scalar(0, 255, 0);  // 红色:左, 绿色:右
        for (const auto& point : traced_points) {
            circle(*visual_img, point, 2, color, -1);
        }
        circle(*visual_img, start_point, 2, Scalar(0, 0, 0), -1);
        ostringstream displayStream1;
        displayStream1 << fixed << setprecision(2);
        displayStream1 << "line_slope:  " << lineParams[1]/lineParams[0];
        string displayText1 = displayStream1.str();
        putText(*visual_img, displayText1, Point(50, 100),
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        if(!right){
            circle(*visual_img, Point(320,100), 7, Scalar(255, 0, 255), -1);
        }
    }
}

//如果右边丢线，就只看左边，因为右边丢线了，所以直接从最右边开始找，找到就是左线，然后把左线拟合成直线，如果左线碰到图片底端，就开始旋转
bool find_left_edge(Mat gray_img,Point& left_edge_point,int brightness_threshold,Mat& visualizeImg){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = false;
    left_edge_point = Point(-1,-1);
    for (int y = height - 1; y >= 69; y--) {
        for (int x = width -1; x > 150; x--) {
            if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                if(gray_img.at<uchar>(y-1, x) >= brightness_threshold){//需要连续看到两个点
                    left_edge_point=Point(x, y);
                    flag = true;
                }
                break;
            }
        }
        if (flag) break;
    }
    if(left_edge_point.x == -1){
        ROS_INFO("没找到左点");
        return false;
    } 
    else {
        if (!visualizeImg.empty()) {
            circle(visualizeImg, left_edge_point, 7, Scalar(0, 0, 255), -1);
        }
        return false;
    }
}

Point find_other_coner_edge(Mat gray_img,Point left_edge_point,int brightness_threshold,Mat& visualizeImg){//拐角的特征是这一行有下一行没
    int height = gray_img.rows;
    int width = gray_img.cols;
    Point maybe_point = Point(-1,-1),last_point = Point(-1,-1);
    if(left_edge_point.x<280){
        Point first_point = Point(-1,-1);
        bool flag1 = false,flag2 = false;
        for(int x=100;x<600;x++){//先把第一个点找出来
            if (!flag1 && gray_img.at<uchar>(50, x) >= brightness_threshold){
                first_point = Point(x, 50);
                ROS_INFO("找到首点%d",first_point.x);
                flag1 = true;
            }
            if (!flag2 && gray_img.at<uchar>(51, x) >= brightness_threshold){
                last_point = Point(x, 51);
                flag2 = true;
            }
        }
        if (!flag1) {
            ROS_INFO("第一个点都没找到");
            return Point(-1, -1);
        }
        int finded_count = 0;//连续三行满足条件退出
        maybe_point = first_point;
        circle(visualizeImg, maybe_point, 5, Scalar(255, 0, 0), -1);
        for (int y = 52; y <= 180; y++) {
            bool flag = false;
            for (int x = maybe_point.x+80; x > 0; x--) {
                if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                    if((x-last_point.x)*(x-last_point.x)+(y-last_point.y)*(y-last_point.y)<100){
                        maybe_point=Point(x, y);
                        circle(visualizeImg, maybe_point, 5, Scalar(255, 90, 100), -1);
                        flag = true;
                        finded_count = 0;
                        last_point = maybe_point;
                        break;
                    }
                }
            }
            if (!flag) {
                finded_count++;
                if(finded_count>4){
                    if(maybe_point.x>520) return Point(-1,-1);
                    circle(visualizeImg, maybe_point, 9, Scalar(0, 0, 255), -1);
                    return maybe_point;
                }
            }
        }
        return Point(-1,-1);
    }
    else{
        maybe_point = left_edge_point;
        int find_atleast_4 = 0;
        for (int y = 230; y > 55; y--) {
            bool flag = false;
            for (int x = 550; x > 150; x--) {
                if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                    int finded_count = 0;//下两行不能有超过5个白点
                    for(int i=x-10;i<x+10;i++){
                        if(gray_img.at<uchar>(y+1, i) >= brightness_threshold) finded_count++;
                        if(gray_img.at<uchar>(y+2, i) >= brightness_threshold) finded_count++;
                        if(finded_count>3) break;
                    }
                    if(finded_count<3){
                        maybe_point=Point(x, y);
                        circle(visualizeImg, maybe_point, 9, Scalar(0, 0, 255), -1);
                        flag = true;
                    }
                    break;
                }
            }
            if (flag) {
                return maybe_point;
            }
        }
        return Point(-1,-1);
    }
}

bool find_left_line(Mat gray_img,vector<Point>& left_edge_points,int brightness_threshold,Mat visualizeImg = Mat()){
    int height = gray_img.rows;
    int width = gray_img.cols;
    bool flag = 1;
    for (int y = height - 1; y >= 69; y--) {
        for (int x = width -1; x > 1; x--) {
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

bool right_to_left(Mat gray_img,int brightness_threshold,bool& left_ready){//右巡线向左巡线跳变
    int height = gray_img.rows;
    int width = gray_img.cols;
    
    if(left_ready){
        int left_count = 0;//左边出现断线，即将进入圆环，圆环较近时进入左巡线
        for (int y = 279; y >= 200; y--) {
            for (int x = 320; x > 1; x--) {
                if ((int)gray_img.at<uchar>(y, x) >= brightness_threshold) {
                    left_count++;
                    break;
                }
            }
        }
        if(left_count>30){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        int left_count = 0; //左边尚未断线，等待进入圆环
        for (int y = 279; y >= 69; y--) {
            for (int x = 320; x > 1; x--) {
                if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                    left_count++;
                    break;
                }
            }
        }
        if(left_count<50){
            left_ready = true;//左线断裂，准备进入圆环
        }
        return false;
    }
}

int recently_white(Mat gray_img,int brightness_threshold,Mat& visualizeImg){//回到路口的时候，只能看到正前方有白线
    int recent = 0;
    for (int x = 310; x <330; x++) {
        int max_bright = 0,best_y = 0;
        for (int y = 269; y >= 50; y--) {
            if (gray_img.at<uchar>(y, x) >= max_bright) {
                max_bright = gray_img.at<uchar>(y, x);
                best_y = y;
            }
        }
        recent += best_y;
    }
    recent /= 20;
    circle(visualizeImg, Point(320,recent), 9, Scalar(0, 0, 180), -1);
    return recent;
}

double double_find(Mat gray_img,int brightness_threshold, Mat& visual_img)//最后阶段采用双边巡线
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
    falg = false;
    failed = 0;
    for (int y = 269; y >= 50; y--) {
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
        error += (640-(left_total[i]+right_total[i]))*(1-i/row);
    }

    int minSize = min(left_total.size(), right_total.size());
    for (int i = 0; i < minSize; i++) {
        int midX = (left_total[i] + right_total[i]) / 2;
        int y = 269 - i;
        midPoints.push_back(Point(midX, y));
    }

    for (const Point& p : midPoints) {
        circle(visual_img, p, 1, Scalar(0, 255, 255), -1);
    }

    return error/row;
}

double right_error_calculater(vector<Point>& traced_points,int ystart,Mat& visualizeImg){
    double total_error = 0;
    for (size_t i=0;i<traced_points.size();i++){
        int y = ystart-i;
        if (i <= 30.0) {
            double mid_error = (traced_points[i].x - (280 - (214-y)*1.34)-320)*(1-i/100);
            total_error += mid_error;
        }
        else {
            double mid_error = (traced_points[i].x - (280 - (214-y)*1.34)-320)*0.7 * exp(-0.064 * (i - 30.0));
            total_error += mid_error;
        }
    }

    for (int i=0;i<traced_points.size();i++) {
        int y = ystart-i;
        Point pt = Point(traced_points[i].x - (280 - (214-y)*1.34),ystart-i);
        circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
    }
    if (traced_points.size()==0){
        return 100.0;
    }
    else{
        return total_error/traced_points.size()*-1;
    }
}

// ------------------------------ 主回调函数（含新增弧线检测逻辑） ------------------------------
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
    ac.waitForServer(); 
    ROS_INFO("move_base action server 已连接.");

    cap.open(0);  // 打开默认摄像头（设备号0，若无效可尝试1）
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头！请检查设备是否连接正常");
        return false;  // 初始化失败则退出
    }
    // 设置摄像头分辨率（根据实际设备支持的参数调整）
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);


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


    Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    Mat map1, map2;
    Mat optimalMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640, 480), 1,Size(640, 480));
    initUndistortRectifyMap(
        cameraMatrix, 
        distCoeffs, 
        Mat(), // 无旋转
        optimalMatrix, 
        Size(640, 480), 
        CV_32FC1, 
        map1, 
        map2
    );
    Mat image,undistorted;
    Rect roi(0, 210, 640, 270);

    double p,i,d,integration,pre_error,leftpoint_p,leftpoint_I,x_max,other_enter_pointy,other_enter_pointx,integration_limit;
    nh.getParam("/right2/right_P", p);
    nh.getParam("/right2/right_I", i);
    nh.getParam("/right2/right_D", d);
    nh.getParam("/right2/leftpoint_p", leftpoint_p);
    nh.getParam("/right2/leftpoint_I", leftpoint_I);
    nh.getParam("/right2/x_max_", x_max);
    nh.getParam("/right2/other_enter_pointy", other_enter_pointy);
    nh.getParam("/right2/other_enter_pointx", other_enter_pointx);
    nh.getParam("/right2/integration_limit", integration_limit);
    ROS_INFO("参数加载P: %f", x_max);
    integration = 0;
    pre_error = 0;
    double pointx_integration = 0;
    double pointx_pre_error = 0;
    double pointy_integration = 0;
    double pointy_pre_error = 0;
    bool right = true;
    int first_point_x_last = 320;
    bool left_forward = true;
    bool point_forward = true;

    out.open(output_file, fourcc, 5, Size(640, 270));
    displayStream << fixed << setprecision(2);

    ros::Time start_time = ros::Time::now();
    ros::Time frame_start = ros::Time::now();
    bool double_line = false;
    int point_confirm = 0;
            
    int height = 270;
    int width = 640;
    int scan_rows = 180;

    bool avoid_done = false;

    bool out_range = false,start = true;
    bool other_enter = false,pass_out = false,pass_enter = false,out_ready = false,pass_enter_ready = false;
    bool start_other_enter = false;
    int out_ready_count = 0,other_enter_count = 0;
    bool left_ready;
    double position_right_change_left = -1;
    Point other_enter_last_conner = Point(-1,-1);

    // ------------------------------ 新增：环岛与弧线检测状态变量 ------------------------------
    bool in_roundabout = false;        // 是否处于环岛内（持续检测弧线的标志）
    int arc_lost_count = 0;            // 弧线丢失计数（连续丢失次数）
    const int MAX_ARC_LOST = 5;        // 允许连续丢失弧线的最大次数
    const double ARC_ASSIST_WEIGHT = 0.3;  // 弧线辅助转向的权重
    double last_arc_curvature = 0;     // 上一帧弧线曲率（用于平滑）

    while(ros::ok()){
        //----------------------------------避障逻辑----------------------------//
        client_line_board.call(board);
        pose_client.call(pose);
        if(board.response.lidar_results[0] != -1){
            ROS_INFO("最短距离%f",board.response.lidar_results[0]);
            if(board.response.lidar_results[0]>0.45){
                x_max = 0.22;
            }
            else{
                float vx = board.response.lidar_results[4];
                float vy = board.response.lidar_results[5];

                double d = std::sqrt(1 + board.response.lidar_results[3]*board.response.lidar_results[3]);

                if (out_range == false)
                {
                    geometry_msgs::PointStamped lidar_point;
                    lidar_point.header.frame_id = "laser_frame";
                    lidar_point.header.stamp = ros::Time(0);
                    lidar_point.point.x = board.response.lidar_results[1] - 0.26*vy;
                    lidar_point.point.y = board.response.lidar_results[2] + 0.26*vx;
                    lidar_point.point.z = 0;
                    geometry_msgs::PointStamped point_base;
                    tf_listener_->transformPoint("map", lidar_point, point_base);
            
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose.position.x = point_base.point.x;
                    goal.target_pose.pose.position.y = point_base.point.y;
                    double goal_yaw = std::atan2(vx, -vy) + pose.response.pose_at[2];
                    tf::Quaternion q = tf::createQuaternionFromYaw(goal_yaw);
                    geometry_msgs::Quaternion q_msg;
                    tf::quaternionTFToMsg(q, q_msg);
                    goal.target_pose.pose.orientation = q_msg;

                    ROS_INFO("坐标变换结果: (%.2f, %.2f, %.2f)",point_base.point.x, point_base.point.y, goal_yaw);
                    ac.sendGoal(goal);
                    ac.waitForResult();
                }
                else
                {
                    geometry_msgs::PointStamped lidar_point;
                    lidar_point.header.frame_id = "laser_frame";
                    lidar_point.header.stamp = ros::Time(0);
                    lidar_point.point.x = board.response.lidar_results[1] + 0.26;
                    lidar_point.point.y = board.response.lidar_results[2];
                    lidar_point.point.z = 0;
                    geometry_msgs::PointStamped point_base;
                    tf_listener_->transformPoint("map", lidar_point, point_base);
                                        
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose.position.x = 3.75;
                    goal.target_pose.pose.position.y = point_base.point.y;
                    double goal_yaw = -1.57;
                    tf::Quaternion q = tf::createQuaternionFromYaw(goal_yaw);
                    geometry_msgs::Quaternion q_msg;
                    tf::quaternionTFToMsg(q, q_msg);
                    goal.target_pose.pose.orientation = q_msg;
            

                    ROS_INFO("坐标变换结果: (%.2f, %.2f, %.2f)",goal.target_pose.pose.position.x, point_base.point.y, goal_yaw);
                    ac.sendGoal(goal);
                    ac.waitForResult();
                }
                
                cap.grab(); cap.grab(); cap.grab(); cap.grab(); cap.grab();
                avoid_done = true;
                ROS_INFO("避障结束");
                nh.getParam("/line_right/x_max_", x_max);
                double_line = true;
                nh.getParam("/line_right/double_P", p);
                nh.getParam("/line_right/double_I", i);
                nh.getParam("/line_right/double_D", d);
                ROS_INFO("p%f",p);
                ROS_INFO("双边巡线");
            }
        }

        //----------------------------------巡线逻辑----------------------------//
        displayStream.str("");
        cap.read(image);
        if (image.empty()) {
            ROS_INFO("获取图片失败");
            continue;
        }
        remap(image, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
        Mat cropped = undistorted(roi);
        flip(cropped, cropped, 1);
        Mat gray_img;
        vector<Mat> channels;
        split(cropped, channels);
        gray_img = channels[2];
        int brightness_threshold = brightness_threshold_calculator(gray_img,cropped);

        // ------------------------------ 新增：更新环岛状态（进入/退出） ------------------------------
        // 进入环岛的判断（基于原有pass_out状态扩展）
        bool enter_roundabout = (pass_out && pose.response.pose_at[0] > 3.0) || 
                               (other_enter && !pass_enter);
        
        // 离开环岛的判断（保留原有逻辑）
        bool exit_roundabout = (out_range && pose.response.pose_at[0] > 4.0) || 
                              (pose.response.pose_at[2] > -1.0 && pose.response.pose_at[0] > 4.2);

        // 同步环岛状态
        if (!in_roundabout && enter_roundabout) {
            in_roundabout = true;
            arc_lost_count = 0;
            last_arc_curvature = 0;
            ROS_INFO("进入环岛，开始持续弧线检测");
        } else if (in_roundabout && exit_roundabout) {
            in_roundabout = false;
            ROS_INFO("离开环岛，停止弧线检测");
        }

        // ------------------------------ 新增：持续弧线检测（仅在环岛内） ------------------------------
        ArcInfo arc_info = detectArcContinuous(gray_img, cropped, brightness_threshold, in_roundabout);

        // ------------------------------ 新增：处理弧线检测结果（影响巡线逻辑） ------------------------------
        if (in_roundabout) {
            // 弧线丢失计数管理
            arc_info.detected ? arc_lost_count = 0 : arc_lost_count++;

            // 若连续丢失弧线，启动转向调整（不中断原有巡线）
            if (arc_lost_count > MAX_ARC_LOST) {
                twist.angular.z += 0.1;  // 附加转动量
                twist.angular.z = min(twist.angular.z, 0.8);  // 限制最大转向
                putText(cropped, "Searching Arc...", Point(50, 220), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
            }
        }

        //-----------------------------原有逻辑：判断是否需要右转左-----------------------------//
        if(!out_range && !pass_out && !other_enter && !pass_enter && !out_ready && !pass_enter_ready){
            pass_out = right_to_left(gray_img,brightness_threshold,left_ready);
            position_right_change_left = pose.response.pose_at[1];
            if(pass_out) {
                ROS_INFO("第一次即将抵达出口%f",position_right_change_left);
            }
        }

        if(!out_range && !pass_out && other_enter && !pass_enter && !out_ready){
            other_enter_last_conner = find_other_coner_edge(gray_img,other_enter_last_conner,brightness_threshold,cropped);
            if(other_enter_last_conner.x != -1){
                other_enter_count++;
                if(other_enter_count>10){
                    start_other_enter = true;
                }
                
                twist.linear.x = (205-other_enter_last_conner.y)*other_enter_pointy+0.08;
                twist.angular.z = (553-other_enter_last_conner.x)*other_enter_pointx*(other_enter_last_conner.y*0.0061+0.28);
                cmd_pub.publish(twist);
                displayStream <<"x:"<< twist.linear.x<<"z:"<< twist.angular.z<<"erx:"<<205-other_enter_last_conner.y<<"ery:"<<553-other_enter_last_conner.x;
                string displayText = displayStream.str();
                putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                out.write(cropped);
                if(other_enter_last_conner.y>200){
                    other_enter = false;
                    pass_enter = true;
                    ROS_INFO("离开另一个路口");
                }
                continue;
            }
        }

        //-----------回到路口--------//
        if(pass_enter_ready){
            ROS_INFO("回到路口特殊逻辑");
            int recent = recently_white(gray_img,brightness_threshold,cropped);
            if(recent<150){
                twist.linear.x = 0.3;
                twist.angular.z = 0;
            }
            else{
                twist.linear.x = 0;
                twist.angular.z = 0.6;
            }
            if(pose.response.pose_at[2]>-2.355 && pose.response.pose_at[2]< -0.7){
                out_range = true;
                pass_enter = false;
                pass_enter_ready = false;
                ROS_INFO("准备离开圆环");
            }
            displayStream <<"z:"<< twist.angular.z<<"x:  "<<twist.linear.x<<"recent:"<<recent;
            string displayText = displayStream.str();
            putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            out.write(cropped);
            cmd_pub.publish(twist);
            continue;
        }
        
        //---------------------------右巡线与左巡线切换逻辑--------------------//
        Point right_edge_point = Point(-1, -1);
        int last_scanned_y;
        find_righttrack_edge(gray_img,right_edge_point, scan_rows, brightness_threshold,cropped);

        // 原有右→左切换逻辑（完全保留）
        if ((out_range || pass_out) && right && (first_point_x_last - right_edge_point.x > 250) && pose.response.pose_at[0] > 3.0) {
            right = false;
            ROS_INFO("左跳变%d,%d", first_point_x_last, right_edge_point.x);
        }
        // 原有左→右切换逻辑（完全保留）
        else if (!right && (right_edge_point.x - first_point_x_last > 250 || right_edge_point.x > 500)) {
            right = true;
            ROS_INFO("右跳变:%d,%d", right_edge_point.x, first_point_x_last);
            if (pass_out) {
                pass_out = false;
                other_enter = true;  
                ROS_INFO("到达另一个入口");
            }                
        }

        //---------------------------主巡线逻辑--------------------//
        vector<Point> traced_right,left_edge_points;
        Point left_edge_point;
        // 追踪右侧边线
        bool right_checker = true;
        if (right) {
            double line_error = 0;
            if(!double_line){
                trace_rightedge(right_edge_point, gray_img, traced_right, right_checker, brightness_threshold, &cropped);
                line_error = right_error_calculater(traced_right,right_edge_point.y,cropped);
            }
            else{
                line_error = double_find(gray_img,brightness_threshold,cropped);
            }
            if(!right_checker){
                displayStream <<"point: "<< traced_right.size();
                string displayText = displayStream.str();
                putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                
                if(pass_enter){
                    pass_enter_ready = true;
                    ROS_INFO("回到路口");
                    continue;
                }
                else{
                    cmd_pub.publish(twist);
                    out.write(cropped);
                    ROS_INFO("丢线，保持原来运动状态");
                    continue;
                }
                
            }
            else{
                if (out_range && !left_forward && !point_forward && (ros::Time::now()-start_time).toSec()>1.0){
                    double_line = true;
                    nh.getParam("/line_right/double_P", p);
                    nh.getParam("/line_right/double_I", i);
                    nh.getParam("/line_right/double_D", d);
                    ROS_INFO("p%f",p);
                    ROS_INFO("双边巡线");
                }

                point_confirm = 0;
                left_forward = true;
                point_forward = true;

                first_point_x_last = right_edge_point.x;
                integration += line_error*0.03;
                integration = std::max(std::min(integration,abs(line_error)/integration_limit+1),-1*abs(line_error)/integration_limit-1);
                double diff = line_error - pre_error;
                diff = std::max(std::min(diff,50.0),-50.0);
                if(avoid_done){
                    twist.linear.x = 0;
                    twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
                    ROS_INFO("避障结束右线%f",twist.linear.y);
                    if(line_error<20){
                        avoid_done = false;
                    }
                }
                else{
                    twist.linear.x = x_max / exp(abs(line_error) / 100.0);
                    twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
                }

                // ------------------------------ 新增：弧线辅助转向（仅在环岛内） ------------------------------
                if (in_roundabout && arc_info.detected) {
                    double arc_steer = arc_info.curvature * ARC_ASSIST_WEIGHT;
                    twist.angular.z += arc_steer;
                    twist.angular.z = max(min(twist.angular.z, 1.0), -1.0);
                }

                pre_error = line_error;
                displayStream << "error: " << line_error << "z:" << twist.angular.z << "twist.x: " << twist.linear.x << "d: " << diff*d<< "integration"<<integration*i;

                string displayText = displayStream.str();
                putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                out.write(cropped);
            }
        } 
        else {
            if(left_forward){
                if(point_forward){
                    ROS_INFO("左点");
                    if(pass_out){
                        find_left_edge(gray_img, left_edge_point,brightness_threshold,cropped);
                        twist.linear.x = (205-left_edge_point.y)*other_enter_pointx+0.05;
                        twist.angular.z = (553-left_edge_point.x)*other_enter_pointy+0.1;
                        if(left_edge_point.x>450 && left_edge_point.y >180){
                            right = true;
                            other_enter = true;
                            pass_out = false;
                            ROS_INFO("到达另一个入口");
                        }
                        displayStream <<"x:"<< twist.linear.x<<"z:"<< twist.angular.z<<"erx:"<<205-left_edge_point.y<<"ery:"<<553-left_edge_point.x;
                        string displayText = displayStream.str();
                        putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                        out.write(cropped);
                    }
                    else{
                        start_time = ros::Time::now();
                        find_left_edge(gray_img, left_edge_point,brightness_threshold,cropped);
                        first_point_x_last = left_edge_point.x;
                        double error_x = 320-left_edge_point.x;
                        pointx_integration += error_x*0.02;
                        if(abs(left_edge_point.x-320) < 20){
                            avoid_done = false;
                            point_confirm++;
                            if(point_confirm>7){
                                pointx_integration = 0;
                                point_forward = false;
                            }
                        }
                        pointx_integration = std::max(std::min(pointx_integration,1.0),-1.0);
                        twist.linear.x = std::max(twist.linear.x-0.2,0.0);
                        twist.angular.z = std::max(std::min(error_x*leftpoint_p + pointx_integration * leftpoint_I,0.5),-0.5);

                        pointx_pre_error = error_x;
                        displayStream <<"z:  "<< twist.angular.z<<"errorx:  "<<error_x<<"pointx_integration:"<<pointx_integration;
                        string displayText = displayStream.str();
                        putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                        out.write(cropped);
                    }
                }
                else{
                    if(find_left_line(gray_img,left_edge_points,brightness_threshold,cropped)){
                        left_forward = false;
                    }
                    else{
                        twist.linear.x = 0.15;
                        twist.angular.z = -0.05;
                    }
                }
            }
            else{
                twist.linear.x = 0;
                twist.angular.z = -0.8;
                out.write(cropped);
            }
        }
 
        int test;
        if(pose.response.pose_at[2]>-1.8&&pose.response.pose_at[2]<-1.3&&pose.response.pose_at[0]>3.3&&pose.response.pose_at[0]<4.2&&pose.response.pose_at[1]<1){
            if(stop_car(gray_img,brightness_threshold,test,cropped)){
                ROS_INFO("巡线结束");
                twist.linear.x = 0;
                twist.angular.z = 0;
                cmd_pub.publish(twist);
                imshow("stop",cropped);
                waitKey(0);
                break;
            }
        }
        
        cmd_pub.publish(twist);
    }
    cap.release();
    out.release();
    return true;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "right2");
    ros::NodeHandle nh_;
    ros::ServiceServer line_server = nh_.advertiseService("right2", line_server_callback);
    ROS_INFO("视觉巡线初始化");
    if(ros::ok()) ros::spin();
    ROS_INFO("结束");
    return 0;
}
    