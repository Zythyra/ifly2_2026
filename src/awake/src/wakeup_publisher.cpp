// 包含必要的ROS头文件
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// 定义ArcInfo结构体
struct ArcInfo {
    bool detected;          // 是否检测到弧线
    double curvature;       // 曲率值
    Point center;           // 弧线中心
    bool should_rotate;     // 是否需要旋转
};

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

// 控制小车前进
void moveForward(ros::Publisher& cmd_pub, double speed = 0.2) {
    geometry_msgs::Twist twist;
    twist.linear.x = speed;  // 前进速度
    twist.angular.z = 0;     // 不旋转
    cmd_pub.publish(twist);
}

// 假设已实现的曲率计算函数
double calculateCurvature(const vector<Point>& contour) {
    // 这里应该是你已有的曲率计算实现
    // 仅作示例，返回一个模拟值
    return contour.size() > 20 ? 0.005 : 0.003;
}

// 环岛内持续弧线检测函数（聚焦左下角特定区域）
ArcInfo detectArcContinuous(Mat& gray_img, Mat& visualizeImg, int brightness_threshold, bool in_roundabout) {
    ArcInfo info = {false, 0, Point(-1, -1), false};
    if (!in_roundabout) return info;

    int img_width = gray_img.cols;
    int img_height = gray_img.rows;
    
    // 左下角检测区域
    int start_x = 0;
    int start_y = img_height - (img_height / 3);
    int area_width = img_width / 6;               
    int area_height = img_height / 3;             
    
    area_width = min(area_width, img_width - start_x);
    area_height = min(area_height, img_height - start_y);
    
    Rect searchArea(start_x, start_y, area_width, area_height);
    Mat roi = gray_img(searchArea);
    
    // 图像预处理
    Mat blurred, edges, binary;
    GaussianBlur(roi, blurred, Size(3, 3), 1);
    threshold(blurred, binary, brightness_threshold - 15, 255, THRESH_BINARY);
    Canny(binary, edges, 30, 100);
    
    // 提取轮廓
    vector<vector<Point>> contours;
    findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    double max_curvature = 0;
    vector<Point> best_arc;

    for (auto& contour : contours) {
        if (contour.size() < 10) continue;

        vector<Point> approx;
        approxPolyDP(contour, approx, 1.5, false);
        
        double curvature = calculateCurvature(approx);
        if (curvature < 0.004) continue;

        if (curvature > max_curvature) {
            max_curvature = curvature;
            best_arc = approx;
            info.detected = true;
        }
    }

    // 处理检测结果
    if (info.detected) {
        info.should_rotate = false;
        info.curvature = max_curvature;
        
        // 弧线可视化
        vector<Point> arc_points;
        for (auto p : best_arc) {
            arc_points.push_back(Point(p.x + searchArea.x, p.y + searchArea.y));
        }
        
        Moments m = moments(arc_points);
        info.center = Point(m.m10/m.m00, m.m01/m.m00);
        
        polylines(visualizeImg, arc_points, false, Scalar(0, 255, 255), 2);
        circle(visualizeImg, info.center, 4, Scalar(0, 165, 255), -1);
        putText(visualizeImg, "Arc: " + to_string(info.curvature).substr(0,5), 
                Point(info.center.x+5, info.center.y), 
                FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 255), 1);
    } else {
        info.should_rotate = true;
        
        putText(visualizeImg, "No arc - Rotate", 
                Point(searchArea.x + 10, searchArea.y + 20), 
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    }

    rectangle(visualizeImg, searchArea, Scalar(255, 0, 0), 2);
    return info;
}

// 主控制逻辑 - 使用ROS发布器和提供的旋转函数
void processNavigation(Mat& frame, ros::Publisher& cmd_pub) {
    Mat gray, visualize;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    frame.copyTo(visualize);
    
    bool in_roundabout = true;  // 根据实际情况设置
    ArcInfo arcInfo = detectArcContinuous(gray, visualize, 100, in_roundabout);
    
    // 根据检测结果调用相应的控制函数
    if (arcInfo.should_rotate) {
        // 未检测到弧线，调用逆时针旋转函数
        rotateCounterClockwise(cmd_pub, 0.15);  // 可调整旋转角度
        ROS_INFO("未检测到弧线，执行逆时针旋转");
    } else {
        // 检测到弧线，继续前进
        moveForward(cmd_pub, 0.2);  // 可调整前进速度
        ROS_INFO("检测到弧线，继续前进");
    }
    
    imshow("Navigation View", visualize);
    waitKey(1);
}

// 示例主函数（展示ROS节点初始化）
int main(int argc, char **argv) {
    ros::init(argc, argv, "awake");
    ros::NodeHandle n;
    
    // 创建速度控制发布器
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    // 这里应该是你的图像获取逻辑
    // 示例：创建一个空白图像用于测试
    Mat frame(480, 640, CV_8UC3, Scalar(0, 0, 0));
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // 处理导航逻辑
        processNavigation(frame, cmd_vel_pub);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
