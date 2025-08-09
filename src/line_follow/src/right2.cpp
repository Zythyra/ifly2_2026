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
//进巡线的时候不能停到里面，不确定赛道起点形状
string output_file = "/home/ucar/ucar_car/src/line_follow/image/right2.avi";//录制视频避免网络传输卡顿
VideoWriter out;
int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D'); // MP4V编码
ostringstream displayStream;

void threshold_image(Mat& gray) {
        int adaptive_block = 45;
        int adaptive_c = -15;
        int min_contour_area = 250;

        Mat binary;
        adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, adaptive_block, adaptive_c);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Mat denoised = Mat::zeros(binary.size(), CV_8UC1);
        for (size_t i = 0; i < contours.size(); i++) {
            if (contourArea(contours[i]) > min_contour_area) {
                drawContours(denoised, contours, i, Scalar(255), FILLED);
            }
        }
        gray = denoised.clone();
    }



bool stop_car(Mat& gray, int& point, Mat& visual_img) {
    int white_count = 0;
    for (int y = 227; y >= 200; y--) {
        for (int x = 1; x < 639; x++) {
            if (gray.at<uchar>(y, x) == 255) {
                white_count++;
                circle(visual_img, Point(x, y), 2, Scalar(0, 0, 0), -1);
            }
        }
    }
    point = white_count;
    return white_count > 2058;
}

// 寻找赛道边缘起点
vector<Point> find_track_edge(Mat& gray_img, int bottom_trace_end, int right_trace_end, Mat& visual_img) {
    bool is_now_white = false;
    vector<Point> maybe_start_point;

    // 底部寻找
    for (int i = 639; i > bottom_trace_end; i--) {
        if (!is_now_white && gray_img.at<uchar>(269, i) == 255) {
            is_now_white = true;
        }
        if (is_now_white && gray_img.at<uchar>(269, i) == 0) {
            maybe_start_point.emplace_back(i-1, 269);
            circle(visual_img, Point(i-1, 269), 5, Scalar(0, 0, 255), -1);
            is_now_white = false;
        }
    }

    // 右部寻找
    is_now_white = true;
    for (int i = 269; i > right_trace_end; i--) {
        if (is_now_white && gray_img.at<uchar>(i, 639) == 0) {
            is_now_white = false;
        }
        if (!is_now_white && gray_img.at<uchar>(i, 639) == 255) {
            maybe_start_point.emplace_back(639, i);
            circle(visual_img, Point(639, i), 5, Scalar(0, 0, 255), -1);
            is_now_white = true;
        }
    }
    return maybe_start_point;
}

// 追踪赛道边缘（修正参数：将int& racetrack改为RaceTrack& racetrack）
bool trace_edge(Mat& gray_img, vector<Point> start_points, RaceTrack& racetrack, Mat& visual_img) {
    int point_number = start_points.size();
    vector<RaceTrack> racetracks(point_number);  // 现在可正常声明RaceTrack向量
    int height = gray_img.rows, width = gray_img.cols, search_range = 40;

    for (int idx = 0; idx < point_number; idx++) {
        bool broken = false, last_left = true, last_right = false;
        int fail_count = 0;
        Point start = start_points[idx];
        int center_x = start.x, center_y = start.y - 1;

        while (center_y > start.y - 100) {
            bool left_found = false, right_found = false;
            for (int dx = 0; dx <= search_range/2; dx++) {
                int cand_x = center_x + dx;
                int cand_x2 = center_x - dx;
                bool left_check = (cand_x2 > 1);
                bool right_check = (cand_x < width - 1);

                if (left_check && gray_img.at<uchar>(center_y, cand_x2) == 255 && gray_img.at<uchar>(center_y, cand_x2 - 1) == 0) {
                    racetracks[idx].points.emplace_back(cand_x2, center_y);
                    left_found = true;
                    center_x = cand_x2;
                }
                if (!left_found && right_check && gray_img.at<uchar>(center_y, cand_x) == 0 && gray_img.at<uchar>(center_y, cand_x + 1) == 255) {
                    right_found = true;
                    center_x = cand_x + 1;
                }
            }

            // 更新方向变化计数
            if (last_left && right_found) {
                racetracks[idx].direction_change++;
                last_left = false;
                last_right = true;
            }
            if (last_right && left_found) {
                racetracks[idx].direction_change++;
                last_left = true;
                last_right = false;
            }

            // 处理追踪结果
            if (left_found || right_found) {
                fail_count = 0;
                center_y--;
            } else {
                fail_count++;
                center_y--;
                if (fail_count >= 4) { broken = true; break; }
            }
            if (center_y <= 0) break;
        }

        // 计算斜率
        if (racetracks[idx].points.size() > 15) {
            Vec4f lineParams;
            fitLine(racetracks[idx].points, lineParams, DIST_L2, 0, 0.01, 0.01);
            racetracks[idx].slope = lineParams[1] / lineParams[0];
        } else {
            racetracks[idx].slope = -2.0;
        }
    }

    // 选择最优赛道
    int best_idx = -1;
    float min_dangerous = 2.1;
    for (int i = 0; i < point_number; i++) {
        if (!(racetracks[i].slope < 0.05 && racetracks[i].slope > -10)) {
            float ratio = racetracks[i].direction_change / (float)racetracks[i].points.size();
            if (ratio < min_dangerous) {
                min_dangerous = ratio;
                best_idx = i;
            }
        }
    }

    if (best_idx != -1) {
        racetrack = racetracks[best_idx];
        for (const auto& p : racetrack.points) {
            circle(visual_img, p, 2, Scalar(0, 255, 0), -1);
        }
        ostringstream oss;
        oss << "斜率: " << racetrack.slope << " 方向变化: " << racetrack.direction_change;
        putText(visual_img, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        return true;
    }
    return false;
}

// 寻找左边缘
bool find_left_edge(Mat gray_img, Point& left_point, Mat& visualizeImg) {
    bool is_now_white = false;
    vector<Point> maybe_left_point;

    // 左部寻找起点
    for (int i = 269; i > 2; i--) {
        if (is_now_white && gray_img.at<uchar>(i, 5) == 0) {
            is_now_white = false;
        }
        if (!is_now_white && gray_img.at<uchar>(i, 5) == 255) {
            maybe_left_point.emplace_back(5, i);
            circle(visualizeImg, Point(5, i), 9, Scalar(255, 0, 0), -1);
            is_now_white = true;
        }
    }

    int point_number = maybe_left_point.size();
    vector<RaceTrack> racetracks(point_number);  // 现在可正常声明
    int search_range = 40;

    // 追踪左边缘
    for (int idx = 0; idx < point_number; idx++) {
        bool broken = false, last_up = false, last_down = false;
        int fail_count = 0;
        Point start = maybe_left_point[idx];
        int center_x = start.x + 1, center_y = start.y;

        while (center_x < 620) {
            bool found = false;
            for (int dy = 0; dy <= search_range/2; dy++) {
                bool up_check = (center_y - dy > 2);
                bool down_check = (center_y + dy < 268);

                if (down_check && gray_img.at<uchar>(center_y + dy, center_x) == 255 && gray_img.at<uchar>(center_y + dy + 1, center_x) == 0) {
                    racetracks[idx].points.emplace_back(center_x, center_y + dy);
                    found = true;
                    center_y += dy;
                    if (last_up) racetracks[idx].direction_change++;
                    last_down = true;
                    last_up = false;
                }
                if (!found && up_check && gray_img.at<uchar>(center_y - dy, center_x) == 255 && gray_img.at<uchar>(center_y - dy + 1, center_x) == 0) {
                    racetracks[idx].points.emplace_back(center_x + 1, center_y - dy);
                    found = true;
                    center_y -= dy;
                    if (last_down) racetracks[idx].direction_change++;
                    last_down = false;
                    last_up = true;
                }
            }

            if (found) {
                fail_count = 0;
                center_x++;
            } else {
                fail_count++;
                center_x++;
                if (fail_count >= 20) { broken = true; break; }
            }
        }
        if (racetracks[idx].points.size() > 120) racetracks[idx].left_point = true;
    }

    // 选择最优左边缘
    int best_idx = -1;
    int lowest_y = 0;
    for (int i = 0; i < point_number; i++) {
        if (racetracks[i].left_point && racetracks[i].points[0].y > lowest_y) {
            lowest_y = racetracks[i].points[0].y;
            best_idx = i;
        }
    }

    if (best_idx != -1) {
        RaceTrack racetrack = racetracks[best_idx];  // 现在可正常使用
        Point best_point(0, 0);
        for (size_t i = 0; i < racetrack.points.size(); i += 3) {
            if (racetrack.points[i].y > best_point.y) best_point = racetrack.points[i];
            circle(visualizeImg, racetrack.points[i], 2, Scalar(255, 0, 0), -1);
        }
        circle(visualizeImg, best_point, 9, Scalar(0, 0, 255), -1);
        left_point = best_point;
        ostringstream oss;
        oss << "左点: (" << best_point.x << "," << best_point.y << ")";
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        return true;
    }
    return false;
}

Point find_other_coner_edge(Mat gray_img,Point left_edge_point,int brightness_threshold,Mat& visualizeImg){//拐角的特征是这一行有下一行没
    int height = gray_img.rows;
    int width = gray_img.cols;
    Point maybe_point = Point(-1,-1),last_point = Point(-1,-1);//计算斜率，如果边界线的点斜率突变了也是不能接受的
    if(left_edge_point.x<280){
        Point first_point = Point(-1,-1);
        bool flag1 = false,flag2 = false;
        for(int x=100;x<600;x++){//先把第一个点找出来
            if (!flag1 && gray_img.at<uchar>(50, x) >= brightness_threshold){
                first_point = Point(x, 50);
                // ROS_INFO("找到首点%d",first_point.x);
                flag1 = true;
            }
            if (!flag2 && gray_img.at<uchar>(51, x) >= brightness_threshold){
                last_point = Point(x, 51);
                // ROS_INFO("找到第二点%d",first_point.x);
                flag2 = true;
            }
        }
        if (!flag1) {
            ROS_INFO("第一个点都没找到");
            return Point(-1, -1);//第一个点都没找到就是还没有
        }
        int finded_count = 0;//连续三行满足条件退出
        maybe_point = first_point;
        circle(visualizeImg, maybe_point, 5, Scalar(255, 0, 0), -1);
        // double last_slope = (last_point.y - first_point.y)/(last_point.x - first_point.x);
        for (int y = 52; y <= 180; y++) {
            bool flag = false;
            for (int x = maybe_point.x+80; x > 0; x--) {
                if (gray_img.at<uchar>(y, x) >= brightness_threshold) {
                    if((x-last_point.x)*(x-last_point.x)+(y-last_point.y)*(y-last_point.y)<100){
                        maybe_point=Point(x, y);
                        // ROS_INFO("test");
                        circle(visualizeImg, maybe_point, 5, Scalar(255, 90, 100), -1);
                        flag = true;
                        finded_count = 0;
                        last_point = maybe_point;
                        break;//
                    }
                }
            }
            if (!flag) {
                finded_count++;
                if(finded_count>4){
                    if(maybe_point.x>520) return Point(-1,-1);//才刚刚看到角点,不接受点在右边
                    circle(visualizeImg, maybe_point, 9, Scalar(0, 0, 255), -1);
                    return maybe_point;
                }
            }
        }
        return Point(-1,-1);
    }
    else{
        maybe_point = left_edge_point;
        int find_atleast_4 = 0;//至少要找到4个有效的点，毕竟往上搞了15行
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

int recently_white(Mat gray_img,int brightness_threshold,Mat& visualizeImg){//回到路口的时候，只能看到正前方有白线，通过白线距离来判断怎么走
    int recent = 0;
    for (int x = 310; x <330; x++) {
        int max_bright = 0,best_y = 0;
        for (int y = 269; y >= 50; y--) {//找到最近的白点，一整列下来最亮的是白点
            if (gray_img.at<uchar>(y, x) >= max_bright) {
                max_bright = gray_img.at<uchar>(y, x);
                best_y = y;
            }
        }
        circle(visualizeImg, Point(x,best_y), 3, Scalar(0, 0, 0), -1);
        recent += best_y;
    }
    recent /= 20;//亮度最大那个点的平均位置
    circle(visualizeImg, Point(320,recent), 9, Scalar(0, 0, 180), -1);
    return recent;
}

// 双边巡线误差计算
double double_find(Mat gray_img, Mat& visual_img) {
    vector<int> left_total, right_total;
    double error = 0.0;
    vector<Point> midPoints;

    // 提取左边界
    bool falg = false;
    int failed = 0;
    for (int y = 269; y >= 50; y--) {
        int left = 0;
        bool find = false;
        for (int x = 319; x > 1; x--) {
            if (gray_img.at<uchar>(y, x) == 255) {
                left = x;
                find = true;
                falg = true;
                failed = 0;
                break;
            }
        }
        if (falg && !find) {
            if (++failed > 10) break;
        }
        left_total.push_back(left);
    }

    // 提取右边界
    falg = false;
    failed = 0;
    for (int y = 269; y >= 50; y--) {
        int right = 639;
        bool find = false;
        for (int x = 319; x < 639; x++) {
            if (gray_img.at<uchar>(y, x) == 255) {
                right = x;
                find = true;
                falg = true;
                failed = 0;
                break;
            }
        }
        if (falg && !find) {
            if (++failed > 10) break;
        }
        right_total.push_back(right);
    }

    // 计算误差
    float row = min(left_total.size(), right_total.size());
    for (int i = 0; i < row; i++) {
        error += (640 - (left_total[i] + right_total[i])) * (1 - i / row);
    }

    // 绘制中线
    int minSize = min(left_total.size(), right_total.size());
    for (int i = 0; i < minSize; i++) {
        int midX = (left_total[i] + right_total[i]) / 2;
        int y = 269 - i;
        midPoints.emplace_back(midX, y);
        circle(visual_img, Point(midX, y), 1, Scalar(0, 255, 255), -1);
    }

    return error / row;
}

// 误差计算
double error_calculater(vector<Point>& traced_points, Mat& visualizeImg) {
    double total_error = 0;
    for (size_t i = 0; i < traced_points.size(); i++) {
        int y = traced_points[i].y;
        double mid_error;
        if (i <= 30.0) {
            mid_error = (traced_points[i].x - (280 - (188 - y) * 1.34) - 320) * (1 - i / 100);
        } else {
            mid_error = (traced_points[i].x - (280 - (188 - y) * 1.34) - 320) * 0.7 * exp(-0.064 * (i - 30.0));
        }
        total_error += mid_error;
    }

    // 绘制轨迹
    for (int i = 0; i < traced_points.size(); i++) {
        int y = traced_points[i].y;
        Point pt(traced_points[i].x - (280 - (188 - y) * 1.34), traced_points[i].y);
        circle(visualizeImg, pt, 3, Scalar(0, 255, 0), -1);
    }

    return traced_points.empty() ? 100.0 : total_error / traced_points.size() * -1;
}

bool line_server_callback(line_follow::line_follow::Request& req,line_follow::line_follow::Response& resp){
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


    // 1. 读取图像并转换为灰度图
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 5);
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
    bool right = true;//判断现在是右边线还是左边线
    int first_point_x_last = 320;//上一帧的赛道起点，发生突变就说明右赛道变成左赛道了
    bool left_forward = true;
    bool point_forward = true;

    out.open(output_file, fourcc, 5, Size(640, 270));
    displayStream << fixed << setprecision(2);

    ros::Time start_time = ros::Time::now();
    ros::Time frame_start = ros::Time::now();
    bool double_line = false;//最后进入两边巡线逻辑
    int point_confirm = 0;//要连续看到那个点20帧才行，不然会超调
            
    int height = 270;
    int width = 640;
    int scan_rows = 180;  // 向上搜索的行数

    bool avoid_done = false;//避障结束后不急着走，先平移恢复一下

    bool out_range = false,start = true;//出圆环判断标志,开始巡线标志，movabese可能导致巡线一开始就压线，先导航到外面，让他自己进来
    bool other_enter = false,pass_out = false,pass_enter = false,out_ready = false,pass_enter_ready = false;//绕环岛期间左巡线
    bool start_other_enter = false;//检测到第一帧另一路口的角点后变为true,如果再返回又巡线逻辑则路口结束
    int out_ready_count = 0,other_enter_count = 0,pass_enter_count = 0;//检测到右线25帧后判定离开，另一入口也要连续判定多帧后才能改变逻辑，避免噪声
    bool left_ready;//判断是否进入圆环需要一个标志位辅助，两边线都看到才算进圆环否则离圆环太远容易出问题
    double position_right_change_left = -1;//右转左的y坐标，用来恢复左转右出圆环
    Point other_enter_last_conner = Point(-1,-1);//另一个入口的角点储存
    while(ros::ok()){
        // ROS_INFO("进入循环");
        // ROS_INFO("耗时:%f",(ros::Time::now()-frame_start).toSec());
        // frame_start = ros::Time::now();
        //----------------------------------避障逻辑----------------------------//
        client_line_board.call(board);
        pose_client.call(pose);
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
        

        //----------------------------------巡线逻辑----------------------------//
        displayStream.str("");//
        cap.read(image);
        if (image.empty()) {
            ROS_INFO("获取图片失败");
            continue;
        }

        Mat cropped = image(roi);
        flip(cropped, cropped, 1);
        Mat gray_img;
        vector<Mat> channels;
        split(cropped, channels);
        gray_img = channels[2];//红色通道代替灰度图
        threshold_image(gray_img);

        //-----------------------------预处理结束开始计算赛道误差
        // ROS_INFO("out%d,pass%d,other%d,enter%d",out_range,pass_out,other_enter,pass_enter);
        // if(!out_range && !pass_out && !other_enter && !pass_enter && !out_ready && !pass_enter_ready){//没有进入左逻辑才要判断需不需要右转左,只有第一圈需要判断
        //     // ROS_INFO("进入if");
        //     pass_out = right_to_left(gray_img,brightness_threshold,left_ready);
        //     position_right_change_left = pose.response.pose_at[1];
        //     if(pass_out) {
        //         ROS_INFO("第一次即将抵达出口%f",position_right_change_left);
        //     }
        // }

        if(!out_range && !pass_out && other_enter && !pass_enter && !out_ready){
            // ROS_INFO("靠近另一个入口");
            other_enter_last_conner = find_other_coner_edge(gray_img,other_enter_last_conner,brightness_threshold,cropped);
            // ROS_INFO("点%d,%d",other_enter_last_conner.x,other_enter_last_conner.y);
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
                    pass_enter = true;//这个的逻辑塞到后面去了
                    // out_ready = true;
                    ROS_INFO("离开另一个路口");
                }
                continue;
            }
        }

        // //-----------回到入口--------//
        // if(pass_enter_ready){
        //     ROS_INFO("回到入口特殊逻辑");
        //     int recent = recently_white(gray_img,brightness_threshold,cropped);
        //     if(recent<150){
        //         twist.linear.x = 0.3;
        //         twist.angular.z = 0;
        //     }
        //     else{
        //         twist.linear.x = 0;
        //         twist.angular.z = 0.6;
        //     }
        //     if(pose.response.pose_at[2]>-2.355 && pose.response.pose_at[2]< -0.7){//准备好出圆环后看到80帧画面，就出圆环
        //         out_range = true;
        //         pass_enter = false;
        //         pass_enter_ready = false;
        //         ROS_INFO("准备离开圆环");
        //     }
        //     displayStream <<"z:"<< twist.angular.z<<"x:  "<<twist.linear.x<<"recent:"<<recent;
        //     string displayText = displayStream.str();
        //     putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
        //     out.write(cropped);
        //     cmd_pub.publish(twist);
        //     continue;
        // }
        
        //---------------------------右巡线逻辑--------------------、、
        Point right_edge_point = Point(-1, -1);//
        int last_scanned_y;
        find_righttrack_edge(gray_img,right_edge_point, scan_rows, brightness_threshold,cropped);
                //-----------回到入口--------//
        if(pass_enter_ready){
            int recent = recently_white(gray_img,brightness_threshold,cropped);
            if(recent<150){
                twist.linear.x = 0.3;
                twist.angular.z = 0;
            }
            else{
                twist.linear.x = 0;
                twist.angular.z = 0.6;
            }
            if(pose.response.pose_at[2]>-2.355 && pose.response.pose_at[2]< -0.7){//准备好出圆环后看到80帧画面，就出圆环
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
            ROS_INFO("入口,白点%d速度z%f速度x%f",recent,twist.angular.z,twist.linear.x);
            continue;
        }
        if ((out_range || pass_out) && right && (first_point_x_last - right_edge_point.x>250) &&pose.response.pose_at[0]>3.0){//如果右边线丢了或者右边界首个点发生剧烈左移动
            right = false;
            ROS_INFO("左跳变%d,%d",first_point_x_last,right_edge_point.x);
        }
        else if(!right && (right_edge_point.x-first_point_x_last>250||right_edge_point.x>500 )){//左线发生剧烈偏移说明又看到右线了左跳右的幅度一般很剧烈|| (right_edge_point.y>170&&right_edge_point.x>300)
            right = true;
            ROS_INFO("右跳变:%d,%d",right_edge_point.x,first_point_x_last);
            if(pass_out){
                pass_out = false;
                other_enter = true;   //已经绕圆环半圈准备离开
                ROS_INFO("到达另一个入口");
            }                
        }
        vector<Point> traced_right,left_edge_points;
        Point left_edge_point;
        // 追踪右侧边线
        bool right_checker = true;//右线不一定真的是右线，可能是太偏的左线，不接受右线向右倾斜，不满足条件切换逻辑
        if (right) {
            double line_error = 0;
            if(!double_line){
                trace_rightedge(right_edge_point, gray_img, traced_right, right_checker, brightness_threshold, &cropped);
                line_error = right_error_calculater(traced_right,right_edge_point.y,cropped);//有调试图片输出
            }
            else{
                line_error = double_find(gray_img,brightness_threshold,cropped);
            }
            if(!right_checker){
                // ROS_INFO("右线斜率出错，舍弃");
                displayStream <<"point: "<< traced_right.size();
                string displayText = displayStream.str();
                putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                
                if(pass_enter){//如果现在是通过路口处
                    pass_enter_count++;//要连续的丢线
                    if(pass_enter_count>3){
                        pass_enter_ready = true;//这个标志用来判断是否已经发生丢线，如果已经发生丢线，再重新看到右线，passenter取消
                        ROS_INFO("回到路口");
                        continue;
                    }
                }
                else{
                    cmd_pub.publish(twist);//
                    out.write(cropped);
                    ROS_INFO("丢线，保持原来运动状态");
                    continue;
                }
                
            }
            else{//
                if (out_range && !left_forward && !point_forward && (ros::Time::now()-start_time).toSec()>1.0){
                    double_line = true;
                    nh.getParam("/line_right/double_P", p);
                    nh.getParam("/line_right/double_I", i);
                    nh.getParam("/line_right/double_D", d);
                    ROS_INFO("p%f",p);
                    ROS_INFO("双边巡线");
                }
                // if(start_other_enter && right_edge_point.x<553){
                //     start_other_enter = false;
                //     other_enter = false;
                //     pass_enter = true;//这个的逻辑塞到后面去了
                //     // out_ready = true;
                //     ROS_INFO("离开另一个路口");
                // }
                point_confirm = 0;
                pass_enter_count = 0;
                left_forward = true;
                point_forward = true;


                // ROS_INFO("右巡线");
                first_point_x_last = right_edge_point.x;
                integration += line_error*0.03;
                integration = std::max(std::min(integration,abs(line_error)/integration_limit+1),-1*abs(line_error)/integration_limit-1);
                double diff = line_error - pre_error;
                diff = std::max(std::min(diff,50.0),-50.0);
                if(avoid_done){
                    twist.linear.x = 0;
                    // twist.linear.y = -1*std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
                    twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
                    ROS_INFO("避障结束右线%f",twist.linear.y);
                    if(line_error<20){
                        avoid_done = false;//视野回到中心，继续前进
                    }
                }
                else{
                    twist.linear.x = x_max / exp(abs(line_error) / 100.0);
                    twist.angular.z = std::max(std::min(line_error*p+integration*i+diff*d,1.0),-1.0);
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
                    // ROS_INFO("左点");
                    if(pass_out){
                        find_left_edge(gray_img, left_edge_point,brightness_threshold,cropped);
                        twist.linear.x = (205-left_edge_point.y)*other_enter_pointx+0.05;
                        twist.angular.z = (553-left_edge_point.x)*other_enter_pointy+0.1;
                        if(left_edge_point.x>450 && left_edge_point.y >180){
                            right = true;//恢复右巡线逻辑
                            other_enter = true;   //已经绕圆环半圈准备离开
                            pass_out = false;
                            ROS_INFO("到达另一个入口");//
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
                        // find_left_edge(gray_img, left_edge_point,brightness_threshold);
                        double error_x = 320-left_edge_point.x;//double error_y = 160-left_edge_point.y;
                        pointx_integration += error_x*0.02;//pointy_integration += error_y*0.02;
                        // ROS_INFO("转折点坐标x%d,y%d",left_edge_point.x,left_edge_point.y);
                        // if(abs(left_edge_point.x-320) < 20 && abs(left_edge_point.y -160)<20){
                        if(abs(left_edge_point.x-320) < 20){
                            avoid_done = false;
                            point_confirm++;
                            if(point_confirm>7){
                                pointx_integration = 0;
                                // pointy_integration = 0;
                                point_forward = false;
                            }
                        }
                        pointx_integration = std::max(std::min(pointx_integration,1.0),-1.0);//pointy_integration = std::max(std::min(pointy_integration,1.0),-1.0);
                        // ROS_INFO("P%f,I%f",error_y/400,pointy_integration/400);
                        twist.linear.x = std::max(twist.linear.x-0.2,0.0);
                        twist.angular.z = std::max(std::min(error_x*leftpoint_p + pointx_integration * leftpoint_I,0.5),-0.5);

                        pointx_pre_error = error_x;//pointy_pre_error = error_y;
                        displayStream <<"z:  "<< twist.angular.z<<"errorx:  "<<error_x<<"pointx_integration:"<<pointx_integration;
                        string displayText = displayStream.str();
                        putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                        out.write(cropped);
                    }
                }
                else{
                    // ROS_INFO("左线");
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
            if(stop_car(gray_img,brightness_threshold,test,cropped)){//
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