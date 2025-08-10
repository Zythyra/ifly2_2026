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
VideoWriter out_;
int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D'); // MP4V编码
ostringstream displayStream;

// 声明赛道结构体（关键修复：在类外部或内部提前声明）
struct RaceTrack {
    double slope;                  // 赛道斜率
    vector<Point> points;          // 赛道点集
    int direction_change;          // 方向变化次数
    int slope_change_count;        // 斜率变化次数
    bool left_point;               // 是否为左赛道标志
};

template <typename T>
T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

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
        int lastx = 0,last_lastx = 0;//用来判断斜率变化
        int count = 0;//每隔五个点判断一次方向
        while (center_y > start.y - 100) {
            bool left_found = false, right_found = false;
            for (int dx = 0; dx <= search_range/2; dx++) {
                int cand_x = center_x + dx;
                int cand_x2 = center_x - dx;
                bool left_check = (cand_x2 > 1);
                bool right_check = (cand_x < width - 1);

                if (left_check && gray_img.at<uchar>(center_y, cand_x2) == 255 && gray_img.at<uchar>(center_y, cand_x2 - 1) == 0) {
                    racetracks[idx].points.emplace_back(cand_x2, center_y);
                    circle(visual_img, Point((cand_x2, center_y)), 2, Scalar(255, 255, 0), -1);
                    left_found = true;
                    count++;
                    center_x = cand_x2;
                }
                if (!left_found && right_check && gray_img.at<uchar>(center_y, cand_x) == 0 && gray_img.at<uchar>(center_y, cand_x + 1) == 255) {
                    right_found = true;
                    racetracks[idx].points.emplace_back(cand_x+1, center_y);
                    count++;
                    center_x = cand_x + 1;
                }
            }

            if(count>5){
                if((racetracks[idx].points.back().x - lastx)*(lastx-last_lastx)*last_lastx<0){//意思就是说看x坐标的变化，先增后减差积为负,多*一个零保证初始不会算错一个数字
                    racetracks[idx].direction_change++;
                }
                last_lastx = lastx;lastx = racetracks[idx].points.back().x;
                count = 0;//隔5看1
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
            // float vx = lineParams[0];
            // float vy = lineParams[1];
            // float x0 = lineParams[2];
            // float y0 = lineParams[3];
            // Point pt1, pt2;
            // double scale = 1000;  // 取足够大的缩放因子覆盖整个图像
            
            // pt1.x = cvRound(x0 - vx * scale);
            // pt1.y = cvRound(y0 - vy * scale);
            // pt2.x = cvRound(x0 + vx * scale);
            // pt2.y = cvRound(y0 + vy * scale);

            // // 裁剪直线到图像边界内
            // Rect imageRect(0, 0, visual_img.cols, visual_img.rows);
            // if (clipLine(imageRect, pt1, pt2)) {
            //     cv::line(visual_img, pt1, pt2, Scalar(255, 0, 0), 2);  // 绿色，线宽2
            // }
            racetracks[idx].slope = lineParams[1] / lineParams[0];
        } else {
            racetracks[idx].slope = -2.0;
        }
    }

    // 选择最优赛道
    int best_idx = -1;
    float min_dangerous = 2.1;
    for (int i = 0; i < point_number; i++) {
        if (!(racetracks[i].slope < 0.05 && racetracks[i].slope > -20)) {
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
        oss << "slope: " << racetrack.slope << " direction_change: " << racetrack.direction_change;
        putText(visual_img, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
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
                }
                if (!found && up_check && gray_img.at<uchar>(center_y - dy, center_x) == 255 && gray_img.at<uchar>(center_y - dy + 1, center_x) == 0) {
                    racetracks[idx].points.emplace_back(center_x + 1, center_y - dy);
                    found = true;
                    center_y -= dy;
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
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        return true;
    }
    return false;
}

bool find_other_coner_edge(Mat gray_img,Point& left_edge_point,Mat& visualizeImg){//拐角的特征是这一行有下一行没
    bool is_now_white = false;
    vector<Point> maybe_left_point;

    // 左部寻找起点
    for (int i = 638; i > 2; i--) {
        if (is_now_white && gray_img.at<uchar>(75, i) == 0) {
            is_now_white = false;
        }
        if (!is_now_white && gray_img.at<uchar>(75, i) == 255) {
            maybe_left_point.emplace_back(i, 75);
            // ROS_INFO("起点%d,%d",i,75);
            circle(visualizeImg, Point(i, 75), 9, Scalar(255, 255, 125), -1);
            is_now_white = true;
        }
    }
    
    int point_number = maybe_left_point.size();
    vector<RaceTrack> racetracks(point_number);  // 现在可正常声明
    int search_range = 40;

    // 追踪左边缘
    for (int idx = 0; idx < point_number; idx++) {
        bool broken = false, last_left = true, last_right = false;
        int fail_count = 0;
        Point start = maybe_left_point[idx];
        int center_x = start.x, center_y = start.y - 1;
        int lastx = 0,last_lastx = 0;//用来判断斜率变化
        int count = 0;//每隔五个点判断一次方向
        while (center_y < 269) {
            bool left_found = false, right_found = false;
            for (int dx = 0; dx <= search_range/2; dx++) {
                int cand_x = center_x + dx;
                int cand_x2 = center_x - dx;
                bool left_check = (cand_x2 > 1);
                bool right_check = (cand_x < 638);

                if (left_check && gray_img.at<uchar>(center_y, cand_x2) == 255 && gray_img.at<uchar>(center_y, cand_x2 + 1) == 0) {
                    racetracks[idx].points.emplace_back(cand_x2, center_y);
                    count++;
                    center_x = cand_x2;
                    left_found = true;
                    break;
                }
                if (right_check && gray_img.at<uchar>(center_y, cand_x) == 0 && gray_img.at<uchar>(center_y, cand_x - 1) == 255) {
                    racetracks[idx].points.emplace_back(cand_x - 1, center_y);
                    count++;
                    center_x = cand_x - 1;
                    right_found = true;
                    break;
                }
            }
            if(count>5){
                if((racetracks[idx].points.back().x - lastx)*(lastx-last_lastx)*last_lastx<0){//意思就是说看x坐标的变化，先增后减差积为负,多*一个零保证初始不会算错一个数字
                    racetracks[idx].direction_change++;
                }
                last_lastx = lastx;lastx = racetracks[idx].points.back().x;
                count = 0;//隔5看1
            }
            // ROS_INFO("处理完成左%d右%d",last_left,last_right);
            // 处理追踪结果
            // ROS_INFO("失败点数%d，y的值%d",fail_count,center_y);
            if (left_found || right_found) {
                fail_count = 0;
                center_y++;
            } else {
                fail_count++;
                center_y++;
                if (fail_count >= 4) { broken = true; break; }
            }
            if (center_y >268) break;
        }
        // ROS_INFO("找到%d个起始点，有%zu个延伸,转折点有%d个",point_number,racetracks[idx].points.size(),racetracks[idx].direction_change);
    }

    // 选择最优赛道
    int best_idx = -1,first_index = -1,second_index = -1;
    float first = 2.1,second = 1.8;//先找出最平滑的两条赛道，然后筛选出靠左的一个
    for (int i = 0; i < point_number; i++) {
        float direction_change_in_total = racetracks[i].direction_change/(float)racetracks[i].points.size();
        if (direction_change_in_total<first) {
            first = direction_change_in_total;
            first_index = i;
        }
        else if(direction_change_in_total<second){
            second = direction_change_in_total;
            second_index = i;
        }
    }

    if(second>0.1){
        best_idx = first_index;//首先乱跳的点不能太多
    }
    else{
        if(racetracks[first_index].points[0].x<racetracks[second_index].points[0].x){
            best_idx = first_index;
        }
        else{
            best_idx = second_index;//都差不多的话选最左边那个
        }
    }
    if (best_idx != -1) {
        Point test = racetracks[best_idx].points[racetracks[best_idx].points.size()-1];
        ROS_INFO("有待选点%d,%d",test.x,test.y);
        if(test.x>500 || test.x<20 || test.y>250 || test.y <60){
            return false;//刚开始点不会跑到边上去
        }
        left_edge_point = test;
        circle(visualizeImg, left_edge_point,9, Scalar(0, 0, 255), -1);
        for (const auto& p : racetracks[best_idx].points) {
            circle(visualizeImg, p, 2, Scalar(255, 0, 0), -1);
        }
        ostringstream oss;
        oss << "direction_change " << racetracks[best_idx].direction_change;
        putText(visualizeImg, oss.str(), Point(50, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        return true;
    }
    return false;
}
bool find_other_coner_edge2(Mat gray_img,Point& left_edge_point,Mat& visualizeImg){//拐角的特征是这一行有下一行没
    bool is_now_white = false;
    vector<Point> maybe_left_point;

    // 左部寻找起点
    for (int i = 269; i > 2; i--) {
        if (is_now_white && gray_img.at<uchar>(i, 270) == 0) {
            is_now_white = false;
        }
        if (!is_now_white && gray_img.at<uchar>(i, 270) == 255) {
            maybe_left_point.emplace_back(270, i);
            circle(visualizeImg, Point(270, i), 9, Scalar(255, 0, 0), -1);
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
        int count = 0, lastx = 0,last_lastx = 0;//用来判断斜率变化;
        while (center_x < 620) {
            bool found = false;
            for (int dy = 0; dy <= search_range/2; dy++) {
                bool up_check = (center_y - dy > 2);
                bool down_check = (center_y + dy < 268);

                if (down_check && gray_img.at<uchar>(center_y + dy, center_x) == 255 && gray_img.at<uchar>(center_y + dy - 1, center_x) == 0) {
                    racetracks[idx].points.emplace_back(center_x, center_y + dy);
                    found = true;
                    center_y += dy;
                    count++;
                    break;
                }
                if (!found && up_check && gray_img.at<uchar>(center_y - dy, center_x) == 255 && gray_img.at<uchar>(center_y - dy - 1, center_x) == 0) {
                    racetracks[idx].points.emplace_back(center_x, center_y - dy);
                    found = true;
                    center_y -= dy;
                    count++;
                    break;
                }
            }
            if(count>5){
                if((racetracks[idx].points.back().x - lastx)*(lastx-last_lastx)*last_lastx<0){//意思就是说看x坐标的变化，先增后减差积为负,多*一个零保证初始不会算错一个数字
                    racetracks[idx].direction_change++;
                }
                last_lastx = lastx;lastx = racetracks[idx].points.back().x;
                count = 0;//隔5看1
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
        if (racetracks[idx].points.size() > 60) racetracks[idx].left_point = true;
    }

    // 选择最优左边缘
    int best_idx = -1;
    float first = 2.1;//先找出最平滑的两条赛道，然后筛选出靠左的一个
    for (int i = 0; i < point_number; i++) {
        if(racetracks[i].left_point){
            float direction_change_in_total = racetracks[i].direction_change/(float)racetracks[i].points.size();
            if (direction_change_in_total<first) {
                first = direction_change_in_total;
                best_idx = i;
            }
        }
    }

    if (best_idx != -1) {
        int lowest_pointy = racetracks[best_idx].points[0].y;
        for (const auto& p : racetracks[best_idx].points) {
            circle(visualizeImg, p, 2, Scalar(255, 0, 0), -1);
            if(p.y>=lowest_pointy){
                lowest_pointy = p.y;
                left_edge_point = p;
            }
        }
        ostringstream oss;
        oss << "direction_change " << racetracks[best_idx].direction_change;
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        circle(visualizeImg, left_edge_point, 9, Scalar(0, 0, 255), -1);
        return true;
    }
    return false;
}

int recently_white(Mat gray_img,Mat& visualizeImg){//回到路口的时候，只能看到正前方有白线，通过白线距离来判断怎么走
    bool is_now_white = false;
    vector<Point> maybe_left_point;

    // 左部寻找起点
    for (int i = 269; i > 2; i--) {
        if (is_now_white && gray_img.at<uchar>(i, 320) == 0) {
            is_now_white = false;
        }
        if (!is_now_white && gray_img.at<uchar>(i, 320) == 255) {
            maybe_left_point.emplace_back(320, i);
            circle(visualizeImg, Point(320, i), 9, Scalar(255, 0, 0), -1);
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
                    circle(visualizeImg, Point((center_x, center_y + dy)), 2, Scalar(255, 0, 0), -1);
                    found = true;
                    center_y += dy;
                }
                if (!found && up_check && gray_img.at<uchar>(center_y - dy, center_x) == 255 && gray_img.at<uchar>(center_y - dy + 1, center_x) == 0) {
                    racetracks[idx].points.emplace_back(center_x + 1, center_y - dy);
                    circle(visualizeImg, Point((center_x + 1, center_y - dy)), 2, Scalar(255, 0, 0), -1);
                    found = true;
                    center_y -= dy;
                }
            }

            if (found) {
                fail_count = 0;
                center_x++;
            } else {
                fail_count++;
                center_x++;
                if (fail_count >= 6) { broken = true; break; }
            }
        }
        // if (racetracks[idx].points.size() > 100) racetracks[idx].left_point = true;
    }

    // 选择最优左边缘
    int best_idx = -1;
    size_t max_point_number = 1;
    for (int i = 0; i < point_number; i++) {
        if (racetracks[i].points.size() > max_point_number) {
            max_point_number = racetracks[i].points.size();
            best_idx = i;
        }
    }

    if (best_idx != -1) {
        RaceTrack racetrack = racetracks[best_idx];  // 现在可正常使用
        for (size_t i = 0; i < racetrack.points.size(); i += 3) {
            circle(visualizeImg, racetrack.points[i], 2, Scalar(0, 0, 255), -1);
        }
        ostringstream oss;
        oss << "前点: (" << racetrack.points[0].x << "," << racetrack.points[0].y << ")";
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        return racetrack.points[0].y;
    }
    return -1;
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


class LineFollowerNode {
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    geometry_msgs::Twist twist;

    ros::ServiceClient client_line_board;
    ztestnav2025::lidar_process board;
    ros::ServiceClient pose_client;
    ztestnav2025::getpose_server pose;

    ros::ServiceServer line_server;

    double p_,i_,d_,integration_,pre_error_,leftpoint_p_,leftpoint_i_,leftpoint_D_,x_max_,other_enter_pointy_,other_enter_pointx_,integration_limit_,out_turn_,out_forward_;
    double point_2_speeda,point_2_speedk,out_turn_angel_;

public:
    LineFollowerNode() : nh(""){
        line_server = nh.advertiseService("right2", &LineFollowerNode::line_server_callback,this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        client_line_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
        pose_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
        board.request.lidar_process_start = -2;
        pose.request.getpose_start = 1;
        ROS_INFO("等待lidar_process服务中---");
        client_line_board.waitForExistence();
        ROS_INFO("等待坐标获取服务中---");
        pose_client.waitForExistence();
        nh.getParam("/right2/right_P", p_);
        nh.getParam("/right2/right_I", i_);
        nh.getParam("/right2/right_D", d_);
        nh.getParam("/right2/leftpoint_p", leftpoint_p_);
        nh.getParam("/right2/leftpoint_I", leftpoint_i_);
        nh.getParam("/right2/leftpoint_D", leftpoint_D_);
        nh.getParam("/right2/out_turn", out_turn_);
        nh.getParam("/right2/out_forward", out_forward_);
        nh.getParam("/right2/out_turn_angel", out_turn_angel_);
        nh.getParam("/right2/x_max_", x_max_);
        nh.getParam("/right2/other_enter_pointy", other_enter_pointy_);
        nh.getParam("/right2/other_enter_pointx", other_enter_pointx_);
        nh.getParam("/right2/integration_limit", integration_limit_);
        nh.getParam("/right2/point_2_speeda", point_2_speeda);
        nh.getParam("/right2/point_2_speedk", point_2_speedk);
        ROS_INFO("参数加载P: %f", x_max_);
    }

        
    bool line_server_callback(line_follow::line_follow::Request& req,line_follow::line_follow::Response& resp){
        // ros::ServiceClient reconfigure_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
        // reconfigure_client.waitForExistence();
        // dynamic_reconfigure::ReconfigureRequest request;
        // dynamic_reconfigure::ReconfigureResponse response;
        // dynamic_reconfigure::DoubleParameter planner_frequency;
        // planner_frequency.name = "planner_frequency";
        // planner_frequency.value = 0.0;
        // request.config.doubles.push_back(planner_frequency);
        // if (reconfigure_client.call(request, response)) {
        //     ROS_INFO("参数更新成功");
        //     double new_value;
        //     if (ros::param::get("/move_base/planner_frequency", new_value)) {
        //         ROS_INFO("Current planner_frequency: %.2f", new_value);
        //     }
        // } else {
        //     ROS_ERROR("参数更新失败");
        // }


        // ROS_INFO("tf变换");
        // tf::TransformListener* tf_listener_;
        // tf_listener_ = new tf::TransformListener();


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
        
        integration_ = 0;
        pre_error_ = 0;
        double pointx_integration = 0;
        double pointx_pre_error = 0;
        double pointy_integration = 0;
        double pointy_pre_error = 0;

        out_.open(output_file, fourcc, 5, Size(640, 270));
        displayStream << fixed << setprecision(2);

        ros::Time start_time = ros::Time::now();
        ros::Time frame_start = ros::Time::now();
        bool double_line = false;//最后进入两边巡线逻辑
        int point_confirm = 0;//要连续看到那个点20帧才行，不然会超调

        bool avoid_done = false;//避障结束后不急着走，先平移恢复一下

        bool out_range = false,start = true;//出圆环判断标志,开始巡线标志，movabese可能导致巡线一开始就压线，先导航到外面，让他自己进来
        bool other_enter = false,pass_out = true,pass_enter = false,out_ready = false,pass_enter_ready = false;//绕环岛期间左巡线
        int out_ready_count = 0,other_enter_count = 0,pass_enter_count = 0;//检测到右线25帧后判定离开，另一入口也要连续判定多帧后才能改变逻辑，避免噪声
        Point other_enter_last_conner = Point(-1,-1);//另一个入口的角点储存
        //-----------从开始到离开出口阶段需要的标志位-----------//
        bool left_point_start_ = false;
        int trace_failed_count_ = 0;
        //--------------从离开出口到错误出口的标志位----------//
        bool pass_out_done = false;
        int right_trace = 0;

        while(ros::ok()){
            // ROS_INFO("进入循环");
            // ROS_INFO("耗时:%f",(ros::Time::now()-frame_start).toSec());
            // frame_start = ros::Time::now();
            //----------------------------------避障逻辑----------------------------//
            client_line_board.call(board);
            pose_client.call(pose);
            if(!avoid_done){
                if(board.response.lidar_results[0] != -1){
                    if(board.response.lidar_results[0]>0.41){//如果还比较远先减速
                        x_max_ = 0.22;
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
                        // nh.getParam("/line_right/x_max_", x_max_);
                        x_max_ = 0.6;
                        double_line = true;
                        nh.getParam("/line_right/double_P", p_);
                        nh.getParam("/line_right/double_I", i_);
                        nh.getParam("/line_right/double_D", d_);
                        ROS_INFO("p %f",p_);
                        ROS_INFO("切换为双边巡线");
                    }
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
            if(pass_out){//第一次通过出口，忽略并离开
                vector<Point> start_points = find_track_edge(gray_img, 300, 130, cropped);
                RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用
                if(left_point_start_){
                    Point left_edge_point;
                    find_left_edge(gray_img, left_edge_point,cropped);
                    // twist.linear.x = (205-left_edge_point.y)*other_enter_pointx_+0.1;
                    // twist.angular.z = (553-left_edge_point.x)*other_enter_pointy_+0.2;
                    // twist.linear.x = x_max_;
                    twist.linear.x = 0.3;
                    twist.angular.z = twist.linear.x*(point_2_speeda-left_edge_point.x)/point_2_speedk;
                    // ROS_INFO("增益是%f",(point_2_speeda-left_edge_point.x)/point_2_speedk);
                    if(left_edge_point.x>450 || left_edge_point.y >180){
                        pass_out_done = true;
                        left_point_start_ = false;
                        ROS_INFO("到达另一个入口");//
                    }
                    displayStream <<"x:"<< twist.linear.x<<"z:"<< twist.angular.z<<"erx:"<<205-left_edge_point.y<<"ery:"<<553-left_edge_point.x;
                    string displayText = displayStream.str();
                    putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                }
                else if (trace_edge(gray_img, start_points, racetrack, cropped)) {
                    // 成功追踪到赛道
                    trace_failed_count_ = 0;
                    double line_error = error_calculater(racetrack.points, cropped);
                    
                    // PID计算
                    integration_ += line_error * 0.03;
                    integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
                    double diff = line_error - pre_error_;
                    diff = clamp(diff, -50.0, 50.0);
                    
                    // 速度控制
                    twist.linear.x = x_max_ / exp(abs(line_error) / 100.0);
                    // twist.linear.x = 0.35;
                    twist.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
                    pre_error_ = line_error;

                    // 显示信息
                    displayStream << "error: " << line_error << " P: " << line_error*p_ << " I: " << integration_*i_ << " D: " << diff*d_ << " twistz: " << twist.angular.z;
                    putText(cropped, displayStream.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                    if(pass_out_done){
                        right_trace++;
                        if(right_trace>10){
                            other_enter = true;   //已经绕圆环半圈准备离开
                            pass_out = false;
                            ROS_INFO("到达另一个入口");//
                        }
                    }
                } else {
                    // 追踪失败计数
                    trace_failed_count_++;
                    if (trace_failed_count_ > 5 && !pass_out_done) {
                        left_point_start_ = true;
                        ROS_INFO("连续追踪失败，切换至左点追踪");
                    }
                }
                out_.write(cropped);
            }

            if(other_enter){//到达另一个路口，这次右边不会断线，拐弯要强制执行
                bool find_coner = false;
                if(other_enter_last_conner.x<320){
                    find_coner = find_other_coner_edge(gray_img,other_enter_last_conner,cropped);//发现拐点，强制执行
                    // ROS_INFO("逻辑1,x%d,y%d,find%d",other_enter_last_conner.x,other_enter_last_conner.y,find_coner);
                }
                else{
                    find_coner = find_other_coner_edge2(gray_img,other_enter_last_conner,cropped);//发现拐点，强制执行
                    // ROS_INFO("逻辑2,x%d,y%d,find%d",other_enter_last_conner.x,other_enter_last_conner.y,find_coner);
                }
                // if(find_coner && other_enter_last_conner.y>100){
                if(find_coner){
                    twist.linear.x = 0.3;
                    twist.angular.z = twist.linear.x*(point_2_speeda-other_enter_last_conner.x)/point_2_speedk/1.5;
                    displayStream <<"x:"<< twist.linear.x<<"z:"<< twist.angular.z<<"positionx"<<other_enter_last_conner.x<<"posiony"<<other_enter_last_conner.y;
                    string displayText = displayStream.str();
                    putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                    out_.write(cropped);
                    if(other_enter_last_conner.y>200){
                        other_enter = false;
                        pass_enter = true;//这个的逻辑塞到后面去了
                        // out_ready = true;
                        ROS_INFO("离开另一个路口");
                    }
                }
                else {
                    vector<Point> start_points = find_track_edge(gray_img, 300, 70, cropped);
                    RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用
                    if(trace_edge(gray_img, start_points, racetrack, cropped)){
                        double line_error = error_calculater(racetrack.points, cropped);
                        // PID计算
                        integration_ += line_error * 0.03;
                        integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
                        double diff = line_error - pre_error_;
                        diff = clamp(diff, -50.0, 50.0);
                        
                        // 速度控制
                        twist.linear.x = x_max_ / exp(abs(line_error) / 100.0);
                        twist.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
                        pre_error_ = line_error;

                        // 显示信息
                        displayStream << "error: " << line_error << " P: " << line_error*p_ << " I: " << integration_*i_ << " D: " << diff*d_ << " twistz: " << twist.angular.z;
                        putText(cropped, displayStream.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5,Scalar(255, 255, 0), 1);
                    }
                    out_.write(cropped);
                }
            }

            if(pass_enter){//此处赛道和底部不是连续的，特殊处理
                vector<Point> maybe_start_point = find_track_edge(gray_img, 300, 123, cropped);
                RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用
                if(trace_edge(gray_img, maybe_start_point, racetrack, cropped)){
                    double line_error = error_calculater(racetrack.points, cropped);
                    // PID计算
                    integration_ += line_error * 0.03;
                    integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
                    double diff = line_error - pre_error_;
                    diff = clamp(diff, -50.0, 50.0);
                    // 速度控制
                    twist.linear.x = x_max_ / exp(abs(line_error) / 100.0);
                    twist.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
                    pre_error_ = line_error;
                    // 显示信息
                    displayStream << "error: " << line_error << " P: " << line_error*p_ << " I: " << integration_*i_ << " D: " << diff*d_ << " twistz: " << twist.angular.z;
                    putText(cropped, displayStream.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                }
                else{
                    pass_enter_count++;//要连续的丢线
                    if(pass_enter_count>3){
                        ROS_INFO("回到路口");
                        int recent = recently_white(gray_img,cropped);
                        if(recent<170){
                            twist.linear.x = 0.4;
                            twist.angular.z = 0.4;
                        }
                        else{
                            twist.linear.x = 0;
                            twist.angular.z = 1.5;
                        }
                        if(pose.response.pose_at[2]>-2.355 && pose.response.pose_at[2]< -0.7){//准备好出圆环后看到80帧画面，就出圆环
                            out_range = true;
                            pass_enter = false;
                            trace_failed_count_ = 0;//这两个标志位前面用过了，复原一下
                            left_point_start_ = false;
                            ROS_INFO("准备离开圆环");
                        }
                        displayStream <<"z:"<< twist.angular.z<<"x:  "<<twist.linear.x<<"recent:"<<recent;
                        string displayText = displayStream.str();
                        putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                        out_.write(cropped);
                    }
                }
                out_.write(cropped);
            }

            if(out_range){
                vector<Point> start_points = find_track_edge(gray_img, 300, 70, cropped);
                RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用
                if(left_point_start_){
                    Point left_edge_point;
                    if(find_left_edge(gray_img, left_edge_point,cropped)){
                        double error_x = 320 - left_edge_point.x;
                        pointx_integration += error_x * 0.02;
                        pointx_integration = clamp(pointx_integration, -1.0, 1.0);
                        
                        // 左点过低时停止前进
                        if (left_edge_point.y > 240) {
                            ROS_INFO("丢线旋转中");
                            twist.linear.x = out_forward_;
                            twist.angular.z = out_turn_;
                            while(ros::ok){
                                // 旋转到位后切换模式
                                pose_client.call(pose);
                                ROS_INFO("位置%f,角度%f",pose.response.pose_at[2],out_turn_angel_);
                                if (pose.response.pose_at[2] < out_turn_angel_) {
                                    left_point_start_ = false;
                                    double_line = true;
                                    out_range = false;
                                    x_max_ = 0.5;
                                    nh.getParam("/line_right/double_P", p_);
                                    nh.getParam("/line_right/double_I", i_);
                                    nh.getParam("/line_right/double_D", d_);
                                    ROS_INFO("旋转完成，切换双边巡线 (P=%.2f)", p_);
                                    break;
                                }
                                cmd_pub.publish(twist);
                            }
                        }

                        // PID计算
                        double point_diff = error_x - pointx_pre_error;
                        twist.linear.x = 0.23;
                        twist.angular.z = error_x*leftpoint_p_ + pointx_integration*leftpoint_i_ + point_diff*leftpoint_D_;
                        pointx_pre_error = error_x;

                        // 显示信息
                        displayStream << "leftxerror: " << error_x << " P: " << error_x*leftpoint_p_ << " I: " << pointx_integration*leftpoint_i_;
                        putText(cropped, displayStream.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                    }
                    out_.write(cropped);
                }
                else if (trace_edge(gray_img, start_points, racetrack, cropped)) {
                    // 成功追踪到赛道
                    trace_failed_count_ = 0;
                    double line_error = error_calculater(racetrack.points, cropped);
                    
                    // PID计算
                    integration_ += line_error * 0.03;
                    integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
                    double diff = line_error - pre_error_;
                    diff = clamp(diff, -50.0, 50.0);
                    
                    // 速度控制
                    twist.linear.x = x_max_ / exp(abs(line_error) / 100.0);
                    twist.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
                    pre_error_ = line_error;

                    // 显示信息
                    displayStream << "error: " << line_error << " P: " << line_error*p_ << " I: " << integration_*i_ << " D: " << diff*d_ << " vz: " << twist.angular.z;
                    putText(cropped, displayStream.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5,Scalar(255, 255, 0), 1);
                } else {
                    // 追踪失败计数
                    trace_failed_count_++;
                    if (trace_failed_count_ > 3) {
                        left_point_start_ = true;
                        ROS_INFO("连续追踪失败，切换至左点追踪");
                    }
                }
                out_.write(cropped);
            }

            if(double_line){
                double line_error = double_find(gray_img, cropped);
            
                // PID计算
                integration_ += line_error * 0.03;
                integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
                double diff = line_error - pre_error_;
                diff = clamp(diff, -50.0, 50.0);
                
                // 速度控制
                twist.linear.x = x_max_ / exp(abs(line_error) / 100.0);
                twist.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
                pre_error_ = line_error;

                // 显示信息
                displayStream << "doubleerror: " << line_error << " P: " << line_error*p_ << " I: " << integration_*i_ << " D: " << diff*d_ << " vz: " << twist.angular.z;
                putText(cropped, displayStream.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
                out_.write(cropped);
            }

            int test;
            if(avoid_done){
                if(stop_car(gray_img,test,cropped)){//
                    ROS_INFO("巡线结束");
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    cmd_pub.publish(twist);
                    // imshow("stop",cropped);
                    // waitKey(0);
                    break;
                }
            }
            // imshow("test",cropped);
            // imshow("origin",gray_img);
            // waitKey(1);
            cmd_pub.publish(twist);
        }
        cap.release();
        out_.release();
        return true;
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "right2");
    LineFollowerNode linefollowernode;

    ROS_INFO("视觉巡线初始化");
    if(ros::ok()) ros::spin();
    ROS_INFO("结束");
    return 0;
}
