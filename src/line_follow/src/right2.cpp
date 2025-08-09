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
string output_file = "/home/ucar/ucar_ws/src/line_follow/image/right2.avi";//录制视频避免网络传输卡顿
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
            ROS_INFO("起点%d,%d",i,75);
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

        while (center_y < 250) {
            bool left_found = false, right_found = false;
            for (int dx = 0; dx <= search_range/2; dx++) {
                int cand_x = center_x + dx;
                int cand_x2 = center_x - dx;
                bool left_check = (cand_x2 > 1);
                bool right_check = (cand_x < 638);

                if (left_check && gray_img.at<uchar>(center_y, cand_x2) == 255 && gray_img.at<uchar>(center_y, cand_x2 + 1) == 0) {
                    racetracks[idx].points.emplace_back(cand_x2, center_y);
                    left_found = true;
                    center_x = cand_x2;
                }
                if (!left_found && right_check && gray_img.at<uchar>(center_y, cand_x) == 0 && gray_img.at<uchar>(center_y, cand_x - 1) == 255) {
                    racetracks[idx].points.emplace_back(cand_x - 1, center_y);
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
                center_y++;
            } else {
                fail_count++;
                center_y++;
                if (fail_count >= 4) { broken = true; break; }
            }
            if (center_y >268) break;
        }
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
        left_edge_point = racetracks[best_idx].points[racetracks[best_idx].points.size()-1];
        if(left_edge_point.x>500){
            return false;//刚开始点不会跑到右边去
        }
        circle(visualizeImg, left_edge_point,9, Scalar(0, 0, 255), -1);
        for (const auto& p : racetracks[best_idx].points) {
            circle(visualizeImg, p, 2, Scalar(255, 0, 0), -1);
        }
        ostringstream oss;
        oss << "direction_change " << racetracks[best_idx].direction_change;
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
    float first = 2.1;//先找出最平滑的两条赛道，然后筛选出靠左的一个
    for (int i = 0; i < point_number; i++) {
        float direction_change_in_total = racetracks[i].direction_change/(float)racetracks[i].points.size();
        if (direction_change_in_total<first) {
            first = direction_change_in_total;
            best_idx = i;
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
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
            maybe_left_point.emplace_back(5, i);
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
                if (fail_count >= 6) { broken = true; break; }
            }
        }
        if (racetracks[idx].points.size() > 100) racetracks[idx].left_point = true;
    }

    // 选择最优左边缘
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
        RaceTrack racetrack = racetracks[best_idx];  // 现在可正常使用
        for (size_t i = 0; i < racetrack.points.size(); i += 3) {
            circle(visualizeImg, racetrack.points[i], 2, Scalar(0, 0, 255), -1);
        }
        ostringstream oss;
        oss << "前点: (" << racetrack.points[0].x << "," << racetrack.points[0].y << ")";
        putText(visualizeImg, oss.str(), Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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

    double p_,i_,d_,integration_,pre_error_,leftpoint_p_,leftpoint_i_,leftpoint_D_,x_max_,other_enter_pointy_,other_enter_pointx_,integration_limit_,out_turn_,out_forward_;
    nh.getParam("/right2/right_P", p_);
    nh.getParam("/right2/right_I", i_);
    nh.getParam("/right2/right_D", d_);
    nh.getParam("/right2/leftpoint_p", leftpoint_p_);
    nh.getParam("/right2/leftpoint_I", leftpoint_i_);
    nh.getParam("/right2/leftpoint_D", leftpoint_D_);
    nh.getParam("/right2/out_turn", out_turn_);
    nh.getParam("/right2/out_forward", out_forward_);
    nh.getParam("/right2/x_max_", x_max_);
    nh.getParam("/right2/other_enter_pointy", other_enter_pointy_);
    nh.getParam("/right2/other_enter_pointx", other_enter_pointx_);
    nh.getParam("/right2/integration_limit", integration_limit_);
    ROS_INFO("参数加载P: %f", x_max_);
    integration_ = 0;
    pre_error_ = 0;
    double pointx_integration = 0;
    double pointx_pre_error = 0;
    double pointy_integration = 0;
    double pointy_pre_error = 0;
    bool right = true;//判断现在是右边线还是左边线
    int first_point_x_last = 320;//上一帧的赛道起点，发生突变就说明右赛道变成左赛道了
    bool left_forward = true;
    bool point_forward = true;

    out_.open(output_file, fourcc, 5, Size(640, 270));
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
    bool other_enter = false,pass_out = true,pass_enter = false,out_ready = false,pass_enter_ready = false;//绕环岛期间左巡线
    bool start_other_enter = false;//检测到第一帧另一路口的角点后变为true,如果再返回又巡线逻辑则路口结束
    int out_ready_count = 0,other_enter_count = 0,pass_enter_count = 0;//检测到右线25帧后判定离开，另一入口也要连续判定多帧后才能改变逻辑，避免噪声
    bool left_ready;//判断是否进入圆环需要一个标志位辅助，两边线都看到才算进圆环否则离圆环太远容易出问题
    double position_right_change_left = -1;//右转左的y坐标，用来恢复左转右出圆环
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
                nh.getParam("/line_right/x_max_", x_max_);
                double_line = true;
                nh.getParam("/line_right/double_P", p_);
                nh.getParam("/line_right/double_I", i_);
                nh.getParam("/line_right/double_D", d_);
                ROS_INFO("p %f",p_);
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
        // Point other_enter_point;
        // find_other_coner_edge(gray_img,other_enter_point,cropped);
        if(pass_out){//第一次通过出口，忽略并离开
            vector<Point> start_points = find_track_edge(gray_img, 300, 70, cropped);
            RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用
            if(left_point_start_){
                Point left_edge_point;
                find_left_edge(gray_img, left_edge_point,cropped);
                // twist.linear.x = (205-left_edge_point.y)*other_enter_pointx_+0.1;
                // twist.angular.z = (553-left_edge_point.x)*other_enter_pointy_+0.2;
                twist.linear.x = 0.3;
                twist.angular.z = twist.linear.x*(553-left_edge_point.x)/(205-left_edge_point.y);
                if(left_edge_point.x>450 || left_edge_point.y >180){
                    pass_out_done = true;
                    ROS_INFO("到达另一个入口");//
                }
                displayStream <<"x:"<< twist.linear.x<<"z:"<< twist.angular.z<<"erx:"<<205-left_edge_point.y<<"ery:"<<553-left_edge_point.x;
                string displayText = displayStream.str();
                putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
                displayStream << "正常误差: " << line_error 
                            << " P: " << line_error*p_ 
                            << " I: " << integration_*i_ 
                            << " D: " << diff*d_ 
                            << " 角速度: " << twist.angular.z;
                putText(cropped, displayStream.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
                if (trace_failed_count_ > 5) {
                    left_point_start_ = true;
                    ROS_INFO("连续追踪失败，切换至左点追踪");
                }
            }
            out_.write(cropped);
        }

        if(other_enter){//到达另一个路口，这次右边不会断线，拐弯要强制执行
            bool find_coner = false;
            if(other_enter_last_conner.x<320){
                ROS_INFO("逻辑1,x%d,y%d",other_enter_last_conner.x,other_enter_last_conner.y);
                find_coner = find_other_coner_edge(gray_img,other_enter_last_conner,cropped);//发现拐点，强制执行
            }
            else{
                ROS_INFO("逻辑2,x%d,y%d",other_enter_last_conner.x,other_enter_last_conner.y);
                find_coner = find_other_coner_edge2(gray_img,other_enter_last_conner,cropped);//发现拐点，强制执行
            }
            if(find_coner){
                twist.linear.x = 0.3;
                twist.angular.z = twist.linear.x*(553-other_enter_last_conner.x)/(205-other_enter_last_conner.y);
                displayStream <<"x:"<< twist.linear.x<<"z:"<< twist.angular.z;
                string displayText = displayStream.str();
                putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
                    displayStream << "正常误差: " << line_error 
                                << " P: " << line_error*p_ 
                                << " I: " << integration_*i_ 
                                << " D: " << diff*d_ 
                                << " 角速度: " << twist.angular.z;
                    putText(cropped, displayStream.str(), Point(50, 50),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
                }
                out_.write(cropped);
            }
        }

        if(pass_enter){//此处赛道和底部不是连续的，特殊处理
            bool is_now_white = false;
            vector<Point> maybe_start_point;
            // 底部寻找
            for (int i = 639; i > 2; i--) {
                if (!is_now_white && gray_img.at<uchar>(200, i) == 255) {
                    is_now_white = true;
                }
                if (is_now_white && gray_img.at<uchar>(200, i) == 0) {
                    maybe_start_point.emplace_back(i-1, 200);
                    circle(cropped, Point(i-1, 200), 5, Scalar(0, 0, 255), -1);
                    is_now_white = false;
                }
            }
            // 右部寻找
            is_now_white = true;
            for (int i = 269; i > 190; i--) {
                if (is_now_white && gray_img.at<uchar>(i, 639) == 0) {
                    is_now_white = false;
                }
                if (!is_now_white && gray_img.at<uchar>(i, 639) == 255) {
                    maybe_start_point.emplace_back(639, i);
                    circle(cropped, Point(639, i), 5, Scalar(0, 0, 255), -1);
                    is_now_white = true;
                }
            }
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
                displayStream << "正常误差: " << line_error 
                            << " P: " << line_error*p_ 
                            << " I: " << integration_*i_ 
                            << " D: " << diff*d_ 
                            << " 角速度: " << twist.angular.z;
                putText(cropped, displayStream.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            }
            else{
                pass_enter_count++;//要连续的丢线
                if(pass_enter_count>3){
                    ROS_INFO("回到路口");
                    int recent = recently_white(gray_img,cropped);
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
                        ROS_INFO("准备离开圆环");
                    }
                    displayStream <<"z:"<< twist.angular.z<<"x:  "<<twist.linear.x<<"recent:"<<recent;
                    string displayText = displayStream.str();
                    putText(cropped, displayText, Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
                find_left_edge(gray_img, left_edge_point,cropped);
                double error_x = 320 - left_edge_point.x;
                pointx_integration += error_x * 0.02;
                pointx_integration = clamp(pointx_integration, -1.0, 1.0);
                
                // 左点过低时停止前进
                if (left_edge_point.y > 240) {
                    ROS_INFO("丢线旋转中");
                    twist.linear.x = out_forward_;
                    twist.angular.z = out_turn_;
                    
                    // 旋转到位后切换模式
                    pose_client.call(pose);
                    if (pose.response.pose_at[2] < -1.4) {
                        left_point_start_ = false;
                        double_line = true;
                        nh.getParam("/line_right/double_P", p_);
                        nh.getParam("/line_right/double_I", i_);
                        nh.getParam("/line_right/double_D", d_);
                        ROS_INFO("旋转完成，切换双边巡线 (P=%.2f)", p_);
                    }
                }

                // PID计算
                double point_diff = error_x - pointx_pre_error;
                twist.linear.x = 0.23;
                twist.angular.z = error_x*leftpoint_p_ + pointx_integration*leftpoint_i_ + point_diff*leftpoint_D_;
                pointx_pre_error = error_x;

                // 显示信息
                displayStream << "左点误差: " << error_x 
                            << " P: " << error_x*leftpoint_p_ 
                            << " I: " << pointx_integration*leftpoint_i_;
                putText(cropped, displayStream.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
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
                displayStream << "正常误差: " << line_error 
                            << " P: " << line_error*p_ 
                            << " I: " << integration_*i_ 
                            << " D: " << diff*d_ 
                            << " 角速度: " << twist.angular.z;
                putText(cropped, displayStream.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            } else {
                // 追踪失败计数
                trace_failed_count_++;
                if (trace_failed_count_ > 5) {
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
            displayStream << "双边误差: " << line_error 
                        << " P: " << line_error*p_ 
                        << " I: " << integration_*i_ 
                        << " D: " << diff*d_ 
                        << " 角速度: " << twist.angular.z;
            putText(cropped, displayStream.str(), Point(50, 50),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
            out_.write(cropped);
        }

        int test;
        if(pose.response.pose_at[2]>-1.8&&pose.response.pose_at[2]<-1.3&&pose.response.pose_at[0]>3.3&&pose.response.pose_at[0]<4.2&&pose.response.pose_at[1]<1){
            if(stop_car(gray_img,test,cropped)){//
                ROS_INFO("巡线结束");
                twist.linear.x = 0;
                twist.angular.z = 0;
                cmd_pub.publish(twist);
                imshow("stop",cropped);
                waitKey(0);
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
