#include "track_detection/track_detector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <ros/console.h>

namespace track_detection {

TrackDetector::TrackDetector(ros::NodeHandle& nh) {
    // 加载HSV阈值参数（白色线条）
    int h_low, s_low, v_low;
    int h_high, s_high, v_high;
    nh.param<int>("hsv_low_h", h_low, 0);
    nh.param<int>("hsv_low_s", s_low, 0);
    nh.param<int>("hsv_low_v", v_low, 150);  // 放宽亮度阈值
    nh.param<int>("hsv_high_h", h_high, 180);
    nh.param<int>("hsv_high_s", s_high, 50);  // 放宽饱和度阈值
    nh.param<int>("hsv_high_v", v_high, 255);
    low_hsv_ = cv::Scalar(h_low, s_low, v_low);
    high_hsv_ = cv::Scalar(h_high, s_high, v_high);
    
    // 加载过滤参数（初始放宽设置）
    nh.param<double>("min_contour_area", min_contour_area_, 20.0);
    nh.param<double>("max_contour_area", max_contour_area_, 500.0);
    nh.param<int>("max_line_width", max_line_width_, 10);
    nh.param<int>("min_line_length", min_line_length_, 10);
    nh.param<double>("max_circularity", max_circularity_, 0.8);
    
    // 加载ROI参数
    nh.param<double>("roi_height_ratio", roi_height_ratio_, 0.7);  // 检测下半部分70%
    if (roi_height_ratio_ < 0.1 || roi_height_ratio_ > 1.0) {
        roi_height_ratio_ = 0.7;
        ROS_WARN("ROI比例超出范围，使用默认值0.7");
    }
    
    // 加载反光点处理参数
    nh.param<int>("window_size", window_size_, 50);  // 检测反光的矩形窗口大小
    nh.param<double>("reflective_ratio_threshold", reflective_ratio_threshold_, 0.6);  // 20%阈值
    nh.param<double>("max_point_distance", max_point_distance_, 30.0);  // 点连接最大距离
    
    // 确保参数有效性
    if (window_size_ < 3) window_size_ = 3;
    if (reflective_ratio_threshold_ < 0.01 || reflective_ratio_threshold_ > 0.99) {
        reflective_ratio_threshold_ = 0.2;
        ROS_WARN("反光比例阈值超出范围，使用默认值0.2");
    }
    
    // 初始化形态学操作核
    erosion_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    dilation_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
}

// 裁剪下半部分ROI区域
cv::Mat TrackDetector::cropROI(const cv::Mat& frame) {
    int frame_height = frame.rows;
    int roi_y = static_cast<int>(frame_height * (1 - roi_height_ratio_));
    int roi_height = frame_height - roi_y;
    
    // 确保ROI有效
    if (roi_y < 0) roi_y = 0;
    if (roi_height <= 0) roi_height = frame_height;
    
    cv::Rect roi_rect(0, roi_y, frame.cols, roi_height);
    return frame(roi_rect).clone();
}

// 转换ROI坐标到原始图像坐标
std::vector<cv::Point> TrackDetector::roiToOriginal(const std::vector<cv::Point>& roi_contour, int roi_y_offset) {
    std::vector<cv::Point> original_contour;
    for (const auto& pt : roi_contour) {
        original_contour.emplace_back(pt.x, pt.y + roi_y_offset);
    }
    return original_contour;
}

// 检测并移除反光区域
cv::Mat TrackDetector::removeReflectiveAreas(const cv::Mat& mask) {
    cv::Mat processed_mask = mask.clone();
    int rows = processed_mask.rows;
    int cols = processed_mask.cols;
    int step = window_size_ / 2;  // 滑动步长为窗口大小的一半，提高检测精度
    
    for (int y = 0; y < rows; y += step) {
        for (int x = 0; x < cols; x += step) {
            // 计算窗口实际区域，避免超出图像边界
            int win_width = std::min(window_size_, cols - x);
            int win_height = std::min(window_size_, rows - y);
            cv::Rect window(x, y, win_width, win_height);
            
            // 计算窗口内白色像素比例
            int white_pixels = cv::countNonZero(processed_mask(window));
            int total_pixels = window.area();
            
            if (total_pixels == 0) continue;
            
            // 如果超过阈值，标记为反光区域并清除
            double ratio = static_cast<double>(white_pixels) / total_pixels;
            if (ratio > reflective_ratio_threshold_) {
                processed_mask(window).setTo(cv::Scalar(0));
            }
        }
    }
    
    return processed_mask;
}

// 处理轮廓点，只保留距离≤max_point_distance的连续点
std::vector<cv::Point> TrackDetector::processContourPoints(const std::vector<cv::Point>& contour) {
    std::vector<cv::Point> processed;
    if (contour.empty()) return processed;
    
    processed.push_back(contour[0]);
    
    for (size_t i = 1; i < contour.size(); ++i) {
        const cv::Point& prev = processed.back();
        const cv::Point& curr = contour[i];
        
        // 计算欧氏距离
        double distance = cv::norm(curr - prev);
        
        // 只有距离小于等于阈值的点才保留
        if (distance <= max_point_distance_) {
            processed.push_back(curr);
        }
        // 否则视为断开连接，不添加当前点
    }
    
    return processed;
}

// 预处理图像（提取白色区域并去除反光）
cv::Mat TrackDetector::preprocess(const cv::Mat& frame) {
    // 裁剪ROI
    cv::Mat roi = cropROI(frame);
    
    // HSV转换与阈值过滤
    cv::Mat hsv, mask, mask_no_reflect;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, low_hsv_, high_hsv_, mask);
    
    // 显示去除反光前的掩码
    cv::imshow("去除反光前的原始掩码", mask);  
    cv::waitKey(1);  
    
    // // 保存图像到指定路径
    // static int count = 0; // 帧计数器，避免文件覆盖
    // std::string save_dir = "/home/ucar/ucar_car/src/track_detection/img/";
    // std::string save_path = save_dir + "reflective_mask_before_" + std::to_string(count++) + ".png";
    

    
    // // 保存图像
    // bool save_success = cv::imwrite(save_path, mask);
    // if (save_success) {
    //     ROS_INFO("已保存去除反光前的掩码图像至：%s", save_path.c_str());
    // } else {
    //     ROS_ERROR("保存图像失败：%s，请检查路径是否正确或权限是否足够", save_path.c_str());
    // }

    // 移除反光区域（直接对原始掩码处理，不经过形态学操作）
    mask_no_reflect = removeReflectiveAreas(mask);
    
    return mask_no_reflect;
}

// // 判断轮廓是否符合赛道特征
// bool TrackDetector::isTrackShape(const std::vector<cv::Point>& contour) {
//     // 面积过滤
//     double area = cv::contourArea(contour);
//     if (area < min_contour_area_ || area > max_contour_area_) {
//         return false;
//     }
    
//     // 周长计算（避免除零）
//     double perimeter = cv::arcLength(contour, true);
//     if (perimeter < 1e-6) return false;
    
//     // 旋转矩形计算（宽度和长度）
//     cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
//     cv::Size2f rect_size = rotated_rect.size;
//     float line_width = std::min(rect_size.width, rect_size.height);
//     float line_length = std::max(rect_size.width, rect_size.height);
    
//     // 宽度和长度过滤
//     bool is_width_valid = (line_width <= max_line_width_);
//     bool is_length_valid = (line_length >= min_line_length_);
    
//     // 圆形度过滤
//     double circularity = 4 * M_PI * area / (perimeter * perimeter);
//     bool is_smooth_line = (circularity <= max_circularity_);
    
//     return is_width_valid && is_length_valid && is_smooth_line;
// }

// 主检测函数
cv::Mat TrackDetector::detect(const cv::Mat& frame) {
    if (frame.empty()) {
        ROS_WARN("输入图像为空！");
        return cv::Mat();
    }
    
    int frame_height = frame.rows;
    int roi_y_offset = static_cast<int>(frame_height * (1 - roi_height_ratio_));
    
    // 1. 裁剪ROI
    cv::Mat roi = cropROI(frame);
    // 2. 预处理获取掩码（包含反光去除）
    cv::Mat mask = preprocess(frame);
    cv::imshow("1. 图像", roi);
    cv::imshow("2. 掩码图像（去除反光后）", mask);
    cv::waitKey(1);  
    return frame.clone();  // 或返回处理后的图像
    // // 3. 初始化结果图像
    // cv::Mat blue_result = frame.clone();  // 所有轮廓（蓝色）
    // cv::Mat green_result = frame.clone(); // 有效轮廓（绿色）
    
    // // 绘制ROI边界线
    // cv::line(blue_result, cv::Point(0, roi_y_offset), 
    //          cv::Point(frame.cols, roi_y_offset), 
    //          cv::Scalar(0, 0, 255), 2);  // 红色ROI边界线
    // cv::line(green_result, cv::Point(0, roi_y_offset), 
    //          cv::Point(frame.cols, roi_y_offset), 
    //          cv::Scalar(0, 0, 255), 2);
    
    // // 4. 寻找轮廓
    // std::vector<std::vector<cv::Point>> roi_contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(mask, roi_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // // 5. 处理轮廓点（过滤距离过远的点）
    // std::vector<std::vector<cv::Point>> processed_roi_contours;
    // for (const auto& contour : roi_contours) {
    //     processed_roi_contours.push_back(processContourPoints(contour));
    // }
    
    // // 6.1 显示所有处理后的轮廓（蓝色）
    // for (const auto& roi_contour : processed_roi_contours) {
    //     std::vector<cv::Point> original_contour = roiToOriginal(roi_contour, roi_y_offset);
    //     cv::drawContours(blue_result, std::vector<std::vector<cv::Point>>{original_contour}, 
    //                     -1, cv::Scalar(255, 0, 0), 2);
    // }
    
    // // 6.2 显示符合条件的赛道轮廓（绿色）
    // for (const auto& roi_contour : processed_roi_contours) {
    //     if (isTrackShape(roi_contour)) {
    //         std::vector<cv::Point> original_contour = roiToOriginal(roi_contour, roi_y_offset);
    //         cv::drawContours(green_result, std::vector<std::vector<cv::Point>>{original_contour}, 
    //                         -1, cv::Scalar(0, 255, 0), 2);
    //     }
    // }
    
    // 显示窗口
    // cv::imshow("3. 所有处理后的轮廓（蓝色）", blue_result);
    // cv::imshow("4. 符合条件的赛道轮廓（绿色）", green_result);
    // cv::waitKey(1);
    
    // return green_result;
}

} // namespace track_detection
