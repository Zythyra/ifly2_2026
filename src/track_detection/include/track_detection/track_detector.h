#ifndef TRACK_DETECTOR_H
#define TRACK_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/node_handle.h>

namespace track_detection {

class TrackDetector {
public:
    // 构造函数
    explicit TrackDetector(ros::NodeHandle& nh);
    
    // 主检测函数
    cv::Mat detect(const cv::Mat& frame);

private:
    // 私有成员变量（新增反光处理和点过滤相关参数）
    cv::Scalar low_hsv_;          // HSV下界
    cv::Scalar high_hsv_;         // HSV上界
    double min_contour_area_;     // 最小轮廓面积
    double max_contour_area_;     // 最大轮廓面积
    int max_line_width_;          // 最大线宽
    int min_line_length_;         // 最小线长
    double max_circularity_;      // 最大圆形度
    double roi_height_ratio_;     // ROI高度比例
    cv::Mat erosion_kernel_;      // 腐蚀核
    cv::Mat dilation_kernel_;     // 膨胀核
    
    // 新增：反光点处理参数
    int window_size_;                   // 检测窗口大小
    double reflective_ratio_threshold_; // 反光比例阈值
    double max_point_distance_;         // 点连接最大距离

    // 私有成员函数（新增反光处理和点过滤函数声明）
    cv::Mat cropROI(const cv::Mat& frame);
    std::vector<cv::Point> roiToOriginal(const std::vector<cv::Point>& roi_contour, int roi_y_offset);
    cv::Mat preprocess(const cv::Mat& frame);
    bool isTrackShape(const std::vector<cv::Point>& contour);
    
    // 新增：反光区域移除函数
    cv::Mat removeReflectiveAreas(const cv::Mat& mask);
    // 新增：轮廓点过滤函数
    std::vector<cv::Point> processContourPoints(const std::vector<cv::Point>& contour);
};

} // namespace track_detection

#endif // TRACK_DETECTOR_H