#ifndef TRACK_DETECTION_TRACK_DETECTOR_H
#define TRACK_DETECTION_TRACK_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <ros/node_handle.h>

namespace track_detection {

class TrackDetector {
public:
    TrackDetector(ros::NodeHandle& nh);
    
    // 主检测函数
    cv::Mat detect(const cv::Mat& frame);
    
private:
    // 裁剪ROI区域
    cv::Mat cropROI(const cv::Mat& frame);
    
    // 转换ROI坐标到原始图像坐标
    std::vector<cv::Point> roiToOriginal(const std::vector<cv::Point>& roi_contour, int roi_y_offset);
    
    // 预处理图像（提取白色区域）
    cv::Mat preprocess(const cv::Mat& frame);
    
    // 判断轮廓是否符合赛道特征
    bool isTrackShape(const std::vector<cv::Point>& contour);
    
    // 从掩码中提取所有有效像素点
    std::vector<cv::Point> extractPointsFromMask(const cv::Mat& mask);
    
    // 计算点到直线的距离
    double pointToLineDistance(const cv::Point& p, double k, double b);
    
    // RANSAC直线拟合算法
    bool ransacLineFitting(const std::vector<cv::Point>& points, 
                          double& k, double& b, 
                          std::vector<cv::Point>& inliers,
                          std::vector<cv::Point>& outliers,
                          int iterations = 100, double threshold = 3.0);
    
    // 多项式曲线拟合 (二次曲线)
    bool polynomialFitting(const std::vector<cv::Point>& points,
                         std::vector<double>& coefficients, // a*x² + b*x + c
                         std::vector<cv::Point>& inliers,
                         std::vector<cv::Point>& outliers,
                         double threshold = 5.0);
    
    // 计算线性相关系数
    double calculateCorrelationCoefficient(const std::vector<cv::Point>& points);
    
    // 绘制拟合曲线
    void drawFittedLine(cv::Mat& image, double k, double b, const cv::Scalar& color, int thickness = 2);
    void drawFittedCurve(cv::Mat& image, const std::vector<double>& coefficients, const cv::Scalar& color, int thickness = 2);
    
    // HSV阈值参数
    cv::Scalar low_hsv_;
    cv::Scalar high_hsv_;
    
    // 过滤参数
    double min_contour_area_;
    double max_contour_area_;
    int max_line_width_;
    int min_line_length_;
    double max_circularity_;
    
    // ROI参数
    double roi_height_ratio_;
    
    // 形态学操作核
    cv::Mat erosion_kernel_;
    cv::Mat dilation_kernel_;
    
    // 拟合相关参数
    double ransac_threshold_;      // RANSAC距离阈值
    int ransac_iterations_;        // RANSAC迭代次数
    double polynomial_threshold_;  // 多项式拟合距离阈值
};

} // namespace track_detection

#endif // TRACK_DETECTION_TRACK_DETECTOR_H
