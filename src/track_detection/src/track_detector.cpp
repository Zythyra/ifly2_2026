#include "track_detection/track_detector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <ros/console.h>
#include <random>
#include <numeric>
#include <algorithm>

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
    
    // 加载拟合参数
    nh.param<double>("ransac_threshold", ransac_threshold_, 3.0);
    nh.param<int>("ransac_iterations", ransac_iterations_, 100);
    nh.param<double>("polynomial_threshold", polynomial_threshold_, 5.0);
    
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

// 预处理图像（提取白色区域）
cv::Mat TrackDetector::preprocess(const cv::Mat& frame) {
    // 裁剪ROI
    cv::Mat roi = cropROI(frame);
    
    // HSV转换与阈值过滤
    cv::Mat hsv, mask, eroded, dilated;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, low_hsv_, high_hsv_, mask);
    
    // 形态学操作
    cv::erode(mask, eroded, erosion_kernel_, cv::Point(-1, -1), 1);
    cv::dilate(eroded, dilated, dilation_kernel_, cv::Point(-1, -1), 1);
    
    return dilated;
}

// 判断轮廓是否符合赛道特征
bool TrackDetector::isTrackShape(const std::vector<cv::Point>& contour) {
    // 面积过滤
    double area = cv::contourArea(contour);
    if (area < min_contour_area_ || area > max_contour_area_) {
        return false;
    }
    
    // 周长计算（避免除零）
    double perimeter = cv::arcLength(contour, true);
    if (perimeter < 1e-6) return false;
    
    // 旋转矩形计算（宽度和长度）
    cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
    cv::Size2f rect_size = rotated_rect.size;
    float line_width = std::min(rect_size.width, rect_size.height);
    float line_length = std::max(rect_size.width, rect_size.height);
    
    // 宽度和长度过滤
    bool is_width_valid = (line_width <= max_line_width_);
    bool is_length_valid = (line_length >= min_line_length_);
    
    // 圆形度过滤
    double circularity = 4 * M_PI * area / (perimeter * perimeter);
    bool is_smooth_line = (circularity <= max_circularity_);
    
    return is_width_valid && is_length_valid && is_smooth_line;
}

// 从掩码中提取所有有效像素点
std::vector<cv::Point> TrackDetector::extractPointsFromMask(const cv::Mat& mask) {
    std::vector<cv::Point> points;
    if (mask.empty() || mask.type() != CV_8UC1) {
        return points;
    }
    
    for (int y = 0; y < mask.rows; ++y) {
        for (int x = 0; x < mask.cols; ++x) {
            if (mask.at<uchar>(y, x) == 255) {  // 白色像素点
                points.emplace_back(x, y);
            }
        }
    }
    return points;
}

// 计算点到直线的距离 (y = kx + b)
double TrackDetector::pointToLineDistance(const cv::Point& p, double k, double b) {
    // 距离公式: |kx - y + b| / sqrt(k² + 1)
    return std::abs(k * p.x - p.y + b) / std::sqrt(k * k + 1);
}

// RANSAC直线拟合算法
bool TrackDetector::ransacLineFitting(const std::vector<cv::Point>& points, 
                                     double& k, double& b, 
                                     std::vector<cv::Point>& inliers,
                                     std::vector<cv::Point>& outliers,
                                     int iterations, double threshold) {
    if (points.size() < 2) {
        return false;
    }
    
    int best_inlier_count = 0;
    double best_k = 0.0, best_b = 0.0;
    std::vector<cv::Point> best_inliers, best_outliers;
    
    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, points.size() - 1);
    
    for (int i = 0; i < iterations; ++i) {
        // 随机选择两个点
        int idx1 = dist(gen);
        int idx2 = dist(gen);
        while (idx1 == idx2) {
            idx2 = dist(gen);
        }
        
        cv::Point p1 = points[idx1];
        cv::Point p2 = points[idx2];
        
        // 避免除以零
        if (p1.x == p2.x) {
            continue;  // 垂直直线暂不处理
        }
        
        // 计算直线参数 y = kx + b
        double current_k = static_cast<double>(p2.y - p1.y) / (p2.x - p1.x);
        double current_b = p1.y - current_k * p1.x;
        
        // 计算内点和外点
        std::vector<cv::Point> current_inliers, current_outliers;
        for (const auto& p : points) {
            double distance = pointToLineDistance(p, current_k, current_b);
            if (distance < threshold) {
                current_inliers.push_back(p);
            } else {
                current_outliers.push_back(p);
            }
        }
        
        // 更新最佳模型
        if (current_inliers.size() > best_inlier_count) {
            best_inlier_count = current_inliers.size();
            best_k = current_k;
            best_b = current_b;
            best_inliers = current_inliers;
            best_outliers = current_outliers;
        }
    }
    
    // 使用所有内点重新拟合，优化直线参数
    if (best_inliers.size() >= 2) {
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        int n = best_inliers.size();
        
        for (const auto& p : best_inliers) {
            sum_x += p.x;
            sum_y += p.y;
            sum_xy += p.x * p.y;
            sum_x2 += p.x * p.x;
        }
        
        double denominator = n * sum_x2 - sum_x * sum_x;
        if (std::abs(denominator) > 1e-6) {
            best_k = (n * sum_xy - sum_x * sum_y) / denominator;
            best_b = (sum_y - best_k * sum_x) / n;
        }
    }
    
    k = best_k;
    b = best_b;
    inliers = best_inliers;
    outliers = best_outliers;
    
    return best_inlier_count > 0;
}

// 多项式曲线拟合 (二次曲线: y = ax² + bx + c)
bool TrackDetector::polynomialFitting(const std::vector<cv::Point>& points,
                                    std::vector<double>& coefficients,
                                    std::vector<cv::Point>& inliers,
                                    std::vector<cv::Point>& outliers,
                                    double threshold) {
    if (points.size() < 3) {  // 二次曲线至少需要3个点
        return false;
    }
    
    int n = points.size();
    cv::Mat X(n, 3, CV_64F);  // 设计矩阵 [x², x, 1]
    cv::Mat Y(n, 1, CV_64F);  // 目标值 [y]
    
    // 填充矩阵
    for (int i = 0; i < n; ++i) {
        double x = points[i].x;
        X.at<double>(i, 0) = x * x;  // x²
        X.at<double>(i, 1) = x;      // x
        X.at<double>(i, 2) = 1.0;    // 常数项
        Y.at<double>(i, 0) = points[i].y;
    }
    
    // 求解最小二乘问题: X * coeff = Y
    cv::Mat coeff_mat;
    cv::solve(X, Y, coeff_mat, cv::DECOMP_SVD);
    
    // 提取系数
    coefficients.resize(3);
    coefficients[0] = coeff_mat.at<double>(0, 0);  // a
    coefficients[1] = coeff_mat.at<double>(1, 0);  // b
    coefficients[2] = coeff_mat.at<double>(2, 0);  // c
    
    // 分类内点和外点
    for (const auto& p : points) {
        double y_pred = coefficients[0] * p.x * p.x + coefficients[1] * p.x + coefficients[2];
        double error = std::abs(p.y - y_pred);
        
        if (error < threshold) {
            inliers.push_back(p);
        } else {
            outliers.push_back(p);
        }
    }
    
    return true;
}

// 计算线性相关系数
double TrackDetector::calculateCorrelationCoefficient(const std::vector<cv::Point>& points) {
    if (points.size() < 2) {
        return 0.0;
    }
    
    double sum_x = 0, sum_y = 0, sum_xy = 0;
    double sum_x2 = 0, sum_y2 = 0;
    int n = points.size();
    
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += p.x * p.y;
        sum_x2 += p.x * p.x;
        sum_y2 += p.y * p.y;
    }
    
    double numerator = n * sum_xy - sum_x * sum_y;
    double denominator = std::sqrt((n * sum_x2 - sum_x * sum_x) * (n * sum_y2 - sum_y * sum_y));
    
    if (denominator < 1e-6) {
        return 0.0;
    }
    
    return numerator / denominator;
}

// 绘制拟合直线
void TrackDetector::drawFittedLine(cv::Mat& image, double k, double b, const cv::Scalar& color, int thickness) {
    if (image.empty()) return;
    
    int x1 = 0;
    int y1 = static_cast<int>(k * x1 + b);
    int x2 = image.cols - 1;
    int y2 = static_cast<int>(k * x2 + b);
    
    // 确保点在图像范围内
    y1 = std::max(0, std::min(y1, image.rows - 1));
    y2 = std::max(0, std::min(y2, image.rows - 1));
    
    cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), color, thickness);
}

// 绘制拟合曲线
void TrackDetector::drawFittedCurve(cv::Mat& image, const std::vector<double>& coefficients, const cv::Scalar& color, int thickness) {
    if (image.empty() || coefficients.size() < 3) return;
    
    cv::Point prev_point;
    bool first_point = true;
    
    // 沿x轴绘制曲线
    for (int x = 0; x < image.cols; ++x) {
        double y = coefficients[0] * x * x + coefficients[1] * x + coefficients[2];
        
        // 确保点在图像范围内
        if (y >= 0 && y < image.rows) {
            cv::Point curr_point(x, static_cast<int>(y));
            
            if (first_point) {
                prev_point = curr_point;
                first_point = false;
            } else {
                cv::line(image, prev_point, curr_point, color, thickness);
                prev_point = curr_point;
            }
        }
    }
}

// 主检测函数（融合拟合算法）
cv::Mat TrackDetector::detect(const cv::Mat& frame) {
    if (frame.empty()) {
        ROS_WARN("输入图像为空！");
        return cv::Mat();
    }
    
    int frame_height = frame.rows;
    int roi_y_offset = static_cast<int>(frame_height * (1 - roi_height_ratio_));
    
    // 1. 裁剪ROI
    cv::Mat roi = cropROI(frame);
    
    // 2. 预处理获取掩码
    cv::Mat mask = preprocess(frame);
    cv::imshow("2. 掩码图像（白色区域）", mask);
    
    // 3. 提取所有符合HSV阈值的像素点
    std::vector<cv::Point> roi_points = extractPointsFromMask(mask);
    std::vector<cv::Point> original_points = roiToOriginal(roi_points, roi_y_offset);
    
    // 4. 初始化结果图像
    cv::Mat result = frame.clone();
    
    // 绘制ROI边界线
    cv::line(result, cv::Point(0, roi_y_offset), 
             cv::Point(frame.cols, roi_y_offset), 
             cv::Scalar(0, 0, 255), 2);  // 红色ROI边界线
    
    // 5. 如果有足够的点，进行拟合
    if (roi_points.size() >= 5) {  // 至少需要5个点进行有效拟合
        // 计算线性相关系数，判断使用直线还是曲线拟合
        double correlation = calculateCorrelationCoefficient(roi_points);
        ROS_DEBUG("线性相关系数: %.2f", correlation);
        
        // 强线性相关 (|r| > 0.7) 使用直线拟合
        if (std::abs(correlation) > 0.7) {
            double k, b;
            std::vector<cv::Point> line_inliers_roi, line_outliers_roi;
            
            if (ransacLineFitting(roi_points, k, b, line_inliers_roi, line_outliers_roi,
                                 ransac_iterations_, ransac_threshold_)) {
                // 转换内点和外点到原始坐标
                std::vector<cv::Point> line_inliers = roiToOriginal(line_inliers_roi, roi_y_offset);
                std::vector<cv::Point> line_outliers = roiToOriginal(line_outliers_roi, roi_y_offset);
                
                // 绘制内点（绿色）和外点（红色）
                for (const auto& p : line_inliers) {
                    cv::circle(result, p, 2, cv::Scalar(0, 255, 0), -1);
                }
                for (const auto& p : line_outliers) {
                    cv::circle(result, p, 2, cv::Scalar(0, 0, 255), -1);
                }
                
                // 绘制拟合直线（蓝色）
                drawFittedLine(result, k, b, cv::Scalar(255, 0, 0), 2);
                ROS_INFO("直线拟合成功 - 内点: %zu, 外点: %zu", 
                         line_inliers.size(), line_outliers.size());
            }
        } 
        // 弱线性相关，使用二次曲线拟合
        else {
            std::vector<double> coefficients;
            std::vector<cv::Point> curve_inliers_roi, curve_outliers_roi;
            
            if (polynomialFitting(roi_points, coefficients, curve_inliers_roi, curve_outliers_roi,
                                 polynomial_threshold_)) {
                // 转换内点和外点到原始坐标
                std::vector<cv::Point> curve_inliers = roiToOriginal(curve_inliers_roi, roi_y_offset);
                std::vector<cv::Point> curve_outliers = roiToOriginal(curve_outliers_roi, roi_y_offset);
                
                // 绘制内点（绿色）和外点（红色）
                for (const auto& p : curve_inliers) {
                    cv::circle(result, p, 2, cv::Scalar(0, 255, 0), -1);
                }
                for (const auto& p : curve_outliers) {
                    cv::circle(result, p, 2, cv::Scalar(0, 0, 255), -1);
                }
                
                // 绘制拟合曲线（紫色）
                drawFittedCurve(result, coefficients, cv::Scalar(255, 0, 255), 2);
                ROS_INFO("曲线拟合成功 - 内点: %zu, 外点: %zu", 
                         curve_inliers.size(), curve_outliers.size());
            }
        }
    } else if (!roi_points.empty()) {
        ROS_WARN("点数量不足，无法进行拟合: %zu个点", roi_points.size());
        // 绘制所有点（黄色）
        for (const auto& p : original_points) {
            cv::circle(result, p, 2, cv::Scalar(0, 255, 255), -1);
        }
    }
    
    // 6. 显示结果
    cv::imshow("3. 拟合结果", result);
    cv::waitKey(1);  // 刷新窗口
    
    return result;
}

} // namespace track_detection
