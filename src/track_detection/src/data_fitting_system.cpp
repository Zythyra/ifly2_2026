#include <ros/ros.h>
#include <Eigen/Dense>
#include <boost/math/distributions/normal.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include <opencv2/opencv.hpp>
Eigen::VectorXd compute_z_score_safe(const Eigen::VectorXd& data);
// 数据结构定义
struct TrackPoint {
    Eigen::VectorXd x;  // 像素x坐标
    Eigen::VectorXd y;  // 像素y坐标
};

struct TrackClusters {
    std::vector<TrackPoint> clusters;       // 多个相关点集群
    std::vector<TrackPoint> reflection_clusters; // 被判定为反光的面状集群
    std::vector<cv::Scalar> colors;         // 每个有效集群的显示颜色
    cv::Scalar reflection_color = cv::Scalar(0, 0, 255); // 反光区域颜色（红色）
};

struct FitResult {
    std::vector<Eigen::VectorXd> y_fits;  // 多个拟合结果
    std::vector<std::string> model_names; // 每个拟合的模型名称
    std::vector<bool> is_linear;          // 每个拟合是否为线性
    std::vector<double> smoothness;       // 平滑度评分(0-1，越高越平滑)
};

// 全局变量和同步机制
TrackClusters track_clusters;
FitResult fit_result;
cv::Mat current_frame;
cv::Mat hsv_frame;          
cv::Mat track_mask;         
std::mutex data_mutex;
std::condition_variable data_cond;
bool data_updated = false;
bool running = true;

// 参数设置
int h_min = 0, h_max = 180;    
int s_min = 0, s_max = 50;     
int v_min = 200, v_max = 255;  
double region_ratio = 0.4;     
int region_top = 0;            
int max_correlation_dist = 100;
int min_cluster_size = 10;     
double smoothness_threshold = 0.7; 
int point_simplify_factor = 5;

// 反光检测参数
double aspect_ratio_threshold = 2.5;   // 宽高比阈值，超过此值视为线性
double area_density_threshold = 0.8;   // 面积密度阈值，低于此值视为面状
double linearity_threshold = 0.6;      // 线性度阈值，低于此值视为面状

// 生成随机颜色
std::vector<cv::Scalar> generate_random_colors(int count) {
    std::vector<cv::Scalar> colors;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> dist(50, 255);
    
    for (int i = 0; i < count; ++i) {
        int r = dist(generator);
        int g = dist(generator);
        int b = dist(generator);
        
        if (r > 200 && g > 200 && b > 200) {
            r = std::min(180, r);
        }
        
        colors.emplace_back(b, g, r);
    }
    
    return colors;
}

// 计算两点距离
double point_distance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx*dx + dy*dy);
}

// 并查集实现传递性聚类
int find_root(std::vector<int>& parent, int i) {
    if (parent[i] != i) {
        parent[i] = find_root(parent, parent[i]);
    }
    return parent[i];
}

// 判断集群是否为面状（反光）
bool is_reflection_cluster(const TrackPoint& cluster) {
    if (cluster.x.size() < min_cluster_size * 2) {
        return false; // 小集群不太可能是大面积反光
    }
    
    // 1. 计算边界框和宽高比
    double x_min = cluster.x.minCoeff();
    double x_max = cluster.x.maxCoeff();
    double y_min = cluster.y.minCoeff();
    double y_max = cluster.y.maxCoeff();
    
    double width = x_max - x_min;
    double height = y_max - y_min;
    
    // 过滤过小的区域
    if (width < 20 || height < 20) {
        return false;
    }
    
    double aspect_ratio = std::max(width, height) / std::min(width, height);
    
    // 2. 计算面积密度（点数量 / 边界框面积）
    double area = width * height;
    double density = cluster.x.size() / area;
    
    // 3. 计算线性度（通过主成分分析）
    Eigen::MatrixXd points(cluster.x.size(), 2);
    for (int i = 0; i < cluster.x.size(); ++i) {
        points(i, 0) = cluster.x[i];
        points(i, 1) = cluster.y[i];
    }
    
    // 中心化
    Eigen::VectorXd mean = points.colwise().mean();
    Eigen::MatrixXd centered = points.rowwise() - mean.transpose();
    
    // 计算协方差矩阵
    Eigen::MatrixXd cov = (centered.adjoint() * centered) / (points.rows() - 1);
    
    // 计算特征值（主成分分析）
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Eigen::VectorXd eigenvalues = eig.eigenvalues();
    
    // 线性度 = 最大特征值 / 特征值之和（越接近1越线性）
    double linearity = eigenvalues[1] / eigenvalues.sum();
    
    // 判定为反光（面状）的条件：
    // - 宽高比小（接近正方形）
    // - 点密度低（分布稀疏但区域大）
    // - 线性度低（非长条状）
    bool is_reflection = (aspect_ratio < aspect_ratio_threshold) && 
                        (density < area_density_threshold) && 
                        (linearity < linearity_threshold);
    
    ROS_DEBUG("集群分析 - 宽高比: %.2f, 密度: %.2f, 线性度: %.2f, 反光: %s",
             aspect_ratio, density, linearity, is_reflection ? "是" : "否");
             
    return is_reflection;
}

TrackClusters cluster_points(const TrackPoint& track) {
    TrackClusters clusters;
    if (track.x.size() < 2) return clusters;
    
    int n = track.x.size();
    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i) {
        parent[i] = i;
    }
    
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double dist = point_distance(track.x[i], track.y[i], track.x[j], track.y[j]);
            
            if (dist < max_correlation_dist) {
                int root_i = find_root(parent, i);
                int root_j = find_root(parent, j);
                if (root_i != root_j) {
                    parent[root_j] = root_i;
                }
            }
        }
    }
    
    std::map<int, std::vector<int>> root_to_points;
    for (int i = 0; i < n; ++i) {
        int root = find_root(parent, i);
        root_to_points[root].push_back(i);
    }
    
    // 分离有效集群和反光集群
    for (const auto& entry : root_to_points) {
        const auto& point_indices = entry.second;
        if (point_indices.size() >= min_cluster_size) {
            TrackPoint cluster;
            cluster.x.resize(point_indices.size());
            cluster.y.resize(point_indices.size());
            
            for (size_t k = 0; k < point_indices.size(); ++k) {
                cluster.x[k] = track.x[point_indices[k]];
                cluster.y[k] = track.y[point_indices[k]];
            }
            
            // 判断是否为反光（面状）集群
            if (is_reflection_cluster(cluster)) {
                clusters.reflection_clusters.push_back(cluster);
            } else {
                clusters.clusters.push_back(cluster);
            }
        }
    }
    
    clusters.colors = generate_random_colors(clusters.clusters.size());
    return clusters;
}

// 点简化：Douglas-Peucker算法
TrackPoint simplify_points(const TrackPoint& track, double epsilon = 3.0) {
    if (track.x.size() <= 3) return track;
    
    TrackPoint simplified;
    std::vector<std::pair<double, double>> points;
    for (int i = 0; i < track.x.size(); ++i) {
        points.emplace_back(track.x[i], track.y[i]);
    }
    
    // 按x坐标排序
    std::sort(points.begin(), points.end());
    
    // Douglas-Peucker算法实现
    auto perpendicular_distance = [](const std::pair<double, double>& p, 
                                    const std::pair<double, double>& a, 
                                    const std::pair<double, double>& b) {
        double A = p.first - a.first;
        double B = p.second - a.second;
        double C = b.first - a.first;
        double D = b.second - a.second;
        
        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = len_sq != 0 ? dot / len_sq : -1;
        
        double xx, yy;
        if (param < 0) {
            xx = a.first;
            yy = a.second;
        } else if (param > 1) {
            xx = b.first;
            yy = b.second;
        } else {
            xx = a.first + param * C;
            yy = a.second + param * D;
        }
        
        double dx = p.first - xx;
        double dy = p.second - yy;
        return std::sqrt(dx * dx + dy * dy);
    };
    
    std::function<void(int, int, std::vector<bool>&)> dp = 
        [&](int start, int end, std::vector<bool>& keep) {
        if (start + 1 >= end) return;
        
        double max_dist = 0;
        int index = start;
        
        for (int i = start + 1; i < end; ++i) {
            double dist = perpendicular_distance(points[i], points[start], points[end]);
            if (dist > max_dist) {
                max_dist = dist;
                index = i;
            }
        }
        
        if (max_dist > epsilon) {
            keep[index] = true;
            dp(start, index, keep);
            dp(index, end, keep);
        }
    };
    
    std::vector<bool> keep(points.size(), false);
    keep[0] = true;
    keep.back() = true;
    dp(0, points.size() - 1, keep);
    
    // 收集保留的点
    for (size_t i = 0; i < points.size(); ++i) {
        if (keep[i]) {
            simplified.x.conservativeResize(simplified.x.size() + 1);
            simplified.y.conservativeResize(simplified.y.size() + 1);
            simplified.x[simplified.x.size() - 1] = points[i].first;
            simplified.y[simplified.y.size() - 1] = points[i].second;
        }
    }
    
    return simplified;
}

// 增强的点过滤
TrackPoint enhanced_filter_points(const TrackPoint& track, double z_threshold = 2.5) {
    if (track.x.size() < 5) return track;
    
    TrackPoint filtered = track;
    
    // 1. Z-score异常值检测
    Eigen::VectorXd z_scores_x = compute_z_score_safe(track.x);
    Eigen::VectorXd z_scores_y = compute_z_score_safe(track.y);
    Eigen::VectorXd abs_z = z_scores_x.array().abs().max(z_scores_y.array().abs());
    
    std::vector<double> x_clean, y_clean;
    for (int i = 0; i < abs_z.size(); ++i) {
        if (abs_z[i] < z_threshold) {
            x_clean.push_back(track.x[i]);
            y_clean.push_back(track.y[i]);
        }
    }
    
    if (x_clean.size() < 5) {
        x_clean.clear(); y_clean.clear();
        for (int i = 0; i < track.x.size(); ++i) {
            x_clean.push_back(track.x[i]);
            y_clean.push_back(track.y[i]);
        }
    }
    
    // 2. 密度过滤：保留周围点更多的点
    TrackPoint dense_filtered;
    int n = x_clean.size();
    std::vector<int> neighbor_counts(n, 0);
    double search_radius = max_correlation_dist * 0.5;
    
    for (int i = 0; i < n; ++i) {
        int count = 0;
        for (int j = 0; j < n; ++j) {
            if (i != j && point_distance(x_clean[i], y_clean[i], x_clean[j], y_clean[j]) < search_radius) {
                count++;
            }
        }
        neighbor_counts[i] = count;
    }
    
    // 保留邻居数量高于平均值的点
    double mean_neighbors = std::accumulate(neighbor_counts.begin(), neighbor_counts.end(), 0.0) / n;
    for (int i = 0; i < n; ++i) {
        if (neighbor_counts[i] >= mean_neighbors * 0.7) {
            dense_filtered.x.conservativeResize(dense_filtered.x.size() + 1);
            dense_filtered.y.conservativeResize(dense_filtered.y.size() + 1);
            dense_filtered.x[dense_filtered.x.size() - 1] = x_clean[i];
            dense_filtered.y[dense_filtered.y.size() - 1] = y_clean[i];
        }
    }
    
    return dense_filtered.x.size() >= 5 ? dense_filtered : track;
}

// 计算平滑度评分
double calculate_smoothness(const Eigen::VectorXd& x, const Eigen::VectorXd& y_fit) {
    if (x.size() < 4) return 0.0;
    
    // 计算相邻点的斜率变化
    std::vector<double> slopes;
    for (int i = 0; i < x.size() - 1; ++i) {
        if (x[i+1] != x[i]) {
            slopes.push_back((y_fit[i+1] - y_fit[i]) / (x[i+1] - x[i]));
        }
    }
    
    if (slopes.size() < 3) return 0.0;
    
    // 计算斜率变化率的标准差
    std::vector<double> slope_changes;
    for (size_t i = 0; i < slopes.size() - 1; ++i) {
        slope_changes.push_back(std::abs(slopes[i+1] - slopes[i]));
    }
    
    double mean_change = std::accumulate(slope_changes.begin(), slope_changes.end(), 0.0) / slope_changes.size();
    double std_change = 0.0;
    for (double c : slope_changes) {
        std_change += (c - mean_change) * (c - mean_change);
    }
    std_change = std::sqrt(std_change / slope_changes.size());
    
    // 归一化到0-1范围
    double smoothness = 1.0 - std::min(1.0, std_change / 5.0);
    return std::max(0.0, smoothness);
}

// 辅助函数
bool has_invalid_values(const Eigen::VectorXd& v) {
    for (int i = 0; i < v.size(); ++i) {
        if (std::isnan(v[i]) || std::isinf(v[i])) {
            return true;
        }
    }
    return false;
}

void clean_track_points(Eigen::VectorXd& x, Eigen::VectorXd& y) {
    std::vector<std::pair<double, double>> unique_points;
    
    for (int i = 0; i < x.size(); ++i) {
        if (std::isnan(x[i]) || std::isinf(x[i]) || 
            std::isnan(y[i]) || std::isinf(y[i])) {
            continue;
        }
        
        bool is_duplicate = false;
        for (const auto& p : unique_points) {
            if (std::abs(x[i] - p.first) < 3.0 && 
                std::abs(y[i] - p.second) < 3.0) {
                is_duplicate = true;
                break;
            }
        }
        
        if (!is_duplicate) {
            unique_points.emplace_back(x[i], y[i]);
        }
    }
    
    x.resize(unique_points.size());
    y.resize(unique_points.size());
    for (size_t i = 0; i < unique_points.size(); ++i) {
        x[i] = unique_points[i].first;
        y[i] = unique_points[i].second;
    }
}

void detect_track_points(const cv::Mat& frame, TrackPoint& track) {
    region_top = frame.rows * (1 - region_ratio);
    
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, 
               cv::Scalar(h_min, s_min, v_min), 
               cv::Scalar(h_max, s_max, v_max), 
               track_mask);
    
    cv::Rect bottom_region(0, region_top, frame.cols, frame.rows - region_top);
    cv::Mat bottom_mask = track_mask(bottom_region);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(bottom_mask, bottom_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(bottom_mask, bottom_mask, cv::MORPH_OPEN, kernel);
    bottom_mask.copyTo(track_mask(bottom_region));
    
    std::vector<double> x_coords, y_coords;
    for (int y = 0; y < bottom_mask.rows; ++y) {
        for (int x = 0; x < bottom_mask.cols; ++x) {
            if (bottom_mask.at<uchar>(y, x) == 255) {
                x_coords.push_back(x);
                y_coords.push_back(y + region_top);
            }
        }
    }
    
    track.x = Eigen::VectorXd::Map(x_coords.data(), x_coords.size());
    track.y = Eigen::VectorXd::Map(y_coords.data(), y_coords.size());
    clean_track_points(track.x, track.y);
}

double compute_median(const Eigen::VectorXd& v) {
    if (v.size() == 0) return 0.0;
    
    Eigen::VectorXd sorted = v;
    std::sort(sorted.data(), sorted.data() + sorted.size());
    
    int n = sorted.size();
    if (n % 2 == 1) {
        return sorted[n / 2];
    } else {
        return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
    }
}

Eigen::VectorXd compute_z_score_safe(const Eigen::VectorXd& data) {
    double mean = data.mean();
    double std_dev = sqrt((data.array() - mean).square().mean());
    
    if (std_dev < 1e-9) {
        return Eigen::VectorXd::Zero(data.size());
    }
    
    return (data.array() - mean) / std_dev;
}

// 改进的Lowess算法
Eigen::VectorXd improved_lowess(const Eigen::VectorXd& x, const Eigen::VectorXd& y, double frac = 0.5) {
    int n = x.size();
    if (n < 2) return Eigen::VectorXd::Zero(n);
    
    // 按x排序
    Eigen::VectorXd sorted_x = x;
    Eigen::VectorXd sorted_y = y;
    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&x](int a, int b) { return x[a] < x[b]; });
    
    for (int i = 0; i < n; ++i) {
        sorted_x[i] = x[indices[i]];
        sorted_y[i] = y[indices[i]];
    }
    
    Eigen::VectorXd y_fit(n);
    int k = std::max(5, (int)(frac * n));
    
    for (int i = 0; i < n; ++i) {
        // 计算权重
        Eigen::VectorXd weights(n);
        for (int j = 0; j < n; ++j) {
            double dist = std::abs(sorted_x[j] - sorted_x[i]) / 
                         (sorted_x.maxCoeff() - sorted_x.minCoeff() + 1e-9);
            weights[j] = std::max(0.0, 1.0 - std::pow(dist, 3));
        }
        
        // 加权线性回归
        Eigen::MatrixXd X(n, 3);
        Eigen::VectorXd W(n);
        
        for (int j = 0; j < n; ++j) {
            double w = std::sqrt(weights[j]);
            X(j, 0) = w;
            X(j, 1) = w * sorted_x[j];
            X(j, 2) = w * sorted_x[j] * sorted_x[j];
            W(j) = w * sorted_y[j];
        }
        
        Eigen::VectorXd beta = X.colPivHouseholderQr().solve(W);
        y_fit[i] = beta[0] + beta[1] * sorted_x[i] + beta[2] * sorted_x[i] * sorted_x[i];
    }
    
    // 反排序回原始顺序
    Eigen::VectorXd result(n);
    for (int i = 0; i < n; ++i) {
        result[indices[i]] = y_fit[i];
    }
    
    return result;
}

// 自适应拟合单个赛道集群
std::tuple<Eigen::VectorXd, std::string, bool, double> fit_single_track_smooth(TrackPoint track) {
    if (track.x.size() < 5) {
        return std::make_tuple(Eigen::VectorXd(), "点数量不足", false, 0.0);
    }
    
    // 增强过滤
    TrackPoint filtered_track = enhanced_filter_points(track);
    
    // 初步拟合
    double initial_frac = 0.5;
    Eigen::VectorXd y_fit = improved_lowess(filtered_track.x, filtered_track.y, initial_frac);
    double smoothness = calculate_smoothness(filtered_track.x, y_fit);
    
    // 平滑度优化
    if (smoothness < smoothness_threshold) {
        ROS_DEBUG("平滑度不足(%.2f)，简化点集...", smoothness);
        
        TrackPoint simplified = filtered_track;
        double current_epsilon = 2.0;
        int attempt = 0;
        
        while (smoothness < smoothness_threshold && simplified.x.size() > 10 && attempt < 5) {
            simplified = simplify_points(filtered_track, current_epsilon);
            y_fit = improved_lowess(simplified.x, simplified.y, std::min(0.7, initial_frac + 0.1 * attempt));
            smoothness = calculate_smoothness(simplified.x, y_fit);
            current_epsilon += 1.0;
            attempt++;
        }
        
        if (smoothness < smoothness_threshold) {
            y_fit = improved_lowess(simplified.x, simplified.y, 0.8);
            smoothness = calculate_smoothness(simplified.x, y_fit);
        }
    }
    
    return std::make_tuple(y_fit, "平滑曲线赛道", false, smoothness);
}

// 拟合多个赛道集群
FitResult fit_multiple_tracks(const TrackClusters& clusters) {
    FitResult result;
    
    for (const auto& cluster : clusters.clusters) {
        auto fit_tuple = fit_single_track_smooth(cluster);
        Eigen::VectorXd y_fit = std::get<0>(fit_tuple);
        std::string model_name = std::get<1>(fit_tuple);
        bool is_linear = std::get<2>(fit_tuple);
        double smoothness = std::get<3>(fit_tuple);
        
        if (y_fit.size() > 0) {
            result.y_fits.push_back(y_fit);
            result.model_names.push_back(model_name + 
                " (平滑度: " + std::to_string((int)(smoothness * 100)) + "%)");
            result.is_linear.push_back(is_linear);
            result.smoothness.push_back(smoothness);
        }
    }
    
    return result;
}

// 滑动条回调函数
void on_h_min_change(int value, void* userdata) { h_min = value; }
void on_h_max_change(int value, void* userdata) { h_max = value; }
void on_s_min_change(int value, void* userdata) { s_min = value; }
void on_s_max_change(int value, void* userdata) { s_max = value; }
void on_v_min_change(int value, void* userdata) { v_min = value; }
void on_v_max_change(int value, void* userdata) { v_max = value; }
void on_region_ratio_change(int value, void* userdata) { 
    region_ratio = value / 100.0;
}
void on_distance_change(int value, void* userdata) { 
    max_correlation_dist = value;
}
void on_cluster_size_change(int value, void* userdata) { 
    min_cluster_size = value;
}
void on_smoothness_threshold_change(int value, void* userdata) { 
    smoothness_threshold = value / 100.0;
}
void on_aspect_ratio_change(int value, void* userdata) { 
    aspect_ratio_threshold = value / 10.0;  // 范围1.0-10.0
}
void on_density_threshold_change(int value, void* userdata) { 
    area_density_threshold = value / 100.0;  // 范围0.1-1.0
}
void on_linearity_threshold_change(int value, void* userdata) { 
    linearity_threshold = value / 100.0;  // 范围0.1-0.9
}

// 可视化线程
void visualization_thread() {
    cv::namedWindow("参数调整", cv::WINDOW_NORMAL);
    cv::createTrackbar("H_min", "参数调整", &h_min, 180, on_h_min_change);
    cv::createTrackbar("H_max", "参数调整", &h_max, 180, on_h_max_change);
    cv::createTrackbar("S_min", "参数调整", &s_min, 255, on_s_min_change);
    cv::createTrackbar("S_max", "参数调整", &s_max, 255, on_s_max_change);
    cv::createTrackbar("V_min", "参数调整", &v_min, 255, on_v_min_change);
    cv::createTrackbar("V_max", "参数调整", &v_max, 255, on_v_max_change);
    cv::createTrackbar("区域比例(%)", "参数调整", nullptr, 100, on_region_ratio_change);
    cv::setTrackbarPos("区域比例(%)", "参数调整", (int)(region_ratio * 100));
    cv::createTrackbar("最大相关距离", "参数调整", nullptr, 200, on_distance_change);
    cv::setTrackbarPos("最大相关距离", "参数调整", max_correlation_dist);
    cv::createTrackbar("最小集群大小", "参数调整", nullptr, 50, on_cluster_size_change);
    cv::setTrackbarPos("最小集群大小", "参数调整", min_cluster_size);
    cv::createTrackbar("平滑度阈值(%)", "参数调整", nullptr, 100, on_smoothness_threshold_change);
    cv::setTrackbarPos("平滑度阈值(%)", "参数调整", (int)(smoothness_threshold * 100));
    
    // 反光检测参数滑动条
    cv::createTrackbar("宽高比阈值(x0.1)", "参数调整", nullptr, 100, on_aspect_ratio_change);
    cv::setTrackbarPos("宽高比阈值(x0.1)", "参数调整", (int)(aspect_ratio_threshold * 10));
    cv::createTrackbar("密度阈值(%)", "参数调整", nullptr, 100, on_density_threshold_change);
    cv::setTrackbarPos("密度阈值(%)", "参数调整", (int)(area_density_threshold * 100));
    cv::createTrackbar("线性度阈值(%)", "参数调整", nullptr, 100, on_linearity_threshold_change);
    cv::setTrackbarPos("线性度阈值(%)", "参数调整", (int)(linearity_threshold * 100));
    
    cv::namedWindow("摄像头视图", cv::WINDOW_NORMAL);
    cv::namedWindow("赛道掩码", cv::WINDOW_NORMAL);
    cv::namedWindow("赛道拟合结果", cv::WINDOW_NORMAL);
    
    while (running && ros::ok()) {
        std::unique_lock<std::mutex> lock(data_mutex);
        data_cond.wait(lock, []{ return data_updated || !running; });
        
        if (!running || !ros::ok()) break;
        if (!data_updated) continue;
        
        TrackClusters current_clusters = track_clusters;
        FitResult current_fit = fit_result;
        cv::Mat frame = current_frame.clone();
        cv::Mat mask = track_mask.clone();
        int top = region_top;
        data_updated = false;
        lock.unlock();
        
        // 绘制处理区域
        cv::line(frame, cv::Point(0, top), cv::Point(frame.cols, top), cv::Scalar(0, 255, 0), 2);
        cv::rectangle(frame, cv::Point(10, top + 10), cv::Point(150, top + 40), cv::Scalar(0, 255, 0), -1);
        cv::putText(frame, "处理区域", cv::Point(15, top + 35),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
        
        // 标记反光集群（红色）
        for (const auto& refl_cluster : current_clusters.reflection_clusters) {
            for (int i = 0; i < refl_cluster.x.size(); ++i) {
                cv::circle(frame, 
                          cv::Point((int)refl_cluster.x[i], (int)refl_cluster.y[i]),
                          2, current_clusters.reflection_color, -1);
            }
        }
        
        // 标记有效集群
        std::string cluster_info = "有效赛道集群: " + std::to_string(current_clusters.clusters.size()) + 
                                 " (反光集群: " + std::to_string(current_clusters.reflection_clusters.size()) + ")";
        cv::putText(frame, cluster_info, cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        
        for (size_t c = 0; c < current_clusters.clusters.size(); ++c) {
            const auto& cluster = current_clusters.clusters[c];
            cv::Scalar color = current_clusters.colors[c];
            
            for (int i = 0; i < cluster.x.size(); ++i) {
                cv::circle(frame, 
                          cv::Point((int)cluster.x[i], (int)cluster.y[i]),
                          2, color, -1);
            }
        }
        
        cv::putText(frame, "有效赛道点（彩色）| 反光区域（红色）", cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
        cv::imshow("摄像头视图", frame);
        cv::imshow("赛道掩码", mask);
        
        // 绘制拟合结果
        cv::Mat fit_img(500, 640, CV_8UC3, cv::Scalar(255, 255, 255));
        
        if (current_clusters.clusters.empty()) {
            std::string msg = current_clusters.reflection_clusters.empty() 
                ? "未检测到足够赛道点" 
                : "所有检测到的点均为反光区域";
            cv::putText(fit_img, msg, cv::Point(50, 250),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            cv::imshow("赛道拟合结果", fit_img);
            if (cv::waitKey(30) == 27) running = false;
            continue;
        }
        
        // 确定坐标范围
        double x_min = current_clusters.clusters[0].x.minCoeff();
        double x_max = current_clusters.clusters[0].x.maxCoeff();
        double y_min = current_clusters.clusters[0].y.minCoeff();
        double y_max = current_clusters.clusters[0].y.maxCoeff();
        
        for (size_t c = 1; c < current_clusters.clusters.size(); ++c) {
            x_min = std::min(x_min, current_clusters.clusters[c].x.minCoeff());
            x_max = std::max(x_max, current_clusters.clusters[c].x.maxCoeff());
            y_min = std::min(y_min, current_clusters.clusters[c].y.minCoeff());
            y_max = std::max(y_max, current_clusters.clusters[c].y.maxCoeff());
        }
        
        double x_pad = (x_max - x_min) * 0.1;
        double y_pad = (y_max - y_min) * 0.1;
        x_min -= x_pad;
        x_max += x_pad;
        y_min -= y_pad;
        y_max += y_pad;
        
        auto x_to_pixel = [&](double x) {
            return (int)(((x - x_min) / (x_max - x_min)) * (fit_img.cols - 100) + 50);
        };
        auto y_to_pixel = [&](double y) {
            return (int)(((y_max - y) / (y_max - y_min)) * (fit_img.rows - 100) + 50);
        };
        
        // 绘制坐标轴
        cv::line(fit_img, cv::Point(50, fit_img.rows - 50), 
                cv::Point(fit_img.cols - 50, fit_img.rows - 50), 
                cv::Scalar(200, 200, 200), 1);
        cv::line(fit_img, cv::Point(50, 50), 
                cv::Point(50, fit_img.rows - 50), 
                cv::Scalar(200, 200, 200), 1);
        
        // 绘制每个集群的点和拟合曲线
        for (size_t c = 0; c < current_clusters.clusters.size(); ++c) {
            const auto& cluster = current_clusters.clusters[c];
            cv::Scalar color = current_clusters.colors[c];
            
            // 绘制赛道点
            for (int i = 0; i < cluster.x.size(); ++i) {
                cv::circle(fit_img,
                          cv::Point(x_to_pixel(cluster.x[i]), y_to_pixel(cluster.y[i])),
                          3, color, -1);
            }
            
            // 绘制拟合曲线
            if (c < current_fit.y_fits.size() && current_fit.y_fits[c].size() == cluster.x.size()) {
                std::vector<cv::Point> curve_points;
                for (int i = 0; i < cluster.x.size(); ++i) {
                    curve_points.emplace_back(
                        x_to_pixel(cluster.x[i]), 
                        y_to_pixel(current_fit.y_fits[c][i])
                    );
                }
                
                cv::polylines(fit_img, curve_points, false, color, 2, cv::LINE_AA);
                
                // 显示模型名称和平滑度
                std::string model_text = "赛道 " + std::to_string(c+1) + ": " + current_fit.model_names[c];
                cv::putText(fit_img, model_text, cv::Point(50, 30 + (c+1)*25),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }
        
        cv::imshow("赛道拟合结果", fit_img);
        
        if (cv::waitKey(30) == 27) {
            running = false;
        }
    }
    
    cv::destroyAllWindows();
}

// 图像处理线程
void processing_thread() {
    while (running && ros::ok()) {
        std::unique_lock<std::mutex> lock(data_mutex);
        data_cond.wait(lock, []{ return data_updated || !running; });
        
        if (!running || !ros::ok()) break;
        if (!data_updated) continue;
        
        cv::Mat frame = current_frame.clone();
        data_updated = false;
        lock.unlock();
        
        TrackPoint track;
        detect_track_points(frame, track);
        
        TrackClusters clusters = cluster_points(track);
        ROS_INFO_THROTTLE(1, "检测到 %d 个有效赛道集群，%d 个反光集群（总点数: %d）", 
                         (int)clusters.clusters.size(), 
                         (int)clusters.reflection_clusters.size(), 
                         (int)track.x.size());
        
        FitResult result = fit_multiple_tracks(clusters);
        
        lock.lock();
        track_clusters = clusters;
        fit_result = result;
        data_updated = true;
        lock.unlock();
        data_cond.notify_one();
    }
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "hsv_anti_reflection_fitting_node");
    ros::NodeHandle nh("~");
    
    int camera_index;
    int frame_width, frame_height;
    nh.param<int>("camera_index", camera_index, 0);
    nh.param<int>("frame_width", frame_width, 640);
    nh.param<int>("frame_height", frame_height, 480);
    
    nh.param<int>("h_min", h_min, 0);
    nh.param<int>("h_max", h_max, 180);
    nh.param<int>("s_min", s_min, 0);
    nh.param<int>("s_max", s_max, 50);
    nh.param<int>("v_min", v_min, 200);
    nh.param<int>("v_max", v_max, 255);
    
    nh.param<double>("region_ratio", region_ratio, 0.4);
    region_ratio = std::max(0.1, std::min(1.0, region_ratio));
    
    nh.param<int>("max_correlation_dist", max_correlation_dist, 100);
    max_correlation_dist = std::max(50, std::min(300, max_correlation_dist));
    
    nh.param<int>("min_cluster_size", min_cluster_size, 10);
    min_cluster_size = std::max(5, std::min(100, min_cluster_size));
    
    nh.param<double>("smoothness_threshold", smoothness_threshold, 0.7);
    smoothness_threshold = std::max(0.3, std::min(0.95, smoothness_threshold));
    
    // 反光检测参数
    nh.param<double>("aspect_ratio_threshold", aspect_ratio_threshold, 2.5);
    aspect_ratio_threshold = std::max(1.0, std::min(10.0, aspect_ratio_threshold));
    
    nh.param<double>("area_density_threshold", area_density_threshold, 0.8);
    area_density_threshold = std::max(0.1, std::min(2.0, area_density_threshold));
    
    nh.param<double>("linearity_threshold", linearity_threshold, 0.6);
    linearity_threshold = std::max(0.1, std::min(0.9, linearity_threshold));
    
    cv::VideoCapture cap(camera_index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头设备 /dev/video%d", camera_index);
        return -1;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    ROS_INFO("摄像头已打开: 分辨率 %dx%d，处理区域: 底部 %.0f%%", 
             frame_width, frame_height, region_ratio * 100);
    ROS_INFO("反光检测参数: 宽高比=%.1f, 密度=%.2f, 线性度=%.2f", 
             aspect_ratio_threshold, area_density_threshold, linearity_threshold);
    
    std::thread proc_thread(processing_thread);
    std::thread viz_thread(visualization_thread);
    
    cv::Mat frame;
    while (running && ros::ok()) {
        if (!cap.read(frame)) {
            ROS_WARN("无法读取摄像头帧，尝试重新获取...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_frame = frame.clone();
            data_updated = true;
        }
        data_cond.notify_one();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
    running = false;
    data_cond.notify_all();
    proc_thread.join();
    viz_thread.join();
    cap.release();
    
    ROS_INFO("程序已退出");
    return 0;
}
