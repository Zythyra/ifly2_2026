// 版本：初始直行找线 + 雷达接管停车版 V4（2026-07-29）
// 修改基线：用户上传的 line_right(5).cpp（898行最终巡线代码）
// 启动巡线服务后先以 x_max_ 直行，检测到有效右边白线后永久切入正常巡线；
// 终点白线只用于触发控制模式切换，不会在白线处发布零速度。
// 切换后使用独立参数控制前进速度；首次读取的左侧最小雷达距离作为目标距离，
// 后续通过左墙拟合保持平行，并根据左侧最小雷达距离保持等距；
// 雷达数据由永久订阅回调缓存，控制循环以固定频率持续发布速度；
// 仅当前方雷达最近有效点不大于设定阈值时停车。

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <random>
#include <string>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <sstream>
#include <limits>
#include <algorithm>
#include <mutex>
#include <cstdint>
#include "line_follow/line_follow.h"
#include "ucarmain2026/getpose_server.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// 定义MoveBase客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv;
using namespace std;

// 声明赛道结构体（关键修复：在类外部或内部提前声明）
struct RaceTrack {
    double slope;                  // 赛道斜率
    vector<Point> points;          // 赛道点集
    int direction_change;          // 方向变化次数
    int slope_change_count;        // 斜率变化次数
    bool left_point;               // 是否为左赛道标志
};

class LineFollowerNode {
private:
    // ROS核心组件
    ros::NodeHandle nh_;                  // 节点句柄
    ros::ServiceServer line_server_;      // 服务端
    ros::Publisher cmd_pub_;              // 速度发布者
    ros::Subscriber scan_sub_;            // 永久雷达订阅者

    ros::ServiceClient pose_client_;      // 位姿服务客户端
    ros::ServiceClient reconfigure_client_;// 动态配置客户端
    tf::TransformListener* tf_listener_;  // TF监听器
    MoveBaseClient* ac_;                  // MoveBase客户端

    // 消息对象
    geometry_msgs::Twist twist_;

    ucarmain2026::getpose_server pose_;

    // 图像处理相关
    Mat cameraMatrix_, distCoeffs_;       // 相机内参和畸变系数
    VideoCapture cap_;                    // 相机捕获
    VideoWriter out_;                     // 视频录制
    string output_file_;                  // 视频保存路径
    int fourcc_;                          // 视频编码格式
    ostringstream displayStream_;         // 信息显示流
    Rect roi_;                            // 图像裁剪区域
    Mat map1_, map2_;                     // 去畸变映射表
    int center_distance;

    // 控制参数
    double p_, i_, d_;                    // PID参数
    double leftpoint_p_, leftpoint_I_, leftpoint_D_; // 左点控制参数
    double x_max_, integration_limit_;    // 速度和积分限制
    double out_turn_, out_forward_,out_turn_angel_;       // 旋转和前进参数
    double integration_, pre_error_;      // 积分和前向误差
    double pointx_integration_, pointx_pre_error_; // 左点积分和前向误差

    // 雷达靠左墙行驶参数
    string scan_topic_;                    // 雷达话题
    double lidar_left_angle_min_deg_;      // 左侧拟合扇区最小角度
    double lidar_left_angle_max_deg_;      // 左侧拟合扇区最大角度
    double lidar_front_half_angle_deg_;    // 前方停车检测半角
    double lidar_forward_speed_;           // 雷达停车阶段的独立前进速度
    double lidar_front_stop_distance_;     // 前方立即停车距离
    double lidar_heading_kp_;              // 平行控制增益
    double lidar_distance_kp_;             // 等距控制增益
    double lidar_max_angular_speed_;        // 雷达阶段最大角速度
    double lidar_min_valid_range_;         // 参与拟合的最小量程
    double lidar_max_valid_range_;         // 参与拟合的最大量程
    double lidar_wall_max_residual_;       // 墙面拟合离群点阈值
    double lidar_filter_alpha_;            // 墙面结果低通滤波系数
    double lidar_control_rate_;            // 雷达阶段速度发布频率
    double lidar_scan_stale_timeout_;       // 雷达数据过期停车阈值
    int lidar_min_wall_points_;             // 拟合左墙所需的最少点数
    int lidar_wall_invalid_grace_scans_;    // 左墙无效帧容错次数
    int lidar_front_invalid_grace_scans_;   // 前方无效帧容错次数

    // 雷达缓存由AsyncSpinner的雷达回调写入、巡线服务线程读取。
    std::mutex scan_mutex_;
    sensor_msgs::LaserScanConstPtr latest_scan_;
    ros::Time latest_scan_receive_time_;
    std::uint64_t scan_sequence_;

    // 状态变量

    bool double_line_;                    // 双边巡线标志
    bool left_point_start_;               // 左点追踪标志
    bool point_forward_;                  // 左点前进标志
    int trace_failed_count_;              // 追踪失败计数

public:
    // 构造函数：初始化所有组件
    LineFollowerNode() : 
        nh_(""),
        tf_listener_(nullptr),
        ac_(nullptr),
        output_file_("/home/ucar/ucar_ws_copy/src/line_follow/image/line_right.avi"),
        fourcc_(VideoWriter::fourcc('X', 'V', 'I', 'D')),
        roi_(0, 210, 640, 270),

        double_line_(false),
        left_point_start_(false),
        point_forward_(true),
        trace_failed_count_(0),
        integration_(0), 
        pre_error_(0),
        pointx_integration_(0),
        pointx_pre_error_(0),
        scan_sequence_(0) {

        ROS_INFO("启动 line_right V4（初始直行找线、正常巡线、雷达接管停车）");

        // 1. 初始化服务端（优先初始化）
        line_server_ = nh_.advertiseService("line_right", &LineFollowerNode::line_server_callback, this);
        ROS_INFO("line_right服务已初始化");

        // 2. 加载参数
        loadParameters();

        // 3. 初始化ROS客户端和发布者
        initRosComponents();

        // 4. 读取相机标定文件并初始化去畸变
        if (!loadCalibrationFile()) {
            ROS_FATAL("标定文件加载失败，节点无法启动");
            ros::shutdown();
            return;
        }

        ROS_INFO("所有组件初始化完成");
    }

    // 析构函数：释放资源
    ~LineFollowerNode() {
        cap_.release();
        out_.release();
        delete tf_listener_;
        delete ac_;
        ROS_INFO("节点资源已释放");
    }

    // 运行节点主循环
    void run() {
        // 服务回调会持续执行视觉巡线和雷达控制循环，至少需要另一个线程
        // 专门接收/缓存LaserScan，否则服务执行期间雷达回调无法运行。
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    }

private:
    // 加载ROS参数
    void loadParameters() {
        nh_.getParam("/line_right/right_P", p_);
        nh_.getParam("/line_right/right_I", i_);
        nh_.getParam("/line_right/right_D", d_);
        nh_.getParam("/line_right/leftpoint_p", leftpoint_p_);
        nh_.getParam("/line_right/leftpoint_I", leftpoint_I_);
        nh_.getParam("/line_right/leftpoint_D", leftpoint_D_);
        nh_.getParam("/line_right/x_max_", x_max_);
        nh_.getParam("/line_right/integration_limit", integration_limit_);
        nh_.getParam("/line_right/out_forward", out_forward_);
        nh_.getParam("/line_right/out_turn", out_turn_);
        nh_.getParam("/line_right/out_turn_angel", out_turn_angel_);
        nh_.getParam("/line_right/center_distance", center_distance);

        nh_.param<string>("/line_right/scan_topic", scan_topic_, "/scan");
        nh_.param("/line_right/lidar_left_angle_min_deg", lidar_left_angle_min_deg_, 60.0);
        nh_.param("/line_right/lidar_left_angle_max_deg", lidar_left_angle_max_deg_, 120.0);
        nh_.param("/line_right/lidar_front_half_angle_deg", lidar_front_half_angle_deg_, 15.0);
        nh_.param("/line_right/lidar_forward_speed", lidar_forward_speed_, 0.50);
        nh_.param("/line_right/lidar_front_stop_distance", lidar_front_stop_distance_, 0.25);
        nh_.param("/line_right/lidar_heading_kp", lidar_heading_kp_, 1.8);
        nh_.param("/line_right/lidar_distance_kp", lidar_distance_kp_, 2.0);
        nh_.param("/line_right/lidar_max_angular_speed", lidar_max_angular_speed_, 0.8);
        nh_.param("/line_right/lidar_min_valid_range", lidar_min_valid_range_, 0.08);
        nh_.param("/line_right/lidar_max_valid_range", lidar_max_valid_range_, 1.50);
        nh_.param("/line_right/lidar_wall_max_residual", lidar_wall_max_residual_, 0.03);
        nh_.param("/line_right/lidar_filter_alpha", lidar_filter_alpha_, 0.35);
        nh_.param("/line_right/lidar_control_rate", lidar_control_rate_, 30.0);
        nh_.param("/line_right/lidar_scan_stale_timeout", lidar_scan_stale_timeout_, 0.25);
        nh_.param("/line_right/lidar_min_wall_points", lidar_min_wall_points_, 8);
        nh_.param("/line_right/lidar_wall_invalid_grace_scans", lidar_wall_invalid_grace_scans_, 3);
        nh_.param("/line_right/lidar_front_invalid_grace_scans", lidar_front_invalid_grace_scans_, 2);

        lidar_filter_alpha_ = clamp(lidar_filter_alpha_, 0.0, 1.0);
        lidar_control_rate_ = std::max(lidar_control_rate_, 1.0);
        lidar_scan_stale_timeout_ = std::max(lidar_scan_stale_timeout_, 0.05);
        lidar_wall_invalid_grace_scans_ = std::max(lidar_wall_invalid_grace_scans_, 0);
        lidar_front_invalid_grace_scans_ = std::max(lidar_front_invalid_grace_scans_, 0);

        ROS_INFO("参数加载完成: center_distance=%d, 雷达话题=%s, 雷达前进速度=%.3f m/s, 前方停车=%.3f m, 控制频率=%.1f Hz",
                 center_distance, scan_topic_.c_str(),
                 lidar_forward_speed_, lidar_front_stop_distance_,
                 lidar_control_rate_);
    }

    // 初始化ROS组件（客户端、发布者等）
    void initRosComponents() {
        // 初始化速度发布者
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ROS_INFO("cmd_vel发布者已初始化");

        // 节点启动后永久订阅雷达。服务回调运行时，由AsyncSpinner的另一个
        // 线程持续更新缓存，雷达控制循环无需重复建立订阅。
        scan_sub_ = nh_.subscribe(
            scan_topic_,
            10,
            &LineFollowerNode::scanCallback,
            this
        );
        ROS_INFO("已永久订阅雷达话题：%s", scan_topic_.c_str());

        // 初始化位姿服务客户端
        ROS_INFO("等待坐标获取服务中...");
        pose_client_ = nh_.serviceClient<ucarmain2026::getpose_server>("/getpose_server");
        pose_.request.getpose_start = 1;
        if (!pose_client_.waitForExistence()) {
            ROS_FATAL("超时未连接到getpose_server服务");
            ros::shutdown();
        }
        ROS_INFO("getpose_server服务已连接");

        // 初始化MoveBase客户端
        ac_ = new MoveBaseClient("move_base", true);
        ROS_INFO("等待movebase服务中...");
        if (!ac_->waitForServer()) {
            ROS_FATAL("超时未连接到move_base服务");
            ros::shutdown();
        }
        ROS_INFO("move_base action server 已连接");

        // 初始化动态配置客户端
        reconfigure_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
        if (!reconfigure_client_.waitForExistence()) {
            ROS_FATAL("超时未连接到动态配置服务");
            ros::shutdown();
        }
        ROS_INFO("动态配置服务已连接");
        configureMoveBaseParameters();

        // 初始化TF监听器
        tf_listener_ = new tf::TransformListener();
        ROS_INFO("TF变换监听器已初始化");
    }

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = scan;
        latest_scan_receive_time_ = ros::Time::now();
        ++scan_sequence_;
    }

    // 配置move_base参数
    void configureMoveBaseParameters() {
        dynamic_reconfigure::ReconfigureRequest request;
        dynamic_reconfigure::ReconfigureResponse response;
        dynamic_reconfigure::DoubleParameter planner_frequency;
        planner_frequency.name = "planner_frequency";
        planner_frequency.value = 0.0;
        request.config.doubles.push_back(planner_frequency);
        
        if (reconfigure_client_.call(request, response)) {
            ROS_INFO("参数更新成功");
            double new_value;
            if (ros::param::get("/move_base/planner_frequency", new_value)) {
                ROS_INFO("Current planner_frequency: %.2f", new_value);
            }
        } else {
            ROS_ERROR("参数更新失败");
        }
    }

    // 加载相机标定文件
    bool loadCalibrationFile() {
    std::string calibration_file;

    nh_.param<std::string>(
        "/line_right/calibration_file",
        calibration_file,
        "/home/ucar/ucar_ws_copy/src/line_follow/camera_info/pinhole.yaml"
    );

    ROS_INFO("准备加载相机标定文件：%s", calibration_file.c_str());

    FileStorage fs(calibration_file, FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_ERROR("无法打开标定文件：%s", calibration_file.c_str());
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;

    if (cameraMatrix_.empty() || distCoeffs_.empty()) {
        ROS_ERROR("标定文件内容不完整，缺少 camera_matrix 或 distortion_coefficients");
        return false;
    }

    Mat optimalMatrix = getOptimalNewCameraMatrix(
        cameraMatrix_,
        distCoeffs_,
        Size(640, 480),
        1,
        Size(640, 480)
    );

    initUndistortRectifyMap(
        cameraMatrix_,
        distCoeffs_,
        Mat(),
        optimalMatrix,
        Size(640, 480),
        CV_16SC2,
        map1_,
        map2_
    );

    ROS_INFO("标定文件加载和去畸变初始化完成");
    return true;
}

    // 初始化相机和视频录制
    bool initCameraAndVideo() {
        // 打开相机
        cap_.open("/dev/video0", cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            ROS_ERROR("无法打开相机");
            return false;
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 5);

        // 初始化视频录制
        out_.open(output_file_, fourcc_, 5, Size(640, 270));
        if (!out_.isOpened()) {
            ROS_ERROR("无法打开视频输出文件");
            return false;
        }
        ROS_INFO("相机和视频录制初始化完成");
        return true;
    }

    // 服务回调函数（核心逻辑）
    bool line_server_callback(line_follow::line_follow::Request& req, line_follow::line_follow::Response& resp) {
        Mat image, brightness_threshold_image, cropped, gray_img;
        bool switch_to_lidar = false;
        // 每次启动巡线服务时，先进入“初始直行找线”状态。
        // 该状态只会退出一次；检测到有效右边线后，后续丢线仍执行原来的固定右转逻辑。
        bool initial_straight_mode = true;

        // 5. 初始化相机和视频录制
        if (!initCameraAndVideo()) {
            ROS_FATAL("相机或视频初始化失败，节点无法启动");
            stopRobot();
            return false;
        }

        ROS_INFO("进入初始直行找线状态：以 x_max_=%.3f m/s 直行，检测到有效白线后进入正常巡线",
                 x_max_);

        while (ros::ok()) {
            // 读取并预处理图像
            cap_.read(image);
            if (image.empty()) continue;
            cropped = image(roi_);
            flip(cropped, cropped, 1); // 翻转图像
            vector<Mat> channels;
            split(cropped, channels);
            gray_img = channels[2]; // 红色通道作为灰度图
            int brightness_threshold = brightness_threshold_calculator(gray_img,cropped);
            threshold(gray_img, brightness_threshold_image, 180, 255, THRESH_BINARY);
            threshold_image(gray_img);
            
            // imshow("test",gray_img);
            // waitKey(0);
            cv::cvtColor(gray_img, cropped, cv::COLOR_GRAY2BGR);

            if (initial_straight_mode) {
                // 初始阶段复用正常巡线的边线识别，但禁止执行“丢线右转”。
                // 未找到有效右边线时，runNormalTracking() 会强制保持 x_max_ 直行；
                // 找到后则在当前帧直接输出正常PID巡线速度。
                if (runNormalTracking(gray_img, cropped, false)) {
                    initial_straight_mode = false;
                    trace_failed_count_ = 0;
                    ROS_INFO("初始直行阶段检测到有效白线，退出初始状态并进入正常巡线");
                }

                cmd_pub_.publish(twist_);
                continue;
            }

            // 检测到白线后不再停车，也不再使用视觉计算速度。
            // 雷达阶段改用独立的前进速度参数。
            int stop_point_count = 0;
            if (stop_car(gray_img, stop_point_count, cropped)) {
                switch_to_lidar = true;
                out_.write(cropped);

                ROS_INFO("检测到白线（白点数=%d），切换为雷达左墙跟随，独立前进速度 %.3f m/s",
                         stop_point_count, lidar_forward_speed_);
                break;
            }

            // 新场地只保留右边巡线模式。
            // 正常情况下跟踪右侧边线；右线连续丢失后，直接执行固定右转。
            runNormalTracking(gray_img, cropped, true);

            // 发布速度指令
            cmd_pub_.publish(twist_);
        }

        if (ros::ok() && switch_to_lidar) {
            runLidarWallFollowing();
        } else {
            stopRobot();
        }

        // 为下一次服务调用复位状态。
        double_line_ = false;
        left_point_start_ = false;
        point_forward_ = true;
        trace_failed_count_ = 0;
        integration_ = 0.0;
        pre_error_ = 0.0;
        pointx_integration_ = 0.0;
        pointx_pre_error_ = 0.0;
        twist_ = geometry_msgs::Twist();
        cap_.release();
        out_.release();
        return true;
    }

    int brightness_threshold_calculator(Mat& gray_img,Mat& visualizeImg){//寻找跳变最剧烈的那个点，这个点的左值就是图像二值化阈值
        int max_brightness_change = 0;
        int best_binary_brightness = 180;//给个默认值，别一会没找到
        Point threshold_keypoint;
        for (int y = 269; y > 100; y--) {
            for (int x = 30; x < 638; x++) {
                int current = (int)gray_img.at<uchar>(y, x);
                int next = (int)gray_img.at<uchar>(y, x + 1);
                if (next>=150&&current>80){   
                    if (next - current >= max_brightness_change) {
                        max_brightness_change = next - current;
                        best_binary_brightness = next-25;
                        threshold_keypoint = Point(x,y);
                    }
                }
            }
        }
        circle(visualizeImg, threshold_keypoint, 7, Scalar(0, 255, 255), -1);
        return best_binary_brightness;
    }

    
    // 停止机器人
    void stopRobot() {
        twist_.linear.x = 0;
        twist_.linear.y = 0;
        twist_.angular.z = 0;
        cmd_pub_.publish(twist_);
    }

    struct WallEstimate {
        double heading;   // 左墙方向相对车头方向的夹角，单位 rad
        double distance;  // 雷达到左墙拟合直线的垂直距离，单位 m
        int point_count;
    };

    bool isValidRange(const sensor_msgs::LaserScan& scan, float range) const {
        if (!std::isfinite(range)) {
            return false;
        }

        const double lower = std::max(
            lidar_min_valid_range_,
            static_cast<double>(scan.range_min)
        );

        double upper = lidar_max_valid_range_;
        if (std::isfinite(scan.range_max) && scan.range_max > 0.0) {
            upper = std::min(
                upper,
                static_cast<double>(scan.range_max)
            );
        }

        return range >= lower && range <= upper;
    }

    bool getFrontMinDistance(
        const sensor_msgs::LaserScan& scan,
        double& front_min_distance
    ) const {
        const double pi = 3.14159265358979323846;
        const double half_angle =
            lidar_front_half_angle_deg_ * pi / 180.0;

        front_min_distance = std::numeric_limits<double>::infinity();
        bool found_valid_sample = false;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double angle =
                scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            if (std::abs(angle) > half_angle) {
                continue;
            }

            const float range = scan.ranges[i];
            // 正无穷通常表示该方向量程内没有回波，应视作“前方无近障”，
            // 而不是无效帧。NaN、负无穷和越界有限值才忽略。
            if (std::isinf(range) && range > 0.0f) {
                found_valid_sample = true;
                continue;
            }

            if (!std::isfinite(range) ||
                range < scan.range_min ||
                (std::isfinite(scan.range_max) && range > scan.range_max)) {
                continue;
            }

            front_min_distance =
                std::min(front_min_distance, static_cast<double>(range));
            found_valid_sample = true;
        }

        return found_valid_sample;
    }

    bool getLeftMinDistance(
        const sensor_msgs::LaserScan& scan,
        double& left_min_distance
    ) const {
        const double pi = 3.14159265358979323846;
        const double min_angle =
            lidar_left_angle_min_deg_ * pi / 180.0;
        const double max_angle =
            lidar_left_angle_max_deg_ * pi / 180.0;

        left_min_distance = std::numeric_limits<double>::infinity();
        bool found = false;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double angle =
                scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            const float range = scan.ranges[i];
            if (!isValidRange(scan, range)) {
                continue;
            }

            left_min_distance =
                std::min(left_min_distance, static_cast<double>(range));
            found = true;
        }

        return found;
    }

    bool fitLeftWall(
        const sensor_msgs::LaserScan& scan,
        WallEstimate& estimate
    ) const {
        const double pi = 3.14159265358979323846;
        const double min_angle =
            lidar_left_angle_min_deg_ * pi / 180.0;
        const double max_angle =
            lidar_left_angle_max_deg_ * pi / 180.0;

        vector<Point2f> points;
        points.reserve(scan.ranges.size());

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double angle =
                scan.angle_min + static_cast<double>(i) * scan.angle_increment;

            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            const float range = scan.ranges[i];
            if (!isValidRange(scan, range)) {
                continue;
            }

            // 雷达坐标系：x向前，y向左。
            points.emplace_back(
                range * std::cos(angle),
                range * std::sin(angle)
            );
        }

        if (static_cast<int>(points.size()) < lidar_min_wall_points_) {
            return false;
        }

        Vec4f first_line;
        fitLine(points, first_line, DIST_L2, 0, 0.01, 0.01);

        const double first_vx = first_line[0];
        const double first_vy = first_line[1];
        const double first_x0 = first_line[2];
        const double first_y0 = first_line[3];

        vector<Point2f> inliers;
        inliers.reserve(points.size());
        for (const auto& point : points) {
            const double residual = std::abs(
                first_vy * (point.x - first_x0) -
                first_vx * (point.y - first_y0)
            );

            if (residual <= lidar_wall_max_residual_) {
                inliers.push_back(point);
            }
        }

        if (static_cast<int>(inliers.size()) < lidar_min_wall_points_) {
            return false;
        }

        Vec4f line;
        fitLine(inliers, line, DIST_L2, 0, 0.01, 0.01);

        double vx = line[0];
        double vy = line[1];
        const double x0 = line[2];
        const double y0 = line[3];

        // fitLine得到的方向存在正反二义性，统一令方向指向车头前方。
        if (vx < 0.0) {
            vx = -vx;
            vy = -vy;
        }

        estimate.heading = std::atan2(vy, vx);

        // 单位法向量(-vy, vx)指向车辆左侧，点积即左墙有符号距离。
        estimate.distance = -vy * x0 + vx * y0;
        estimate.point_count = static_cast<int>(inliers.size());

        return std::isfinite(estimate.heading) &&
               std::isfinite(estimate.distance) &&
               estimate.distance > 0.0;
    }

    bool runLidarWallFollowing() {
        bool filter_initialized = false;
        bool target_distance_initialized = false;
        bool control_command_initialized = false;
        bool front_distance_initialized = false;

        double filtered_heading = 0.0;
        double filtered_distance = 0.0;
        double target_left_distance = 0.0;
        double last_front_min_distance =
            std::numeric_limits<double>::infinity();
        double last_angular_command = 0.0;
        int last_wall_point_count = 0;

        int wall_invalid_count = 0;
        int front_invalid_count = 0;
        std::uint64_t last_processed_sequence = 0;

        ros::Rate control_rate(lidar_control_rate_);

        while (ros::ok()) {
            sensor_msgs::LaserScanConstPtr scan;
            ros::Time scan_receive_time;
            std::uint64_t scan_sequence = 0;

            {
                std::lock_guard<std::mutex> lock(scan_mutex_);
                scan = latest_scan_;
                scan_receive_time = latest_scan_receive_time_;
                scan_sequence = scan_sequence_;
            }

            if (!scan || scan_receive_time.isZero()) {
                ROS_ERROR_THROTTLE(1.0, "尚未收到雷达数据，保持停车");
                stopRobot();
                return false;
            }

            const double scan_age =
                (ros::Time::now() - scan_receive_time).toSec();
            if (scan_age > lidar_scan_stale_timeout_) {
                ROS_ERROR("雷达数据已过期 %.3f s > %.3f s，安全停车",
                          scan_age, lidar_scan_stale_timeout_);
                stopRobot();
                return false;
            }

            // 只有新LaserScan到来时才重新计算并累加无效帧次数。
            // 控制循环的其余周期继续发布上一条有效速度指令。
            if (scan_sequence != last_processed_sequence) {
                last_processed_sequence = scan_sequence;

                double front_min_distance =
                    std::numeric_limits<double>::infinity();
                if (getFrontMinDistance(*scan, front_min_distance)) {
                    front_invalid_count = 0;
                    front_distance_initialized = true;
                    last_front_min_distance = front_min_distance;

                    // 前方急停优先级最高，不等待左墙拟合结果。
                    if (front_min_distance <= lidar_front_stop_distance_) {
                        ROS_INFO("前方最近障碍 %.3f m <= %.3f m，立即停车",
                                 front_min_distance,
                                 lidar_front_stop_distance_);
                        stopRobot();
                        return true;
                    }
                } else {
                    ++front_invalid_count;
                    ROS_WARN("前方扇区雷达数据无效（%d/%d），短暂沿用上一帧结果",
                             front_invalid_count,
                             lidar_front_invalid_grace_scans_);

                    if (front_invalid_count >
                        lidar_front_invalid_grace_scans_) {
                        ROS_ERROR("前方雷达连续无效，安全停车");
                        stopRobot();
                        return false;
                    }
                }

                double left_min_distance = 0.0;
                WallEstimate wall;
                if (getLeftMinDistance(*scan, left_min_distance) &&
                    fitLeftWall(*scan, wall)) {
                    wall_invalid_count = 0;
                    last_wall_point_count = wall.point_count;

                    // 第一帧有效左侧雷达数据决定本次要保持的距离。
                    if (!target_distance_initialized) {
                        target_left_distance = left_min_distance;
                        target_distance_initialized = true;
                        ROS_INFO("锁存左侧雷达最小距离 %.3f m 作为本次目标距离",
                                 target_left_distance);
                    }

                    if (!filter_initialized) {
                        filtered_heading = wall.heading;
                        filtered_distance = left_min_distance;
                        filter_initialized = true;
                    } else {
                        filtered_heading =
                            lidar_filter_alpha_ * wall.heading +
                            (1.0 - lidar_filter_alpha_) * filtered_heading;
                        filtered_distance =
                            lidar_filter_alpha_ * left_min_distance +
                            (1.0 - lidar_filter_alpha_) * filtered_distance;
                    }

                    const double heading_error = filtered_heading;
                    const double distance_error =
                        filtered_distance - target_left_distance;

                    // 左墙向左张开或离左墙过远时，正角速度向左修正。
                    const double angular_command =
                        lidar_heading_kp_ * heading_error +
                        lidar_distance_kp_ * distance_error;

                    last_angular_command = clamp(
                        angular_command,
                        -lidar_max_angular_speed_,
                        lidar_max_angular_speed_
                    );
                    control_command_initialized = true;
                } else {
                    ++wall_invalid_count;
                    ROS_WARN("左侧最小距离无效或墙面拟合失败（%d/%d），短暂沿用上一角速度",
                             wall_invalid_count,
                             lidar_wall_invalid_grace_scans_);

                    if (wall_invalid_count >
                        lidar_wall_invalid_grace_scans_) {
                        ROS_ERROR("左墙连续拟合失败，安全停车");
                        stopRobot();
                        return false;
                    }
                }
            }

            // 第一条有效控制量出现前不盲目前进。永久订阅通常会使这里
            // 在进入雷达模式的首个周期就满足，不会再等待临时订阅。
            if (!front_distance_initialized ||
                !control_command_initialized) {
                stopRobot();
                control_rate.sleep();
                continue;
            }

            // 无论雷达发布频率是多少，均以固定控制频率重复发布最新有效指令，
            // 避免底盘因/cmd_vel超时出现“走一下、停一下”。
            twist_.linear.x = lidar_forward_speed_;
            twist_.linear.y = 0.0;
            twist_.angular.z = last_angular_command;
            cmd_pub_.publish(twist_);

            ROS_INFO_THROTTLE(
                0.5,
                "雷达连续跟随：前方=%.3f m，左侧最小值=%.3f m，目标=%.3f m，方向误差=%.2f°，角速度=%.3f，拟合点=%d，发布频率=%.1f Hz",
                last_front_min_distance,
                filtered_distance,
                target_left_distance,
                filtered_heading * 180.0 / 3.14159265358979323846,
                twist_.angular.z,
                last_wall_point_count,
                lidar_control_rate_
            );

            control_rate.sleep();
        }

        stopRobot();
        return false;
    }

    // 双边巡线逻辑
    void runDoubleLineTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        double line_error = double_find(gray_img, cropped);
        
        // PID计算
        integration_ += line_error * 0.03;
        integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
        double diff = line_error - pre_error_;
        diff = clamp(diff, -50.0, 50.0);
        
        // 速度控制
        twist_.linear.x = x_max_ / exp(abs(line_error) / 100.0);
        twist_.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
        pre_error_ = line_error;

        // 显示信息
        displayStream_ << "doubleerror: " << line_error 
                      << " P: " << line_error*p_ 
                      << " I: " << integration_*i_ 
                      << " D: " << diff*d_ 
                      << " 角速度: " << twist_.angular.z;
        putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        out_.write(cropped);
    }

    // 左点追踪逻辑
    void runLeftPointTracking(Mat& gray_img, Mat& cropped) {
        displayStream_.str("");
        if (!point_forward_) {
            // 丢线旋转
            ROS_INFO("丢线旋转中");
            // 此处保持原赛道“左点完成后转向”的原有速度；
            // out_forward/out_turn 仅用于右线丢失后的固定右转。
            twist_.linear.x = 0.075;
            twist_.angular.z = -1.0;
            out_.write(cropped);
            
            // 旋转到位后切换模式
            pose_client_.call(pose_);
            // ROS_INFO("角度%f,位姿%f",out_turn_angel_,pose_.response.pose_at[2]);
            if (pose_.response.pose_at[2] < out_turn_angel_) {
                left_point_start_ = false;
                double_line_ = true;
                x_max_ = 0.5;
                nh_.getParam("/line_right/double_P", p_);
                nh_.getParam("/line_right/double_I", i_);
                nh_.getParam("/line_right/double_D", d_);
                ROS_INFO("旋转完成，切换双边巡线 (P=%.2f)", p_);
            }
            return;
        }

        // 寻找左点并控制
        Point left_point;
        if (find_left_edge(gray_img, left_point, cropped)) {
            double error_x = 320 - left_point.x;
            pointx_integration_ += error_x * 0.02;
            pointx_integration_ = clamp(pointx_integration_, -1.0, 1.0);
            
            // 左点过低时停止前进
            if (left_point.y > 240) {
                point_forward_ = false;
            }

            // PID计算
            double point_diff = error_x - pointx_pre_error_;
            twist_.linear.x = 0.23;
            twist_.angular.z = error_x*leftpoint_p_ + pointx_integration_*leftpoint_I_ + point_diff*leftpoint_D_;
            pointx_pre_error_ = error_x;

            // 显示信息
            displayStream_ << "lefterror: " << error_x << " P: " << error_x*leftpoint_p_ << " I: " << pointx_integration_*leftpoint_I_;
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        }
        out_.write(cropped);
    }

    // 正常巡线逻辑
    bool runNormalTracking(Mat& gray_img, Mat& cropped, bool enable_lost_turn = true) {
        displayStream_.str("");
        vector<Point> start_points = find_track_edge(gray_img, 340, 70, cropped);
        RaceTrack racetrack;  // 现在RaceTrack已声明，可正常使用

        if (trace_edge(gray_img, start_points, racetrack, cropped)) {
            // 成功追踪到赛道
            trace_failed_count_ = 0;
            double line_error = error_calculater(racetrack.points, cropped);
            
            // PID计算
            integration_ += line_error * 0.03;
            integration_ = clamp(integration_, -abs(line_error)/integration_limit_ -1, abs(line_error)/integration_limit_ +1);
            double diff = line_error - pre_error_;
            diff = clamp(diff, -50.0, 50.0);
            
            // 速度控制
            twist_.linear.x = x_max_ / exp(abs(line_error) / 100.0);
            twist_.angular.z = clamp(line_error*p_ + integration_*i_ + diff*d_, -1.0, 1.0);
            pre_error_ = line_error;

            // 显示信息
            displayStream_ << "正常误差: " << line_error 
                          << " P: " << line_error*p_ 
                          << " I: " << integration_*i_ 
                          << " D: " << diff*d_ 
                          << " 角速度: " << twist_.angular.z;
            putText(cropped, displayStream_.str(), Point(50, 50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
            out_.write(cropped);
            return true;
        } else {
            if (!enable_lost_turn) {
                // 初始找线阶段视野内没有线是正常情况，不能累计丢线次数，
                // 也不能触发固定右转；始终以x_max_保持正向直行。
                trace_failed_count_ = 0;
                twist_.linear.x = x_max_;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;

                displayStream_ << "初始直行找线"
                               << " 线速度: " << twist_.linear.x;
                putText(cropped, displayStream_.str(), Point(50, 50),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
            } else {
                // 保留原来的连续5帧丢线容错，避免单帧识别波动引起误转。
                trace_failed_count_++;
                if (trace_failed_count_ > 5) {
                    // ROS约定 angular.z < 0 为顺时针旋转，也就是向右转。
                    // 使用 -abs() 后，YAML中的 out_turn 写正值或负值都能保证向右。
                    twist_.linear.x = out_forward_;
                    twist_.linear.y = 0.0;
                    twist_.angular.z = -std::abs(out_turn_);

                    if (trace_failed_count_ == 6) {
                        ROS_INFO("右线连续丢失，开始固定右转：线速度=%.3f，角速度=%.3f",
                                 twist_.linear.x, twist_.angular.z);
                    }

                    displayStream_ << "右线丢失，固定右转"
                                   << " 线速度: " << twist_.linear.x
                                   << " 角速度: " << twist_.angular.z;
                    putText(cropped, displayStream_.str(), Point(50, 50),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 165, 255), 1);
                }
            }
        }
        out_.write(cropped);
        return false;
    }

    // 工具函数：数值 clamping
    template <typename T>
    T clamp(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    // 图像处理：阈值化
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

    // 停车检测
    bool stop_car(Mat& gray, int& point, Mat& visual_img) {
        int white_count = 0;
        for (int y = 254; y >= 227; y--) {
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
                        right_found = false;
                        left_found = true;
                        center_x = cand_x2;
                        break;
                    }
                    if (right_check && gray_img.at<uchar>(center_y, cand_x) == 0 && gray_img.at<uchar>(center_y, cand_x + 1) == 255) {
                        right_found = true;
                        left_found = false;
                        center_x = cand_x + 1;
                        break;
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
                if (center_y <= 0 || racetracks[idx].points.size()>60) break;
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
                        if (last_up) racetracks[idx].direction_change++;
                        last_down = true;
                        last_up = false;
                        break;
                    }
                    if (!found && up_check && gray_img.at<uchar>(center_y - dy, center_x) == 255 && gray_img.at<uchar>(center_y - dy + 1, center_x) == 0) {
                        racetracks[idx].points.emplace_back(center_x + 1, center_y - dy);
                        found = true;
                        center_y -= dy;
                        if (last_down) racetracks[idx].direction_change++;
                        last_down = false;
                        last_up = true;
                        break;
                    }
                }

                if (found) {
                    fail_count = 0;
                    center_x++;
                } else {
                    fail_count++;
                    center_x++;
                    if (fail_count >= 10) { broken = true; break; }
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
        if (traced_points.empty()) {
            return 100.0;
        }

        double total_error = 0.0;

        for (size_t i = 0; i < traced_points.size(); i++) {
            double y = static_cast<double>(traced_points[i].y);

            // 今年二代车相机标定结果
            double center_offset =
                center_distance + (y - 140.0) * 1.40;

            // 根据右侧边线推算赛道中线
            double estimated_center_x =
                traced_points[i].x - center_offset;

            double weight;
            if (i <= 30) {
                weight = 1.0 - static_cast<double>(i) / 100.0;
            } else {
                weight = 0.7 *
                    exp(-0.064 * (static_cast<double>(i) - 30.0));
            }

            total_error +=
                (estimated_center_x - 320.0) * weight;
        }

        // 绘制推算出的中线
        for (const auto& point : traced_points) {
            double y = static_cast<double>(point.y);
            double center_offset =
                center_distance + (y - 140.0) * 1.40;

            Point center_point(
                cvRound(point.x - center_offset),
                point.y
            );

            circle(
                visualizeImg,
                center_point,
                3,
                Scalar(0, 255, 0),
                -1
            );
        }

        return -total_error / traced_points.size();
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "line_right");
    
    // 创建节点对象（构造函数中完成所有初始化）
    LineFollowerNode node;
    
    // 运行节点
    node.run();
    
    return 0;
}