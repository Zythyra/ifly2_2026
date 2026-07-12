#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>

// 相机内参矩阵（从 YAML 文件获取）
cv::Mat K = (cv::Mat_<double>(3,3) <<
    612.7148, 0.0, 616.11004,
    0.0, 611.02227, 346.95483,
    0.0, 0.0, 1.0
);

cv::Mat distCoeffs = (cv::Mat_<double>(1,5) <<
    -0.262284, 0.048112, 0.000962, 0.003192, 0.0
);

// 假设相机到地面的高度为 15 cm，俯视角为  度
float camera_height = 0.15f;  // 相机高度 15 cm
float pitch_deg = 0.0f;      // 相机俯视角 0

cv::Point2f pixelToGround(int u, int v, const cv::Mat& K, const cv::Mat& distCoeffs, float cameraHeight, float pitchDeg) {
    // Step 1: 畸变校正
    std::vector<cv::Point2f> pt(1);
    pt[0] = cv::Point2f(u, v);
    std::vector<cv::Point2f> undistorted_pt;
    cv::undistortPoints(pt, undistorted_pt, K, distCoeffs);

    // Step 2: 像素归一化（归一化成光线方向）
    float x = undistorted_pt[0].x;
    float y = undistorted_pt[0].y;
    float z = 1.0f;

    // Step 3: 光线在相机坐标系下的单位向量
    Eigen::Vector3f ray_cam(x, y, z);
    ray_cam.normalize();

    // Step 4: 构造旋转矩阵（俯视角）
    float pitchRad = pitchDeg * M_PI / 180.0;
    // Eigen::Matrix3f R = Eigen::AngleAxisf(-pitchRad, Eigen::Vector3f::UnitX()); // 绕X轴旋转
    Eigen::AngleAxisf angleAxis(-pitchRad, Eigen::Vector3f::UnitX());  // 绕X轴旋转
    Eigen::Matrix3f R = angleAxis.toRotationMatrix();  // 转换为旋转矩阵

    // Step 5: 将视线旋转到机器人坐标系中
    Eigen::Vector3f ray_robot = R * ray_cam;

    // Step 6: 射线与地面 Z = 0 相交
    float t = cameraHeight / ray_robot.z();  // ray_robot.z 应该为正数
    Eigen::Vector3f ground_point = t * ray_robot;

    return cv::Point2f(ground_point.x(), ground_point.y());  // Z=0
}

// 处理相机图像，提取白板的角点并进行坐标转换
void imageCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    // 读取图像文件并转换为灰度图
    cv::Mat image = cv::imread("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/62.jpg");  
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // 假设你的白板是棋盘格，设置棋盘格的行列数
    cv::Size pattern_size(6, 9); // 棋盘格的行列数，设置为实际尺寸
    std::vector<cv::Point2f> corners;

    // 查找棋盘格角点
    bool found = cv::findChessboardCorners(gray, pattern_size, corners);
    if (found) {
        // 如果找到了角点，则进行畸变校正
        std::vector<cv::Point2f> undistorted_corners;
        cv::undistortPoints(corners, undistorted_corners, K, distCoeffs);

        // 选择底部两个角点（假设第一个和最后一个角点为底部角点）
        int u1 = undistorted_corners[0].x, v1 = undistorted_corners[0].y;
        int u2 = undistorted_corners[pattern_size.width - 1].x, v2 = undistorted_corners[pattern_size.width - 1].y;

        // 输出底部角点在地面上的坐标
        cv::Point2f ground1 = pixelToGround(u1, v1, K, distCoeffs, camera_height, pitch_deg);
        cv::Point2f ground2 = pixelToGround(u2, v2, K, distCoeffs, camera_height, pitch_deg);

        ROS_INFO("Ground Point 1 in Robot frame: (%.3f, %.3f)", ground1.x, ground1.y);
        ROS_INFO("Ground Point 2 in Robot frame: (%.3f, %.3f)", ground2.x, ground2.y);
    } else {
        ROS_WARN("Chessboard not detected!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixel_to_ground_node");
    ros::NodeHandle nh;

    // 订阅相机信息
    ros::Subscriber sub = nh.subscribe("/head_camera/camera_info", 1, imageCallback);

    ros::spin();
    return 0;
}
