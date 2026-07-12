#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        cout << "用法: " << argv[0] << " <图片路径>" << endl;
        return -1;
    }
    string pic_path = argv[1];
    Mat image = imread(pic_path);
    if (image.empty()) {
        cout << "Error: 图像未找到或路径错误！" << pic_path << endl;
        return -1;
    }

    // ========== 1. 粗定位白板（同方案1的ROI提取） ==========
    Mat hsv, v_channel, thresh;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    
    // 声明channels向量
    vector<Mat> channels;
    // 分离HSV通道
    split(hsv, channels);
    // 获取V通道（亮度通道）
    v_channel = channels[2].clone();
    
    threshold(v_channel, thresh, 180, 255, THRESH_BINARY);
    morphologyEx(thresh, thresh, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3,3)));

    vector<vector<Point>> contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return -1;
    auto max_cnt = max_element(contours.begin(), contours.end(), 
        [](const vector<Point>& a, const vector<Point>& b) {
            return contourArea(a) < contourArea(b);
        });
    Rect roi_rect = boundingRect(*max_cnt); // 白板的包围矩形
    Mat roi_gray;
    cvtColor(image(roi_rect), roi_gray, COLOR_BGR2GRAY); // ROI的灰度图

    // ========== 2. 优化角点检测参数 ==========
    GaussianBlur(roi_gray, roi_gray, Size(3, 3), 0); // 弱模糊，保留角点细节
    vector<Point2f> corners;
    int maxCorners = 4;
    double qualityLevel = 0.005; // 降低阈值（如0.001~0.01，越小检测越灵敏）
    double minDistance = 5;      // 减小角点间距（如5~10）
    int blockSize = 5;           // 增大邻域（如5~7，增强角点响应）

    goodFeaturesToTrack(roi_gray, corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize);

    // ========== 3. 转换坐标并绘制 ==========
    for (auto& p : corners) {
        p.x += roi_rect.x; // 转换为原图坐标
        p.y += roi_rect.y;
        circle(image, p, 5, Scalar(0, 0, 255), -1);
    }

    imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/find_corners.jpg", image);
    waitKey(0);
    return 0;
}