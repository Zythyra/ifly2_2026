#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

// 局部大津法函数
void localOtsu(cv::Mat& gray, cv::Mat& binary, int blockSize) {
    cv::Mat padded;
    int padX = (blockSize - gray.cols % blockSize) % blockSize;
    int padY = (blockSize - gray.rows % blockSize) % blockSize;
    
    // 添加边缘填充确保可分块
    cv::copyMakeBorder(gray, padded, 0, padY, 0, padX, 
                       cv::BORDER_CONSTANT, cv::Scalar(0));
    
    binary = cv::Mat::zeros(padded.size(), CV_8UC1);
    
    // 分块处理
    for(int y = 0; y < padded.rows; y += blockSize) {
        for(int x = 0; x < padded.cols; x += blockSize) {
            cv::Mat tile = padded(cv::Rect(x, y, blockSize, blockSize));
            double thresh = cv::threshold(tile, tile, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);
        }
    }
    
    // 转换为二值图
    padded.convertTo(binary, CV_8UC1);
    binary = binary(cv::Rect(0, 0, gray.cols, gray.rows));
}

int main() {
    cv::VideoCapture cap(0); // 打开摄像头
    if(!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }
    
    // 设置分辨率640x480
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    int pooling_ratio = 1;     // 初始池化比例
    int adaptive_block = 23;    // 自适应邻域大小（基数）
    int adaptive_c = -15;       // 阈值偏移量（关键参数）
    int morph_size = 0;      // 形态学操作核大小
    int min_contour_area = 60; // 最小轮廓面积阈值
    

    
    cv::namedWindow("Result", cv::WINDOW_AUTOSIZE);
    
    while(true) {
        auto start = std::chrono::high_resolution_clock::now(); // 开始计时
        
        cv::Mat frame;
        cap >> frame; // 捕获帧
        if(frame.empty()) break;
        
        // 截取底部640x270区域
        cv::Mat bottom = frame(cv::Rect(0, frame.rows - 270, 640, 270));
        
        // 分割RGB获取R通道
        std::vector<cv::Mat> channels;
        cv::split(bottom, channels);
        cv::Mat r_channel = channels[2];
        
        // 池化处理降低分辨率
        cv::Mat pooled;
        if(pooling_ratio > 1) {
            cv::resize(r_channel, pooled, 
                       cv::Size(r_channel.cols / pooling_ratio, 
                                r_channel.rows / pooling_ratio), 
                       0, 0, cv::INTER_NEAREST);
        } else {
            pooled = r_channel.clone();
        }
        
        // 应用局部大津法
        cv::Mat binary;
        int actualBlockSize = std::max(3, adaptive_block); // 确保最小值
        if(actualBlockSize % 2 == 0) actualBlockSize++;     // 强制奇数
        cv::adaptiveThreshold(
            pooled, binary, 255, 
            cv::ADAPTIVE_THRESH_MEAN_C, // 使用局部均值
            cv::THRESH_BINARY,           // 二值化类型
            actualBlockSize,             // 邻域大小（必须奇数）
            adaptive_c                   // 关键偏移量
        );
        // === 新增：形态学开运算（先腐蚀后膨胀）去噪 ===
        cv::Mat denoised = cv::Mat::zeros(binary.size(), CV_8UC1);
        if(morph_size > 0) {
            // 创建核（奇数尺寸）
            int size = 2 * morph_size + 1;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size));
            
            // 先腐蚀后膨胀（开运算）
            cv::morphologyEx(binary, denoised, cv::MORPH_OPEN, kernel);
        } 
        // else {
        //     denoised = binary.clone();
        // }

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 过滤小面积轮廓（可能是噪声）
        for(size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if(area > min_contour_area) {
                cv::drawContours(denoised, contours, static_cast<int>(i), 
                                cv::Scalar(255), cv::FILLED);
            }
        }
        
        // 恢复原始显示尺寸
        cv::Mat display;
        cv::resize(denoised, display, cv::Size(640, 270), 0, 0, cv::INTER_NEAREST);
        
        cv::Mat color_display;
        cv::cvtColor(display, color_display, cv::COLOR_GRAY2BGR);

        // 计算处理时间
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        double frame_time = duration.count() / 1000.0;
        
        // 显示帧处理时间
        std::string timeText = "Time: " + std::to_string(frame_time) + "ms";
        cv::putText(color_display, timeText, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
        
        // 显示分辨率信息
        // std::string resText = "Resolution: " + std::to_string(pooled.cols) + 
        //                      "x" + std::to_string(pooled.rows);
        // cv::putText(color_display, resText, cv::Point(10, 60), 
        //            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
        
        // 显示池化比例
        // std::string poolText = "Pooling: 1/" + std::to_string(pooling_ratio);
        // cv::putText(color_display, poolText, cv::Point(10, 90), 
        //            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
        
        cv::imshow("Result", color_display);
        
        // 按键处理
        int key = cv::waitKey(1);
        if(key == 'q') break;
        if(key == '+' && pooling_ratio < 8) {
            pooling_ratio *= 2; // 增大池化比例
            std::cout << "池化比例变为"<<pooling_ratio << std::endl;
        }
        if(key == '-' && pooling_ratio > 1){
            pooling_ratio /= 2; // 减小池化比例
            std::cout << "池化比例变为"<< pooling_ratio << std::endl;
        }
        if(key == 'a') {
            adaptive_c -= 1;
            std::cout << "二值化偏移常数为"<< adaptive_c << std::endl;
        }
        if(key == 'd') {
            adaptive_c += 1;
            std::cout << "二值化偏移常数为"<< adaptive_c << std::endl;
        }
        if(key == 'w') {
            morph_size += 1;
            std::cout << "膨胀和腐蚀核大小为"<< morph_size << std::endl;
        }
        if(key == 's' && morph_size>0){
            morph_size -= 1;
            std::cout << "膨胀和腐蚀核大小为"<< morph_size << std::endl;
        } 
        if(key == '0' && adaptive_block>3){
            adaptive_block -= 2;
            std::cout << "二值化核大小为"<< adaptive_block << std::endl;
        }
        if(key == '1'){
            adaptive_block += 2;
            std::cout << "二值化核大小为"<< adaptive_block << std::endl;
        }
        if(key == '2' && min_contour_area>1){
            min_contour_area -= 1;
            std::cout << "最小轮廓面积为"<< min_contour_area << std::endl;
        }
        if(key == '3'){
            min_contour_area += 1;
            std::cout << "最小轮廓面积为"<< min_contour_area << std::endl;
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}