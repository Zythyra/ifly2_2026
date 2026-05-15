#include <ros/ros.h>
#include <qr_01/qr_code.h> 
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <string>
#include <signal.h>

cv::VideoCapture cap;
cv::VideoWriter video_out;
bool camera_active = false;

void sigintHandler(int sig) {
    if (cap.isOpened()) cap.release();
    if (video_out.isOpened()) video_out.release();
    ros::shutdown();
}

bool qr_detect_cb(qr_01::qr_code::Request& req, qr_01::qr_code::Response& resp) {
    // 状态机 - 使用全新的 req.command
    if (req.command == -1) {
        if (!camera_active) {
            cap.open(0, cv::CAP_V4L2);
            cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            camera_active = true;
            ROS_INFO("[QR Server] 摄像头已开启预热");
            if (!video_out.isOpened()) {
                video_out.open("/home/ucar/ucar_car/qr_debug/qr_debug.avi", 
                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0, cv::Size(640, 480));
            }
        }
        resp.result = ""; 
        return true;
    }
    
    if (req.command == -2) {
        if (camera_active) { cap.release(); camera_active = false; ROS_INFO("[QR Server] 摄像头已释放"); }
        resp.result = "";
        return true;
    }

    if (req.command == -3) {
        if (camera_active) { cap.grab(); cap.grab(); ROS_INFO("[QR Server] 缓存已清空"); }
        resp.result = "";
        return true;
    }

    // 识别流程
    if (!camera_active) {
        ROS_ERROR("摄像头未打开！");
        resp.result = "ERROR: CAMERA_CLOSED";
        return true; 
    }

    cv::Mat frame, gray;
    cap >> frame;
    if (frame.empty()) {
        resp.result = "ERROR: FRAME_EMPTY";
        return true;
    }

    cv::flip(frame, frame, 1);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
    int detected = scanner.scan(zbar_image);

    resp.result = ""; // 默认返回空字符串

    if (detected > 0) {
        for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
            symbol != zbar_image.symbol_end(); ++symbol) {
            
            // 直接把扫到的内容塞进字符串返回给主程序！
            resp.result = symbol->get_data(); 
            ROS_INFO_STREAM("成功获取二维码内容: " << resp.result);
            
            // 画框
            std::vector<cv::Point> pts;
            for (int i = 0; i < symbol->get_location_size(); i++) pts.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
            if (pts.size() == 4) for (int i = 0; i < 4; i++) cv::line(frame, pts[i], pts[(i+1)%4], cv::Scalar(0, 255, 0), 3);
            break; 
        }
    }

    if (video_out.isOpened()) video_out.write(frame);
    return true;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "qr_server_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigintHandler);
    ros::NodeHandle nh;
    
    // 注册服务
    ros::ServiceServer service = nh.advertiseService("qr_detect", qr_detect_cb);
    ROS_INFO("二维码服务端启动完毕，等待调用...");
    ros::spin();
    return 0;
}