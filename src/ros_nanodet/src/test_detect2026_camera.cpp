#include <clocale>
#include <cstddef>
#include <exception>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "ros_nanodet/ocr_result_srv.h"


class ContinuousDetectionViewer {
public:
    ContinuousDetectionViewer(
        ros::NodeHandle& node_handle,
        ros::NodeHandle& private_node_handle)
        : service_client_(
              node_handle.serviceClient<ros_nanodet::ocr_result_srv>(
                  "/nanodet_ocr")),
          camera_open_(false),
          show_image_(true) {
        private_node_handle.param("show_image", show_image_, true);
        if (show_image_) {
            image_subscriber_ = node_handle.subscribe(
                "/nanodet/debug_image",
                1,
                &ContinuousDetectionViewer::imageCallback,
                this);
        }
    }

    bool waitForDetectionService() {
        ROS_INFO("等待文字框检测与OCR服务 /nanodet_ocr...");
        if (!service_client_.waitForExistence(ros::Duration(15.0))) {
            ROS_ERROR("等待检测服务超时，请先启动 detect2026.launch");
            return false;
        }
        ROS_INFO("检测与OCR服务已经连接");
        return true;
    }

    bool openCamera() {
        ros_nanodet::ocr_result_srv service;
        service.request.command = -1;
        if (!service_client_.call(service) || !service.response.success) {
            ROS_ERROR("打开摄像头的服务请求失败");
            return false;
        }
        camera_open_ = true;
        ROS_INFO("已请求检测节点打开摄像头");
        return true;
    }

    bool requestDetection() {
        ros_nanodet::ocr_result_srv service;
        service.request.command = 1;
        const ros::WallTime start_time = ros::WallTime::now();

        if (!service_client_.call(service) || !service.response.success) {
            ROS_ERROR("文字框检测与OCR服务调用失败");
            return false;
        }

        const double elapsed = (ros::WallTime::now() - start_time).toSec();
        const std::size_t count = service.response.text.size();
        ROS_INFO_THROTTLE(
            1.0,
            "持续检测与OCR中：本帧结果=%zu，完整服务耗时=%.3f秒",
            count,
            elapsed);

        std::ostringstream result_stream;
        for (std::size_t index = 0; index < count; ++index) {
            if (index >= service.response.confidence.size() ||
                index >= service.response.detect_score.size() ||
                index >= service.response.x0.size() ||
                index >= service.response.y0.size() ||
                index >= service.response.x1.size() ||
                index >= service.response.y1.size()) {
                ROS_WARN("检测服务返回的数组长度不一致");
                break;
            }

            if (index != 0) {
                result_stream << "；";
            }
            result_stream
                << "框" << index
                << " 原始文字='" << service.response.text[index] << "'"
                << " OCR置信度=" << service.response.confidence[index]
                << " 检测置信度=" << service.response.detect_score[index]
                << " 坐标=(" << service.response.x0[index]
                << "," << service.response.y0[index]
                << ")-(" << service.response.x1[index]
                << "," << service.response.y1[index] << ")";
        }

        if (count > 0) {
            ROS_INFO_THROTTLE(
                1.0,
                "PP-OCR原始结果：%s",
                result_stream.str().c_str());
        }
        return true;
    }

    void closeCamera() {
        if (!camera_open_) {
            return;
        }

        ros_nanodet::ocr_result_srv service;
        service.request.command = -2;
        if (service_client_.call(service)) {
            ROS_INFO("摄像头已经释放");
        } else {
            ROS_WARN("释放摄像头的服务请求失败");
        }
        camera_open_ = false;
    }

    void run() {
        if (show_image_) {
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(window_name_, 960, 720);
            ROS_INFO("检测窗口已打开，按Q或ESC退出");
        } else {
            ROS_INFO("无窗口测速模式已启动，按Ctrl+C退出");
        }

        ros::Rate loop_rate(10.0);
        while (ros::ok()) {
            if (!requestDetection()) {
                break;
            }

            ros::spinOnce();
            if (show_image_) {
                const int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 'Q' || key == 27) {
                    ROS_INFO("收到退出按键");
                    break;
                }
            }
            loop_rate.sleep();
        }

        closeCamera();
        if (show_image_) {
            cv::destroyAllWindows();
        }
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& message) {
        if (!show_image_) {
            return;
        }
        try {
            const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(
                message,
                sensor_msgs::image_encodings::BGR8);
            cv::imshow(window_name_, image->image);
        } catch (const cv_bridge::Exception& error) {
            ROS_ERROR_THROTTLE(1.0, "调试图像转换失败：%s", error.what());
        }
    }

    const std::string window_name_ = "NanoDet + PP-OCR 2026 持续识别";
    ros::ServiceClient service_client_;
    ros::Subscriber image_subscriber_;
    bool camera_open_;
    bool show_image_;
};


int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "test_detect2026_camera");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    ContinuousDetectionViewer viewer(node_handle, private_node_handle);
    if (!viewer.waitForDetectionService()) {
        return 1;
    }
    if (!viewer.openCamera()) {
        return 1;
    }

    viewer.run();
    return 0;
}