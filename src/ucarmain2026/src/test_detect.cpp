/**
 * @file test_detect_single_class_imshow.cpp
 * @brief NanoDet-Plus 2026 单类别 target 检测测试节点
 *
 * 当前 detect2026.py 接口：
 *   detect_start = -1 : 打开摄像头
 *   detect_start = -2 : 释放摄像头
 *   detect_start = -3 : 丢弃两帧缓存
 *   detect_start =  4 : 低阈值检测
 *   其他值           : 正常检测
 *
 * 检测模型为单类别：
 *   class_name = 0 -> target
 *
 * 图像显示：
 *   detect2026.py 在 enable_debug_image=true 时，
 *   会把“本次检测的同一帧 + 已画好的检测框”发布到：
 *      /nanodet/debug_image
 *
 * 本节点订阅该话题后用 cv::imshow() 显示。
 */

#include <clocale>
#include <cstddef>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <ros_nanodet/detect_result_srv.h>


class SingleClassDetectViewer {
public:
    SingleClassDetectViewer(ros::NodeHandle& nh,
                            ros::NodeHandle& pnh)
        : detect_client_(
              nh.serviceClient<ros_nanodet::detect_result_srv>(
                  "/nanodet_detect")),
          camera_open_(false),
          image_received_(false) {
        pnh.param("detect_command", detect_command_, 1);
        pnh.param("loop_hz", loop_hz_, 10.0);
        pnh.param<std::string>(
            "debug_image_topic",
            debug_image_topic_,
            std::string("/nanodet/debug_image"));

        if (loop_hz_ <= 0.0) {
            loop_hz_ = 10.0;
        }

        image_sub_ = nh.subscribe(
            debug_image_topic_,
            1,
            &SingleClassDetectViewer::imageCallback,
            this);
    }

    ~SingleClassDetectViewer() {
        closeCamera();
        cv::destroyAllWindows();
    }

    bool waitForService() {
        ROS_INFO("========== NanoDet 2026 单类检测测试 ==========");
        ROS_INFO("等待检测服务 /nanodet_detect ...");

        if (!detect_client_.waitForExistence(ros::Duration(15.0))) {
            ROS_ERROR(
                "等待 /nanodet_detect 超时，请先启动 detect2026.py");
            return false;
        }

        ROS_INFO("检测服务已连接。当前模型类别固定为 target(ID=0)");
        ROS_INFO(
            "调试画面话题：%s",
            debug_image_topic_.c_str());
        return true;
    }

    bool openCamera() {
        ros_nanodet::detect_result_srv srv;
        srv.request.detect_start = -1;

        if (!detect_client_.call(srv)) {
            ROS_ERROR("发送打开摄像头指令(-1)失败");
            return false;
        }

        camera_open_ = true;
        ROS_INFO("已请求 detect2026.py 打开摄像头");

        // 当前服务端已经把 V4L2 缓冲设为1。
        // 再主动执行一次 -3，保证测试第一帧尽量新鲜。
        ros_nanodet::detect_result_srv clear_srv;
        clear_srv.request.detect_start = -3;
        if (!detect_client_.call(clear_srv)) {
            ROS_WARN("首次清理摄像头缓存(-3)失败，继续测试");
        } else {
            ROS_INFO("已清理摄像头缓存");
        }

        return true;
    }

    void closeCamera() {
        if (!camera_open_) {
            return;
        }

        ros_nanodet::detect_result_srv srv;
        srv.request.detect_start = -2;

        if (detect_client_.call(srv)) {
            ROS_INFO("摄像头已释放");
        } else {
            ROS_WARN("释放摄像头指令(-2)调用失败");
        }

        camera_open_ = false;
    }

    bool requestDetection() {
        ros_nanodet::detect_result_srv srv;
        srv.request.detect_start = detect_command_;

        const ros::WallTime begin = ros::WallTime::now();

        if (!detect_client_.call(srv)) {
            ROS_WARN_THROTTLE(
                1.0,
                "调用 /nanodet_detect 失败");
            return false;
        }

        const double elapsed =
            (ros::WallTime::now() - begin).toSec();

        // 当前单类接口中，x0/y0/x1/y1同一下标共同构成一个框。
        const std::size_t count = std::min(
            std::min(
                srv.response.x0.size(),
                srv.response.y0.size()),
            std::min(
                srv.response.x1.size(),
                srv.response.y1.size()));

        if (count == 0) {
            ROS_INFO_THROTTLE(
                1.0,
                "本帧未检测到 target，检测耗时=%.3fs",
                elapsed);
            return true;
        }

        ROS_INFO_THROTTLE(
            0.5,
            "本帧检测到 %zu 个 target，检测耗时=%.3fs",
            count,
            elapsed);

        for (std::size_t i = 0; i < count; ++i) {
            const int x0 = srv.response.x0[i];
            const int y0 = srv.response.y0[i];
            const int x1 = srv.response.x1[i];
            const int y1 = srv.response.y1[i];

            const double center_x =
                0.5 * static_cast<double>(x0 + x1);
            const double center_y =
                0.5 * static_cast<double>(y0 + y1);

            // 当前模型是单类，即使服务端class_name数组缺失，
            // 也按target处理；若存在则正常应固定为0。
            int class_id = 0;
            if (i < srv.response.class_name.size()) {
                class_id = srv.response.class_name[i];
            }

            ROS_INFO(
                "target[%zu]：class_id=%d，"
                "框=(%d,%d)-(%d,%d)，"
                "center=(%.1f,%.1f)，"
                "w=%d，h=%d",
                i,
                class_id,
                x0, y0,
                x1, y1,
                center_x, center_y,
                x1 - x0,
                y1 - y0);
        }

        return true;
    }

    void run() {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, 960, 720);

        ROS_INFO(
            "检测窗口已创建。按 Q 或 ESC 退出；Ctrl+C 也可退出。");
        ROS_INFO(
            "注意：必须以 enable_debug_image:=true 启动 detect2026.py，"
            "否则服务检测正常但窗口收不到画面。");

        ros::Rate rate(loop_hz_);

        while (ros::ok()) {
            if (!requestDetection()) {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // 服务端在本次检测结束时发布 /nanodet/debug_image。
            // spinOnce后回调拿到的就是本次推理对应的已框选画面。
            ros::spinOnce();

            if (!image_received_) {
                ROS_WARN_THROTTLE(
                    2.0,
                    "尚未收到 %s。请确认 detect2026.launch "
                    "的 enable_debug_image=true",
                    debug_image_topic_.c_str());
            }

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q' || key == 27) {
                ROS_INFO("收到退出按键");
                break;
            }

            rate.sleep();
        }

        closeCamera();
        cv::destroyAllWindows();
        ROS_INFO("========== 单类检测测试结束 ==========");
    }

private:
    void imageCallback(
            const sensor_msgs::ImageConstPtr& msg) {
        try {
            const cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::BGR8);

            // detect2026.py 发布的是：
            // 当前检测的同一帧 + 服务端已画好的 target 检测框。
            cv::Mat display = cv_ptr->image.clone();

            // 额外画出640宽图像的中心参考线，方便现场观察框中心。
            // 不参与检测，只用于调试。
            if (!display.empty()) {
                const int center_x = display.cols / 2;
                cv::line(
                    display,
                    cv::Point(center_x, 0),
                    cv::Point(center_x, display.rows - 1),
                    cv::Scalar(255, 255, 255),
                    1,
                    cv::LINE_AA);
            }

            cv::imshow(window_name_, display);
            image_received_ = true;
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_THROTTLE(
                1.0,
                "调试图像转换失败：%s",
                e.what());
        }
    }

private:
    const std::string window_name_ =
        "NanoDet 2026 Single-Class Target";

    ros::ServiceClient detect_client_;
    ros::Subscriber image_sub_;

    bool camera_open_;
    bool image_received_;

    int detect_command_;
    double loop_hz_;
    std::string debug_image_topic_;
};


int main(int argc, char** argv) {
    setlocale(LC_ALL, "");

    ros::init(
        argc,
        argv,
        "test_detect");
        

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    SingleClassDetectViewer viewer(nh, pnh);

    if (!viewer.waitForService()) {
        return 1;
    }

    if (!viewer.openCamera()) {
        return 1;
    }

    viewer.run();
    return 0;
}