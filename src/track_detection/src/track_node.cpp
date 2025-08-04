#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "track_detection/track_detector.h"

namespace track_detection {

class TrackNode {
private:
    // 修正1：类成员变量声明时不能直接初始化带参数的构造函数，需在初始化列表中初始化
    ros::NodeHandle nh_;  // 私有句柄，变量名统一用nh_（末尾加下划线区分成员变量）
    image_transport::ImageTransport it_;
    image_transport::Publisher result_pub_;
    TrackDetector detector_;
    cv::VideoCapture cap_;
    std::string camera_topic_;

public:
    // 修正2：在初始化列表中初始化所有需要参数的成员变量
    TrackNode() : nh_("~"), it_(nh_), detector_(nh_) {
        // 读取摄像头话题参数
        nh_.param<std::string>("camera_topic", camera_topic_, "");
        
        // 初始化摄像头或订阅图像话题
        if (camera_topic_.empty()) {
            if (!initCamera()) {
                ROS_FATAL("无法初始化摄像头！");
                ros::shutdown();
                return;
            }
        } else {
            image_transport::Subscriber img_sub = it_.subscribe(
                camera_topic_, 1, &TrackNode::imageCallback, this);
            ROS_INFO_STREAM("已订阅图像话题: " << camera_topic_);
        }
        
        // 发布检测结果
        result_pub_ = it_.advertise("/track_detection/result", 1);
        ROS_INFO("赛道检测节点已启动");
        
        // 处理摄像头图像
        if (camera_topic_.empty() && cap_.isOpened()) {
            processCamera();
        }
    }

    // 初始化摄像头设备
    bool initCamera() {
        for (int i = 0; i < 2; ++i) {
            if (cap_.open(i, cv::CAP_V4L2)) {
                ROS_INFO_STREAM("成功打开摄像头: /dev/video" << i);
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                cap_.set(cv::CAP_PROP_FPS, 30);
                return true;
            }
        }
        
        ROS_ERROR("无法打开任何摄像头设备，请检查连接或权限");
        return false;
    }

    // 处理摄像头图像（直接读取设备时使用）
    void processCamera() {
        cv::Mat frame;
        ros::Rate rate(30);
        
        while (nh_.ok()) {  // 使用修正后的nh_
            cap_ >> frame;
            if (frame.empty()) {
                ROS_WARN("无法获取图像帧，重试...");
                rate.sleep();
                continue;
            }
            
            // 检测赛道并发布结果
            cv::Mat result = detector_.detect(frame);
            publishResult(result);
            
            rate.sleep();
        }
        cap_.release();
    }

    // 图像回调函数（订阅话题时使用）
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat result = detector_.detect(frame);
            publishResult(result);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge转换错误: %s", e.what());
        }
    }

    // 发布检测结果
    void publishResult(const cv::Mat& result) {
        if (result.empty()) return;
        
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "camera";
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = result;
        result_pub_.publish(out_msg.toImageMsg());
    }
};

} // namespace track_detection

int main(int argc, char**argv) {
    ros::init(argc, argv, "track_detection_node");
    track_detection::TrackNode node;
    ros::spin();
    return 0;
}
    