#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<geometry_msgs/Twist.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/imgproc/types_c.h>
#include<opencv2/core/core.hpp>
#include<nav_msgs/Odometry.h>
#include<std_msgs/UInt16.h>

sensor_msgs::Image hsv_image;						//s
std_msgs::UInt16 flag;
int xflag;

void image_Callback(const sensor_msgs::Image& msg);

int main(int argc, char **argv){

    ros::init(argc, argv, "to_find_zhang");			// init note
    ros::NodeHandle nh;
    ros::param::set("/xflag",0);
    ros::param::set("/zhang",0);
    ros::Subscriber img_sub = nh.subscribe("/ucar_camera/image_raw", 10, image_Callback); 	// 更改为订阅 /usb_cam/image_raw 订阅者img_sub来接收来自USB摄像头的原始图像,and image_Callback
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/image_hsv",10);
    ros::Publisher zhang_pub = nh.advertise<std_msgs::UInt16>("/zhang",10);


    while(ros::ok()){
        ros::param::get("/xflag",xflag);
        if(xflag == 1){
            img_pub.publish(hsv_image);
            zhang_pub.publish(flag);
            ros::spinOnce();
        }
    }
    return 0;
}

void image_Callback(const sensor_msgs::Image& msg){// 当从摄像头接收到图像时，函数触发, public speed cmd

    cv_bridge::CvImagePtr cv_ptr;							


    // 确保使用正确的图像编码
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);		// 使用cv_bridge将ROS的图像消息转换为OpenCV的图像格式
    } 

    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    

    cv::Mat image = cv_ptr->image;			// 原始图像
    cv::Mat hsv = image.clone();			// 用于后续的HSV转换
    cv::Mat res = image.clone();			// 用于存储颜色过滤后的结果 (keep medium)

    // cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);	// 颜色空间转换
    // cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 46), res);		// 颜色过滤 -> res
    // cv::inRange(hsv, cv::Scalar(0, 0, 46), cv::Scalar(180, 43, 220), res);		// 颜色过滤 -> res
    // cv::inRange(hsv, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), res);		// 颜色过滤 -> res


    // 处理逻辑
				// origin image
    int h = image.rows;   //720
    int w = image.cols;   //1280
	// ROS_INFO("h: %d w: %d", h, w);
				// search window
    int search_top = 5 * h / 6-40;
    int	search_bot = search_top + 20;

    cv::Mat gray, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Canny(gray, edges, 50, 150);
    for(int i = 0; i < search_top-100; i ++){
        for(int j = 0; j < w; j ++){

            edges.at<uchar>(i,j) = 0;			// set = 0 ,if not in search window
        }
    }

    for(int i = search_bot; i < h; i++){
        for(int j = 0; j < w; j ++){

            edges.at<uchar>(i,j) = 0;			// set = 0 ,if not in search window
        }
    }
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 80, 50, 10);  // 参数可调

    bool found_obstacle = false;

    for (const auto& l : lines) {
        int x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];
        double dx = x2 - x1;
        double dy = y2 - y1;
        double angle = std::atan2(dy, dx) * 180 / CV_PI;
        double length = std::sqrt(dx*dx + dy*dy);
        // ROS_INFO("Line: length=%.1f angle=%.1f y1=%d y2=%d", length, angle, y1, y2);
        // ROS_INFO("%d %d",x1,x2);
        int midx = (x1+x2)/2;
        if (std::abs(angle) < 5.0 && length > 170 && length < 270 && 880 > midx && midx > 400 &&
            // std::max(y1, y2) > edges.rows + 40 &&
            std::max(y1, y2) < image.rows - 5)  // ✅ 排除最底线
        {
            found_obstacle = true;
            cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            ROS_INFO("Detected horizontal line: angle=%.1f length=%.1f,x=%d", angle, length,(x1+x2)/2);
            cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test_edges.jpg", edges);
            break;
        }
    }
    flag.data = found_obstacle ? 1 : 0;
    if(found_obstacle)
    {
        ros::param::set("/zhang",1);
        ros::shutdown();
    }
}
