#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<geometry_msgs/Twist.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/imgproc/types_c.h>
#include<opencv2/core/core.hpp>

double twist_linear_x , twist_angular_z;				// two kinds speed


sensor_msgs::Image hsv_image;						//s

float dynamic_angular_gain;

void image_Callback(const sensor_msgs::Image& msg);



int main(int argc, char **argv){

    ros::init(argc, argv, "follower_line");			// init note
    ros::NodeHandle nh;

    ros::Subscriber img_sub = nh.subscribe("/ucar_camera/image_raw", 10, image_Callback); 	// 更改为订阅 /usb_cam/image_raw 订阅者img_sub来接收来自USB摄像头的原始图像,and image_Callback

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);	// 分别用于发布	小车的速度指令	和	处理后的图像。		
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/image_hsv",10);

    while(ros::ok()){
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        cmd_pub.publish(twist);
        img_pub.publish(hsv_image);
        ros::spinOnce();
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
    cv::Size kernelSize(5, 5);

    // cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);	// 颜色空间转换
    // cv::inRange(hsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), res);		// 颜色过滤 -> res
    // cv::inRange(hsv, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), res);		// 颜色过滤 -> res
    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY); //转换灰度图
    // cv::threshold(image, res, 180, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    // show
    
    cv::imshow("Filtered Image", res);  // 显示过滤后的图像
    cv::waitKey(1);  // 等待1毫秒以更新窗口
    cv::GaussianBlur(image,image,kernelSize,0);


    cv::Mat sobelX, sobelY;
    cv::Sobel(image, sobelX, CV_16S, 1, 0);
    cv::Sobel(image, sobelY, CV_16S, 0, 1);


    // 转换为绝对值
    cv::Mat sobelXAbs, sobelYAbs;
    cv::convertScaleAbs(sobelX, sobelXAbs);
    cv::convertScaleAbs(sobelY, sobelYAbs);

    // 整幅图的一阶边缘

    res = sobelXAbs + sobelYAbs;
    cv::threshold(res, res, 180, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::medianBlur(res,res,5);
    cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test2.jpg", res);
    // 处理逻辑
				// origin image
    int h = image.rows;   //720
    int w = image.cols;   //1280
	// ROS_INFO("h: %d w: %d", h, w);
				// search window
    int search_top = 5 * h / 6+20;
    int	search_bot = search_top + 30;


    cv::blur(res,res,kernelSize);
    for(int i = 0; i < search_top; i ++){
        for(int j = 0; j < w; j ++){

            res.at<uchar>(i,j) = 0;			// set = 0 ,if not in search window
        }
    }

    for(int i = search_top; i < search_bot; i ++){
        for(int j = w/2; j < w; j ++){

            res.at<uchar>(i,j) = 0;			// set = 0 ,if not in search window
        }
    }

    for(int i = search_bot; i < h; i++){
        for(int j = 0; j < w; j ++){

            res.at<uchar>(i,j) = 0;			// set = 0 ,if not in search window
        }
    }
    
    cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test1.jpg", res);
    cv::Moments M = cv::moments(res);			// 图像矩


    if(M.m00 > 0){

        // int cx = int (cvRound(M.m10 / M.m00+w/3+50));
        int cx = int (cvRound(M.m10 / M.m00+545-40));
        int cy = int (cvRound(M.m01 / M.m00));

	// center in image
        ROS_INFO("cx: %d cy: %d", cx, cy);
        cv::circle(image, cv::Point(cx, cy), 10, (255, 255, 255));
        cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test.jpg", image);
	// set speed 
	// 假设摄像头是再中间的

        int v = cx - w / 2;
        twist_linear_x = 0.25;
        // if(cx>700){
        //     twist_linear_x =0.05;
        //     ROS_INFO("W");
        // }
        dynamic_angular_gain = (300 / 0.5) / (twist_linear_x / 0.1 );
        twist_angular_z = -float(v) / dynamic_angular_gain;
        ROS_INFO("%f",twist_angular_z);
        //cmd_pub.publish(twist);
    } 
    else{
        ROS_INFO("not found line!");
        twist_linear_x = 0.15;                                                                   //notline:x=0.15,z=0.4;xx:x=0.15,z=300/0.5,cx=w/3+120
        twist_angular_z = 0.8;
        //cmd_pub.publish(twist);
    }

    // line's center,in image 
    sensor_msgs::ImagePtr hsv_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    hsv_image = *hsv_image_;
    // cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test.jpg", image);
}