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
double Kp,Ki,Kd;
double error=0;
double integral = 0.0;
double previous_error = 0.0;
int miss=0,mflag=1;
int lu=0;
void image_Callback(const sensor_msgs::Image& msg);



int main(int argc, char **argv){

    ros::init(argc, argv, "xian");			// init note
    ros::NodeHandle nh;
    ros::param::set("/lor",0);
    while(lu==0)
        ros::param::get("/lor",lu);
    ros::Subscriber img_sub = nh.subscribe("/ucar_camera/image_raw", 10, image_Callback); 	// 更改为订阅 /usb_cam/image_raw 订阅者img_sub来接收来自USB摄像头的原始图像,and image_Callback
    ros::Publisher pid_pub = nh.advertise<geometry_msgs::Twist>("/pidl",10);	
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/image_hsv",10);

    while(ros::ok()){
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        twist.angular.x = mflag;
        img_pub.publish(hsv_image);
        pid_pub.publish(twist);
        ros::spinOnce();
    }
    return 0;
}

double pidControl(double error) {
    double P = Kp * error;
    integral += error ;
    double I = Ki * integral;
    double derivative = (error - previous_error) ;
    double D = Kd * derivative;
    double output = P + I + D;
    previous_error = error;
    return output;
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

    cv::cvtColor(image, image, cv::COLOR_RGB2GRAY); //转换灰度图
    // cv::threshold(image, res, 180, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

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
    int search_top = 5 * h / 6-15;
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
    ros::Rate rate(30);
    if(M.m00 > 0){
        // int cx = int (cvRound(M.m10 / M.m00+w/3+50));
        int cx = int (cvRound(M.m10 / M.m00+545+40));
        int cy = int (cvRound(M.m01 / M.m00));

	// center in image
        ROS_INFO("cx: %d cy: %d", cx, cy);
        cv::circle(image, cv::Point(cx, cy), 10, (255, 255, 255));
        cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test.jpg", image);
	// set speed 
	// 假设摄像头是再中间的

        if(miss>=3 & mflag==1){
            twist_linear_x = 0;
            mflag=0;
            for(int i=0;i<15;i++){
                error = (cx - w / 2) * 0.03; 
                twist_angular_z = -pidControl(error);
                rate.sleep();
            }
            twist_linear_x = 0.35;
            miss=0;
        }

        Kp=0.1;                                                        //p=0.10,i=0.01,d=0.02,x=0.45，top+20,z=0.8
        Ki=0.0;                                                        // p=0.13,i=0.01,d=0.022,x=0.45，top-15,z=1.5
        Kd=0.6;                                                       //p=0.15,d=0.022,x=0.35
        error = (cx - w / 2) * 0.03;                                   //p=0.12,d=0.5。x=0.18,z=-1.0
        twist_linear_x = 0.35;
        twist_angular_z = -pidControl(error);
        // ROS_INFO("%f",twist_angular_z);
        miss=0;
        //cmd_pub.publish(twist);
    } 
    else{
        ROS_INFO("not found line!");
        miss+=1;
        twist_linear_x = 0.28;   //z=-1.0,x=0.3
        twist_angular_z = 1.2; //z=-1.5,x=0.25
        if(miss>=3){
            previous_error = 0.0;
            integral = 0.0;
            ROS_INFO("turn 0");
        }
        //cmd_pub.publish(twist);
    }

    // line's center,in image 
    sensor_msgs::ImagePtr hsv_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    hsv_image = *hsv_image_;
    // cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test.jpg", image);
}