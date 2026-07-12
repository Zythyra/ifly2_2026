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

double twist_linear_x , twist_linear_y , twist_angular_z;				// two kinds speed
double ix, iy, px, py;//initation & pose x,y
sensor_msgs::Image hsv_image;						//s
float dynamic_angular_gain;
bool is_starty1=true, is_startx=false, is_starty2=false;
double county1=0.5,county2=-0.5,countx=0.5;
double last_count=0;
int t=0;
bool is_avoiding = false;
bool avoid_done = false;
bool flag_bai = true;
ros::Publisher cmd_pub;
geometry_msgs::Twist twist;
int lu=0,zhong,zf;
int zhang=0;
int all_done=0;
// str topic;

void pid_Callback(const geometry_msgs::Twist& v);
void zhang_Callback(const std_msgs::UInt16& flag);

void poseCallback(const nav_msgs::Odometry &p_msg){
    px = p_msg.pose.pose.position.x;
    py = p_msg.pose.pose.position.y;
    // ROS_INFO("Robot walked %.2f m",px);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "fz");			// init note
    ros::NodeHandle nh;
    ros::Subscriber pid_sub;
    ros::param::set("/lor",0);
    while(lu==0)
        ros::param::get("/lor",lu);
    if(lu==1)
        pid_sub = nh.subscribe("/pidr", 10, pid_Callback);
    else if(lu==2)
        pid_sub = nh.subscribe("/pidl", 10, pid_Callback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);	// 分别用于发布	小车的速度指令	和	处理后的图像。		
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/image_hsv",10);
    // ros::Subscriber zhang = nh.subscribe("/zhang",10,zhang_Callback);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    ros::Rate loop_rate(15); // 每秒执行20次，根据图像处理速度调整

    while(ros::ok()){
        // geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.linear.y = twist_linear_y;
        twist.angular.z = twist_angular_z;
        cmd_pub.publish(twist);
        // ROS_WARN("xunxian is pubing cmd");
        // ROS_WARN("%d %d",is_avoiding,avoid_done);
        img_pub.publish(hsv_image);
        ros::param::get("/zhong",zhong);
        while(is_avoiding){
            // ROS_WARN("XUN is void loop");
            ros::param::get("/zf",zf);
            if(zf == 1) {
                is_avoiding = false;
                avoid_done = true;
            }
        }
        // if(zhong==1)
        //     ros::shutdown();
        ros::spinOnce();
        loop_rate.sleep(); // 控制频率
    }
    return 0;
}

// void zhang_Callback(const std_msgs::UInt16& flag){
//     // if(is_avoiding)
//     //     {
//     //         // ROS_INFO("zhang");
//     //         // twist_linear_x=0;
//     //         // is_avoiding=true;
//     //         if(is_starty1)
//     //         {
//     //             iy = py;
//     //             is_starty1 =false;
//     //             twist_linear_x = 0;
//     //             county1=0;
//     //         }
//     //         if(county1 < 0.5){
//     //             twist_linear_y = 0.5;
//     //             ROS_INFO("y,%.2f , %.2f , %.2f",county1,py,iy);
//     //             last_count = county1;
//     //             county1 = py - iy;
//     //             if(county1==last_count){
//     //                 t++;
//     //             }
//     //         }
//     //         if(county1 > 0.5 || t>5)
//     //         {
//     //             twist_linear_y = 0;
//     //             is_startx = true;
//     //             county1=0.5;
//     //             ROS_INFO("y finish");
//     //         }
//     //         if(is_startx)
//     //         {
//     //             ix = px;
//     //             is_startx =false;
//     //             countx=0;
//     //             last_count=0;
//     //             t=0;
//     //         }
//     //         if(countx < 0.5){
//     //             twist_linear_x = 0.3;
//     //             ROS_INFO("x,%.2f , %.2f , %.2f",countx,px,ix);
//     //             last_count = countx;
//     //             countx = px - ix;
//     //             if(countx==last_count){
//     //                 t++;
//     //             }
//     //         }
//     //         if(countx > 0.5 || t>5){
//     //             twist_linear_x = 0;
//     //             is_starty2=true;
//     //             countx=0.5;
//     //             ROS_INFO("x finish");
//     //         }
//     //         if(is_starty2)
//     //         {
//     //             iy = py;
//     //             is_starty2 =false;
//     //             county2=0;
//     //             last_count=0;
//     //             t=0;
//     //         }
//     //         if(county2 > -0.5){
//     //             twist_linear_y = -0.5;
//     //             last_count = countx;
//     //             county2 = py - iy;
//     //             if(county2==last_count){
//     //                 t++;
//     //             }
//     //         }
//     //         if(county2 < -0.5 || t>5){
//     //             twist_linear_y = 0;
//     //             county2=-0.5;
//     //             avoid_done = true;
//     //             ROS_INFO("success");
//     //             is_avoiding=false;
//     //         }
//     //     }
//     ros::param::get("/zhang",zhang);
//     if(zhang==1 && !avoid_done && !is_avoiding)
//     {
//         twist_linear_x=0;
//         twist_angular_z=0;
//         // geometry_msgs::Twist stop_twist;
//         // stop_twist.linear.x = 0;
//         // stop_twist.linear.y = 0;
//         // stop_twist.angular.z = 0;
//         is_avoiding=true;
//         // for(int i=0; i<20; i++){
//         //     cmd_pub.publish(stop_twist);
//         //     ros::Duration(0.05).sleep();
//         // }
//         ROS_INFO("stop");
//         return;
//     }
// }
void pid_Callback(const geometry_msgs::Twist& v){
    ros::param::get("/zhang",zhang);
    if(zhang==1 && !avoid_done && !is_avoiding){
        twist_linear_x=0;
        twist_angular_z=0;
        is_avoiding=true;
        ROS_INFO("stop");
        return;
    }
    if(is_avoiding==true){
        return;
    }
    ros::param::get("/all_done",all_done);
    if(all_done)
    {
        twist_linear_x=0;
        twist_angular_z=0;
        system("rosnode kill xunxian");
        // system("rosnode kill sxunleft");
        // system("rosnode kill sxunright");
        system("rosnode kill hdl");
        system("rosnode kill hdr");
        return;
    }
    // ROS_INFO("%f",twist_angular_z);
    twist_angular_z = v.angular.z;
    twist_linear_x = v.linear.x;
    ros::Rate rate(30);
    if(v.angular.x==0 && flag_bai){
        for(int i=0;i<15;i++){
            rate.sleep();
        }
        flag_bai=false;
    }
}