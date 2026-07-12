#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<geometry_msgs/Twist.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/imgproc/types_c.h>
#include<opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>

double twist_linear_x , twist_angular_z;				// two kinds speed

using namespace std;
sensor_msgs::Image hsv_image;						//s

float dynamic_angular_gain;
double Kp,Ki,Kd;
double error=0;
double integral = 0.0;
double previous_error = 0.0;
int miss=0,mflag=1,flag=0,flag2=0;
int lu=0;
int cwcount=0;
void image_Callback(const sensor_msgs::Image& msg);

int Col = 256;
int Row = 144;
int Pixle[144][256];
int leftline[144], rightline[144], midline[144];
int leftlineflag[144], rightlineflag[144];
int track_width[144];
int prev_midline = 127;
int endline = 80;
// int endline = 60;//x=0.6
int view = 15;
int bias = 0;
int status=0;
int incount=0;
int inflag=0;

int lostrightline()
{
  int valid_rightline_count = 0,strong_count=0,rlcount=0;
    for (int i = Row - 1; i >= endline+30; i--) {
        if(rightlineflag[i] > 0){
            valid_rightline_count++;
        }
        if(rightlineflag[i] == 2){
          strong_count++;
        }
    }
    if((valid_rightline_count < 1 && strong_count < 1))
    {
      rlcount++;
    }
    else{
      rlcount=0;
    }
    bool is_rightline_all_lost = (rlcount>0);
    return is_rightline_all_lost;
}

int find_change_left()//可能有问题，如果在口上右线是丢失状态，及根据左线补的，就会失效，要验证,尝试下面注释的检测左边线突变代码
{
    int change_point_flag = 0;
    int r=0;
    for (int i = 120; i >= 81; i--) {
        // ROS_INFO("left: %d,right: %d,width: %d",leftline[i],rightline[i],track_width[i]);
        if(lostrightline()) r=255;
        else r=rightline[i];
        // ROS_INFO("r: %d",r);
        if (r - leftline[i] > track_width[i]+40) {
            // ROS_INFO("cha:%d",r - leftline[i]-track_width[i]);
            change_point_flag = i;
            break;
        }
        if((leftline[i]+rightline[i])/2 < 150)//保底机制，看看可用不可用
        {
          change_point_flag = i;
          break;
        }
    }
    return change_point_flag;
}

int Continuity_Change_Left_Adapted(int start, int end)
{
    int continuity_change_flag = 0;

    if (start < end) {
        std::swap(start, end);
    }

    if (start >= Row - 5)
        start = Row - 5;
    if (end <= 5)
        end = 5;

    for (int i = start; i >= end; i--) {
        if (abs(leftline[i] - leftline[i - 1]) >= 5) {
            continuity_change_flag = i;
            break;
        }
    }

    return continuity_change_flag;
}

void K_Add_Boundry_Left(float k, int startX, int startY, int endY)
{
    if (startY >= Row) startY = Row - 1;
    if (startY < 0) startY = 0;
    if (endY >= Row) endY = Row - 1;
    if (endY < 0) endY = 0;

    if (startY < endY) {
        std::swap(startY, endY);
    }

    for (int i = startY; i >= endY; i--) {
        int x = (int)((i - startY) / k + startX);
        if (x < 0) x = 0;
        if (x >= Col) x = Col - 1;

        leftline[i] = x;
        leftlineflag[i] = 1;
    }
}

void get_route_one(void) //传入需要检索的那一行
{
  //清零
  for (uint hang = Row/2; hang < Row; hang++)
  {
    leftline[hang] = 0;
    rightline[hang] = Col - 1;
    leftlineflag[hang] = 0;
    rightlineflag[hang] = 0;
    midline[hang] = Col / 2;
  }
//  endline = 0;

    //左边线

      //向左找
      for (uint lie = Col/2; lie >= 1; lie--)
      {
        if (Pixle[Row - 1][lie] != Pixle[Row - 1][lie - 1])
        {
          leftline[Row - 1] = lie;
          leftlineflag[Row - 1] = 2;
          break;
        }
      }
    


    //右边线

      //向右找
      for (uint lie = Col/2; lie <= Col - 2; lie++)
      {
        if (Pixle[Row - 1][lie] != Pixle[Row - 1][lie + 1])
        {
          rightline[Row - 1] = lie;
          rightlineflag[Row - 1] = 2;
          break;
        }
      }

    track_width[Row - 1] = rightline[Row - 1] - leftline[Row - 1];


    
    //如果没找到，直接给定值
//    cout << leftlineflag[Row - 1] << rightlineflag[Row - 1] << endl;
    //如果左右边线都不绝对可信
}

void get_route_all(void)
{
  //初始化
  int Lstart = 0, L_max = 0, L_min = 0;
  int Rstart = 0, R_max = 0, R_min = 0;
  int range = 30; //搜线范围 15

  for (uint hang = Row - 2; hang > endline; hang--)
  {
    //左边线
    //确定搜线范围
    //根据上一行确定本行寻线点
    if (leftlineflag[hang] && rightlineflag[hang]) {
        track_width[hang] = rightline[hang] - leftline[hang];
    } else {
        track_width[hang] = track_width[hang + 1];  // 继承上一行值
    }
    if (leftlineflag[hang + 1] == 0) //上一行丢线
    {
      if (rightlineflag[hang + 1] != 0)
      {                                                          //上一行右线不丢
        L_max = rightline[hang + 1] + range - track_width[hang]; //搜线的起始条件 - track_width[hang]
        L_min = rightline[hang + 1] - range - track_width[hang]; //- track_width[hang]
        Lstart = rightline[hang + 1] - track_width[hang];// - track_width[hang]
      
        if (L_max > Col - 1) //限幅
          L_max = Col - 1;
        if (L_max < 0)
          L_max = 0;
        if (L_min > Col - 1)
          L_min = Col - 1;
        if (L_min < 0)
          L_min = 0;
        if (Lstart > Col - 1)
          Lstart = Col - 1;
        if (Lstart < 0)
          Lstart = 0;
      }
      else
      {
        L_max = Col * 1 / 2;
        L_min = 0;
        Lstart = Col * 1 / 2; //为什么-10           todo
      }
    }
    else //上一行不丢线
    {
      L_max = leftline[hang + 1] + range;
      L_min = leftline[hang + 1] - range;
      Lstart = leftline[hang + 1];
      if (L_max > Col - 1) //限幅
        L_max = Col - 1;
      if (L_min < 0)
        L_min = 0;
    }
    //左边线
     //向左找
//     cout << "final is" << Lstart << endl;
// 	 for (uint lie = Lstart; lie > L_min; lie--)
//       {
        
//         if (Pixle[hang][lie] != Pixle[hang][lie - 1])
//         {
// //        	cout << L_min << Lstart << endl;
//           leftline[hang] = lie;
//           leftlineflag[hang] = 2;
//           break;
//         }
//       }
      for (int lie = Lstart; lie > L_min && lie > 0; lie--) {
          if (Pixle[hang][lie] != Pixle[hang][lie - 1]) {
              leftline[hang] = lie;
              leftlineflag[hang] = 2;
              break;
          }
      }

      
      //向右找
      if(leftlineflag[hang] == 0){
      	for (uint lie = Lstart; lie < L_max; lie++)
         {
	        if (Pixle[hang][lie] != Pixle[hang][lie + 1])
	        {
	          leftline[hang] = lie;
	          leftlineflag[hang] = 1;
	          break;
	        }
         }
      }
      

    //右边线
    //确定寻线范围
    if (rightlineflag[hang + 1] == 0) //上一行丢线
    {
      if (leftlineflag[hang + 1] != 0)
      { //上一行左线不丢
        R_max = leftline[hang + 1] + range + track_width[hang];//
        R_min = leftline[hang + 1] - range + track_width[hang];// + track_width[hang]
        Rstart = leftline[hang + 1] + track_width[hang];// + track_width[hang]
        if (R_max > Col - 1) //限幅
          R_max = Col - 1;
        if (R_max < 0)
          R_max = 0;
        if (R_min > Col - 1)
          R_min = Col - 1;
        if (R_min < 0)
          R_min = 0;
        if (Rstart > Col - 1)
          Rstart = Col - 1;
        if (Rstart < 0)
          Rstart = 0;
      }
      else
      {
        R_max = Col - 1;
        R_min = Col * 1 / 2;
        Rstart = Col * 1 / 2;
      }
    }
    else
    {
      Rstart = rightline[hang + 1];
      R_max = rightline[hang + 1] + range;
      R_min = rightline[hang + 1] - range;
      if (R_max > Col - 1)
        R_max = Col - 1;
      if (R_min < 0)
        R_min = 0;
    }
    
    //右边线
      //向右找
      for (uint lie = Rstart; lie < R_max; lie++)
      {
        if (Pixle[hang][lie] != Pixle[hang][lie + 1])
        {
          rightline[hang] = lie;
          rightlineflag[hang] = 2;
          break;
        }
      }
      //向左找
      if(rightlineflag[hang] == 0){
	      for (uint lie = Rstart; lie > R_min; lie--)
	      {
	        if (Pixle[hang][lie] != Pixle[hang][lie - 1])
	        {
	          rightline[hang] = lie;
	          rightlineflag[hang] = 1;
	          break;
	        }
	      }
      }
      
    if(!rightlineflag[hang]){
    	if(leftlineflag[hang])rightline[hang] = leftline[hang] + track_width[hang];
    	else rightline[hang] = rightline[hang + 1];
    }
    
    if(!leftlineflag[hang]){
    	if(rightlineflag[hang])leftline[hang] = rightline[hang] - track_width[hang];
    	else leftline[hang] = leftline[hang + 1];
    }
  //  cout << "left is" << leftline[hang] << endl;
//    cout << "leftflag is" << leftlineflag[hang] << endl;
  //  cout << "right is" << rightline[hang] << endl;
    

    midline[hang] = leftline[hang] + (rightline[hang] - leftline[hang]) / 2;
    midline[hang] = midline[hang] < leftline[hang]?leftline[hang]:midline[hang] > rightline[hang]?rightline[hang]:midline[hang];
    // if (abs(midline[hang] - prev_midline) > 20)  // 如果变化过大，就保持原值
    //   midline[hang] = prev_midline;
    // prev_midline = midline[hang];
    // cout << "midle is" << midline[hang] << endl;
    leftline[hang] = leftline[hang] < 0?0:leftline[hang];
    rightline[hang] = rightline[hang] > Col - 1?Col - 1:rightline[hang];
  //  cout<<hang<<"宽度 == "<<rightline[hang]-leftline[hang]<<endl;        //量赛道宽度
  //  cout<<hang<<"宽度 == "<<track_width[hang]<<endl;        //量赛道宽度
  }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "xianl");			// init note
    ros::NodeHandle nh;
    ros::param::set("/lor",0);
    while(lu==0)
        ros::param::get("/lor",lu);
    ros::Subscriber img_sub = nh.subscribe("/ucar_camera/image_raw", 10, image_Callback); 	// 更改为订阅 /usb_cam/image_raw 订阅者img_sub来接收来自USB摄像头的原始图像,and image_Callback
    ros::Publisher pid_pub = nh.advertise<geometry_msgs::Twist>("/pidr",10);	
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
    
    cv::Mat raw = cv_ptr->image.clone();
    cv::Mat image = cv_ptr->image;			// 原始图像
    cv::Mat hsv = image.clone();			// 用于后续的HSV转换
    cv::Mat res = image.clone();			// 用于存储颜色过滤后的结果 (keep medium)
    cv::Size kernelSize(5, 5);
    cv::resize(cv_ptr->image, image, cv::Size(Col, Row), 0, 0, cv::INTER_LINEAR);
    cv::resize(raw, raw, cv::Size(Col, Row));
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
    for (int i = 0; i < Row; i++) {
        for (int j = 0; j < Col; j++) {
            Pixle[i][j] = (res.at<uchar>(i, j) == 0) ? 0 : 1;
        }
    }
    cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test1.jpg", res);
    get_route_one();
    get_route_all();
        static int last_left_turn_line = 0;
    static float last_left_slope = -1.0f;  // 固定为正值，例如左线向右倾斜
    for (int i = Row - 2; i > endline; i--){
        midline[i] = rightline[i] - 80;
        // midline[i] = rightline[i] - 65;
    }
    // int left_turn_line = Continuity_Change_Left_Adapted(Row - 1, endline + 10);

    // if (left_turn_line > 0) {
    //     // ROS_INFO("find LEFT turning point");

    //     float fix_k = -1.0;  // 左线向右倾斜，k 要为正（例：1.0 ~ 2.0）
    //     int start_x = leftline[left_turn_line];
    //     int start_y = left_turn_line;
    //     int end_y = endline + 5;

    //     K_Add_Boundry_Left(fix_k, start_x, start_y, end_y);

    //     // 可视化补线（绿色线）
    //     for (int i = end_y; i <= start_y; i++) {
    //         int x = leftline[i];
    //         if (x >= 0 && x < Col && i >= 0 && i < Row) {
    //             raw.at<cv::Vec3b>(i, x) = cv::Vec3b(0, 255, 0);
    //         }
    //     }

    //     // 重算中线
    //     for (int i = end_y; i <= start_y; i++) {
    //         if (leftlineflag[i] && rightlineflag[i]) {
    //             midline[i] = leftline[i] + (rightline[i] - leftline[i]) / 2;
    //         } else if (rightlineflag[i]) {
    //             leftline[i] = rightline[i] - track_width[i];
    //             midline[i] = (leftline[i] + rightline[i]) / 2;
    //         } else if (leftlineflag[i]) {
    //             rightline[i] = leftline[i] + track_width[i];
    //             midline[i] = (leftline[i] + rightline[i]) / 2;
    //         }
    //     }

    //     last_left_turn_line = left_turn_line;
    //     last_left_slope = fix_k;

    // } else {
    //     // ROS_WARN("No LEFT turning point found, fallback to previous slope");

    //     if (last_left_turn_line > 0 && last_left_slope != 0) {
    //         int start_x = leftline[last_left_turn_line];
    //         int start_y = last_left_turn_line;
    //         int end_y = endline + 5;

    //         K_Add_Boundry_Left(last_left_slope, start_x, start_y, end_y);

    //         // 可视化补线（青色线）
    //         for (int i = end_y; i <= start_y; i++) {
    //             int x = leftline[i];
    //             if (x >= 0 && x < Col && i >= 0 && i < Row) {
    //                 raw.at<cv::Vec3b>(i, x) = cv::Vec3b(255, 255, 0);  // 青色线
    //             }
    //         }

    //         for (int i = end_y; i <= start_y; i++) {
    //             if (leftlineflag[i] && rightlineflag[i]) {
    //                 midline[i] = leftline[i] + (rightline[i] - leftline[i]) / 2;
    //             } else if (rightlineflag[i]) {
    //                 leftline[i] = rightline[i] - track_width[i];
    //                 midline[i] = (leftline[i] + rightline[i]) / 2;
    //             } else if (leftlineflag[i]) {
    //                 rightline[i] = leftline[i] + track_width[i];
    //                 midline[i] = (leftline[i] + rightline[i]) / 2;
    //             }
    //         }
    //     } else {
    //         ROS_ERROR("No valid LEFT fallback. Degraded mode recommended.");
    //     }
    // }
    int cx = (midline[endline + view]+midline[endline + view+1]+midline[endline + view+2])/3+bias;
    // int cx = midline[endline + view +10]+bias;//x=0.6

  //   if(flag2 && cx<170)
  //   {
  //     cwcount++;
  //   }
  //   if(flag2 && cx>170){
  //     cwcount=0;
  //   }
  //   if(cwcount>=1){
  //     bias=0;
  //   }
  //  if(cx<110){//x=0.6的代码
  //     flag=1;
  //   }
  //   if(flag && cx>100 && !flag2){
  //     bias=60;
  //     flag2=1;
  //     ros::param::set("/locate_pic_ransac/control",2);
  //   }

    // if(cx<90){//x=0.5的代码
    //   flag=1;
    // }
    // if(flag && cx>110 && !flag2){
    //   bias=30;
    //   flag2=1;
    //   ros::param::set("/locate_pic_ransac/control",2);
    // }
    // if(flag2 && cx<170)
    // {
    //   cwcount++;
    // }
    // if(flag2 && cx>170){
    //   cwcount=0;
    // }
    // if(cwcount>=2){
    //   bias=0;
    // }

    // if(cx<90){//x=0.35的代码
    //   flag=1;
    // }
    // if(flag && cx>110){
    //   ros::param::set("/locate_pic_ransac/control",2);
    // }
    if(status==0){
      if(midline[endline+view]>115 && midline[endline+view]<140)//直线加速版
      {
        incount++;
      }
      else{
        incount=0;
      }
      if(incount>5){
        inflag=true;
      }
      if(midline[endline+view]>160 && inflag){//***midline好像不够严谨，考虑修改
        status++;
        ROS_WARN("turn 1");
      }
    }
    if(status==1){
      flag=find_change_left();
      if(flag!=0){
        bias=50;
        ros::param::set("/locate_pic_ransac/control",2);
      }
      if(flag && cx<170){
        cwcount++;
      }
      if(cwcount>=2){
        bias=0;
      }
    }

    ROS_INFO("cx: %d bias: %d status: %d", cx,bias,status);
    cv::circle(raw, cv::Point(cx, endline+view), 10, (255, 255, 255));
    cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test.jpg", raw);
    // if(status==0){
    //   Kp=0.5;
    //   Ki=0.0;
    //   Kd=2.2;
    //   twist_linear_x = 0.5;
    // }
    // else{
    //   Kp=0.5;    
    //   Ki=0.0;                                                  //x=0.35,p=0.1,d=0.5
    //   Kd=1.8;                                                   //x=0.35,p=0.5,d=0.8 //x=0.5,p=0.5,d=1.8                            
    //   twist_linear_x = 0.35;
    // }
    Kp=0.5;    
    Ki=0.0;                                                  //x=0.35,p=0.1,d=0.5
    Kd=2.2;                                                   //x=0.35,p=0.5,d=0.8 //x=0.5,p=0.5,d=1.8
    error = (cx - Col / 2) * 0.03;                                  
    twist_linear_x = 0.5;
    twist_angular_z = -pidControl(error);
    // ROS_INFO("%f",twist_angular_z);
}