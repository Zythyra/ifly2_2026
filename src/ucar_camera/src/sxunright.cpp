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
int llcount=0;
int last_change_point_y = -1;
int last_change_point_x = -1;
bool is_using_last_right_patch = false;
int right_discontinuity_line = -1;   // 保存突变点行号
int right_discontinuity_line_x = -1;
int patch_frame_count = 0;  // 补线帧计数
static int last_kink_row = 0;
float right_discontinuity_k = 1.0f;  // 保存补线斜率
bool is_using_right_patch = false;   // 是否正在使用右边补线
int cwcount=0;
void image_Callback(const sensor_msgs::Image& msg);

int Col = 256;
int Row = 144;
int Pixle[144][256];
int leftline[144], rightline[144], midline[144];
int leftlineflag[144], rightlineflag[144];
int track_width[144];
int prev_midline = 127;
// int endline = 60;//x=0.6
int endline = 80;//x=0.35和x=0.5
int view = 15;
int bias = 0;
int incount=0;
int inflag=0;
int status =0;


int Continuity_Change_Right_Adapted(int start, int end)
{
    int continuity_change_flag = 0;

    // 保证 start > end
    if (start < end) {
        int t = start;
        start = end;
        end = t;
    }

    // 防止越界，Row是你的图像高度
    if (start >= Row - 5)
        start = Row - 5;
    if (end <= 5)
        end = 5;

    for (int i = start; i >= end; i--) {
        if (abs(rightline[i] - rightline[i - 1]) >= 10) {  // 阈值5可调
            ROS_INFO("%d",abs(rightline[i] - rightline[i - 1]));
            continuity_change_flag = i;
            break;
        }
    }

    return continuity_change_flag;
}

int find_change_point(int start, int end)
{
    int change_point_flag = 0;
    for (int i = start; i >= end; i--) {
        if (rightlineflag[i]==2 || leftlineflag[i]==2) {  // 阈值5可调
            change_point_flag = i;
            break;
        }
    }
    return change_point_flag;
}

void K_Add_Boundry_Right(float k, int startX, int startY, int endY)
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

        rightline[i] = x;
        rightlineflag[i] = 1;
    }
}
void Add_change_point_line(int startX, int startY)
{
    for (int i = startY; i <= 143; i++) {
        int x = (int)(float(i - startY) / float(144-startY) * (256-startX) + startX);
        if (x < 0) x = 0;
        if (x >= Col) x = Col - 1;

        rightline[i] = x;
        rightlineflag[i] = 1;
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

    ros::init(argc, argv, "xianr");			// init note
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
    static int last_right_turn_line = 0;
    static float last_right_slope = 1.0f;  // 固定为负值，例如右线向左倾斜
    int valid_leftline_count = 0,strong_count=0;

    for (int i = Row - 2; i > endline; i--){
      midline[i] = leftline[i] + 80;//x=0.35和x=0.5
      // midline[i] = leftline[i] + 60;//x=0.6
    }

    int cx = midline[endline + view]+bias;//x=0.35和x=0.5
    // int cx = midline[endline + view +10]+bias;//x=0.6

    // if(flag && cx>=60){
    //   cwcount++;
    // }
    // if(cwcount>=1){
    //   bias=0;
    // }
    // if(cx<100 && !flag){//x=0.6的代码
    //   bias=-40;
    //   flag=1;
    //   ros::param::set("/locate_pic_ransac/control",2);
    // }

    // if(cx<90 && !flag){//x=0.5的代码
    //   bias=-25;
    //   flag=1;
    //   ros::param::set("/locate_pic_ransac/control",2);
    // }
    // if(flag && cx>100){
    //   cwcount++;
    // }
    // if(cwcount>=3){
    //   bias=0;
    // }

    if(cx<90){//x=0.35的代码
      ros::param::set("/locate_pic_ransac/control",2);
    }

    // if(midline[endline+view]>115 && midline[endline+view]<140)//直线加速版
    // {
    //   incount++;
    // }
    // else{
    //   incount=0;
    // }
    // if(incount>5){
    //   inflag=true;
    // }
    // if(cx>145 && inflag){
    //   flag=1;
    //   status=1;
    // }
    // if(flag && cx<100){
    //   flag2=1;
    //   bias=20;
    //   ros::param::set("/locate_pic_ransac/control",2);
    // }
    // if(flag2 && cx>100){
    //   cwcount++;
    // }
    // if(cwcount>=3){
    //   bias=0;
    // }
    

    ROS_INFO("cx: %d bias: %d", cx,bias);
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
    Ki=0.0;                                                  //x=0.35,p=0.3,d=0.7压角，//p=0.5,d=1.8不会压角
    Kd=1.8;                                                  //x=0.5,p=0.5,d=2.2
    error = (cx - Col / 2) * 0.03;                           //x=0.6,p=0.6,d=2.2
    twist_linear_x = 0.35;
    twist_angular_z = -pidControl(error);
    // ROS_INFO("%f",twist_angular_z);
}