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
int miss=0,mflag=1;
int lu=0;
int change_y=0;
int corner_ysite=0;
int ring_flag=0;
int status = 0;//status=0对应进弯口的操作，因为会根据左线补右线，不进行额外操作，只做返回判断；status=1对应出弯口操作；status=2对应另一边入口的操作；status=3万一本入口也需要补线的话的状态，待测试；最后一次回到出弯口不需要处理就能出去，应该不再需要状态。
bool flag;
int flagz=0;
int last_change_point_y = -1;
int last_change_point_x = -1;
int last_change_point_x2 = -1;
bool buxianflag = false;//补线标志位
int backflag=0;//补线取消标志位
int backflag1=0;//状态1的backflag
int backflag2=0;
int back_count=0;//补线取消记数
int back_count1=0;//状态1的back_count
int corner_x = -1; 
int corner_y = -1; // 初始值为 -1 表示未检测到
int last_left=48;
bool status_count = false;
int cwcount=0;
int incount=0;//进入状态变化阶段的计数器(防止进入巡线一开始导航偏差导致直接进入状态1)
bool inflag=false;//判断开始进行状态变化阶段

void image_Callback(const sensor_msgs::Image& msg);
int lostrightline();

int Col = 256;
int Row = 144;
int Pixle[144][256];
int leftline[144], rightline[144], midline[144];
int leftlineflag[144], rightlineflag[144];
int prev_midline = 127;
int endline = 80;
int view = 15;
int bias = 0;
int track_width[144] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 70-79
    164, 168, 171, 174, 175, 177, 175, 176, 178, 178, // 80-89
    179, 181, 154, 156, 160, 164, 167, 171, 175, 178, // 90-99
    182, 185, 187, 191, 194, 198, 201, 204, 207, 209, // 100-109
    213, 216, 218, 221, 225, 227, 230, 232, 235, 237, // 110-119
    231, 233, 236, 238, 240, 243, 245, 246, 247, 249, // 120-129
    250, 252, 254, 242, 243, 245, 246, 247, 249, 250, // 130-139
    251, 252, 253, 255, // 140-143
};

int find_change_left()//可能有问题，如果在口上右线是丢失状态，及根据左线补的，就会失效，要验证,尝试下面注释的检测左边线突变代码
{
    int change_point_flag = 0;
    int r=0;
    for (int i = 120; i >= 81; i--) {
        // ROS_INFO("left: %d,right: %d,width: %d",leftline[i],rightline[i],track_width[i]);
        if(lostrightline()) r=255;
        else r=rightline[i];
        // ROS_INFO("r: %d",r);
        if (r - leftline[i] > track_width[i]+50) {
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

int find_change_left2()//可能有问题，如果在口上右线是丢失状态，及根据左线补的，就会失效，要验证,尝试下面注释的检测左边线突变代码
{
    int change_point_flag = 0;
    int r=0;
    for (int i = 120; i >= 81; i--) {
        // ROS_INFO("left: %d,right: %d,width: %d",leftline[i],rightline[i],track_width[i]);
        if(lostrightline()) r=255;
        else r=rightline[i];
        // ROS_INFO("r: %d",r);
        if (r - leftline[i] > track_width[i]+50) {
            // ROS_INFO("cha:%d",r - leftline[i]-track_width[i]);
            change_point_flag = i;
            break;
        }
        // if((leftline[i]+rightline[i])/2 < 150)//保底机制，看看可用不可用
        // {
        //   change_point_flag = i;
        //   break;
        // }
    }
    return change_point_flag;
}

int find_change_right()//同上，可能有问题，如果在口上右线是丢失状态，及根据左线补的，就会失效
{
  int change_point_flag = 0;       //尝试如果右边丢线的话，取255作为右边线的值在宽度判断里(不改变实际rightline[i])，在弯道右线丢的情况下左线也不会太远，只有在另一入口上，左线向左走，宽度变大，识别。
  int r=0;
      for (int i = 120; i >= 81; i--) {
          // ROS_INFO("left: %d,right: %d,width: %d",leftline[i],rightline[i],track_width[i]);
          if(lostrightline()) r=255;
          else r=rightline[i];
          // ROS_INFO("r: %d",r);
          if (r - leftline[i] > track_width[i]+55) {
              change_point_flag = i;
              break;
          }
      }
      return change_point_flag;
}

int find_change_right2()//同上，可能有问题，如果在口上右线是丢失状态，及根据左线补的，就会失效
{
  int change_point_flag = 0;       //尝试如果右边丢线的话，取255作为右边线的值在宽度判断里(不改变实际rightline[i])，在弯道右线丢的情况下左线也不会太远，只有在另一入口上，左线向左走，宽度变大，识别。
  int r=0;
      for (int i = 120; i >= 81; i--) {
          // ROS_INFO("left: %d,right: %d,width: %d",leftline[i],rightline[i],track_width[i]);
          if(lostrightline()) r=255;
          else r=rightline[i];
          // ROS_INFO("r: %d",r);
          if (r - leftline[i] > track_width[i]+70) {
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

int lostleftline()
{
  int valid_leftline_count = 0,strong_count=0,rlcount=0;
    for (int i = Row - 1; i >= endline+30; i--) {
        if(leftlineflag[i] > 0){
            valid_leftline_count++;
        }
        if(leftlineflag[i] == 2){
          strong_count++;
        }
    }
    if((valid_leftline_count < 1 && strong_count < 1))
    {
      rlcount++;
    }
    else{
      rlcount=0;
    }
    bool is_leftline_all_lost = (rlcount>0);
    return is_leftline_all_lost;
}
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

bool back_left()
{
  int left_lines1=0;
  for (int i = endline+view; i >= endline; i--) {
        if (leftlineflag[i]==2) left_lines1++;
  }
  if(left_lines1>13) backflag1=1;
//   ROS_INFO("left_lines %d",left_lines);
  if(backflag1==1){
    if(left_lines1==0){
      back_count1++;
    }
    else{
      back_count1=0;
    }
    if(back_count1>5){
      backflag1=0;
      back_count1=0;
      ROS_INFO("back1");
      return true;
    }
  }
  return false;
}

bool back_left2()
{
  // back_count=0;
  int left_lines=0;
  for (int i = endline+view; i >= endline; i--) {
        if (leftlineflag[i]==2) left_lines++;
  }
  if(left_lines>13) backflag=1;
//   ROS_INFO("left_lines %d",left_lines);
  if(backflag==1){
    if(left_lines==0){
      back_count++;
    }
    else{
      back_count=0;
    }
    if(back_count>3){
      backflag=0;
      back_count=0;
      ROS_INFO("back2");
      return true;
    }
  }
  return false;
}

// bool back_left3()
// {
//   int left_lines=0;
//   for (int i = endline+view; i >= endline; i--) {
//         if (leftlineflag[i]==2) left_lines++;
//   }
//   if(left_lines>13) backflag=1;
// //   ROS_INFO("left_lines %d",left_lines);
//   if(backflag==1){
//     if(left_lines==0){
//       back_count++;
//     }
//     else{
//       back_count=0;
//     }
//     if(back_count>7){
//       backflag=0;
//       back_count=0;
//       return true;
//     }
//   }
//   return false;
// }

bool back_left3()
{
  int left_lines=0;
  for (int i = endline+view; i >= endline; i--) {
        if (leftlineflag[i]==2) left_lines++;
  }
  // ROS_INFO("left_lines %d backflag: %d backflag2: %d",left_lines,backflag,backflag2);
  if(backflag==0){
    if(left_lines==0){
      back_count++;
    }
    else{
      back_count=0;
    }
    if(back_count>4){
      backflag=1;
      back_count=0;
    }
  }
  if(backflag==1 && left_lines!=0){
    backflag2=1;
  }
  if(backflag2==1){
    if(left_lines==0){
      back_count++;
    }
    else{
      back_count=0;
    }
    if(back_count>6){
      backflag=0;
      backflag2=0;
      back_count=0;
      ROS_INFO("back4");
      return true;
    }
  }
  return false;
}

int find_left_ring_corner()
{
    int corner_y = 0;
    int corner_flag = 0;
    int search_range = 15;
    int miss_right_lines = 0;
    int miss_left_lines = 0;

    for (int y = 120; y > 80; y--) {
        if (!leftlineflag[y] || leftline[y] < 0 || leftline[y] >= Col) continue;

        // 在左边界附近搜索像素跳变
        for (int x = leftline[y]; x < leftline[y] + search_range && x < Col - 1; x++) {
            if (Pixle[y][x] == 1 && Pixle[y][x + 1] == 0 && corner_flag == 0) { // 白到黑跳变
                corner_flag = 1;
            }
            if (Pixle[y][x] == 0 && Pixle[y][x + 1] == 1 && corner_flag == 1) { // 黑到白跳变
                corner_flag = 2;
                corner_y = y;
                break;
            }
        }
        if (corner_flag == 2) {
            break; // 找到拐点，退出循环
        }
    }
    return corner_y;
}

int find_top_corner() {//找上部中间的角点，待验证
    int start_x = 0;
    int end_x = Col;
    int start_y = endline-30;
    int end_y = endline+30;
    int jump_count = 0;
    const int min_jump_count = 2; // 要求至少两个跳变

    for (int y = start_y; y < end_y; y++) {
        for (int x = start_x; x < end_x - 1; x++) {
            if (Pixle[y][x] == 0 && Pixle[y][x + 1] == 1) {
                jump_count++;
            } else if (Pixle[y][x] == 1 && Pixle[y][x + 1] == 0 && jump_count > 0) {
                jump_count++;
                if (jump_count >= min_jump_count) {
                    corner_x = x;
                    corner_y = y;
                    break;
                }
            }
        }
        if (corner_y > 0) {
            ROS_INFO("Top corner found at (%d, %d)", corner_x, corner_y);
            break;
        }
    }

    return corner_x;
}

void Add_change_point_line(int startX, int startY, int startX2)//根据前5行的点连线斜率补
{
  int change;
  if(startX2>startX){
    change=startX2;
    startX2=startX;
    startX=change;
  }
   for (int i = endline+view; i <= 143; i++) {
        int x = (int)(startX - float(i - startY) / float(5) * (startX - startX2));
        if (x < 0) x = 0;
        if (x >= Col) x = Col - 1;
        if(leftline[i]+20<x){//避免补线时左线写死，不会根据情况调整，解决转小，贴左边线
          leftline[i] = x;
          leftlineflag[i] = 1;
        }
        if(rightline[i]==2){//待测试，有没有用
          leftline[endline+view] = rightline[i]-track_width[i];
          leftlineflag[i] = 1;
        }
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
    // if (leftlineflag[hang] && rightlineflag[hang]) {
    //     track_width[hang] = rightline[hang] - leftline[hang];
    // } else {
    //     track_width[hang] = track_width[hang + 1];  // 继承上一行值
    // }
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
//    cout<<hang<<"宽度 == "<<rightline[hang]-leftline[hang]<<endl;        //量赛道宽度
//    cout<<hang<<"宽度 == "<<track_width[hang]<<endl;        //量赛道宽度
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
    // cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test2.jpg", res);
    // 处理逻辑
				// origin image
    for (int i = 0; i < Row; i++) {
        for (int j = 0; j < Col; j++) {
            Pixle[i][j] = (res.at<uchar>(i, j) == 0) ? 0 : 1;
        }
    }
    // hsv_image.data = res;
    cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test1.jpg", res);
    get_route_one();
    get_route_all();

    if(status==0){
      if(midline[endline+view]>115 && midline[endline+view]<140)
      {
        incount++;
      }
      else{
        incount=0;
      }
      if(incount>5){
        inflag=true;
      }
      if(midline[endline+view]>160 && inflag){
        status++;
        ROS_WARN("turn 1");
      }
    }

    if(status>=1 && status<4){
      bias=35;//32
      flagz=find_change_left();
      if(flagz!=0){
        // ROS_INFO("status1 turn to status2");
        if (!buxianflag) {
            int change_point = find_left_ring_corner();
            last_change_point_y = change_point;
            last_change_point_x = leftline[change_point];
            last_change_point_x2 = leftline[change_point + 5];
            buxianflag = true;
        }
        // bias=0;
      }
      if(buxianflag && last_change_point_y != -1){
        // 使用历史记录的补线起点
        int start_x = last_change_point_x;
        int start_y = last_change_point_y;
        int start_x2 = last_change_point_x2;
        // ROS_INFO("%d %d",start_x,start_y);

        Add_change_point_line(start_x,start_y,start_x2);

        for (int i = endline; i <= 143; i++) {
            int x = leftline[i];
            if (x >= 0 && x < Col && i >= 0 && i < Row) {
                raw.at<cv::Vec3b>(i, x) = cv::Vec3b(0, 0, 255);
            }
        }

        for (int i = endline; i <= 143; i++) {
            if (leftlineflag[i] && rightlineflag[i]) {
                midline[i] = leftline[i] + (rightline[i] - leftline[i]) / 2;
            } else if (rightlineflag[i]) {
                leftline[i] = rightline[i] - track_width[i];
                midline[i] = (leftline[i] + rightline[i]) / 2;
            } else if (leftlineflag[i]) {
                rightline[i] = leftline[i] + track_width[i];
                midline[i] = (leftline[i] + rightline[i]) / 2;
            }
        }
      }
      if(buxianflag && back_left() && status==1){//早点回可能可以避免压内线的问题
        ROS_WARN("turn 2");
        status++;
      }
    }

    if(status==2)
    {
      bias=35;
      flagz=find_change_left2();
      if(flagz!=0 || midline[endline+view]<170){
        ROS_INFO("status2 turn to status3");
        corner_x=find_top_corner();
        if (!status_count && (corner_y !=-1 || leftline[endline+view]>120)) {
              status_count=true;
          }
      }
      if(status_count){
        bias=45;
      }

      if(status_count && (back_left2() || leftline[endline+view]>120)){//如果右线恢复，根据右线补左线
        status++;
        backflag=0;
        status_count=false;
        ROS_WARN("turn 3");
      }
    }

    if(status==3)//本入弯口，可能需要处理,及右线是丢失的情况，本来如果根据右线补左线，不需要处理
    {
      bias=35;
      flagz=find_change_right2();
      if(flagz!=0){
        ROS_INFO("status3 turn to status4");
        status_count=true;
      }

      if(status_count && back_left3()){//如何让状态3到4稳定的逻辑是问题
        status++;
        status_count=false;
      }
    }

    if(status==4)//出弯口，大概率不需要，先写着以防万一。不对，这个状态的时候要把bias置0
    {
        bias=10;
        status++;
    }

    ROS_INFO("status: %d, bias: %d",status,bias);
    // if(leftline[endline+view]-last_left>80)
    //     leftline[endline+view]=last_left+20;
    // last_left=leftline[endline+view];
    int cx = (leftline[endline+view]+leftline[endline+view+1]+leftline[endline+view+2])/3 + 80 + bias;
    // if(cx<85 && !flag && status==5){
    //   bias=-40;
    //   flag=1;
    //   ros::param::set("/locate_pic_ransac/control",2);
    // }
    // if(flag && cx>85){
    //   cwcount++;
    // }
    // if(cwcount>=3){
    //   bias=0;
    // }
    if(cx<100 && !flag && status==5){
      bias=0;
      flag=1;
      ros::param::set("/locate_pic_ransac/control",2);
    }
    ROS_INFO("cx: %d", cx);
    ROS_INFO("left: %d ,right: %d",leftline[endline+view],rightline[endline+view]);
    ROS_INFO("------------------------------");
    cv::circle(raw, cv::Point(cx, endline+view), 10, (255, 255, 255));
    cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test.jpg", raw);
    Kp=0.3;    
    Ki=0.0;                                                  //x=0.35,p=0.3,d=0.7//p=0.5,d=1.3//可能要调PID，每次跑的结果相差大有影响
    Kd=1.2;                                                  //x=0.5,p=0.5,d=2.2
    error = (cx - Col / 2) * 0.03;                                  
    twist_linear_x = 0.3;
    // if((status==0 && (midline[endline+view]<135 || !inflag))){//直线加速
    //   Kp=0.5;
    //   Ki=0.0;
    //   Kd=1.9;
    //   twist_linear_x = 0.5;
    // }
    twist_angular_z = -pidControl(error);
    // ROS_INFO("%f",twist_angular_z);
}
