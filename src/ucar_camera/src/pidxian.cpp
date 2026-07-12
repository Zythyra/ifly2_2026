#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <ros/ros.h>  
#include <std_msgs/Int8.h>  
// #include <json/json.h>  
#include "std_msgs/String.h"
#include <unistd.h>

using namespace std;
using namespace cv;

int Col = 256;//64
int Row = 144;//48
int Pixle[144][256];
int leftline[144];
int rightline[144];
int leftlineflag[144];
int rightlineflag[144];
int midline[144];

int xx_thre;
int view;
int bias;
int draw;
int prev_midline=127;
Mat frame;
int black = 0;
int white = 1;
int endline = 30;//30
// int track_width[48] = { 
//     14, 17, 19,14, 17, 19,14, 17, 19,14, 17, 19,
//     23, 24, 26, 28, 30, 32, 34, 36, 38,
//     27, 27, 27, 27, 32, 36, 36, 36, 30,
//     32, 34, 36, 38, 40, 42, 44, 46, 48,
//     48, 50, 52, 54, 56, 58, 60, 62, 64};
int track_width[144];
    
VideoCapture capture;

void Get_Pixle(Mat frame)
{
  for (int i = 0; i < Row; i++){
    for (int j = 0; j < Col; j++){
      if (frame.at<uchar>(i, j) == 0){Pixle[i][j] = black;}
      else{Pixle[i][j] = white;}
    }
  }
}

void get_route_one(void) //传入需要检索的那一行
{
  //清零
  for (uint hang = 0; hang < Row; hang++)
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
    

    
    //如果没找到，直接给定值
//    cout << leftlineflag[Row - 1] << rightlineflag[Row - 1] << endl;
    //如果左右边线都不绝对可信
}

void get_route_all(void)
{
  //初始化
  int Lstart = 0, L_max = 0, L_min = 0;
  int Rstart = 0, R_max = 0, R_min = 0;
  int range = 15; //搜线范围 15

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
	 for (uint lie = Lstart; lie > L_min; lie--)
      {
        
        if (Pixle[hang][lie] != Pixle[hang][lie - 1])
        {
//        	cout << L_min << Lstart << endl;
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
   cout << "left is" << leftline[hang] << endl;
//    cout << "leftflag is" << leftlineflag[hang] << endl;
   cout << "right is" << rightline[hang] << endl;
    

    midline[hang] = leftline[hang] + (rightline[hang] - leftline[hang]) / 2;
    midline[hang] = midline[hang] < leftline[hang]?leftline[hang]:midline[hang] > rightline[hang]?rightline[hang]:midline[hang];
    if (abs(midline[hang] - prev_midline) > 20)  // 如果变化过大，就保持原值
      midline[hang] = prev_midline;
    prev_midline = midline[hang];
    cout << "midle is" << midline[hang] << endl;
    leftline[hang] = leftline[hang] < 0?0:leftline[hang];
    rightline[hang] = rightline[hang] > Col - 1?Col - 1:rightline[hang];
   cout<<hang<<"宽度 == "<<rightline[hang]-leftline[hang]<<endl;        //量赛道宽度
   cout<<hang<<"宽度 == "<<track_width[hang]<<endl;        //量赛道宽度
  }
}

int cor_flag = 0;
int cor_find_Row;
int cor_find_Row_Start;
int cor_find(void){
	for(int i = cor_find_Row_Start;i > cor_find_Row;i--){
		for(int j = 0;j < Col;j++){
			if(Pixle[i][j])return 0;
		}
	}
	return 1;
}
int l = 46, r = 46;
int OTSU = 1;
Mat ImageProcess(Mat frame)
{ //定义图像处理函数
  Mat dst, dstt;
  cv::Size kernelSize(5, 5);
  resize(frame, dst, Size(Col, Row), 0, 0);
  // cv::cvtColor(dst, dst, cv::COLOR_BGR2HSV);	// 颜色空间转换
  // cv::inRange(dst, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), dstt);	

  cvtColor(dst, dst, cv::COLOR_RGB2GRAY); //转换灰度图
  // if(OTSU){threshold(dst, dstt, xx_thre, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);}//二值化图像		| THRESH_OTSU
  // else {threshold(dst, dstt, xx_thre, 255, CV_THRESH_BINARY);} //二值化图像		| THRESH_OTSU
  cv::GaussianBlur(dst,dst,kernelSize,0);


  cv::Mat sobelX, sobelY;
  cv::Sobel(dst, sobelX, CV_16S, 1, 0);
  cv::Sobel(dst, sobelY, CV_16S, 0, 1);

    // 转换为绝对值
  cv::Mat sobelXAbs, sobelYAbs;
  cv::convertScaleAbs(sobelX, sobelXAbs);
  cv::convertScaleAbs(sobelY, sobelYAbs);

    // 整幅图的一阶边缘
  dstt = sobelXAbs + sobelYAbs;
  cv::threshold(dstt, dstt, 180, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  cv::medianBlur(dstt,dstt,5);
  cv::imwrite("/home/ucar/ucar_ws_copy/src/ucar_camera/pic/test2.jpg", dstt);
 
  Get_Pixle(dstt); //转换为二维黑白数组

  get_route_one();
  get_route_all();

  cor_flag = cor_find();
//  
//  l = Row - 2, r = Row - 2;
//  while(!leftlineflag[l--]);
//  while(!rightlineflag[r--]);

//  cout << "left is " << l << "right is " << r << endl;
//画图
  if(draw){
  	 cvtColor(dstt, dstt, cv::COLOR_GRAY2RGB);     //转回三通道
	  for(int i = Row - 1;i >= endline;i--){
	  	circle(dstt, Point(midline[i], i), 0.1, Scalar(0, 255, 0), -1);
		  circle(dstt, Point(leftline[i], i), 0.1, Scalar(255, 0, 0), -1);
		  circle(dstt, Point(rightline[i], i), 0.1, Scalar(0, 0, 255), -1);
	  }
  	flip(dstt, dstt, 1);
  }
 
  
//  get_route();
//
//  get_deviation();
//
//  Track_Line();

  return dstt;
}

int cap_init = 1;

void Callback(const std_msgs::String::ConstPtr& msg)
{
    Mat res;
    std_msgs::Int8 result;
    static ros::NodeHandle n;  
    static ros::Publisher pub = n.advertise<std_msgs::Int8>("xx_msg", 1);
    static int cor_ctr_flag = 0;
    if(msg->data == "2" && cap_init){
    	  capture.open("/dev/video0");
    	  // capture.set(CAP_PROP_BUFFERSIZE,1);
    	//   capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
      //  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
       double rate = capture.get(CAP_PROP_FPS);
       double width = capture.get(CAP_PROP_FRAME_WIDTH);
       double height = capture.get(CAP_PROP_FRAME_HEIGHT);
       std::cout << "Camera Param: frame rate = " << rate << " width = " << width
             << " height = " << height << std::endl;

      while(!capture.isOpened());
	      cap_init = 0;
	  
      result.data = 60;
	      pub.publish(result);
	  
    }
    else{
	    	capture.grab();
	    if (!capture.retrieve(frame)){
	     	std::cout << "no video frame" << std::endl;
	  	}
	
		res = ImageProcess(frame);
//	     cout << cor_ctr_flag << endl;
	     if(!cor_flag && cor_ctr_flag)cor_ctr_flag--;
		if(cor_flag && !cor_ctr_flag){
			result.data = 50;
			pub.publish(result);
			cor_ctr_flag = 3;
			cout << "find cor!!!" << endl;
		}
		else{
//			int min = l > r?r:l;
        ROS_INFO("2");
		  	result.data = midline[endline + view] - 128+bias> 128?128:midline[endline + view] - 128 + bias;
		  	cout << result.data << endl;
        ROS_INFO("%d",result.data);
			pub.publish(result);
//			cout << "no cor" << endl;
		}
 
	    	if(draw)imshow("result", res);
 
	    	waitKey(1);
    }
    
}

// void read_json(void){
// 	Json::Reader reader;
// 	Json::Value root;
 
// 	//从文件中读取，保证当前文件有demo.json文件  
// 	ifstream in("/home/ucar/ucar_ws_copy/src/ucar_camera/src/config.json", ios::binary);
 
// 	if (!in.is_open())
// 	{
// 		cout << "Error opening file\n";
// 		return;
// 	}
 
// 	if (reader.parse(in, root))
// 	{
// 		//读取根节点信息  
// 		endline = root["endline"].asInt();
// 		xx_thre = root["xx_thre"].asInt();
// 		view = root["view"].asInt();
//           cor_find_Row = root["cor_find_Row"].asInt();
//           cor_find_Row_Start = root["cor_find_Row_Start"].asInt();
//           bias = root["bias"].asInt();
//           draw = root["draw"].asInt();
//     OTSU = root["OTSU"].asInt();
          
// 		cout << "My endline is " << endline << endl;
// 		cout << "My xx_thre is " << xx_thre << endl;
// 		cout << "My view is " << view << endl;
// 		cout << "My bias is " << view << endl;
// 		cout << "My cor_find_Row is " << cor_find_Row << endl;
// 		cout << "My cor_find_Row_Start is " << cor_find_Row_Start << endl;
		
// 	}
// 	else
// 	{
// 		cout << "parse error\n" << endl;
// 	}
 
// 	in.close();

// }



int main(int argc, char* argv[])
{
              
    // namedWindow("result", WINDOW_NORMAL);
    // resizeWindow("result", 640, 480);
    // resizeWindow("frame", 640, 480);

    // read_json();
    endline = 30;
    xx_thre = 180;
    view = 15;//9
    cor_find_Row = 30;
    cor_find_Row_Start = 40;
    bias = 0;
    draw = 0;
    OTSU = 0;
    ros::init(argc, argv, "xunxian");  
    ros::NodeHandle nh; 
    
    // 订阅者设置  
    ros::Subscriber sub = nh.subscribe("start_xx", 1, Callback);
    ros::spin();
//    while(1){
//    	  if (!capture.read(frame)){
//     	std::cout << "no video frame" << std::endl;
//     	continue;
//  	  }
//  	  
//	  res = ImageProcess(frame);
//	  
//    	  imshow("result", res);
//    	  waitKey(1);
//    }
   return 0;
}