#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <iostream>

using namespace std;

ros::Publisher voice_mode_pub;
std_msgs::Int8 voice_mode;


void voice_words_callback(const std_msgs::String& msg)
{
	/***语音指令***/
	string str = msg.data.c_str();    //取传入数据
	string str0 = "小飞小飞";
	
	
	if(str == str0){
		voice_mode.data = 0;
		voice_mode_pub.publish(voice_mode);
		system("play /home/ucar/ucar_ws_copy/src/speech_command/audio/sound/tts_1.wav");
	}
	
}






int main(int argc, char** argv)
{
	ros::init(argc, argv, "voice_test");
	ros::NodeHandle n;

	voice_mode_pub = n.advertise<std_msgs::Int8>("/voice_mode", 1000);
	/***创建离线命令词识别结果话题订阅者***/
	ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback);

	ros::Rate loop_rate(100);



	while(ros::ok())
	{
	  
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}