#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"
#include "ztestnav2025/traffic_light.h"
#include "line_follow/line_follow.h"
#include "qr_01/qr_srv.h"
#include "communication/msg_1.h"
#include "communication/msg_2.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

#include <cmath>
//找板优化新增头文件
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 全局变量定义
int room_index = 0;       // 当前房间号
int awake_flag = 0;      // 语音唤醒标志位

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void publishInitialPose(double x,double y,double yaw,tf2::Quaternion &q,ros::Publisher& initial_pose_pub_ ) {
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    
    // 设置header
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.header.frame_id = "map";  // 坐标系设置为map
    
    // 设置位置
    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.position.z = 0.0;
    
    // 设置方向
    q.setRPY(0, 0, yaw);
    initial_pose.pose.pose.orientation.x = q.x();
    initial_pose.pose.pose.orientation.y = q.y();
    initial_pose.pose.pose.orientation.z = q.z();
    initial_pose.pose.pose.orientation.w = q.w();
    
    // 设置协方差矩阵
    boost::array<double, 36> covariance = {{
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0076
    }};
    initial_pose.pose.covariance = covariance;
    
    // 发布初始位置
    initial_pose_pub_.publish(initial_pose);
    ROS_INFO("已发布初始位置: x=%.2f, y=%.2f, yaw=%.2f",
            initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y,yaw);
}

class AwakeDetector {
public:
    AwakeDetector(ros::NodeHandle& nh) : nh_(nh), awake_received_(false) {
        // 订阅awake_flag话题
        sub_ = nh_.subscribe("/awake_flag", 10, &AwakeDetector::awakeCallback, this);
    }

    // 等待唤醒信号
    bool waitForAwake(tf2::Quaternion &q,ros::Publisher& initial_pose_pub_ ) {
        ROS_INFO("等待语音唤醒信号...");
        ros::Rate rate(2);  // 10Hz检查频率
        ros::Time awake_limit = ros::Time::now();//防止超时
        while (ros::ok() && !awake_received_) {
            ros::spinOnce();
            rate.sleep();
            publishInitialPose(0.25,0.25,0,q,initial_pose_pub_ );
            // if((ros::Time::now()-awake_limit).toSec()>40){
            //     break;
            // }
        }
        
        if (awake_received_) {
            ROS_INFO("已接收到唤醒信号!");
            return true;
        }
        return false;
    }

private:
    void awakeCallback(const std_msgs::Int8ConstPtr& msg) {
        if (msg->data == 1) {
            awake_received_ = true;
            ROS_INFO("检测到唤醒信号 (awake_flag=1)");
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool awake_received_;
};

class Sim_talkto_car{
public:
    Sim_talkto_car(ros::NodeHandle& nh):
        nh_(nh),
        pub_(nh.advertise<communication::msg_1>("send_class", 10)),
        sub_(nh.subscribe<communication::msg_2>("send_room_class",10,&Sim_talkto_car::simreturn, this))
    {}

    bool sim_done = 0;
    int sim_room = -1;
    int sim_detect_class = 0;
    void simreturn(const communication::msg_2::ConstPtr& sim_result){
        ROS_INFO("房间编号：%d,检测类型：%d", sim_result->room, sim_result->detected_class);
        sim_room = sim_result->room;
        sim_detect_class = sim_result->detected_class;
        sim_done = 1;
        return;
    }
    
    void car_msg_publish(int target){
        target_msg_.target_class = target;
        pub_.publish(target_msg_);
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    communication::msg_1 target_msg_;
    ros::Subscriber sub_;
};

void goal_set(move_base_msgs::MoveBaseGoal &goal,double x,double y,double yaw,tf2::Quaternion q){
    q.setRPY(0, 0, yaw);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
}

void go_destination(move_base_msgs::MoveBaseGoal &goal,double x,double y,double yaw,tf2::Quaternion &q,MoveBaseClient &ac){
    goal.target_pose.header.stamp = ros::Time::now();
    goal_set(goal,x,y,yaw,q);
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("到达目标");
    else
        ROS_INFO("无法到达目标");
}

bool checkTrafficLightWithSearch(ros::Publisher& cmd_pub) {
    bool greenLightFound = false;
    ros::Time startTime = ros::Time::now();
    // 设置发布频率为10Hz
    ros::Rate control_rate(10);
    
    // 初始化速度消息
    geometry_msgs::Twist twist_msg;
    
    // 循环检测，直到超时或找到绿灯
    while (ros::ok()) {
        double elapsedTime = (ros::Time::now()- startTime).toSec();
        
        // 每次循环都检测红绿灯状态
        int lightStatus = detectTrafficLightStatus();
        
        // 发现红灯，立即停止并返回false
        if (lightStatus == 1) {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_pub.publish(twist_msg);  // 发布停止指令
            return false;
        }
        
        // 发现绿灯，立即停止并返回true
        if (lightStatus == 2) {
            greenLightFound = true;
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_pub.publish(twist_msg);  // 发布停止指令
            return true;
        }
        
        // 控制平移阶段
        if (elapsedTime < 2.0) {
            // 前2秒左移（y轴负方向）
            twist_msg.linear.x = 0.0;     // 禁止前后移动
            twist_msg.linear.y = -0.2;    // 左移速度
            twist_msg.angular.z = 0.0;    // 禁止旋转
        } 
        else if (elapsedTime < 6.0) {
            // 接下来4秒右移（y轴正方向）
            twist_msg.linear.x = 0.0;     // 禁止前后移动
            twist_msg.linear.y = 0.2;     // 右移速度
            twist_msg.angular.z = 0.0;    // 禁止旋转
        } 
        else {
            // 超时（超过6秒），停止移动
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_pub.publish(twist_msg);  // 确保发送停止指令
            break;
        }
        
        // 发布速度指令
        cmd_pub.publish(twist_msg);
        
        // 按照10Hz频率休眠
        control_rate.sleep();
        ros::spinOnce();  // 处理可能的回调
    }
    
    // 超时后再次检查红绿灯状态
    return detectTrafficLightStatus() == 2;
}

bool go_enter2(ros::ServiceClient& poseget_client,ros::Publisher& cmd_pub){
    ztestnav2025::getpose_server pose;
    pose.request.getpose_start = 1;
    poseget_client.call(pose);
    geometry_msgs::Twist twist_msg;
    while(ros::ok()){
        if(pose.response.pose_at[0]<3.7){
            twist_msg.linear.y = -0.6;
        }
        else if(pose.response.pose_at[0]>4.5){
            twist_msg.linear.y = 0.3;
        }
        else if(pose.response.pose_at[0]>=3.7 && pose.response.pose_at[0]<4.2){
            twist_msg.linear.y = pose.response.pose_at[0]-4.25;
        }
        else if(pose.response.pose_at[0]>=4.3 && pose.response.pose_at[0]<4.5){
            twist_msg.linear.y = pose.response.pose_at[0]-4.25;
        }
        else{
            twist_msg.linear.y = 0;
        }

        if(pose.response.pose_at[1]<4.0){
            twist_msg.linear.x = 0.2;
        }
        else if(pose.response.pose_at[1]>4.7){
            twist_msg.linear.x = -0.1;
        }
        else if(pose.response.pose_at[1]>=4.0 && pose.response.pose_at[1]<4.45){
            twist_msg.linear.x = 4.5-pose.response.pose_at[1];
        }
        else if(pose.response.pose_at[1]>=4.55 && pose.response.pose_at[1]<4.7){
            twist_msg.linear.x = 4.5-pose.response.pose_at[1];
        }
        else{
            twist_msg.linear.x = 0;
        }

        if(pose.response.pose_at[2]<1.4){
            twist_msg.angular.z = 0.3;
        }
        else if(pose.response.pose_at[2]>1.74){
            twist_msg.angular.z = -0.3;
        }
        else if(pose.response.pose_at[2]>=1.4 && pose.response.pose_at[2]<1.5){
            twist_msg.angular.z = 1.62-pose.response.pose_at[2];
        }
        else if(pose.response.pose_at[2]>=1.64 && pose.response.pose_at[2]<1.74){
            twist_msg.angular.z = 1.52-pose.response.pose_at[2];
        }
        else{
            twist_msg.angular.z = 0;
        }
        cmd_pub.publish(twist_msg);
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    std::map<int, std::string> name = {
        {0, "辣椒"},
        {1, "番茄"},
        {2, "土豆"},
        {3, "香蕉"},
        {4, "苹果"},
        {5, "西瓜"},
        {6, "可乐"},
        {7, "蛋糕"},
        {8, "牛奶"}
    };
    ROS_INFO("省赛主干代码开始，初始化对象，等待服务中"); 
    //-----------------------------------初始化movebase，实例对象---------------------------//
    ros::init(argc,argv,"myplannernav");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true); 
    tf2::Quaternion q;  
    //等待action回应
    while(!ac.waitForServer()){//这里之前只有等待五秒,
        ROS_INFO("等待movebase服务中---");
    } 
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    //实例对象初始化
    MecanumController mecanumController(nh);
    //客户端初始化
    ROS_INFO("等待lidar_process服务中---");
    ros::ServiceClient client_find_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
    ztestnav2025::lidar_process where_board;
    client_find_board.waitForExistence();
    ROS_INFO("等待二维码服务中---");
    ros::ServiceClient client_qr = nh.serviceClient<qr_01::qr_srv>("qr_detect");
    qr_01::qr_srv what_qr;
    int board_class = 0;
    client_qr.waitForExistence();
    ROS_INFO("等待坐标获取服务中---");
    ros::ServiceClient poseget_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
    ztestnav2025::getpose_server pose_result;
    pose_result.request.getpose_start = 1;
    poseget_client.waitForExistence();
    ROS_INFO("等待视觉巡线服务中---");
    ros::ServiceClient line_client1 = nh.serviceClient<line_follow::line_follow>("line_right");
    ros::ServiceClient line_client2 = nh.serviceClient<line_follow::line_follow>("line_left");
    ros::ServiceClient line_client3 = nh.serviceClient<line_follow::line_follow>("right2");
    ros::ServiceClient line_client4 = nh.serviceClient<line_follow::line_follow>("left2");
    line_follow::line_follow linefollow_start;
    linefollow_start.request.line_follow_start = 1;
    line_client4.waitForExistence();
    //发布话题以供仿真通信
    Sim_talkto_car sim_talkto_car(nh);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //清理代价地图和重新amcl定位
    ros::ServiceClient clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    ros::Publisher initial_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    ros::Publisher audio_pub = nh.advertise<std_msgs::Int32>("/audio_alert", 10, true);
    std_msgs::Int32 msg;

    ros::ServiceClient node_client = nh.serviceClient<std_srvs::Trigger>("/kill_node");
    std_srvs::Trigger node_srv;

    //--------------------------------------语音唤醒等待--------------------------------//
    AwakeDetector awakeDetector(nh);
    if (!awakeDetector.waitForAwake(q,initial_pose_pub_ )) {
        ROS_ERROR("未接收到唤醒信号，程序退出");
        return 1;
    }
    ROS_INFO("已唤醒，开始执行任务...");

    //--------------------------------------走廊环境导航，发布目标点--------------------------------//
    ROS_INFO("走廊环境导航开始");
    go_destination(goal,1.25,0.75,3.14,q,ac);
    // ros::Duration(0.5).sleep();
    what_qr.request.qr_start = 1;
    while(ros::ok()){
        if (!client_qr.call(what_qr)){
            ROS_INFO("没有请求到服务");
        }
        board_class = what_qr.response.qr_result;
        ROS_INFO("二维码结果:%d",board_class);
        if (board_class>0){
            ROS_INFO("二维码结果:%d",what_qr.response.qr_result);
            break;
        }
        else{
            ROS_ERROR("请求二维码失败");
        }
    }
    // //board_class 为 1（蔬菜）、2（水果）、3（甜品）
    if (board_class >= 1 && board_class <= 3  ) {
        msg.data = board_class-1;
        audio_pub.publish(msg);
        ros::Time test = ros::Time::now();
        play_audio(voice[0][board_class-1]);
        ROS_INFO("播报耗时%f",(ros::Time::now()-test).toSec());
    }

    // go_destination(goal,0.50,2.25,3.14,q,ac);
    go_destination(goal,1.25,3.75,0,q,ac);
    ROS_INFO("走廊环境导航完成");


    //----------------------------------------目标检测区域开始-------------------------------------------//
    ROS_INFO("拣货区域任务开始");
    int board_name;
    bool flag=false;//判断到达与否
    //第一点视觉识别
    //视觉识别开始，先传个-1把摄像头打开
    publishInitialPose(1.25,3.75,0,q,initial_pose_pub_ );
    std::vector<std::vector<int>> a = {{-1},{-1},{-1},{-1},{-1},{-1}};
    mecanumController.detect(a,-1);
    
    //然后去中间，识别目标，或者定位遮挡视野的板子
    double targetx, targety, targetz, targetx2, targety2, targetz2;
    bool target2flag = false,targetflag = false,use_forward = false;
    if(mecanumController.turn_and_find_plus(17,board_class,0.52,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1)){
        if (!use_forward)
        {
            if (std::min(abs(targetx2-0),abs(targetx2 - 2.5))>0.4 && std::min(abs(targety2 - 2),abs(targety2 - 5))>0.4)//终点太靠墙直接视觉过去也能避开
            {
                ROS_INFO ("有障碍物，先绕行");
                go_destination(goal, targetx2, targety2, targetz2, q, ac);
            }
            else{
                ROS_INFO ("障碍物较远,直接前进");

            }
        }
        board_name = mecanumController.forward_and_adjust(board_class,0.35);
        if(board_name<0){//出现这种情况，比较糟糕，要么是路被封死了，要么是走一半目标丢了
            if(mecanumController.turn_and_find_plus(17,board_class,0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1)){
                where_board.request.lidar_process_start = 4;
                client_find_board.call(where_board);
                std::vector<float> position = mecanumController.getCurrentPose();
                ROS_INFO("出现了比较糟糕的情况，旋转找到了，前进失败，板子%f",where_board.response.lidar_results[0]);
                ROS_INFO("定位%f,%f,%f",position[0],position[1],position[2]);
                if(where_board.response.lidar_results[1]<0){
                    ROS_INFO("不能直线前进，用movebase");//需要再次请求雷达服务，准确计算目的地
                    where_board.request.lidar_process_start = 5;
                    client_find_board.call(where_board);
                    geometry_msgs::PointStamped scan_point;
                    scan_point.header.frame_id = "base_link";
                    scan_point.header.stamp = ros::Time(0); // 或使用对应的时间，如果使用ros::Time(0)则用最新时间
                    scan_point.point.x = where_board.response.lidar_results[1];
                    scan_point.point.y = where_board.response.lidar_results[2];
                    scan_point.point.z = 0.0;
                    geometry_msgs::PointStamped output_point;
                    try {
                        mecanumController.tf_buffer_.transform(scan_point, output_point, "map");
                        ROS_INFO("map下目的地坐标: (%.2f, %.2f)",output_point.point.x, output_point.point.y);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_ERROR("坐标系变换失败: %s", ex.what());
                    }
                    go_destination(goal,output_point.point.x,output_point.point.y,where_board.response.lidar_results[3]+position[2],q,ac);
                    mecanumController.cap_buffer_clear();
                    board_name = mecanumController.forward_and_adjust(board_class,0.5);
                    if(board_name<0){
                        ROS_ERROR("拣货失败");
                    }
                    else{
                        flag=true;
                    }
                }
            }
            else{//板子被挡了，中间看不到
                if(targetflag){
                    double passx, passy, passz, passx2, passy2, passz2;
                    bool find1,find2;
                    ROS_INFO("前往%f,%f,%f",targetx,targety,targetz);
                    go_destination(goal,targetx,targety,targetz,q,ac);
                    mecanumController.cap_buffer_clear();
                    if(mecanumController.turn_and_find_plus(4.25,board_class,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(board_class,0.5);
                        flag=true;
                    }
                    else if(mecanumController.turn_and_find_plus(8.5,board_class,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(board_class,0.5);
                        flag=true;
                    }
                }
                if(target2flag && !flag){
                    double passx, passy, passz, passx2, passy2, passz2;
                    bool find1,find2;
                    ROS_INFO("前往%f,%f,%f",targetx2, targety2, targetz2);
                    go_destination(goal,targetx2, targety2, targetz2,q,ac);
                    mecanumController.cap_buffer_clear();
                    if(mecanumController.turn_and_find_plus(4.25,board_class,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(board_class,0.5);
                        flag=true;
                    }
                    else if(mecanumController.turn_and_find_plus(8.5,board_class,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(board_class,0.5);
                        flag=true;
                    }
                }
            }
        }
        else{
            flag=true;
        }
    }
    else{//板子被挡了，中间看不到
        if(targetflag){
            double passx, passy, passz, passx2, passy2, passz2;
            bool find1,find2;
            ROS_INFO("前往%f,%f,%f",targetx,targety,targetz);
            go_destination(goal,targetx,targety,targetz,q,ac);
            mecanumController.cap_buffer_clear();
            if(mecanumController.turn_and_find_plus(5.0,board_class,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(board_class,0.5);
                flag=true;
            }
            else if(mecanumController.turn_and_find_plus(11.0,board_class,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(board_class,0.5);
                flag=true;
            }
        }
        if(target2flag && !flag){
            double passx, passy, passz, passx2, passy2, passz2;
            bool find1,find2;
            ROS_INFO("前往%f,%f,%f",targetx2, targety2, targetz2);
            go_destination(goal,targetx2, targety2, targetz2,q,ac);
            mecanumController.cap_buffer_clear();
            if(mecanumController.turn_and_find_plus(5.0,board_class,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(board_class,0.5);
                flag=true;
            }
            else if(mecanumController.turn_and_find_plus(11.0,board_class,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(board_class,0.5);
                flag=true;
            }
        }
    }
    //如果上面的逻辑都没能找到板，就前往出口和入口旋转找板
    if (!flag)
    {
        ROS_INFO("前往出口处找板");
        go_destination(goal, 2.25, 4.25, 1.1, q, ac);
        mecanumController.cap_buffer_clear();
        targetflag = false; target2flag = false; // 重置标志位
        if(mecanumController.turn_and_find_plus(7, board_class, 0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1))
        {
            board_name = mecanumController.forward_and_adjust(board_class,0.35);
            if(board_name >= 0)
            {
                flag = true;
            }
        }

    }
    if (!flag)
    {
        ROS_INFO("前往入口处找板");
        go_destination(goal, 0.3, 2.25, 0, q, ac);
        mecanumController.cap_buffer_clear();
        targetflag = false; target2flag = false; // 重置标志位
        if(mecanumController.turn_and_find_plus(7, board_class, 0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1))
        {
            board_name = mecanumController.forward_and_adjust(board_class,0.35);
            if(board_name >= 0)
            {
                flag = true; 
            }
        }

    }

    if (!flag){
        ROS_INFO("找不到板子，直接走了");
    }
    mecanumController.cap_close();
    ROS_INFO("%d",board_name);
    if (board_name >= 0 && board_name <= 9 && flag==1) {
        msg.data = board_name + 3;
        audio_pub.publish(msg);
        ros::Time test = ros::Time::now();
        play_audio(voice[1][board_name]);
        ROS_INFO("播报耗时%f",(ros::Time::now()-test).toSec());
    }
    
    //-----------------------------------------------仿真开始--------------------------------------------//
    ROS_INFO("前往仿真区域");
    go_destination(goal,1.25,3.75,0.0,q,ac);
    //发送仿真消息
    ros::Rate rate(1);
    bool shut_down = false;
    // while (ros::ok()) {
    //     sim_talkto_car.car_msg_publish(board_name);
    //     rate.sleep();
    //     ros::spinOnce();
    //     publishInitialPose(1.25,3.75,0.0,q,initial_pose_pub_ );
    //         if(!shut_down){
    //             if (node_client.call(node_srv)) {
    //                 if (node_srv.response.success) {
    //                     ROS_INFO("成功: %s", node_srv.response.message.c_str());
    //                     shut_down = true;
    //                 } else {
    //                     ROS_WARN("失败: %s", node_srv.response.message.c_str());
    //                 }
    //             } else {
    //                 ROS_ERROR("服务调用失败");
    //             }
    //             if (clear_costmaps_client.call(srv)) { // 发送请求
    //                 ROS_INFO("清理代价地图");
    //             } else {
    //                 ROS_ERROR("清理失败");
    //             }
    //         }
    //     if(sim_talkto_car.sim_done==1){
    //         break;
    //     }
    // }
    // if(sim_talkto_car.sim_room>=0){
    //     msg.data = sim_talkto_car.sim_room + 11;
    //     audio_pub.publish(msg);
    //     play_audio(voice[2][sim_talkto_car.sim_room-1]);
    // }
    // else {
    //     ROS_INFO("仿真失败");
    // }
    

    //--------------------------------------------前往红绿灯识别区域--------------------------------------------//
    bool enter1 = true;
    ROS_INFO("前往红绿灯区域路口1");
    go_destination(goal,3.25,4.50,1.57,q,ac);  
    if (checkTrafficLightWithSearch(cmd_pub)){
        // ROS_INFO("路口1可通过");
        msg.data = 15;
        audio_pub.publish(msg);
        play_audio(voice[3][0]);
        // go_destination(goal,2.83,3.5,-1.18,q,ac);
        go_destination(goal,2.75,3.6,-1.57,q,ac);
    } 
    else {
        ROS_INFO("前往红绿灯区域路口2");
        enter1 = false;
        go_destination(goal,4.25,4.50,1.57,q,ac);
        // go_enter2(poseget_client, cmd_pub);
        if (checkTrafficLightWithSearch(cmd_pub)){
            // ROS_INFO("路口2可通过");
            msg.data = 16;
            audio_pub.publish(msg);
            play_audio(voice[3][1]);
            // go_destination(goal,4.75,3.44,-1.86,q,ac);
            go_destination(goal,4.75,3.6,-1.57,q,ac);
        } 
        else {
            ROS_WARN("两个路口均未找到绿灯，执行备选方案通过路口2");
            play_audio(voice[3][1]);
            go_destination(goal,4.75,3.44,-1.86,q,ac);
        }
    }

    //-----------------------------------------视觉巡线---------------------------------------------//
    if(enter1){
        if(line_client1.call(linefollow_start)){
            ROS_INFO("省赛右边视觉巡线结束");
        }
        else{
            ROS_ERROR("视觉巡线失败....");
        }
    }
    else{
        if(line_client2.call(linefollow_start)){//
            ROS_INFO("省赛左边视觉巡线结束");
        }
        else{
            ROS_ERROR("视觉巡线失败....");
        }
    }

    msg.data = 17 + board_name*3 + sim_talkto_car.sim_detect_class%3;
    audio_pub.publish(msg);
    play_audio(voice[4+board_name][sim_talkto_car.sim_detect_class-board_name/3*3]);
    ros::spin();

    return 0;
}

