#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <boost/algorithm/string.hpp>  
#include <tf/transform_listener.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


struct pathPoint {
    double x;
    double y;
    double yaw;
};

std::string toLower(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return lower_str;
}

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

class Robot {
public:
    Robot(ros::NodeHandle& nh):
    // 构造函数,加载move_base服务端，一些订阅者和发布者
    /*
        argc:
            nh: ros节点句柄(ros::NodeHandle)
        return:
            None
    */

    nh_(nh),

    move_base_client_("move_base", true)
    {
        ROS_INFO("initialize ing");
        // 等待move_base Action服务器启动
        ROS_INFO("wait move_base Action server...");
        move_base_client_.waitForServer();
        ROS_INFO("move_base Action already linked。");
        voiceFlag = false;
        voice_sub_ = nh_.subscribe<std_msgs::Int32>("/angle", 5, &Robot::voiceCallback, this);
        getPoint();
        ro_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        strait_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        yd_loc_control = nh_.advertise<std_msgs::Int8>("start_yd", 1);//发布雷达检测消息
        reinitial_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
        yd_control.data = 0;

        Point_number = 0;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 1.0;
        task_map["toQC"] = 1;   // 去二维码采集任务
        task_map["toMidFrame"] = 2; // 去中央白色线框
        task_map["collectPic"] = 3; // 采集、播报,并返回到中央白框
        task_map["sim"] = 4;  // 仿真任务，包括播报
        task_map["navTrack"] = 5;  // 通过红绿灯标识，进入巡线任务
        cur_task = "toQC";

        // 初始化仿真任务通信接口
        car_to_pc_pub_ = nh_.advertise<std_msgs::String>("/car_to_pc", 10);
        pc_to_car_sub_ = nh_.subscribe("/pc_to_car", 10, &Robot::simPCCallback, this);
    }

    ~Robot() { }

    // 任务方法声明

    void getPoint(int num = 9);
    void voiceCallback(const std_msgs::Int32::ConstPtr& msg);
    void navToGoal(int Point_number);
    void toQC();
    void ewmCallback(const std_msgs::String::ConstPtr& msg);
    void toMidFrame();
    void collectPic();
    void ydCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void viCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void navTrack();
    void sim();
    // 新增私有回调函数
    void simPCCallback(const std_msgs::String::ConstPtr& msg);
    // 新增私有处理函数
    void processDetectedItems();
    // 任务主程序
    void run();
private:
    std::mutex mutex_;                // 互斥量，保护共享数据
    bool callback_triggered;           // 用于标识回调是否已触发
    // ROS 节点句柄
    ros::NodeHandle nh_;
    
    // move_base Atcion Client
    MoveBaseClient move_base_client_;
    // 导航的目标地点
    move_base_msgs::MoveBaseGoal goal;
    // 目标点数组
    pathPoint point[9];
    // 当前的目标点序号
    int Point_number;

    // 旋转参数，将用于ro_pub
    geometry_msgs::Twist twist;
    
    // 板前定位控制信号
    std_msgs::Int8 yd_control;
    // 保存雷达的原始数据
    sensor_msgs::LaserScan::ConstPtr laserCloudIn;

    // 存储可能目标物品点的位置信息
    double goal_p_list[8][4];
    // 记录扫描的障碍物的数量
    int ob_num = 0;
    // ROS 发布者和订阅者
    ros::Publisher ro_pub;  // 旋转的发布者
    ros::Publisher strait_pub; // 直行的发布者
    ros::Publisher yd_loc_control;
    ros::Publisher reinitial_pub;
    ros::Subscriber voice_sub_;
    ros::Subscriber ewm_sub_;
    ros::Subscriber yd_sub;
    ros::Subscriber yd_sub2;
    ros::Subscriber yd_sub3;  // 这个是采取雷达原始数据的
    ros::Subscriber vi_sub;

    //仿真
    ros::Publisher car_to_pc_pub_;
    ros::Subscriber pc_to_car_sub_;


    // 视觉识别
    // 由话题/nanodet/detect管理的检测结果将会发布数组下标
    const std::vector<std::string> class_names = {"Watermelon", "Cake", "Apple", "Banana",
     "Chili", "Tomato", "Milk", "Cola", "Potato"};

    // 任务图，字符串->任务序号
    std::map<std::string, int> task_map;
    std::string cur_task;
    // 语音唤醒标志位
    bool voiceFlag;

    // 等待采集的物品
    std::string thing;
    // 采集到的物品
    std::string rething;
    // 视觉判断数值
    bool target_found = false;
    // 判断当前位置是否能扫到目标板
    bool is_kind_point = false;
    double target_pixel_x;
    //巡线
    int finish;
    //红绿灯值
    int light;
    bool is_avoiding = false;
    bool avoid_done = false;
    int zhang =0;
    int zf;
    double x,y;

    //仿真变量相关#
    std::vector<std::string> detected_items;  // 从PC端获取的检测物品列表
    int sim_room = -1;                       // 匹配到的目标物品所在房间号，-1表示未匹配
    std::string sim_thing;                    // 匹配到的目标物品名称
    bool nav_complete = false;                // 仿真任务完成标志
    bool pc_ready = true; 

};


void Robot::getPoint(int num) {
    // 从参数服务器读取路经点的参数
    /*
        argc:
            num: 路径点的个数(int)(具有默认参数7)
        return:
            None
    */
    std::vector<double> point_coords;
    std::string param_name = "point0/x_y_yaw";
    for(int i = 0; i < num; i++) {
        param_name[5] = i + '0';
        if(nh_.getParam(param_name, point_coords)) {
            if(point_coords.size() == 3) {
                point[i].x = point_coords[0];
                point[i].y = point_coords[1];
                point[i].yaw = point_coords[2];
            }
            else {
                ROS_ERROR("param %s err,expect 3 double values,but %zu。",
                param_name.c_str(), point_coords.size());
            }

        }
        else {
            ROS_ERROR("cant get param %s。", param_name.c_str());
        }
    }
}


void Robot::voiceCallback(const std_msgs::Int32::ConstPtr& msg) {
    // 语音唤醒回调函数，当 /angle 的数据不为0时，将 voidFlag 设置为 true
    // 一旦 voidFlag 设置为 true, speech_command_node 也会被杀掉
    /*
        argc:
            msg: 语音标志位的数据(std_msgs::Int32::ConstPtr)
        return:
            None
    */
    if(msg->data != 0) {
        voiceFlag = true;
        ROS_INFO("voiceFlag %d", voiceFlag);
        // 显式调用 shutdown() 关闭订阅者
        if (voice_sub_) { // 这是一个好的做法，检查对象是否有效再调用 shutdown
            voice_sub_.shutdown();
            ROS_INFO("voice_sub_ shut down successfully.");
            } else {
            ROS_WARN("voice_sub_ was already shut down or invalid.");
        }
    }
}


void Robot::navToGoal(int Point_number) {
    // 导航到一个预设的目标点 在 config/pathPoint.yaml 文件中设置
    /*
        argc:
            Point_number: 目标点的序号，即src/ucar_all_control/config/pathPoint.yaml中的(int)
        return;
            None
    */
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = point[Point_number].x;
    goal.target_pose.pose.position.y = point[Point_number].y;
    goal.target_pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(point[Point_number].yaw);
    goal.target_pose.pose.orientation = goal_quat; 
    move_base_client_.sendGoal(goal);
    move_base_client_.waitForResult(ros::Duration(10.0));
    if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("I am now in point %d", Point_number);
        return;
    }
    else {
        ROS_ERROR("NAV ERR");
        return;
    }
}


void Robot::ewmCallback(const std_msgs::String::ConstPtr& msg) {
    // 二维码识别任务，识别成功后会把对应的字符串传递给thing(std::string)
    // 如果已经成功识别了，还会把 /ewm 的发布节点杀掉
    /*
        argc:
            msg: /ewm 话题下的数据(std_msgs::String::ConstPtr)
        return:
            None
    */
    if (!msg->data.empty()) {
        thing = msg->data.c_str();  // 正确地赋值给 std_msgs::String
        ROS_INFO("Received task thing: %s", thing.c_str());
        // 简单地通过系统调用杀掉ewm节点(好吧其实没有杀掉)
        // 获取数据后关闭订阅器
        if (ewm_sub_) {
            ewm_sub_.shutdown();
            ROS_INFO("ewm_sub_ shut down after receiving task.");
        }
    }
}


void Robot::toQC() {
    // 前往二维码前，并播报本次采集任务
    // 识别成功后会把 thing 设置为 Dessert Fruit Vegetable 中的一个
    /*
        Argc: 
            None
        Return:
            None
    */
    ROS_INFO("------ 开始toQC -------");
    Point_number = 0;
    navToGoal(Point_number);
    Point_number++;
    ewm_sub_ = nh_.subscribe<std_msgs::String>("/ewm", 10, &Robot::ewmCallback, this);
    ros::Rate loop_rate(10);
    ros::Time start_time = ros::Time::now();
    ros::Duration duration(5.0);
    while((ros::Time::now() - start_time) <= duration && ros::ok()) {
        if(!thing.empty()) {
            break;
        }
        ros::spinOnce();  // 在5s内不断回调ewm识别
        loop_rate.sleep();
    }
    std::string command = "rosnode kill ewm";
    int result = system(command.c_str());
    if(result) {
        ROS_INFO("I kill the ewm");
    }
    ROS_INFO("I get %s", thing.c_str());
    if(thing == "Dessert") {
        ROS_INFO("yes I got Dessert");
        system("aplay /home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/dessert_x.wav");
    }
    else if(thing == "Fruit") {
        ROS_INFO("yes I got Fruit");
        system("aplay /home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/fruit_x.wav");
    }
    else {
        system("aplay /home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/vegetable_x.wav");
    }
    
}


void Robot::toMidFrame() {
    // 到指定旋转点
    /*
        Argc: 
            None
        Return:
            None
    */
    ROS_INFO("------ 开始toMidFrame ------");
    navToGoal(Point_number);
}


void Robot::ydCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    // 雷达回调函数，用于板前定位
    // 将当前 msg 中的航点位姿传递给 goal_p_list
    /*
        Argc: 
            msg(geometry_msgs::PoseArray::ConstPtr) 由 /yd_msg 话题发布
        Return:
            None
    */
    ob_num = msg->poses.size();
    ROS_INFO("ob_num = %d", ob_num);
    for (int j = 0; j < ob_num; j++) {
        goal_p_list[j][0] = msg->poses[j].position.x;
        goal_p_list[j][1] = msg->poses[j].position.y;
        goal_p_list[j][2] = msg->poses[j].orientation.z;
        goal_p_list[j][3] = msg->poses[j].orientation.w;
    }
    callback_triggered = true;  // 设置回调触发标志
    ROS_INFO("race_all get a goal_p_list 0 is: (%lf, %lf)", goal_p_list[0][0], goal_p_list[0][1]);
}


void Robot::viCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 视觉回调函数，用于板前定位
    // msg->data 为一个浮点数数组, [classId1, lx1, ly1, rx1, ry1, classId2, lx2, ly2, rx2, ry2, ..., ]
    // 当车子视野中有目标板
    // 且识别到目的种类的物品时，该函数会把 target_found 设置为 1, 并把目标物体的中心坐标值予以 target_pixel_x
    /*
        Argc: 
            msg(std_msgs::Float64MultiArray::ConstPtr) 由 /nanodet/detect发布
        Return：
            None
    */
    std::lock_guard<std::mutex> lock(mutex_);  // 锁定互斥量
    std::vector<double> vi_msg = msg->data;
    int lenofMsg = msg->data.size();      
    for(int i = 0; i < (lenofMsg) / 5; ++i) {
        rething = class_names[static_cast<int>(msg->data[5*i])];
        target_found = (
            (thing == "Dessert"   && (rething == "Cake" || rething == "Milk" || rething == "Cola")) ||
            (thing == "Fruit"     && (rething == "Watermelon" || rething == "Apple" || rething == "Banana")) ||
            (thing == "Vegetable" && (rething == "Chili" || rething == "Tomato" || rething == "Potato"))
        );
        ROS_INFO("%d",target_found);
        if(target_found) {
            double Px1 = msg->data[5*i+1];
            double Px2 = msg->data[5*i+3];
            target_pixel_x = (Px1 + Px2) / 2;  
            break;
        }
    }
}


void Robot::collectPic() {
    // 从拣货区中心出发，进行任务板子的识别与定位，计算出板前的停靠点，并实现相应的播报，最后返回到仿真区
    // Todo: 当对齐但未雷达识别出板子时，使用简单基于相对位置的拣货方法
    /*  
        Argc:
            None
        Return:
            None
    */
    ROS_INFO("-------- 开始 collectPic 任务, 目标类别: '%s' --------", thing.c_str());
    // 1. 初始化
    ros::param::set("/detect", 1);
    // yd_control.data = 0;
    // yd_loc_control.publish(yd_control);
    nh_.setParam("/locate_pic_ransac/control", 1);
    yd_sub = nh_.subscribe<geometry_msgs::PoseArray>("/yd_msg", 1, &Robot::ydCallback, this);
    vi_sub = nh_.subscribe<std_msgs::Float64MultiArray>("/yolo/detect", 1, &Robot::viCallback, this);
    tf::TransformListener listener;
    bool parked_successfully = false;
    is_kind_point = false;
    ros::Rate rate(10); // 提高频率以获得更及时的响应
    std::set<std::string> class_thing;

    // --- 新增变量 ---
    int lost_target_count = 0; // 连续未检测到目标的帧数
    const int LOST_THRESHOLD = 5; // 连续5帧未检测到才算真正丢失
    // 2. 主循环
    while (ros::ok() && !parked_successfully) {
        
        // --- 阶段一：粗略旋转搜索 ---
        ROS_INFO("阶段1:正在旋转以寻找 '%s' 类的物品...", thing.c_str());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_found = false; 
            target_pixel_x = -1.0; // 重置像素位置
        }
        lost_target_count = 0; // 新一轮搜索开始，重置计数器
        ros::Time start_time_detect = ros::Time::now();
        ros::Duration time_one_circle(7.0);   // 预计旋转一圈的时间
        while (ros::ok() && (ros::Time::now() - start_time_detect) <= time_one_circle) {     
       	ros::spinOnce(); 

            bool is_target_found_current_frame;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                is_target_found_current_frame = target_found; // 获取当前帧的检测结果
            }

            if (is_target_found_current_frame) {
                ROS_INFO("初步发现目标！准备进入对准阶段。");
                class_thing.insert(rething);
                lost_target_count = 0; // 成功检测到，重置计数器
                is_kind_point = true;
                break; 
            } else {
                lost_target_count++; // 未检测到，计数器加1
                if (lost_target_count >= LOST_THRESHOLD) {
                    // 连续多帧未检测到，可能已经完全丢失，继续旋转搜索
                    ROS_WARN("阶段1:连续%d帧未检测到目标,继续旋转搜索...", LOST_THRESHOLD);
                    // 不中断循环，让机器人继续旋转直到找到
                }
            }

            twist.linear.x = 0.0;
            twist.angular.z = 1.0; // 继续旋转
            ro_pub.publish(twist);
            rate.sleep();
        }
        if (!is_kind_point) {
            twist.angular.z = 0.0;
            ro_pub.publish(twist);
            break; // 这里不可以
        }
        
        // 1. 停止机器人
        twist.angular.z = 0.0;
        ro_pub.publish(twist);
        ROS_INFO("初步发现目标，正在停止并稳定数据...");


        // 3. 清空关键状态标志位，准备接收新数据
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_found = false; // 强制重置，丢弃所有旧的检测结果
            target_pixel_x = -1.0;
        }

        // 4. 让ROS回调队列处理一下，确保上面的重置生效，并尝试接收一次新的检测数据
        ros::spinOnce(); 
        ros::Duration(0.1).sleep(); // 再次短暂等待，确保有新的视觉数据到来
        ros::spinOnce(); // 再次处理回调

        // --- 现在可以安全进入阶段二：精确对准微调 (Servoing) ---
        ROS_INFO("阶段2:正在微调以对准目标...");
        const double CENTER_DEAD_ZONE_LEFT = 580.0;
        const double CENTER_DEAD_ZONE_RIGHT = 720.0;
        const double MAX_ANGULAR_VEL = 0.35; // 降低最大角速度，防止冲过头  0.2
        const double MIN_ANGULAR_VEL = 0.1; // 提高最小角速度，防止点击不去动  yuan 0.1
        const double ALIGN_STABLE_TIME = 0.35; // 目标在中心区域稳定停留时间（秒）  from 0.35

        ros::Time align_start_time = ros::Time::now();
        ros::Time in_deadzone_start_time = ros::Time::now(); // 记录进入死区的时间
        lost_target_count = 0; // 进入微调阶段，重置计数器

        while(ros::ok() && (ros::Time::now() - align_start_time < ros::Duration(15.0))) { // 增加微调超时时间
        //while(ros::ok()) { 
            ros::spinOnce();

            double current_pixel_x;
            bool is_still_found_current_frame; // 这一帧 viCallback 报告的目标检测结果
            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_pixel_x = target_pixel_x;
                is_still_found_current_frame = target_found; 
            }

            // 更新连续未检测计数器
            if (is_still_found_current_frame) {
                lost_target_count = 0; // 成功检测到，重置计数器
            } else {
                lost_target_count++; // 未检测到，计数器加1
            }

            // 判断是否真正丢失目标（连续多帧未检测到）
            if (lost_target_count >= LOST_THRESHOLD) {
                ROS_WARN("阶段2：在微调过程中连续%d帧丢失目标。将重新开始搜索...", LOST_THRESHOLD);
                break; // 真正丢失目标，退出微调循环，回到外层主循环重新开始搜索
            }
            
            // 如果当前视觉系统检测到目标，并且目标在死区内
            // 这里我们用 is_still_found_current_frame 来保证只有当帧有检测结果时才进入此判断
            if (is_still_found_current_frame && current_pixel_x > CENTER_DEAD_ZONE_LEFT && current_pixel_x < CENTER_DEAD_ZONE_RIGHT) {
                if ((ros::Time::now() - in_deadzone_start_time).toSec() > ALIGN_STABLE_TIME) {
                    ROS_INFO("已对准目标并稳定！(Pixel X: %f)", current_pixel_x);
                    break; 
                }
                twist.angular.z = 0.0; // 在死区内停止旋转，等待稳定
                ROS_INFO("目标在死区内，等待稳定... (Pixel X: %f)", current_pixel_x);
            } else { 
                // 目标不在死区，或者虽然在死区但当前帧未检测到（但未达到LOST_THRESHOLD）
                // 此时，如果lost_target_count < LOST_THRESHOLD，我们仍然假定目标在附近，继续根据上次有效位置进行调整
                in_deadzone_start_time = ros::Time::now(); // 目标不在死区，重置稳定计时
                // 动态调整角速度
                double error = (CENTER_DEAD_ZONE_LEFT + CENTER_DEAD_ZONE_RIGHT) / 2.0 - current_pixel_x;
                double angular_vel = error * 0.01; // 简单的P控制器，系数需要根据实际情况调整 yuan 0.001
                
                // 限制最大角速度
                if (angular_vel > MAX_ANGULAR_VEL) angular_vel = MAX_ANGULAR_VEL;
                if (angular_vel < -MAX_ANGULAR_VEL) angular_vel = -MAX_ANGULAR_VEL;
                if (angular_vel < MIN_ANGULAR_VEL && angular_vel > 0) angular_vel = MIN_ANGULAR_VEL;
                if (angular_vel > -MIN_ANGULAR_VEL && angular_vel < 0) angular_vel = -MIN_ANGULAR_VEL;
                
                twist.angular.z = angular_vel;
                ROS_INFO("正在调整方向以对准目标... (Pixel X: %f, Angular Vel: %f)", current_pixel_x, angular_vel);
            }

            twist.linear.x = 0.0;
            ro_pub.publish(twist);
            rate.sleep();
        }

        // 检查微调循环的结果
        bool aligned_successfully = false;
        // 只有当 lost_target_count 低于阈值 (即不是因为真正丢失而退出)，且像素位置在死区内，且稳定时间满足，才算成功对准
        if (lost_target_count < LOST_THRESHOLD) { // 确认不是因为目标丢失而退出循环
            double final_pixel_x;
            bool final_target_found_state; // 退出循环时 target_found 的状态
            {
                std::lock_guard<std::mutex> lock(mutex_);
                final_pixel_x = target_pixel_x;
                final_target_found_state = target_found;
            }
            if (final_target_found_state && // 再次确认最终是识别到的状态
                final_pixel_x > CENTER_DEAD_ZONE_LEFT && 
                final_pixel_x < CENTER_DEAD_ZONE_RIGHT &&
                (ros::Time::now() - in_deadzone_start_time).toSec() > ALIGN_STABLE_TIME) { // 再次检查稳定时间
                aligned_successfully = true;
            }
        }

        // 如果没有对准成功 (因为超时或目标丢失)   28日被注释，因为当板子倾斜角度很大时，可能会陷入一直微调的状态
        if (!aligned_successfully) {
            ROS_WARN("微调失败，将重新开始搜索...");
            // 使用 continue 跳到外层主循环的开头，重新从阶段一开始
            is_kind_point = false;
            break;
        }

        // 只有在对准成功后，才执行后续步骤
        twist.angular.z = 0.0;
        ro_pub.publish(twist);
        ROS_INFO("机器人已停止并对准。");
        ros::Duration(0.5).sleep(); 

        // 5. --- 阶段三：分析并导航 ---
        // (这部分逻辑与之前相同，但在机器人完全对准后执行，效果会好很多)
        ROS_INFO("阶段3:正在分析雷达数据并导航...");

        // Todo: 添加分支逻辑
        ros::Time start_time = ros::Time::now();
        ros::Duration duration(3);
        while(ros::ok() && (ros::Time::now() - start_time) < duration){
            ros::spinOnce();
            if(ob_num != 0) {
                break;
            }
            rate.sleep();
        }
        int local_ob_num;
        double local_goal_p_list[8][4];
        {
            std::lock_guard<std::mutex> lock(mutex_);
            local_ob_num = ob_num;
            for(int i=0; i < ob_num; ++i) {
                for(int j=0; j<4; ++j) {
                    local_goal_p_list[i][j] = goal_p_list[i][j];
                }
            }
        }
            int best_target_index = -1;
            double min_y_abs = 1e9;
            for (int i = 0; i < local_ob_num; i++) {
                geometry_msgs::PointStamped map_point;
                map_point.header.frame_id = "map";
                map_point.header.stamp = ros::Time(0);
                map_point.point.x = local_goal_p_list[i][0];
                map_point.point.y = local_goal_p_list[i][1];
                map_point.point.z = 0;
                try {
                    geometry_msgs::PointStamped base_link_point;
                    listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
                    listener.transformPoint("base_link", map_point, base_link_point);
                    if (base_link_point.point.x > 0.0) {
                        if (std::abs(base_link_point.point.y) < min_y_abs) {
                            min_y_abs = std::abs(base_link_point.point.y);
                            best_target_index = i;
                        }
                    }
                    ROS_INFO("best_target_index: %d", best_target_index);
                } catch (tf::TransformException &ex) {
                    ROS_ERROR("TF坐标变换失败: %s. 跳过此物体。", ex.what());
                    continue;
                }
            }

                ROS_INFO("目标已关联！准备导航至物体索引 %d。", best_target_index);
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = local_goal_p_list[best_target_index][0];
                goal.target_pose.pose.position.y = local_goal_p_list[best_target_index][1];
                goal.target_pose.pose.orientation.z = local_goal_p_list[best_target_index][2];
                goal.target_pose.pose.orientation.w = local_goal_p_list[best_target_index][3];
                move_base_client_.sendGoal(goal);
                bool nav_success = move_base_client_.waitForResult(ros::Duration(15.0));

                if (nav_success && move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("已成功停靠在目标前方。");
                    parked_successfully = true; 
                } else {
                    ROS_WARN("导航至目标失败或超时。将重新开始搜索...");
                    move_base_client_.cancelGoal();
                }
        
    }
    if (is_kind_point) {
        // 7. 任务收尾 (与之前相同)
        ROS_INFO("任务完成，已到达 '%s' 前方。", rething.c_str());
        std::string wav_thing;
        for (auto it = class_thing.begin(); it != class_thing.end(); ++it) {
            if ((thing == "Dessert"   && (*it == "Cake" || *it == "Milk" || *it == "Cola")) ||
                (thing == "Fruit"     && (*it == "Watermelon" || *it == "Apple" || *it == "Banana")) ||
                (thing == "Vegetable" && (*it == "Chili" || *it == "Tomato" || *it == "Potato"))) {
                wav_thing = *it;
                break;
            }
        }
        system("rosnode kill rknn_detect");
        std::string wav_path = "/home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/get_" + toLower(wav_thing) + "_x.wav";
        system(("aplay " + wav_path).c_str());
        Point_number = 2;
        navToGoal(Point_number++);
        ros::param::set("/detect", -1);
        ros::param::set("/thing1",wav_thing);
        // yd_control.data = 1;
        // yd_loc_control.publish(yd_control);
        yd_sub.shutdown();
        vi_sub.shutdown();
    }
}


void Robot::sim() {
    ROS_INFO("-------- 开始仿真任务 --------");

    // 直接发送导航启动指令，不再等待PC准备就绪
    std_msgs::String start_msg;
    start_msg.data = "START_NAVIGATION";
    car_to_pc_pub_.publish(start_msg);
    ROS_INFO("已发送导航启动指令");


    ROS_WARN("sim start rknn");
    system("roslaunch ucar_all_control start_twice.launch &");



    // 等待任务完成
    ROS_INFO("等待导航完成...");
    while (ros::ok() && !nav_complete) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // 处理检测结果
    if (!detected_items.empty()) {
        processDetectedItems();
    }

    // 任务结果处理
    if (sim_room != -1) {
        ROS_INFO("仿真任务完成！在房间%d找到目标物品: %s", sim_room, sim_thing.c_str());
    
        std::string room_str = std::to_string(sim_room); 
        std::string cmd = "aplay /home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/room_" + room_str + "_x.wav"; 
        ros::param::set("/thing2",sim_thing.c_str());
        system(cmd.c_str());
    } else {
        ROS_WARN("未找到目标物品");
    }

    // 发送任务完成信号
    std_msgs::String complete_msg;
    complete_msg.data = "TASK_COMPLETED";
    car_to_pc_pub_.publish(complete_msg);
    ROS_INFO("已发送任务完成信号");
}


void Robot::simPCCallback(const std_msgs::String::ConstPtr& msg) {
    const std::string& data = msg->data;
    
    if (data == "NAVIGATION_COMPLETED") {
        nav_complete = true;
        ROS_INFO("收到PC端导航完成信号");
    }
    else if (data.find("DETECTED_ITEMS:") != std::string::npos) {
        // 解析物品列表格式：DETECTED_ITEMS:item1,item2,item3
        detected_items.clear();
        std::string items_str = data.substr(15); // 去掉"DETECTED_ITEMS:"
        std::istringstream iss(items_str);
        std::string item;
        while (std::getline(iss, item, ',')) {
            detected_items.push_back(item);
        }
        ROS_INFO("收到检测物品列表: %s", items_str.c_str());
    }
}


void Robot::processDetectedItems() {
    // 根据当前任务类型匹配物品
    for (size_t i = 0; i < detected_items.size(); ++i) {
        const std::string& item = detected_items[i];
        
        if ((thing == "Dessert" && (item == "Cake" || item == "Milk" || item == "Cola")) ||
            (thing == "Fruit" && (item == "Watermelon" || item == "Apple" || item == "Banana")) ||
            (thing == "Vegetable" && (item == "Chili" || item == "Tomato" || item == "Potato"))) {
            
            sim_room = i + 1; // 房间号从1开始
            sim_thing = item;
            ROS_INFO("匹配到物品: %s (房间%d)", item.c_str(), sim_room);
            break;
        }
    }
}


void Robot::navTrack() {
    // 先从仿真区导航到左边的信号灯前方，进行判断后选择进入哪一条巡线道路
    /*
        argc:
            none
        return:
            none
    */
    // system("rosrun nanodet rknn_detect2.py &");
    geometry_msgs::PoseWithCovarianceStamped msg;


    // system("roslaunch ucar_all_control start_twice.launch &");



    ros::param::set("/teb/update_flag",true);
    ROS_INFO("-------- 开始 navTrack 任务--------");
    ros::param::set("/detect2", 1);
    ros::param::set("/light", 0);
    Point_number = 3;
    navToGoal(Point_number);
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(3.0); // 设置10秒超时
    ros::Rate loop_rate(10); // 设置循环频率，例如10Hz

    while (ros::ok() && (ros::Time::now() - start_time < timeout)) {
        ros::param::get("/light",light);
        if(light) {
            break;
        }
        loop_rate.sleep(); // <--- 控制循环频率，避免CPU占用过高
    }

    ROS_INFO("light为%d",light);
    ros::param::set("/detect2", -1);
    system("rosnode kill rknn_detect2");
    if(light==1)//左红灯
    {
        system("rosnode kill sxunleft");
        // system("rosnode kill hdl03");
        Point_number++;
        navToGoal(Point_number);
        std::string cmd = "aplay /home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/cross_2_x.wav"; 
        system(cmd.c_str());
        Point_number = 6;
        navToGoal(Point_number);
        ros::param::set("/lor",2);//左线4.669, 3.475, -1.933

        // 时间戳和坐标系
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";  // AMCL 期望的是 "map" 坐标系

        // 位置
        msg.pose.pose.position.x = 4.669;
        msg.pose.pose.position.y = 3.457;
        msg.pose.pose.position.z = 0.0;

        // 方向（四元数）
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = sin(-1.933/2);
        msg.pose.pose.orientation.w = cos(-1.933/2);

        // 协方差矩阵 (6x6 展平成 36 个元素)
        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = 0.0;
        }
        msg.pose.covariance[0] = 0.0;   // x 方向方差
        msg.pose.covariance[7] = 0.0;   // y 方向方差
        msg.pose.covariance[35] = 0.05; // 角度方差

        // 发布一次
        reinitial_pub.publish(msg);
    }
    else{
        system("rosnode kill sxunright");
        // system("rosnode kill hdr03");
        std::string cmd = "aplay /home/ucar/ucar_ws_copy/src/ucar_all_control/voice/wav/cross_1_x.wav"; 
        system(cmd.c_str());
        Point_number = 5;
        navToGoal(Point_number);
        ros::param::set("/lor",1);//右线

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";  // AMCL 期望的是 "map" 坐标系

        // 位置
        msg.pose.pose.position.x = 2.768;
        msg.pose.pose.position.y = 3.457;
        msg.pose.pose.position.z = 0.0;

        // 方向（四元数）
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = sin(-1.273/2);
        msg.pose.pose.orientation.w = cos(-1.273/2);

        // 协方差矩阵 (6x6 展平成 36 个元素)
        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = 0.0;
        }
        msg.pose.covariance[0] = 0.0;   // x 方向方差
        msg.pose.covariance[7] = 0.0;   // y 方向方差
        msg.pose.covariance[35] = 0.05; // 角度方差

        // 发布一次
        reinitial_pub.publish(msg);
    }
    ROS_INFO("开始巡线");
    ros::Duration(1).sleep();
    ros::param::set("/xflag",1);
    callback_triggered = false;
    yd_sub2 = nh_.subscribe<geometry_msgs::PoseArray>("/yd_msg", 1, &Robot::ydCallback, this);
    ros::Rate loop_rate_2(10);
    zf=0;
    while(ros::ok() && zf==0){
        ros::param::get("/zhang",zhang);
        if(zhang == 1)
            {   
                ROS_WARN("I got /zhong sign");
                ros::Time start_time = ros::Time::now();
                ros::Duration timelimit(1.0);
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = 0;
                // 给它2秒中时间更新雷达数据
                while(ros::ok() && (ros::Time::now() - start_time) < timelimit) {
                    ros::spinOnce();
                    ro_pub.publish(twist);
                    loop_rate_2.sleep();
                }
                ROS_INFO("nav");
                // ros::param::get("/pointx",x);
                // ros::param::set("/pointy",y);
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.pose.position.x = goal_p_list[0][0];
                goal.target_pose.pose.position.y = goal_p_list[0][1];
                goal.target_pose.pose.orientation.z = goal_p_list[0][2];
                goal.target_pose.pose.orientation.w = goal_p_list[0][3];
                ROS_WARN("I will go to (%lf, %lf)", goal_p_list[0][0], goal_p_list[0][1]);
                move_base_client_.sendGoal(goal);
                move_base_client_.waitForResult(ros::Duration(10.0));
                if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("I am now in point (%lf, %lf)", goal_p_list[0][0], goal_p_list[0][1]);
                    ros::param::set("/zf",1);
                    // return;
                }
                else {
                    ROS_ERROR("NAV ERR(b)");
                    // return;
                }
            }
        // ros::param::set("/zf",1);
        ros::param::get("/zf",zf);
        ros::spinOnce();
    }
    ros::shutdown();
    exit(0);  

}


void Robot::run() {
    // 任务入口
    // 通过cur_task(current task)的值，跳转到具体的任务。
    // 如果想要调试具体任务，可以在函数开始处设置cur_task的值。
    /*
        Argc: 
            None
        Return:
            None
    */
    ROS_INFO("------- start by key now -------");
    ros::Rate loop_rate(10);
    ros::Time start_time = ros::Time::now();
    ros::Duration duration(10.0);

    while (ros::ok()) {
        if (kbhit()) {
            char c = getchar();
            if (c == 's') {
                ROS_INFO("按下 s,启动功能");
                voiceFlag = true;
                break;
            }
            else if (c == 'q') {
                ROS_INFO("按下 q,退出程序");
                ros::shutdown();
            }
        }
    }

    
    std::string command = "rosnode kill speech_command_node";
    int result = system(command.c_str());
    if(result) {
        ROS_INFO("I kill the speech_command_node");
    }
    // cur_task = "navTrack"; // 为了测试q
    // Point_number = 1;


    ROS_INFO("机器人比赛任务开始！");
    while(ros::ok()){
        if(voiceFlag) {
            // 主任务循环
            switch (task_map[cur_task])
            {
            case 1:
                toQC();
		        cur_task = "toMidFrame";
                break;
            case 2:
		        toMidFrame();
                cur_task = "collectPic";
                break;
            case 3:
		        ros::Duration(0.5).sleep();
                collectPic();
                if (!is_kind_point) {
                    if (Point_number == 1) {
                        Point_number = 7;
                        cur_task = "toMidFrame";
                    }
                    else if (Point_number == 7) {
                        Point_number = 8;
                        cur_task = "toMidFrame";
                    }
                    else if (Point_number == 8) {
                        cur_task = "sim";
                    }
                }
                // sleep(3);
                // cur_task="finish";
		        // cur_task = "finish";
                else
                    cur_task = "sim";
                    // cur_task = "navTrack";
                break;
            case 4:
                sim();
                cur_task = "navTrack";
                break;
            case 5:
                navTrack();
                cur_task="finish";
                break;
            default:
                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep(); 
    }
}


int main(int argc, char* argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "race_all");
    ros::NodeHandle nh;
    Robot robot(nh);
    robot.run();
    return 0;
}
