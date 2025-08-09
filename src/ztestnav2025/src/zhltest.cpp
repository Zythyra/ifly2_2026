#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"
#include "line_follow/line_follow.h"
#include "ztestnav2025/traffic_light.h"
#include "qr_01/qr_srv.h"
#include "communication/msg_1.h"
#include "communication/msg_2.h"

#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
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


int main(int argc, char *argv[]){
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
    ROS_INFO("主干代码开始，初始化对象，等待服务中"); 
    //-----------------------------------初始化movebase，实例对象---------------------------//
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    MecanumController mecanumController(nh);
    //客户端初始化
    ROS_INFO("等待lidar_process服务中---");
    ros::ServiceClient client_find_board = nh.serviceClient<ztestnav2025::lidar_process>("/lidar_process/lidar_process");
    ztestnav2025::lidar_process where_board;
    client_find_board.waitForExistence();
    ROS_INFO("等待坐标获取服务中---");
    ros::ServiceClient poseget_client = nh.serviceClient<ztestnav2025::getpose_server>("getpose_server");
    ztestnav2025::getpose_server pose_result;
    pose_result.request.getpose_start = 1;
    poseget_client.waitForExistence();

    ROS_INFO("拣货区域任务开始");
    size_t board_count;
    int board_name;
    MoveBaseClient ac("move_base", true); 
    tf2::Quaternion q;  
    //等待action回应
    while(!ac.waitForServer()){//这里之前只有等待五秒,
        ROS_INFO("等待movebase服务中---");
    } 
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    ROS_INFO("拣货区域任务开始");
    bool flag=false;//判断雷达识别的点是否和视觉对得上
    //第一点视觉识别
    //视觉识别开始，先传个-1把摄像头打开
    
    std::vector<std::vector<int>> a = {{-1},{-1},{-1},{-1},{-1},{-1}};
    mecanumController.detect(a,-1);
    mecanumController.cap_buffer_clear();
    go_destination(goal,1.25,3.75,0,q,ac);  
    //然后去中间，识别目标，或者定位遮挡视野的板子
    // go_destination(goal,0.75,1.75,1.57,q,ac); 
    double targetx, targety, targetz, targetx2, targety2, targetz2;
    bool target2flag = false,targetflag = false,use_forward = false;
    if(mecanumController.turn_and_find_plus(17,3,0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1)){
        if(!use_forward)//如果返回false，就先绕行再用原有逻辑靠近
        {
            ROS_INFO("有障碍物，先绕行");
            if (std::min(abs(targetx2-0),abs(targetx2 - 2.5))>0.4 && std::min(abs(targety2 - 2),abs(targety2 - 5))>0.4)//终点太靠墙直接视觉过去也能避开
            {
                go_destination(goal, targetx2, targety2, targetz2, q, ac);
            }
            else{
                ROS_INFO ("障碍物较远,直接前进");
                
            }
            
        }
        board_name = mecanumController.forward_and_adjust(3,0.35);
        if(board_name<0){//出现这种情况，比较糟糕，要么是路被封死了，要么是走一半目标丢了
            if(mecanumController.turn_and_find_plus(17,3,0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1)){
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
                    board_name = mecanumController.forward_and_adjust(3,0.5);
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
                    if(mecanumController.turn_and_find_plus(5.0,3,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(3,0.5);
                        flag=true;
                    }
                    else if(mecanumController.turn_and_find_plus(11.0,3,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(3,0.5);
                        flag=true;
                    }
                }
                if(target2flag && !flag){
                    double passx, passy, passz, passx2, passy2, passz2;
                    bool find1,find2;
                    ROS_INFO("前往%f,%f,%f",targetx2, targety2, targetz2);
                    go_destination(goal,targetx2, targety2, targetz2,q,ac);
                    mecanumController.cap_buffer_clear();
                    if(mecanumController.turn_and_find_plus(5.0,3,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(3,0.5);
                        flag=true;
                    }
                    else if(mecanumController.turn_and_find_plus(11.0,3,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                        board_name = mecanumController.forward_and_adjust(3,0.5);
                        flag=true;
                    }
                }
            }
        }
        else{
            flag=true;
        }
    }
    
    
    
    
    
    
    
    
    
    
    
    else{//板子被挡了，中间看不到,绕到两个板的后面
        if(targetflag){
            double passx, passy, passz, passx2, passy2, passz2;
            bool find1,find2;
            ROS_INFO("前往%f,%f,%f",targetx,targety,targetz);
            go_destination(goal,targetx,targety,targetz,q,ac);
            mecanumController.cap_buffer_clear();
            if(mecanumController.turn_and_find_plus(5.0,3,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(3,0.5);
                flag=true;
            }
            else if(mecanumController.turn_and_find_plus(11.0,3,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(3,0.5);
                flag=true;
            }
        }
        if(target2flag && !flag){
            double passx, passy, passz, passx2, passy2, passz2;
            bool find1,find2;
            ROS_INFO("前往%f,%f,%f",targetx2, targety2, targetz2);
            go_destination(goal,targetx2, targety2, targetz2,q,ac);
            mecanumController.cap_buffer_clear();
            if(mecanumController.turn_and_find_plus(5.0,3,0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(3,0.5);
                flag=true;
            }
            else if(mecanumController.turn_and_find_plus(11.0,3,-0.4,passx, passy, passz, find1,passx2, passy2, passz2,find2,use_forward)){
                board_name = mecanumController.forward_and_adjust(3,0.5);
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
        if(mecanumController.turn_and_find_plus(7, 3, 0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1))
        {
            board_name = mecanumController.forward_and_adjust(3,0.35);
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
        if(mecanumController.turn_and_find_plus(7, 3, 0.4,targetx, targety, targetz, targetflag,targetx2, targety2, targetz2,target2flag,use_forward,1))
        {
            board_name = mecanumController.forward_and_adjust(3,0.35);
            if(board_name >= 0)
            {
                flag = true; 
            }
        }

    }
    






    // mecanumController.adjust(2,0.4);//
    // int de = mecanumController.turn_and_find(17,1,-0.4);
    ROS_INFO("结束了");
    // ROS_INFO("%d",de);
}


// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"zhltest");
//     ros::NodeHandle nh;

//     MecanumController mecanumController(nh);

//     std::vector<int> a = {-1,-1,-1,-1,-1,-1};
//     mecanumController.detect(a,-1);
// restart:
//     mecanumController.turn_and_find(1,1,1,0.4);
//     ROS_INFO("结束了");
//     while(ros::ok()){
//         ros::spinOnce();
//         if(mecanumController.pid_change_flag == 1){
//             mecanumController.pid_change_flag = 0;
//             goto restart;
//         }
//     }
//     ros::spin();

//     return 0;
// }

// int main(int argc, char *argv[]){
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"zhltest");
//     play_audio(voice[0][0]);
//     waitForContinue();
//     play_audio(voice[0][1]);
//     waitForContinue();
//     play_audio(voice[0][2]);
//     waitForContinue();
//     play_audio(voice[1][0]);
//     waitForContinue();
//     play_audio(voice[1][1]);
//     waitForContinue();
//     play_audio(voice[1][2]);
//     waitForContinue();
//     play_audio(voice[1][3]);
//     waitForContinue();
//     play_audio(voice[1][4]);
//     waitForContinue();
//     play_audio(voice[1][5]);
//     play_audio(voice[1][6]);
//     play_audio(voice[1][7]);
//     play_audio(voice[1][8]);
//     play_audio(voice[2][0]);
//     play_audio(voice[2][1]);
//     play_audio(voice[2][2]);
//     play_audio(voice[3][0]);
//     play_audio(voice[3][1]);
    // play_audio(cost[0]);
    // play_audio(cost[1]);
    // play_audio(cost[2]);
    // play_audio(cost[3]);
    // play_audio(cost[4]);
    // play_audio(cost[5]);
    // play_audio(cost[6]);
    // play_audio(cost[7]);
    // play_audio(cost[8]);
    // play_audio(cost[9]);
    // play_audio(cost[10]);
    // play_audio(cost[11]);
    // play_audio(cost[12]);
    // play_audio(cost[13]);
    // play_audio(cost[14]);
    // play_audio(cost[15]);
    // play_audio(cost[16]);
    // play_audio(cost[17]);
// }

