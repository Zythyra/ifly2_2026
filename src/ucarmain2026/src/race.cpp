/**
 * @file main_competition_node.cpp
 * @brief 2026 智能车比赛 - 核心流程总控程序 (融入定点扫码 + 离线播报 + 找板停靠全闭环)
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

// 硬件与自定义服务
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <ucarmain2026/GetTaskSemantics.h> 
#include <ucarmain2026/ItemClassify.h> 
#include <qr_01/qr_code.h>
#include <curl/curl.h>

// 【新增】：引入找板类的头文件与底盘服务
#include "ucarmain2026/turn_detect.h"
#include "ucarmain2026/set_speed.h"

#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <cstdlib> // 用于 system()

using namespace std;

// ================= 全局状态机枚举 =================
enum MainState {
    WAIT_WAKEUP,        
    RECORDING,          
    SEMANTIC_PARSING,   
    NAVIGATING,         
    QR_SCANNING,        
    ITEM_CLASSIFYING,   
    TTS_BROADCASTING,   
    FINDING_BOARD_NAV,  // 【新增】：前往预设找板点
    FINDING_BOARD_SCAN, // 【新增】：原地旋转找板与几何停靠
    PLAY_3_4_WAV,       // 【新增】：播放 3.wav 和 4.wav
    GO_FINAL_POINT,     // 【新增】：前往终点坐标
    PLAY_5_WAV,         // 【新增】：播放 5.wav
    ALL_FINISHED        
};
MainState current_state = WAIT_WAKEUP;

// ================= 本地核心存储变量 =================
string target_real = "";
string target_sim = "";

string target_qr_1 = "未知物1";
string target_qr_2 = "未知物2";
string target_qr_3 = "未知物3";

string final_real_item = "";
string final_sim_item = "";

// 找板专用变量
int target_board_class = -1; // 0=日用品, 1=食品, 2=电子产品
int board_waypoint_idx = 0;
double preset_board_x[3]   = {1.25, 2.75, 3.75};
double preset_board_y[3]   = {3.5, 3.5, 3.5};
double preset_board_yaw[3] = {0.0, 0.0, 0.0};
double bound_x_min = 0.0;
double bound_x_max = 5.0;
double bound_y_min = 2.5;
double bound_y_max = 4.5;

// ================= 客户端与发布者 =================
ros::ServiceClient record_client;
ros::ServiceClient semantic_client;
ros::ServiceClient qr_client;
ros::ServiceClient classifier_client; 
ros::Publisher cmd_pub;

// ================= 语音与硬件基础 =================
bool is_recording = false;
ros::Time record_start_time;
vector<char> audio_buffer;

#pragma pack(push, 1)
struct WavHeader {
    char riff_id[4] = {'R', 'I', 'F', 'F'};
    uint32_t riff_size;
    char wave_id[4] = {'W', 'A', 'V', 'E'};
    char fmt_id[4] = {'f', 'm', 't', ' '};
    uint32_t fmt_size = 16;
    uint16_t audio_format = 1;      
    uint16_t num_channels = 1;      
    uint32_t sample_rate = 16000;   
    uint32_t byte_rate = 32000;     
    uint16_t block_align = 2;       
    uint16_t bits_per_sample = 16;  
    char data_id[4] = {'d', 'a', 't', 'a'};
    uint32_t data_size;
};
#pragma pack(pop)

void saveAsWav(const string& filename, const vector<char>& pcm_data) {
    WavHeader header;
    header.data_size = pcm_data.size();
    header.riff_size = header.data_size + 36;
    ofstream out_file(filename, ios::binary);
    if (out_file.is_open()) {
        out_file.write(reinterpret_cast<const char*>(&header), sizeof(WavHeader));
        out_file.write(pcm_data.data(), pcm_data.size());
        out_file.close();
    }
}

// 导航 Action
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool go_destination(double x, double y, double yaw, MoveBaseClient &ac) {
    move_base_msgs::MoveBaseGoal goal;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();
    ac.sendGoal(goal);
    ac.waitForResult();
    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

// 扫码核心遍历参数
int qr_waypoint_idx = 0;
ros::Time scan_start_time;
vector<string> scanned_urls;
int valid_qr_count = 0;

// HTTP 解析
size_t WriteCallback(void* contents, size_t size, size_t nmemb, string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}
string httpGet(string url) {
    CURL* curl; CURLcode res; string readBuffer;
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L); 
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    return readBuffer;
}
int extractCode(const string& json_str) {
    size_t key_pos = json_str.find("\"code\"");
    if (key_pos == string::npos) return -1;
    size_t colon_pos = json_str.find(":", key_pos);
    try { return stoi(json_str.substr(colon_pos + 1)); } catch (...) { return -1; }
}
string extractResult(const string& json_str) {
    size_t key_pos = json_str.find("\"result\"");
    if (key_pos == string::npos) return "";
    size_t start_quote = json_str.find("\"", key_pos + 8); 
    if (start_quote == string::npos) return "";
    size_t end_quote = json_str.find("\"", start_quote + 1);
    return json_str.substr(start_quote + 1, end_quote - start_quote - 1);
}

// 唤醒回调
void awakeCallback(const std_msgs::Int8::ConstPtr& msg) {
    if (current_state == WAIT_WAKEUP && msg->data == 1) {
        ROS_INFO("检测到唤醒！触发 9 秒录音...");
        xf_mic_asr_offline::Start_Record_srv srv;
        srv.request.whether_start = 1; 
        record_client.call(srv);
        audio_buffer.clear(); 
        is_recording = true;
        record_start_time = ros::Time::now();
        current_state = RECORDING;
    }
}
void pcmCallback(const xf_mic_asr_offline::Pcm_Msg::ConstPtr& msg) {
    if (current_state == RECORDING) audio_buffer.insert(audio_buffer.end(), msg->pcm_buf.begin(), msg->pcm_buf.end());
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    ros::init(argc, argv, "main_competition_node");
    ros::NodeHandle nh;

    record_client = nh.serviceClient<xf_mic_asr_offline::Start_Record_srv>("/xf_asr_offline_node/start_record_srv");
    semantic_client = nh.serviceClient<ucarmain2026::GetTaskSemantics>("/get_task_semantics");
    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");
    classifier_client = nh.serviceClient<ucarmain2026::ItemClassify>("/get_item_classification"); 
    
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // 【新增】：初始化视觉底层控制器
    MecanumController mecanumController(nh);

    ros::Subscriber awake_sub = nh.subscribe("/awake_flag", 10, awakeCallback);
    ros::Subscriber pcm_sub = nh.subscribe("/mic/pcm/deno", 100, pcmCallback);
    
    MoveBaseClient ac("move_base", true); 

    ROS_INFO("智能车总控节点已启动！等待语音唤醒...");

    ros::Rate rate(20); 
    while (ros::ok() && current_state != ALL_FINISHED) {
        ros::spinOnce(); 

        switch (current_state) {
            case RECORDING:
                if ((ros::Time::now() - record_start_time).toSec() >= 9.0) {
                    xf_mic_asr_offline::Start_Record_srv srv_rec; srv_rec.request.whether_start = 0; 
                    record_client.call(srv_rec);
                    saveAsWav("/home/ucar/ucar_car/wakeup_record/test_record.wav", audio_buffer);
                    current_state = SEMANTIC_PARSING;
                }
                break;

            case SEMANTIC_PARSING:
            {
                ucarmain2026::GetTaskSemantics srv_task;
                if (semantic_client.call(srv_task) && srv_task.response.success) {
                    target_real = srv_task.response.target_real;
                    target_sim = srv_task.response.target_sim;
                    ROS_INFO("语义解析成功! 实体区=[%s], 仿真区=[%s]", target_real.c_str(), target_sim.c_str());
                    current_state = NAVIGATING;
                } else {
                    ROS_WARN("⚠️ 解析失败！正在播放提示音并重新录制...");
                    system("aplay -q /home/ucar/ucar_car/src/ucarmain2026/audios/1.wav");
                    
                    xf_mic_asr_offline::Start_Record_srv srv_rec;
                    srv_rec.request.whether_start = 1;
                    record_client.call(srv_rec);
                    
                    audio_buffer.clear();
                    is_recording = true; 
                    record_start_time = ros::Time::now(); 
                    current_state = RECORDING; 
                }
                break;
            }

            case NAVIGATING:
            {
                ac.waitForServer();
                double qr_wp_x[4]   = {0.5, 0.75, 1.0, 0.75};
                double qr_wp_y[4]   = {5.25, 5.0, 5.25, 5.5};
                double qr_wp_yaw[4] = {0.0, 1.57, 3.14, -1.57};

                if (qr_waypoint_idx >= 4) {
                    ROS_INFO("4个扫码点遍历完毕！进行分类...");
                    qr_01::qr_code srv_qr; srv_qr.request.command = -2; qr_client.call(srv_qr);
                    current_state = ITEM_CLASSIFYING;
                    break;
                }

                ROS_INFO("前往第 %d 个定点扫码位置...", qr_waypoint_idx + 1);
                if (go_destination(qr_wp_x[qr_waypoint_idx], qr_wp_y[qr_waypoint_idx], qr_wp_yaw[qr_waypoint_idx], ac)) {
                    qr_01::qr_code srv_qr; 
                    srv_qr.request.command = -1; qr_client.call(srv_qr); 
                    ros::Duration(1.0).sleep(); 
                    srv_qr.request.command = -3; qr_client.call(srv_qr); 
                    scan_start_time = ros::Time::now();
                    current_state = QR_SCANNING;
                } else {
                    qr_waypoint_idx++;
                }
                break;
            }

            case QR_SCANNING:
            {
                if ((ros::Time::now() - scan_start_time).toSec() > 2.0) {
                    qr_01::qr_code srv_qr; srv_qr.request.command = -2; qr_client.call(srv_qr);
                    qr_waypoint_idx++;
                    current_state = NAVIGATING;
                    break;
                }

                qr_01::qr_code srv; srv.request.command = 1;
                if (qr_client.call(srv) && !srv.response.result.empty()) {
                    string raw_res = srv.response.result;
                    size_t split_pos = raw_res.find("|");
                    string captured_url = (split_pos != string::npos) ? raw_res.substr(split_pos + 1) : raw_res;
                    
                    if (find(scanned_urls.begin(), scanned_urls.end(), captured_url) == scanned_urls.end()) {
                        scanned_urls.push_back(captured_url); 
                        string json = httpGet(captured_url);
                        int code = extractCode(json);
                        
                        if (code == 200) {
                            string res_text = extractResult(json);
                            if (valid_qr_count == 0) target_qr_1 = res_text;
                            else if (valid_qr_count == 1) target_qr_2 = res_text;
                            else if (valid_qr_count == 2) target_qr_3 = res_text;
                            valid_qr_count++;
                            
                            if (valid_qr_count >= 3) {
                                ROS_INFO("成功收集 3 个有效二维码！");
                                qr_01::qr_code srv_qr; srv_qr.request.command = -2; qr_client.call(srv_qr);
                                current_state = ITEM_CLASSIFYING;
                            } else {
                                qr_01::qr_code srv_qr; srv_qr.request.command = -2; qr_client.call(srv_qr);
                                qr_waypoint_idx++;
                                current_state = NAVIGATING;
                            }
                        } else if (code != 400) {
                            scanned_urls.pop_back(); 
                        }
                    }
                }
                break;
            }
            
            case ITEM_CLASSIFYING:
            {
                ROS_INFO("正在请求大模型分类...");
                ucarmain2026::ItemClassify classify_srv;
                classify_srv.request.item1 = target_qr_1;
                classify_srv.request.item2 = target_qr_2;
                classify_srv.request.item3 = target_qr_3;
                classify_srv.request.target_real = target_real;
                classify_srv.request.target_sim = target_sim;
                
                if (classifier_client.call(classify_srv) && classify_srv.response.success) {
                    final_real_item = classify_srv.response.real_item;
                    final_sim_item  = classify_srv.response.sim_item;
                    ROS_INFO("分类成功！实体区应放: %s", final_real_item.c_str());
                    current_state = TTS_BROADCASTING; 
                } else {
                    ros::Duration(2.0).sleep();
                }
                break;
            }

            // ================= 核心修改：生成 4 段音频，播放 2.wav，提取找板 ID =================
            case TTS_BROADCASTING:
            {
                string ch_real_category = target_real;
                string ch_real_workshop = target_real + "车间"; 
                if (target_real == "food") { ch_real_category = "食品"; ch_real_workshop = "食品加工车间"; target_board_class = 1; } 
                else if (target_real == "daily") { ch_real_category = "日用品"; ch_real_workshop = "日用品加工车间"; target_board_class = 0; } 
                else if (target_real == "electronic") { ch_real_category = "电子产品"; ch_real_workshop = "电子产品生产车间"; target_board_class = 2; }
                
                string ch_sim_category = target_sim;
                string ch_sim_workshop = target_sim + "车间"; 
                if (target_sim == "food") { ch_sim_category = "食品"; ch_sim_workshop = "食品加工车间"; } 
                else if (target_sim == "daily") { ch_sim_category = "日用品"; ch_sim_workshop = "日用品加工车间"; } 
                else if (target_sim == "electronic") { ch_sim_category = "电子产品"; ch_sim_workshop = "电子产品生产车间"; }

                string text2 = "取得" + final_real_item + "属于" + ch_real_category + "，应放置于" + ch_real_workshop + "；" +
                               "仿真环境中取得" + final_sim_item + "属于" + ch_sim_category + "，应放置于" + ch_sim_workshop + "。";
                string text3 = "已将" + final_real_item + "放入" + ch_real_workshop + "。";
                string text4 = "仿真任务已完成，已将" + final_sim_item + "放入" + ch_sim_category + "仓库。";
                string text5 = "任务完成。"; // 新增第五个音频文本
                
                ROS_INFO("正在调用本地脚本批量合成音频 (2~4.wav)...");
                string cmd = "python3 /home/ucar/ucar_car/src/ucarmain2026/scripts/generate_task_audios.py \"" + text2 + "\" \"" + text3 + "\" \"" + text4 + "\" \"" + text5 + "\"";
                system(cmd.c_str());
                
                ROS_INFO("播放：获取分类汇总 (2.wav)...");
                system("aplay -q /home/ucar/ucar_car/src/ucarmain2026/audios/2.wav");
                
                // 唤醒摄像头，准备找板
                std::vector<std::vector<int>> dummy_result = {{-1},{-1},{-1},{-1},{-1},{-1}};
                mecanumController.detect(dummy_result, -1);
                ros::Duration(1.0).sleep();
                
                current_state = FINDING_BOARD_NAV;
                break;
            }

            // ================= 核心新增：找板导航 =================
            case FINDING_BOARD_NAV:
            {
                if (board_waypoint_idx >= 3) {
                    ROS_WARN("⚠️ 找板预设点全部遍历完毕均未发现目标！直接强行前往终点...");
                    mecanumController.cap_close();
                    current_state = GO_FINAL_POINT;
                    break;
                }

                ROS_INFO("--------------------------------------------------");
                ROS_INFO("前往第 %d 个找板预设点...", board_waypoint_idx + 1);
                
                if(go_destination(preset_board_x[board_waypoint_idx], preset_board_y[board_waypoint_idx], preset_board_yaw[board_waypoint_idx], ac)) {
                    mecanumController.cap_buffer_clear();
                    ROS_INFO("到达预设点，开始原地旋转对准目标板...");
                    current_state = FINDING_BOARD_SCAN;
                } else {
                    board_waypoint_idx++;
                }
                break;
            }

            // ================= 核心新增：视觉找板与几何停靠 =================
            case FINDING_BOARD_SCAN:
            {
                double targetx, targety, targetz, targetx2, targety2, targetz2;
                bool targetflag = false, target2flag = false, use_forward = false;

                bool found = mecanumController.turn_and_find_plus(
                    14.0, target_board_class, 0.4, 
                    targetx, targety, targetz, targetflag, 
                    targetx2, targety2, targetz2, target2flag, 
                    use_forward, 1
                );

                if (found) {
                    ROS_INFO("成功锁定并对准目标！交还底盘控制权至 move_base...");
                    
                    // 强制关闭底盘的 20Hz 控制流，防止打架
                    ucarmain2026::set_speed srv_stop;
                    srv_stop.request.work = false;
                    ros::service::call("/set_speed", srv_stop);
                    ros::Duration(0.5).sleep();

                    std::vector<float> cur_pose = mecanumController.getCurrentPose();
                    double cur_x = cur_pose[0];
                    double cur_y = cur_pose[1];
                    double cur_yaw = cur_pose[2];

                    // --- 几何截距计算 ---
                    double t_min = 9999.0; 
                    int hit_wall = -1;     
                    
                    if (cos(cur_yaw) > 1e-4) {
                        double t = (bound_x_max - cur_x) / cos(cur_yaw);
                        if (t > 0 && t < t_min) { t_min = t; hit_wall = 0; }
                    } else if (cos(cur_yaw) < -1e-4) {
                        double t = (bound_x_min - cur_x) / cos(cur_yaw);
                        if (t > 0 && t < t_min) { t_min = t; hit_wall = 1; }
                    }
                    if (sin(cur_yaw) > 1e-4) {
                        double t = (bound_y_max - cur_y) / sin(cur_yaw);
                        if (t > 0 && t < t_min) { t_min = t; hit_wall = 2; }
                    } else if (sin(cur_yaw) < -1e-4) {
                        double t = (bound_y_min - cur_y) / sin(cur_yaw);
                        if (t > 0 && t < t_min) { t_min = t; hit_wall = 3; }
                    }

                    if (t_min != 9999.0 && hit_wall != -1) {
                        double board_x = cur_x + t_min * cos(cur_yaw);
                        double board_y = cur_y + t_min * sin(cur_yaw);
                        
                        // 正交停靠约束
                        double safe_distance = 0.3; 
                        double dock_x = board_x, dock_y = board_y, dock_yaw = 0.0;

                        if (hit_wall == 0) { dock_x = bound_x_max - safe_distance; dock_yaw = 0.0; } 
                        else if (hit_wall == 1) { dock_x = bound_x_min + safe_distance; dock_yaw = 3.1415926; } 
                        else if (hit_wall == 2) { dock_y = bound_y_max - safe_distance; dock_yaw = 1.5707963; } 
                        else if (hit_wall == 3) { dock_y = bound_y_min + safe_distance; dock_yaw = -1.5707963; }
                        
                        ROS_INFO("视觉对准完毕！执行几何制导停靠...");
                        go_destination(dock_x, dock_y, dock_yaw, ac);
                    }
                    
                    mecanumController.cap_close();
                    current_state = PLAY_3_4_WAV; // 去播放 3 和 4
                } else {
                    ROS_INFO("此预设点未发现目标，准备前往下一个...");
                    board_waypoint_idx++;
                    current_state = FINDING_BOARD_NAV;
                }
                break;
            }

            // ================= 核心新增：播放 3 和 4 =================
            case PLAY_3_4_WAV:
            {
                ROS_INFO("播放：实体区放置完毕 (3.wav)...");
                system("aplay -q /home/ucar/ucar_car/src/ucarmain2026/audios/3.wav");
                
                ROS_INFO("播放：仿真区放置完毕 (4.wav)...");
                system("aplay -q /home/ucar/ucar_car/src/ucarmain2026/audios/4.wav");
                
                current_state = GO_FINAL_POINT;
                break;
            }

            // ================= 核心新增：前往终点 =================
            case GO_FINAL_POINT:
            {
                ROS_INFO("任务核心已全部跑完！正在前往最终停放点...");
                go_destination(0.25, 0.25, 3.14, ac);
                current_state = PLAY_5_WAV;
                break;
            }

            // ================= 核心新增：播放 5 =================
            case PLAY_5_WAV:
            {
                ROS_INFO("播放：任务完成 (5.wav)...");
                system("aplay -q /home/ucar/ucar_car/src/ucarmain2026/audios/5.wav");
                current_state = ALL_FINISHED;
                break;
            }
            
            case ALL_FINISHED:
                break;
        } 
        
        rate.sleep();
    } 

    
    ROS_INFO("完成巡线以外所有内容");
    
    return 0;
}