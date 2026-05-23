/**
 * @file main_competition_node.cpp
 * @brief 2026 智能车比赛 - 核心流程总控程序 (完整闭环版)
 * 流程：唤醒 -> 录音 -> 语义解析 -> 导航 -> 扫码 -> 大模型分类 -> 语音播报
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// 硬件与自定义服务
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <ucarmain2026/GetTaskSemantics.h> 
#include <ucarmain2026/ItemClassify.h> // 【新增】：大模型分类服务
#include <qr_01/qr_code.h>
#include <curl/curl.h>

#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <algorithm>

using namespace std;

// ================= 全局状态机枚举 =================
enum MainState {
    WAIT_WAKEUP,        // 等待语音唤醒
    RECORDING,          // 正在录音(9秒)
    SEMANTIC_PARSING,   // 呼叫大模型解析任务大类
    NAVIGATING,         // 导航前往二维码区
    QR_SCANNING,        // 旋转扫描二维码
    ITEM_CLASSIFYING,   // 【新增】：呼叫大模型对物品分类
    TTS_BROADCASTING,   // 【新增】：进行最终语音播报
    ALL_FINISHED        // 全部完成
};
MainState current_state = WAIT_WAKEUP;

// ================= 本地核心存储变量 =================
string target_real = "";
string target_sim = "";

string target_qr_1 = "未知物1";
string target_qr_2 = "未知物2";
string target_qr_3 = "未知物3";

// 最终分类结果
string final_real_item = "";
string final_sim_item = "";

// ================= 客户端与发布者 =================
ros::ServiceClient record_client;
ros::ServiceClient semantic_client;
ros::ServiceClient qr_client;
ros::ServiceClient classifier_client; // 分类服务客户端
ros::Publisher cmd_pub;
ros::Publisher tts_pub;               // 语音播报发布者

// ================= 语音与硬件相关的基础函数 (与上一版相同) =================
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

// 里程计与扫码
double last_yaw = 0.0;
double total_rotated = 0.0; 
bool odom_initialized = false; 
vector<string> scanned_urls;
int valid_qr_count = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (current_state != QR_SCANNING) return;
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (!odom_initialized) { last_yaw = yaw; odom_initialized = true; return; }
    double delta = yaw - last_yaw;
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;
    total_rotated += delta; 
    last_yaw = yaw;
}

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
        ROS_INFO("🔔 检测到唤醒！触发 9 秒录音...");
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

    // 初始化客户端与发布者
    record_client = nh.serviceClient<xf_mic_asr_offline::Start_Record_srv>("/xf_asr_offline_node/start_record_srv");
    semantic_client = nh.serviceClient<ucarmain2026::GetTaskSemantics>("/get_task_semantics");
    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");
    classifier_client = nh.serviceClient<ucarmain2026::ItemClassify>("/get_item_classification"); // 新增
    
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    tts_pub = nh.advertise<std_msgs::String>("/voice_tts", 10); // 新增

    ros::Subscriber awake_sub = nh.subscribe("/awake_flag", 10, awakeCallback);
    ros::Subscriber pcm_sub = nh.subscribe("/mic/pcm/deno", 100, pcmCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
    
    MoveBaseClient ac("move_base", true); 

    ROS_INFO("🚀 智能车总控节点已启动！等待语音唤醒...");

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
                    ROS_INFO("🎉 语义解析成功! 实体区=[%s], 仿真区=[%s]", target_real.c_str(), target_sim.c_str());
                    current_state = NAVIGATING;
                } else {
                    ROS_WARN("⚠️ 解析失败，2秒后重试..."); ros::Duration(2.0).sleep();
                }
                break;
            }

            case NAVIGATING:
            {
                ac.waitForServer();
                if (go_destination(0.75, 5.25, 0.0, ac)) { // !!! 替换为实际第一扫码点 !!!
                    odom_initialized = false; total_rotated = 0.0;
                    qr_01::qr_code srv_qr; srv_qr.request.command = -1; qr_client.call(srv_qr); // 开相机
                    ros::Duration(1.0).sleep(); 
                    current_state = QR_SCANNING;
                }
                break;
            }

            case QR_SCANNING:
            {
                geometry_msgs::Twist vel;
                double rotated_deg = std::abs(total_rotated) * 180.0 / M_PI;

                if (rotated_deg >= 275.0 || valid_qr_count >= 3) {
                    ROS_INFO("🏁 扫码阶段完成！");
                    vel.angular.z = 0.0; cmd_pub.publish(vel);
                    
                    qr_01::qr_code srv_qr; srv_qr.request.command = -2; qr_client.call(srv_qr); // 关相机
                    
                    current_state = ITEM_CLASSIFYING; // 【跳转到分类状态】
                    break;
                }

                vel.angular.z = 0.4; // 匀速逆时针
                qr_01::qr_code srv; srv.request.command = 1;
                
                if (qr_client.call(srv) && !srv.response.result.empty()) {
                    string raw_res = srv.response.result;
                    size_t split_pos = raw_res.find("|");
                    string captured_url = (split_pos != string::npos) ? raw_res.substr(split_pos + 1) : raw_res;
                    
                    if (find(scanned_urls.begin(), scanned_urls.end(), captured_url) == scanned_urls.end()) {
                        vel.angular.z = 0.0; cmd_pub.publish(vel); ros::Duration(0.5).sleep(); // 刹车
                        scanned_urls.push_back(captured_url); 
                        
                        string json = httpGet(captured_url);
                        if (extractCode(json) == 200) {
                            string res_text = extractResult(json);
                            if (valid_qr_count == 0) target_qr_1 = res_text;
                            else if (valid_qr_count == 1) target_qr_2 = res_text;
                            else if (valid_qr_count == 2) target_qr_3 = res_text;
                            valid_qr_count++;
                            ROS_INFO("📥 录入第 %d 个二维码内容: %s", valid_qr_count, res_text.c_str());
                        }
                    }
                }
                cmd_pub.publish(vel);
                break;
            }
            
            // ================= 【核心新增阶段】：大模型分类 =================
            case ITEM_CLASSIFYING:
            {
                ROS_INFO("⏳ 正在请求大模型进行物品分类推理...");
                
                ucarmain2026::ItemClassify classify_srv;
                classify_srv.request.item1 = target_qr_1;
                classify_srv.request.item2 = target_qr_2;
                classify_srv.request.item3 = target_qr_3;
                classify_srv.request.target_real = target_real;
                classify_srv.request.target_sim = target_sim;
                
                if (classifier_client.call(classify_srv) && classify_srv.response.success) {
                    final_real_item = classify_srv.response.real_item;
                    final_sim_item  = classify_srv.response.sim_item;
                    ROS_INFO("🎉 分理成功！实体区应放: %s, 仿真区应放: %s", final_real_item.c_str(), final_sim_item.c_str());
                    
                    current_state = TTS_BROADCASTING; // 【进入最终播报状态】
                } else {
                    ROS_WARN("⚠️ 分类推理失败，2秒后重试...");
                    ros::Duration(2.0).sleep();
                }
                break;
            }

            // ================= 【核心新增阶段】：TTS 语音播报 =================

            case TTS_BROADCASTING:
            {
                // 1. 英文代号转回中文（为了 TTS 播报顺畅）
                string ch_real = target_real;
                if (target_real == "food") ch_real = "食品";
                else if (target_real == "daily") ch_real = "日用品";
                else if (target_real == "electronic") ch_real = "电子产品";
                
                string ch_sim = target_sim;
                if (target_sim == "food") ch_sim = "食品";
                else if (target_sim == "daily") ch_sim = "日用品";
                else if (target_sim == "electronic") ch_sim = "电子产品";

                // 2. 拼接纯中文的播报字符串
                string tts_str = "取得" + final_real_item + "属于" + ch_real + "应放置于" + ch_real + "车间，" +
                                 "取得" + final_sim_item + "属于" + ch_sim + "应放置于" + ch_sim + "车间。";
                
                ROS_INFO("📢 准备触发语音播报：[%s]", tts_str.c_str());
                
                std_msgs::String tts_msg;
                tts_msg.data = tts_str;
                tts_pub.publish(tts_msg);
                
                // 留出时间让扬声器播报完毕（约 12 秒）
                ROS_INFO("⏳ 等待音频播报完毕...");
                ros::Duration(12.0).sleep();
                
                current_state = ALL_FINISHED;
                break;
            }
            
            case ALL_FINISHED:
                break;
        }
        rate.sleep();
    }

    ROS_INFO("=================================================");
    ROS_INFO("🏆 第一阶段【信息获取+大模型推理】全流程完美闭环！");
    ROS_INFO("=================================================");
    return 0;
}