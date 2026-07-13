/**
 * @file main_competition_node.cpp
 * @brief 2026 智能车比赛 - 核心流程总控程序（语音任务 + 定点扫码 + 三段语音播报）
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

// 自定义服务
#include <ucarmain2026/GetTaskSemantics.h> 
#include <ucarmain2026/ItemClassify.h> 
#include <qr_01/qr_code.h>
#include <curl/curl.h>

#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <cstdlib> // 用于 system()
#include <cstdio>

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

// ================= 客户端与发布者 =================
ros::ServiceClient semantic_client;
ros::ServiceClient qr_client;
ros::ServiceClient classifier_client; 

// ================= 二代车语音唤醒与录音 =================
const char* const WAKEUP_TOPIC = "/angle";
const char* const SPEECH_NODE = "/speech_command_node";
const char* const AUDIO_DEVICE = "hw:XFMDPV0018";
const char* const AUDIO_FILE =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record/test_record.wav";
const char* const ERROR_AUDIO =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/1.wav";
const int RECORD_SECONDS = 9;

bool wakeup_received = false;

void awakeCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (current_state == WAIT_WAKEUP && !wakeup_received) {
        wakeup_received = true;
        ROS_INFO("检测到‘小飞小飞’，唤醒角度：%d", msg->data);
    }
}

bool stopSpeechCommandNode() {
    const string command = string("rosnode kill ") + SPEECH_NODE;
    const int kill_result = system(command.c_str());

    if (kill_result != 0) {
        ROS_WARN("关闭 %s 时命令返回非零值：%d", SPEECH_NODE, kill_result);
    }

    ROS_INFO("等待 speech_command_node 退出并释放麦克风...");
    ros::Duration(1.0).sleep();

    // grep 找到节点时返回 0；未找到时返回非 0。
    const int still_running =
        system("rosnode list 2>/dev/null | grep -qx '/speech_command_node'");
    if (still_running == 0) {
        ROS_ERROR("%s 仍在运行，暂不开始录音", SPEECH_NODE);
        return false;
    }

    ROS_INFO("%s 已退出，麦克风可供任务录音使用", SPEECH_NODE);
    return true;
}

bool audioFileLooksValid() {
    ifstream input(AUDIO_FILE, ios::binary | ios::ate);
    if (!input.is_open()) {
        return false;
    }

    return input.tellg() > static_cast<streampos>(44);
}

bool recordNineSeconds() {
    // 避免录音失败后 Spark 误用上一次的旧文件。
    std::remove(AUDIO_FILE);

    if (system("mkdir -p /home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record") != 0) {
        ROS_ERROR("无法创建任务录音目录");
        return false;
    }

    const string command =
        string("arecord -q") +
        " -D " + AUDIO_DEVICE +
        " -t wav" +
        " -f S16_LE" +
        " -r 16000" +
        " -c 1" +
        " -d " + to_string(RECORD_SECONDS) +
        " \"" + AUDIO_FILE + "\"";

    ROS_INFO("开始录制任务语音，请在 %d 秒内说完任务...", RECORD_SECONDS);
    const int result = system(command.c_str());

    if (result != 0) {
        ROS_ERROR("arecord 录音失败，system 返回值：%d", result);
        ROS_ERROR("请检查 XFMDPV0018 是否存在以及麦克风是否仍被占用");
        return false;
    }

    if (!audioFileLooksValid()) {
        ROS_ERROR("录音文件不存在或没有有效音频数据");
        return false;
    }

    ROS_INFO("9 秒录音完成：%s", AUDIO_FILE);
    return true;
}

void playRetryPrompt() {
    const string command = string("aplay -q \"") + ERROR_AUDIO + "\"";
    const int result = system(command.c_str());
    if (result != 0) {
        ROS_WARN("重录提示音播放失败，system 返回值：%d", result);
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

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    ros::init(argc, argv, "main_competition_node");
    ros::NodeHandle nh;

    semantic_client = nh.serviceClient<ucarmain2026::GetTaskSemantics>("/get_task_semantics");
    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");
    classifier_client = nh.serviceClient<ucarmain2026::ItemClassify>("/get_item_classification"); 
    
    ros::Subscriber awake_sub =
        nh.subscribe<std_msgs::Int32>(WAKEUP_TOPIC, 5, awakeCallback);
    
    MoveBaseClient ac("move_base", true); 

    ROS_INFO("等待 Spark 语义服务 /get_task_semantics...");
    semantic_client.waitForExistence();
    ROS_INFO("智能车总控节点已启动！请说‘小飞小飞’唤醒...");

    ros::Rate rate(20); 
    while (ros::ok() && current_state != ALL_FINISHED) {
        ros::spinOnce(); 

        switch (current_state) {
            case WAIT_WAKEUP:
                if (wakeup_received) {
                    awake_sub.shutdown();

                    if (stopSpeechCommandNode()) {
                        current_state = RECORDING;
                    } else {
                        ROS_WARN("speech_command_node 尚未完全退出，1 秒后重试...");
                        ros::Duration(1.0).sleep();
                    }
                }
                break;

            case RECORDING:
                if (recordNineSeconds()) {
                    current_state = SEMANTIC_PARSING;
                } else {
                    ROS_WARN("录音失败，播放提示音后直接重新录制...");
                    playRetryPrompt();
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
                    ROS_WARN("解析失败！播放提示音后直接重新录制...");
                    playRetryPrompt();
                    current_state = RECORDING;
                }
                break;
            }

            case NAVIGATING:
            {
                ac.waitForServer();
                double qr_wp_x[4]   = {1.0, 0.75, 0.5, 0.75};
                double qr_wp_y[4]   = {5.25, 5.0, 5.25, 5.5};
                double qr_wp_yaw[4] = {3.14, 1.57, 0.0, -1.57};

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

            // ================= 二维码分类完成后，生成并依次播放三段任务语音 =================
            case TTS_BROADCASTING:
            {
                string ch_real_category = target_real;
                string ch_real_workshop = target_real + "车间"; 
                if (target_real == "food") { ch_real_category = "食品"; ch_real_workshop = "食品加工车间"; } 
                else if (target_real == "daily") { ch_real_category = "日用品"; ch_real_workshop = "日用品加工车间"; } 
                else if (target_real == "electronic") { ch_real_category = "电子产品"; ch_real_workshop = "电子产品生产车间"; }
                
                string ch_sim_category = target_sim;
                string ch_sim_workshop = target_sim + "车间"; 
                if (target_sim == "food") { ch_sim_category = "食品"; ch_sim_workshop = "食品加工车间"; } 
                else if (target_sim == "daily") { ch_sim_category = "日用品"; ch_sim_workshop = "日用品加工车间"; } 
                else if (target_sim == "electronic") { ch_sim_category = "电子产品"; ch_sim_workshop = "电子产品生产车间"; }

                string text2 = "取得" + final_real_item + "属于" + ch_real_category + "，应放置于" + ch_real_workshop + "；" +
                               "仿真环境中取得" + final_sim_item + "属于" + ch_sim_category + "，应放置于" + ch_sim_workshop + "。";
                string text3 = "已将" + final_real_item + "放入" + ch_real_workshop + "。";
                string text4 = "仿真任务已完成，已将" + final_sim_item + "放入" + ch_sim_category + "仓库。";
                
                ROS_INFO("正在调用本地脚本批量合成音频（2.wav、3.wav、4.wav）...");
                string cmd = "python3 /home/ucar/ucar_ws_copy/src/ucarmain2026/scripts/generate_task_audios.py \"" +
                             text2 + "\" \"" + text3 + "\" \"" + text4 + "\"";
                const int generate_result = system(cmd.c_str());
                if (generate_result != 0) {
                    ROS_ERROR("三段任务语音合成失败，system 返回值：%d", generate_result);
                    current_state = ALL_FINISHED;
                    break;
                }
                
                ROS_INFO("播放第 1 段：分类汇总（2.wav）...");
                system("aplay -q /home/ucar/ucar_ws_copy/src/ucarmain2026/audios/2.wav");
                ROS_INFO("播放第 2 段：实体区放置结果（3.wav）...");
                system("aplay -q /home/ucar/ucar_ws_copy/src/ucarmain2026/audios/3.wav");
                
                ROS_INFO("播放第 3 段：仿真区放置结果（4.wav）...");
                system("aplay -q /home/ucar/ucar_ws_copy/src/ucarmain2026/audios/4.wav");
                
                current_state = ALL_FINISHED;
                break;
            }
            
            case ALL_FINISHED:
                break;
        } 
        
        rate.sleep();
    } 

    
    ROS_INFO("二维码识别、分类及三段语音播报全部完成");
    
    return 0;
}