/**
 * @file voice_wakeup_test.cpp
 * @brief 2026 智能车比赛 - 语音唤醒与9秒录音测试 (带自动硬件阀门控制与云端大模型解析)
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h> 

// 【核心新增】：引入咱们之前亲手编译出来的自定义服务头文件
#include <ucarmain2026/GetTaskSemantics.h> 

#include <vector>
#include <fstream>
#include <string>

// ================= 全局状态变量 =================
bool is_recording = false;
bool task_finished = false;
ros::Time record_start_time;
std::vector<char> audio_buffer;

ros::ServiceClient record_client;   // 硬件录音开关客户端
ros::ServiceClient semantic_client; // 【新增】：星火大模型语义服务客户端

// =============== C++ 手搓 WAV 44字节文件头 ===============
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

void saveAsWav(const std::string& filename, const std::vector<char>& pcm_data) {
    WavHeader header;
    header.data_size = pcm_data.size();
    header.riff_size = header.data_size + 36;

    std::ofstream out_file(filename, std::ios::binary);
    if (!out_file.is_open()) {
        ROS_ERROR("无法创建音频文件: %s", filename.c_str());
        return;
    }
    out_file.write(reinterpret_cast<const char*>(&header), sizeof(WavHeader));
    out_file.write(pcm_data.data(), pcm_data.size());
    out_file.close();
    
    ROS_INFO("✅ 音频已保存至: %s", filename.c_str());
}

// ================= ROS 回调函数 =================

void awakeCallback(const std_msgs::Int8::ConstPtr& msg) {
    if (msg->data == 1 && !is_recording && !task_finished) {
        ROS_INFO("=================================================");
        ROS_INFO("🔔 检测到小车被喊醒！信号码: %d", msg->data);
        
        xf_mic_asr_offline::Start_Record_srv srv;
        srv.request.whether_start = 1; 
        if (record_client.call(srv) && srv.response.result == "ok") {
            ROS_INFO("🚰 硬件录音水龙头已成功打开！");
        } else {
            ROS_WARN("⚠️ 呼叫开阀服务失败，但仍将尝试接收音频流...");
        }

        ROS_INFO("🔴 触发录音！请在 9 秒内对麦克风说话...");
        ROS_INFO("=================================================");
        
        audio_buffer.clear(); 
        audio_buffer.reserve(16000 * 2 * 10); 
        is_recording = true;
        record_start_time = ros::Time::now(); 
    }
}

void pcmCallback(const xf_mic_asr_offline::Pcm_Msg::ConstPtr& msg) {
    if (is_recording) {
        audio_buffer.insert(audio_buffer.end(), msg->pcm_buf.begin(), msg->pcm_buf.end());
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    
    ros::init(argc, argv, "voice_wakeup_test_node");
    ros::NodeHandle nh;

    ROS_INFO("🎙️ 唤醒+录音测试节点启动...");
    ROS_INFO("-> 请对麦克风大喊：‘小飞小飞’ ");

    // 1. 初始化服务客户端
    record_client = nh.serviceClient<xf_mic_asr_offline::Start_Record_srv>("/xf_asr_offline_node/start_record_srv");
    
    // 【新增】：初始化语义解析服务客户端
    semantic_client = nh.serviceClient<ucarmain2026::GetTaskSemantics>("/get_task_semantics");
    
    // 2. 订阅
    ros::Subscriber awake_sub = nh.subscribe("/awake_flag", 10, awakeCallback);
    ros::Subscriber pcm_sub = nh.subscribe("/mic/pcm/deno", 100, pcmCallback);

    ros::Rate rate(10); 
    while (ros::ok() && !task_finished) {
        ros::spinOnce(); 
        
        if (is_recording) {
            // 检查是否录满 9 秒
            if ((ros::Time::now() - record_start_time).toSec() >= 9.0) {
                is_recording = false; 
                
                ROS_INFO("⏹️ 9秒时间到！正在关闭硬件水龙头...");
                xf_mic_asr_offline::Start_Record_srv srv_rec;
                srv_rec.request.whether_start = 0; 
                record_client.call(srv_rec);

                std::string save_path = "/home/ucar/ucar_car/wakeup_record/test_record.wav";
                saveAsWav(save_path, audio_buffer);

                // =========================================================
                // 【核心新增】：死循环呼叫星火大模型服务，直到成功解析出两个目标
                // =========================================================
                bool parse_success = false;
                ucarmain2026::GetTaskSemantics srv_task;

                while (ros::ok() && !parse_success) {
                    ROS_INFO("⏳ 正在呼叫星火大模型服务，预计需要 4~5 秒，请耐心等待...");
                    
                    // .call() 会在这里阻塞卡住，直到 Python 节点返回结果
                    if (semantic_client.call(srv_task)) {
                        if (srv_task.response.success) {
                            ROS_INFO("=================================================");
                            ROS_INFO("🎉 任务解析成功！");
                            ROS_INFO("-> 实体区要去抓：[%s]", srv_task.response.target_real.c_str());
                            ROS_INFO("-> 仿真区要去抓：[%s]", srv_task.response.target_sim.c_str());
                            ROS_INFO("=================================================");
                            parse_success = true; // 打破死循环
                        } else {
                            ROS_WARN("⚠️ 解析失败（语音中未找到足够的规定物品），2秒后将重新发起调用...");
                            ros::Duration(2.0).sleep();
                        }
                    } else {
                        ROS_ERROR("❌ 呼叫 Python 节点失败！请检查 Python 服务端是否已启动。3秒后重试...");
                        ros::Duration(3.0).sleep();
                    }
                }
                
                // 全部测试跑通，结束程序
                task_finished = true; 
                ROS_INFO("🎉 程序自动退出。");
            }
        }
        rate.sleep();
    }

    return 0;
}