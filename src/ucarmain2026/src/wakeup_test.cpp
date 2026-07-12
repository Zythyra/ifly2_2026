/**
 * @file voice_wakeup_test.cpp
 * @brief 2026 智能车比赛 - 语音唤醒与9秒录音测试 (带失败自动重试与播报)
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h> 
#include <ucarmain2026/GetTaskSemantics.h> 

#include <vector>
#include <fstream>
#include <string>
#include <cstdlib> // 用于 system() 播放系统音频

// ================= 全局状态变量 =================
bool is_recording = false;
bool task_finished = false;
ros::Time record_start_time;
std::vector<char> audio_buffer;

ros::ServiceClient record_client;   
ros::ServiceClient semantic_client; 

// =============== WAV 44字节文件头 ===============
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

    record_client = nh.serviceClient<xf_mic_asr_offline::Start_Record_srv>("/xf_asr_offline_node/start_record_srv");
    semantic_client = nh.serviceClient<ucarmain2026::GetTaskSemantics>("/get_task_semantics");
    
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

                std::string save_path = "/home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record/test_record.wav";
                saveAsWav(save_path, audio_buffer);

                ROS_INFO("⏳ 正在呼叫星火大模型服务进行语义解析...");
                ucarmain2026::GetTaskSemantics srv_task;

                // 请求大模型服务
                if (semantic_client.call(srv_task) && srv_task.response.success) {
                    // 解析成功
                    ROS_INFO("=================================================");
                    ROS_INFO("🎉 任务解析成功！");
                    ROS_INFO("-> 实体区要去抓：[%s]", srv_task.response.target_real.c_str());
                    ROS_INFO("-> 仿真区要去抓：[%s]", srv_task.response.target_sim.c_str());
                    ROS_INFO("=================================================");
                    task_finished = true; // 标志任务完成，准备退出程序
                } 
                else {
                    // =======================================================
                    // 【核心修改】：解析失败时的重试机制
                    // =======================================================
                    ROS_WARN("⚠️ 解析失败（语音中未找到足够的规定物品或网络异常）！");
                    ROS_INFO("📢 正在播放提示音，准备重新录制...");
                    
                    // system 会阻塞线程，必须等小车说完这句话，代码才会往下执行，防止自己录到自己的声音
                    system("aplay -q /home/ucar/ucar_ws_copy/src/ucarmain2026/audios/1.wav");
                    
                    // 重新开启麦克风硬件录音
                    srv_rec.request.whether_start = 1;
                    if (record_client.call(srv_rec) && srv_rec.response.result == "ok") {
                        ROS_INFO("🚰 硬件录音水龙头已重新打开！");
                    } else {
                        ROS_WARN("⚠️ 重新呼叫开阀服务失败，但仍将尝试接收音频流...");
                    }
                    
                    ROS_INFO("🔴 触发重新录音！请在 9 秒内再次对麦克风说话...");
                    audio_buffer.clear();
                    is_recording = true; // 标记重新开始录制
                    record_start_time = ros::Time::now(); // 重置计时器
                }
            }
        }
        rate.sleep();
    }

    return 0;
}