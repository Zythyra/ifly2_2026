/**
 * @file voice_wakeup_test.cpp
 * @brief 2026 智能车比赛 - 语音唤醒与9秒录音测试 (带自动硬件阀门控制)
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h> // 【新增】录音服务头文件
#include <vector>
#include <fstream>
#include <string>

// ================= 全局状态变量 =================
bool is_recording = false;
bool task_finished = false;
ros::Time record_start_time;
std::vector<char> audio_buffer;
ros::ServiceClient record_client; // 声明服务客户端

// =============== C++ 手搓 WAV 44字节文件头 ===============
#pragma pack(push, 1)
struct WavHeader {
    char riff_id[4] = {'R', 'I', 'F', 'F'};
    uint32_t riff_size;
    char wave_id[4] = {'W', 'A', 'V', 'E'};
    char fmt_id[4] = {'f', 'm', 't', ' '};
    uint32_t fmt_size = 16;
    uint16_t audio_format = 1;      // PCM = 1
    uint16_t num_channels = 1;      
    uint32_t sample_rate = 16000;   // 16kHz
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
    
    ROS_INFO("✅ 测试成功！音频已保存至: %s", filename.c_str());
    if (pcm_data.empty()) {
        ROS_ERROR("🚨 警告：录音文件大小为 0 字节！");
    } else {
        ROS_INFO("-> 文件大小: %zu 字节 (约 %.1f 秒)", pcm_data.size(), pcm_data.size() / 32000.0);
    }
}

// ================= ROS 回调函数 =================

void awakeCallback(const std_msgs::Int8::ConstPtr& msg) {
    if (msg->data == 1 && !is_recording && !task_finished) {
        ROS_INFO("=================================================");
        ROS_INFO("🔔 检测到小车被喊醒！信号码: %d", msg->data);
        
        // 【核心绝杀】：在这里向底层发送指令，暴力拧开硬件录音水龙头！
        xf_mic_asr_offline::Start_Record_srv srv;
        srv.request.whether_start = 1; // 1代表开启录音
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
        // 疯狂吸入经过降噪的甜美音频数据
        audio_buffer.insert(audio_buffer.end(), msg->pcm_buf.begin(), msg->pcm_buf.end());
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    
    ros::init(argc, argv, "voice_wakeup_test_node");
    ros::NodeHandle nh;

    ROS_INFO("🎙️ 唤醒+录音测试节点启动...");
    ROS_INFO("-> 请对麦克风大喊：‘小飞小飞’ ");

    // 1. 初始化服务客户端（去请求刚刚查到的服务绝对路径）
    record_client = nh.serviceClient<xf_mic_asr_offline::Start_Record_srv>("/xf_asr_offline_node/start_record_srv");
    
    // 2. 订阅唤醒标志
    ros::Subscriber awake_sub = nh.subscribe("/awake_flag", 10, awakeCallback);
    
    // 3. 订阅降噪音频水管
    ros::Subscriber pcm_sub = nh.subscribe("/mic/pcm/deno", 100, pcmCallback);

    ros::Rate rate(10); 
    while (ros::ok() && !task_finished) {
        ros::spinOnce(); 
        
        if (is_recording) {
            // 检查是否录满 9 秒
            if ((ros::Time::now() - record_start_time).toSec() >= 9.0) {
                is_recording = false; 
                task_finished = true; 
                
                ROS_INFO("⏹️ 9秒时间到！正在关闭硬件水龙头...");
                
                // 【善后处理】：发送指令关掉底层麦克风，节省资源
                xf_mic_asr_offline::Start_Record_srv srv;
                srv.request.whether_start = 0; // 0代表停止录音
                record_client.call(srv);

                ROS_INFO("正在打包为 WAV 文件...");
                std::string save_path = "/home/ucar/ucar_car/wakeup_record/test_record.wav";
                saveAsWav(save_path, audio_buffer);
                ROS_INFO("🎉 程序自动退出。");
            }
        }
        rate.sleep();
    }

    return 0;
}