/**
 * @file wakeup_test.cpp
 * @brief 二代车语音唤醒、9 秒录音和 Spark 任务语义解析测试
 *
 * 流程：
 * 1. speech_command_node 识别“小飞小飞”并向 /angle 发布唤醒角度；
 * 2. 本节点收到首次唤醒后关闭 speech_command_node，释放麦克风；
 * 3. 使用 ALSA 录制 9 秒任务语音；
 * 4. 调用 /get_task_semantics；
 * 5. 解析失败时播放提示音并直接重新录音，不再重新等待唤醒。
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <ucarmain2026/GetTaskSemantics.h>

#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace
{
const char* const kWakeupTopic = "/angle";
const char* const kSemanticService = "/get_task_semantics";
const char* const kSpeechNode = "/speech_command_node";

const char* const kAudioDevice = "hw:XFMDPV0018";
const char* const kAudioFile =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record/test_record.wav";
const char* const kErrorAudio =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/1.wav";

const int kRecordSeconds = 9;

bool wakeup_received = false;

void wakeupCallback(const std_msgs::Int32::ConstPtr& msg)
{
    // speech_command_node 只在检测到唤醒后才会发布 /angle。
    // 因此收到消息本身即可作为唤醒信号，同时保留角度用于日志。
    if (!wakeup_received)
    {
        wakeup_received = true;
        ROS_INFO("检测到‘小飞小飞’，唤醒角度：%d", msg->data);
    }
}

bool stopSpeechCommandNode()
{
    const std::string command = std::string("rosnode kill ") + kSpeechNode;
    const int result = std::system(command.c_str());

    if (result != 0)
    {
        ROS_ERROR("关闭 %s 失败，system 返回值：%d", kSpeechNode, result);
        return false;
    }

    ROS_INFO("%s 已关闭，等待麦克风设备释放……", kSpeechNode);
    ros::Duration(1.0).sleep();
    return true;
}

bool audioFileLooksValid()
{
    std::ifstream input(kAudioFile, std::ios::binary | std::ios::ate);
    if (!input.is_open())
    {
        return false;
    }

    // 标准 WAV 文件头为 44 字节；必须存在实际音频数据。
    return input.tellg() > static_cast<std::streampos>(44);
}

bool recordNineSeconds()
{
    // 防止录音失败后，Spark 误用上一次留下的旧音频。
    std::remove(kAudioFile);

    const std::string create_directory =
        "mkdir -p /home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record";
    if (std::system(create_directory.c_str()) != 0)
    {
        ROS_ERROR("无法创建录音目录");
        return false;
    }

    std::string command =
        std::string("arecord -q") +
        " -D " + kAudioDevice +
        " -t wav" +
        " -f S16_LE" +
        " -r 16000" +
        " -c 1" +
        " -d " + std::to_string(kRecordSeconds) +
        " \"" + kAudioFile + "\"";

    ROS_INFO("开始录制任务语音，请在 %d 秒内说完任务……", kRecordSeconds);

    const int result = std::system(command.c_str());
    if (result != 0)
    {
        ROS_ERROR("arecord 录音失败，system 返回值：%d", result);
        ROS_ERROR("请检查 arecord -l 是否存在 XFMDPV0018，以及声卡是否仍被占用");
        return false;
    }

    if (!audioFileLooksValid())
    {
        ROS_ERROR("录音命令结束，但生成的 WAV 文件不存在或没有音频数据");
        return false;
    }

    ROS_INFO("9 秒录音完成：%s", kAudioFile);
    return true;
}

bool requestTaskSemantics(ros::ServiceClient& semantic_client)
{
    ucarmain2026::GetTaskSemantics srv;

    ROS_INFO("正在调用 Spark 语音识别与任务语义解析服务……");

    if (!semantic_client.call(srv))
    {
        ROS_ERROR("调用 %s 失败，请检查 spark_semantic_server.py", kSemanticService);
        return false;
    }

    if (!srv.response.success)
    {
        ROS_WARN("Spark 未能从本次录音中解析出两个有效任务类别");
        ROS_WARN("target_real=%s, target_sim=%s",
                 srv.response.target_real.c_str(),
                 srv.response.target_sim.c_str());
        return false;
    }

    ROS_INFO("=================================================");
    ROS_INFO("任务解析成功");
    ROS_INFO("实体区目标类别：[%s]", srv.response.target_real.c_str());
    ROS_INFO("仿真区目标类别：[%s]", srv.response.target_sim.c_str());
    ROS_INFO("=================================================");
    return true;
}

void playRetryPrompt()
{
    const std::string command = std::string("aplay -q \"") + kErrorAudio + "\"";
    const int result = std::system(command.c_str());

    if (result != 0)
    {
        ROS_WARN("重录提示音播放失败，system 返回值：%d", result);
    }
}
}  // namespace

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "voice_wakeup_test_node");
    ros::NodeHandle nh;

    ros::ServiceClient semantic_client =
        nh.serviceClient<ucarmain2026::GetTaskSemantics>(kSemanticService);

    ROS_INFO("等待 Spark 服务 %s……", kSemanticService);
    semantic_client.waitForExistence();
    ROS_INFO("Spark 服务已连接");

    ros::Subscriber wakeup_sub =
        nh.subscribe<std_msgs::Int32>(kWakeupTopic, 5, wakeupCallback);

    ROS_INFO("等待语音唤醒，请说‘小飞小飞’……");

    ros::Rate rate(20);
    while (ros::ok() && !wakeup_received)
    {
        ros::spinOnce();
        rate.sleep();
    }

    if (!ros::ok())
    {
        return 1;
    }

    // 唤醒只需要一次，立即停止订阅，避免重复触发。
    wakeup_sub.shutdown();

    if (!stopSpeechCommandNode())
    {
        return 2;
    }

    // 解析失败后只重复“提示—录音—解析”，不再重新等待“小飞小飞”。
    while (ros::ok())
    {
        if (recordNineSeconds() && requestTaskSemantics(semantic_client))
        {
            return 0;
        }

        ROS_WARN("本次录音或解析失败，播放提示音后直接重新录制……");
        playRetryPrompt();
    }

    return 1;
}