/**
 * @file wakeup_test.cpp
 * @brief 2026 智能车比赛：语音唤醒 + 同流 PCM + Python VAD 动态录音测试
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ucarmain2026/GetTaskSemantics.h>

#include <cerrno>
#include <clocale>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr int kSampleRate = 16000;
constexpr int kChannels = 1;
constexpr int kSampleWidth = 2;
constexpr std::size_t kPrebufferBytes =
    static_cast<std::size_t>(kSampleRate * kChannels * kSampleWidth * 0.30);

bool g_task_finished = false;
bool g_vad_running = false;
bool g_pipe_error_reported = false;
bool g_pcm_received = false;
bool g_wakeup_latched = false;
std::uint64_t g_pcm_bytes_received = 0;

pid_t g_vad_pid = -1;
int g_vad_stdin_fd = -1;

std::deque<uint8_t> g_prebuffer;

ros::ServiceClient g_semantic_client;
ros::Subscriber g_awake_sub;
ros::Subscriber g_pcm_sub;

std::string g_vad_script;
std::string g_output_wav;
std::string g_status_file;
std::string g_retry_audio;
std::string g_vad_backend;
std::string g_wakeup_topic;
std::string g_pcm_topic;

double g_min_seconds = 2.0;
double g_silence_seconds = 1.2;
double g_max_seconds = 9.0;
double g_tail_seconds = 0.30;

bool ensureDirectory(const std::string& directory) {
    if (::mkdir(directory.c_str(), 0755) == 0 || errno == EEXIST) {
        return true;
    }

    ROS_ERROR(
        "无法创建录音目录 %s：%s",
        directory.c_str(),
        std::strerror(errno)
    );
    return false;
}

std::string parentDirectory(const std::string& path) {
    const std::string::size_type position = path.find_last_of('/');
    if (position == std::string::npos) {
        return ".";
    }
    if (position == 0) {
        return "/";
    }
    return path.substr(0, position);
}

bool fileLooksLikeWav(const std::string& path) {
    struct stat file_stat {};
    return ::stat(path.c_str(), &file_stat) == 0 && file_stat.st_size > 44;
}

bool writeAllToVad(const uint8_t* data, std::size_t size) {
    if (g_vad_stdin_fd < 0 || data == nullptr || size == 0) {
        return false;
    }

    std::size_t offset = 0;
    while (offset < size) {
        const ssize_t written = ::write(
            g_vad_stdin_fd,
            data + offset,
            size - offset
        );

        if (written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }

        if (written < 0 && errno == EINTR) {
            continue;
        }

        if (!g_pipe_error_reported) {
            ROS_WARN(
                "向 vad_record.py 写入 PCM 失败：%s",
                std::strerror(errno)
            );
            g_pipe_error_reported = true;
        }
        return false;
    }

    return true;
}

void closeVadPipe() {
    if (g_vad_stdin_fd >= 0) {
        ::close(g_vad_stdin_fd);
        g_vad_stdin_fd = -1;
    }
}

bool startVadProcess(bool include_prebuffer) {
    if (g_vad_running) {
        ROS_WARN("VAD 已经在运行，忽略重复启动请求");
        return false;
    }

    if (!ensureDirectory(parentDirectory(g_output_wav))) {
        return false;
    }

    ::unlink(g_output_wav.c_str());
    ::unlink(g_status_file.c_str());

    int pipe_fds[2] = {-1, -1};
    if (::pipe(pipe_fds) != 0) {
        ROS_ERROR("创建 VAD PCM 管道失败：%s", std::strerror(errno));
        return false;
    }

    const std::string min_seconds = std::to_string(g_min_seconds);
    const std::string silence_seconds = std::to_string(g_silence_seconds);
    const std::string max_seconds = std::to_string(g_max_seconds);
    const std::string tail_seconds = std::to_string(g_tail_seconds);

    const pid_t child = ::fork();
    if (child < 0) {
        ROS_ERROR("fork vad_record.py 失败：%s", std::strerror(errno));
        ::close(pipe_fds[0]);
        ::close(pipe_fds[1]);
        return false;
    }

    if (child == 0) {
        ::close(pipe_fds[1]);

        if (::dup2(pipe_fds[0], STDIN_FILENO) < 0) {
            _exit(126);
        }
        ::close(pipe_fds[0]);

        ::execlp(
            "python3",
            "python3",
            "-u",
            g_vad_script.c_str(),
            "--output",
            g_output_wav.c_str(),
            "--status-file",
            g_status_file.c_str(),
            "--sample-rate",
            "16000",
            "--channels",
            "1",
            "--sample-width",
            "2",
            "--frame-ms",
            "20",
            "--min-seconds",
            min_seconds.c_str(),
            "--silence-seconds",
            silence_seconds.c_str(),
            "--max-seconds",
            max_seconds.c_str(),
            "--tail-seconds",
            tail_seconds.c_str(),
            "--backend",
            g_vad_backend.c_str(),
            static_cast<char*>(nullptr)
        );

        _exit(127);
    }

    ::close(pipe_fds[0]);
    g_vad_pid = child;
    g_vad_stdin_fd = pipe_fds[1];
    g_vad_running = true;
    g_pipe_error_reported = false;

    ROS_INFO(
        "VAD 动态录音已启动：最短 %.1fs，静音 %.1fs，最长 %.1fs",
        g_min_seconds,
        g_silence_seconds,
        g_max_seconds
    );

    if (include_prebuffer && !g_prebuffer.empty()) {
        std::string buffered_pcm;
        buffered_pcm.reserve(g_prebuffer.size());
        for (const uint8_t value : g_prebuffer) {
            buffered_pcm.push_back(static_cast<char>(value));
        }

        writeAllToVad(
            reinterpret_cast<const uint8_t*>(buffered_pcm.data()),
            buffered_pcm.size()
        );

        ROS_INFO(
            "已向 VAD 补入 %.0f ms 唤醒衔接音频",
            1000.0 * g_prebuffer.size()
                / static_cast<double>(
                    kSampleRate * kChannels * kSampleWidth
                )
        );
    }

    return true;
}

void appendToPrebuffer(
    const uint8_t* data,
    std::size_t size
) {
    if (data == nullptr || size == 0) {
        return;
    }

    for (std::size_t index = 0; index < size; ++index) {
        g_prebuffer.push_back(data[index]);
    }

    while (g_prebuffer.size() > kPrebufferBytes) {
        g_prebuffer.pop_front();
    }
}

bool requestSemantics() {
    ROS_INFO("正在调用任务语义解析服务...");

    ucarmain2026::GetTaskSemantics service;
    if (g_semantic_client.call(service) && service.response.success) {
        ROS_INFO("==============================================");
        ROS_INFO("任务解析成功");
        ROS_INFO(
            "实体区目标：[%s]",
            service.response.target_real.c_str()
        );
        ROS_INFO(
            "仿真区目标：[%s]",
            service.response.target_sim.c_str()
        );
        ROS_INFO("==============================================");
        return true;
    }

    ROS_WARN("任务语义解析失败，准备播放提示音后重新录音");
    return false;
}

void startRetryRecording() {
    const std::string command = "aplay -q '" + g_retry_audio + "'";
    const int play_result = std::system(command.c_str());
    if (play_result != 0) {
        ROS_WARN(
            "重录提示音播放失败，system 返回值：%d",
            play_result
        );
    }

    g_prebuffer.clear();
    if (!startVadProcess(false)) {
        ROS_ERROR("重新启动 VAD 失败，请再次说唤醒词重试");
        g_wakeup_latched = false;
    } else {
        ROS_INFO("请重新说出完整任务，VAD 将在说完后自动停止");
    }
}

void handleVadExit(int status) {
    g_vad_running = false;
    g_vad_pid = -1;
    closeVadPipe();

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        if (WIFEXITED(status)) {
            ROS_ERROR(
                "vad_record.py 异常退出，退出码：%d",
                WEXITSTATUS(status)
            );
        } else if (WIFSIGNALED(status)) {
            ROS_ERROR(
                "vad_record.py 被信号 %d 终止",
                WTERMSIG(status)
            );
        } else {
            ROS_ERROR("vad_record.py 未正常结束");
        }

        ROS_INFO("请再次说“小飞小飞”重新测试");
        g_wakeup_latched = false;
        return;
    }

    if (!fileLooksLikeWav(g_output_wav)) {
        ROS_ERROR(
            "VAD 已退出，但没有生成有效 WAV：%s",
            g_output_wav.c_str()
        );
        ROS_INFO("请再次说“小飞小飞”重新测试");
        g_wakeup_latched = false;
        return;
    }

    ROS_INFO("VAD 录音完成：%s", g_output_wav.c_str());

    if (requestSemantics()) {
        g_task_finished = true;
        return;
    }

    startRetryRecording();
}

void pollVadProcess() {
    if (!g_vad_running || g_vad_pid <= 0) {
        return;
    }

    int status = 0;
    const pid_t result = ::waitpid(g_vad_pid, &status, WNOHANG);
    if (result == 0) {
        return;
    }

    if (result < 0) {
        if (errno != EINTR) {
            ROS_ERROR("waitpid 检查 VAD 进程失败：%s", std::strerror(errno));
        }
        return;
    }

    handleVadExit(status);
}

void awakeCallback(const std_msgs::Int32::ConstPtr& message) {
    if (g_wakeup_latched || g_vad_running || g_task_finished) {
        return;
    }
    g_wakeup_latched = true;

    ROS_INFO("==============================================");
    ROS_INFO(
        "检测到“小飞小飞”，唤醒角度：%d，立即启动同流 VAD 录音",
        message->data
    );

    if (g_pcm_sub.getNumPublishers() == 0) {
        ROS_ERROR(
            "无法启动 VAD：PCM 话题 %s 没有发布者",
            g_pcm_topic.c_str()
        );
        ROS_ERROR(
            "请检查麦克风节点是否启动，并执行：rostopic info %s",
            g_pcm_topic.c_str()
        );
        ROS_INFO("==============================================");
        return;
    }

    if (!g_pcm_received) {
        ROS_WARN(
            "PCM 话题已有发布者，但暂未收到数据；先启动 VAD 并等待数据"
        );
    }

    if (!startVadProcess(true)) {
        ROS_ERROR("VAD 启动失败");
        g_wakeup_latched = false;
        return;
    }

    ROS_INFO("请直接衔接说任务内容，不需要等待提示");
    ROS_INFO("==============================================");
}

void pcmCallback(
    const std_msgs::UInt8MultiArray::ConstPtr& message
) {
    if (message->data.empty()) {
        return;
    }

    const uint8_t* data = message->data.data();
    const std::size_t size = message->data.size();

    if (!g_pcm_received) {
        g_pcm_received = true;
        ROS_INFO(
            "已收到 PCM 音频流：话题=%s，首包=%zu 字节",
            g_pcm_topic.c_str(),
            size
        );
    }
    g_pcm_bytes_received += static_cast<std::uint64_t>(size);

    appendToPrebuffer(data, size);

    if (g_vad_running) {
        writeAllToVad(data, size);
    }
}

void stopVadProcess() {
    closeVadPipe();

    if (g_vad_pid > 0) {
        ::kill(g_vad_pid, SIGTERM);
        ::waitpid(g_vad_pid, nullptr, 0);
    }

    g_vad_pid = -1;
    g_vad_running = false;
}

}  // namespace

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    std::signal(SIGPIPE, SIG_IGN);

    ros::init(argc, argv, "voice_wakeup_test_node");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node("~");

    private_node.param<std::string>(
        "vad_script",
        g_vad_script,
        "/home/ucar/ucar_ws_copy/src/ucarmain2026/scripts/vad_record.py"
    );
    private_node.param<std::string>(
        "output_wav",
        g_output_wav,
        "/home/ucar/ucar_ws_copy/src/ucarmain2026/"
        "wakeup_record/test_record.wav"
    );
    private_node.param<std::string>(
        "status_file",
        g_status_file,
        "/home/ucar/ucar_ws_copy/src/ucarmain2026/"
        "wakeup_record/test_record.vad.json"
    );
    private_node.param<std::string>(
        "retry_audio",
        g_retry_audio,
        "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/1.wav"
    );
    private_node.param<std::string>(
        "vad_backend",
        g_vad_backend,
        "auto"
    );
    private_node.param<std::string>(
        "wakeup_topic",
        g_wakeup_topic,
        "/angle"
    );
    private_node.param<std::string>(
        "pcm_topic",
        g_pcm_topic,
        "/mic/pcm/deno"
    );
    private_node.param("vad_min_seconds", g_min_seconds, 2.0);
    private_node.param("vad_silence_seconds", g_silence_seconds, 1.2);
    private_node.param("vad_max_seconds", g_max_seconds, 9.0);
    private_node.param("vad_tail_seconds", g_tail_seconds, 0.30);

    g_semantic_client =
        node_handle.serviceClient<ucarmain2026::GetTaskSemantics>(
            "/get_task_semantics"
        );

    g_awake_sub = node_handle.subscribe(
        g_wakeup_topic,
        10,
        awakeCallback
    );
    g_pcm_sub = node_handle.subscribe(
        g_pcm_topic,
        100,
        pcmCallback
    );
    node_handle.setParam(
        "/speech_command/internal_task_recording",
        false
    );

    ROS_INFO("VAD 唤醒录音测试节点已启动");
    ROS_INFO("请连续说：“小飞小飞”+任务内容");
    ROS_INFO(
        "接口：唤醒=%s(std_msgs/Int32)，PCM=%s",
        g_wakeup_topic.c_str(),
        g_pcm_topic.c_str()
    );
    ROS_INFO(
        "参数：min=%.1fs，silence=%.1fs，max=%.1fs，backend=%s",
        g_min_seconds,
        g_silence_seconds,
        g_max_seconds,
        g_vad_backend.c_str()
    );

    ros::Rate loop_rate(50);
    ros::Time next_diagnostic_time = ros::Time::now() + ros::Duration(2.0);
    while (ros::ok() && !g_task_finished) {
        ros::spinOnce();
        pollVadProcess();

        if (
            !g_vad_running
            && ros::Time::now() >= next_diagnostic_time
        ) {
            const std::uint32_t wakeup_publishers =
                g_awake_sub.getNumPublishers();
            const std::uint32_t pcm_publishers =
                g_pcm_sub.getNumPublishers();

            if (wakeup_publishers == 0 || pcm_publishers == 0) {
                ROS_WARN(
                    "上游连接诊断：%s 发布者=%u，%s 发布者=%u，"
                    "累计 PCM=%llu 字节",
                    g_wakeup_topic.c_str(),
                    wakeup_publishers,
                    g_pcm_topic.c_str(),
                    pcm_publishers,
                    static_cast<unsigned long long>(g_pcm_bytes_received)
                );
            } else if (!g_pcm_received) {
                ROS_WARN(
                    "两个话题均已连接，但 %s 尚未收到 PCM 数据",
                    g_pcm_topic.c_str()
                );
            }

            next_diagnostic_time =
                ros::Time::now() + ros::Duration(2.0);
        }

        loop_rate.sleep();
    }

    stopVadProcess();
    node_handle.setParam(
        "/speech_command/internal_task_recording",
        true
    );
    return 0;
}