/**
 * @file race.cpp
 * @brief 2026 智能车比赛 - 核心流程总控程序
 *        （语音任务 + 定点扫码 + Sherpa-ONNX 离线语音播报）
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

// 自定义服务
#include <ucarmain2026/GetTaskSemantics.h>
#include <ucarmain2026/ItemClassify.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <clocale>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

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
ros::Publisher target_class_pub;

// ================= 二代车语音唤醒、录音与播报 =================
const char* const WAKEUP_TOPIC = "/angle";
const char* const PCM_TOPIC = "/mic/pcm/deno";
const char* const SPEECH_NODE = "/speech_command_node";
const char* const AUDIO_FILE =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record/test_record.wav";
const char* const VAD_STATUS_FILE =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/wakeup_record/test_record.vad.json";
const char* const VAD_SCRIPT =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/scripts/vad_record.py";
const char* const ERROR_AUDIO =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/1.wav";
const char* const TASK_AUDIO_SCRIPT =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/scripts/generate_task_audios.py";
const char* const AUDIO_DIR =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios";

bool wakeup_received = false;

constexpr int PCM_SAMPLE_RATE = 16000;
constexpr int PCM_CHANNELS = 1;
constexpr int PCM_SAMPLE_WIDTH = 2;
constexpr size_t PCM_PREBUFFER_BYTES =
    static_cast<size_t>(
        PCM_SAMPLE_RATE * PCM_CHANNELS * PCM_SAMPLE_WIDTH * 0.30
    );

bool vad_running = false;
bool vad_pipe_error_reported = false;
bool pcm_received = false;
pid_t vad_pid = -1;
int vad_stdin_fd = -1;
deque<uint8_t> pcm_prebuffer;
ros::Subscriber pcm_sub;

string vad_script = VAD_SCRIPT;
string vad_status_file = VAD_STATUS_FILE;
string vad_backend = "auto";
double vad_min_seconds = 2.0;
double vad_silence_seconds = 1.2;
double vad_max_seconds = 9.0;
double vad_tail_seconds = 0.30;

/**
 * @brief 把任意 UTF-8 文本安全地作为一个 shell 参数传给 system()。
 *
 * 二维码或分类结果中即使出现空格、引号、分号等字符，也不会破坏命令。
 */
string shellQuote(const string& value) {
    string quoted = "'";
    for (char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted += character;
        }
    }
    quoted += "'";
    return quoted;
}

bool playAudio(const string& audio_file, const string& description) {
    ROS_INFO("播放%s：%s", description.c_str(), audio_file.c_str());
    const string command = "aplay -q " + shellQuote(audio_file);
    const int result = system(command.c_str());

    if (result != 0) {
        ROS_WARN(
            "%s播放失败，system 返回值：%d",
            description.c_str(),
            result
        );
        return false;
    }
    return true;
}

bool isSafeAudioText(const string& text) {
    if (text.empty() || text == "." || text == ".." || text.size() > 200) {
        return false;
    }

    for (unsigned char character : text) {
        if (
            character == '/'
            || character == '\\'
            || character < 0x20
            || character == 0x7f
        ) {
            return false;
        }
    }
    return true;
}

bool buildAudioPath(const string& text, string& output_path) {
    if (!isSafeAudioText(text)) {
        ROS_ERROR("音频文字不能安全地用作文件名：[%s]", text.c_str());
        return false;
    }

    output_path = string(AUDIO_DIR) + "/" + text + ".wav";
    return true;
}

bool audioFileExists(const string& audio_file) {
    struct stat file_status;
    return (
        stat(audio_file.c_str(), &file_status) == 0
        && S_ISREG(file_status.st_mode)
        && file_status.st_size > 44
    );
}

bool prepareItemAudios(
    const string& real_item,
    const string& sim_item
) {
    string real_audio;
    string sim_audio;
    if (
        !buildAudioPath(real_item, real_audio)
        || !buildAudioPath(sim_item, sim_audio)
    ) {
        return false;
    }

    const bool real_cached = audioFileExists(real_audio);
    const bool sim_cached = audioFileExists(sim_audio);
    ROS_INFO(
        "物品音频库检查：实体物品[%s]=%s，仿真物品[%s]=%s",
        real_item.c_str(),
        real_cached ? "命中" : "未命中",
        sim_item.c_str(),
        sim_cached ? "命中" : "未命中"
    );

    const string command =
        "python3 " + shellQuote(TASK_AUDIO_SCRIPT) + " "
        + shellQuote(real_item) + " "
        + shellQuote(sim_item);

    ROS_INFO(
        "向预加载 TTS 提交物品名；仅合成音频库中未命中的名称..."
    );
    const int result = system(command.c_str());
    if (result != 0) {
        ROS_ERROR(
            "物品名离线语音准备失败，system 返回值：%d",
            result
        );
        return false;
    }

    if (!audioFileExists(real_audio) || !audioFileExists(sim_audio)) {
        ROS_ERROR("TTS 返回成功，但所需物品音频仍不完整");
        return false;
    }

    ROS_INFO("两个物品名音频均已就绪，TTS 后台进程已退出");
    return true;
}

struct CategoryAudioTexts {
    string classification;
    string workshop_placement;
    string warehouse_placement;
};

bool getCategoryAudioTexts(
    const string& category_code,
    CategoryAudioTexts& audio_texts
) {
    if (category_code == "food") {
        audio_texts.classification =
            "属于食品，应放置于食品加工车间";
        audio_texts.workshop_placement =
            "放入食品加工车间";
        audio_texts.warehouse_placement =
            "放入食品仓库";
        return true;
    }

    if (category_code == "daily") {
        audio_texts.classification =
            "属于日用品，应放置于日用品加工车间";
        audio_texts.workshop_placement =
            "放入日用品加工车间";
        audio_texts.warehouse_placement =
            "放入日用品仓库";
        return true;
    }

    if (category_code == "electronic") {
        // 这里严格匹配已经录制好的文件名：
        // 分类说明使用“电子产品加工车间”，实际放置使用“电子产品生产车间”。
        audio_texts.classification =
            "属于电子产品，应放置于电子产品加工车间";
        audio_texts.workshop_placement =
            "放入电子产品生产车间";
        audio_texts.warehouse_placement =
            "放入电子产品仓库";
        return true;
    }

    ROS_ERROR(
        "无法为类别代码 [%s] 选择固定整句音频",
        category_code.c_str()
    );
    return false;
}

bool playAudioTextSequence(
    const vector<string>& audio_texts,
    const string& description
) {
    if (audio_texts.empty()) {
        ROS_WARN("%s为空，不执行播放", description.c_str());
        return true;
    }

    vector<string> audio_files;
    audio_files.reserve(audio_texts.size());
    for (const string& text : audio_texts) {
        string audio_file;
        if (!buildAudioPath(text, audio_file)) {
            return false;
        }
        if (!audioFileExists(audio_file)) {
            ROS_ERROR(
                "%s缺少音频片段：文字=[%s]，文件=%s",
                description.c_str(),
                text.c_str(),
                audio_file.c_str()
            );
            return false;
        }
        audio_files.push_back(audio_file);
    }

    string command = "aplay -q";
    for (const string& audio_file : audio_files) {
        command += " " + shellQuote(audio_file);
    }

    ROS_INFO(
        "开始%s，共 %zu 个音频片段",
        description.c_str(),
        audio_files.size()
    );
    const int result = system(command.c_str());
    if (result != 0) {
        ROS_WARN(
            "%s失败，system 返回值：%d",
            description.c_str(),
            result
        );
        return false;
    }
    return true;
}

bool ensureDirectory(const string& directory) {
    if (mkdir(directory.c_str(), 0755) == 0 || errno == EEXIST) {
        return true;
    }

    ROS_ERROR(
        "无法创建录音目录 %s：%s",
        directory.c_str(),
        strerror(errno)
    );
    return false;
}

string parentDirectory(const string& path) {
    const string::size_type position = path.find_last_of('/');
    if (position == string::npos) {
        return ".";
    }
    if (position == 0) {
        return "/";
    }
    return path.substr(0, position);
}

bool writeAllToVad(const uint8_t* data, size_t size) {
    if (vad_stdin_fd < 0 || data == nullptr || size == 0) {
        return false;
    }

    size_t offset = 0;
    while (offset < size) {
        const ssize_t written = write(
            vad_stdin_fd,
            data + offset,
            size - offset
        );

        if (written > 0) {
            offset += static_cast<size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }

        if (!vad_pipe_error_reported) {
            ROS_WARN(
                "向 vad_record.py 写入 PCM 失败：%s",
                strerror(errno)
            );
            vad_pipe_error_reported = true;
        }
        return false;
    }

    return true;
}

void closeVadPipe() {
    if (vad_stdin_fd >= 0) {
        close(vad_stdin_fd);
        vad_stdin_fd = -1;
    }
}

void appendToPcmPrebuffer(const uint8_t* data, size_t size) {
    if (data == nullptr || size == 0) {
        return;
    }

    for (size_t index = 0; index < size; ++index) {
        pcm_prebuffer.push_back(data[index]);
    }
    while (pcm_prebuffer.size() > PCM_PREBUFFER_BYTES) {
        pcm_prebuffer.pop_front();
    }
}

bool startVadProcess(bool include_prebuffer) {
    if (vad_running) {
        ROS_WARN("VAD 已经在运行，忽略重复启动请求");
        return false;
    }
    if (!ensureDirectory(parentDirectory(AUDIO_FILE))) {
        return false;
    }

    remove(AUDIO_FILE);
    remove(vad_status_file.c_str());

    int pipe_fds[2] = {-1, -1};
    if (pipe(pipe_fds) != 0) {
        ROS_ERROR("创建 VAD PCM 管道失败：%s", strerror(errno));
        return false;
    }

    const string min_seconds = to_string(vad_min_seconds);
    const string silence_seconds = to_string(vad_silence_seconds);
    const string max_seconds = to_string(vad_max_seconds);
    const string tail_seconds = to_string(vad_tail_seconds);

    const pid_t child = fork();
    if (child < 0) {
        ROS_ERROR("fork vad_record.py 失败：%s", strerror(errno));
        close(pipe_fds[0]);
        close(pipe_fds[1]);
        return false;
    }

    if (child == 0) {
        close(pipe_fds[1]);
        if (dup2(pipe_fds[0], STDIN_FILENO) < 0) {
            _exit(126);
        }
        close(pipe_fds[0]);

        execlp(
            "python3",
            "python3",
            "-u",
            vad_script.c_str(),
            "--output",
            AUDIO_FILE,
            "--status-file",
            vad_status_file.c_str(),
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
            vad_backend.c_str(),
            static_cast<char*>(nullptr)
        );
        _exit(127);
    }

    close(pipe_fds[0]);
    vad_pid = child;
    vad_stdin_fd = pipe_fds[1];
    vad_running = true;
    vad_pipe_error_reported = false;

    ROS_INFO(
        "VAD 动态录音已启动：最短 %.1fs，静音 %.1fs，最长 %.1fs",
        vad_min_seconds,
        vad_silence_seconds,
        vad_max_seconds
    );

    if (include_prebuffer && !pcm_prebuffer.empty()) {
        vector<uint8_t> buffered_pcm(
            pcm_prebuffer.begin(),
            pcm_prebuffer.end()
        );
        writeAllToVad(buffered_pcm.data(), buffered_pcm.size());
        ROS_INFO(
            "已向 VAD 补入 %.0f ms 唤醒衔接音频",
            1000.0 * pcm_prebuffer.size()
                / static_cast<double>(
                    PCM_SAMPLE_RATE
                    * PCM_CHANNELS
                    * PCM_SAMPLE_WIDTH
                )
        );
    }

    return true;
}

void pcmCallback(
    const std_msgs::UInt8MultiArray::ConstPtr& message
) {
    if (message->data.empty()) {
        return;
    }

    const uint8_t* data = message->data.data();
    const size_t size = message->data.size();

    if (!pcm_received) {
        pcm_received = true;
        ROS_INFO(
            "已收到 PCM 音频流：话题=%s，首包=%zu 字节",
            PCM_TOPIC,
            size
        );
    }
    appendToPcmPrebuffer(data, size);

    if (vad_running) {
        writeAllToVad(data, size);
    }
}

void awakeCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (
        current_state != WAIT_WAKEUP
        || wakeup_received
        || vad_running
    ) {
        return;
    }

    ROS_INFO(
        "检测到‘小飞小飞’，唤醒角度：%d，立即启动同流 VAD 录音",
        msg->data
    );

    if (pcm_sub.getNumPublishers() == 0) {
        ROS_ERROR(
            "无法启动 VAD：PCM 话题 %s 没有发布者",
            PCM_TOPIC
        );
        return;
    }

    if (!pcm_received) {
        ROS_WARN("PCM 话题已有发布者，但暂未收到数据");
    }

    if (!startVadProcess(true)) {
        ROS_ERROR("VAD 启动失败，请再次说‘小飞小飞’");
        return;
    }

    wakeup_received = true;
    ROS_INFO("请直接衔接说任务内容，不需要等待提示");
}

string runCommandAndCapture(const string& command) {
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        return "";
    }

    string output;
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    pclose(pipe);
    return output;
}

pid_t getRosNodePid(const string& node_name) {
    const string command =
        "rosnode info " + shellQuote(node_name)
        + " 2>/dev/null | awk '/^Pid:/{print $2; exit}'";
    const string output = runCommandAndCapture(command);

    if (output.empty()) {
        return -1;
    }

    try {
        const long pid_value = stol(output);
        return pid_value > 0
            ? static_cast<pid_t>(pid_value)
            : static_cast<pid_t>(-1);
    } catch (...) {
        return -1;
    }
}

bool processIsAlive(pid_t pid) {
    if (pid <= 0) {
        return false;
    }

    // kill(pid, 0) 对僵尸进程仍会返回成功，但僵尸进程已经关闭了
    // 包括 ALSA 句柄在内的全部文件描述符，应当视为已经退出。
    ifstream stat_file(
        "/proc/" + to_string(pid) + "/stat"
    );
    string stat_line;
    if (getline(stat_file, stat_line)) {
        const size_t command_end = stat_line.rfind(')');
        if (
            command_end != string::npos
            && command_end + 2 < stat_line.size()
            && stat_line[command_end + 2] == 'Z'
        ) {
            return false;
        }
    }

    if (kill(pid, 0) == 0) {
        return true;
    }
    return errno == EPERM;
}

bool waitForProcessExit(pid_t pid, double timeout_seconds) {
    const auto deadline =
        chrono::steady_clock::now()
        + chrono::milliseconds(
            static_cast<int>(timeout_seconds * 1000.0)
        );

    while (
        processIsAlive(pid)
        && chrono::steady_clock::now() < deadline
    ) {
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    return !processIsAlive(pid);
}
bool stopSpeechCommandNodeFast() {
    const pid_t speech_pid = getRosNodePid(SPEECH_NODE);

    if (speech_pid <= 0) {
        ROS_WARN(
            "录音已完成，但未能取得 %s 的 PID；节点可能已经退出",
            SPEECH_NODE
        );
        return true;
    }

    ROS_INFO(
        "任务录音已经写完，立即结束 %s（PID=%d）...",
        SPEECH_NODE,
        static_cast<int>(speech_pid)
    );

    if (kill(speech_pid, SIGTERM) != 0 && errno != ESRCH) {
        ROS_WARN(
            "向 %s 发送 SIGTERM 失败：%s",
            SPEECH_NODE,
            strerror(errno)
        );
        return false;
    }

    if (!waitForProcessExit(speech_pid, 0.8)) {
        ROS_WARN(
            "%s 在 0.8 秒内未退出，发送 SIGKILL",
            SPEECH_NODE
        );
        if (kill(speech_pid, SIGKILL) != 0 && errno != ESRCH) {
            ROS_ERROR(
                "强制结束 %s 失败：%s",
                SPEECH_NODE,
                strerror(errno)
            );
            return false;
        }
        waitForProcessExit(speech_pid, 0.3);
    }

    ROS_INFO("%s 已结束", SPEECH_NODE);
    return true;
}

bool audioFileLooksValid() {
    ifstream input(AUDIO_FILE, ios::binary | ios::ate);
    if (!input.is_open()) {
        return false;
    }

    return input.tellg() > static_cast<streampos>(44);
}

enum VadPollResult {
    VAD_NOT_RUNNING,
    VAD_STILL_RUNNING,
    VAD_RECORDING_SUCCEEDED,
    VAD_RECORDING_FAILED
};

VadPollResult pollVadProcess() {
    if (!vad_running || vad_pid <= 0) {
        return VAD_NOT_RUNNING;
    }

    int status = 0;
    const pid_t result = waitpid(vad_pid, &status, WNOHANG);
    if (result == 0) {
        return VAD_STILL_RUNNING;
    }
    if (result < 0) {
        if (errno == EINTR) {
            return VAD_STILL_RUNNING;
        }
        ROS_ERROR("waitpid 检查 VAD 进程失败：%s", strerror(errno));
        vad_running = false;
        vad_pid = -1;
        closeVadPipe();
        return VAD_RECORDING_FAILED;
    }

    vad_running = false;
    vad_pid = -1;
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
        return VAD_RECORDING_FAILED;
    }

    if (!audioFileLooksValid()) {
        ROS_ERROR(
            "VAD 已退出，但没有生成有效 WAV：%s",
            AUDIO_FILE
        );
        return VAD_RECORDING_FAILED;
    }

    ROS_INFO("VAD 任务录音完成：%s", AUDIO_FILE);
    return VAD_RECORDING_SUCCEEDED;
}

void stopVadProcess() {
    closeVadPipe();

    if (vad_pid > 0) {
        kill(vad_pid, SIGTERM);
        waitpid(vad_pid, nullptr, 0);
    }
    vad_pid = -1;
    vad_running = false;
}

void playRetryPrompt() {
    playAudio(ERROR_AUDIO, "重录提示音");
}

bool startRetryRecording() {
    playRetryPrompt();
    // 播放提示音期间主循环被阻塞，先处理并丢弃队列中残留的提示音 PCM，
    // 避免提示音本身被下一轮 VAD 当成用户任务语音。
    ros::spinOnce();
    pcm_prebuffer.clear();

    if (!startVadProcess(false)) {
        ROS_ERROR("重新启动 VAD 失败");
        return false;
    }

    ROS_INFO("请重新说出完整任务，VAD 将在说完后自动停止");
    return true;
}

// ================= 导航 Action =================
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

bool go_destination(
    double x,
    double y,
    double yaw,
    MoveBaseClient& ac
) {
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
    return ac.getState()
        == actionlib::SimpleClientGoalState::SUCCEEDED;
}

// ================= 扫码核心遍历参数 =================
int qr_waypoint_idx = 0;
ros::Time scan_start_time;
vector<string> scanned_urls;
int valid_qr_count = 0;

// ================= HTTP 解析 =================
size_t WriteCallback(
    void* contents,
    size_t size,
    size_t nmemb,
    string* userp
) {
    userp->append(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
}

string httpGet(const string& url) {
    CURL* curl = curl_easy_init();
    string read_buffer;

    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);

        const CURLcode result = curl_easy_perform(curl);
        if (result != CURLE_OK) {
            ROS_WARN(
                "二维码地址请求失败：%s",
                curl_easy_strerror(result)
            );
            read_buffer.clear();
        }
        curl_easy_cleanup(curl);
    }

    return read_buffer;
}

int extractCode(const string& json_str) {
    const size_t key_pos = json_str.find("\"code\"");
    if (key_pos == string::npos) {
        return -1;
    }

    const size_t colon_pos = json_str.find(":", key_pos);
    if (colon_pos == string::npos) {
        return -1;
    }

    try {
        return stoi(json_str.substr(colon_pos + 1));
    } catch (...) {
        return -1;
    }
}

string extractResult(const string& json_str) {
    const size_t key_pos = json_str.find("\"result\"");
    if (key_pos == string::npos) {
        return "";
    }

    const size_t start_quote = json_str.find("\"", key_pos + 8);
    if (start_quote == string::npos) {
        return "";
    }

    const size_t end_quote = json_str.find("\"", start_quote + 1);
    if (end_quote == string::npos) {
        return "";
    }

    return json_str.substr(
        start_quote + 1,
        end_quote - start_quote - 1
    );
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    signal(SIGPIPE, SIG_IGN);
    ros::init(argc, argv, "main_competition_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<string>(
        "vad_script",
        vad_script,
        VAD_SCRIPT
    );
    private_nh.param<string>(
        "vad_status_file",
        vad_status_file,
        VAD_STATUS_FILE
    );
    private_nh.param<string>(
        "vad_backend",
        vad_backend,
        "auto"
    );
    private_nh.param(
        "vad_min_seconds",
        vad_min_seconds,
        2.0
    );
    private_nh.param(
        "vad_silence_seconds",
        vad_silence_seconds,
        1.2
    );
    private_nh.param(
        "vad_max_seconds",
        vad_max_seconds,
        9.0
    );
    private_nh.param(
        "vad_tail_seconds",
        vad_tail_seconds,
        0.30
    );

    semantic_client =
        nh.serviceClient<ucarmain2026::GetTaskSemantics>(
            "/get_task_semantics"
        );
    qr_client =
        nh.serviceClient<qr_01::qr_code>("qr_detect");
    classifier_client =
        nh.serviceClient<ucarmain2026::ItemClassify>(
            "/get_item_classification"
        );

    ros::Subscriber awake_sub =
        nh.subscribe<std_msgs::Int32>(
            WAKEUP_TOPIC,
            5,
            awakeCallback
        );
    pcm_sub =
        nh.subscribe<std_msgs::UInt8MultiArray>(
            PCM_TOPIC,
            100,
            pcmCallback
        );

    // 关闭 speech_command_node 内部旧版固定 9 秒写盘，只保留同流
    // PCM 发布；正式录音统一交给本节点启动的 vad_record.py。
    nh.setParam(
        "/speech_command/internal_task_recording",
        false
    );

    MoveBaseClient ac("move_base", true);

    ROS_INFO("等待 Spark 语义服务 /get_task_semantics...");
    semantic_client.waitForExistence();

    // 防止 Spark 语义服务误读上一轮比赛遗留的录音。
    remove(AUDIO_FILE);
    remove(vad_status_file.c_str());

    ROS_INFO("智能车总控节点已启动！请说‘小飞小飞’唤醒...");
    ROS_INFO(
        "VAD 参数：min=%.1fs，silence=%.1fs，max=%.1fs，backend=%s",
        vad_min_seconds,
        vad_silence_seconds,
        vad_max_seconds,
        vad_backend.c_str()
    );

    ros::Rate rate(20);
    while (ros::ok() && current_state != ALL_FINISHED) {
        ros::spinOnce();

        switch (current_state) {
            case WAIT_WAKEUP:
                if (wakeup_received) {
                    awake_sub.shutdown();
                    // 唤醒回调已经在同一条 PCM 流上启动 Python VAD。
                    current_state = RECORDING;
                }
                break;

            case RECORDING:
            {
                const VadPollResult vad_result = pollVadProcess();
                if (vad_result == VAD_RECORDING_SUCCEEDED) {
                    current_state = SEMANTIC_PARSING;
                } else if (
                    vad_result == VAD_RECORDING_FAILED
                    || vad_result == VAD_NOT_RUNNING
                ) {
                    ROS_WARN(
                        "录音失败，播放提示音后直接重新录制..."
                    );
                    if (!startRetryRecording()) {
                        ROS_ERROR("无法继续任务录音，结束本次任务");
                        current_state = ALL_FINISHED;
                    }
                }
                break;
            }

            case SEMANTIC_PARSING:
            {
                ucarmain2026::GetTaskSemantics srv_task;
                if (
                    semantic_client.call(srv_task)
                    && srv_task.response.success
                ) {
                    target_real = srv_task.response.target_real;
                    target_sim = srv_task.response.target_sim;
                    ROS_INFO(
                        "语义解析成功！实体区=[%s]，仿真区=[%s]",
                        target_real.c_str(),
                        target_sim.c_str()
                    );
                    // 语义成功后不再需要麦克风，立即释放后台算力。
                    stopSpeechCommandNodeFast();
                    current_state = NAVIGATING;
                } else {
                    ROS_WARN(
                        "解析失败！播放提示音后直接重新录制..."
                    );
                    if (startRetryRecording()) {
                        current_state = RECORDING;
                    } else {
                        ROS_ERROR("无法重新启动 VAD，结束本次任务");
                        current_state = ALL_FINISHED;
                    }
                }
                break;
            }

            case NAVIGATING:
            {
                ac.waitForServer();
                const double qr_wp_x[4] =
                    {1.0, 0.75, 0.5, 0.75};
                const double qr_wp_y[4] =
                    {5.25, 5.0, 5.25, 5.5};
                const double qr_wp_yaw[4] =
                    {3.14, 1.57, 0.0, -1.57};

                if (qr_waypoint_idx >= 4) {
                    ROS_INFO("4 个扫码点遍历完毕！进行分类...");
                    qr_01::qr_code srv_qr;
                    srv_qr.request.command = -2;
                    qr_client.call(srv_qr);
                    current_state = ITEM_CLASSIFYING;
                    break;
                }

                ROS_INFO(
                    "前往第 %d 个定点扫码位置...",
                    qr_waypoint_idx + 1
                );
                if (
                    go_destination(
                        qr_wp_x[qr_waypoint_idx],
                        qr_wp_y[qr_waypoint_idx],
                        qr_wp_yaw[qr_waypoint_idx],
                        ac
                    )
                ) {
                    qr_01::qr_code srv_qr;
                    srv_qr.request.command = -1;
                    qr_client.call(srv_qr);
                    ros::Duration(1.0).sleep();
                    srv_qr.request.command = -3;
                    qr_client.call(srv_qr);
                    scan_start_time = ros::Time::now();
                    current_state = QR_SCANNING;
                } else {
                    qr_waypoint_idx++;
                }
                break;
            }

            case QR_SCANNING:
            {
                if (
                    (ros::Time::now() - scan_start_time).toSec()
                    > 2.0
                ) {
                    qr_01::qr_code srv_qr;
                    srv_qr.request.command = -2;
                    qr_client.call(srv_qr);
                    qr_waypoint_idx++;
                    current_state = NAVIGATING;
                    break;
                }

                qr_01::qr_code srv;
                srv.request.command = 1;
                if (
                    qr_client.call(srv)
                    && !srv.response.result.empty()
                ) {
                    const string raw_res = srv.response.result;
                    const size_t split_pos = raw_res.find("|");
                    const string captured_url =
                        (split_pos != string::npos)
                        ? raw_res.substr(split_pos + 1)
                        : raw_res;

                    if (
                        find(
                            scanned_urls.begin(),
                            scanned_urls.end(),
                            captured_url
                        ) == scanned_urls.end()
                    ) {
                        scanned_urls.push_back(captured_url);
                        const string json = httpGet(captured_url);
                        const int code = extractCode(json);

                        if (code == 200) {
                            const string res_text =
                                extractResult(json);
                            if (valid_qr_count == 0) {
                                target_qr_1 = res_text;
                            } else if (valid_qr_count == 1) {
                                target_qr_2 = res_text;
                            } else if (valid_qr_count == 2) {
                                target_qr_3 = res_text;
                            }
                            valid_qr_count++;

                            if (valid_qr_count >= 3) {
                                ROS_INFO(
                                    "成功收集 3 个有效二维码！"
                                );
                                qr_01::qr_code srv_qr;
                                srv_qr.request.command = -2;
                                qr_client.call(srv_qr);
                                current_state = ITEM_CLASSIFYING;
                            } else {
                                qr_01::qr_code srv_qr;
                                srv_qr.request.command = -2;
                                qr_client.call(srv_qr);
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

                if (
                    classifier_client.call(classify_srv)
                    && classify_srv.response.success
                ) {
                    final_real_item =
                        classify_srv.response.real_item;
                    final_sim_item =
                        classify_srv.response.sim_item;
                    ROS_INFO(
                        "分类成功！实体区应放：%s",
                        final_real_item.c_str()
                    );
                    current_state = TTS_BROADCASTING;
                } else {
                    ros::Duration(2.0).sleep();
                }
                break;
            }

            case TTS_BROADCASTING:
            {
                if (!prepareItemAudios(final_real_item, final_sim_item)) {
                    ROS_ERROR(
                        "本次不播放旧音频，结束语音播报阶段"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                CategoryAudioTexts real_audio_texts;
                CategoryAudioTexts sim_audio_texts;
                if (
                    !getCategoryAudioTexts(
                        target_real,
                        real_audio_texts
                    )
                    || !getCategoryAudioTexts(
                        target_sim,
                        sim_audio_texts
                    )
                ) {
                    ROS_ERROR(
                        "类别固定音频映射失败，结束语音播报阶段"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                // 两个动态物品名音频可以在 audios 中提前缓存；类别与放置
                // 内容使用长句 WAV。保留“取得”“仿真环境中”“已将”三个
                // 必要衔接片段，确保播报文字和原来的 2/3/4.wav 完全一致。
                const vector<string> broadcast_audio_texts = {
                    "取得",
                    final_real_item,
                    real_audio_texts.classification,
                    "仿真环境中",
                    "取得",
                    final_sim_item,
                    sim_audio_texts.classification,
                    "已将",
                    final_real_item,
                    real_audio_texts.workshop_placement,
                    "仿真任务已完成，已将",
                    final_sim_item,
                    sim_audio_texts.warehouse_placement
                };

                if (!playAudioTextSequence(
                        broadcast_audio_texts,
                        "整句版任务语音播报"
                    )) {
                    ROS_ERROR("任务语音片段不完整或播放失败");
                }

                current_state = ALL_FINISHED;
                break;
            }

            case ALL_FINISHED:
                break;
        }

        rate.sleep();
    }

    stopVadProcess();
    nh.setParam(
        "/speech_command/internal_task_recording",
        true
    );
    ROS_INFO("二维码识别、分类及整句片段离线语音播报全部完成");
    return 0;
}