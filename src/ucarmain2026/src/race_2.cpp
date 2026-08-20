/**
 * @file race.cpp
 * @brief 2026 智能车国赛完整流程阶段版
 *        （语音唤醒/语义 + 坡道固定Reference + 二维码 + 分类
 *         + 离线TTS + V14.9找板停靠 + 仿真通信；暂不含红绿灯）
 * V17：现实目标停靠后先纯旋转并计算法向距离，随后
 *      允许快速横移与航向修正同时进行，实时消除转向过冲和横移漂移。
 */

// V17_TWO_STAGE_DOCK_CLEAN_20260819：停靠严格为两段式move_base；已彻底删除旧横移居中+雷达逼近三段式遗留代码。

// V18_QR_HTTP_RELIABLE_20260819：二维码HTTP连接/总超时拆分，扫码会话复用CURL，输出DNS/TCP/TLS/TTFB/Total诊断。
// V20_STABILITY_FALLBACK_20260819：
// 1) QR最终失败 -> 牛奶/水杯/显示器固定三类别候选；
// 2) 找板最终允许sim-only / real-only / neither继续仿真任务；
// 3) 仿真任务总等待120s超时自动按完成处理。

// V22_CORNER_COSTMAP_ADAPTIVE_TRANSITION_20260819：
// 前三条固定巡线段终点完全不变；每段完成后查询local costmap指定5cm区域。
// 有路障死区保持原V13安全角点；无路障时向下一条巡线方向提前10cm并同步转向。
// costmap不可用、过旧或不覆盖检测区时按“有路障”保守处理。

// V23_NEAR_END_CORNER_SHORTCUT_20260819：
// 当前墙现实目标完成停靠后，若本次识别首次生成的停靠导航点距本段终点<1m，
// 则跳过V17回线和剩余固定Path，直接执行本段现有角点旋转平移并结束本段。
// 第四段没有后续角点，因此不启用；其他逻辑保持V22不变。

// V24_NONCURRENT_TARGET_EARLY_DOCK_20260819：
// 非当前左墙现实目标若距当前巡检段终点<1m，则先记录并继续巡线；
// 当机器人自身距当前段终点也<1m时，提前取消剩余Path并去停靠。
// 停靠后不返回旧段终点/角点，而是从当前位置V17并入目标板所在墙继续巡检。
// 仿真目标在现实目标未停靠前的既有优先级保持不变。

// V25_UDISK_TTS_FIX_20260820：
// 主程序TTS名称合法性检查增加“U盘”精确例外。
// Python分类服务统一返回“U盘”后，可正常通过分类阶段和音频准备阶段；
// 其他含英文字母的名称仍保持拒绝，找板/巡检/仿真/红绿灯逻辑均不变。

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros_nanodet/detect_result_srv.h>
#include <ros_nanodet/ocr_result_srv.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ucarmain2026/set_speed.h>
#include <line_follow/line_follow.h>

// 自定义服务
#include <ucarmain2026/GetTaskSemantics.h>
#include <ucarmain2026/ItemClassify.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <clocale>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <limits>
#include <sstream>
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
    SLOPE_STAGE,
    NAVIGATING,
    QR_SCANNING,
    ITEM_CLASSIFYING,
    TTS_BROADCASTING,
    TRAFFIC_LIGHT_CONTROL,
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

ros::Publisher simulation_target_pub;
ros::Subscriber simulation_result_sub;
bool simulation_result_received = false;
string simulation_result_text = "";

// V20：从进入仿真任务开始计时。
// 超过该时间仍没有 /car_task_finished 非空结果时，
// 自动按“仿真任务已完成”继续后续流程，避免无限等待。
double simulation_result_timeout = 120.0;

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
const char* const TASK_COMPLETE_AUDIO =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/任务完成.wav";

// 最终巡线成功返回后，“任务完成.wav”固定以 1.5 倍速播放。
// 该参数与可动态配置的 tts_playback_speed 相互独立，避免比赛现场
// 调整普通播报速度时意外改变最终完成提示音的速度。
constexpr double TASK_COMPLETE_PLAYBACK_SPEED = 1.50;

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
bool discard_pcm = false;
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
double retry_pcm_guard_seconds = 0.15;

// ================= 最终任务播报：连续 PCM 播放参数 =================
// 保留“片段缓存 + 仅补生成随机物品名”的 TTS 架构。
// 最终播放时将所有 WAV 的 PCM 数据读入内存，裁掉头尾多余静音，
// 再通过一个 aplay 进程的一条原始 PCM 流连续播放。
bool tts_trim_silence = true;
int tts_trim_silence_threshold = 200;
double tts_trim_keep_ms = 20.0;
double tts_crossfade_ms = 8.0;

// 最终播报倍速。
// 直接提高 aplay raw PCM 的播放采样率，不增加 TTS 生成耗时。
// 1.50 = 1.5 倍速；音调也会同步升高。
double tts_playback_speed = 1.50;

/**
 * @brief 把任意 UTF-8 文本安全地作为一个 shell 参数传给 system()。
 *
 * 二维码或分类结果中即使出现空格、引号、分号等字符，也不会破坏命令。
 */

/**
 * @brief 判断分类服务返回的物品名是否适合作为中文 TTS 输入。
 *
 * 当前比赛规则：
 * - 普通物品播报名必须包含非 ASCII UTF-8 字节（中文）；
 * - 普通物品不允许残留 A-Z / a-z 英文字母；
 * - 唯一允许的中英混合固定名称是“U盘”；
 * - Python 分类服务会把 U盘/u盘/USB盘/usb盘/优盘统一规范成“U盘”；
 * - 具体候选来源合法性仍由 Python 分类服务负责校验。
 *
 * 这是一道主程序侧最后保险：
 * 只为比赛中的固定物品“U盘”开放精确例外，
 * 其他英文或中英混合名称仍然禁止进入 generate_task_audios.py。
 */
bool isChineseTtsItemText(const string& text) {
    if (text.empty()) {
        return false;
    }

    // V25：U盘是比赛物品中的唯一英文字符例外。
    //
    // 必须精确等于“U盘”才放行：
    //   U盘     -> 允许
    //   USB盘   -> 拒绝
    //   u盘     -> 拒绝
    //   TV电视  -> 拒绝
    //   Laptop  -> 拒绝
    //
    // 新版 spark_classifier_server 会在ROS服务返回前统一规范成“U盘”，
    // 因此主程序只需要接受这一种稳定写法。
    if (text == "U盘") {
        return true;
    }

    bool has_non_ascii = false;

    for (unsigned char ch : text) {
        if (ch >= 0x80) {
            has_non_ascii = true;
            continue;
        }

        if (
            (ch >= 'a' && ch <= 'z')
            || (ch >= 'A' && ch <= 'Z')
        ) {
            return false;
        }
    }

    return has_non_ascii;
}

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
    if (
        !isChineseTtsItemText(real_item)
        || !isChineseTtsItemText(sim_item)
    ) {
        ROS_ERROR(
            "拒绝生成非法TTS物品音频：real=[%s] sim=[%s]；"
            "普通物品必须为中文，唯一英文字符例外为精确名称“U盘”",
            real_item.c_str(),
            sim_item.c_str()
        );
        return false;
    }

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


void simulationResultCallback(
    const std_msgs::String::ConstPtr& message
) {
    if (!message || message->data.empty()) {
        ROS_WARN("/car_task_finished 收到空消息，忽略并继续等待");
        return;
    }

    simulation_result_text = message->data;
    simulation_result_received = true;

    ROS_WARN(
        "收到仿真任务返回：[%s]；按当前规则不区分成功或失败，均进入最终播报",
        simulation_result_text.c_str()
    );
}

bool simulationBridgeCategoryText(
    const string& category_code,
    string& category_text
) {
    // 注意：这里是“通信类别”，不能翻译成中文。
    // 仿真端 /target_class 只接受：
    //   food / daily / electronic
    if (
        category_code == "food"
        || category_code == "daily"
        || category_code == "electronic"
    ) {
        category_text = category_code;
        return true;
    }

    ROS_ERROR(
        "仿真目标类别 [%s] 非法，合法值只能是 "
        "food / daily / electronic",
        category_code.c_str()
    );
    return false;
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

struct WavPcm16 {
    uint16_t audio_format = 0;
    uint16_t channels = 0;
    uint32_t sample_rate = 0;
    uint16_t bits_per_sample = 0;
    uint16_t block_align = 0;
    vector<int16_t> samples;
};

uint16_t readLe16(const unsigned char* data) {
    return static_cast<uint16_t>(data[0])
        | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readLe32(const unsigned char* data) {
    return static_cast<uint32_t>(data[0])
        | (static_cast<uint32_t>(data[1]) << 8)
        | (static_cast<uint32_t>(data[2]) << 16)
        | (static_cast<uint32_t>(data[3]) << 24);
}

bool loadWavPcm16(
    const string& audio_file,
    WavPcm16& wav,
    string& error_message
) {
    ifstream input(audio_file, ios::binary);
    if (!input.is_open()) {
        error_message = "无法打开文件";
        return false;
    }

    char riff_header[12] = {};
    input.read(riff_header, sizeof(riff_header));
    if (
        input.gcount() != static_cast<streamsize>(sizeof(riff_header))
        || memcmp(riff_header, "RIFF", 4) != 0
        || memcmp(riff_header + 8, "WAVE", 4) != 0
    ) {
        error_message = "不是有效的 RIFF/WAVE 文件";
        return false;
    }

    bool got_fmt = false;
    bool got_data = false;
    vector<unsigned char> pcm_bytes;

    while (input && (!got_fmt || !got_data)) {
        char chunk_id[4] = {};
        unsigned char chunk_size_bytes[4] = {};

        input.read(chunk_id, 4);
        if (input.gcount() != 4) {
            break;
        }

        input.read(
            reinterpret_cast<char*>(chunk_size_bytes),
            4
        );
        if (input.gcount() != 4) {
            break;
        }

        const uint32_t chunk_size =
            readLe32(chunk_size_bytes);

        if (memcmp(chunk_id, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                error_message = "fmt 块长度异常";
                return false;
            }

            vector<unsigned char> fmt_data(chunk_size);
            input.read(
                reinterpret_cast<char*>(fmt_data.data()),
                static_cast<streamsize>(chunk_size)
            );
            if (
                input.gcount()
                != static_cast<streamsize>(chunk_size)
            ) {
                error_message = "fmt 块读取不完整";
                return false;
            }

            wav.audio_format = readLe16(fmt_data.data());
            wav.channels = readLe16(fmt_data.data() + 2);
            wav.sample_rate = readLe32(fmt_data.data() + 4);
            wav.block_align = readLe16(fmt_data.data() + 12);
            wav.bits_per_sample = readLe16(fmt_data.data() + 14);
            got_fmt = true;
        } else if (memcmp(chunk_id, "data", 4) == 0) {
            pcm_bytes.resize(chunk_size);
            if (chunk_size > 0) {
                input.read(
                    reinterpret_cast<char*>(pcm_bytes.data()),
                    static_cast<streamsize>(chunk_size)
                );
                if (
                    input.gcount()
                    != static_cast<streamsize>(chunk_size)
                ) {
                    error_message = "data 块读取不完整";
                    return false;
                }
            }
            got_data = true;
        } else {
            input.seekg(
                static_cast<streamoff>(chunk_size),
                ios::cur
            );
            if (!input) {
                error_message = "跳过未知 WAV 块失败";
                return false;
            }
        }

        if (chunk_size & 1U) {
            input.seekg(1, ios::cur);
        }
    }

    if (!got_fmt || !got_data) {
        error_message = "缺少 fmt 或 data 块";
        return false;
    }

    if (
        wav.audio_format != 1
        || wav.bits_per_sample != 16
        || wav.channels == 0
        || wav.sample_rate == 0
    ) {
        ostringstream stream;
        stream
            << "仅支持 PCM16，实际 format="
            << wav.audio_format
            << " channels=" << wav.channels
            << " rate=" << wav.sample_rate
            << " bits=" << wav.bits_per_sample;
        error_message = stream.str();
        return false;
    }

    const uint16_t expected_block_align =
        static_cast<uint16_t>(wav.channels * 2);
    if (wav.block_align != expected_block_align) {
        error_message = "PCM16 block_align 与声道数不匹配";
        return false;
    }

    if (
        pcm_bytes.empty()
        || pcm_bytes.size() % 2 != 0
        || pcm_bytes.size() % wav.block_align != 0
    ) {
        error_message = "PCM data 长度异常";
        return false;
    }

    wav.samples.resize(pcm_bytes.size() / 2);

    for (size_t i = 0; i < wav.samples.size(); ++i) {
        const size_t byte_index = i * 2;
        const uint16_t raw =
            static_cast<uint16_t>(pcm_bytes[byte_index])
            | (
                static_cast<uint16_t>(
                    pcm_bytes[byte_index + 1]
                ) << 8
            );
        wav.samples[i] = static_cast<int16_t>(raw);
    }

    return true;
}

int pcm16Abs(int16_t sample) {
    const int value = static_cast<int>(sample);
    return value >= 0 ? value : -value;
}

vector<int16_t> trimPcm16Silence(
    const WavPcm16& wav,
    size_t& trimmed_leading_frames,
    size_t& trimmed_trailing_frames
) {
    trimmed_leading_frames = 0;
    trimmed_trailing_frames = 0;

    if (
        !tts_trim_silence
        || wav.samples.empty()
        || wav.channels == 0
    ) {
        return wav.samples;
    }

    const size_t channels =
        static_cast<size_t>(wav.channels);
    const size_t total_frames =
        wav.samples.size() / channels;

    if (total_frames == 0) {
        return wav.samples;
    }

    const int threshold =
        max(0, min(32767, tts_trim_silence_threshold));

    auto frame_is_active =
        [&](size_t frame_index) -> bool {
            const size_t base = frame_index * channels;
            for (size_t channel = 0; channel < channels; ++channel) {
                if (
                    pcm16Abs(wav.samples[base + channel])
                    > threshold
                ) {
                    return true;
                }
            }
            return false;
        };

    size_t first_active = total_frames;
    for (size_t frame = 0; frame < total_frames; ++frame) {
        if (frame_is_active(frame)) {
            first_active = frame;
            break;
        }
    }

    if (first_active == total_frames) {
        return wav.samples;
    }

    size_t last_active = first_active;
    for (size_t frame = total_frames; frame > first_active; --frame) {
        const size_t index = frame - 1;
        if (frame_is_active(index)) {
            last_active = index;
            break;
        }
    }

    const size_t keep_frames =
        static_cast<size_t>(
            max(0.0, tts_trim_keep_ms)
            * static_cast<double>(wav.sample_rate)
            / 1000.0
        );

    const size_t start_frame =
        first_active > keep_frames
        ? first_active - keep_frames
        : 0;

    const size_t end_frame_exclusive =
        min(
            total_frames,
            last_active + 1 + keep_frames
        );

    trimmed_leading_frames = start_frame;
    trimmed_trailing_frames =
        total_frames - end_frame_exclusive;

    return vector<int16_t>(
        wav.samples.begin()
            + static_cast<ptrdiff_t>(
                start_frame * channels
            ),
        wav.samples.begin()
            + static_cast<ptrdiff_t>(
                end_frame_exclusive * channels
            )
    );
}

void appendPcm16WithCrossfade(
    vector<int16_t>& output,
    const vector<int16_t>& input,
    uint16_t channels,
    uint32_t sample_rate
) {
    if (input.empty()) {
        return;
    }

    if (output.empty()) {
        output = input;
        return;
    }

    const size_t channel_count =
        static_cast<size_t>(channels);

    if (channel_count == 0) {
        output.insert(output.end(), input.begin(), input.end());
        return;
    }

    const size_t output_frames =
        output.size() / channel_count;
    const size_t input_frames =
        input.size() / channel_count;

    size_t crossfade_frames =
        static_cast<size_t>(
            max(0.0, tts_crossfade_ms)
            * static_cast<double>(sample_rate)
            / 1000.0
        );

    crossfade_frames = min(
        crossfade_frames,
        min(output_frames, input_frames)
    );

    if (crossfade_frames == 0) {
        output.insert(output.end(), input.begin(), input.end());
        return;
    }

    const size_t output_overlap_start =
        output.size()
        - crossfade_frames * channel_count;

    for (
        size_t frame = 0;
        frame < crossfade_frames;
        ++frame
    ) {
        const double alpha =
            static_cast<double>(frame + 1)
            / static_cast<double>(crossfade_frames + 1);

        for (
            size_t channel = 0;
            channel < channel_count;
            ++channel
        ) {
            const size_t output_index =
                output_overlap_start
                + frame * channel_count
                + channel;

            const size_t input_index =
                frame * channel_count
                + channel;

            const double mixed =
                static_cast<double>(output[output_index])
                    * (1.0 - alpha)
                + static_cast<double>(input[input_index])
                    * alpha;

            int mixed_int = static_cast<int>(mixed);
            mixed_int = max(-32768, min(32767, mixed_int));

            output[output_index] =
                static_cast<int16_t>(mixed_int);
        }
    }

    output.insert(
        output.end(),
        input.begin()
            + static_cast<ptrdiff_t>(
                crossfade_frames * channel_count
            ),
        input.end()
    );
}

bool playContinuousPcm16WithAplay(
    const vector<int16_t>& samples,
    uint16_t channels,
    uint32_t sample_rate,
    const string& description,
    double playback_speed
) {
    if (
        samples.empty()
        || channels == 0
        || sample_rate == 0
    ) {
        ROS_ERROR("%s没有可播放的 PCM 数据", description.c_str());
        return false;
    }

    if (playback_speed <= 0.0) {
        ROS_ERROR(
            "%s播放倍速非法：%.3f",
            description.c_str(),
            playback_speed
        );
        return false;
    }

    const uint32_t playback_sample_rate =
        static_cast<uint32_t>(
            static_cast<double>(sample_rate)
            * playback_speed
        );

    ROS_INFO(
        "%s播放倍速：%.2fx，原采样率=%u Hz，实际播放采样率=%u Hz",
        description.c_str(),
        playback_speed,
        static_cast<unsigned int>(sample_rate),
        static_cast<unsigned int>(playback_sample_rate)
    );

    const string command =
        "aplay -q -t raw -f S16_LE -r "
        + to_string(playback_sample_rate)
        + " -c "
        + to_string(channels);

    FILE* pipe = popen(command.c_str(), "w");
    if (pipe == nullptr) {
        ROS_ERROR(
            "无法启动连续 aplay：%s",
            strerror(errno)
        );
        return false;
    }

    const uint8_t* bytes =
        reinterpret_cast<const uint8_t*>(samples.data());
    const size_t total_bytes =
        samples.size() * sizeof(int16_t);

    size_t offset = 0;
    const int output_fd = fileno(pipe);

    while (offset < total_bytes) {
        const ssize_t written = write(
            output_fd,
            bytes + offset,
            total_bytes - offset
        );

        if (written > 0) {
            offset += static_cast<size_t>(written);
            continue;
        }

        if (written < 0 && errno == EINTR) {
            continue;
        }

        ROS_ERROR(
            "%s连续 PCM 写入 aplay 失败：%s",
            description.c_str(),
            written < 0 ? strerror(errno) : "write 返回 0"
        );
        pclose(pipe);
        return false;
    }

    const int status = pclose(pipe);
    if (status == -1) {
        ROS_ERROR(
            "%s等待 aplay 结束失败：%s",
            description.c_str(),
            strerror(errno)
        );
        return false;
    }

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        if (WIFEXITED(status)) {
            ROS_WARN(
                "%s播放失败，aplay 退出码：%d",
                description.c_str(),
                WEXITSTATUS(status)
            );
        } else if (WIFSIGNALED(status)) {
            ROS_WARN(
                "%s播放被信号 %d 终止",
                description.c_str(),
                WTERMSIG(status)
            );
        } else {
            ROS_WARN("%s播放未正常结束", description.c_str());
        }
        return false;
    }

    return true;
}

/**
 * @brief 以指定倍速播放单个 16-bit PCM WAV 文件。
 *
 * 复用连续 TTS 已验证的 WAV 解析和 aplay 原始 PCM 播放链路，
 * 通过提高播放采样率实现倍速；不会修改磁盘上的原音频文件。
 */
bool playWavAtSpeed(
    const string& audio_file,
    const string& description,
    double playback_speed
) {
    WavPcm16 wav;
    string wav_error;

    if (!loadWavPcm16(audio_file, wav, wav_error)) {
        ROS_ERROR(
            "%s无法加载 WAV：文件=%s，原因=%s",
            description.c_str(),
            audio_file.c_str(),
            wav_error.c_str()
        );
        return false;
    }

    ROS_INFO(
        "播放%s：%s，固定倍速=%.2fx",
        description.c_str(),
        audio_file.c_str(),
        playback_speed
    );

    return playContinuousPcm16WithAplay(
        wav.samples,
        wav.channels,
        wav.sample_rate,
        description,
        playback_speed
    );
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

    vector<int16_t> joined_pcm;
    uint16_t expected_channels = 0;
    uint32_t expected_sample_rate = 0;

    size_t total_original_frames = 0;
    size_t total_trimmed_leading_frames = 0;
    size_t total_trimmed_trailing_frames = 0;

    for (size_t index = 0; index < audio_files.size(); ++index) {
        WavPcm16 wav;
        string wav_error;

        if (!loadWavPcm16(
                audio_files[index],
                wav,
                wav_error
            )) {
            ROS_ERROR(
                "%s无法加载第 %zu/%zu 段 WAV：文字=[%s]，"
                "文件=%s，原因=%s",
                description.c_str(),
                index + 1,
                audio_files.size(),
                audio_texts[index].c_str(),
                audio_files[index].c_str(),
                wav_error.c_str()
            );
            return false;
        }

        if (index == 0) {
            expected_channels = wav.channels;
            expected_sample_rate = wav.sample_rate;
        } else if (
            wav.channels != expected_channels
            || wav.sample_rate != expected_sample_rate
        ) {
            ROS_ERROR(
                "%s音频片段格式不一致：第 %zu 段"
                " channels=%u rate=%u，期望 channels=%u rate=%u",
                description.c_str(),
                index + 1,
                static_cast<unsigned int>(wav.channels),
                static_cast<unsigned int>(wav.sample_rate),
                static_cast<unsigned int>(expected_channels),
                static_cast<unsigned int>(expected_sample_rate)
            );
            return false;
        }

        const size_t original_frames =
            wav.samples.size()
            / static_cast<size_t>(wav.channels);

        total_original_frames += original_frames;

        size_t trimmed_leading_frames = 0;
        size_t trimmed_trailing_frames = 0;

        vector<int16_t> trimmed_pcm =
            trimPcm16Silence(
                wav,
                trimmed_leading_frames,
                trimmed_trailing_frames
            );

        total_trimmed_leading_frames +=
            trimmed_leading_frames;
        total_trimmed_trailing_frames +=
            trimmed_trailing_frames;

        appendPcm16WithCrossfade(
            joined_pcm,
            trimmed_pcm,
            expected_channels,
            expected_sample_rate
        );
    }

    if (joined_pcm.empty()) {
        ROS_ERROR("%s拼接后 PCM 为空", description.c_str());
        return false;
    }

    const size_t joined_frames =
        joined_pcm.size()
        / static_cast<size_t>(expected_channels);

    const double original_seconds =
        static_cast<double>(total_original_frames)
        / static_cast<double>(expected_sample_rate);

    const double joined_seconds =
        static_cast<double>(joined_frames)
        / static_cast<double>(expected_sample_rate);

    const double trimmed_seconds =
        static_cast<double>(
            total_trimmed_leading_frames
            + total_trimmed_trailing_frames
        )
        / static_cast<double>(expected_sample_rate);

    ROS_INFO(
        "开始%s：%zu 个缓存 WAV → 单一连续 PCM 流，"
        "%u Hz，%u 声道；裁剪头尾静音约 %.3f 秒，"
        "交叉淡化 %.1f ms；原始总时长约 %.3f 秒，"
        "连续播报约 %.3f 秒",
        description.c_str(),
        audio_files.size(),
        static_cast<unsigned int>(expected_sample_rate),
        static_cast<unsigned int>(expected_channels),
        trimmed_seconds,
        max(0.0, tts_crossfade_ms),
        original_seconds,
        joined_seconds
    );

    return playContinuousPcm16WithAplay(
        joined_pcm,
        expected_channels,
        expected_sample_rate,
        description,
        tts_playback_speed
    );
}


bool playInitialTaskBroadcast(
    const string& real_item,
    const string& sim_item,
    const CategoryAudioTexts& real_audio_texts,
    const CategoryAudioTexts& sim_audio_texts
) {
    const vector<string> audio_texts = {
        "取得",
        real_item,
        real_audio_texts.classification,
        "仿真环境中",
        "取得",
        sim_item,
        sim_audio_texts.classification
    };

    ROS_INFO(
        "第一阶段播报：取得%s，%s；仿真环境中取得%s，%s；",
        real_item.c_str(),
        real_audio_texts.classification.c_str(),
        sim_item.c_str(),
        sim_audio_texts.classification.c_str()
    );

    return playAudioTextSequence(
        audio_texts,
        "二维码分类结果播报"
    );
}

bool playRealDockedBroadcast(
    const string& real_item,
    const CategoryAudioTexts& real_audio_texts
) {
    const vector<string> audio_texts = {
        "已将",
        real_item,
        real_audio_texts.workshop_placement
    };

    ROS_INFO(
        "现实停靠播报：已将%s%s",
        real_item.c_str(),
        real_audio_texts.workshop_placement.c_str()
    );

    return playAudioTextSequence(
        audio_texts,
        "现实目标停靠播报"
    );
}

bool playSimulationFinishedBroadcast(
    const string& sim_item,
    const CategoryAudioTexts& sim_audio_texts
) {
    const vector<string> audio_texts = {
        "仿真任务已完成，已将",
        sim_item,
        sim_audio_texts.warehouse_placement
    };

    ROS_INFO(
        "最终播报：仿真任务已完成，已将%s%s",
        sim_item.c_str(),
        sim_audio_texts.warehouse_placement.c_str()
    );

    return playAudioTextSequence(
        audio_texts,
        "仿真任务完成播报"
    );
}

bool waitForSimulationBridgeSubscriberUntil(
    const ros::WallTime& deadline
) {
    ros::WallRate rate(10.0);

    while (
        ros::ok()
        && simulation_target_pub.getNumSubscribers() == 0
        && ros::WallTime::now() < deadline
    ) {
        const double remaining =
            max(
                0.0,
                (deadline - ros::WallTime::now()).toSec()
            );

        ROS_INFO_THROTTLE(
            2.0,
            "等待 car_comm_bridge 订阅 /detected_target ... "
            "仿真总超时剩余 %.1fs",
            remaining
        );

        ros::spinOnce();
        rate.sleep();
    }

    return (
        ros::ok()
        && simulation_target_pub.getNumSubscribers() > 0
    );
}

bool sendSimulationTargetAndWait(
    const string& simulation_category_code
) {
    string category_text;
    if (!simulationBridgeCategoryText(
            simulation_category_code,
            category_text
        )) {
        return false;
    }

    const double timeout_seconds =
        max(
            1.0,
            simulation_result_timeout
        );

    // “进入仿真任务”从这里开始计时。
    // 等subscriber和等待/car_task_finished共用同一个截止时间。
    const ros::WallTime simulation_start =
        ros::WallTime::now();

    const ros::WallTime deadline =
        simulation_start
        + ros::WallDuration(
            timeout_seconds
        );

    simulation_result_received = false;
    simulation_result_text.clear();

    ROS_WARN(
        "进入仿真任务：总等待上限=%.1fs；"
        "超过上限仍无结果时自动按任务完成继续后续流程",
        timeout_seconds
    );

    if (!waitForSimulationBridgeSubscriberUntil(
            deadline
        )) {
        if (!ros::ok()) {
            return false;
        }

        simulation_result_text =
            "SIMULATION_TIMEOUT_ASSUMED_COMPLETE";

        ROS_ERROR(
            "仿真任务进入后 %.1fs 内始终没有可用 "
            "car_comm_bridge 订阅者；"
            "按稳定性兜底规则直接视为仿真任务完成。",
            timeout_seconds
        );

        return true;
    }

    std_msgs::String message;
    message.data = category_text;
    simulation_target_pub.publish(message);

    ROS_WARN(
        "已发布仿真目标类别码 [%s] 到 /detected_target；"
        "仿真端合法值仅为 food / daily / electronic；"
        "由 car_comm_bridge 负责 TCP ACK、重发和断线重连",
        category_text.c_str()
    );

    ROS_WARN(
        "小车保持停止等待 /car_task_finished；"
        "收到任意非空返回正常继续，"
        "仿真任务总计时达到 %.1fs 后也会自动继续",
        timeout_seconds
    );

    ros::WallRate rate(20.0);

    while (
        ros::ok()
        && !simulation_result_received
        && ros::WallTime::now() < deadline
    ) {
        ros::spinOnce();

        const double elapsed =
            (ros::WallTime::now() - simulation_start).toSec();

        const double remaining =
            max(
                0.0,
                timeout_seconds - elapsed
            );

        ROS_INFO_THROTTLE(
            3.0,
            "仍在等待仿真任务返回... 已等待 %.1fs，剩余 %.1fs",
            elapsed,
            remaining
        );

        rate.sleep();
    }

    if (!ros::ok()) {
        return false;
    }

    if (!simulation_result_received) {
        simulation_result_text =
            "SIMULATION_TIMEOUT_ASSUMED_COMPLETE";

        ROS_ERROR(
            "进入仿真任务后已达到 %.1fs，总流程仍未收到 "
            "/car_task_finished；"
            "按稳定性兜底规则自动视为仿真任务完成并继续后续流程。",
            timeout_seconds
        );

        return true;
    }

    ROS_WARN(
        "仿真任务返回内容：[%s]",
        simulation_result_text.c_str()
    );

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

        if (written < 0 && errno == EPIPE) {
            // VAD 已完成录音并关闭了管道读取端，这是正常收尾竞态。
            // 主循环随后仍会通过 waitpid() 回收子进程并确认录音结果。
            close(vad_stdin_fd);
            vad_stdin_fd = -1;
            return false;
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

    // 重录提示音播放及尾音保护期间，PCM 直接丢弃，
    // 不允许进入 VAD 或 300 ms 预缓存。
    if (discard_pcm) {
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
    // 从提示音开始播放就关闭 PCM 接收。aplay 是阻塞调用，播放期间
    // speech_command_node 仍会持续发布 PCM，因此这些消息会积压在 ROS 队列中。
    discard_pcm = true;
    pcm_prebuffer.clear();

    playRetryPrompt();

    const double guard_seconds =
        retry_pcm_guard_seconds > 0.0
        ? retry_pcm_guard_seconds
        : 0.0;
    const ros::WallTime guard_deadline =
        ros::WallTime::now() + ros::WallDuration(guard_seconds);

    // 持续处理并丢弃播放期间积压的 PCM，同时覆盖扬声器/声学链路尾音。
    do {
        ros::spinOnce();
        if (ros::WallTime::now() >= guard_deadline) {
            break;
        }
        ros::WallDuration(0.01).sleep();
    } while (ros::ok());

    pcm_prebuffer.clear();
    discard_pcm = false;

    // 重录仍按原设计：无需再次说唤醒词，提示音结束后直接启动新一轮 VAD。
    if (!startVadProcess(false)) {
        ROS_ERROR("重新启动 VAD 失败");
        return false;
    }

    ROS_INFO(
        "重录提示音已隔离，尾音保护 %.2f s；请直接重新说出完整任务",
        guard_seconds
    );
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
int qr_scan_round = 0;
int valid_qr_count = 0;
ros::Time scan_start_time;
ros::Time qr_post_success_deadline;
bool qr_found_item_at_current_waypoint = false;

// 单个墙面观察点的视角状态。
// NORMAL：原设定朝向；LEFT：相对原朝向左转；RIGHT：相对原朝向右转。
enum QrViewMode {
    QR_VIEW_NORMAL,
    QR_VIEW_LEFT,
    QR_VIEW_RIGHT
};
QrViewMode qr_view_mode = QR_VIEW_NORMAL;

// 当前这个“视角”在扫描窗口内是否至少成功解出过一个二维码字符串。
// 这里只判断视觉层是否解码成功，与 URL 是否重复、HTTP 是否成功无关。
bool qr_decoded_any_at_current_view = false;

vector<double> qr_wp_x = {1.0, 0.75, 0.5, 0.75};
vector<double> qr_wp_y = {5.25, 5.0, 5.25, 5.5};
vector<double> qr_wp_yaw = {3.14, 1.57, 0.0, -1.57};

double qr_scan_timeout = 1.2;
double qr_retry_scan_timeout = 2.0;
double qr_camera_warmup = 0.7;
// 当前墙面成功获得二维码后，只再短暂观察几帧，兼顾“一墙一码”快速离开
// 与同一墙面意外出现多个二维码时的连续读取能力。
double qr_post_success_scan_seconds = 0.15;

// 正常朝向整个扫描窗口都没有解出二维码时，依次尝试：
// 原朝向左转 30° -> 原朝向右转 30°。
// 角度做成 ROS 参数，比赛默认固定 30°。
double qr_fallback_yaw_deg = 30.0;

// V18：二维码URL HTTP层可靠性增强。
// 连接阶段单独限制，避免DNS/TCP/TLS异常时长时间卡住；
// 总超时稍微放宽，给服务器返回JSON留出余量。
double qr_http_connect_timeout = 1.0;
double qr_http_total_timeout = 3.0;
double qr_http_retry_delay = 0.15;
int qr_max_scan_rounds = 2;
int qr_http_retry_count = 2;

bool qr_camera_started = false;
vector<string> successful_qr_urls;
map<string, int> qr_url_failures_this_round;

// 整个二维码扫码会话复用一个 CURL easy handle。
// libcurl 会自动复用 DNS 缓存、TCP/TLS 连接（服务器允许时），
// 避免三个二维码每次都从零开始建立 HTTPS。
CURL* qr_http_curl = nullptr;
bool qr_curl_global_initialized = false;

// ================= HTTP 解析 =================
size_t WriteCallback(
    void* contents,
    size_t size,
    size_t nmemb,
    string* userp
) {
    userp->append(
        static_cast<char*>(contents),
        size * nmemb
    );
    return size * nmemb;
}

bool ensureQrHttpSession() {
    if (!qr_curl_global_initialized) {
        const CURLcode init_code =
            curl_global_init(CURL_GLOBAL_DEFAULT);

        if (init_code != CURLE_OK) {
            ROS_ERROR(
                "初始化 libcurl 全局环境失败：%s",
                curl_easy_strerror(init_code)
            );
            return false;
        }

        qr_curl_global_initialized = true;
    }

    if (qr_http_curl != nullptr) {
        return true;
    }

    qr_http_curl = curl_easy_init();

    if (qr_http_curl == nullptr) {
        ROS_ERROR("创建二维码 HTTP CURL easy handle 失败");
        return false;
    }

    // 这些选项在整个二维码阶段保持不变。
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_WRITEFUNCTION,
        WriteCallback
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_NOSIGNAL,
        1L
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_FOLLOWLOCATION,
        1L
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_MAXREDIRS,
        3L
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_TCP_KEEPALIVE,
        1L
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_USERAGENT,
        "ucar-ifly2026-qr/1.0"
    );

    ROS_INFO(
        "二维码 HTTP 会话已建立：本轮扫码将复用同一个 CURL handle，"
        "连接超时=%.2fs，总超时=%.2fs",
        qr_http_connect_timeout,
        qr_http_total_timeout
    );

    return true;
}

void cleanupQrHttpSession() {
    if (qr_http_curl != nullptr) {
        curl_easy_cleanup(qr_http_curl);
        qr_http_curl = nullptr;

        ROS_INFO(
            "二维码 HTTP 会话已释放"
        );
    }
}

void cleanupQrCurlGlobal() {
    cleanupQrHttpSession();

    if (qr_curl_global_initialized) {
        curl_global_cleanup();
        qr_curl_global_initialized = false;
    }
}

void logQrHttpTiming(
    CURL* curl,
    CURLcode perform_result,
    const string& url
) {
    double name_lookup_time = 0.0;
    double connect_time = 0.0;
    double app_connect_time = 0.0;
    double start_transfer_time = 0.0;
    double total_time = 0.0;
    long http_status = 0;
    char* primary_ip = nullptr;

    curl_easy_getinfo(
        curl,
        CURLINFO_NAMELOOKUP_TIME,
        &name_lookup_time
    );
    curl_easy_getinfo(
        curl,
        CURLINFO_CONNECT_TIME,
        &connect_time
    );
    curl_easy_getinfo(
        curl,
        CURLINFO_APPCONNECT_TIME,
        &app_connect_time
    );
    curl_easy_getinfo(
        curl,
        CURLINFO_STARTTRANSFER_TIME,
        &start_transfer_time
    );
    curl_easy_getinfo(
        curl,
        CURLINFO_TOTAL_TIME,
        &total_time
    );
    curl_easy_getinfo(
        curl,
        CURLINFO_RESPONSE_CODE,
        &http_status
    );
    curl_easy_getinfo(
        curl,
        CURLINFO_PRIMARY_IP,
        &primary_ip
    );

    // libcurl 各阶段时间都是“从请求开始累计”的时间。
    // 为了现场更直观，这里把 TCP/TLS 单独换算成阶段耗时。
    const double tcp_time =
        max(
            0.0,
            connect_time - name_lookup_time
        );

    const double tls_time =
        app_connect_time > 0.0
        ? max(
              0.0,
              app_connect_time - connect_time
          )
        : 0.0;

    const double server_wait_start =
        app_connect_time > 0.0
        ? app_connect_time
        : connect_time;

    const double server_wait_time =
        start_transfer_time > 0.0
        ? max(
              0.0,
              start_transfer_time - server_wait_start
          )
        : 0.0;

    if (perform_result == CURLE_OK) {
        ROS_INFO(
            "二维码 HTTP 耗时：HTTP=%ld，IP=%s，"
            "DNS=%.3fs，TCP=%.3fs，TLS=%.3fs，"
            "服务端等待=%.3fs，首字节=%.3fs，总耗时=%.3fs",
            http_status,
            primary_ip != nullptr ? primary_ip : "未知",
            name_lookup_time,
            tcp_time,
            tls_time,
            server_wait_time,
            start_transfer_time,
            total_time
        );
    } else {
        ROS_WARN(
            "二维码 HTTP 失败耗时：error=%s，HTTP=%ld，IP=%s，"
            "DNS=%.3fs，TCP=%.3fs，TLS=%.3fs，"
            "首字节=%.3fs，总耗时=%.3fs，URL=%s",
            curl_easy_strerror(perform_result),
            http_status,
            primary_ip != nullptr ? primary_ip : "未知",
            name_lookup_time,
            tcp_time,
            tls_time,
            start_transfer_time,
            total_time,
            url.c_str()
        );
    }
}

string httpGet(const string& url) {
    string read_buffer;

    if (!ensureQrHttpSession()) {
        return "";
    }

    const long connect_timeout_ms =
        static_cast<long>(
            max(
                0.2,
                qr_http_connect_timeout
            ) * 1000.0
        );

    const long total_timeout_ms =
        static_cast<long>(
            max(
                qr_http_connect_timeout + 0.2,
                qr_http_total_timeout
            ) * 1000.0
        );

    // 每次请求只替换 URL、输出缓冲区和本次超时。
    // easy handle 本身保持不变，因此连接池/DNS缓存可跨二维码复用。
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_URL,
        url.c_str()
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_WRITEDATA,
        &read_buffer
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_CONNECTTIMEOUT_MS,
        connect_timeout_ms
    );
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_TIMEOUT_MS,
        total_timeout_ms
    );

    char error_buffer[CURL_ERROR_SIZE] = {0};
    curl_easy_setopt(
        qr_http_curl,
        CURLOPT_ERRORBUFFER,
        error_buffer
    );

    const CURLcode result =
        curl_easy_perform(qr_http_curl);

    logQrHttpTiming(
        qr_http_curl,
        result,
        url
    );

    if (result != CURLE_OK) {
        ROS_WARN(
            "二维码地址请求失败：%s%s%s",
            curl_easy_strerror(result),
            error_buffer[0] != '\0' ? "；详细信息：" : "",
            error_buffer[0] != '\0' ? error_buffer : ""
        );

        read_buffer.clear();
        return "";
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

string trimCopy(const string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == string::npos) {
        return "";
    }
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool startsWith(const string& value, const string& prefix) {
    return value.size() >= prefix.size()
        && value.compare(0, prefix.size(), prefix) == 0;
}

vector<string> splitQrServerResults(const string& raw_result) {
    vector<string> results;
    istringstream stream(raw_result);
    string line;

    while (getline(stream, line)) {
        line = trimCopy(line);
        if (line.empty()) {
            continue;
        }

        // 兼容旧链路中可能出现的 “前缀|URL” 格式。
        const size_t split_pos = line.find("|");
        if (split_pos != string::npos) {
            line = trimCopy(line.substr(split_pos + 1));
        }

        if (!line.empty()) {
            results.push_back(line);
        }
    }

    return results;
}

bool qrUrlAlreadyAccepted(const string& url) {
    return find(
        successful_qr_urls.begin(),
        successful_qr_urls.end(),
        url
    ) != successful_qr_urls.end();
}

void stopQrCamera() {
    if (!qr_camera_started) {
        return;
    }

    qr_01::qr_code srv;
    srv.request.command = -2;
    if (!qr_client.call(srv)) {
        ROS_WARN("释放二维码摄像头服务调用失败");
    }
    qr_camera_started = false;
}

bool ensureQrCameraReady() {
    if (qr_camera_started) {
        return true;
    }

    for (int attempt = 1; attempt <= 2; ++attempt) {
        qr_01::qr_code srv;
        srv.request.command = -1;

        if (!qr_client.call(srv)) {
            ROS_WARN(
                "开启二维码摄像头服务调用失败，第 %d/2 次",
                attempt
            );
        } else if (
            srv.response.result.empty()
            || !startsWith(srv.response.result, "ERROR:")
        ) {
            qr_camera_started = true;
            ROS_INFO(
                "二维码摄像头已开启，预热 %.2f 秒",
                qr_camera_warmup
            );
            ros::Duration(max(0.0, qr_camera_warmup)).sleep();
            return true;
        } else {
            ROS_WARN(
                "二维码摄像头开启失败：%s，第 %d/2 次",
                srv.response.result.c_str(),
                attempt
            );
        }

        if (attempt < 2) {
            ros::Duration(0.20).sleep();
        }
    }

    return false;
}

bool flushQrCamera() {
    qr_01::qr_code srv;
    srv.request.command = -3;

    if (
        qr_client.call(srv)
        && (
            srv.response.result.empty()
            || !startsWith(srv.response.result, "ERROR:")
        )
    ) {
        return true;
    }

    if (!srv.response.result.empty()) {
        ROS_WARN(
            "清理二维码摄像头缓存失败：%s，尝试重新初始化相机",
            srv.response.result.c_str()
        );
    } else {
        ROS_WARN("清理二维码摄像头缓存服务调用失败，尝试重新初始化相机");
    }

    stopQrCamera();
    if (!ensureQrCameraReady()) {
        return false;
    }

    srv.request.command = -3;
    if (
        !qr_client.call(srv)
        || startsWith(srv.response.result, "ERROR:")
    ) {
        ROS_WARN(
            "重新初始化后仍无法清理摄像头缓存：%s",
            srv.response.result.c_str()
        );
        return false;
    }

    return true;
}

double qrFallbackYawRad() {
    return qr_fallback_yaw_deg
        * 3.14159265358979323846 / 180.0;
}

double currentQrViewYaw() {
    const double base_yaw = qr_wp_yaw[qr_waypoint_idx];

    if (qr_view_mode == QR_VIEW_LEFT) {
        return base_yaw + qrFallbackYawRad();
    }
    if (qr_view_mode == QR_VIEW_RIGHT) {
        return base_yaw - qrFallbackYawRad();
    }
    return base_yaw;
}

const char* currentQrViewName() {
    if (qr_view_mode == QR_VIEW_LEFT) {
        return "左转fallback";
    }
    if (qr_view_mode == QR_VIEW_RIGHT) {
        return "右转fallback";
    }
    return "正常朝向";
}

void advanceQrWaypoint() {
    ++qr_waypoint_idx;
    qr_view_mode = QR_VIEW_NORMAL;
    qr_decoded_any_at_current_view = false;
}

double currentQrScanTimeout() {
    return qr_scan_round == 0
        ? qr_scan_timeout
        : qr_retry_scan_timeout;
}

bool acceptQrItem(const string& url, const string& item) {
    if (item.empty() || qrUrlAlreadyAccepted(url)) {
        return false;
    }

    successful_qr_urls.push_back(url);

    if (valid_qr_count == 0) {
        target_qr_1 = item;
    } else if (valid_qr_count == 1) {
        target_qr_2 = item;
    } else if (valid_qr_count == 2) {
        target_qr_3 = item;
    } else {
        return false;
    }

    valid_qr_count++;

    ROS_INFO(
        "录入第 %d 个有效二维码：%s",
        valid_qr_count,
        item.c_str()
    );
    return true;
}

bool processQrUrl(const string& url) {
    if (url.empty() || qrUrlAlreadyAccepted(url)) {
        return false;
    }

    const auto fail_it = qr_url_failures_this_round.find(url);
    if (
        fail_it != qr_url_failures_this_round.end()
        && fail_it->second >= qr_http_retry_count
    ) {
        return false;
    }

    const int max_attempts = max(1, qr_http_retry_count);
    bool accepted = false;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        ROS_INFO(
            "请求二维码链接，第 %d/%d 次：%s",
            attempt,
            max_attempts,
            url.c_str()
        );

        const string json = httpGet(url);
        const int code = extractCode(json);
        const string result_text =
            code == 200 ? trimCopy(extractResult(json)) : "";

        if (code == 200 && !result_text.empty()) {
            accepted = acceptQrItem(url, result_text);
            qr_url_failures_this_round.erase(url);
            break;
        }

        if (code == 200 && result_text.empty()) {
            ROS_WARN("二维码接口返回 code=200，但 result 为空");
        } else if (code == 400) {
            ROS_WARN("二维码接口返回 code=400，本轮进行有限重试");
        } else {
            ROS_WARN(
                "二维码接口返回异常或网络失败，code=%d",
                code
            );
        }

        if (attempt < max_attempts) {
            ros::Duration(max(0.0, qr_http_retry_delay)).sleep();
        }
    }

    if (!accepted) {
        qr_url_failures_this_round[url] = max_attempts;
    }

    return accepted;
}

bool processQrServerResponse(const string& raw_result) {
    if (raw_result.empty()) {
        return false;
    }

    if (startsWith(raw_result, "ERROR:")) {
        ROS_WARN(
            "二维码服务返回相机错误：%s",
            raw_result.c_str()
        );

        // CAMERA_CLOSED / FRAME_EMPTY 等异常不应被当成 URL。
        // 下一观察点到位时会重新检查相机状态。
        if (
            raw_result.find("CAMERA_CLOSED") != string::npos
            || raw_result.find("CAMERA_OPEN_FAILED") != string::npos
        ) {
            qr_camera_started = false;
        }
        return false;
    }

    const vector<string> urls = splitQrServerResults(raw_result);
    if (urls.empty()) {
        return false;
    }

    // 只要 QR Server 返回了至少一个真实二维码字符串，就说明当前视角
    // 的视觉解码已经成功。即使它是旧 URL，或者后续 HTTP 请求失败，
    // 也不再把这个视角当作“视觉识别失败”触发左右转 fallback。
    qr_decoded_any_at_current_view = true;

    const ros::WallTime http_start = ros::WallTime::now();
    bool found_new_item = false;

    for (const string& url : urls) {
        if (valid_qr_count >= 3) {
            break;
        }
        if (processQrUrl(url)) {
            found_new_item = true;
        }
    }

    const double http_elapsed =
        (ros::WallTime::now() - http_start).toSec();

    // HTTP 请求不占用当前观察点的视觉驻留时间：把阻塞的 HTTP 时间
    // 加回扫描起点，而不是重新赠送完整的扫描时长。
    if (http_elapsed > 0.001) {
        scan_start_time += ros::Duration(http_elapsed);
    }

    if (found_new_item) {
        qr_found_item_at_current_waypoint = true;
        qr_post_success_deadline =
            ros::Time::now()
            + ros::Duration(max(0.0, qr_post_success_scan_seconds));
    }

    return found_new_item;
}

void loadQrParameters(ros::NodeHandle& private_nh) {
    const vector<double> default_x = qr_wp_x;
    const vector<double> default_y = qr_wp_y;
    const vector<double> default_yaw = qr_wp_yaw;

    private_nh.getParam("qr_wp_x", qr_wp_x);
    private_nh.getParam("qr_wp_y", qr_wp_y);
    private_nh.getParam("qr_wp_yaw", qr_wp_yaw);

    if (
        qr_wp_x.empty()
        || qr_wp_x.size() != qr_wp_y.size()
        || qr_wp_x.size() != qr_wp_yaw.size()
    ) {
        ROS_ERROR(
            "二维码观察点参数长度不一致，恢复代码默认四观察点"
        );
        qr_wp_x = default_x;
        qr_wp_y = default_y;
        qr_wp_yaw = default_yaw;
    }

    private_nh.param(
        "qr_scan_timeout",
        qr_scan_timeout,
        1.2
    );
    private_nh.param(
        "qr_retry_scan_timeout",
        qr_retry_scan_timeout,
        2.0
    );
    private_nh.param(
        "qr_camera_warmup",
        qr_camera_warmup,
        0.7
    );
    private_nh.param(
        "qr_post_success_scan_seconds",
        qr_post_success_scan_seconds,
        0.15
    );
    private_nh.param(
        "qr_fallback_yaw_deg",
        qr_fallback_yaw_deg,
        30.0
    );
    // V18：HTTP连接阶段和整个请求总时长分别限制。
    // 默认连接1.0s、总请求3.0s。
    const bool has_new_total_timeout =
        private_nh.hasParam(
            "qr_http_total_timeout"
        );

    private_nh.param(
        "qr_http_connect_timeout",
        qr_http_connect_timeout,
        1.0
    );
    private_nh.param(
        "qr_http_total_timeout",
        qr_http_total_timeout,
        3.0
    );

    // 兼容旧launch里的 qr_http_timeout：
    // 如果只配置了旧参数，则把它当成旧“总超时”参考值，
    // 但比赛默认至少保留3.0s，避免旧2.0s继续造成偶发误杀。
    if (
        !has_new_total_timeout
        && private_nh.hasParam(
            "qr_http_timeout"
        )
    ) {
        double legacy_timeout = 2.0;

        private_nh.getParam(
            "qr_http_timeout",
            legacy_timeout
        );

        qr_http_total_timeout =
            max(
                3.0,
                legacy_timeout
            );

        ROS_WARN(
            "检测到旧参数 ~qr_http_timeout=%.2fs；"
            "V18已拆分为 qr_http_connect_timeout / "
            "qr_http_total_timeout，本次总超时采用 %.2fs",
            legacy_timeout,
            qr_http_total_timeout
        );
    }

    private_nh.param(
        "qr_http_retry_delay",
        qr_http_retry_delay,
        0.15
    );
    private_nh.param(
        "qr_max_scan_rounds",
        qr_max_scan_rounds,
        2
    );
    private_nh.param(
        "qr_http_retry_count",
        qr_http_retry_count,
        2
    );

    qr_max_scan_rounds =
        max(
            1,
            qr_max_scan_rounds
        );

    qr_http_retry_count =
        max(
            1,
            qr_http_retry_count
        );

    qr_http_connect_timeout =
        max(
            0.2,
            qr_http_connect_timeout
        );

    qr_http_total_timeout =
        max(
            qr_http_connect_timeout + 0.2,
            qr_http_total_timeout
        );

    qr_http_retry_delay =
        max(
            0.0,
            qr_http_retry_delay
        );

    ROS_INFO(
        "二维码参数：观察点=%zu，首轮每点=%.2fs，兜底每点=%.2fs，"
        "相机预热=%.2fs，单墙成功后追加扫描=%.2fs，"
        "视觉失败左右fallback=±%.1f°，最大轮数=%d；"
        "HTTP连接超时=%.2fs，总超时=%.2fs，重试=%d次，间隔=%.2fs",
        qr_wp_x.size(),
        qr_scan_timeout,
        qr_retry_scan_timeout,
        qr_camera_warmup,
        qr_post_success_scan_seconds,
        qr_fallback_yaw_deg,
        qr_max_scan_rounds,
        qr_http_connect_timeout,
        qr_http_total_timeout,
        qr_http_retry_count,
        qr_http_retry_delay
    );
}


// ============================================================================
// 集成 V14.9 找板停靠模块。
// race 先到 (1.25,4.30,-90°)，随后本模块执行
// (1.25,4.30,0°) 起点调整，再开始四墙固定 Path 巡检。
// ============================================================================
namespace {

const double kPi = 3.14159265358979323846;

double clampValue(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

double normalizeAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double distance2D(double x0, double y0, double x1, double y1) {
    return std::hypot(x1 - x0, y1 - y0);
}

}  // namespace

class TargetPatrolDocking {
public:
    TargetPatrolDocking(
            const std::string& real_target_category,
            const std::string& simulation_target_category,
            const std::function<void()>& real_docked_callback)
        : nh_(),
          pnh_("~"),
          move_base_("move_base", true),
          tf_listener_(tf_buffer_),
          real_docked_callback_(real_docked_callback) {
        real_target_category_ = real_target_category;
        simulation_target_category_ = simulation_target_category;

        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("room_min_x", room_min_x_, 0.0);
        pnh_.param("room_max_x", room_max_x_, 5.0);
        pnh_.param("room_min_y", room_min_y_, 2.5);
        pnh_.param("room_max_y", room_max_y_, 4.5);
        pnh_.param("start_x", start_x_, 1.25);
        pnh_.param("start_y", start_y_, 4.30);
        pnh_.param("start_yaw_deg", start_yaw_deg_, 0.0);
        pnh_.param("navigation_timeout", navigation_timeout_, 180.0);

        pnh_.param("planner_private_namespace", planner_private_namespace_,
                   std::string("/move_base/MyPlanner"));
        pnh_.param("patrol_path_spacing", patrol_path_spacing_, 0.02);
        pnh_.param("patrol_speed_limit", patrol_speed_limit_, 0.50);
        pnh_.param("normal_navigation_speed_limit",
                   normal_navigation_speed_limit_, 1.00);
        pnh_.param("patrol_cancel_timeout", patrol_cancel_timeout_, 3.0);
        pnh_.param("patrol_interface_timeout", patrol_interface_timeout_, 3.0);
        pnh_.param("move_base_reconfigure_service",
                   move_base_reconfigure_service_,
                   std::string("/move_base/set_parameters"));
        pnh_.param("disable_move_base_oscillation_during_patrol",
                   disable_move_base_oscillation_during_patrol_, true);
        pnh_.param("normal_move_base_oscillation_timeout",
                   normal_move_base_oscillation_timeout_, 8.0);
        pnh_.param("patrol_aborted_retry_count",
                   patrol_aborted_retry_count_, 2);

        pnh_.param("segment_end_tolerance", segment_end_tolerance_, 0.015);
        pnh_.param("control_rate", control_rate_, 15.0);

        //  V12：角点安全偏移与同时转向/平移控制参数。
        pnh_.param("patrol_transition_position_kp",
                   patrol_transition_position_kp_, 2.50);
        pnh_.param("patrol_transition_yaw_kp",
                   patrol_transition_yaw_kp_, 3.00);
        pnh_.param("patrol_transition_min_linear_speed",
                   patrol_transition_min_linear_speed_, 0.025);
        pnh_.param("patrol_transition_max_linear_speed",
                   patrol_transition_max_linear_speed_, 0.12);
        pnh_.param("patrol_transition_min_angular_speed",
                   patrol_transition_min_angular_speed_, 0.25);
        pnh_.param("patrol_transition_max_angular_speed",
                   patrol_transition_max_angular_speed_, 1.20);
        pnh_.param("patrol_transition_linear_accel",
                   patrol_transition_linear_accel_, 0.60);
        pnh_.param("patrol_transition_angular_accel",
                   patrol_transition_angular_accel_, 4.00);
        pnh_.param("patrol_transition_yaw_priority_start_deg",
                   patrol_transition_yaw_priority_start_deg_, 55.0);
        pnh_.param("patrol_transition_yaw_priority_release_deg",
                   patrol_transition_yaw_priority_release_deg_, 20.0);
        pnh_.param("patrol_transition_yaw_priority_min_linear_scale",
                   patrol_transition_yaw_priority_min_linear_scale_, 0.15);
        pnh_.param("patrol_transition_position_tolerance",
                   patrol_transition_position_tolerance_, 0.012);
        pnh_.param("patrol_transition_yaw_tolerance_deg",
                   patrol_transition_yaw_tolerance_deg_, 1.5);
        pnh_.param("patrol_transition_stable_frames",
                   patrol_transition_stable_frames_, 3);
        pnh_.param("patrol_transition_timeout",
                   patrol_transition_timeout_, 8.0);

        // V22：角点自适应过渡。
        // 到达前三条固定巡线段终点后，在指定map坐标附近查询local costmap。
        // 只要半径内存在高代价死区，就继续沿用原V13安全角点；
        // 没有死区时则向下一条巡线段方向额外提前10cm。
        pnh_.param(
            "corner_obstacle_costmap_topic",
            corner_obstacle_costmap_topic_,
            std::string("/move_base/local_costmap/costmap"));
        pnh_.param(
            "corner_obstacle_check_radius",
            corner_obstacle_check_radius_,
            0.05);
        pnh_.param(
            "corner_obstacle_lethal_cost_threshold",
            corner_obstacle_lethal_cost_threshold_,
            99);
        pnh_.param(
            "corner_obstacle_costmap_max_age",
            corner_obstacle_costmap_max_age_,
            3.0);

        // V23：现实目标在当前巡检墙完成停靠后，
        // 如果“本次识别时首次生成的停靠导航点”距当前巡检段终点很近，
        // 则跳过V17回线和剩余固定Path，直接执行本段角点旋转平移。
        pnh_.param(
            "patrol_near_end_corner_shortcut_distance",
            patrol_near_end_corner_shortcut_distance_,
            1.0);

        // V24：
        // 非当前左墙现实目标若位于当前巡检段终点附近，
        // 不再强制等到整段完全结束。
        // 当目标板墙面坐标距本段终点小于该值，并且机器人自身
        // 也进入本段终点同样距离范围时，允许提前中断当前Path去停靠。
        pnh_.param(
            "patrol_noncurrent_target_early_dock_distance",
            patrol_noncurrent_target_early_dock_distance_,
            1.0);

        pnh_.param("image_width", image_width_, 640);
        pnh_.param("camera_fx", camera_fx_, 554.256);
        pnh_.param("camera_yaw_offset_deg", camera_yaw_offset_deg_, 0.0);
        pnh_.param("docking_standoff", docking_standoff_, 0.50);
        pnh_.param("settle_time", settle_time_, 0.25);
        pnh_.param("ocr_attempts", ocr_attempts_, 3);
        pnh_.param("ocr_retry_interval", ocr_retry_interval_, 0.12);
        pnh_.param("ocr_recovery_turn_deg", ocr_recovery_turn_deg_, 30.0);
        pnh_.param("ocr_recovery_turn_kp", ocr_recovery_turn_kp_, 2.0);
        pnh_.param("ocr_recovery_turn_min_speed",
                   ocr_recovery_turn_min_speed_, 0.18);
        pnh_.param("ocr_recovery_turn_max_speed",
                   ocr_recovery_turn_max_speed_, 0.45);
        pnh_.param("ocr_recovery_turn_tolerance_deg",
                   ocr_recovery_turn_tolerance_deg_, 1.5);
        pnh_.param("ocr_recovery_turn_stable_frames",
                   ocr_recovery_turn_stable_frames_, 3);
        pnh_.param("ocr_recovery_turn_timeout",
                   ocr_recovery_turn_timeout_, 5.0);
        pnh_.param("ocr_recovery_settle_time",
                   ocr_recovery_settle_time_, 0.35);
        pnh_.param("max_detection_duration", max_detection_duration_, 0.50);
        // V14.3保护1：仅当巡检NanoDet框中心x严格小于该像素值时，
        // 才允许中断固定巡检Path并停车进入OCR。默认640宽图像下为600。
        pnh_.param("patrol_stop_max_center_x",
                   patrol_stop_max_center_x_, 600.0);

        // V14.5：当前左侧巡检墙视觉目标接近减速。
        // 与line2o挡板减速同样使用线性速度比例：
        // 1.50m -> 1.00倍，0.50m -> 0.20倍，到0.50m取消巡检goal并停车OCR。
        pnh_.param("patrol_target_slowdown_start_distance",
                   patrol_target_slowdown_start_distance_, 1.50);
        pnh_.param("patrol_target_stop_distance", patrol_target_stop_distance_, 0.40);
        pnh_.param("patrol_target_min_speed_ratio",
                   patrol_target_min_speed_ratio_, 0.20);

        // V14.7保护1：
        // 非当前巡检墙只有在机器人距该墙本身小于该阈值时才允许立即停车。
        pnh_.param("patrol_noncurrent_wall_stop_max_distance",
                   patrol_noncurrent_wall_stop_max_distance_, 1.50);

        // V14.7保护2：
        // 当前左侧巡检墙目标框的左边界距图像左缘不足该像素值时立即停车。
        pnh_.param("patrol_target_left_edge_stop_px",
                   patrol_target_left_edge_stop_px_, 100);

        pnh_.param("duplicate_coordinate_distance",
                   duplicate_coordinate_distance_, 0.50);
        pnh_.param("max_track_jump_px", max_track_jump_px_, 140.0);
        pnh_.param("docking_recovery_turn_deg",
                   docking_recovery_turn_deg_, 30.0);
        pnh_.param("docking_recovery_detection_attempts",
                   docking_recovery_detection_attempts_, 3);
        pnh_.param("docking_recovery_detection_interval",
                   docking_recovery_detection_interval_, 0.0);
        // 到达第一段预停靠点后，第二次视觉只接受与第一次墙面估计
        // 同墙且坐标足够接近的框，防止把其他文字板当成当前目标。
        pnh_.param("docking_refine_max_board_shift",
                   docking_refine_max_board_shift_, 0.80);

        // V14.2：仅用于±30度恢复旋转后的即时缓存刷新。
        // 正常第一段导航期间摄像头会被关闭，因此到点重开后没有历史缓存。
        pnh_.param("docking_refresh_clear_calls",
                   docking_refresh_clear_calls_, 2);

        // V17：现实目标停靠后的快速恢复巡检。
        //
        // 不再使用单独的慢速恢复参数，速度策略完全复用
        // 两条巡检路线转角 runPatrolPoseTransition() 的 patrol_transition_*。
        //
        // 初始阶段仍严格：
        //   先纯旋转 -> 停稳 -> 再计算最短法向距离。
        //
        // 进入横移阶段以后允许 vy + wz 同时输出：
        //   vy 使用转角同款位置P控制/线速度/线加速度限制；
        //   wz 使用转角同款yaw P控制/角速度/角加速度限制。
        //
        // 因此第一次旋转稍微过头，或横移过程中底盘航向漂移时，
        // 可以一边继续横移，一边实时把车头修回 segment.travel_yaw。
        pnh_.param("patrol_resume_max_cross_track_error",
                   patrol_resume_max_cross_track_error_, 0.030);
        pnh_.param("approach_stop_distance", approach_stop_distance_, 0.28);

        normalizeParameters();
        buildSegments();
        configuration_valid_ = validateConfiguration();

        detect_client_ =
            nh_.serviceClient<ros_nanodet::detect_result_srv>("/nanodet_detect");
        ocr_client_ =
            nh_.serviceClient<ros_nanodet::ocr_result_srv>("/nanodet_ocr");
        set_speed_client_ =
            nh_.serviceClient<ucarmain2026::set_speed>("/set_speed");
        move_base_reconfigure_client_ =
            nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
                move_base_reconfigure_service_);
        patrol_path_publisher_ = nh_.advertise<nav_msgs::Path>(
            planner_private_namespace_ + "/patrol_path", 1, true);
        patrol_path_lock_client_ = nh_.serviceClient<std_srvs::SetBool>(
            planner_private_namespace_ + "/lock_patrol_path");
        controller_reset_client_ = nh_.serviceClient<std_srvs::Trigger>(
            planner_private_namespace_ + "/reset_controller_state");

        corner_costmap_subscriber_ =
            nh_.subscribe<nav_msgs::OccupancyGrid>(
                corner_obstacle_costmap_topic_,
                1,
                &TargetPatrolDocking::cornerCostmapCallback,
                this);
        ROS_INFO("车头向前巡检找板：现实目标=%s，仿真目标=%s",
                 categoryChinese(real_target_category_),
                 categoryChinese(simulation_target_category_));
        ROS_INFO("找板房间边界=[%.2f, %.2f]×[%.2f, %.2f]；"
                 "巡检视觉门限=centerX<%.1fpx；"
                 "当前左墙目标%.2fm开始减速，%.2fm处降到%.0f%%并停车；"
                 "当前墙框x0<%dpx立即停车；"
                 "非当前墙只有距该墙<%.2fm才立即停车；"
                 "停靠导航点距墙%.2fm",
                 room_min_x_, room_max_x_, room_min_y_, room_max_y_,
                 patrol_stop_max_center_x_,
                 patrol_target_slowdown_start_distance_,
                 patrol_target_stop_distance_,
                 patrol_target_min_speed_ratio_ * 100.0,
                 patrol_target_left_edge_stop_px_,
                 patrol_noncurrent_wall_stop_max_distance_,
                 docking_standoff_);
        ROS_WARN("IFLY2026_FIXED_PATH_PATROL_V14_TWO_STAGE_MOVEBASE_DOCK_20260812："
                 "target_patrol_docking 固定move_base路线巡检版已启动。"
                 "巡检速度=%.2f，普通导航速度=%.2f，路径间距=%.3f。",
                 patrol_speed_limit_, normal_navigation_speed_limit_,
                 patrol_path_spacing_);
        ROS_INFO(
            "角点自适应过渡：costmap=%s，检测半径=%.3fm，"
            "死区阈值>=%d，地图最大允许年龄=%.1fs；"
            "costmap异常时按有障碍物处理并保持原安全角点。",
            corner_obstacle_costmap_topic_.c_str(),
            corner_obstacle_check_radius_,
            corner_obstacle_lethal_cost_threshold_,
            corner_obstacle_costmap_max_age_);

        ROS_INFO(
            "段末停靠捷径：识别生成的停靠点距当前巡检段终点<%.2fm时，"
            "现实目标停靠完成后跳过V17回线和剩余固定Path，"
            "直接执行本段角点旋转平移；第四段无后续角点，不启用。",
            patrol_near_end_corner_shortcut_distance_);

        ROS_INFO(
            "非当前墙目标提前停靠："
            "目标板坐标距当前段终点<%.2fm时记录为可提前处理；"
            "随后机器人距当前段终点<%.2fm即取消剩余Path并去停靠，"
            "停靠后从当前位置并入目标板所在墙继续巡检。",
            patrol_noncurrent_target_early_dock_distance_,
            patrol_noncurrent_target_early_dock_distance_);

        ROS_INFO("OCR单字分类：食→食品，日/用→日用品，"
                 "电/子/产/生→电子产品；"
                 "仅识别到品/加/工/车/间时逆时针转%.1f度复识一次。",
                 ocr_recovery_turn_deg_);
        ROS_INFO("停靠点视觉恢复：第一段预停靠后若当前朝向无法复定位原目标，"
                 "按到点朝向依次扫描左侧+%.1f度和右侧-%.1f度；"
                 "找到后直接计算最终move_base停靠点。",
                 docking_recovery_turn_deg_,
                 docking_recovery_turn_deg_);
    }

    ~TargetPatrolDocking() {
        stopRobot();
        closeCamera();
    }

    void stopAndHold() {
        stopRobot();
    }

    bool realTargetDocked() const {
        return real_docked_;
    }

    bool simulationTargetDocked() const {
        return simulation_docked_;
    }

    bool simulationTargetRecorded() const {
        return (
            simulation_target_pending_
            && simulation_observation_.valid
            && !simulation_docked_
        );
    }

    bool run() {
        if (!configuration_valid_) return false;
        if (!waitForDependencies()) return false;

        if (!navigateToPose(start_x_, start_y_,
                            start_yaw_deg_ * kPi / 180.0,
                            "初始巡检点")) {
            return false;
        }

        // 初始move_base直接到(1.25,4.30,0deg)，
        // 第一条固定Patrol Path仍保持原路线，因此不再执行旧的
        // (0.25,4.25)->(0.25,4.30)额外安全横移。
        setShadowModeActiveOnce();
        if (!openCamera()) return false;

        for (std::size_t index = 0; index < segments_.size() && ros::ok();) {
            current_segment_index_ = static_cast<int>(index);
            const SegmentResult result = patrolSegment(index);
            if (result == SEGMENT_MISSION_COMPLETE) {
                finishPatrolMode();
                printSummary(true);
                return true;
            }
            if (result == SEGMENT_ABORTED) {
                finishPatrolMode();
                printSummary(false);
                return false;
            }

            if (result == SEGMENT_SWITCH_TO_TARGET_WALL) {
                if (
                    forced_next_segment_index_ < 0
                    || forced_next_segment_index_
                        >= static_cast<int>(segments_.size())
                    || forced_next_segment_index_
                        == static_cast<int>(index)
                ) {
                    ROS_ERROR(
                        "V24目标墙切换索引非法：current=%zu next=%d",
                        index,
                        forced_next_segment_index_);
                    finishPatrolMode();
                    printSummary(false);
                    return false;
                }

                ROS_WARN(
                    "V24直接切换巡检墙：%s -> %s；"
                    "机器人已经从现实目标停靠位置并入目标墙巡检线，"
                    "下一轮从当前位置继续该墙剩余固定Path。",
                    segments_[index].name.c_str(),
                    segments_[
                        static_cast<std::size_t>(
                            forced_next_segment_index_)
                    ].name.c_str());

                index =
                    static_cast<std::size_t>(
                        forced_next_segment_index_);

                forced_next_segment_index_ = -1;
                continue;
            }

            ++index;
        }

        finishPatrolMode();
        stopRobot();
        closeCamera();

        // V14.9：只有四面墙完整巡检后任务仍未完成，才启动unknown候选回访。
        // 正常巡检过程中候选只记录，不改变任何既有停车/停靠决策。
        if ((!real_docked_ || !simulation_docked_) &&
            !unknown_candidates_.empty()) {
            ROS_WARN(
                "四面墙完整巡检结束但双目标尚未全部完成；"
                "已记录%zu个unknown候选，开始按发现顺序逐个回访。",
                unknown_candidates_.size());

            if (!revisitUnknownCandidates()) {
                finishPatrolMode();
                stopRobot();
                closeCamera();
                printSummary(false);
                return false;
            }
        }

        // V20最终兜底：
        // 四墙巡检 + unknown回访已经全部结束。
        // 如果只找到了仿真目标而现实目标始终没找到，
        // 不再继续等待现实目标，直接停靠已经记录的仿真目标。
        if (
            !real_docked_
            && !simulation_docked_
            && hasRecordedSimulationTarget()
        ) {
            ROS_ERROR(
                "找板最终结果：已找到仿真目标，但现实目标始终未找到；"
                "放弃现实目标实体停靠，直接前往已记录仿真目标。"
            );

            simulation_target_blocked_until_segment_end_ = false;

            if (!dockPendingSimulationTarget(true)) {
                finishPatrolMode();
                stopRobot();
                closeCamera();
                printSummary(false);
                return false;
            }
        }

        finishPatrolMode();
        stopRobot();
        closeCamera();

        const bool both_docked =
            real_docked_ && simulation_docked_;

        printSummary(both_docked);

        if (both_docked) {
            ROS_INFO(
                "找板最终结果：现实目标和仿真目标均已完成实体停靠。"
            );
        } else if (
            !real_docked_
            && simulation_docked_
        ) {
            ROS_WARN(
                "找板最终结果：仅仿真目标完成实体停靠；"
                "现实目标未找到，已放弃现实目标停靠，继续后续仿真任务。"
            );
        } else if (
            real_docked_
            && !simulation_docked_
        ) {
            ROS_WARN(
                "找板最终结果：现实目标已完成实体停靠，"
                "仿真目标最终未找到；"
                "不再搜索仿真目标，保持当前位置直接进入仿真任务。"
            );
        } else {
            ROS_ERROR(
                "找板最终结果：现实目标和仿真目标均未完成实体停靠；"
                "不再继续找板，保持当前位置直接进入仿真任务。"
            );
        }

        // 注意：这里的true表示“找板阶段已经按照兜底规则正常收尾，
        // 可以进入仿真通信”，不再要求两个实体目标都停靠成功。
        return true;
    }

private:
    void notifyRealDockedOnce() {
        if (real_dock_notification_sent_) {
            return;
        }

        real_dock_notification_sent_ = true;

        ROS_WARN("现实目标最终停靠完成，触发 race 中间播报");

        if (real_docked_callback_) {
            real_docked_callback_();
        }
    }

    struct Pose2D {
        double x;
        double y;
        double yaw;

        Pose2D() : x(0.0), y(0.0), yaw(0.0) {}
        Pose2D(double px, double py, double pyaw) : x(px), y(py), yaw(pyaw) {}
    };

    enum WallType {
        WALL_LEFT = 0,
        WALL_RIGHT = 1,
        WALL_BOTTOM = 2,
        WALL_TOP = 3
    };

    struct Segment {
        std::string name;
        double start_x;
        double start_y;
        double end_x;
        double end_y;
        double dir_x;
        double dir_y;
        double length;
        double travel_yaw;
        double docking_yaw;
        WallType wall;
    };

    struct Box {
        int class_id;
        int x0;
        int y0;
        int x1;
        int y1;

        double centerX() const {
            return 0.5 * static_cast<double>(x0 + x1);
        }
        double centerY() const {
            return 0.5 * static_cast<double>(y0 + y1);
        }
        double width() const {
            return std::max(1, x1 - x0);
        }
        double height() const {
            return std::max(1, y1 - y0);
        }
    };

    struct OcrRecord {
        bool success;
        std::string text;
        std::string category;
        double confidence;
        Box box;

        OcrRecord()
            : success(false),
              text("<未识别>"),
              category("unknown"),
              confidence(0.0),
              box{0, 0, 0, 0, 0} {}
    };

    struct TargetObservation {
        bool valid;
        Pose2D pose;
        int segment_index;
        std::string category;
        // 保存第一次视觉估计出的真实墙面坐标。第二阶段视觉复定位
        // 用它关联“同一块板”，不再按画面中心随意挑框。
        bool board_valid;
        WallType board_wall;
        double board_x;
        double board_y;

        TargetObservation()
            : valid(false),
              segment_index(-1),
              category("unknown"),
              board_valid(false),
              board_wall(WALL_LEFT),
              board_x(0.0),
              board_y(0.0) {}
    };

    struct PatrolCheckpoint {
        bool valid;
        int segment_index;
        double stopped_progress;
        Pose2D stopped_pose;

        PatrolCheckpoint()
            : valid(false), segment_index(-1), stopped_progress(0.0) {}
    };

    struct BoardBoundaryEstimate {
        bool valid;
        WallType wall;
        double x;
        double y;

        BoardBoundaryEstimate()
            : valid(false), wall(WALL_LEFT), x(0.0), y(0.0) {}
    };

    // V14.9：OCR最终仍为unknown的墙面候选。
    // 只用于整圈巡检完成后的兜底回访，不参与正常巡检决策。
    struct UnknownCandidate {
        BoardBoundaryEstimate board;
        int source_segment_index;
        Pose2D source_pose;
        Box source_box;
        bool attempted;
        bool resolved;

        UnknownCandidate()
            : source_segment_index(-1),
              source_box{0, 0, 0, 0, 0},
              attempted(false),
              resolved(false) {}
    };

    // V14.5：当前左侧巡检墙目标的接近状态。
    // 一旦在1.5m减速范围内确认，就锁存墙面map坐标。
    // 即使随后NanoDet偶发丢1~2帧，仍可依据map位姿连续减速到0.5m。
    struct PatrolVisualApproach {
        bool valid;
        int segment_index;
        BoardBoundaryEstimate board;
        Box latest_box;
        bool have_latest_box;

        PatrolVisualApproach()
            : valid(false),
              segment_index(-1),
              latest_box{0, 0, 0, 0, 0},
              have_latest_box(false) {}
    };

    enum SegmentResult {
        SEGMENT_COMPLETE,

        // V24：
        // 当前巡检段尚未跑到原终点，但已经按新规则提前停靠
        // 非当前墙现实目标，并从当前位置并入目标墙巡检线。
        // 外层run()收到后直接把index切换到目标墙对应Segment。
        SEGMENT_SWITCH_TO_TARGET_WALL,

        SEGMENT_MISSION_COMPLETE,
        SEGMENT_ABORTED
    };

    enum DetectionResult {
        DETECTION_CONTINUE,

        // V23：
        // 当前现实目标停靠点已接近本段终点，并且已经直接完成
        // 本段角点旋转平移。patrolSegment收到后不再重跑剩余Path，
        // 只执行段末状态收尾并进入下一段。
        DETECTION_SEGMENT_COMPLETE_AFTER_CORNER,

        DETECTION_MISSION_COMPLETE,
        DETECTION_ABORT
    };

    enum PatrolStartResult {
        PATROL_STARTED,
        PATROL_ALREADY_COMPLETE,
        PATROL_START_FAILED
    };

    static bool isValidCategory(const std::string& category) {
        return category == "food" || category == "daily" ||
               category == "electronic";
    }

    static std::string classifyText(const std::string& text) {
        // OCR可能因为停车过晚只读到类别名称中的一部分，因此三个类别
        // 都按任意一个关键字命中。优先级保持为食品、日用品、电子产品。
        if (text.find("食") != std::string::npos) return "food";
        if (text.find("日") != std::string::npos ||
            text.find("用") != std::string::npos) {
            return "daily";
        }
        if (text.find("电") != std::string::npos ||
            text.find("子") != std::string::npos ||
            text.find("产") != std::string::npos ||
            text.find("生") != std::string::npos) {
            return "electronic";
        }
        return "unknown";
    }

    static bool hasWorkshopFragment(const std::string& text) {
        return text.find("品") != std::string::npos ||
               text.find("加") != std::string::npos ||
               text.find("工") != std::string::npos ||
               text.find("车") != std::string::npos ||
               text.find("间") != std::string::npos;
    }

    static const char* categoryChinese(const std::string& category) {
        if (category == "food") return "食品加工车间";
        if (category == "daily") return "日用品加工车间";
        if (category == "electronic") return "电子产品生产车间";
        return "未知";
    }

    void normalizeParameters() {
        if (!planner_private_namespace_.empty() &&
            planner_private_namespace_[0] != '/') {
            planner_private_namespace_ = "/" + planner_private_namespace_;
        }
        while (planner_private_namespace_.size() > 1 &&
               planner_private_namespace_.back() == '/') {
            planner_private_namespace_.pop_back();
        }
        patrol_path_spacing_ = std::max(0.005, std::fabs(patrol_path_spacing_));
        patrol_speed_limit_ = std::max(0.05, std::fabs(patrol_speed_limit_));
        normal_navigation_speed_limit_ =
            std::max(0.05, std::fabs(normal_navigation_speed_limit_));
        patrol_cancel_timeout_ = std::max(0.5, patrol_cancel_timeout_);
        patrol_interface_timeout_ = std::max(0.5, patrol_interface_timeout_);
        normal_move_base_oscillation_timeout_ =
            std::max(0.0, normal_move_base_oscillation_timeout_);
        patrol_aborted_retry_count_ =
            std::max(0, patrol_aborted_retry_count_);
        patrol_transition_position_kp_ =
            std::fabs(patrol_transition_position_kp_);
        patrol_transition_yaw_kp_ =
            std::fabs(patrol_transition_yaw_kp_);
        patrol_transition_min_linear_speed_ =
            std::fabs(patrol_transition_min_linear_speed_);
        patrol_transition_max_linear_speed_ = std::max(
            std::fabs(patrol_transition_max_linear_speed_),
            patrol_transition_min_linear_speed_);
        patrol_transition_min_angular_speed_ =
            std::fabs(patrol_transition_min_angular_speed_);
        patrol_transition_max_angular_speed_ = std::max(
            std::fabs(patrol_transition_max_angular_speed_),
            patrol_transition_min_angular_speed_);
        patrol_transition_linear_accel_ =
            std::max(0.05, std::fabs(patrol_transition_linear_accel_));
        patrol_transition_angular_accel_ =
            std::max(0.05, std::fabs(patrol_transition_angular_accel_));
        patrol_transition_yaw_priority_start_deg_ =
            std::max(5.0, std::fabs(patrol_transition_yaw_priority_start_deg_));
        patrol_transition_yaw_priority_release_deg_ = clampValue(
            std::fabs(patrol_transition_yaw_priority_release_deg_),
            0.0, patrol_transition_yaw_priority_start_deg_ - 1.0);
        patrol_transition_yaw_priority_min_linear_scale_ = clampValue(
            patrol_transition_yaw_priority_min_linear_scale_, 0.0, 1.0);
        patrol_transition_position_tolerance_ =
            std::max(0.003, std::fabs(patrol_transition_position_tolerance_));
        patrol_transition_yaw_tolerance_deg_ =
            std::max(0.2, std::fabs(patrol_transition_yaw_tolerance_deg_));
        patrol_transition_stable_frames_ =
            std::max(1, patrol_transition_stable_frames_);
        patrol_transition_timeout_ =
            std::max(1.0, patrol_transition_timeout_);
        patrol_stop_max_center_x_ = clampValue(
            patrol_stop_max_center_x_, 1.0,
            std::max(1.0, static_cast<double>(image_width_)));
        patrol_target_stop_distance_ =
            std::max(0.05, std::fabs(patrol_target_stop_distance_));
        patrol_target_slowdown_start_distance_ =
            std::max(patrol_target_stop_distance_ + 0.05,
                     std::fabs(patrol_target_slowdown_start_distance_));
        patrol_target_min_speed_ratio_ =
            clampValue(patrol_target_min_speed_ratio_, 0.05, 1.0);
        patrol_noncurrent_wall_stop_max_distance_ =
            std::max(0.05,
                     std::fabs(patrol_noncurrent_wall_stop_max_distance_));
        patrol_target_left_edge_stop_px_ =
            std::max(1,
                     std::min(patrol_target_left_edge_stop_px_,
                              std::max(1, image_width_)));
        duplicate_coordinate_distance_ =
            std::fabs(duplicate_coordinate_distance_);
        ocr_recovery_turn_deg_ = clampValue(
            std::fabs(ocr_recovery_turn_deg_), 1.0, 180.0);
        ocr_recovery_turn_kp_ = std::fabs(ocr_recovery_turn_kp_);
        ocr_recovery_turn_min_speed_ =
            std::fabs(ocr_recovery_turn_min_speed_);
        ocr_recovery_turn_max_speed_ = std::max(
            std::fabs(ocr_recovery_turn_max_speed_),
            ocr_recovery_turn_min_speed_);
        ocr_recovery_turn_tolerance_deg_ = std::max(
            0.2, std::fabs(ocr_recovery_turn_tolerance_deg_));
        ocr_recovery_turn_stable_frames_ =
            std::max(1, ocr_recovery_turn_stable_frames_);
        ocr_recovery_turn_timeout_ =
            std::max(1.0, ocr_recovery_turn_timeout_);
        ocr_recovery_settle_time_ =
            std::max(0.0, ocr_recovery_settle_time_);
        docking_recovery_turn_deg_ = clampValue(
            std::fabs(docking_recovery_turn_deg_), 1.0, 90.0);
        docking_recovery_detection_attempts_ =
            std::max(1, docking_recovery_detection_attempts_);
        docking_recovery_detection_interval_ =
            std::max(0.0, docking_recovery_detection_interval_);
        docking_refine_max_board_shift_ =
            std::max(0.10, std::fabs(docking_refine_max_board_shift_));
        docking_refresh_clear_calls_ =
            std::max(1, docking_refresh_clear_calls_);

        corner_obstacle_check_radius_ =
            std::max(
                0.005,
                std::fabs(corner_obstacle_check_radius_));
        corner_obstacle_lethal_cost_threshold_ =
            std::max(
                1,
                std::min(
                    100,
                    corner_obstacle_lethal_cost_threshold_));
        corner_obstacle_costmap_max_age_ =
            std::max(
                0.2,
                corner_obstacle_costmap_max_age_);

        patrol_near_end_corner_shortcut_distance_ =
            std::max(
                0.0,
                patrol_near_end_corner_shortcut_distance_);

        patrol_noncurrent_target_early_dock_distance_ =
            std::max(
                0.0,
                patrol_noncurrent_target_early_dock_distance_);

        camera_fx_ = std::fabs(camera_fx_);
        docking_standoff_ = std::fabs(docking_standoff_);
        // V16复用patrol_transition_*速度策略。
        // 最终接管阈值至少不能小于转角位置容差。
        patrol_resume_max_cross_track_error_ =
            std::max(
                patrol_transition_position_tolerance_,
                std::fabs(patrol_resume_max_cross_track_error_));
    }

    void buildSegments() {
        segments_.clear();
        // V14.6：首段直接从(1.25,4.25)发布；首段终点及之后三个
        // V13安全角点全部保持原值。后续段仍由runPatrolPoseTransition()
        // 同时完成安全小平移和90度转向，再启动下一条固定Path。
        // V14.6：第一条固定Patrol Path直接从新的巡检准备点发布。
        // 第一段终点和后续V13安全角点全部保持原值。
        addSegment("上墙巡检", 1.25, 4.25, 4.75, 4.30,
                   0.0, 0.5 * kPi, WALL_TOP);
        addSegment("右墙巡检", 4.80, 4.30, 4.80, 2.75,
                   -0.5 * kPi, 0.0, WALL_RIGHT);
        addSegment("下墙巡检", 4.80, 2.70, 0.25, 2.70,
                   -kPi, -0.5 * kPi, WALL_BOTTOM);
        addSegment("左墙巡检", 0.20, 2.70, 0.20, 4.25,
                   0.5 * kPi, kPi, WALL_LEFT);
    }

    void addSegment(const std::string& name,
                    double start_x, double start_y,
                    double end_x, double end_y,
                    double travel_yaw, double docking_yaw,
                    WallType wall) {
        Segment segment;
        segment.name = name;
        segment.start_x = start_x;
        segment.start_y = start_y;
        segment.end_x = end_x;
        segment.end_y = end_y;
        segment.length = distance2D(start_x, start_y, end_x, end_y);
        segment.dir_x = (end_x - start_x) / segment.length;
        segment.dir_y = (end_y - start_y) / segment.length;
        segment.travel_yaw = travel_yaw;
        segment.docking_yaw = docking_yaw;
        segment.wall = wall;
        segments_.push_back(segment);
    }

    bool validateConfiguration() {
        if (!isValidCategory(real_target_category_) ||
            !isValidCategory(simulation_target_category_)) {
            ROS_ERROR("real_target_category和simulation_target_category只允许"
                      "food、daily、electronic");
            return false;
        }
        if (real_target_category_ == simulation_target_category_) {
            ROS_ERROR("现实目标和仿真目标不能设置为同一类别");
            return false;
        }
        if (room_max_x_ <= room_min_x_ || room_max_y_ <= room_min_y_) {
            ROS_ERROR("房间坐标边界无效");
            return false;
        }
        if (max_detection_duration_ <= 0.0 ||
            planner_private_namespace_.empty() ||
            camera_fx_ <= 0.0 ||
            docking_standoff_ <= 0.0 ||
            duplicate_coordinate_distance_ <= 0.0 ||
            approach_stop_distance_ <= 0.0 ||
            patrol_transition_position_kp_ <= 0.0 ||
            patrol_transition_yaw_kp_ <= 0.0 ||
            patrol_transition_max_linear_speed_ <= 0.0 ||
            patrol_transition_max_angular_speed_ <= 0.0 ||
            patrol_resume_max_cross_track_error_ <
                patrol_transition_position_tolerance_) {
            ROS_ERROR("视觉、重复判定、V16安全回巡检或雷达逼近参数无效");
            return false;
        }
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            if (!isInsideRoom(segments_[i].start_x, segments_[i].start_y) ||
                !isInsideRoom(segments_[i].end_x, segments_[i].end_y)) {
                ROS_ERROR("第%zu段巡检端点超出房间坐标边界", i + 1);
                return false;
            }
        }
        return true;
    }

    bool waitForDependencies() {
        ROS_INFO("等待move_base、NanoDet、OCR、运动控制和固定巡检路径接口...");
        while (ros::ok() && !move_base_.waitForServer(ros::Duration(3.0))) {
            ROS_INFO("仍在等待move_base");
        }
        if (!ros::ok()) return false;
        cacheMoveBaseOscillationTimeout();
        if (disable_move_base_oscillation_during_patrol_ &&
            !move_base_reconfigure_client_.waitForExistence(
                ros::Duration(5.0))) {
            ROS_WARN("未找到%s；巡检仍可运行，但无法自动关闭move_base振荡监视。",
                     move_base_reconfigure_service_.c_str());
        }
        if (!ros::service::waitForService("/nanodet_detect",
                                          ros::Duration(20.0))) {
            ROS_ERROR("等待/nanodet_detect超时");
            return false;
        }
        if (!ros::service::waitForService("/nanodet_ocr",
                                          ros::Duration(20.0))) {
            ROS_ERROR("等待/nanodet_ocr超时");
            return false;
        }
        if (!ros::service::waitForService("/set_speed",
                                          ros::Duration(20.0))) {
            ROS_ERROR("等待/set_speed超时");
            return false;
        }
        if (!patrol_path_lock_client_.waitForExistence(ros::Duration(20.0))) {
            ROS_ERROR("等待%s/lock_patrol_path超时",
                      planner_private_namespace_.c_str());
            return false;
        }
        if (!controller_reset_client_.waitForExistence(ros::Duration(20.0))) {
            ROS_ERROR("等待%s/reset_controller_state超时",
                      planner_private_namespace_.c_str());
            return false;
        }
        const ros::WallTime subscriber_deadline =
            ros::WallTime::now() + ros::WallDuration(5.0);
        while (ros::ok() && patrol_path_publisher_.getNumSubscribers() == 0 &&
               ros::WallTime::now() < subscriber_deadline) {
            ros::Duration(0.05).sleep();
        }
        if (patrol_path_publisher_.getNumSubscribers() == 0) {
            ROS_ERROR("固定巡检路径话题%s/patrol_path没有订阅者",
                      planner_private_namespace_.c_str());
            return false;
        }
        return true;
    }

    bool getRobotPose(Pose2D& pose) {
        try {
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                           ros::Duration(0.20));
            pose.x = transform.transform.translation.x;
            pose.y = transform.transform.translation.y;
            pose.yaw = tf2::getYaw(transform.transform.rotation);
            return true;
        } catch (const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(1.0, "读取机器人map位姿失败：%s", error.what());
            return false;
        }
    }

    bool isInsideRoom(double x, double y) const {
        const double epsilon = 1e-9;
        return x >= room_min_x_ - epsilon &&
               x <= room_max_x_ + epsilon &&
               y >= room_min_y_ - epsilon &&
               y <= room_max_y_ + epsilon;
    }

    void clampToRoom(double& x, double& y) const {
        x = clampValue(x, room_min_x_, room_max_x_);
        y = clampValue(y, room_min_y_, room_max_y_);
    }

    std::string plannerParameter(const std::string& name) const {
        return planner_private_namespace_ + "/" + name;
    }

    void cacheMoveBaseOscillationTimeout() {
        if (move_base_oscillation_timeout_cached_) return;

        double configured_timeout = normal_move_base_oscillation_timeout_;
        if (ros::param::get("/move_base/oscillation_timeout",
                            configured_timeout)) {
            normal_move_base_oscillation_timeout_ =
                std::max(0.0, configured_timeout);
        }
        move_base_oscillation_timeout_cached_ = true;
        ROS_INFO("已记录普通导航move_base振荡超时：%.2fs。",
                 normal_move_base_oscillation_timeout_);
    }

    bool setMoveBaseOscillationTimeout(double timeout) {
        dynamic_reconfigure::Reconfigure service;
        dynamic_reconfigure::DoubleParameter parameter;
        parameter.name = "oscillation_timeout";
        parameter.value = std::max(0.0, timeout);
        service.request.config.doubles.push_back(parameter);

        if (!move_base_reconfigure_client_.call(service)) {
            ROS_ERROR("调用%s设置oscillation_timeout=%.2f失败。",
                      move_base_reconfigure_service_.c_str(),
                      parameter.value);
            return false;
        }
        return true;
    }

    void setMoveBasePatrolOscillationGuard(bool patrol_mode) {
        if (!disable_move_base_oscillation_during_patrol_) return;
        if (patrol_mode == move_base_patrol_oscillation_guard_active_) return;

        cacheMoveBaseOscillationTimeout();
        const double timeout = patrol_mode
            ? 0.0
            : normal_move_base_oscillation_timeout_;
        if (!setMoveBaseOscillationTimeout(timeout)) {
            ROS_WARN("未能%smove_base振荡监视；巡检Action仍启用有限自动续跑保护。",
                     patrol_mode ? "关闭" : "恢复");
            return;
        }

        move_base_patrol_oscillation_guard_active_ = patrol_mode;
        if (patrol_mode) {
            ROS_WARN("固定巡检期间已将move_base oscillation_timeout临时设为0："
                     "允许原地换向及连续小幅横移，不再误判振荡。" );
        } else {
            ROS_WARN("已恢复普通导航move_base oscillation_timeout=%.2fs。",
                     normal_move_base_oscillation_timeout_);
        }
    }

    void setPlannerFourSpeedLimits(double speed) {
        ros::param::set(plannerParameter("c2_max_reference_speed"), speed);
        ros::param::set(plannerParameter("mpc_max_vx"), speed);
        ros::param::set(
            plannerParameter("mpc_max_translational_speed"), speed);
        ros::param::set(plannerParameter("max_vel_x"), speed);
    }

    void setPatrolRuntimeSpeedLimit(double speed, bool force = false) {
        const double minimum_speed =
            patrol_speed_limit_ * patrol_target_min_speed_ratio_;
        speed = clampValue(speed, minimum_speed, patrol_speed_limit_);

        if (!force &&
            std::isfinite(current_patrol_runtime_speed_limit_) &&
            std::fabs(speed - current_patrol_runtime_speed_limit_) < 0.01) {
            return;
        }

        setPlannerFourSpeedLimits(speed);
        current_patrol_runtime_speed_limit_ = speed;
    }

    void restorePatrolCruiseSpeedIfNeeded() {
        if (!patrol_goal_active_) return;
        setPatrolRuntimeSpeedLimit(patrol_speed_limit_);
    }

    void setPlannerRuntimeParameters(bool patrol_mode) {
        const double speed = patrol_mode
                                 ? patrol_speed_limit_
                                 : normal_navigation_speed_limit_;

        setPlannerFourSpeedLimits(speed);
        current_patrol_runtime_speed_limit_ =
            patrol_mode
                ? speed
                : std::numeric_limits<double>::quiet_NaN();

        ros::param::set(
            plannerParameter("enable_path_replanning"), !patrol_mode);
        setMoveBasePatrolOscillationGuard(patrol_mode);
        ROS_WARN("局部规划器已切换为%s参数：四项速度=%.2f，"
                 "enable_path_replanning=%s。",
                 patrol_mode ? "巡检" : "普通导航",
                 speed, patrol_mode ? "false" : "true");
    }

    void setShadowModeActiveOnce() {
        if (shadow_mode_has_been_disabled_) return;
        ros::param::set(
            plannerParameter("clearance_optimizer/shadow_mode"), false);
        shadow_mode_has_been_disabled_ = true;
        ROS_WARN("已到达第一条路线起点：shadow_mode=false；"
                 "本次任务后续不再修改该参数。");
    }

    bool requestPatrolPathLock(bool lock_path) {
        std_srvs::SetBool service;
        service.request.data = lock_path;
        if (!patrol_path_lock_client_.call(service)) {
            ROS_ERROR("调用%s/lock_patrol_path失败",
                      planner_private_namespace_.c_str());
            return false;
        }
        if (!service.response.success) {
            ROS_WARN("%s固定巡检路径暂未成功：%s",
                     lock_path ? "锁定" : "解除",
                     service.response.message.c_str());
            return false;
        }
        patrol_path_locked_ = lock_path;
        return true;
    }

    bool resetPlannerControllerState() {
        std_srvs::Trigger service;
        if (!controller_reset_client_.call(service) ||
            !service.response.success) {
            ROS_ERROR("复位局部规划器控制状态失败：%s",
                      service.response.message.c_str());
            return false;
        }
        ROS_INFO("局部规划器控制状态已复位：%s",
                 service.response.message.c_str());
        return true;
    }

    bool cancelPatrolGoalAndWait() {
        if (!patrol_goal_active_) return true;

        move_base_.cancelGoal();
        const bool terminal = move_base_.waitForResult(
            ros::Duration(patrol_cancel_timeout_));
        const std::string state = move_base_.getState().toString();
        patrol_goal_active_ = false;
        stopRobot();

        if (!terminal) {
            ROS_ERROR("取消巡检move_base目标超时，当前状态=%s",
                      state.c_str());
            return false;
        }
        ROS_INFO("巡检move_base目标已停止：%s", state.c_str());
        return true;
    }

    void finishPatrolMode() {
        if (patrol_goal_active_) {
            cancelPatrolGoalAndWait();
        }
        if (patrol_path_locked_) {
            requestPatrolPathLock(false);
        }
        setPlannerRuntimeParameters(false);
    }

    bool prepareNormalNavigation() {
        if (patrol_goal_active_ && !cancelPatrolGoalAndWait()) {
            return false;
        }
        if (patrol_path_locked_ && !requestPatrolPathLock(false)) {
            return false;
        }
        setPlannerRuntimeParameters(false);
        return resetPlannerControllerState();
    }

    nav_msgs::Path buildRemainingPatrolPath(
            const Segment& segment,
            const Pose2D& pose,
            double& start_progress,
            double& remaining_distance) {
        start_progress = clampValue(
            segmentProgress(segment, pose), 0.0, segment.length);
        if (patrol_checkpoint_.valid &&
            patrol_checkpoint_.segment_index == current_segment_index_) {
            // 停靠导航可能使车辆沿巡检线方向产生少量回退。剩余路线的
            // 起点不得退到停车断点之前，避免重复巡检已经扫过的区域。
            start_progress = std::max(
                start_progress,
                clampValue(patrol_checkpoint_.stopped_progress,
                           0.0, segment.length));
        }
        remaining_distance = segment.length - start_progress;

        nav_msgs::Path path;
        path.header.frame_id = map_frame_;
        path.header.stamp = ros::Time::now();
        path.header.seq = ++patrol_path_sequence_;

        if (remaining_distance <= segment_end_tolerance_) {
            return path;
        }

        const int intervals = std::max(
            1, static_cast<int>(
                   std::ceil(remaining_distance / patrol_path_spacing_)));
        path.poses.reserve(static_cast<std::size_t>(intervals + 1));
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, segment.travel_yaw);

        for (int i = 0; i <= intervals; ++i) {
            const double ratio =
                static_cast<double>(i) / static_cast<double>(intervals);
            const double progress =
                start_progress + ratio * remaining_distance;
            geometry_msgs::PoseStamped point;
            point.header = path.header;
            point.pose.position.x =
                segment.start_x + progress * segment.dir_x;
            point.pose.position.y =
                segment.start_y + progress * segment.dir_y;
            point.pose.orientation = tf2::toMsg(quaternion);
            path.poses.push_back(point);
        }
        return path;
    }

    PatrolStartResult startPatrolFromPose(
            const Segment& segment,
            const Pose2D& pose) {
        double start_progress = 0.0;
        double remaining_distance = 0.0;
        nav_msgs::Path path = buildRemainingPatrolPath(
            segment, pose, start_progress, remaining_distance);
        if (remaining_distance <= segment_end_tolerance_) {
            ROS_INFO("%s剩余距离仅%.3fm，直接判定本段完成。",
                     segment.name.c_str(), remaining_distance);
            return PATROL_ALREADY_COMPLETE;
        }

        // 每次开始或断点恢复都先解除旧锁。规划器解除锁时会清空旧的
        // 暂存路线，因此随后SetBool(true)成功必然对应本次新发布的Path。
        if (patrol_path_locked_ && !requestPatrolPathLock(false)) {
            return PATROL_START_FAILED;
        }
        if (!resetPlannerControllerState()) {
            return PATROL_START_FAILED;
        }
        setPlannerRuntimeParameters(true);

        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(patrol_interface_timeout_);
        bool locked = false;
        do {
            path.header.stamp = ros::Time::now();
            for (std::size_t i = 0; i < path.poses.size(); ++i) {
                path.poses[i].header = path.header;
            }
            patrol_path_publisher_.publish(path);
            ros::Duration(0.08).sleep();
            locked = requestPatrolPathLock(true);
            if (!locked) ros::Duration(0.08).sleep();
        } while (ros::ok() && !locked &&
                 ros::WallTime::now() < deadline);

        if (!locked) {
            setPlannerRuntimeParameters(false);
            ROS_ERROR("%s固定路线未能在%.1f秒内被局部规划器确认",
                      segment.name.c_str(), patrol_interface_timeout_);
            return PATROL_START_FAILED;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = path.poses.back();
        goal.target_pose.header.stamp = ros::Time::now();
        move_base_.sendGoal(goal);
        patrol_goal_active_ = true;
        patrol_checkpoint_.valid = false;
        ROS_INFO("%s已异步启动：断点进度=%.3f/%.3fm，剩余=%.3fm，"
                 "固定路径点数=%zu，终点=(%.3f, %.3f, %.1f度)。",
                 segment.name.c_str(), start_progress, segment.length,
                 remaining_distance, path.poses.size(),
                 goal.target_pose.pose.position.x,
                 goal.target_pose.pose.position.y,
                 segment.travel_yaw * 180.0 / kPi);
        return PATROL_STARTED;
    }

    void releaseCameraBeforePredockNavigation(
            const std::string& target_name) {
        if (!camera_opened_) {
            ROS_INFO(
                "[停靠相机] %s准备第一段move_base：摄像头当前已关闭，"
                "无需额外处理。",
                target_name.c_str());
            return;
        }

        // 这是V14.2处理缓存的核心：
        // 与其在第一段到点后等待、grab、丢帧，不如在离开巡检时直接
        // release VideoCapture。第一段行驶期间没有相机会话，自然不会
        // 积累任何旧帧；到点后重新open就是全新的V4L2缓冲。
        ROS_WARN(
            "[停靠相机] %s准备第一段move_base：立即关闭NanoDet摄像头，"
            "彻底释放当前V4L2缓存；第一段到点后再即时重开。",
            target_name.c_str());
        closeCamera();
    }

    bool navigateToPose(double x, double y, double yaw,
                        const std::string& purpose) {
        clampToRoom(x, y);
        if (!isInsideRoom(x, y)) {
            ROS_ERROR("%s目标(%.3f, %.3f, %.1f度)超出房间坐标边界",
                      purpose.c_str(), x, y, yaw * 180.0 / kPi);
            return false;
        }

        if (!prepareNormalNavigation()) return false;
        stopRobot();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(quaternion);

        ROS_INFO("%s：发送move_base目标(%.3f, %.3f, %.1f度)",
                 purpose.c_str(), x, y, yaw * 180.0 / kPi);
        move_base_.sendGoal(goal);
        if (!move_base_.waitForResult(ros::Duration(navigation_timeout_))) {
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("%s超时", purpose.c_str());
            return false;
        }
        if (move_base_.getState() !=
            actionlib::SimpleClientGoalState::SUCCEEDED) {
            const std::string state = move_base_.getState().toString();
            move_base_.cancelGoal();
            stopRobot();
            ROS_ERROR("%s失败：%s", purpose.c_str(), state.c_str());
            return false;
        }

        stopRobot();
        return true;
    }

    double segmentProgress(const Segment& segment, const Pose2D& pose) const {
        return (pose.x - segment.start_x) * segment.dir_x +
               (pose.y - segment.start_y) * segment.dir_y;
    }

    static const char* wallName(WallType wall) {
        switch (wall) {
            case WALL_LEFT: return "左墙";
            case WALL_RIGHT: return "右墙";
            case WALL_BOTTOM: return "下墙";
            case WALL_TOP: return "上墙";
        }
        return "未知墙面";
    }

    bool estimateBoardBoundary(const Pose2D& robot_pose,
                               const Box& box,
                               BoardBoundaryEstimate& estimate) const {
        estimate = BoardBoundaryEstimate();
        const double image_center = 0.5 * static_cast<double>(image_width_);
        // 图像右侧对应机器人右侧，因此像素向右时相对航向角为负。
        const double relative_yaw =
            std::atan2(image_center - box.centerX(), camera_fx_);
        const double ray_yaw =
            robot_pose.yaw +
            camera_yaw_offset_deg_ * kPi / 180.0 +
            relative_yaw;
        const double ray_x = std::cos(ray_yaw);
        const double ray_y = std::sin(ray_yaw);
        double nearest_t = std::numeric_limits<double>::infinity();
        BoardBoundaryEstimate nearest;
        const double epsilon = 1e-6;

        // 扩大到左侧三分之二画面后，转角处可能同时看到当前左墙和
        // 前方相邻墙。不能再强制把目标投影到segment.wall，而应取
        // 相机射线与矩形场地四条边界的最近正向交点。
        const auto consider =
            [&](WallType wall, double t, double x, double y) {
                if (!std::isfinite(t) || t <= epsilon ||
                    t >= nearest_t) {
                    return;
                }
                if (x < room_min_x_ - epsilon ||
                    x > room_max_x_ + epsilon ||
                    y < room_min_y_ - epsilon ||
                    y > room_max_y_ + epsilon) {
                    return;
                }
                nearest_t = t;
                nearest.valid = true;
                nearest.wall = wall;
                nearest.x = clampValue(x, room_min_x_, room_max_x_);
                nearest.y = clampValue(y, room_min_y_, room_max_y_);
            };

        if (std::fabs(ray_x) >= epsilon) {
            double t = (room_min_x_ - robot_pose.x) / ray_x;
            consider(WALL_LEFT, t, room_min_x_,
                     robot_pose.y + t * ray_y);
            t = (room_max_x_ - robot_pose.x) / ray_x;
            consider(WALL_RIGHT, t, room_max_x_,
                     robot_pose.y + t * ray_y);
        }
        if (std::fabs(ray_y) >= epsilon) {
            double t = (room_min_y_ - robot_pose.y) / ray_y;
            consider(WALL_BOTTOM, t,
                     robot_pose.x + t * ray_x, room_min_y_);
            t = (room_max_y_ - robot_pose.y) / ray_y;
            consider(WALL_TOP, t,
                     robot_pose.x + t * ray_x, room_max_y_);
        }

        if (!nearest.valid) return false;
        estimate = nearest;
        return true;
    }

    int segmentIndexForWall(WallType wall) const {
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            if (segments_[i].wall == wall) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool makeDockingObservationAtStandoff(
            int segment_index,
            const std::string& category,
            const BoardBoundaryEstimate& board_estimate,
            double standoff,
            TargetObservation& observation) const {
        observation = TargetObservation();
        if (!board_estimate.valid || standoff <= 0.0) return false;

        observation.valid = true;
        observation.board_valid = true;
        observation.board_wall = board_estimate.wall;
        observation.board_x = board_estimate.x;
        observation.board_y = board_estimate.y;
        observation.pose.x = board_estimate.x;
        observation.pose.y = board_estimate.y;
        switch (board_estimate.wall) {
            case WALL_TOP:
                observation.pose.y = room_max_y_ - standoff;
                observation.pose.yaw = 0.5 * kPi;
                break;
            case WALL_RIGHT:
                observation.pose.x = room_max_x_ - standoff;
                observation.pose.yaw = 0.0;
                break;
            case WALL_BOTTOM:
                observation.pose.y = room_min_y_ + standoff;
                observation.pose.yaw = -0.5 * kPi;
                break;
            case WALL_LEFT:
                observation.pose.x = room_min_x_ + standoff;
                observation.pose.yaw = kPi;
                break;
        }
        if (!isInsideRoom(observation.pose.x, observation.pose.y)) {
            return false;
        }
        observation.segment_index = segment_index;
        observation.category = category;
        return true;
    }

    bool makeDockingObservation(
            int segment_index,
            const std::string& category,
            const BoardBoundaryEstimate& board_estimate,
            TargetObservation& observation) const {
        return makeDockingObservationAtStandoff(
            segment_index, category, board_estimate,
            docking_standoff_, observation);
    }

    bool isDuplicateBoard(const BoardBoundaryEstimate& estimate) const {
        if (!estimate.valid) return false;
        for (std::size_t i = 0; i < seen_board_coordinates_.size(); ++i) {
            const BoardBoundaryEstimate& previous =
                seen_board_coordinates_[i];
            if (!previous.valid || previous.wall != estimate.wall) continue;
            if (distance2D(previous.x, previous.y,
                           estimate.x, estimate.y) <=
                duplicate_coordinate_distance_) {
                return true;
            }
        }
        return false;
    }

    int findUnknownCandidate(
            const BoardBoundaryEstimate& estimate) const {
        if (!estimate.valid) return -1;

        for (std::size_t i = 0;
             i < unknown_candidates_.size();
             ++i) {
            const UnknownCandidate& candidate =
                unknown_candidates_[i];

            if (!candidate.board.valid ||
                candidate.board.wall != estimate.wall) {
                continue;
            }

            if (distance2D(
                    candidate.board.x,
                    candidate.board.y,
                    estimate.x,
                    estimate.y) <=
                duplicate_coordinate_distance_) {
                return static_cast<int>(i);
            }
        }

        return -1;
    }

    void addUnknownCandidate(
            std::size_t segment_index,
            const Pose2D& pose,
            const Box& reference_box,
            const BoardBoundaryEstimate& fallback_estimate,
            const std::string& reason) {
        BoardBoundaryEstimate estimate;

        // 优先使用OCR最终参考框与“当前最终静止位姿”重新投影。
        // 若该框无法求墙面交点，再退回最初触发停车时的墙面估计。
        if (!estimateBoardBoundary(
                pose,
                reference_box,
                estimate)) {
            estimate = fallback_estimate;
        }

        if (!estimate.valid) {
            ROS_WARN(
                "V14.9候选记录失败：%s；"
                "当前OCR仍unknown，但没有有效墙面坐标，无法生成候选导航点。",
                reason.c_str());
            return;
        }

        const int existing =
            findUnknownCandidate(estimate);
        if (existing >= 0) {
            ROS_INFO(
                "V14.9候选去重：%s(%.3f,%.3f)与candidate[%d]"
                "距离<=%.2fm，不重复加入。",
                wallName(estimate.wall),
                estimate.x,
                estimate.y,
                existing + 1,
                duplicate_coordinate_distance_);
            return;
        }

        UnknownCandidate candidate;
        candidate.board = estimate;
        candidate.source_segment_index =
            static_cast<int>(segment_index);
        candidate.source_pose = pose;
        candidate.source_box = reference_box;

        unknown_candidates_.push_back(candidate);

        ROS_WARN(
            "V14.9新增unknown候选[%zu]：%s(%.3f,%.3f)，"
            "来源巡检段=%zu，原因=%s。"
            "当前不前往候选点；只有整圈巡检结束且任务未完成时才回访。",
            unknown_candidates_.size(),
            wallName(estimate.wall),
            estimate.x,
            estimate.y,
            segment_index + 1,
            reason.c_str());
    }

    void resolveUnknownCandidateNear(
            const BoardBoundaryEstimate& estimate,
            const std::string& reason) {
        const int index =
            findUnknownCandidate(estimate);
        if (index < 0) return;

        UnknownCandidate& candidate =
            unknown_candidates_[
                static_cast<std::size_t>(index)];
        if (candidate.resolved) return;

        candidate.resolved = true;
        ROS_INFO(
            "V14.9候选[%d]已解决：%s(%.3f,%.3f)，%s。",
            index + 1,
            wallName(candidate.board.wall),
            candidate.board.x,
            candidate.board.y,
            reason.c_str());
    }

    bool recognizeUnknownCandidate(
            const TargetObservation& candidate_observation,
            Box& matched_box,
            BoardBoundaryEstimate& matched_board,
            OcrRecord& ocr) {
        TargetObservation unused;

        if (!detectDockingRefinedObservation(
                candidate_observation,
                "unknown候选回访视觉匹配",
                docking_standoff_,
                unused,
                false,
                &matched_box,
                &matched_board,
                true)) {
            return false;
        }

        ocr = recognizeStaticTarget(matched_box);

        // 保留主巡检原有“只读到车间通用残片时逆时针30度再识别一次”。
        // 候选点此时已经在0.5m正面附近，这一步只是最终OCR兜底。
        if (ocr.category == "unknown" &&
            hasWorkshopFragment(ocr.text)) {
            ROS_WARN(
                "V14.9候选回访只识别到通用残片“%s”；"
                "沿用原OCR补偿机制，逆时针转%.1f度后再识别一次。",
                ocr.text.c_str(),
                ocr_recovery_turn_deg_);

            Pose2D recovery_pose;
            if (rotateCounterClockwiseForOcr(
                    recovery_pose)) {
                Box retry_box{0, 0, 0, 0, 0};
                BoardBoundaryEstimate retry_board;
                TargetObservation retry_unused;

                if (detectDockingRefinedObservation(
                        candidate_observation,
                        "unknown候选OCR补偿后匹配",
                        docking_standoff_,
                        retry_unused,
                        true,
                        &retry_box,
                        &retry_board,
                        true)) {
                    matched_box = retry_box;
                    matched_board = retry_board;
                    ocr = recognizeStaticTarget(
                        retry_box);
                }
            }
        }

        return true;
    }

    bool revisitUnknownCandidates() {
        // 已经完成整圈巡检，任何旧的“非当前墙必须等当前段结束”保护
        // 在这里都不再有意义。
        simulation_target_blocked_until_segment_end_ = false;

        for (std::size_t i = 0;
             i < unknown_candidates_.size() &&
             ros::ok();
             ++i) {
            if (real_docked_ &&
                simulation_docked_) {
                ROS_INFO(
                    "V14.9双目标已完成，停止剩余候选回访。");
                return true;
            }

            UnknownCandidate& candidate =
                unknown_candidates_[i];

            if (candidate.resolved) {
                continue;
            }

            candidate.attempted = true;

            const int segment_index =
                segmentIndexForWall(
                    candidate.board.wall);
            TargetObservation predock;

            if (segment_index < 0 ||
                !makeDockingObservation(
                    segment_index,
                    "unknown",
                    candidate.board,
                    predock)) {
                ROS_WARN(
                    "V14.9候选[%zu]无法生成距墙%.2fm临时停靠点，"
                    "跳过该候选。",
                    i + 1,
                    docking_standoff_);
                continue;
            }

            closeCamera();

            ROS_WARN(
                "V14.9回访候选[%zu/%zu]：板=%s(%.3f,%.3f)，"
                "导航到临时停靠点=(%.3f,%.3f,%.1f度)。",
                i + 1,
                unknown_candidates_.size(),
                wallName(candidate.board.wall),
                candidate.board.x,
                candidate.board.y,
                predock.pose.x,
                predock.pose.y,
                predock.pose.yaw * 180.0 / kPi);

            if (!navigateToPose(
                    predock.pose.x,
                    predock.pose.y,
                    predock.pose.yaw,
                    "前往unknown候选临时停靠点")) {
                ROS_WARN(
                    "V14.9候选[%zu]导航失败，继续下一个候选。",
                    i + 1);
                continue;
            }

            if (!openCamera()) {
                return false;
            }

            Box matched_box{0, 0, 0, 0, 0};
            BoardBoundaryEstimate matched_board;
            OcrRecord ocr;

            if (!recognizeUnknownCandidate(
                    predock,
                    matched_box,
                    matched_board,
                    ocr)) {
                ROS_WARN(
                    "V14.9候选[%zu]到点后未重新找到同一目标板，"
                    "继续下一个候选。",
                    i + 1);
                closeCamera();
                continue;
            }

            if (ocr.category == "unknown") {
                ROS_WARN(
                    "V14.9候选[%zu]已到正面临时停靠点，"
                    "但OCR仍为unknown：%s；继续下一个候选。",
                    i + 1,
                    ocr.text.c_str());
                closeCamera();
                continue;
            }

            Pose2D current_pose;
            BoardBoundaryEstimate classified_board =
                matched_board;

            if (getRobotPose(current_pose)) {
                BoardBoundaryEstimate ocr_board;
                if (estimateBoardBoundary(
                        current_pose,
                        ocr.box,
                        ocr_board)) {
                    classified_board =
                        ocr_board;
                }
            }

            candidate.resolved = true;

            // 成功分类后的板加入原seen名单（若尚未存在）。
            if (classified_board.valid &&
                !isDuplicateBoard(
                    classified_board)) {
                seen_board_coordinates_.push_back(
                    classified_board);
            }

            ROS_WARN(
                "V14.9候选[%zu]重新分类成功：文字=%s，类别=%s，"
                "墙面坐标=%s(%.3f,%.3f)。",
                i + 1,
                ocr.text.c_str(),
                categoryChinese(ocr.category),
                wallName(classified_board.wall),
                classified_board.x,
                classified_board.y);

            // 第三类非任务目标：候选已经被证明不是本轮两个任务目标，
            // 标记resolved后继续回访即可。
            if (ocr.category != real_target_category_ &&
                ocr.category != simulation_target_category_) {
                ROS_INFO(
                    "V14.9候选[%zu]分类为非任务类别%s，跳过停靠。",
                    i + 1,
                    categoryChinese(ocr.category));
                closeCamera();
                continue;
            }

            TargetObservation observation;
            const int classified_segment =
                segmentIndexForWall(
                    classified_board.wall);

            if (classified_segment < 0 ||
                !makeDockingObservation(
                    classified_segment,
                    ocr.category,
                    classified_board,
                    observation)) {
                ROS_ERROR(
                    "V14.9候选[%zu]虽然分类成功，但无法生成有效停靠点。",
                    i + 1);
                closeCamera();
                return false;
            }

            if (ocr.category ==
                simulation_target_category_) {
                if (!simulation_docked_) {
                    simulation_observation_ =
                        observation;
                    simulation_target_pending_ = true;
                    simulation_target_blocked_until_segment_end_ =
                        false;
                }

                if (!real_docked_) {
                    ROS_WARN(
                        "V14.9候选[%zu]确认是仿真目标，但现实目标尚未停靠；"
                        "先保存仿真目标，继续回访后续候选寻找现实目标。",
                        i + 1);
                    closeCamera();
                    continue;
                }

                closeCamera();
                if (!dockPendingSimulationTarget()) {
                    return false;
                }
                continue;
            }

            // 现实目标候选：整圈巡检已经结束，不再区分当前/非当前墙，
            // 直接按正常两段式停靠处理。
            if (ocr.category ==
                real_target_category_) {
                if (!real_docked_) {
                    real_observation_ =
                        observation;
                    real_target_pending_ = false;
                    real_target_defer_segment_index_ = -1;

                    releaseCameraBeforePredockNavigation(
                        "候选回访现实目标");

                    if (!navigateToPose(
                            observation.pose.x,
                            observation.pose.y,
                            observation.pose.yaw,
                            "前往候选回访现实目标临时停靠点")) {
                        return false;
                    }

                    if (!dockTarget(
                            real_observation_,
                            "候选回访现实目标",
                            false)) {
                        return false;
                    }

                    real_docked_ = true;
                    notifyRealDockedOnce();
                } else {
                    closeCamera();
                }

                // 若仿真目标此前已经正常巡检识别/候选回访识别并保存，
                // 现实目标一完成就立即补做仿真停靠。
                simulation_target_blocked_until_segment_end_ =
                    false;
                if (hasPendingSimulationTarget()) {
                    ROS_INFO(
                        "V14.9现实候选停靠完成，"
                        "立即处理此前已经保存的仿真目标。");
                    if (!dockPendingSimulationTarget()) {
                        return false;
                    }
                }
            }
        }

        // 防御性：候选循环末尾现实已完成，且仿真已经被保存但还没停。
        simulation_target_blocked_until_segment_end_ = false;
        if (hasPendingSimulationTarget()) {
            if (!dockPendingSimulationTarget()) {
                return false;
            }
        }

        return true;
    }

    double boardProgressOnSegment(
            const Segment& segment,
            const BoardBoundaryEstimate& board) const {
        return (board.x - segment.start_x) * segment.dir_x +
               (board.y - segment.start_y) * segment.dir_y;
    }

    double boardAheadProgress(
            const Segment& segment,
            const Pose2D& pose,
            const BoardBoundaryEstimate& board) const {
        return boardProgressOnSegment(segment, board) -
               segmentProgress(segment, pose);
    }

    double patrolTargetSpeedRatio(double distance_to_board) const {
        if (distance_to_board >=
            patrol_target_slowdown_start_distance_) {
            return 1.0;
        }
        if (distance_to_board <=
            patrol_target_stop_distance_) {
            return patrol_target_min_speed_ratio_;
        }

        const double span =
            patrol_target_slowdown_start_distance_ -
            patrol_target_stop_distance_;
        const double progress = clampValue(
            (distance_to_board - patrol_target_stop_distance_) / span,
            0.0, 1.0);

        // 与line2o障碍物减速同形：
        // stop处=min_ratio，slowdown_start处=1.0。
        return patrol_target_min_speed_ratio_ +
               (1.0 - patrol_target_min_speed_ratio_) * progress;
    }

    void clearPatrolVisualApproach(bool restore_speed) {
        patrol_visual_approach_ = PatrolVisualApproach();
        if (restore_speed) {
            restorePatrolCruiseSpeedIfNeeded();
        }
    }

    double distanceFromPoseToWall(
            const Pose2D& pose,
            WallType wall) const {
        switch (wall) {
            case WALL_LEFT:
                return std::fabs(pose.x - room_min_x_);
            case WALL_RIGHT:
                return std::fabs(room_max_x_ - pose.x);
            case WALL_BOTTOM:
                return std::fabs(pose.y - room_min_y_);
            case WALL_TOP:
                return std::fabs(room_max_y_ - pose.y);
        }
        return std::numeric_limits<double>::infinity();
    }

    // V14.7保护2：
    // 当前左侧正在巡检的墙上，如果框的左边界已经逼近图像左缘，
    // 说明继续依赖map距离有把目标直接开出视野的风险。
    // 该条件优先于1.5m->0.7m距离减速/停车条件。
    int chooseCurrentWallLeftEdgeEmergencyBox(
            const std::vector<Box>& boxes,
            const Pose2D& pose,
            const Segment& segment,
            BoardBoundaryEstimate& selected_estimate) const {
        int selected = -1;
        int best_x0 = std::numeric_limits<int>::max();

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() >=
                patrol_stop_max_center_x_) {
                continue;
            }

            // 严格“不足100px”：默认参数100时x0=99触发，x0=100不触发。
            if (boxes[i].x0 >=
                patrol_target_left_edge_stop_px_) {
                continue;
            }

            BoardBoundaryEstimate estimate;
            if (!estimateBoardBoundary(
                    pose, boxes[i], estimate)) {
                continue;
            }

            if (estimate.wall != segment.wall ||
                isDuplicateBoard(estimate)) {
                continue;
            }

            // 仍要求目标没有明显落到车辆后方。
            const double ahead =
                boardAheadProgress(
                    segment, pose, estimate);
            if (ahead < -0.03) {
                continue;
            }

            if (boxes[i].x0 < best_x0) {
                best_x0 = boxes[i].x0;
                selected = static_cast<int>(i);
                selected_estimate = estimate;
            }
        }

        return selected;
    }

    // V14.7保护1：
    // 非当前巡检墙不再“远远看见就停车”。
    // 已经能判断墙面时，必须机器人到该墙本身的垂直距离<1.5m才停车。
    // 射线无法判断墙面的框无法执行这层几何保护，保留旧安全逻辑立即停车。
    int chooseImmediateNonCurrentBoardBox(
            const std::vector<Box>& boxes,
            const Pose2D& pose,
            const Segment& segment) const {
        const double image_center =
            0.5 * static_cast<double>(image_width_);
        int selected = -1;
        double best_error =
            std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() >= patrol_stop_max_center_x_) {
                continue;
            }

            BoardBoundaryEstimate estimate;
            const bool estimate_ok =
                estimateBoardBoundary(
                    pose, boxes[i], estimate);

            if (estimate_ok && isDuplicateBoard(estimate)) {
                continue;
            }

            bool immediate = false;

            if (!estimate_ok) {
                // 无法判断属于哪面墙，保持V14.6之前的安全行为。
                immediate = true;
            } else if (estimate.wall != segment.wall) {
                const double wall_distance =
                    distanceFromPoseToWall(
                        pose, estimate.wall);

                if (wall_distance >=
                    patrol_noncurrent_wall_stop_max_distance_) {
                    ROS_INFO_THROTTLE(
                        0.8,
                        "V14.7非当前墙远距离保护：检测到%s目标框"
                        "center=(%.1f,%.1f)，但机器人距该墙=%.3fm>=%.3fm；"
                        "本帧只观察，不中断%s。",
                        wallName(estimate.wall),
                        boxes[i].centerX(),
                        boxes[i].centerY(),
                        wall_distance,
                        patrol_noncurrent_wall_stop_max_distance_,
                        segment.name.c_str());
                    continue;
                }

                immediate = true;
                ROS_INFO_THROTTLE(
                    0.8,
                    "V14.7非当前墙允许停车：%s，机器人距该墙=%.3fm<%.3fm。",
                    wallName(estimate.wall),
                    wall_distance,
                    patrol_noncurrent_wall_stop_max_distance_);
            }

            if (!immediate) {
                continue;
            }

            const double error =
                std::fabs(
                    boxes[i].centerX() - image_center);
            if (error < best_error) {
                best_error = error;
                selected = static_cast<int>(i);
            }
        }

        return selected;
    }

    // 从当前帧中寻找“车体左侧正在巡检的这一面墙”上的最近前方新目标。
    // 只在进入1.5m减速范围后锁存，避免远距离框直接打断巡检。
    void updateCurrentWallApproachFromBoxes(
            std::size_t segment_index,
            const Segment& segment,
            const Pose2D& pose,
            const std::vector<Box>& boxes) {
        int selected = -1;
        double best_distance =
            std::numeric_limits<double>::infinity();
        BoardBoundaryEstimate best_board;

        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].centerX() >=
                patrol_stop_max_center_x_) {
                continue;
            }

            BoardBoundaryEstimate estimate;
            if (!estimateBoardBoundary(
                    pose, boxes[i], estimate)) {
                continue;
            }
            if (estimate.wall != segment.wall ||
                isDuplicateBoard(estimate)) {
                continue;
            }

            // 必须位于当前巡检方向前方；已经驶过的板不参与接近减速。
            const double ahead =
                boardAheadProgress(
                    segment, pose, estimate);
            if (ahead < -0.03) {
                continue;
            }

            const double physical_distance =
                distance2D(
                    pose.x, pose.y,
                    estimate.x, estimate.y);

            if (physical_distance >
                patrol_target_slowdown_start_distance_) {
                continue;
            }

            if (physical_distance < best_distance) {
                best_distance = physical_distance;
                best_board = estimate;
                selected = static_cast<int>(i);
            }
        }

        if (selected < 0) {
            return;
        }

        patrol_visual_approach_.valid = true;
        patrol_visual_approach_.segment_index =
            static_cast<int>(segment_index);
        patrol_visual_approach_.board = best_board;
        patrol_visual_approach_.latest_box =
            boxes[static_cast<std::size_t>(selected)];
        patrol_visual_approach_.have_latest_box = true;

        ROS_INFO_THROTTLE(
            0.6,
            "V14.5锁定当前左墙目标：%s(%.3f,%.3f)，"
            "当前直线距离=%.3fm，开始/继续视觉接近减速。",
            wallName(best_board.wall),
            best_board.x,
            best_board.y,
            best_distance);
    }

    bool updatePatrolVisualApproachSpeed(
            const Segment& segment,
            const Pose2D& pose,
            double& distance_to_board) {
        distance_to_board =
            std::numeric_limits<double>::infinity();

        if (!patrol_visual_approach_.valid ||
            patrol_visual_approach_.segment_index !=
                current_segment_index_) {
            return false;
        }

        const double ahead =
            boardAheadProgress(
                segment,
                pose,
                patrol_visual_approach_.board);

        // 理论上在0.5m处已停车；若因定位瞬跳目标已明显到车后，
        // 清掉状态并恢复巡检速度，防止反向追一个已经驶过的板。
        if (ahead < -0.08) {
            ROS_WARN(
                "V14.5当前左墙目标已位于巡检方向后方%.3fm，"
                "取消本次接近状态并恢复巡检速度。",
                -ahead);
            clearPatrolVisualApproach(true);
            return false;
        }

        distance_to_board =
            distance2D(
                pose.x, pose.y,
                patrol_visual_approach_.board.x,
                patrol_visual_approach_.board.y);

        const double ratio =
            patrolTargetSpeedRatio(
                distance_to_board);
        const double speed_limit =
            patrol_speed_limit_ * ratio;

        setPatrolRuntimeSpeedLimit(speed_limit);

        ROS_INFO_THROTTLE(
            0.35,
            "V14.5巡检视觉减速：当前左墙目标距离=%.3fm，"
            "前向投影剩余=%.3fm，速度比例=%.3f，"
            "MyPlanner四项速度上限=%.3fm/s（巡检基准=%.3f）。",
            distance_to_board,
            ahead,
            ratio,
            speed_limit,
            patrol_speed_limit_);

        return true;
    }

    // 到0.5m停车后再取一帧近距离NanoDet，避免把1.5m处的小框
    // 作为OCR参考框。优先选与锁存墙面坐标最接近的同墙框。
    bool reacquireApproachBoxAfterStop(
            const Segment& segment,
            Pose2D& stopped_pose,
            Box& trigger_box) {
        const BoardBoundaryEstimate locked_board =
            patrol_visual_approach_.board;

        for (int attempt = 0;
             attempt < docking_recovery_detection_attempts_ &&
             ros::ok();
             ++attempt) {
            ros::spinOnce();

            std::vector<Box> boxes;
            if (!detectBoxes(boxes) ||
                boxes.empty()) {
                continue;
            }

            getRobotPose(stopped_pose);

            int selected = -1;
            double best_shift =
                std::numeric_limits<double>::infinity();

            for (std::size_t i = 0;
                 i < boxes.size(); ++i) {
                BoardBoundaryEstimate estimate;
                if (!estimateBoardBoundary(
                        stopped_pose,
                        boxes[i],
                        estimate)) {
                    continue;
                }
                if (estimate.wall != segment.wall ||
                    estimate.wall != locked_board.wall) {
                    continue;
                }

                const double shift =
                    distance2D(
                        estimate.x, estimate.y,
                        locked_board.x, locked_board.y);
                if (shift <=
                        docking_refine_max_board_shift_ &&
                    shift < best_shift) {
                    best_shift = shift;
                    selected = static_cast<int>(i);
                }
            }

            if (selected >= 0) {
                trigger_box =
                    boxes[
                        static_cast<std::size_t>(
                            selected)];

                ROS_WARN(
                    "V14.5近距离停车后重新取框成功："
                    "attempt=%d/%d，center=(%.1f,%.1f)，"
                    "相对减速锁存板坐标偏移=%.3fm。",
                    attempt + 1,
                    docking_recovery_detection_attempts_,
                    trigger_box.centerX(),
                    trigger_box.centerY(),
                    best_shift);
                return true;
            }
        }

        if (patrol_visual_approach_.have_latest_box) {
            trigger_box =
                patrol_visual_approach_.latest_box;
            ROS_WARN(
                "V14.5在0.5m停车后未重新取得匹配框；"
                "回退使用减速过程中最近一次有效NanoDet框作为OCR参考。");
            return true;
        }

        ROS_ERROR(
            "V14.5当前左墙目标到达停车距离，但没有任何可用触发框。");
        return false;
    }

    double limitPatrolTransitionRate(double desired,
                                     double previous,
                                     double max_delta) const {
        return previous + clampValue(
            desired - previous, -max_delta, max_delta);
    }

    void cornerCostmapCallback(
        const nav_msgs::OccupancyGrid::ConstPtr& message
    ) {
        if (!message) {
            return;
        }

        corner_costmap_message_ = message;
        corner_costmap_received_wall_time_ =
            ros::WallTime::now();
    }

    std::string normalizedFrameName(
        const std::string& frame
    ) const {
        if (!frame.empty() && frame[0] == '/') {
            return frame.substr(1);
        }
        return frame;
    }

    bool hasCornerObstacleDeadZone(
        double center_x,
        double center_y,
        const std::string& label
    ) const {
        // 安全优先：costmap没有数据、太旧、坐标系异常或检测区域
        // 不在当前map覆盖范围内时，一律按“存在路障”处理，
        // 即继续使用原V13安全角点。
        if (!corner_costmap_message_) {
            ROS_WARN(
                "%s：尚未收到角点costmap(%s)，"
                "按存在路障处理，保持原安全角点。",
                label.c_str(),
                corner_obstacle_costmap_topic_.c_str()
            );
            return true;
        }

        const double age =
            (
                ros::WallTime::now()
                - corner_costmap_received_wall_time_
            ).toSec();

        if (
            !std::isfinite(age)
            || age > corner_obstacle_costmap_max_age_
        ) {
            ROS_WARN(
                "%s：角点costmap数据过旧，age=%.2fs>%.2fs，"
                "按存在路障处理，保持原安全角点。",
                label.c_str(),
                age,
                corner_obstacle_costmap_max_age_
            );
            return true;
        }

        const nav_msgs::OccupancyGrid& grid =
            *corner_costmap_message_;

        if (
            normalizedFrameName(grid.header.frame_id)
            != normalizedFrameName(map_frame_)
        ) {
            ROS_WARN(
                "%s：角点costmap坐标系=%s，与map_frame=%s不一致，"
                "按存在路障处理，保持原安全角点。",
                label.c_str(),
                grid.header.frame_id.c_str(),
                map_frame_.c_str()
            );
            return true;
        }

        if (
            grid.info.resolution <= 0.0
            || grid.info.width == 0
            || grid.info.height == 0
            || grid.data.empty()
        ) {
            ROS_WARN(
                "%s：角点costmap尺寸/分辨率无效，"
                "按存在路障处理，保持原安全角点。",
                label.c_str()
            );
            return true;
        }

        const double resolution =
            static_cast<double>(
                grid.info.resolution);

        const double origin_yaw =
            tf2::getYaw(
                grid.info.origin.orientation);

        const double dx =
            center_x
            - grid.info.origin.position.x;

        const double dy =
            center_y
            - grid.info.origin.position.y;

        const double c =
            std::cos(origin_yaw);

        const double s =
            std::sin(origin_yaw);

        // map坐标转换到OccupancyGrid原点局部坐标。
        const double local_center_x =
            c * dx + s * dy;

        const double local_center_y =
            -s * dx + c * dy;

        const double radius =
            corner_obstacle_check_radius_;

        const int min_cell_x =
            static_cast<int>(
                std::floor(
                    (local_center_x - radius)
                    / resolution));

        const int max_cell_x =
            static_cast<int>(
                std::floor(
                    (local_center_x + radius)
                    / resolution));

        const int min_cell_y =
            static_cast<int>(
                std::floor(
                    (local_center_y - radius)
                    / resolution));

        const int max_cell_y =
            static_cast<int>(
                std::floor(
                    (local_center_y + radius)
                    / resolution));

        if (
            max_cell_x < 0
            || max_cell_y < 0
            || min_cell_x
                >= static_cast<int>(grid.info.width)
            || min_cell_y
                >= static_cast<int>(grid.info.height)
        ) {
            ROS_WARN(
                "%s：检测中心(%.2f,%.2f)半径%.2fm不在当前local costmap覆盖范围，"
                "按存在路障处理，保持原安全角点。",
                label.c_str(),
                center_x,
                center_y,
                radius
            );
            return true;
        }

        const int begin_x =
            std::max(
                0,
                min_cell_x);

        const int end_x =
            std::min(
                static_cast<int>(grid.info.width) - 1,
                max_cell_x);

        const int begin_y =
            std::max(
                0,
                min_cell_y);

        const int end_y =
            std::min(
                static_cast<int>(grid.info.height) - 1,
                max_cell_y);

        int checked_cells = 0;
        int lethal_cells = 0;
        int maximum_cost = -1;

        for (
            int cell_y = begin_y;
            cell_y <= end_y;
            ++cell_y
        ) {
            for (
                int cell_x = begin_x;
                cell_x <= end_x;
                ++cell_x
            ) {
                const double cell_center_x =
                    (
                        static_cast<double>(cell_x)
                        + 0.5
                    ) * resolution;

                const double cell_center_y =
                    (
                        static_cast<double>(cell_y)
                        + 0.5
                    ) * resolution;

                const double offset_x =
                    cell_center_x
                    - local_center_x;

                const double offset_y =
                    cell_center_y
                    - local_center_y;

                if (
                    std::hypot(
                        offset_x,
                        offset_y)
                    > radius
                ) {
                    continue;
                }

                const std::size_t index =
                    static_cast<std::size_t>(cell_y)
                    * static_cast<std::size_t>(
                        grid.info.width)
                    + static_cast<std::size_t>(
                        cell_x);

                if (index >= grid.data.size()) {
                    continue;
                }

                const int cost =
                    static_cast<int>(
                        grid.data[index]);

                // -1为unknown，不作为“确认存在路障死区”的依据。
                if (cost < 0) {
                    continue;
                }

                ++checked_cells;

                maximum_cost =
                    std::max(
                        maximum_cost,
                        cost);

                if (
                    cost
                    >= corner_obstacle_lethal_cost_threshold_
                ) {
                    ++lethal_cells;
                }
            }
        }

        if (checked_cells <= 0) {
            ROS_WARN(
                "%s：检测中心(%.2f,%.2f)半径%.2fm内没有可用costmap单元，"
                "按存在路障处理，保持原安全角点。",
                label.c_str(),
                center_x,
                center_y,
                radius
            );
            return true;
        }

        const bool obstacle_present =
            lethal_cells > 0;

        ROS_WARN(
            "%s：角点路障死区检测 center=(%.2f,%.2f)，radius=%.2fm，"
            "有效格=%d，最高代价=%d，>=阈值%d的死区格=%d，结果=%s。",
            label.c_str(),
            center_x,
            center_y,
            radius,
            checked_cells,
            maximum_cost,
            corner_obstacle_lethal_cost_threshold_,
            lethal_cells,
            obstacle_present
                ? "存在路障死区，保持原角点"
                : "无路障死区，使用提前10cm的新角点"
        );

        return obstacle_present;
    }

    bool runPatrolPoseTransition(
        double target_x,
        double target_y,
        double target_yaw,
        const std::string& label,
        double timeout_override = -1.0
    ) {
        if (!isInsideRoom(target_x, target_y)) {
            ROS_ERROR("%s目标(%.3f, %.3f)超出房间边界",
                      label.c_str(), target_x, target_y);
            return false;
        }

        // 角点过渡只由/set_speed直接接管。调用前固定巡检Path已经解除，
        // 不让move_base最终姿态调整在墙边原地旋转。
        stopRobot();

        const double effective_timeout =
            timeout_override > 0.0
            ? std::max(
                  patrol_transition_timeout_,
                  timeout_override)
            : patrol_transition_timeout_;

        const ros::WallTime deadline =
            ros::WallTime::now()
            + ros::WallDuration(
                effective_timeout);
        ros::WallTime last_time = ros::WallTime::now();
        ros::Rate rate(std::max(10.0, control_rate_));

        double command_vx = 0.0;
        double command_vy = 0.0;
        double command_wz = 0.0;
        int stable_frames = 0;

        ROS_INFO("%s开始：目标=(%.3f, %.3f, %.1f度)，"
                 "采用全向XY+yaw同时P闭环，并启用V13旋转优先。",
                 label.c_str(), target_x, target_y,
                 target_yaw * 180.0 / kPi);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            const double world_error_x = target_x - pose.x;
            const double world_error_y = target_y - pose.y;
            const double position_error =
                std::hypot(world_error_x, world_error_y);
            const double yaw_error =
                normalizeAngle(target_yaw - pose.yaw);

            // map误差转到当前车体坐标，因此在旋转过程中仍能正确向绝对目标横移。
            const double c = std::cos(pose.yaw);
            const double s = std::sin(pose.yaw);
            const double base_error_x =
                c * world_error_x + s * world_error_y;
            const double base_error_y =
                -s * world_error_x + c * world_error_y;

            double desired_vx = 0.0;
            double desired_vy = 0.0;
            double desired_wz = 0.0;

            if (position_error > patrol_transition_position_tolerance_) {
                desired_vx = clampValue(
                    patrol_transition_position_kp_ * base_error_x,
                    -patrol_transition_max_linear_speed_,
                    patrol_transition_max_linear_speed_);
                desired_vy = clampValue(
                    patrol_transition_position_kp_ * base_error_y,
                    -patrol_transition_max_linear_speed_,
                    patrol_transition_max_linear_speed_);

                const double linear_norm =
                    std::hypot(desired_vx, desired_vy);
                if (linear_norm > 1e-9 &&
                    linear_norm < patrol_transition_min_linear_speed_) {
                    const double scale =
                        patrol_transition_min_linear_speed_ / linear_norm;
                    desired_vx *= scale;
                    desired_vy *= scale;
                }

                const double limited_norm =
                    std::hypot(desired_vx, desired_vy);
                if (limited_norm > patrol_transition_max_linear_speed_) {
                    const double scale =
                        patrol_transition_max_linear_speed_ / limited_norm;
                    desired_vx *= scale;
                    desired_vy *= scale;
                }
            }

            // V13：角点过渡采用“旋转优先的同步平移”。
            // 当车头仍大幅朝向墙面时，只保留少量平移；随着yaw误差减小，
            // 线速度连续恢复到100%。这样仍然边转边移，但不会先走完5cm。
            double rotation_priority_scale = 1.0;
            const double abs_yaw_error_deg =
                std::fabs(yaw_error) * 180.0 / kPi;
            if (abs_yaw_error_deg >= patrol_transition_yaw_priority_start_deg_) {
                rotation_priority_scale =
                    patrol_transition_yaw_priority_min_linear_scale_;
            } else if (abs_yaw_error_deg >
                       patrol_transition_yaw_priority_release_deg_) {
                const double span = std::max(
                    1.0,
                    patrol_transition_yaw_priority_start_deg_
                    - patrol_transition_yaw_priority_release_deg_);
                const double progress = clampValue(
                    (patrol_transition_yaw_priority_start_deg_
                     - abs_yaw_error_deg) / span,
                    0.0, 1.0);
                rotation_priority_scale =
                    patrol_transition_yaw_priority_min_linear_scale_
                    + (1.0 - patrol_transition_yaw_priority_min_linear_scale_)
                      * progress;
            }
            desired_vx *= rotation_priority_scale;
            desired_vy *= rotation_priority_scale;

            const double yaw_tolerance =
                patrol_transition_yaw_tolerance_deg_ * kPi / 180.0;
            if (std::fabs(yaw_error) > yaw_tolerance) {
                desired_wz = clampValue(
                    patrol_transition_yaw_kp_ * yaw_error,
                    -patrol_transition_max_angular_speed_,
                    patrol_transition_max_angular_speed_);
                if (std::fabs(desired_wz) <
                    patrol_transition_min_angular_speed_) {
                    desired_wz = std::copysign(
                        patrol_transition_min_angular_speed_, yaw_error);
                }
            }

            const ros::WallTime now = ros::WallTime::now();
            double dt = (now - last_time).toSec();
            last_time = now;
            if (!std::isfinite(dt) || dt <= 0.0) dt = 0.05;
            dt = clampValue(dt, 0.01, 0.20);

            command_vx = limitPatrolTransitionRate(
                desired_vx, command_vx,
                patrol_transition_linear_accel_ * dt);
            command_vy = limitPatrolTransitionRate(
                desired_vy, command_vy,
                patrol_transition_linear_accel_ * dt);
            command_wz = limitPatrolTransitionRate(
                desired_wz, command_wz,
                patrol_transition_angular_accel_ * dt);

            const bool position_ok =
                position_error <= patrol_transition_position_tolerance_;
            const bool yaw_ok = std::fabs(yaw_error) <= yaw_tolerance;

            if (position_ok && yaw_ok) {
                ++stable_frames;
                command_vx = 0.0;
                command_vy = 0.0;
                command_wz = 0.0;
                if (stable_frames >= patrol_transition_stable_frames_) {
                    stopRobot();
                    ROS_INFO("%s完成：当前位置=(%.3f, %.3f, %.1f度)。",
                             label.c_str(), pose.x, pose.y,
                             pose.yaw * 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
            }

            publishVelocity(command_vx, command_vy, command_wz);
            ROS_INFO_THROTTLE(
                0.5,
                "%s中：pose=(%.3f,%.3f,%.1f度)，"
                "pos_err=%.3fm，yaw_err=%.1f度，cmd=(%.3f,%.3f,%.3f)",
                label.c_str(), pose.x, pose.y,
                pose.yaw * 180.0 / kPi,
                position_error, yaw_error * 180.0 / kPi,
                command_vx, command_vy, command_wz);
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR(
            "%s超时：未能在%.1fs内完成安全位姿过渡。",
            label.c_str(),
            effective_timeout);
        return false;
    }

    struct CornerTransitionTarget {
        bool valid = false;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        std::string label;
    };

    bool selectCornerTransitionTarget(
        std::size_t segment_index,
        CornerTransitionTarget& target
    ) const {
        target = CornerTransitionTarget();

        switch (segment_index) {
            case 0:
            {
                // 第一段固定巡线终点仍为(4.75,4.30)，完全不改。
                // 检查(4.50,4.00)周围5cm：
                // 有路障 -> 原角点(4.80,4.30,-90°)
                // 无路障 -> 新角点(4.80,4.20,-90°)
                const bool obstacle_present =
                    hasCornerObstacleDeadZone(
                        4.50,
                        4.00,
                        "第一转角");

                target.valid = true;
                target.x = 4.80;
                target.y =
                    obstacle_present
                    ? 4.30
                    : 4.20;
                target.yaw = -0.5 * kPi;
                target.label =
                    obstacle_present
                    ? "上墙→右墙安全角点过渡【有路障：原模式】"
                    : "上墙→右墙安全角点过渡【无路障：提前10cm】";
                return true;
            }

            case 1:
            {
                // 第二段固定巡线终点仍为(4.80,2.75)，完全不改。
                // 检查(4.50,3.00)周围5cm：
                // 有路障 -> 原角点(4.80,2.70,180°)
                // 无路障 -> 新角点(4.70,2.70,180°)
                const bool obstacle_present =
                    hasCornerObstacleDeadZone(
                        4.50,
                        3.00,
                        "第二转角");

                target.valid = true;
                target.x =
                    obstacle_present
                    ? 4.80
                    : 4.70;
                target.y = 2.70;
                target.yaw = kPi;
                target.label =
                    obstacle_present
                    ? "右墙→下墙安全角点过渡【有路障：原位置】"
                    : "右墙→下墙安全角点过渡【无路障：提前10cm】";
                return true;
            }

            case 2:
            {
                // 第三段固定巡线终点仍为(0.25,2.70)，完全不改。
                // 检查(0.50,3.00)周围5cm：
                // 有路障 -> 原角点(0.20,2.70,+90°)
                // 无路障 -> 新角点(0.20,2.80,+90°)
                const bool obstacle_present =
                    hasCornerObstacleDeadZone(
                        0.50,
                        3.00,
                        "第三转角");

                target.valid = true;
                target.x = 0.20;
                target.y =
                    obstacle_present
                    ? 2.70
                    : 2.80;
                target.yaw = 0.5 * kPi;
                target.label =
                    obstacle_present
                    ? "下墙→左墙安全角点过渡【有路障：原模式】"
                    : "下墙→左墙安全角点过渡【无路障：提前10cm】";
                return true;
            }

            default:
                // 第四条巡检线最终仍直接停在(0.20,4.25)。
                // 当前没有第四段结束后的额外角点。
                return false;
        }
    }

    bool runCornerTransitionAfterSegment(
        std::size_t segment_index,
        double timeout_override = -1.0
    ) {
        CornerTransitionTarget target;

        if (!selectCornerTransitionTarget(
                segment_index,
                target
            )) {
            // 第四段没有后续角点，保持原逻辑直接成功。
            return true;
        }

        return runPatrolPoseTransition(
            target.x,
            target.y,
            target.yaw,
            target.label,
            timeout_override);
    }

    bool runNearEndCornerShortcut(
        std::size_t segment_index,
        const Segment& segment,
        double recognized_docking_x,
        double recognized_docking_y
    ) {
        // 第四段没有后续角点，不能启用该捷径。
        CornerTransitionTarget target;

        if (!selectCornerTransitionTarget(
                segment_index,
                target
            )) {
            return false;
        }

        Pose2D current_pose;

        if (!getRobotPose(current_pose)) {
            ROS_ERROR(
                "V23段末停靠捷径失败：无法读取当前机器人位姿。");
            return false;
        }

        const double current_to_corner_distance =
            distance2D(
                current_pose.x,
                current_pose.y,
                target.x,
                target.y);

        // 这个捷径可能从目标板停靠位直接跨越接近1m到角点。
        // 原来的8s角点超时只针对5~10cm安全小平移，因此这里仅对
        // 本次V23调用自动放宽超时；正常三个角点的原超时完全不变。
        const double safe_linear_speed =
            std::max(
                0.03,
                patrol_transition_max_linear_speed_);

        const double shortcut_timeout =
            std::max(
                patrol_transition_timeout_,
                current_to_corner_distance
                    / safe_linear_speed
                + 4.0);

        ROS_WARN(
            "V23段末停靠捷径触发："
            "本次识别停靠点=(%.3f,%.3f)，"
            "%s终点=(%.3f,%.3f)，"
            "识别停靠点距终点<%.2fm；"
            "跳过V17回线和剩余固定Path，"
            "直接执行本段角点旋转平移到"
            "(%.3f,%.3f,%.1f°)。"
            "当前车位距角点=%.3fm，本次允许超时=%.1fs。",
            recognized_docking_x,
            recognized_docking_y,
            segment.name.c_str(),
            segment.end_x,
            segment.end_y,
            patrol_near_end_corner_shortcut_distance_,
            target.x,
            target.y,
            target.yaw * 180.0 / kPi,
            current_to_corner_distance,
            shortcut_timeout);

        return runPatrolPoseTransition(
            target.x,
            target.y,
            target.yaw,
            target.label
                + "【V23停靠点近段末直接跳转】",
            shortcut_timeout);
    }

    SegmentResult completePatrolSegment(
        std::size_t segment_index,
        bool corner_transition_already_completed = false
    ) {
        const Segment& segment = segments_[segment_index];
        patrol_checkpoint_.valid = false;
        clearPatrolVisualApproach(false);
        finishPatrolMode();
        ROS_INFO("%s完成；已恢复普通导航速度%.1f和路径重规划。",
                 segment.name.c_str(), normal_navigation_speed_limit_);
        if (!corner_transition_already_completed) {
            if (!runCornerTransitionAfterSegment(
                    segment_index
                )) {
                ROS_ERROR(
                    "%s完成后安全角点过渡失败。",
                    segment.name.c_str());
                return SEGMENT_ABORTED;
            }
        } else {
            ROS_WARN(
                "%s使用V23段末停靠捷径："
                "角点旋转平移已经提前完成，"
                "本段收尾不再重复执行角点过渡。",
                segment.name.c_str());
        }

        // V14.3保护2：
        // 非当前巡检墙的仿真目标只阻塞到“发现它时正在巡检的这一整面墙”
        // 完成。到这里才允许解除；在patrolSegment循环、现实目标停靠完成
        // 等任何更早时刻都绝不能绕过该保护。
        if (simulation_target_blocked_until_segment_end_) {
            simulation_target_blocked_until_segment_end_ = false;
            ROS_WARN(
                "%s已经完整巡检结束：此前记录的非当前巡检墙仿真目标"
                "现已解除段内停靠保护。",
                segment.name.c_str());
        }

        if (hasDeferredRealTargetAfterSegment(segment_index)) {
            const DetectionResult deferred_result =
                dockDeferredRealTargetAfterSegment(segment_index);
            if (deferred_result == DETECTION_MISSION_COMPLETE) {
                return SEGMENT_MISSION_COMPLETE;
            }
            if (deferred_result == DETECTION_ABORT) {
                return SEGMENT_ABORTED;
            }
        }

        // 如果现实目标早已停靠，而本段仅延后了一个非当前墙仿真目标，
        // 现在整面墙已扫完，可以在段末正式处理。
        if (hasPendingSimulationTarget()) {
            ROS_INFO(
                "%s整面墙巡检完成，开始处理此前允许在段末执行的仿真目标。",
                segment.name.c_str());
            if (!dockPendingSimulationTarget()) {
                return SEGMENT_ABORTED;
            }
            return SEGMENT_MISSION_COMPLETE;
        }

        return SEGMENT_COMPLETE;
    }

    SegmentResult patrolSegment(std::size_t segment_index) {
        const Segment& segment = segments_[segment_index];

        // 新的一面墙必须从干净的视觉接近状态开始。
        clearPatrolVisualApproach(false);

        ROS_INFO("开始%s：(%.2f, %.2f)→(%.2f, %.2f)，"
                 "固定路径由move_base和MyPlanner跟踪；不使用/set_speed巡检。",
                 segment.name.c_str(), segment.start_x, segment.start_y,
                 segment.end_x, segment.end_y);

        Pose2D start_pose;
        if (!getRobotPose(start_pose)) return SEGMENT_ABORTED;
        PatrolStartResult start_result =
            startPatrolFromPose(segment, start_pose);
        if (start_result == PATROL_ALREADY_COMPLETE) {
            return completePatrolSegment(segment_index);
        }
        if (start_result == PATROL_START_FAILED) {
            finishPatrolMode();
            return SEGMENT_ABORTED;
        }

        int aborted_retry_count = 0;
        ros::Rate rate(control_rate_);
        while (ros::ok()) {
            ros::spinOnce();

            // V24：
            // 非当前左墙现实目标如果在识别时已经确认“目标板距当前段终点<阈值”，
            // 则继续当前巡线，直到机器人自身也进入段终点同样阈值范围。
            // 一旦满足，立即取消当前剩余Path，提前去停靠该目标。
            if (
                hasNearEndDeferredRealTarget(
                    segment_index)
            ) {
                Pose2D current_pose;

                if (getRobotPose(current_pose)) {
                    const double robot_to_segment_end =
                        distance2D(
                            current_pose.x,
                            current_pose.y,
                            segment.end_x,
                            segment.end_y);

                    ROS_INFO_THROTTLE(
                        0.8,
                        "V24等待非当前墙现实目标提前停靠窗口："
                        "目标板距%s终点=%.3fm<%.3fm；"
                        "当前机器人距终点=%.3fm。",
                        segment.name.c_str(),
                        real_target_distance_to_source_segment_end_,
                        patrol_noncurrent_target_early_dock_distance_,
                        robot_to_segment_end);

                    if (
                        robot_to_segment_end
                        < patrol_noncurrent_target_early_dock_distance_
                    ) {
                        ROS_WARN(
                            "V24提前停靠条件满足："
                            "目标板距当前段终点=%.3fm，"
                            "机器人距当前段终点=%.3fm，"
                            "均小于%.3fm；"
                            "立即结束当前剩余固定Path并前往非当前墙现实目标。",
                            real_target_distance_to_source_segment_end_,
                            robot_to_segment_end,
                            patrol_noncurrent_target_early_dock_distance_);

                        if (!cancelPatrolGoalAndWait()) {
                            finishPatrolMode();
                            return SEGMENT_ABORTED;
                        }

                        finishPatrolMode();

                        return
                            dockNearEndDeferredRealTargetAndSwitchWall(
                                segment_index);
                    }
                }
            }

            if (hasPendingSimulationTarget() &&
                !simulation_target_blocked_until_segment_end_) {
                ROS_INFO("检测到可立即处理的已记录仿真目标，暂停巡检并前往");
                if (!cancelPatrolGoalAndWait()) {
                    finishPatrolMode();
                    return SEGMENT_ABORTED;
                }
                finishPatrolMode();
                if (!dockPendingSimulationTarget()) {
                    return SEGMENT_ABORTED;
                }
                return SEGMENT_MISSION_COMPLETE;
            }
            if (hasPendingSimulationTarget() &&
                simulation_target_blocked_until_segment_end_) {
                ROS_INFO_THROTTLE(
                    1.0,
                    "V14.4保护：存在待停靠仿真目标，它是在现实目标尚未停靠时"
                    "于非当前巡检墙被识别；必须先完整巡检完%s，当前不允许中断。",
                    segment.name.c_str());
            }

            const actionlib::SimpleClientGoalState state =
                move_base_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                patrol_goal_active_ = false;
                return completePatrolSegment(segment_index);
            }
            if (state.isDone()) {
                patrol_goal_active_ = false;
                const std::string state_text = state.toString();

                if (state == actionlib::SimpleClientGoalState::ABORTED &&
                    aborted_retry_count < patrol_aborted_retry_count_) {
                    Pose2D retry_pose;
                    if (getRobotPose(retry_pose)) {
                        ++aborted_retry_count;
                        ROS_WARN("%s的move_base目标出现ABORTED；不终止任务，"
                                 "从当前位置重建剩余固定Path并续跑（%d/%d）。",
                                 segment.name.c_str(), aborted_retry_count,
                                 patrol_aborted_retry_count_);
                        start_result = startPatrolFromPose(segment, retry_pose);
                        if (start_result == PATROL_ALREADY_COMPLETE) {
                            return completePatrolSegment(segment_index);
                        }
                        if (start_result == PATROL_STARTED) {
                            rate.sleep();
                            continue;
                        }
                    }
                }

                finishPatrolMode();
                ROS_ERROR("%s的move_base目标异常结束：%s",
                          segment.name.c_str(), state_text.c_str());
                return SEGMENT_ABORTED;
            }

            Pose2D pose;
            if (!getRobotPose(pose)) {
                rate.sleep();
                continue;
            }

            std::vector<Box> boxes;
            const bool detection_ok = detectBoxes(boxes);

            // ----------------------------------------------------------
            // V14.7优先级：
            // 1) 当前左墙目标即将从画面左缘消失 -> 立刻停车；
            // 2) 非当前墙且距该墙<1.5m -> 保持原立即停车；
            // 3) 否则当前墙继续按1.5m->0.7m map距离减速。
            // ----------------------------------------------------------
            int current_wall_edge_selected = -1;
            BoardBoundaryEstimate current_wall_edge_estimate;

            int immediate_selected = -1;

            if (detection_ok && !boxes.empty()) {
                current_wall_edge_selected =
                    chooseCurrentWallLeftEdgeEmergencyBox(
                        boxes,
                        pose,
                        segment,
                        current_wall_edge_estimate);

                if (current_wall_edge_selected < 0) {
                    immediate_selected =
                        chooseImmediateNonCurrentBoardBox(
                            boxes,
                            pose,
                            segment);
                }
            }

            bool should_stop_for_ocr = false;

            // 普通map距离停车：仍需要停车后重新取近距离框。
            bool stop_from_current_wall_approach = false;

            // V14.8：x0<100视觉急停已经是合适OCR距离，
            // 直接使用当前触发框，不再二次NanoDet。
            bool direct_ocr_from_edge_guard = false;

            Box trigger_box{0, 0, 0, 0, 0};

            if (current_wall_edge_selected >= 0) {
                trigger_box =
                    boxes[
                        static_cast<std::size_t>(
                            current_wall_edge_selected)];

                should_stop_for_ocr = true;
                direct_ocr_from_edge_guard = true;

                ROS_WARN(
                    "V14.8画面左缘直接OCR急停：当前%s上的目标框"
                    "x0=%d<%dpx，centerX=%.1f；"
                    "该视野已属于实测适合OCR的近距离范围，"
                    "立即取消巡检goal并停车，直接使用当前触发框进入OCR，"
                    "不再执行停车后二次NanoDet取框。",
                    wallName(current_wall_edge_estimate.wall),
                    trigger_box.x0,
                    patrol_target_left_edge_stop_px_,
                    trigger_box.centerX());
            } else if (immediate_selected >= 0) {
                trigger_box =
                    boxes[
                        static_cast<std::size_t>(
                            immediate_selected)];
                should_stop_for_ocr = true;

                ROS_WARN(
                    "V14.7检测到允许立即处理的非当前巡检墙目标/"
                    "墙面估计失败目标；中断%s进入OCR。",
                    segment.name.c_str());
            } else {
                // ------------------------------------------------------
                // 当前车体左侧巡检墙目标：
                // 进入1.5m后锁存，并根据map距离连续限制MyPlanner速度。
                // ------------------------------------------------------
                if (detection_ok && !boxes.empty()) {
                    updateCurrentWallApproachFromBoxes(
                        segment_index,
                        segment,
                        pose,
                        boxes);
                }

                double distance_to_board =
                    std::numeric_limits<double>::infinity();
                if (updatePatrolVisualApproachSpeed(
                        segment,
                        pose,
                        distance_to_board)) {
                    if (distance_to_board <=
                        patrol_target_stop_distance_) {
                        should_stop_for_ocr = true;
                        stop_from_current_wall_approach = true;

                        setPatrolRuntimeSpeedLimit(
                            patrol_speed_limit_ *
                            patrol_target_min_speed_ratio_,
                            true);

                        ROS_WARN(
                            "V14.7当前左墙目标达到map停车距离："
                            "%.3fm<=%.3fm；速度上限已降至巡检速度的%.0f%%，"
                            "现在取消巡检goal并停车。",
                            distance_to_board,
                            patrol_target_stop_distance_,
                            patrol_target_min_speed_ratio_ * 100.0);
                    }
                }
            }

            if (!should_stop_for_ocr) {
                if ((!detection_ok || boxes.empty()) &&
                    !patrol_visual_approach_.valid) {
                    restorePatrolCruiseSpeedIfNeeded();
                }
                rate.sleep();
                continue;
            }

            if (!cancelPatrolGoalAndWait()) {
                clearPatrolVisualApproach(false);
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            Pose2D stopped_pose = pose;
            getRobotPose(stopped_pose);

            // 普通map距离停车仍使用V14.5的二次近距离取框。
            // V14.8的x0<100视觉急停明确跳过这里，直接使用触发急停的当前框。
            if (stop_from_current_wall_approach &&
                !direct_ocr_from_edge_guard) {
                if (!reacquireApproachBoxAfterStop(
                        segment,
                        stopped_pose,
                        trigger_box)) {
                    clearPatrolVisualApproach(false);

                    // 没框时不做错误OCR；从当前断点重新启动剩余固定Path。
                    start_result =
                        startPatrolFromPose(
                            segment,
                            stopped_pose);
                    if (start_result ==
                        PATROL_ALREADY_COMPLETE) {
                        return completePatrolSegment(
                            segment_index);
                    }
                    if (start_result ==
                        PATROL_START_FAILED) {
                        finishPatrolMode();
                        return SEGMENT_ABORTED;
                    }

                    ROS_WARN(
                        "V14.5停车后未能重新取得当前左墙目标框；"
                        "已恢复%s，等待后续重新检测。",
                        segment.name.c_str());
                    rate.sleep();
                    continue;
                }
            }

            patrol_checkpoint_.valid = true;
            patrol_checkpoint_.segment_index =
                static_cast<int>(segment_index);
            patrol_checkpoint_.stopped_progress =
                clampValue(
                    segmentProgress(
                        segment,
                        stopped_pose),
                    0.0,
                    segment.length);
            patrol_checkpoint_.stopped_pose =
                stopped_pose;

            ROS_INFO(
                "已在%s中断点停车：进度=%.3f/%.3fm。",
                segment.name.c_str(),
                patrol_checkpoint_.stopped_progress,
                segment.length);

            // 当前接近目标已经真正停车，后续交给原V14.4 OCR/停靠状态机。
            clearPatrolVisualApproach(false);

            if (direct_ocr_from_edge_guard) {
                ROS_WARN(
                    "V14.8视觉急停已完成：直接使用触发框"
                    "(%d,%d)-(%d,%d)，center=(%.1f,%.1f)"
                    "进入handleDetectedBoard/OCR。",
                    trigger_box.x0,
                    trigger_box.y0,
                    trigger_box.x1,
                    trigger_box.y1,
                    trigger_box.centerX(),
                    trigger_box.centerY());
            }

            const DetectionResult detection_result =
                handleDetectedBoard(
                    segment_index,
                    segment,
                    stopped_pose,
                    trigger_box);

            if (detection_result ==
                DETECTION_MISSION_COMPLETE) {
                finishPatrolMode();
                return SEGMENT_MISSION_COMPLETE;
            }

            if (detection_result ==
                DETECTION_SEGMENT_COMPLETE_AFTER_CORNER) {
                // handleDetectedBoard已经完成现实目标停靠以及本段角点
                // 旋转平移。这里直接执行段末状态收尾，不再重启本段Path。
                SegmentResult segment_result =
                    completePatrolSegment(
                        segment_index,
                        true);

                // 若段末处理没有直接完成双目标任务，则下一段仍需要摄像头。
                if (
                    segment_result ==
                    SEGMENT_COMPLETE
                ) {
                    if (!openCamera()) {
                        finishPatrolMode();
                        return SEGMENT_ABORTED;
                    }

                    ROS_INFO(
                        "V23段末停靠捷径收尾完成；"
                        "摄像头已重新打开，准备进入下一条巡检段。");
                }

                return segment_result;
            }

            if (detection_result ==
                DETECTION_ABORT) {
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            Pose2D resume_pose;
            if (!getRobotPose(resume_pose)) {
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            start_result =
                startPatrolFromPose(
                    segment,
                    resume_pose);
            if (start_result ==
                PATROL_ALREADY_COMPLETE) {
                return completePatrolSegment(
                    segment_index);
            }
            if (start_result ==
                PATROL_START_FAILED) {
                finishPatrolMode();
                return SEGMENT_ABORTED;
            }

            ROS_INFO(
                "不执行额外朝向恢复；MyPlanner将按剩余固定路径"
                "自行完成初始姿态对准并继续%s。",
                segment.name.c_str());
            rate.sleep();
        }

        finishPatrolMode();
        return SEGMENT_ABORTED;
    }

    DetectionResult handleDetectedBoard(std::size_t segment_index,
                                        const Segment& segment,
                                        const Pose2D& detection_pose,
                                        const Box& trigger_box) {
        stopRobot();
        ROS_INFO("NanoDet发现文字框，已停车；等待%.2f秒后调用OCR",
                 settle_time_);
        ros::Duration(settle_time_).sleep();

        Pose2D stopped_pose = detection_pose;
        getRobotPose(stopped_pose);

        BoardBoundaryEstimate boundary_estimate;
        bool boundary_coordinate_recorded = false;
        if (estimateBoardBoundary(stopped_pose,
                                  trigger_box, boundary_estimate)) {
            if (isDuplicateBoard(boundary_estimate)) {
                ROS_INFO("估计坐标%s(%.3f, %.3f)距已记录板不超过%.2fm，"
                         "判定为重复检测并跳过OCR",
                         wallName(boundary_estimate.wall),
                         boundary_estimate.x, boundary_estimate.y,
                         duplicate_coordinate_distance_);
                return DETECTION_CONTINUE;
            }

            if (boundary_estimate.wall == segment.wall) {
                // 当前巡检墙保持旧行为：停车后立即加入去重名单，
                // 即使OCR失败也不在同一面墙上反复停同一块板。
                seen_board_coordinates_.push_back(boundary_estimate);
                boundary_coordinate_recorded = true;
                ROS_INFO(
                    "记录当前巡检墙新文字板坐标：%s(%.3f, %.3f)",
                    wallName(boundary_estimate.wall),
                    boundary_estimate.x, boundary_estimate.y);
            } else {
                // V14.3保护3：
                // 非当前巡检墙只有OCR最终能分类后才允许进入去重名单。
                // 若本次OCR失败，就保留重新检测机会。
                ROS_WARN(
                    "检测到非当前巡检墙候选：%s(%.3f, %.3f)；"
                    "暂不加入去重名单，等待OCR成功分类后再记录",
                    wallName(boundary_estimate.wall),
                    boundary_estimate.x, boundary_estimate.y);
            }
        } else {
            ROS_WARN("无法由NanoDet框中心估计墙面坐标，"
                     "本次继续OCR但不加入重复坐标表");
        }

        OcrRecord ocr = recognizeStaticTarget(trigger_box);
        // 与已经实测稳定的target_scan_test保持一致：以可分类关键词为准。
        // 某些OCR服务版本即使返回了有效文字，success字段也可能未置true。
        // 若只读到车间名称的通用残片，说明车辆可能因速度较快越过了
        // 类别文字。此时只执行一次逆时针30度补偿旋转和一次完整OCR。
        // 复识成功后必须刷新map位姿，保证后面的相机射线使用新朝向。
        if (ocr.category == "unknown" && hasWorkshopFragment(ocr.text)) {
            ROS_WARN("OCR只识别到车间通用残片“%s”，"
                     "未命中类别关键字；准备原地逆时针转%.1f度复识一次",
                     ocr.text.c_str(), ocr_recovery_turn_deg_);

            Pose2D recovery_pose;
            if (!rotateCounterClockwiseForOcr(recovery_pose)) {
                if (boundary_estimate.valid &&
                    boundary_estimate.wall != segment.wall &&
                    !boundary_coordinate_recorded) {
                    ROS_WARN(
                        "V14.3保护：非当前巡检墙候选OCR补偿旋转失败，"
                        "坐标%s(%.3f, %.3f)不加入去重名单",
                        wallName(boundary_estimate.wall),
                        boundary_estimate.x, boundary_estimate.y);
                }
                addUnknownCandidate(
                    segment_index,
                    stopped_pose,
                    ocr.box,
                    boundary_estimate,
                    "OCR通用残片补偿旋转失败");

                ROS_WARN("OCR补偿旋转失败，本次不再复识，继续%s",
                         segment.name.c_str());
                return DETECTION_CONTINUE;
            }
            stopped_pose = recovery_pose;

            // 旋转后目标在图像中的位置会变化。优先用新一帧NanoDet框
            // 作为OCR关联参考；若该帧未返回框，OCR服务仍用上次框尝试。
            Box retry_reference = ocr.box;
            std::vector<Box> retry_boxes;
            if (detectBoxes(retry_boxes) && !retry_boxes.empty()) {
                const int retry_index = chooseClosestCenterBox(retry_boxes);
                if (retry_index >= 0) {
                    retry_reference = retry_boxes[
                        static_cast<std::size_t>(retry_index)];
                }
            } else {
                ROS_WARN("补偿旋转后NanoDet未返回文字框，"
                         "仍调用OCR完成唯一一次复识");
            }

            ocr = recognizeStaticTarget(retry_reference);
            if (ocr.category == "unknown") {
                if (boundary_estimate.valid &&
                    boundary_estimate.wall != segment.wall &&
                    !boundary_coordinate_recorded) {
                    ROS_WARN(
                        "V14.3保护：非当前巡检墙%s(%.3f, %.3f)"
                        "补偿复识后仍无法分类，不加入去重名单",
                        wallName(boundary_estimate.wall),
                        boundary_estimate.x, boundary_estimate.y);
                }
                addUnknownCandidate(
                    segment_index,
                    stopped_pose,
                    ocr.box,
                    boundary_estimate,
                    "OCR补偿旋转后仍为unknown");

                ROS_WARN("补偿旋转后的唯一一次复识仍无法分类：%s；"
                         "继续%s",
                         ocr.text.c_str(), segment.name.c_str());
                return DETECTION_CONTINUE;
            }
            ROS_INFO("补偿旋转复识成功：%s，分类=%s",
                     ocr.text.c_str(), categoryChinese(ocr.category));
        }
        if (ocr.category == "unknown") {
            if (boundary_estimate.valid &&
                boundary_estimate.wall != segment.wall &&
                !boundary_coordinate_recorded) {
                ROS_WARN(
                    "V14.3保护：非当前巡检墙%s(%.3f, %.3f)本次OCR无法分类，"
                    "不加入去重名单；恢复巡检后允许再次检测并重新OCR",
                    wallName(boundary_estimate.wall),
                    boundary_estimate.x, boundary_estimate.y);
            }
            addUnknownCandidate(
                segment_index,
                stopped_pose,
                ocr.box,
                boundary_estimate,
                "巡检OCR最终分类为unknown");

            ROS_WARN(
                "本次文字无法分类；已按V14.9记录为候选点，继续%s",
                segment.name.c_str());
            return DETECTION_CONTINUE;
        }

        // V14.3保护3：非当前巡检墙只有到这里（OCR最终分类成功）
        // 才加入不重复检测名单。
        if (boundary_estimate.valid &&
            boundary_estimate.wall != segment.wall &&
            !boundary_coordinate_recorded) {
            seen_board_coordinates_.push_back(boundary_estimate);
            boundary_coordinate_recorded = true;
            ROS_INFO(
                "非当前巡检墙候选OCR分类成功为%s；"
                "现在加入去重名单：%s(%.3f, %.3f)",
                categoryChinese(ocr.category),
                wallName(boundary_estimate.wall),
                boundary_estimate.x, boundary_estimate.y);
        }

        // 停车后的OCR框更接近静止图像，用它重新估计板在场地边界
        // 上的位置；失败时才退回触发本次停车的NanoDet框。
        BoardBoundaryEstimate static_board_estimate;
        bool estimate_ok =
            estimateBoardBoundary(stopped_pose,
                                  ocr.box, static_board_estimate);
        if (!estimate_ok && boundary_estimate.valid) {
            static_board_estimate = boundary_estimate;
            estimate_ok = true;
            ROS_WARN("静止OCR框无法计算板坐标，"
                     "改用NanoDet触发框的墙面交点");
        }

        TargetObservation observation;
        const int target_segment_index =
            estimate_ok ? segmentIndexForWall(static_board_estimate.wall) : -1;
        if (!estimate_ok ||
            target_segment_index < 0 ||
            !makeDockingObservation(
                target_segment_index, ocr.category,
                static_board_estimate, observation)) {
            ROS_WARN("无法由框中心生成距墙%.2fm的停靠导航点，"
                     "本次不使用小车当前位置，继续巡检等待重新检测",
                     docking_standoff_);
            return DETECTION_CONTINUE;
        }

        resolveUnknownCandidateNear(
            static_board_estimate,
            std::string("后续巡检已成功分类为") +
                categoryChinese(ocr.category));

        ROS_INFO("目标%s墙面坐标=%s(%.3f, %.3f)，"
                 "向场内延伸%.2fm后的move_base点=(%.3f, %.3f, %.1f度)",
                 categoryChinese(ocr.category),
                 wallName(static_board_estimate.wall),
                 static_board_estimate.x, static_board_estimate.y,
                 docking_standoff_,
                 observation.pose.x, observation.pose.y,
                 observation.pose.yaw * 180.0 / kPi);

        // V23判断严格使用“本次OCR识别后首次生成的停靠导航点”。
        // dockTarget()后续第二阶段视觉会把observation.pose更新成最终停靠点，
        // 但不会反过来改变这次是否满足<1m捷径条件。
        const double recognized_docking_x =
            observation.pose.x;

        const double recognized_docking_y =
            observation.pose.y;

        const double recognized_docking_to_segment_end =
            distance2D(
                recognized_docking_x,
                recognized_docking_y,
                segment.end_x,
                segment.end_y);

        const bool target_on_current_left_wall =
            static_board_estimate.wall == segment.wall;
        if (!target_on_current_left_wall) {
            ROS_INFO("该目标位于%s，不是当前车体左侧正在扫描的%s",
                     wallName(static_board_estimate.wall),
                     wallName(segment.wall));
        }

        if (ocr.category == simulation_target_category_) {
            // 显式记录“仿真目标待停靠”状态。后续不再只依赖
            // simulation_observation_.valid在单个分支中临时跳转。
            // 保持旧逻辑：仿真目标先于现实目标出现时，只保存第一次
            // 有效记录，避免后续误识别覆盖已经确认的停靠点。
            if (!simulation_observation_.valid) {
                simulation_observation_ = observation;
                // 只对首次有效记录绑定“是否必须等本段结束”。
                // 后续重复/误识别不得覆盖已经确认的目标与保护状态。
                // V14.4例外：
                // 非当前巡检墙仿真目标通常需要等当前整面墙结束；
                // 但如果现实目标已经完成停靠，则双目标任务只剩仿真目标，
                // 此时允许立即中断当前墙直接前往，不再设置段末锁。
                simulation_target_blocked_until_segment_end_ =
                    !target_on_current_left_wall && !real_docked_;
            }
            simulation_target_pending_ = true;

            ROS_INFO(
                "已记录仿真目标%s并置为待停靠："
                "墙面估计坐标%s(%.3f, %.3f)，"
                "对应停靠导航点(%.3f, %.3f, %.1f度)",
                categoryChinese(ocr.category),
                wallName(static_board_estimate.wall),
                static_board_estimate.x,
                static_board_estimate.y,
                simulation_observation_.pose.x,
                simulation_observation_.pose.y,
                simulation_observation_.pose.yaw * 180.0 / kPi);

            if (simulation_target_blocked_until_segment_end_) {
                ROS_WARN(
                    "V14.4保护：仿真目标位于非当前巡检墙%s，且现实目标尚未停靠；"
                    "必须先完整巡检完当前%s，段末才允许前往该仿真目标。",
                    wallName(static_board_estimate.wall),
                    segment.name.c_str());
                return DETECTION_CONTINUE;
            }

            if (!real_docked_) {
                ROS_INFO(
                    "仿真目标已记录，但现实目标尚未停靠；"
                    "继续巡检寻找现实目标。");
                return DETECTION_CONTINUE;
            }

            if (!target_on_current_left_wall) {
                ROS_WARN(
                    "V14.4例外生效：现实目标已经完成停靠，"
                    "当前识别到的仿真目标位于非当前巡检墙%s；"
                    "允许立即中断%s并直接前往仿真目标。",
                    wallName(static_board_estimate.wall),
                    segment.name.c_str());
            } else {
                ROS_INFO(
                    "现实目标已经完成停靠，当前巡检墙识别到仿真目标；"
                    "立即前往停靠。");
            }

            if (!dockPendingSimulationTarget()) {
                return DETECTION_ABORT;
            }
            return DETECTION_MISSION_COMPLETE;
        }

        if (ocr.category == real_target_category_) {
            if (real_docked_) {
                ROS_INFO("现实目标已经完成停靠，忽略重复识别");
                return DETECTION_CONTINUE;
            }

            if (!target_on_current_left_wall) {
                if (!real_target_pending_) {
                    real_observation_ = observation;
                    real_target_pending_ = true;
                    real_target_defer_segment_index_ =
                        static_cast<int>(segment_index);

                    real_target_resume_segment_index_ =
                        target_segment_index;

                    // V24这里使用“目标板本身的墙面坐标”，
                    // 而不是0.5m内缩后的停靠导航点。
                    real_target_distance_to_source_segment_end_ =
                        distance2D(
                            static_board_estimate.x,
                            static_board_estimate.y,
                            segment.end_x,
                            segment.end_y);

                    real_target_near_end_early_dock_eligible_ =
                        patrol_noncurrent_target_early_dock_distance_ > 0.0
                        && real_target_distance_to_source_segment_end_
                            < patrol_noncurrent_target_early_dock_distance_
                        && real_target_resume_segment_index_ >= 0
                        && real_target_resume_segment_index_
                            != static_cast<int>(segment_index);

                    ROS_INFO(
                        "现实目标位于非当前左墙，已保存停靠点"
                        "(%.3f, %.3f, %.1f度)；"
                        "目标板坐标=%s(%.3f,%.3f)，"
                        "距当前%s终点=%.3fm，提前停靠阈值=%.3fm -> %s。",
                        observation.pose.x,
                        observation.pose.y,
                        observation.pose.yaw * 180.0 / kPi,
                        wallName(static_board_estimate.wall),
                        static_board_estimate.x,
                        static_board_estimate.y,
                        segment.name.c_str(),
                        real_target_distance_to_source_segment_end_,
                        patrol_noncurrent_target_early_dock_distance_,
                        real_target_near_end_early_dock_eligible_
                            ? "进入V24待提前停靠状态"
                            : "保持原段末延后停靠");

                    if (
                        real_target_near_end_early_dock_eligible_
                    ) {
                        ROS_WARN(
                            "V24：该非当前墙现实目标位于本段终点1m范围内；"
                            "当前先继续%s，"
                            "当机器人自身进入终点1m范围后立即去停靠，"
                            "随后直接并入%s继续巡检。",
                            segment.name.c_str(),
                            segments_[
                                static_cast<std::size_t>(
                                    real_target_resume_segment_index_)
                            ].name.c_str());
                    } else {
                        ROS_INFO(
                            "目标不满足V24提前停靠条件；"
                            "继续完成%s整段扫描，沿用原段末停靠逻辑。",
                            segment.name.c_str());
                    }
                } else {
                    ROS_INFO(
                        "已有一个待处理的非当前左墙现实目标，"
                        "保留首次有效坐标和V24状态并继续当前边界扫描");
                }
                return DETECTION_CONTINUE;
            }

            if (real_target_pending_) {
                ROS_INFO("现实目标已作为非当前左墙目标记录，"
                         "等待本段扫描完成后统一停靠");
                return DETECTION_CONTINUE;
            }

            real_observation_ = observation;
            releaseCameraBeforePredockNavigation("现实目标");
            if (!navigateToPose(observation.pose.x,
                                observation.pose.y,
                                observation.pose.yaw,
                                "前往现实目标停靠导航点")) {
                return DETECTION_ABORT;
            }
            if (!dockTarget(real_observation_, "现实目标", false)) {
                return DETECTION_ABORT;
            }
            real_docked_ = true;
            notifyRealDockedOnce();

            if (hasPendingSimulationTarget() &&
                !simulation_target_blocked_until_segment_end_) {
                ROS_INFO("仿真目标此前已找到且不受段末保护；"
                         "现实目标停靠完成后立即处理，不恢复巡检");
                if (!dockPendingSimulationTarget()) {
                    return DETECTION_ABORT;
                }
                return DETECTION_MISSION_COMPLETE;
            }

            if (hasPendingSimulationTarget() &&
                simulation_target_blocked_until_segment_end_) {
                ROS_WARN(
                    "V14.3保护：仿真目标此前虽已找到，但属于非当前巡检墙；"
                    "现实目标停靠完成后仍需要到达当前段末，"
                    "段末才能解除仿真目标保护。");
            } else {
                ROS_INFO(
                    "尚未记录可立即处理的仿真目标；"
                    "判断是否需要执行V23段末停靠捷径。");
            }

            const bool can_use_near_end_shortcut =
                segment_index < 3
                && patrol_near_end_corner_shortcut_distance_ > 0.0
                && recognized_docking_to_segment_end
                    < patrol_near_end_corner_shortcut_distance_;

            ROS_INFO(
                "V23段末捷径判定："
                "识别停靠点=(%.3f,%.3f)，"
                "%s终点=(%.3f,%.3f)，距离=%.3fm，阈值=%.3fm，"
                "本段%s后续角点 -> %s。",
                recognized_docking_x,
                recognized_docking_y,
                segment.name.c_str(),
                segment.end_x,
                segment.end_y,
                recognized_docking_to_segment_end,
                patrol_near_end_corner_shortcut_distance_,
                segment_index < 3 ? "存在" : "不存在",
                can_use_near_end_shortcut
                    ? "直接执行角点旋转平移"
                    : "保持V17回线+剩余固定Path");

            if (can_use_near_end_shortcut) {
                if (!runNearEndCornerShortcut(
                        segment_index,
                        segment,
                        recognized_docking_x,
                        recognized_docking_y
                    )) {
                    return DETECTION_ABORT;
                }

                return
                    DETECTION_SEGMENT_COMPLETE_AFTER_CORNER;
            }

            if (!returnToPatrolLineSafely(segment)) {
                return DETECTION_ABORT;
            }
            if (!openCamera()) return DETECTION_ABORT;
            ROS_INFO("V17已快速恢复到%s：位置与巡检航向均已收敛；"
                     "后续startPatrolFromPose正常启动剩余固定Path",
                     segment.name.c_str());
            return DETECTION_CONTINUE;
        }

        ROS_INFO("OCR结果%s既不是现实目标也不是仿真目标，直接忽略",
                 categoryChinese(ocr.category));
        return DETECTION_CONTINUE;
    }

    bool hasNearEndDeferredRealTarget(
        std::size_t source_segment_index
    ) const {
        return (
            real_target_pending_
            && real_observation_.valid
            && !real_docked_
            && real_target_near_end_early_dock_eligible_
            && real_target_defer_segment_index_
                == static_cast<int>(source_segment_index)
            && real_target_resume_segment_index_ >= 0
            && real_target_resume_segment_index_
                < static_cast<int>(segments_.size())
            && real_target_resume_segment_index_
                != static_cast<int>(source_segment_index)
        );
    }

    SegmentResult dockNearEndDeferredRealTargetAndSwitchWall(
        std::size_t source_segment_index
    ) {
        if (!hasNearEndDeferredRealTarget(
                source_segment_index
            )) {
            ROS_ERROR(
                "V24提前停靠状态无效：source_segment=%zu，"
                "pending=%s eligible=%s resume_segment=%d",
                source_segment_index,
                real_target_pending_ ? "是" : "否",
                real_target_near_end_early_dock_eligible_
                    ? "是" : "否",
                real_target_resume_segment_index_);

            return SEGMENT_ABORTED;
        }

        const int target_segment_index =
            real_target_resume_segment_index_;

        const Segment& source_segment =
            segments_[source_segment_index];

        const Segment& target_segment =
            segments_[
                static_cast<std::size_t>(
                    target_segment_index)];

        ROS_WARN(
            "V24开始提前停靠非当前墙现实目标："
            "当前段=%s，目标墙段=%s，"
            "停靠导航点=(%.3f,%.3f,%.1f°)。",
            source_segment.name.c_str(),
            target_segment.name.c_str(),
            real_observation_.pose.x,
            real_observation_.pose.y,
            real_observation_.pose.yaw
                * 180.0 / kPi);

        patrol_checkpoint_.valid = false;
        clearPatrolVisualApproach(false);

        releaseCameraBeforePredockNavigation(
            "V24提前处理的非当前墙现实目标");

        if (!navigateToPose(
                real_observation_.pose.x,
                real_observation_.pose.y,
                real_observation_.pose.yaw,
                "V24前往非当前墙现实目标"
            )) {
            return SEGMENT_ABORTED;
        }

        if (!dockTarget(
                real_observation_,
                "V24提前处理的非当前墙现实目标",
                false
            )) {
            return SEGMENT_ABORTED;
        }

        real_docked_ = true;
        notifyRealDockedOnce();

        real_target_pending_ = false;
        real_target_defer_segment_index_ = -1;
        real_target_near_end_early_dock_eligible_ = false;
        real_target_distance_to_source_segment_end_ =
            std::numeric_limits<double>::infinity();
        real_target_resume_segment_index_ = -1;

        // 当前巡检段虽然没有跑到精确终点，但V24明确允许在终点1m范围
        // 提前结束并切入目标墙，因此原来“非当前仿真目标必须等本段结束”
        // 的段内锁在这里等价视为已经到达允许释放时机。
        if (simulation_target_blocked_until_segment_end_) {
            simulation_target_blocked_until_segment_end_ = false;

            ROS_WARN(
                "V24已提前结束%s并切换到%s；"
                "此前绑定当前段的仿真目标段末保护同步解除。",
                source_segment.name.c_str(),
                target_segment.name.c_str());
        }

        // 保持原有优先级：
        // 现实目标完成后，如果此前已经记录了可处理的仿真目标，
        // 仍然立即处理仿真目标并结束双目标实体阶段。
        if (hasPendingSimulationTarget()) {
            ROS_INFO(
                "V24现实目标停靠完成；"
                "此前已记录仿真目标，沿用原逻辑立即前往仿真目标。");

            if (!dockPendingSimulationTarget()) {
                return SEGMENT_ABORTED;
            }

            return SEGMENT_MISSION_COMPLETE;
        }

        // 用户要求：停靠完成后“直接从当前位置开始继续巡检目标板所在墙”。
        // 这里不返回目标墙起点，不执行旧角点。
        // 只用V17把当前停靠姿态旋转/横移到目标墙最近巡检线位置，
        // 外层随后用startPatrolFromPose从这个当前位置对应进度继续剩余Path。
        ROS_WARN(
            "V24现实目标停靠完成，"
            "不返回%s终点、不执行其角点过渡；"
            "从当前位置直接并入目标板所在%s继续巡检。",
            source_segment.name.c_str(),
            target_segment.name.c_str());

        if (!returnToPatrolLineSafely(
                target_segment
            )) {
            return SEGMENT_ABORTED;
        }

        if (!openCamera()) {
            return SEGMENT_ABORTED;
        }

        forced_next_segment_index_ =
            target_segment_index;

        ROS_INFO(
            "V24已从现实目标停靠位置并入%s；"
            "下一步将从当前位置进度直接启动该墙剩余固定Path。",
            target_segment.name.c_str());

        return SEGMENT_SWITCH_TO_TARGET_WALL;
    }

    bool hasDeferredRealTargetAfterSegment(
            std::size_t segment_index) const {
        return real_target_pending_ &&
               real_observation_.valid &&
               !real_docked_ &&
               real_target_defer_segment_index_ ==
                   static_cast<int>(segment_index);
    }

    DetectionResult dockDeferredRealTargetAfterSegment(
            std::size_t completed_segment_index) {
        if (!hasDeferredRealTargetAfterSegment(
                completed_segment_index)) {
            ROS_ERROR("待处理现实目标状态无效");
            return DETECTION_ABORT;
        }

        ROS_INFO("当前左侧边界已经完整扫描，"
                 "现在前往此前记录的非当前左墙现实目标："
                 "(%.3f, %.3f, %.1f度)",
                 real_observation_.pose.x,
                 real_observation_.pose.y,
                 real_observation_.pose.yaw * 180.0 / kPi);
        releaseCameraBeforePredockNavigation("延后处理的现实目标");
        if (!navigateToPose(real_observation_.pose.x,
                            real_observation_.pose.y,
                            real_observation_.pose.yaw,
                            "前往延后处理的现实目标")) {
            return DETECTION_ABORT;
        }
        if (!dockTarget(real_observation_,
                        "延后处理的现实目标", false)) {
            return DETECTION_ABORT;
        }

        real_docked_ = true;
        notifyRealDockedOnce();
        real_target_pending_ = false;
        real_target_defer_segment_index_ = -1;

        // 无论是V24没触发还是目标本身不满足提前条件，
        // 一旦走到原段末延后停靠流程，清理全部V24临时状态。
        real_target_near_end_early_dock_eligible_ = false;
        real_target_distance_to_source_segment_end_ =
            std::numeric_limits<double>::infinity();
        real_target_resume_segment_index_ = -1;

        if (hasPendingSimulationTarget()) {
            ROS_INFO("现实目标停靠完成，立即前往此前记录的仿真目标");
            if (!dockPendingSimulationTarget()) {
                return DETECTION_ABORT;
            }
            return DETECTION_MISSION_COMPLETE;
        }

        const std::size_t next_segment_index =
            completed_segment_index + 1;
        if (next_segment_index < segments_.size()) {
            const Segment& next_segment =
                segments_[next_segment_index];
            if (real_observation_.segment_index ==
                static_cast<int>(next_segment_index)) {
                ROS_INFO("尚未找到仿真目标；从现实目标处执行V17快速恢复："
                         "先转到下一条巡检方向，再低速横移到下一条巡检线");
                if (!returnToPatrolLineSafely(next_segment)) {
                    return DETECTION_ABORT;
                }
            } else {
                ROS_WARN("延后现实目标不在紧邻的下一面墙，"
                         "改用move_base返回下一段起点");
                if (!navigateToPose(next_segment.start_x,
                                    next_segment.start_y,
                                    next_segment.travel_yaw,
                                    "返回下一段巡检起点")) {
                    return DETECTION_ABORT;
                }
            }
            if (!openCamera()) return DETECTION_ABORT;
            ROS_INFO("已准备进入%s，继续寻找仿真目标",
                     next_segment.name.c_str());
        }
        return DETECTION_CONTINUE;
    }

    bool hasRecordedSimulationTarget() const {
        return simulation_target_pending_ &&
               simulation_observation_.valid &&
               !simulation_docked_;
    }

    bool hasPendingSimulationTarget() const {
        // 正常巡检流程保持原规则：
        // 必须现实目标已经停靠，才允许立即处理仿真目标。
        return real_docked_ &&
               hasRecordedSimulationTarget();
    }

    bool dockPendingSimulationTarget(
        bool allow_without_real = false
    ) {
        if (
            simulation_target_blocked_until_segment_end_
            && !allow_without_real
        ) {
            ROS_ERROR(
                "V14.4保护拒绝停靠：该仿真目标是在现实目标尚未停靠时于"
                "非当前巡检墙被识别，当前整面墙尚未完成，禁止提前调用"
                "dockPendingSimulationTarget()");
            return false;
        }

        if (allow_without_real) {
            // 只有四墙巡检和unknown回访都已经结束后才会使用这个入口。
            // 此时明确放弃未找到的现实目标，因此旧的“必须等现实目标”
            // 和“非当前墙必须等段末”保护都不再需要。
            simulation_target_blocked_until_segment_end_ = false;
        }

        const bool pending_valid =
            allow_without_real
            ? hasRecordedSimulationTarget()
            : hasPendingSimulationTarget();

        if (!pending_valid) {
            ROS_ERROR(
                "待停靠仿真目标状态无效："
                "允许无现实目标=%s，现实停靠=%s，"
                "仿真记录=%s，待停靠=%s，仿真停靠=%s",
                allow_without_real ? "是" : "否",
                real_docked_ ? "是" : "否",
                simulation_observation_.valid ? "是" : "否",
                simulation_target_pending_ ? "是" : "否",
                simulation_docked_ ? "是" : "否"
            );
            return false;
        }

        ROS_INFO("前往已记录仿真目标：导航点=(%.3f, %.3f, %.1f度)",
                 simulation_observation_.pose.x,
                 simulation_observation_.pose.y,
                 simulation_observation_.pose.yaw * 180.0 / kPi);

        releaseCameraBeforePredockNavigation("仿真目标");
        if (!navigateToPose(simulation_observation_.pose.x,
                            simulation_observation_.pose.y,
                            simulation_observation_.pose.yaw,
                            "前往已记录的仿真目标")) {
            return false;
        }

        if (!dockTarget(simulation_observation_,
                        "仿真目标", false)) {
            return false;
        }

        simulation_docked_ = true;
        simulation_target_pending_ = false;
        simulation_target_blocked_until_segment_end_ = false;
        if (real_docked_) {
            ROS_INFO("已完成待停靠仿真目标，现实/仿真两个实体停靠均已完成");
        } else {
            ROS_WARN(
                "已完成仿真目标停靠；现实目标最终未找到，"
                "已按兜底规则放弃现实目标实体停靠"
            );
        }
        return true;
    }

    bool rotateToDockingRecoveryYaw(double target_yaw,
                                    const std::string& action_name) {
        const double tolerance =
            ocr_recovery_turn_tolerance_deg_ * kPi / 180.0;
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(ocr_recovery_turn_timeout_);
        int stable_frames = 0;
        ros::Rate rate(20.0);

        ROS_INFO("%s：目标朝向=%.1f度",
                 action_name.c_str(), target_yaw * 180.0 / kPi);
        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                stable_frames = 0;
                rate.sleep();
                continue;
            }

            const double yaw_error = normalizeAngle(target_yaw - pose.yaw);
            if (std::fabs(yaw_error) <= tolerance) {
                publishVelocity(0.0, 0.0, 0.0);
                ++stable_frames;
                if (stable_frames >= ocr_recovery_turn_stable_frames_) {
                    stopRobot();
                    Pose2D settled_pose = pose;
                    getRobotPose(settled_pose);
                    ROS_INFO("%s完成：最终朝向=%.1f度，误差=%.2f度",
                             action_name.c_str(),
                             settled_pose.yaw * 180.0 / kPi,
                             normalizeAngle(
                                 target_yaw - settled_pose.yaw) *
                                 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
                double angular_z = clampValue(
                    ocr_recovery_turn_kp_ * yaw_error,
                    -ocr_recovery_turn_max_speed_,
                    ocr_recovery_turn_max_speed_);
                if (std::fabs(angular_z) <
                    ocr_recovery_turn_min_speed_) {
                    angular_z = yaw_error >= 0.0
                                    ? ocr_recovery_turn_min_speed_
                                    : -ocr_recovery_turn_min_speed_;
                }
                publishVelocity(0.0, 0.0, angular_z);
                ROS_INFO_THROTTLE(
                    0.5,
                    "%s：剩余角度=%.2f度，angular.z=%.3f",
                    action_name.c_str(),
                    yaw_error * 180.0 / kPi, angular_z);
            }
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("%s在%.1f秒内未完成",
                  action_name.c_str(), ocr_recovery_turn_timeout_);
        return false;
    }

    bool refreshDockingDetectionFrames(const std::string& reason) {
        if (!openCamera()) {
            ROS_ERROR("[停靠诊断][帧刷新] %s：摄像头未能打开",
                      reason.c_str());
            return false;
        }

        // V14.2：这里仅用于±30度恢复旋转后的刷新。
        // NanoDet的detect_start=-3内部连续cap.grab()两帧；
        // 直接连续调用，不额外sleep、不再读取并丢弃正常检测帧。
        ROS_WARN(
            "[停靠诊断][帧刷新] %s：即时清缓存开始，"
            "clear_calls=%d，无固定等待、无额外丢弃帧",
            reason.c_str(), docking_refresh_clear_calls_);

        for (int i = 0; i < docking_refresh_clear_calls_; ++i) {
            ros_nanodet::detect_result_srv clear_service;
            clear_service.request.detect_start = -3;
            if (!detect_client_.call(clear_service)) {
                ROS_ERROR(
                    "[停靠诊断][帧刷新] %s：第%d/%d次-3清缓存调用失败",
                    reason.c_str(), i + 1, docking_refresh_clear_calls_);
                return false;
            }
        }

        ROS_WARN(
            "[停靠诊断][帧刷新] %s：即时清缓存完成；"
            "下一次detect直接作为正式计算帧",
            reason.c_str());
        return true;
    }

    double boardCoordinateDistance(
            const TargetObservation& observation,
            const BoardBoundaryEstimate& estimate) const {
        if (!observation.board_valid ||
            observation.board_wall != estimate.wall) {
            return std::numeric_limits<double>::infinity();
        }
        return distance2D(observation.board_x, observation.board_y,
                          estimate.x, estimate.y);
    }

    bool detectDockingRefinedObservation(
            const TargetObservation& old_observation,
            const std::string& scan_name,
            double final_standoff,
            TargetObservation& refined_observation,
            bool force_refresh,
            Box* matched_box_out = nullptr,
            BoardBoundaryEstimate* matched_board_out = nullptr,
            bool match_only = false) {
        // 正常第二段：第一段导航前已关闭摄像头，到点后刚重新打开，
        // 不存在旧会话缓存，因此立即使用正式检测帧。
        // ±30度恢复：旋转期间相机保持打开但没有持续read，才需要-3即时刷新。
        if (force_refresh &&
            !refreshDockingDetectionFrames(scan_name)) {
            ROS_ERROR("%s刷新NanoDet实时帧失败", scan_name.c_str());
            return false;
        }

        for (int attempt = 0;
             attempt < docking_recovery_detection_attempts_ && ros::ok();
             ++attempt) {
            ros::spinOnce();
            std::vector<Box> boxes;
            const bool detection_ok = detectBoxes(boxes);
            if (!detection_ok || boxes.empty()) {
                ROS_WARN("%s第%d/%d次未检测到文字板",
                         scan_name.c_str(), attempt + 1,
                         docking_recovery_detection_attempts_);
                ros::Duration(
                    docking_recovery_detection_interval_).sleep();
                continue;
            }

            Pose2D pose;
            if (!getRobotPose(pose)) {
                ROS_WARN("%s第%d/%d次检测到框，但无法读取机器人位姿",
                         scan_name.c_str(), attempt + 1,
                         docking_recovery_detection_attempts_);
                ros::Duration(
                    docking_recovery_detection_interval_).sleep();
                continue;
            }

            const double image_center =
                0.5 * static_cast<double>(image_width_);
            ROS_WARN(
                "[停靠诊断][本帧] %s attempt=%d/%d："
                "小车map坐标=(%.4f, %.4f, %.2f度)，"
                "第一段预停靠目标=(%.4f, %.4f, %.2f度)，"
                "第一次板坐标=%s(%.4f, %.4f)，检测框数=%d",
                scan_name.c_str(), attempt + 1,
                docking_recovery_detection_attempts_,
                pose.x, pose.y, pose.yaw * 180.0 / kPi,
                old_observation.pose.x, old_observation.pose.y,
                old_observation.pose.yaw * 180.0 / kPi,
                old_observation.board_valid
                    ? wallName(old_observation.board_wall)
                    : "未知墙",
                old_observation.board_x, old_observation.board_y,
                static_cast<int>(boxes.size()));

            int selected = -1;
            double best_score = std::numeric_limits<double>::infinity();
            BoardBoundaryEstimate best_estimate;
            for (std::size_t i = 0; i < boxes.size(); ++i) {
                const double pixel_error =
                    image_center - boxes[i].centerX();
                const double relative_yaw =
                    std::atan2(pixel_error, camera_fx_);
                const double ray_yaw =
                    pose.yaw +
                    camera_yaw_offset_deg_ * kPi / 180.0 +
                    relative_yaw;

                BoardBoundaryEstimate estimate;
                if (!estimateBoardBoundary(pose, boxes[i], estimate)) {
                    ROS_WARN(
                        "[停靠诊断][候选%d] 框=(%d,%d)-(%d,%d)，"
                        "center=(%.1f,%.1f)，pixel_error=%.1fpx，"
                        "relative_yaw=%.2f度，ray_yaw=%.2f度；"
                        "墙面射线无有效交点",
                        static_cast<int>(i),
                        boxes[i].x0, boxes[i].y0,
                        boxes[i].x1, boxes[i].y1,
                        boxes[i].centerX(), boxes[i].centerY(),
                        pixel_error,
                        relative_yaw * 180.0 / kPi,
                        ray_yaw * 180.0 / kPi);
                    continue;
                }

                const double old_distance =
                    old_observation.board_valid
                        ? boardCoordinateDistance(old_observation, estimate)
                        : std::numeric_limits<double>::infinity();
                if (std::isfinite(old_distance)) {
                    ROS_WARN(
                        "[停靠诊断][候选%d] 框=(%d,%d)-(%d,%d)，"
                        "center=(%.1f,%.1f)，pixel_error=%.1fpx[%s]，"
                        "relative_yaw=%.2f度，ray_yaw=%.2f度；"
                        "计算板坐标=%s(%.4f,%.4f)，"
                        "相对第一次板坐标距离=%.4fm",
                        static_cast<int>(i),
                        boxes[i].x0, boxes[i].y0,
                        boxes[i].x1, boxes[i].y1,
                        boxes[i].centerX(), boxes[i].centerY(),
                        pixel_error,
                        pixel_error < 0.0 ? "板在画面右侧"
                                          : (pixel_error > 0.0
                                                 ? "板在画面左侧"
                                                 : "板在画面中线"),
                        relative_yaw * 180.0 / kPi,
                        ray_yaw * 180.0 / kPi,
                        wallName(estimate.wall),
                        estimate.x, estimate.y, old_distance);
                } else {
                    ROS_WARN(
                        "[停靠诊断][候选%d] 框=(%d,%d)-(%d,%d)，"
                        "center=(%.1f,%.1f)，pixel_error=%.1fpx[%s]，"
                        "relative_yaw=%.2f度，ray_yaw=%.2f度；"
                        "计算板坐标=%s(%.4f,%.4f)，"
                        "与第一次板不在同墙/无可比距离",
                        static_cast<int>(i),
                        boxes[i].x0, boxes[i].y0,
                        boxes[i].x1, boxes[i].y1,
                        boxes[i].centerX(), boxes[i].centerY(),
                        pixel_error,
                        pixel_error < 0.0 ? "板在画面右侧"
                                          : (pixel_error > 0.0
                                                 ? "板在画面左侧"
                                                 : "板在画面中线"),
                        relative_yaw * 180.0 / kPi,
                        ray_yaw * 180.0 / kPi,
                        wallName(estimate.wall),
                        estimate.x, estimate.y);
                }

                double score = 0.0;
                if (old_observation.board_valid) {
                    // 第二阶段只允许关联第一次确认的同一面墙。
                    // 这一步替代旧版“挑画面中心最近框”的不可靠策略。
                    if (estimate.wall != old_observation.board_wall) {
                        continue;
                    }
                    score = boardCoordinateDistance(
                        old_observation, estimate);
                    if (!std::isfinite(score) ||
                        score > docking_refine_max_board_shift_) {
                        continue;
                    }
                } else {
                    // 仅为兼容历史状态；正常新记录一定有board坐标。
                    score = std::fabs(
                        boxes[i].centerX() - 0.5 * image_width_);
                }

                if (score < best_score) {
                    best_score = score;
                    selected = static_cast<int>(i);
                    best_estimate = estimate;
                }
            }

            if (selected >= 0) {
                const Box& box = boxes[
                    static_cast<std::size_t>(selected)];

                if (matched_box_out) {
                    *matched_box_out = box;
                }
                if (matched_board_out) {
                    *matched_board_out = best_estimate;
                }

                // V14.6边缘视野预检查：
                // 此模式只确认“是不是原来那块板”并返回bbox/墙面坐标，
                // 严格不生成最终停靠点。若框位于左右1/4，调用方会先旋转。
                if (match_only) {
                    ROS_WARN(
                        "[停靠诊断][边缘预检查] %s匹配到原目标："
                        "center=(%.1f,%.1f)，板=%s(%.4f,%.4f)，"
                        "相对第一次板坐标修正=%.4fm；"
                        "当前尚未计算最终停靠点。",
                        scan_name.c_str(),
                        box.centerX(), box.centerY(),
                        wallName(best_estimate.wall),
                        best_estimate.x, best_estimate.y,
                        old_observation.board_valid ? best_score : 0.0);
                    return true;
                }

                const int segment_index =
                    segmentIndexForWall(best_estimate.wall);
                if (segment_index >= 0 &&
                    makeDockingObservationAtStandoff(
                        segment_index,
                        old_observation.category,
                        best_estimate,
                        final_standoff,
                        refined_observation)) {
                    const double pixel_error =
                        image_center - box.centerX();
                    const double world_dx =
                        refined_observation.pose.x - pose.x;
                    const double world_dy =
                        refined_observation.pose.y - pose.y;
                    const double c = std::cos(pose.yaw);
                    const double sn = std::sin(pose.yaw);
                    const double base_dx =
                        c * world_dx + sn * world_dy;
                    const double base_dy =
                        -sn * world_dx + c * world_dy;

                    ROS_WARN(
                        "[停靠诊断][最终计算] ===== %s =====",
                        scan_name.c_str());
                    ROS_WARN(
                        "[停靠诊断][最终计算] 小车实际map坐标="
                        "(%.4f, %.4f, %.2f度)",
                        pose.x, pose.y, pose.yaw * 180.0 / kPi);
                    ROS_WARN(
                        "[停靠诊断][最终计算] 第一段预停靠目标="
                        "(%.4f, %.4f, %.2f度)；实际到点误差="
                        "dx=%.4f, dy=%.4f, dist=%.4fm",
                        old_observation.pose.x,
                        old_observation.pose.y,
                        old_observation.pose.yaw * 180.0 / kPi,
                        pose.x - old_observation.pose.x,
                        pose.y - old_observation.pose.y,
                        distance2D(pose.x, pose.y,
                                   old_observation.pose.x,
                                   old_observation.pose.y));
                    ROS_WARN(
                        "[停靠诊断][最终计算] 选中框center=(%.1f,%.1f)，"
                        "image_center=%.1f，pixel_error=%.1fpx[%s]",
                        box.centerX(), box.centerY(),
                        image_center, pixel_error,
                        pixel_error < 0.0 ? "板在画面右侧"
                                          : (pixel_error > 0.0
                                                 ? "板在画面左侧"
                                                 : "板在画面中线"));
                    ROS_WARN(
                        "[停靠诊断][最终计算] 第一次板位置="
                        "%s(%.4f, %.4f)；第二次计算板位置="
                        "%s(%.4f, %.4f)；板坐标修正距离=%.4fm",
                        old_observation.board_valid
                            ? wallName(old_observation.board_wall)
                            : "未知墙",
                        old_observation.board_x,
                        old_observation.board_y,
                        wallName(best_estimate.wall),
                        best_estimate.x, best_estimate.y,
                        old_observation.board_valid ? best_score : 0.0);
                    ROS_WARN(
                        "[停靠诊断][最终计算] approach_stop_distance="
                        "%.4fm；最终move_base停靠坐标="
                        "(%.4f, %.4f, %.2f度)",
                        final_standoff,
                        refined_observation.pose.x,
                        refined_observation.pose.y,
                        refined_observation.pose.yaw * 180.0 / kPi);
                    ROS_WARN(
                        "[停靠诊断][最终计算] 从当前小车到最终点："
                        "map增量=(dx=%.4f,dy=%.4f)，"
                        "车体坐标增量=(forward=%.4f,left=%.4f)，"
                        "距离=%.4fm",
                        world_dx, world_dy, base_dx, base_dy,
                        std::hypot(world_dx, world_dy));
                    ROS_WARN(
                        "[停靠诊断][最终计算] ============================");

                    ROS_INFO(
                        "%s成功：框中心=(%.1f, %.1f)，"
                        "旧墙面坐标=%s(%.3f, %.3f)，"
                        "新墙面坐标=%s(%.3f, %.3f)，修正量=%.3fm",
                        scan_name.c_str(),
                        box.centerX(), box.centerY(),
                        old_observation.board_valid
                            ? wallName(old_observation.board_wall)
                            : "未知墙",
                        old_observation.board_x,
                        old_observation.board_y,
                        wallName(best_estimate.wall),
                        best_estimate.x,
                        best_estimate.y,
                        old_observation.board_valid ? best_score : 0.0);
                    return true;
                }
            }

            ROS_WARN(
                "%s第%d/%d次检测到%d个框，但没有与旧目标同墙且"
                "墙面坐标误差<=%.2fm的有效框",
                scan_name.c_str(), attempt + 1,
                docking_recovery_detection_attempts_,
                static_cast<int>(boxes.size()),
                docking_refine_max_board_shift_);
            ros::Duration(
                docking_recovery_detection_interval_).sleep();
        }
        return false;
    }

    bool recoverDockingObservation(TargetObservation& observation,
                                   const std::string& target_name,
                                   double final_standoff) {
        const TargetObservation old_observation = observation;
        const double center_yaw = old_observation.pose.yaw;
        const double turn = docking_recovery_turn_deg_ * kPi / 180.0;
        const double scan_yaws[2] = {
            normalizeAngle(center_yaw + turn),
            normalizeAngle(center_yaw - turn)};
        const char* scan_names[2] = {
            "左侧恢复识别", "右侧恢复识别"};

        ROS_WARN("%s在预停靠点无法可靠复定位原目标，开始左右视野恢复；"
                 "基准朝向=%.1f度",
                 target_name.c_str(), center_yaw * 180.0 / kPi);
        for (int side = 0; side < 2 && ros::ok(); ++side) {
            if (!rotateToDockingRecoveryYaw(
                    scan_yaws[side], scan_names[side])) {
                ROS_WARN("%s旋转失败，继续尝试下一恢复方向",
                         scan_names[side]);
                continue;
            }

            TargetObservation recovered;
            if (!detectDockingRefinedObservation(
                    old_observation, scan_names[side],
                    final_standoff, recovered, true)) {
                ROS_WARN("%s未找到与旧目标匹配的文字板",
                         scan_names[side]);
                continue;
            }

            observation = recovered;
            ROS_WARN(
                "%s恢复识别成功：最终墙面坐标=%s(%.3f, %.3f)，"
                "最终move_base点=(%.3f, %.3f, %.1f度)",
                target_name.c_str(),
                wallName(observation.board_wall),
                observation.board_x, observation.board_y,
                observation.pose.x, observation.pose.y,
                observation.pose.yaw * 180.0 / kPi);
            return true;
        }

        stopRobot();
        ROS_ERROR("%s左右%.1f度恢复识别均未找到原目标板",
                  target_name.c_str(), docking_recovery_turn_deg_);
        return false;
    }

    bool dockTarget(TargetObservation& observation,
                    const std::string& target_name,
                    bool camera_is_already_open) {
        if (!camera_is_already_open && !openCamera()) return false;

        ROS_INFO(
            "%s开始V14.2两段式停靠：第一段预停靠已完成；"
            "摄像头刚从全新V4L2会话打开，立即进行第二段正式视觉定位，"
            "不再固定等待、不再丢弃正常检测帧。",
            target_name.c_str());

        Pose2D before_refine_pose;
        if (getRobotPose(before_refine_pose)) {
            ROS_WARN(
                "[停靠诊断][第二段入口] 小车实际map坐标="
                "(%.4f, %.4f, %.2f度)；"
                "第一段预停靠目标=(%.4f, %.4f, %.2f度)；"
                "第一次板位置=%s(%.4f, %.4f)",
                before_refine_pose.x, before_refine_pose.y,
                before_refine_pose.yaw * 180.0 / kPi,
                observation.pose.x, observation.pose.y,
                observation.pose.yaw * 180.0 / kPi,
                observation.board_valid
                    ? wallName(observation.board_wall)
                    : "未知墙",
                observation.board_x, observation.board_y);
        } else {
            ROS_WARN("[停靠诊断][第二段入口] 无法读取当前小车map坐标");
        }

        TargetObservation final_observation;

        // --------------------------------------------------------------
        // V14.6边缘视野保护
        //
        // 到达第一段临时停靠点后：
        // 1. 先只匹配同一块板，严格不计算最终停靠点；
        // 2. 若centerX位于左/右1/4，先向对应方向转30度；
        // 3. 转完-3清缓存并重新匹配；
        // 4. 只有此时才根据新帧计算最终approach_stop_distance停靠点。
        // --------------------------------------------------------------
        Box precheck_box{0, 0, 0, 0, 0};
        BoardBoundaryEstimate precheck_board;
        TargetObservation unused_precheck_observation;

        const bool precheck_ok =
            detectDockingRefinedObservation(
                observation,
                target_name + "第二段边缘视野预检查",
                approach_stop_distance_,
                unused_precheck_observation,
                false,
                &precheck_box,
                &precheck_board,
                true);

        bool refined = false;

        if (precheck_ok) {
            const double left_quarter =
                0.25 * static_cast<double>(image_width_);
            const double right_quarter =
                0.75 * static_cast<double>(image_width_);

            const bool in_left_quarter =
                precheck_box.centerX() <= left_quarter;
            const bool in_right_quarter =
                precheck_box.centerX() >= right_quarter;

            if (in_left_quarter || in_right_quarter) {
                Pose2D current_pose;
                if (!getRobotPose(current_pose)) {
                    ROS_ERROR(
                        "%s边缘视野30度修正前无法读取当前map位姿",
                        target_name.c_str());
                    return false;
                }

                const double turn =
                    docking_recovery_turn_deg_ * kPi / 180.0;

                // 当前图像/相机语义已在实车确认：
                // 目标在画面左侧 -> 目标位于车体左侧 -> yaw正方向(CCW)；
                // 目标在画面右侧 -> yaw负方向(CW)。
                const double signed_turn =
                    in_left_quarter ? turn : -turn;
                const double adjusted_yaw =
                    normalizeAngle(
                        current_pose.yaw + signed_turn);

                ROS_WARN(
                    "V14.6临时停靠边缘保护：%s匹配框centerX=%.1f，"
                    "图像宽=%d，左1/4<=%.1f，右1/4>=%.1f；"
                    "框位于%s侧，先向%s旋转%.1f度："
                    "当前yaw=%.1f度 -> 目标yaw=%.1f度。"
                    "此时尚未计算最终停靠点。",
                    target_name.c_str(),
                    precheck_box.centerX(),
                    image_width_,
                    left_quarter,
                    right_quarter,
                    in_left_quarter ? "左" : "右",
                    in_left_quarter
                        ? "左/逆时针"
                        : "右/顺时针",
                    docking_recovery_turn_deg_,
                    current_pose.yaw * 180.0 / kPi,
                    adjusted_yaw * 180.0 / kPi);

                if (rotateToDockingRecoveryYaw(
                        adjusted_yaw,
                        target_name + "临时停靠边缘30度修正")) {
                    // 旋转过程中摄像头保持打开但没有持续read，
                    // force_refresh=true会先执行既有的NanoDet -3缓存刷新。
                    refined =
                        detectDockingRefinedObservation(
                            observation,
                            target_name + "边缘修正后最终视觉复定位",
                            approach_stop_distance_,
                            final_observation,
                            true);

                    if (refined) {
                        ROS_WARN(
                            "V14.6 %s边缘30度修正成功："
                            "已使用转向后的新帧重新计算最终停靠点。",
                            target_name.c_str());
                    }
                } else {
                    ROS_WARN(
                        "%s边缘30度修正旋转失败，"
                        "不使用转向前边缘帧计算最终点。",
                        target_name.c_str());
                }
            } else {
                // 中间二分之一不需要再次取帧：
                // 预检查得到的同墙板坐标就是当前实时新V4L2帧结果，
                // 现在才正式生成最终停靠点。
                const int segment_index =
                    segmentIndexForWall(precheck_board.wall);
                if (segment_index >= 0) {
                    refined =
                        makeDockingObservationAtStandoff(
                            segment_index,
                            observation.category,
                            precheck_board,
                            approach_stop_distance_,
                            final_observation);
                }

                if (refined) {
                    ROS_INFO(
                        "%s临时停靠框centerX=%.1f位于图像中间二分之一"
                        "(%.1f, %.1f)，不额外旋转；"
                        "现在正式计算最终停靠点。",
                        target_name.c_str(),
                        precheck_box.centerX(),
                        left_quarter,
                        right_quarter);
                }
            }
        }

        if (!refined) {
            ROS_WARN(
                "%s当前朝向/边缘30度修正后未能可靠复定位原目标；"
                "启动保留的±%.1f度视觉恢复。",
                target_name.c_str(), docking_recovery_turn_deg_);
            final_observation = observation;
            if (!recoverDockingObservation(
                    final_observation, target_name,
                    approach_stop_distance_)) {
                ROS_ERROR("%s最终视觉复定位失败，不执行盲目前进",
                          target_name.c_str());
                return false;
            }
        }

        ROS_INFO(
            "%s最终停靠目标：板=%s(%.3f, %.3f)，"
            "沿用原approach_stop_distance=%.3fm，"
            "move_base目标=(%.3f, %.3f, %.1f度)",
            target_name.c_str(),
            wallName(final_observation.board_wall),
            final_observation.board_x,
            final_observation.board_y,
            approach_stop_distance_,
            final_observation.pose.x,
            final_observation.pose.y,
            final_observation.pose.yaw * 180.0 / kPi);

        // 保持摄像头开启直到最终目标已计算完成；最终move_base不再依赖
        // 独立linear.y横移或雷达linear.x前进，因此不存在旧版误判居中后盲目前进。
        observation = final_observation;
        if (!navigateToPose(observation.pose.x,
                            observation.pose.y,
                            observation.pose.yaw,
                            target_name + "最终视觉停靠点")) {
            ROS_ERROR("%s最终move_base停靠失败", target_name.c_str());
            return false;
        }

        closeCamera();
        ROS_INFO("%s两段式move_base停靠成功", target_name.c_str());
        return true;
    }

    // V15：停靠后的安全回巡检。
    // 必须先完成纯旋转，停稳后再计算最短法向距离。
    // ------------------------------------------------------------------------
    // V16：快速顺序回巡检
    //
    // 速度策略完全复用巡检线转角 runPatrolPoseTransition()：
    //   旋转：yaw P + 最小/最大角速度 + 角加速度限制
    //   平移：位置 P + 最小/最大线速度 + 线加速度限制
    //
    // 与转角逻辑唯一关键区别：
    //   绝不同时旋转和平移。
    //
    // 顺序：
    //   1. vx=0, vy=0，只旋转到 segment.travel_yaw；
    //   2. 停稳后重新读取map位姿；
    //   3. 此时才计算最短法向距离；
    //   4. vx=0, wz=0，只用vy快速横移回线；
    //   5. 横移中若航向偏差超限：立刻停止vy -> 纯旋转校正 ->
    //      再重新计算法向距离并继续纯横移。
    // ------------------------------------------------------------------------

    bool rotateToPatrolDirectionForRejoin(
            const Segment& segment,
            Pose2D& pose_after_rotation,
            const std::string& reason) {
        stopRobot();

        const double target_yaw =
            normalizeAngle(segment.travel_yaw);
        const double yaw_tolerance =
            patrol_transition_yaw_tolerance_deg_ *
            kPi / 180.0;

        const ros::WallTime deadline =
            ros::WallTime::now() +
            ros::WallDuration(
                patrol_transition_timeout_);

        ros::WallTime last_time =
            ros::WallTime::now();
        ros::Rate rate(
            std::max(10.0, control_rate_));

        double command_wz = 0.0;
        int stable_frames = 0;

        ROS_WARN(
            "V16回巡检纯旋转[%s]：目标=%s巡检方向%.1f度；"
            "复用转角速度策略：kp=%.2f，wz=%.2f~%.2f，"
            "角加速度=%.2f；vx=0、vy=0。",
            reason.c_str(),
            segment.name.c_str(),
            target_yaw * 180.0 / kPi,
            patrol_transition_yaw_kp_,
            patrol_transition_min_angular_speed_,
            patrol_transition_max_angular_speed_,
            patrol_transition_angular_accel_);

        while (
            ros::ok() &&
            ros::WallTime::now() < deadline
        ) {
            ros::spinOnce();

            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(
                    0.0, 0.0, 0.0);
                rate.sleep();
                continue;
            }

            const double yaw_error =
                normalizeAngle(
                    target_yaw - pose.yaw);

            if (
                std::fabs(yaw_error)
                <= yaw_tolerance
            ) {
                command_wz = 0.0;
                publishVelocity(
                    0.0, 0.0, 0.0);
                ++stable_frames;

                if (
                    stable_frames >=
                    patrol_transition_stable_frames_
                ) {
                    stopRobot();
                    ros::Duration(0.05).sleep();

                    // 必须在纯旋转结束以后重新读位姿。
                    pose_after_rotation = pose;
                    getRobotPose(
                        pose_after_rotation);

                    ROS_WARN(
                        "V16纯旋转完成[%s]："
                        "停稳后位姿=(%.3f,%.3f,%.2f度)。",
                        reason.c_str(),
                        pose_after_rotation.x,
                        pose_after_rotation.y,
                        pose_after_rotation.yaw *
                            180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;

                double desired_wz =
                    clampValue(
                        patrol_transition_yaw_kp_ *
                            yaw_error,
                        -patrol_transition_max_angular_speed_,
                        patrol_transition_max_angular_speed_);

                if (
                    std::fabs(desired_wz) <
                    patrol_transition_min_angular_speed_
                ) {
                    desired_wz =
                        std::copysign(
                            patrol_transition_min_angular_speed_,
                            yaw_error);
                }

                const ros::WallTime now =
                    ros::WallTime::now();
                double dt =
                    (now - last_time).toSec();
                last_time = now;

                if (
                    !std::isfinite(dt) ||
                    dt <= 0.0
                ) {
                    dt = 0.05;
                }
                dt =
                    clampValue(
                        dt, 0.01, 0.20);

                command_wz =
                    limitPatrolTransitionRate(
                        desired_wz,
                        command_wz,
                        patrol_transition_angular_accel_ *
                            dt);

                publishVelocity(
                    0.0,
                    0.0,
                    command_wz);

                ROS_INFO_THROTTLE(
                    0.30,
                    "V16纯旋转[%s]：当前=%.1f度，目标=%.1f度，"
                    "误差=%.1f度，wz=%.3f；vx=vy=0。",
                    reason.c_str(),
                    pose.yaw * 180.0 / kPi,
                    target_yaw * 180.0 / kPi,
                    yaw_error * 180.0 / kPi,
                    command_wz);
            }

            rate.sleep();
        }

        stopRobot();
        ROS_ERROR(
            "V16纯旋转[%s]超时：%.1fs内未完成。",
            reason.c_str(),
            patrol_transition_timeout_);
        return false;
    }

    // 有符号法向误差。
    // segment.dir为单位切向量，左法向=(-dir_y, dir_x)。
    // 车头对准travel_yaw后，左法向对应车体+Y。
    double signedCrossTrackError(
            const Segment& segment,
            const Pose2D& pose) const {
        const double normal_x =
            -segment.dir_y;
        const double normal_y =
            segment.dir_x;

        return
            (pose.x - segment.start_x) *
                normal_x
            +
            (pose.y - segment.start_y) *
                normal_y;
    }

    void closestPointOnPatrolSegment(
            const Segment& segment,
            const Pose2D& pose,
            double& progress,
            double& target_x,
            double& target_y) const {
        progress =
            clampValue(
                segmentProgress(
                    segment,
                    pose),
                0.0,
                segment.length);

        target_x =
            segment.start_x +
            progress * segment.dir_x;

        target_y =
            segment.start_y +
            progress * segment.dir_y;
    }

    bool returnToPatrolLineSafely(
            const Segment& segment) {
        // ============================================================
        // 步骤1：
        // 初始恢复仍必须先纯旋转到巡检方向。
        // 此阶段 vx=0、vy=0。
        // ============================================================
        Pose2D aligned_pose;

        if (
            !rotateToPatrolDirectionForRejoin(
                segment,
                aligned_pose,
                "初始对准")
        ) {
            return false;
        }

        // ============================================================
        // 步骤2：
        // 只有初始纯旋转完成并停稳以后，才计算最近投影点
        // 和最短法向距离。
        // ============================================================
        double progress = 0.0;
        double target_x = 0.0;
        double target_y = 0.0;

        closestPointOnPatrolSegment(
            segment,
            aligned_pose,
            progress,
            target_x,
            target_y);

        const double initial_cross_track =
            signedCrossTrackError(
                segment,
                aligned_pose);

        const double raw_progress =
            segmentProgress(
                segment,
                aligned_pose);

        const double progress_guard = 0.08;

        if (
            raw_progress < -progress_guard ||
            raw_progress >
                segment.length + progress_guard
        ) {
            stopRobot();

            ROS_ERROR(
                "V17拒绝回线：转向后沿%s投影进度=%.3fm，"
                "有效段=[0,%.3f]m，超出%.2fm保护范围。",
                segment.name.c_str(),
                raw_progress,
                segment.length,
                progress_guard);

            return false;
        }

        ROS_WARN(
            "V17步骤2：初始转向完成后才计算法向距离："
            "当前位置=(%.3f,%.3f)，"
            "最近投影点=(%.3f,%.3f)，"
            "进度=%.3f/%.3fm，"
            "cross_track=%.3fm，"
            "最短法向距离=%.3fm。",
            aligned_pose.x,
            aligned_pose.y,
            target_x,
            target_y,
            progress,
            segment.length,
            initial_cross_track,
            std::fabs(initial_cross_track));

        // ============================================================
        // 步骤3：
        // 快速横移 + 实时航向修正。
        //
        // vy：
        //   patrol_transition_position_kp_
        //   patrol_transition_min_linear_speed_
        //   patrol_transition_max_linear_speed_
        //   patrol_transition_linear_accel_
        //
        // wz：
        //   patrol_transition_yaw_kp_
        //   patrol_transition_min_angular_speed_
        //   patrol_transition_max_angular_speed_
        //   patrol_transition_angular_accel_
        //
        // 关键：
        //   vx始终为0；
        //   vy和wz允许同时存在。
        // ============================================================
        const double yaw_tolerance =
            patrol_transition_yaw_tolerance_deg_ *
            kPi / 180.0;

        const ros::WallTime deadline =
            ros::WallTime::now() +
            ros::WallDuration(
                patrol_transition_timeout_);

        ros::WallTime last_time =
            ros::WallTime::now();

        ros::Rate rate(
            std::max(10.0, control_rate_));

        double command_vy = 0.0;
        double command_wz = 0.0;
        int stable_frames = 0;

        ROS_WARN(
            "V17步骤3开始快速横移+航向保持："
            "vy复用转角线速度策略(kp=%.2f，%.3f~%.3fm/s，acc=%.2f)，"
            "wz复用转角角速度策略(kp=%.2f，%.3f~%.3frad/s，acc=%.2f)；"
            "vx始终为0。",
            patrol_transition_position_kp_,
            patrol_transition_min_linear_speed_,
            patrol_transition_max_linear_speed_,
            patrol_transition_linear_accel_,
            patrol_transition_yaw_kp_,
            patrol_transition_min_angular_speed_,
            patrol_transition_max_angular_speed_,
            patrol_transition_angular_accel_);

        while (
            ros::ok() &&
            ros::WallTime::now() < deadline
        ) {
            ros::spinOnce();

            Pose2D pose;

            if (!getRobotPose(pose)) {
                command_vy = 0.0;
                command_wz = 0.0;
                stable_frames = 0;

                publishVelocity(
                    0.0, 0.0, 0.0);

                rate.sleep();
                continue;
            }

            const double cross_track =
                signedCrossTrackError(
                    segment,
                    pose);

            const double abs_cross_track =
                std::fabs(cross_track);

            const double yaw_error =
                normalizeAngle(
                    segment.travel_yaw -
                    pose.yaw);

            // --------------------------------------------------------
            // vy目标：与转角位置P控制同一套速度策略。
            // --------------------------------------------------------
            double desired_vy = 0.0;

            if (
                abs_cross_track >
                patrol_transition_position_tolerance_
            ) {
                // 正cross_track说明车位于路径左侧(+Y侧)，
                // 所以要给负vy；负值反之。
                desired_vy =
                    -patrol_transition_position_kp_ *
                    cross_track;

                desired_vy =
                    clampValue(
                        desired_vy,
                        -patrol_transition_max_linear_speed_,
                        patrol_transition_max_linear_speed_);

                if (
                    std::fabs(desired_vy) <
                    patrol_transition_min_linear_speed_
                ) {
                    desired_vy =
                        std::copysign(
                            patrol_transition_min_linear_speed_,
                            desired_vy);
                }
            }

            // --------------------------------------------------------
            // wz目标：与转角yaw P控制同一套速度策略。
            //
            // 这里与V16不同：
            // 横移过程中只要航向超出容差，就直接同时输出wz修回来，
            // 不再先停车再单独旋转。
            // --------------------------------------------------------
            double desired_wz = 0.0;

            if (
                std::fabs(yaw_error) >
                yaw_tolerance
            ) {
                desired_wz =
                    clampValue(
                        patrol_transition_yaw_kp_ *
                        yaw_error,
                        -patrol_transition_max_angular_speed_,
                        patrol_transition_max_angular_speed_);

                if (
                    std::fabs(desired_wz) <
                    patrol_transition_min_angular_speed_
                ) {
                    desired_wz =
                        std::copysign(
                            patrol_transition_min_angular_speed_,
                            yaw_error);
                }
            }

            const ros::WallTime now =
                ros::WallTime::now();

            double dt =
                (now - last_time).toSec();

            last_time = now;

            if (
                !std::isfinite(dt) ||
                dt <= 0.0
            ) {
                dt = 0.05;
            }

            dt =
                clampValue(
                    dt,
                    0.01,
                    0.20);

            // vy沿用转角线加速度限制。
            command_vy =
                limitPatrolTransitionRate(
                    desired_vy,
                    command_vy,
                    patrol_transition_linear_accel_ *
                        dt);

            // wz沿用转角角加速度限制。
            command_wz =
                limitPatrolTransitionRate(
                    desired_wz,
                    command_wz,
                    patrol_transition_angular_accel_ *
                        dt);

            const bool line_ok =
                abs_cross_track <=
                patrol_transition_position_tolerance_;

            const bool yaw_ok =
                std::fabs(yaw_error) <=
                yaw_tolerance;

            if (
                line_ok &&
                yaw_ok
            ) {
                command_vy = 0.0;
                command_wz = 0.0;

                publishVelocity(
                    0.0, 0.0, 0.0);

                ++stable_frames;

                ROS_INFO_THROTTLE(
                    0.35,
                    "V17恢复稳定：cross_track=%.3fm<=%.3fm，"
                    "yaw_err=%.2f度<=%.2f度，稳定帧=%d/%d。",
                    cross_track,
                    patrol_transition_position_tolerance_,
                    yaw_error * 180.0 / kPi,
                    patrol_transition_yaw_tolerance_deg_,
                    stable_frames,
                    patrol_transition_stable_frames_);

                if (
                    stable_frames >=
                    patrol_transition_stable_frames_
                ) {
                    stopRobot();

                    ros::Duration(0.05).sleep();

                    Pose2D final_pose =
                        pose;

                    getRobotPose(
                        final_pose);

                    const double final_cross_track =
                        signedCrossTrackError(
                            segment,
                            final_pose);

                    const double final_yaw_error =
                        normalizeAngle(
                            segment.travel_yaw -
                            final_pose.yaw);

                    if (
                        std::fabs(final_cross_track) <=
                            patrol_resume_max_cross_track_error_ &&
                        std::fabs(final_yaw_error) <=
                            yaw_tolerance
                    ) {
                        ROS_WARN(
                            "V17快速恢复完成："
                            "final=(%.3f,%.3f,%.2f度)，"
                            "cross_track=%.3fm<=接管阈值%.3fm，"
                            "yaw_err=%.2f度；"
                            "允许Patrol-R2重新接管。",
                            final_pose.x,
                            final_pose.y,
                            final_pose.yaw *
                                180.0 / kPi,
                            final_cross_track,
                            patrol_resume_max_cross_track_error_,
                            final_yaw_error *
                                180.0 / kPi);

                        return true;
                    }

                    ROS_WARN(
                        "V17最终接管保护未通过："
                        "cross_track=%.3fm，yaw_err=%.2f度；"
                        "继续闭环恢复。",
                        final_cross_track,
                        final_yaw_error *
                            180.0 / kPi);

                    stable_frames = 0;
                }

                rate.sleep();
                continue;
            }

            stable_frames = 0;

            // V17：
            // vx固定0；
            // vy负责法向回线；
            // wz负责横移过程中实时修正航向。
            publishVelocity(
                0.0,
                command_vy,
                command_wz);

            ROS_INFO_THROTTLE(
                0.25,
                "V17快速回线：cross_track=%.3fm，"
                "vy=%.3fm/s；yaw_err=%.2f度，"
                "wz=%.3frad/s；vx=0。",
                cross_track,
                command_vy,
                yaw_error * 180.0 / kPi,
                command_wz);

            rate.sleep();
        }

        stopRobot();

        ROS_ERROR(
            "V17快速恢复超时："
            "%.1fs内未能回到%s。",
            patrol_transition_timeout_,
            segment.name.c_str());

        return false;
    }

    bool rotateCounterClockwiseForOcr(Pose2D& pose_after_rotation) {
        Pose2D start_pose;
        if (!getRobotPose(start_pose)) {
            ROS_ERROR("OCR补偿旋转前无法读取机器人位姿");
            return false;
        }

        const double target_yaw = normalizeAngle(
            start_pose.yaw + ocr_recovery_turn_deg_ * kPi / 180.0);
        const double tolerance =
            ocr_recovery_turn_tolerance_deg_ * kPi / 180.0;
        const ros::WallTime deadline = ros::WallTime::now() +
            ros::WallDuration(ocr_recovery_turn_timeout_);
        int stable_frames = 0;
        ros::Rate rate(20.0);

        ROS_INFO("OCR补偿旋转开始：当前朝向=%.1f度，"
                 "目标朝向=%.1f度（逆时针%.1f度）",
                 start_pose.yaw * 180.0 / kPi,
                 target_yaw * 180.0 / kPi,
                 ocr_recovery_turn_deg_);

        while (ros::ok() && ros::WallTime::now() < deadline) {
            ros::spinOnce();
            Pose2D pose;
            if (!getRobotPose(pose)) {
                publishVelocity(0.0, 0.0, 0.0);
                stable_frames = 0;
                rate.sleep();
                continue;
            }

            const double yaw_error = normalizeAngle(target_yaw - pose.yaw);
            if (std::fabs(yaw_error) <= tolerance) {
                publishVelocity(0.0, 0.0, 0.0);
                ++stable_frames;
                if (stable_frames >= ocr_recovery_turn_stable_frames_) {
                    stopRobot();
                    pose_after_rotation = pose;
                    ros::Duration(ocr_recovery_settle_time_).sleep();
                    // 静止后再读一次最终位姿；短暂TF失败时保留最后一帧。
                    getRobotPose(pose_after_rotation);
                    ROS_INFO("OCR补偿旋转完成：最终朝向=%.1f度，"
                             "目标误差=%.2f度",
                             pose_after_rotation.yaw * 180.0 / kPi,
                             normalizeAngle(
                                 target_yaw - pose_after_rotation.yaw) *
                                 180.0 / kPi);
                    return true;
                }
            } else {
                stable_frames = 0;
                double angular_z = clampValue(
                    ocr_recovery_turn_kp_ * yaw_error,
                    -ocr_recovery_turn_max_speed_,
                    ocr_recovery_turn_max_speed_);
                if (std::fabs(angular_z) <
                    ocr_recovery_turn_min_speed_) {
                    angular_z = yaw_error >= 0.0
                                    ? ocr_recovery_turn_min_speed_
                                    : -ocr_recovery_turn_min_speed_;
                }
                publishVelocity(0.0, 0.0, angular_z);
                ROS_INFO_THROTTLE(
                    0.5,
                    "OCR补偿旋转中：剩余角度=%.2f度，angular.z=%.3f",
                    yaw_error * 180.0 / kPi, angular_z);
            }
            rate.sleep();
        }

        stopRobot();
        ROS_ERROR("OCR补偿旋转在%.1f秒内未完成",
                  ocr_recovery_turn_timeout_);
        return false;
    }

    bool openCamera() {
        if (camera_opened_) return true;
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -1;
        if (!detect_client_.call(service)) {
            ROS_ERROR("打开NanoDet摄像头失败");
            return false;
        }
        camera_opened_ = true;
        return true;
    }

    void closeCamera() {
        if (!camera_opened_) return;
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = -2;
        detect_client_.call(service);
        camera_opened_ = false;
    }

    bool detectBoxes(std::vector<Box>& boxes) {
        boxes.clear();
        ros_nanodet::detect_result_srv service;
        service.request.detect_start = 1;
        const ros::WallTime begin = ros::WallTime::now();
        if (!detect_client_.call(service)) {
            ROS_WARN_THROTTLE(1.0, "调用/nanodet_detect失败");
            return false;
        }
        const double elapsed =
            (ros::WallTime::now() - begin).toSec();
        if (elapsed > max_detection_duration_) {
            ROS_ERROR("NanoDet耗时%.3f秒，超过%.3f秒，过期结果已丢弃",
                      elapsed, max_detection_duration_);
            return false;
        }

        const std::size_t count = std::min(
            std::min(service.response.x0.size(), service.response.y0.size()),
            std::min(service.response.x1.size(), service.response.y1.size()));
        for (std::size_t i = 0; i < count; ++i) {
            Box box;
            box.class_id = i < service.response.class_name.size()
                               ? service.response.class_name[i] : 0;
            box.x0 = service.response.x0[i];
            box.y0 = service.response.y0[i];
            box.x1 = service.response.x1[i];
            box.y1 = service.response.y1[i];
            if (box.x1 > box.x0 && box.y1 > box.y0) boxes.push_back(box);
        }
        return true;
    }

    int chooseClosestCenterBox(const std::vector<Box>& boxes) const {
        const double center = 0.5 * image_width_;
        int selected = -1;
        double best_error = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double error = std::fabs(boxes[i].centerX() - center);
            if (error < best_error) {
                best_error = error;
                selected = static_cast<int>(i);
            }
        }
        return selected;
    }

    static double intersectionOverUnion(const Box& first, const Box& second) {
        const int left = std::max(first.x0, second.x0);
        const int top = std::max(first.y0, second.y0);
        const int right = std::min(first.x1, second.x1);
        const int bottom = std::min(first.y1, second.y1);
        const double intersection =
            static_cast<double>(std::max(0, right - left) *
                                std::max(0, bottom - top));
        const double union_area =
            first.width() * first.height() +
            second.width() * second.height() - intersection;
        return union_area > 0.0 ? intersection / union_area : 0.0;
    }

    int associateSelectedBox(const std::vector<Box>& boxes,
                             const Box& previous) const {
        int best_iou_index = -1;
        double best_iou = 0.0;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double iou = intersectionOverUnion(boxes[i], previous);
            if (iou > best_iou) {
                best_iou = iou;
                best_iou_index = static_cast<int>(i);
            }
        }
        if (best_iou_index >= 0 && best_iou >= 0.05) {
            return best_iou_index;
        }

        // IoU可能在横移速度较快、检测框尺寸突变或目标跨过画面中线时
        // 瞬间降为0。此时继续按框中心距离寻找同一目标。
        int nearest = -1;
        double nearest_distance =
            std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double dx = boxes[i].centerX() - previous.centerX();
            const double dy = boxes[i].centerY() - previous.centerY();
            const double distance = std::hypot(dx, dy);
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest = static_cast<int>(i);
            }
        }

        // 旧逻辑把max_track_jump_px_作为硬门限：单帧跳动一旦超过
        // 门限就返回-1，而tracked_box仍停留在旧框，导致后续即使
        // NanoDet每帧都检测到目标也无法重新关联，并被累计成连续丢失。
        // 现在该参数只触发重新捕获日志，不再屏蔽全画面（尤其右侧
        // 三分之一）中仍然有效的检测框。
        if (nearest >= 0 && nearest_distance > max_track_jump_px_) {
            ROS_WARN_THROTTLE(
                1.0,
                "横移居中：目标框跳动%.1fpx超过%.1fpx，"
                "已使用全画面最近框重新捕获",
                nearest_distance, max_track_jump_px_);
        }
        return nearest;
    }

    OcrRecord recognizeStaticTarget(const Box& trigger_box) {
        OcrRecord best_any;
        OcrRecord best_keyword;
        bool have_any = false;
        bool have_keyword = false;
        Box reference = trigger_box;

        ros_nanodet::ocr_result_srv clear_service;
        clear_service.request.command = -3;
        if (!ocr_client_.call(clear_service)) {
            ROS_WARN("OCR缓冲帧清理失败，将继续识别");
        }

        for (int attempt = 0; attempt < ocr_attempts_ && ros::ok(); ++attempt) {
            ros_nanodet::ocr_result_srv service;
            service.request.command = 1;
            if (!ocr_client_.call(service)) {
                ROS_WARN("第%d次OCR服务调用失败", attempt + 1);
                ros::Duration(ocr_retry_interval_).sleep();
                continue;
            }
            const std::size_t count = std::min(
                std::min(service.response.text.size(),
                         service.response.confidence.size()),
                std::min(
                    std::min(service.response.x0.size(),
                             service.response.y0.size()),
                    std::min(service.response.x1.size(),
                             service.response.y1.size())));
            int selected = -1;
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < count; ++i) {
                const double center_x =
                    0.5 * (service.response.x0[i] +
                           service.response.x1[i]);
                const double center_y =
                    0.5 * (service.response.y0[i] +
                           service.response.y1[i]);
                const double distance =
                    std::hypot(center_x - reference.centerX(),
                               center_y - reference.centerY());
                if (distance < nearest) {
                    nearest = distance;
                    selected = static_cast<int>(i);
                }
            }
            if (selected >= 0) {
                const std::size_t i = static_cast<std::size_t>(selected);
                OcrRecord candidate;
                candidate.success = service.response.success;
                candidate.text = service.response.text[i];
                candidate.category = classifyText(candidate.text);
                candidate.confidence = service.response.confidence[i];
                candidate.box = Box{0, service.response.x0[i],
                                    service.response.y0[i],
                                    service.response.x1[i],
                                    service.response.y1[i]};
                reference = candidate.box;
                ROS_INFO("OCR第%d/%d次：%s，类别=%s，置信度=%.3f",
                         attempt + 1, ocr_attempts_,
                         candidate.text.c_str(),
                         candidate.category.c_str(),
                         candidate.confidence);
                if (!candidate.text.empty() &&
                    (!have_any ||
                     candidate.confidence > best_any.confidence)) {
                    best_any = candidate;
                    have_any = true;
                }
                if (candidate.category != "unknown" &&
                    (!have_keyword ||
                     candidate.confidence > best_keyword.confidence)) {
                    best_keyword = candidate;
                    have_keyword = true;
                }
            } else {
                ROS_WARN("第%d次OCR没有返回文字框", attempt + 1);
            }
            ros::Duration(ocr_retry_interval_).sleep();
        }

        OcrRecord result;
        if (have_keyword) result = best_keyword;
        else if (have_any) result = best_any;
        else result.box = trigger_box;
        ROS_INFO("OCR最终结果：%s，分类=%s",
                 result.text.c_str(),
                 categoryChinese(result.category));
        return result;
    }

    void publishVelocity(double linear_x, double linear_y,
                         double angular_z) {
        ucarmain2026::set_speed service;
        service.request.target_twist.linear.x = linear_x;
        service.request.target_twist.linear.y = linear_y;
        service.request.target_twist.linear.z = 0.0;
        service.request.target_twist.angular.x = 0.0;
        service.request.target_twist.angular.y = 0.0;
        service.request.target_twist.angular.z = angular_z;
        service.request.work = true;
        service.request.movebase_flag = false;
        service.request.target_x = 0.0;
        service.request.target_y = 0.0;
        service.request.target_yaw = 0.0;
        if (!set_speed_client_.call(service) ||
            !service.response.success) {
            ROS_ERROR_THROTTLE(1.0, "调用/set_speed失败");
        }
    }

    void stopRobot() {
        if (!set_speed_client_.exists()) return;
        ucarmain2026::set_speed service;
        service.request.target_twist = geometry_msgs::Twist();
        service.request.work = true;
        service.request.movebase_flag = false;
        service.request.target_x = 0.0;
        service.request.target_y = 0.0;
        service.request.target_yaw = 0.0;
        set_speed_client_.call(service);
        ros::Duration(0.12).sleep();
        service.request.work = false;
        if (!set_speed_client_.call(service)) {
            ROS_WARN_THROTTLE(1.0, "停止/set_speed控制失败");
        }
    }

    void printSummary(bool success) const {
        ROS_INFO("================ 找板停靠结果 ================");
        ROS_INFO("现实目标：%s；找到=%s；停靠=%s",
                 categoryChinese(real_target_category_),
                 real_observation_.valid ? "是" : "否",
                 real_docked_ ? "成功" : "未完成");
        ROS_INFO("现实目标延后停靠状态：%s",
                 real_target_pending_ ? "是" : "否");
        ROS_INFO("仿真目标：%s；找到=%s；停靠=%s",
                 categoryChinese(simulation_target_category_),
                 simulation_observation_.valid ? "是" : "否",
                 simulation_docked_ ? "成功" : "未完成");
        ROS_INFO("仿真目标待停靠状态：%s",
                 simulation_target_pending_ ? "是" : "否");
        ROS_INFO("仿真目标非当前墙段末保护（仅现实目标未完成时）：%s",
                 simulation_target_blocked_until_segment_end_
                     ? "锁定中" : "未锁定");

        ROS_INFO("unknown候选总数：%zu", unknown_candidates_.size());
        for (std::size_t i = 0;
             i < unknown_candidates_.size();
             ++i) {
            const UnknownCandidate& candidate =
                unknown_candidates_[i];
            ROS_INFO(
                "  候选[%zu]：%s(%.3f,%.3f)，来源段=%d，"
                "已回访=%s，已解决=%s",
                i + 1,
                wallName(candidate.board.wall),
                candidate.board.x,
                candidate.board.y,
                candidate.source_segment_index + 1,
                candidate.attempted ? "是" : "否",
                candidate.resolved ? "是" : "否");
        }

        ROS_INFO("任务总结果：%s", success ? "成功" : "失败");
        ROS_INFO("==============================================");
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    MoveBaseClient move_base_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::ServiceClient detect_client_;
    ros::ServiceClient ocr_client_;
    ros::ServiceClient set_speed_client_;
    ros::ServiceClient move_base_reconfigure_client_;
    ros::ServiceClient patrol_path_lock_client_;
    ros::ServiceClient controller_reset_client_;
    ros::Publisher patrol_path_publisher_;

    // V22角点自适应costmap判断。
    ros::Subscriber corner_costmap_subscriber_;
    nav_msgs::OccupancyGrid::ConstPtr corner_costmap_message_;
    ros::WallTime corner_costmap_received_wall_time_;
    std::string corner_obstacle_costmap_topic_;
    double corner_obstacle_check_radius_;
    int corner_obstacle_lethal_cost_threshold_;
    double corner_obstacle_costmap_max_age_;

    // V23：识别停靠点距当前巡检段终点小于该值时，
    // 现实目标停靠后直接执行本段角点旋转平移。
    double patrol_near_end_corner_shortcut_distance_;

    // V24：
    // 非当前墙现实目标提前停靠的“双1m”判定阈值。
    double patrol_noncurrent_target_early_dock_distance_;

    std::string real_target_category_;
    std::string simulation_target_category_;
    std::string map_frame_;
    std::string base_frame_;

    double room_min_x_;
    double room_max_x_;
    double room_min_y_;
    double room_max_y_;
    double start_x_;
    double start_y_;
    double start_yaw_deg_;
    double navigation_timeout_;
    std::string planner_private_namespace_;
    double patrol_path_spacing_;
    double patrol_speed_limit_;
    double normal_navigation_speed_limit_;
    double patrol_cancel_timeout_;
    double patrol_interface_timeout_;
    std::string move_base_reconfigure_service_;
    bool disable_move_base_oscillation_during_patrol_;
    double normal_move_base_oscillation_timeout_;
    int patrol_aborted_retry_count_;

    double segment_end_tolerance_;
    double control_rate_;

    // V12角点过渡：短距离全向XY+yaw同时闭环。
    double patrol_transition_position_kp_;
    double patrol_transition_yaw_kp_;
    double patrol_transition_min_linear_speed_;
    double patrol_transition_max_linear_speed_;
    double patrol_transition_min_angular_speed_;
    double patrol_transition_max_angular_speed_;
    double patrol_transition_linear_accel_;
    double patrol_transition_angular_accel_;
    double patrol_transition_yaw_priority_start_deg_;
    double patrol_transition_yaw_priority_release_deg_;
    double patrol_transition_yaw_priority_min_linear_scale_;
    double patrol_transition_position_tolerance_;
    double patrol_transition_yaw_tolerance_deg_;
    int patrol_transition_stable_frames_;
    double patrol_transition_timeout_;

    int image_width_;
    double camera_fx_;
    double camera_yaw_offset_deg_;
    double docking_standoff_;
    double settle_time_;
    int ocr_attempts_;
    double ocr_retry_interval_;
    double ocr_recovery_turn_deg_;
    double ocr_recovery_turn_kp_;
    double ocr_recovery_turn_min_speed_;
    double ocr_recovery_turn_max_speed_;
    double ocr_recovery_turn_tolerance_deg_;
    int ocr_recovery_turn_stable_frames_;
    double ocr_recovery_turn_timeout_;
    double ocr_recovery_settle_time_;
    double max_detection_duration_;
    double patrol_stop_max_center_x_;

    // V14.5当前左墙目标接近减速参数。
    double patrol_target_slowdown_start_distance_;
    double patrol_target_stop_distance_;
    double patrol_target_min_speed_ratio_;

    // V14.7双停车保护参数。
    double patrol_noncurrent_wall_stop_max_distance_;
    int patrol_target_left_edge_stop_px_;

    double duplicate_coordinate_distance_;
    double max_track_jump_px_;
    double docking_recovery_turn_deg_;
    int docking_recovery_detection_attempts_;
    double docking_recovery_detection_interval_;
    double docking_refine_max_board_shift_;
    int docking_refresh_clear_calls_;
    // V17快速回巡检只保留最终接管保护阈值。
    // 旋转/横移速度全部复用patrol_transition_*参数。
    double patrol_resume_max_cross_track_error_;
    double approach_stop_distance_;

    std::vector<Segment> segments_;
    TargetObservation real_observation_;
    TargetObservation simulation_observation_;
    bool configuration_valid_ = false;
    bool camera_opened_ = false;
    bool real_docked_ = false;
    bool simulation_docked_ = false;

    std::function<void()> real_docked_callback_;
    bool real_dock_notification_sent_ = false;
    bool real_target_pending_ = false;
    int real_target_defer_segment_index_ = -1;

    // V24非当前墙现实目标提前处理状态。
    bool real_target_near_end_early_dock_eligible_ = false;
    double real_target_distance_to_source_segment_end_ =
        std::numeric_limits<double>::infinity();
    int real_target_resume_segment_index_ = -1;

    // V24外层巡检墙跳转目标；-1表示不跳转。
    int forced_next_segment_index_ = -1;

    bool simulation_target_pending_ = false;
    // V14.4保护2：
    // 非当前巡检墙仿真目标只有在识别时real_docked_=false才锁到段末；
    // 若real_docked_=true，则不加锁，允许立即停靠。
    bool simulation_target_blocked_until_segment_end_ = false;
    int current_segment_index_ = 0;
    PatrolCheckpoint patrol_checkpoint_;
    bool patrol_goal_active_ = false;
    bool patrol_path_locked_ = false;
    bool move_base_oscillation_timeout_cached_ = false;
    bool move_base_patrol_oscillation_guard_active_ = false;
    bool shadow_mode_has_been_disabled_ = false;
    std::uint32_t patrol_path_sequence_ = 0;
    std::vector<BoardBoundaryEstimate> seen_board_coordinates_;

    // V14.9：巡检OCR=unknown时记录，整圈后任务未完成才依次回访。
    std::vector<UnknownCandidate> unknown_candidates_;

    // V14.5视觉接近减速状态。
    PatrolVisualApproach patrol_visual_approach_;
    double current_patrol_runtime_speed_limit_ =
        std::numeric_limits<double>::quiet_NaN();
    bool lidar_layout_logged_ = false;

};


// ============================================================================
// 国赛新增：扫码前坡道固定直线路径阶段
//
// 该类严格来自用户最新确认可用的 race_2(2).cpp：
//   1. (4.75,4.75,180°)
//   2. AMCL重定位
//   3. 再发同点180°收正
//   4. clearance_optimizer/enabled=false
//      enable_path_replanning=false
//      四项速度上限=0.6
//   5. 固定Reference直线到(2.50,4.75,180°)
//      move_base使用近距离activation goal启动控制循环
//   6. 终点AMCL重定位
//   7. 原地转90°
//   8. 两个布尔参数恢复true，四项速度仍保持0.6
//
// 注意：clearance_optimizer/enabled 要真正动态生效，MyPlanner必须已经应用
// 前一版确认通过的 runtime parameter 修复。
// ============================================================================
constexpr double kSlopePi = 3.14159265358979323846;

class NationalSlopeStage
{
public:
    NationalSlopeStage()
        : nh_(),
          pnh_("~"),
          move_base_("move_base", true),
          patrol_path_locked_(false),
          slope_parameters_disabled_(false),
          path_sequence_(0)
    {
        pnh_.param<std::string>(
            "map_frame",
            map_frame_,
            std::string("map"));

        pnh_.param<std::string>(
            "planner_private_namespace",
            planner_private_namespace_,
            std::string("/move_base/MyPlanner"));

        pnh_.param(
            "navigation_timeout",
            navigation_timeout_,
            180.0);

        pnh_.param(
            "patrol_path_spacing",
            patrol_path_spacing_,
            0.02);

        pnh_.param(
            "patrol_interface_timeout",
            patrol_interface_timeout_,
            3.0);

        // 固定 Path 锁定以后，move_base 仍需要成功生成一次普通全局路径，
        // 才会进入 BaseLocalPlanner 控制循环。
        // 因此这里不给 GlobalPlanner 真正的坡道终点，而给一个很近、
        // 位于起点自由区域内的“激活目标”。
        pnh_.param(
            "activation_goal_x",
            activation_goal_x_,
            4.80);

        pnh_.param(
            "activation_goal_y",
            activation_goal_y_,
            4.75);

        pnh_.param(
            "activation_goal_yaw_deg",
            activation_goal_yaw_deg_,
            180.0);

        pnh_.param(
            "relocalization_publish_count",
            relocalization_publish_count_,
            3);

        pnh_.param(
            "relocalization_publish_interval",
            relocalization_publish_interval_,
            0.10);

        pnh_.param(
            "relocalization_settle_time",
            relocalization_settle_time_,
            0.50);

        pnh_.param<std::string>(
            "cmd_vel_topic",
            cmd_vel_topic_,
            std::string("/cmd_vel"));

        patrol_path_spacing_ =
            std::max(0.005, std::fabs(patrol_path_spacing_));

        navigation_timeout_ =
            std::max(1.0, navigation_timeout_);

        patrol_interface_timeout_ =
            std::max(0.5, patrol_interface_timeout_);

        relocalization_publish_count_ =
            std::max(1, relocalization_publish_count_);

        relocalization_publish_interval_ =
            std::max(0.02, relocalization_publish_interval_);

        relocalization_settle_time_ =
            std::max(0.0, relocalization_settle_time_);

        initial_pose_pub_ =
            nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "/initialpose",
                1,
                false);

        cmd_vel_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                cmd_vel_topic_,
                1,
                false);

        patrol_path_pub_ =
            nh_.advertise<nav_msgs::Path>(
                planner_private_namespace_ + "/patrol_path",
                1,
                true);

        patrol_path_lock_client_ =
            nh_.serviceClient<std_srvs::SetBool>(
                planner_private_namespace_ + "/lock_patrol_path");

        controller_reset_client_ =
            nh_.serviceClient<std_srvs::Trigger>(
                planner_private_namespace_ + "/reset_controller_state");

        ROS_WARN(
            "国赛坡道阶段节点已创建：真实固定Reference="
            "(4.75,4.75)->(2.50,4.75)，方向保持180度，"
            "路径间距=%.3fm；move_base激活目标=(%.2f,%.2f,%.1f度)",
            patrol_path_spacing_,
            activation_goal_x_,
            activation_goal_y_,
            activation_goal_yaw_deg_);
    }

    ~NationalSlopeStage()
    {
        stopRobot();

        if (patrol_path_locked_)
        {
            requestPatrolPathLock(false);
        }

        // 防止测试中途退出后把规划器永久留在坡道特殊模式。
        if (slope_parameters_disabled_)
        {
            restoreSlopePlannerParameters();
        }
    }

    bool run()
    {
        if (!waitForDependencies())
        {
            ROS_ERROR("依赖接口未准备完成，坡道阶段无法启动");
            return false;
        }

        // ============================================================
        // 1. 普通 move_base 到坡前/坡上准备点
        // ============================================================
        if (!navigateToPose(
                4.75,
                4.75,
                kSlopePi,
                "前往坡道固定路径起点"))
        {
            return failAndCleanup(
                "无法到达坡道固定路径起点");
        }

        // ============================================================
        // 2. 第一次强制重定位
        // ============================================================
        if (!publishRelocalization(
                4.75,
                4.75,
                kSlopePi,
                "坡道起点重定位"))
        {
            return failAndCleanup(
                "坡道起点重定位失败");
        }

        // ============================================================
        // 3. 再发一次完全相同的 move_base 目标
        //    让局部规划器重新把车头收正到180度
        // ============================================================
        if (!navigateToPose(
                4.75,
                4.75,
                kSlopePi,
                "重定位后再次校正坡道起点姿态"))
        {
            return failAndCleanup(
                "坡道起点二次姿态校正失败");
        }

        // ============================================================
        // 4. 关闭路径优化 + 关闭路径重规划
        // ============================================================
        if (!setSlopePlannerParameters(false))
        {
            return failAndCleanup(
                "无法关闭坡道阶段规划器开关");
        }

        // 与两个 false 同一阶段，把坡道巡检所需的四项速度上限
        // 统一设置为 0.6。按当前流程要求，坡道结束后不恢复旧速度，
        // 只恢复 clearance_optimizer/enabled 和 enable_path_replanning。
        if (!setSlopeSpeedLimits(0.60))
        {
            return failAndCleanup(
                "无法设置坡道阶段四项速度上限为0.6");
        }

        // ============================================================
        // 5. 强制固定直线路径通过斜坡
        // ============================================================
        if (!runForcedSlopePath())
        {
            return failAndCleanup(
                "固定直线路径通过斜坡失败");
        }

        // 到达以后立即解除固定路线所有权。
        if (patrol_path_locked_)
        {
            if (!requestPatrolPathLock(false))
            {
                return failAndCleanup(
                    "到达坡道终点后解除固定路径锁失败");
            }
        }

        stopRobot();

        if (!resetPlannerControllerState())
        {
            return failAndCleanup(
                "坡道结束后复位局部规划器控制状态失败");
        }

        // ============================================================
        // 6. 下坡以后重新定位为 (2.5,4.75,180°)
        // ============================================================
        if (!publishRelocalization(
                2.50,
                4.75,
                kSlopePi,
                "坡道终点重定位"))
        {
            return failAndCleanup(
                "坡道终点重定位失败");
        }

        // ============================================================
        // 7. 原地发送同坐标 90°，旋转消除下坡带来的定位姿态误差
        //
        // 按用户要求，这一步结束以前：
        //   clearance_optimizer/enabled 仍为 false
        //   enable_path_replanning      仍为 false
        // ============================================================
        if (!navigateToPose(
                2.50,
                4.75,
                0.5 * kSlopePi,
                "坡道终点原地旋转至90度"))
        {
            return failAndCleanup(
                "坡道终点90度姿态修正失败");
        }

        // ============================================================
        // 8. 两个规划器参数恢复
        // ============================================================
        if (!restoreSlopePlannerParameters())
        {
            ROS_ERROR(
                "坡道流程已经完成，但恢复规划器参数失败；"
                "请立即检查参数服务器");
            return false;
        }

        stopRobot();

        ROS_WARN(
            "国赛坡道阶段全部完成：当前逻辑截止于 "
            "(2.50,4.75,90°)，clearance_optimizer/enabled=true，"
            "enable_path_replanning=true；"
            "四项速度上限保持0.6；后续扫码流程暂未接入。");

        return true;
    }

private:
    std::string plannerParameter(
        const std::string& name) const
    {
        return planner_private_namespace_ + "/" + name;
    }

    bool waitForDependencies()
    {
        ROS_INFO(
            "等待 move_base、固定巡检路径锁、控制器复位接口...");

        while (
            ros::ok()
            && !move_base_.waitForServer(ros::Duration(3.0)))
        {
            ROS_INFO("仍在等待 move_base...");
        }

        if (!ros::ok())
        {
            return false;
        }

        if (!patrol_path_lock_client_.waitForExistence(
                ros::Duration(20.0)))
        {
            ROS_ERROR(
                "等待 %s/lock_patrol_path 超时",
                planner_private_namespace_.c_str());
            return false;
        }

        if (!controller_reset_client_.waitForExistence(
                ros::Duration(20.0)))
        {
            ROS_ERROR(
                "等待 %s/reset_controller_state 超时",
                planner_private_namespace_.c_str());
            return false;
        }

        const ros::WallTime subscriber_deadline =
            ros::WallTime::now()
            + ros::WallDuration(5.0);

        while (
            ros::ok()
            && patrol_path_pub_.getNumSubscribers() == 0
            && ros::WallTime::now() < subscriber_deadline)
        {
            ros::Duration(0.05).sleep();
        }

        if (patrol_path_pub_.getNumSubscribers() == 0)
        {
            ROS_ERROR(
                "固定巡检路径话题 %s/patrol_path 没有订阅者",
                planner_private_namespace_.c_str());
            return false;
        }

        // /initialpose 在 AMCL 启动以后应该存在订阅者。
        // 不把它作为绝对失败条件，但尽量等待，降低第一次消息丢失概率。
        const ros::WallTime initialpose_deadline =
            ros::WallTime::now()
            + ros::WallDuration(3.0);

        while (
            ros::ok()
            && initial_pose_pub_.getNumSubscribers() == 0
            && ros::WallTime::now() < initialpose_deadline)
        {
            ros::Duration(0.05).sleep();
        }

        if (initial_pose_pub_.getNumSubscribers() == 0)
        {
            ROS_WARN(
                "/initialpose 当前没有订阅者；仍继续运行，"
                "但请确认 AMCL 已经启动");
        }

        return ros::ok();
    }

    void stopRobot()
    {
        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
    }

    bool navigateToPose(
        double x,
        double y,
        double yaw,
        const std::string& purpose)
    {
        stopRobot();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame_;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation =
            tf2::toMsg(quaternion);

        ROS_WARN(
            "%s：发送 move_base 目标 (%.2f, %.2f, %.1f度)",
            purpose.c_str(),
            x,
            y,
            yaw * 180.0 / kSlopePi);

        move_base_.sendGoal(goal);

        const bool finished =
            move_base_.waitForResult(
                ros::Duration(navigation_timeout_));

        if (!finished)
        {
            move_base_.cancelGoal();
            stopRobot();

            ROS_ERROR(
                "%s超时 %.1fs，已取消目标",
                purpose.c_str(),
                navigation_timeout_);
            return false;
        }

        const actionlib::SimpleClientGoalState state =
            move_base_.getState();

        stopRobot();

        if (state !=
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR(
                "%s失败，move_base状态=%s",
                purpose.c_str(),
                state.toString().c_str());
            return false;
        }

        ROS_INFO(
            "%s完成",
            purpose.c_str());

        return true;
    }

    bool publishRelocalization(
        double x,
        double y,
        double yaw,
        const std::string& purpose)
    {
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.frame_id = map_frame_;

        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);
        pose.pose.pose.orientation =
            tf2::toMsg(quaternion);

        // 沿用项目中已有 AMCL initialpose 的置信度设置。
        for (std::size_t i = 0;
             i < pose.pose.covariance.size();
             ++i)
        {
            pose.pose.covariance[i] = 0.0;
        }

        pose.pose.covariance[0] = 0.01;
        pose.pose.covariance[7] = 0.01;
        pose.pose.covariance[35] = 0.0076;

        ROS_WARN(
            "%s：强制 AMCL 重定位为 "
            "(%.2f, %.2f, %.1f度)，连续发布%d次",
            purpose.c_str(),
            x,
            y,
            yaw * 180.0 / kSlopePi,
            relocalization_publish_count_);

        ros::Rate publish_rate(
            1.0 / relocalization_publish_interval_);

        for (
            int i = 0;
            ros::ok() && i < relocalization_publish_count_;
            ++i)
        {
            pose.header.stamp = ros::Time::now();
            initial_pose_pub_.publish(pose);

            ros::spinOnce();

            if (i + 1 < relocalization_publish_count_)
            {
                publish_rate.sleep();
            }
        }

        if (!ros::ok())
        {
            return false;
        }

        if (relocalization_settle_time_ > 0.0)
        {
            ROS_INFO(
                "等待 AMCL 重定位稳定 %.2fs",
                relocalization_settle_time_);

            ros::Duration(
                relocalization_settle_time_).sleep();
        }

        return ros::ok();
    }

    bool setDoubleParameter(
        const std::string& full_name,
        double value)
    {
        ros::param::set(full_name, value);

        ros::Duration(0.05).sleep();

        double readback = 0.0;
        if (!ros::param::get(full_name, readback))
        {
            ROS_ERROR(
                "设置参数 %s=%.3f 后读取失败",
                full_name.c_str(),
                value);
            return false;
        }

        if (std::fabs(readback - value) > 1.0e-6)
        {
            ROS_ERROR(
                "参数 %s 设置校验失败：期望=%.3f，实际=%.3f",
                full_name.c_str(),
                value,
                readback);
            return false;
        }

        ROS_WARN(
            "参数已设置：%s=%.3f",
            full_name.c_str(),
            value);

        return true;
    }

    bool setSlopeSpeedLimits(double value)
    {
        const std::string c2_speed =
            plannerParameter(
                "c2_max_reference_speed");

        const std::string mpc_vx =
            plannerParameter(
                "mpc_max_vx");

        const std::string mpc_trans =
            plannerParameter(
                "mpc_max_translational_speed");

        const std::string max_vx =
            plannerParameter(
                "max_vel_x");

        if (!setDoubleParameter(c2_speed, value))
            return false;

        if (!setDoubleParameter(mpc_vx, value))
            return false;

        if (!setDoubleParameter(mpc_trans, value))
            return false;

        if (!setDoubleParameter(max_vx, value))
            return false;

        ROS_WARN(
            "坡道阶段四项速度上限已统一设置为 %.2f："
            "c2_max_reference_speed、mpc_max_vx、"
            "mpc_max_translational_speed、max_vel_x",
            value);

        return true;
    }

    bool setBooleanParameter(
        const std::string& full_name,
        bool value)
    {
        ros::param::set(full_name, value);

        // 给参数服务器和控制循环一个很短的同步时间。
        ros::Duration(0.05).sleep();

        bool readback = !value;
        if (!ros::param::get(full_name, readback))
        {
            ROS_ERROR(
                "设置参数 %s=%s 后读取失败",
                full_name.c_str(),
                value ? "true" : "false");
            return false;
        }

        if (readback != value)
        {
            ROS_ERROR(
                "参数 %s 设置校验失败：期望=%s，实际=%s",
                full_name.c_str(),
                value ? "true" : "false",
                readback ? "true" : "false");
            return false;
        }

        ROS_WARN(
            "参数已设置：%s=%s",
            full_name.c_str(),
            value ? "true" : "false");

        return true;
    }

    bool setSlopePlannerParameters(bool enabled)
    {
        const std::string optimizer_enabled =
            plannerParameter(
                "clearance_optimizer/enabled");

        const std::string replanning_enabled =
            plannerParameter(
                "enable_path_replanning");

        // 关闭时先关 clearance optimizer，再关 replan；
        // 恢复时先恢复 clearance optimizer，再恢复 replan。
        if (!setBooleanParameter(
                optimizer_enabled,
                enabled))
        {
            return false;
        }

        if (!setBooleanParameter(
                replanning_enabled,
                enabled))
        {
            // 若第二项失败，尽量把第一项恢复到相反状态，
            // 避免只改成功一半。
            setBooleanParameter(
                optimizer_enabled,
                !enabled);
            return false;
        }

        slope_parameters_disabled_ = !enabled;

        ROS_WARN(
            "坡道规划器模式：clearance_optimizer/enabled=%s，"
            "enable_path_replanning=%s",
            enabled ? "true" : "false",
            enabled ? "true" : "false");

        return true;
    }

    bool restoreSlopePlannerParameters()
    {
        if (!setSlopePlannerParameters(true))
        {
            return false;
        }

        slope_parameters_disabled_ = false;
        return true;
    }

    bool resetPlannerControllerState()
    {
        std_srvs::Trigger service;

        if (!controller_reset_client_.call(service))
        {
            ROS_ERROR(
                "调用 %s/reset_controller_state 失败",
                planner_private_namespace_.c_str());
            return false;
        }

        if (!service.response.success)
        {
            ROS_ERROR(
                "局部规划器控制状态复位失败：%s",
                service.response.message.c_str());
            return false;
        }

        ROS_INFO(
            "局部规划器控制状态已复位：%s",
            service.response.message.c_str());

        return true;
    }

    bool requestPatrolPathLock(bool lock_path)
    {
        std_srvs::SetBool service;
        service.request.data = lock_path;

        if (!patrol_path_lock_client_.call(service))
        {
            ROS_ERROR(
                "调用 %s/lock_patrol_path 失败",
                planner_private_namespace_.c_str());
            return false;
        }

        if (!service.response.success)
        {
            ROS_ERROR(
                "%s固定巡检路径失败：%s",
                lock_path ? "锁定" : "解除",
                service.response.message.c_str());
            return false;
        }

        patrol_path_locked_ = lock_path;

        ROS_INFO(
            "%s固定巡检路径：%s",
            lock_path ? "已锁定" : "已解除",
            service.response.message.c_str());

        return true;
    }

    nav_msgs::Path buildSlopePath()
    {
        const double start_x = 4.75;
        const double start_y = 4.75;
        const double end_x = 2.50;
        const double end_y = 4.75;
        const double yaw = kSlopePi;

        const double length =
            std::hypot(
                end_x - start_x,
                end_y - start_y);

        const int intervals =
            std::max(
                1,
                static_cast<int>(
                    std::ceil(
                        length
                        / patrol_path_spacing_)));

        nav_msgs::Path path;
        path.header.frame_id = map_frame_;
        path.header.stamp = ros::Time::now();
        path.header.seq = ++path_sequence_;

        path.poses.reserve(
            static_cast<std::size_t>(
                intervals + 1));

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, yaw);

        for (int i = 0;
             i <= intervals;
             ++i)
        {
            const double ratio =
                static_cast<double>(i)
                / static_cast<double>(intervals);

            geometry_msgs::PoseStamped pose;
            pose.header = path.header;

            pose.pose.position.x =
                start_x
                + ratio * (end_x - start_x);

            pose.pose.position.y =
                start_y
                + ratio * (end_y - start_y);

            pose.pose.position.z = 0.0;
            pose.pose.orientation =
                tf2::toMsg(quaternion);

            path.poses.push_back(pose);
        }

        ROS_INFO(
            "已生成坡道固定 Path：起点=(%.2f,%.2f)，"
            "终点=(%.2f,%.2f)，长度=%.2fm，点数=%zu，方向=180度",
            start_x,
            start_y,
            end_x,
            end_y,
            length,
            path.poses.size());

        return path;
    }

    bool runForcedSlopePath()
    {
        if (patrol_path_locked_)
        {
            if (!requestPatrolPathLock(false))
            {
                return false;
            }
        }

        if (!resetPlannerControllerState())
        {
            return false;
        }

        nav_msgs::Path path =
            buildSlopePath();

        if (path.poses.size() < 2)
        {
            ROS_ERROR(
                "坡道固定 Path 点数不足");
            return false;
        }

        // 与找板巡检模式一致：
        // 反复发布本次 Path -> 请求 lock=true，
        // 直到规划器确认锁定或超时。
        const ros::WallTime deadline =
            ros::WallTime::now()
            + ros::WallDuration(
                patrol_interface_timeout_);

        bool locked = false;

        do
        {
            path.header.stamp =
                ros::Time::now();

            for (std::size_t i = 0;
                 i < path.poses.size();
                 ++i)
            {
                path.poses[i].header =
                    path.header;
            }

            patrol_path_pub_.publish(path);

            ros::Duration(0.08).sleep();

            locked =
                requestPatrolPathLock(true);

            if (!locked)
            {
                ros::Duration(0.08).sleep();
            }

        } while (
            ros::ok()
            && !locked
            && ros::WallTime::now() < deadline);

        if (!locked)
        {
            ROS_ERROR(
                "坡道固定 Path 未能在 %.1fs 内被局部规划器确认",
                patrol_interface_timeout_);
            return false;
        }

        // ============================================================
        // 关键：这里只发送“激活目标”，绝不把真正坡道终点直接交给
        // GlobalPlanner。
        //
        // 原因：
        //   move_base 在调用 MyPlanner::setPlan() 之前，
        //   必须先由 GlobalPlanner 成功 makePlan()。
        //   如果坡道在 global costmap 中被判为不可通行，
        //   直接发送 (2.50,4.75) 会在这里提前报：
        //       Failed to get a plan.
        //
        // 当前 MyPlanner 的 Patrol-R2 逻辑已经保证：
        //   lock_patrol_path=true
        //       -> staged_patrol_plan_ 取得 Reference 所有权
        //       -> 后续 setPlan() 中普通 move_base 路径会被替换
        //       -> 实际执行的 goal_pose_ 也是 staged patrol 的终点
        //
        // 所以 move_base 的这个近距离目标只负责让全局规划成功一次，
        // 从而启动 BaseLocalPlanner 控制循环；它不是真正运动目标。
        // ============================================================
        move_base_msgs::MoveBaseGoal activation_goal;
        activation_goal.target_pose.header.frame_id =
            map_frame_;
        activation_goal.target_pose.header.stamp =
            ros::Time::now();

        activation_goal.target_pose.pose.position.x =
            activation_goal_x_;
        activation_goal.target_pose.pose.position.y =
            activation_goal_y_;
        activation_goal.target_pose.pose.position.z =
            0.0;

        tf2::Quaternion activation_quaternion;
        activation_quaternion.setRPY(
            0.0,
            0.0,
            activation_goal_yaw_deg_ * kSlopePi / 180.0);

        activation_goal.target_pose.pose.orientation =
            tf2::toMsg(activation_quaternion);

        ROS_WARN(
            "坡道固定Reference已锁定：真实终点=(2.50,4.75,180°)；"
            "仅向move_base发送近距离激活目标=(%.2f,%.2f,%.1f度)，"
            "等待MyPlanner以固定Reference接管控制循环",
            activation_goal_x_,
            activation_goal_y_,
            activation_goal_yaw_deg_);

        move_base_.sendGoal(activation_goal);

        const bool finished =
            move_base_.waitForResult(
                ros::Duration(
                    navigation_timeout_));

        if (!finished)
        {
            move_base_.cancelGoal();
            stopRobot();

            ROS_ERROR(
                "坡道固定Reference执行超时 %.1fs；"
                "move_base激活目标已取消",
                navigation_timeout_);
            return false;
        }

        const actionlib::SimpleClientGoalState state =
            move_base_.getState();

        stopRobot();

        if (state !=
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR(
                "坡道固定Reference执行失败，move_base状态=%s",
                state.toString().c_str());
            return false;
        }

        ROS_WARN(
            "坡道固定Reference已完成：实际到达 "
            "(2.50,4.75,180°)；近距离move_base目标仅用于激活控制循环");

        return true;
    }

    bool failAndCleanup(
        const std::string& reason)
    {
        ROS_ERROR(
            "国赛坡道阶段失败：%s",
            reason.c_str());

        move_base_.cancelAllGoals();
        stopRobot();

        if (patrol_path_locked_)
        {
            requestPatrolPathLock(false);
        }

        if (slope_parameters_disabled_)
        {
            ROS_WARN(
                "异常退出：自动恢复坡道阶段关闭的两个规划器参数");

            restoreSlopePlannerParameters();
        }

        return false;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    MoveBaseClient move_base_;

    ros::Publisher initial_pose_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher patrol_path_pub_;

    ros::ServiceClient patrol_path_lock_client_;
    ros::ServiceClient controller_reset_client_;

    std::string map_frame_;
    std::string planner_private_namespace_;
    std::string cmd_vel_topic_;

    double navigation_timeout_;
    double patrol_path_spacing_;
    double patrol_interface_timeout_;

    double activation_goal_x_;
    double activation_goal_y_;
    double activation_goal_yaw_deg_;

    int relocalization_publish_count_;
    double relocalization_publish_interval_;
    double relocalization_settle_time_;

    bool patrol_path_locked_;
    bool slope_parameters_disabled_;

    uint32_t path_sequence_;
};



// ============================================================================
// V19：最终红绿灯 RKNN + 避障巡线模块
// 来源：当前实测 traffic_light2.cpp。
// YOLOv5模型映射：0=LEFT，1=RIGHT，2=STRAIGHT（实车测试已确认）。
// 路线保持：LEFT->/line2o_left，STRAIGHT->/lineo_right，RIGHT->/line2o_right。
// standalone 版巡线结束后的 ros::shutdown() 仅为集成需要改成返回 race 状态机。
// ============================================================================
class TrafficLightRecognizer
{
public:
    enum Direction
    {
        DIR_NONE = 0,
        DIR_LEFT,
        DIR_STRAIGHT,
        DIR_RIGHT,
        DIR_UNKNOWN
    };

    TrafficLightRecognizer()
        : nh_(),
          pnh_("/traffic_light"),
          route_started_(false),
          route_call_success_(false),
          camera_opened_(false),
          traffic_mode_ready_(false),
          stable_count_(0),
          last_direction_(DIR_NONE)
    {
        // =========================
        // ROS参数
        // =========================
        pnh_.param<std::string>(
            "cmd_vel_topic",
            cmd_vel_topic_,
            std::string("/cmd_vel"));

        pnh_.param<std::string>(
            "direction_topic",
            direction_topic_,
            std::string("/traffic_light/direction"));

        pnh_.param<std::string>(
            "detect_service",
            detect_service_name_,
            std::string("/nanodet_detect"));

        pnh_.param(
            "detect_rate",
            detect_rate_,
            8.0);

        pnh_.param(
            "service_wait_timeout",
            service_wait_timeout_,
            8.0);

        pnh_.param(
            "camera_warmup",
            camera_warmup_,
            0.70);

        pnh_.param(
            "camera_flush_calls",
            camera_flush_calls_,
            3);

        pnh_.param(
            "stable_frames",
            stable_frames_,
            3);

        // 正常交通灯检测命令。
        // detect2026.py中：
        //   10 -> traffic normal threshold
        //   14 -> traffic low threshold
        pnh_.param(
            "traffic_detect_command",
            traffic_detect_command_,
            10);

        // 独立测试阶段默认正常阈值。
        // 若后续现场目标较远，可临时改成14测试较低阈值。
        pnh_.param(
            "traffic_switch_command",
            traffic_switch_command_,
            -10);

        if (detect_rate_ < 1.0)
        {
            ROS_WARN(
                "detect_rate=%.2f过低，自动修正为1.0Hz",
                detect_rate_);
            detect_rate_ = 1.0;
        }

        if (stable_frames_ < 1)
        {
            ROS_WARN(
                "stable_frames=%d非法，自动修正为1",
                stable_frames_);
            stable_frames_ = 1;
        }

        camera_flush_calls_ =
            std::max(
                0,
                camera_flush_calls_);

        // =========================
        // ROS发布/服务客户端
        // =========================
        cmd_pub_ =
            nh_.advertise<geometry_msgs::Twist>(
                cmd_vel_topic_,
                1);

        direction_pub_ =
            nh_.advertise<std_msgs::String>(
                direction_topic_,
                1,
                true);

        detect_client_ =
            nh_.serviceClient<ros_nanodet::detect_result_srv>(
                detect_service_name_);

        left_client_ =
            nh_.serviceClient<line_follow::line_follow>(
                "/line2o_left");

        straight_client_ =
            nh_.serviceClient<line_follow::line_follow>(
                "/lineo_right");

        right_client_ =
            nh_.serviceClient<line_follow::line_follow>(
                "/line2o_right");

        ROS_INFO(
            "YOLOv5红绿灯RKNN模块启动："
            "YOLOv5模型映射 0=左转，1=右转，2=直行；"
            "检测服务=%s，稳定帧=%d，检测频率=%.1fHz",
            detect_service_name_.c_str(),
            stable_frames_,
            detect_rate_);
    }

    ~TrafficLightRecognizer()
    {
        publishStop();
        releaseCamera();
    }

    bool initialize()
    {
        publishStop();

        ROS_INFO(
            "等待RKNN检测服务：%s",
            detect_service_name_.c_str());

        if (!detect_client_.waitForExistence(
                ros::Duration(
                    service_wait_timeout_)))
        {
            ROS_ERROR(
                "检测服务%s在%.1fs内未出现",
                detect_service_name_.c_str(),
                service_wait_timeout_);
            return false;
        }

        if (!switchToTrafficModel())
        {
            return false;
        }

        if (!openCamera())
        {
            return false;
        }

        ROS_INFO(
            "红绿灯摄像头预热%.2fs",
            camera_warmup_);

        ros::WallDuration(
            std::max(
                0.0,
                camera_warmup_))
            .sleep();

        for (int i = 0;
             i < camera_flush_calls_;
             ++i)
        {
            if (!flushCamera())
            {
                ROS_WARN(
                    "第%d/%d次清理摄像头缓存失败，继续测试",
                    i + 1,
                    camera_flush_calls_);
            }
        }

        ROS_INFO(
            "红绿灯RKNN识别初始化完成，开始等待稳定方向");

        return true;
    }

    bool run()
    {
        ros::WallRate rate(
            detect_rate_);

        while (
            ros::ok() &&
            !route_started_)
        {
            // 在方向正式确认前，小车必须持续停车。
            publishStop();

            Direction direction =
                DIR_NONE;

            cvBoxInfo box;
            int detection_count = 0;

            const bool detect_ok =
                detectTrafficDirection(
                    direction,
                    box,
                    detection_count);

            if (!detect_ok)
            {
                resetStable();
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            if (
                direction == DIR_NONE ||
                direction == DIR_UNKNOWN)
            {
                ROS_INFO_THROTTLE(
                    0.5,
                    "红绿灯：当前帧未发现有效绿箭头 -> 保持停车");

                resetStable();
            }
            else
            {
                ROS_INFO_THROTTLE(
                    0.3,
                    "红绿灯检测：direction=%s，"
                    "box=(%d,%d)-(%d,%d)，"
                    "本帧候选数=%d",
                    directionName(
                        direction).c_str(),
                    box.x0,
                    box.y0,
                    box.x1,
                    box.y1,
                    detection_count);

                updateStableResult(
                    direction);
            }

            ros::spinOnce();
            rate.sleep();
        }
        publishStop();

        return (
            ros::ok()
            && route_started_
            && route_call_success_
        );
    }

private:
    struct cvBoxInfo
    {
        int x0 = -1;
        int y0 = -1;
        int x1 = -1;
        int y1 = -1;
    };

    void publishStop()
    {
        geometry_msgs::Twist stop;
        cmd_pub_.publish(stop);
    }

    std::string directionName(
        Direction direction) const
    {
        switch (direction)
        {
        case DIR_LEFT:
            return "LEFT";

        case DIR_RIGHT:
            return "RIGHT";

        case DIR_STRAIGHT:
            return "STRAIGHT";

        case DIR_UNKNOWN:
            return "UNKNOWN";

        default:
            return "WAIT";
        }
    }

    Direction classIdToDirection(
        int class_id) const
    {
        // 当前YOLOv5模型固定映射（实车测试已确认）：
        // 0 左转，1 右转，2 直行。
        switch (class_id)
        {
        case 0:
            return DIR_LEFT;

        case 1:
            return DIR_RIGHT;

        case 2:
            return DIR_STRAIGHT;

        default:
            return DIR_UNKNOWN;
        }
    }

    bool callDetectorCommand(
        int command,
        ros_nanodet::detect_result_srv &srv)
    {
        srv.request.detect_start =
            command;

        if (!detect_client_.call(srv))
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "调用%s失败，command=%d",
                detect_service_name_.c_str(),
                command);
            return false;
        }

        return true;
    }

    bool switchToTrafficModel()
    {
        ros_nanodet::detect_result_srv srv;

        ROS_WARN(
            "请求detect2026切换到YOLOv5红绿灯三分类RKNN模型...");

        if (!callDetectorCommand(
                traffic_switch_command_,
                srv))
        {
            return false;
        }

        if (
            srv.response.class_name.empty() ||
            srv.response.class_name[0] !=
                traffic_switch_command_)
        {
            const int response_code =
                srv.response.class_name.empty()
                ? -999
                : srv.response.class_name[0];

            ROS_ERROR(
                "红绿灯模型切换失败：返回码=%d，"
                "期望=%d",
                response_code,
                traffic_switch_command_);

            return false;
        }

        traffic_mode_ready_ =
            true;

        ROS_INFO(
            "YOLOv5红绿灯三分类RKNN模型已就绪");

        return true;
    }

    bool openCamera()
    {
        ros_nanodet::detect_result_srv srv;

        if (!callDetectorCommand(
                -1,
                srv))
        {
            return false;
        }

        camera_opened_ = true;

        ROS_INFO(
            "detect2026摄像头已打开");

        return true;
    }

    bool flushCamera()
    {
        if (!camera_opened_)
        {
            return false;
        }

        ros_nanodet::detect_result_srv srv;

        return callDetectorCommand(
            -3,
            srv);
    }

    void releaseCamera()
    {
        if (!camera_opened_)
        {
            return;
        }

        // 析构/切控制权阶段不要因为一次服务异常反复报错。
        ros_nanodet::detect_result_srv srv;
        srv.request.detect_start = -2;

        if (detect_client_.exists())
        {
            if (detect_client_.call(srv))
            {
                ROS_INFO(
                    "红绿灯阶段已释放detect2026摄像头");
            }
            else
            {
                ROS_WARN(
                    "释放detect2026摄像头服务调用失败");
            }
        }

        camera_opened_ = false;
    }

    bool detectTrafficDirection(
        Direction &direction,
        cvBoxInfo &best_box,
        int &detection_count)
    {
        direction =
            DIR_NONE;

        best_box =
            cvBoxInfo();

        detection_count = 0;

        if (
            !traffic_mode_ready_ ||
            !camera_opened_)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "红绿灯RKNN尚未初始化完成");
            return false;
        }

        ros_nanodet::detect_result_srv srv;

        if (!callDetectorCommand(
                traffic_detect_command_,
                srv))
        {
            return false;
        }

        const size_t count =
            std::min(
                srv.response.class_name.size(),
                std::min(
                    srv.response.x0.size(),
                    std::min(
                        srv.response.y0.size(),
                        std::min(
                            srv.response.x1.size(),
                            srv.response.y1.size()))));

        detection_count =
            static_cast<int>(count);

        if (count == 0)
        {
            return true;
        }

        // Python端保证按照置信度从高到低返回。
        // 找到第一个合法0/1/2类别即可。
        for (size_t i = 0;
             i < count;
             ++i)
        {
            const int class_id =
                srv.response.class_name[i];

            const Direction candidate =
                classIdToDirection(
                    class_id);

            if (
                candidate == DIR_UNKNOWN ||
                candidate == DIR_NONE)
            {
                ROS_WARN(
                    "忽略未知红绿灯类别ID=%d",
                    class_id);
                continue;
            }

            direction =
                candidate;

            best_box.x0 =
                srv.response.x0[i];

            best_box.y0 =
                srv.response.y0[i];

            best_box.x1 =
                srv.response.x1[i];

            best_box.y1 =
                srv.response.y1[i];

            return true;
        }

        direction =
            DIR_UNKNOWN;

        return true;
    }

    void resetStable()
    {
        stable_count_ = 0;
        last_direction_ = DIR_NONE;
    }

    void updateStableResult(
        Direction direction)
    {
        if (
            direction == DIR_NONE ||
            direction == DIR_UNKNOWN)
        {
            resetStable();
            return;
        }

        if (
            direction ==
            last_direction_)
        {
            ++stable_count_;
        }
        else
        {
            last_direction_ =
                direction;

            stable_count_ = 1;
        }

        ROS_INFO(
            "红绿灯候选=%s，稳定=%d/%d",
            directionName(
                direction).c_str(),
            stable_count_,
            stable_frames_);

        if (
            stable_count_ <
            stable_frames_)
        {
            return;
        }

        std_msgs::String result;
        result.data =
            directionName(
                direction);

        direction_pub_.publish(
            result);

        ROS_WARN(
            "红绿灯方向确认：%s",
            result.data.c_str());

        callRouteService(
            direction);
    }

    void callRouteService(
        Direction direction)
    {
        ros::ServiceClient *client =
            nullptr;

        const char *service_name =
            nullptr;

        if (direction == DIR_LEFT)
        {
            client =
                &left_client_;

            service_name =
                "/line2o_left";
        }
        else if (
            direction == DIR_RIGHT)
        {
            client =
                &right_client_;

            service_name =
                "/line2o_right";
        }
        else if (
            direction == DIR_STRAIGHT)
        {
            client =
                &straight_client_;

            service_name =
                "/lineo_right";
        }
        else
        {
            return;
        }

        // 先检查巡线服务存在。
        // 服务不存在时继续停车和红绿灯识别，
        // 不提前释放摄像头。
        if (!client->waitForExistence(
                ros::Duration(1.0)))
        {
            ROS_ERROR(
                "巡线服务%s不存在 -> 保持停车并继续识别",
                service_name);

            resetStable();
            publishStop();
            return;
        }

        // 到这里方向已经正式锁定。
        route_started_ = true;

        publishStop();

        // 巡线程序也要使用/dev/video0，
        // 所以必须先让detect2026释放摄像头。
        releaseCamera();

        ros::WallDuration(
            0.05)
            .sleep();

        ROS_WARN(
            "红绿灯方向=%s，调用巡线服务：%s",
            directionName(
                direction).c_str(),
            service_name);

        line_follow::line_follow srv;

        if (!client->call(srv))
        {
            ROS_ERROR(
                "调用巡线服务%s失败",
                service_name);

            route_call_success_ = false;
            publishStop();
            return;
        }

        route_call_success_ = true;

        ROS_INFO(
            "巡线服务%s执行结束，返回race主状态机",
            service_name);

        publishStop();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher cmd_pub_;
    ros::Publisher direction_pub_;

    ros::ServiceClient detect_client_;
    ros::ServiceClient left_client_;
    ros::ServiceClient straight_client_;
    ros::ServiceClient right_client_;

    std::string cmd_vel_topic_;
    std::string direction_topic_;
    std::string detect_service_name_;

    double detect_rate_;
    double service_wait_timeout_;
    double camera_warmup_;

    int camera_flush_calls_;
    int stable_frames_;
    int traffic_detect_command_;
    int traffic_switch_command_;

    bool route_started_;
    bool route_call_success_;
    bool camera_opened_;
    bool traffic_mode_ready_;

    int stable_count_;
    Direction last_direction_;
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    signal(SIGPIPE, SIG_IGN);

    ros::init(argc, argv, "main_competition_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // ============================================================
    // 仿真通信
    // ============================================================
    simulation_target_pub =
        nh.advertise<std_msgs::String>(
            "/detected_target",
            1,
            false
        );

    simulation_result_sub =
        nh.subscribe<std_msgs::String>(
            "/car_task_finished",
            10,
            simulationResultCallback
        );

    // ============================================================
    // 语音 / VAD 参数
    // 完全沿用最新 race.cpp 默认值。
    // ============================================================
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
    private_nh.param(
        "retry_pcm_guard_seconds",
        retry_pcm_guard_seconds,
        0.15
    );

    // ============================================================
    // TTS 参数
    // 最终确定使用 1.50x。
    // ============================================================
    private_nh.param(
        "tts_trim_silence",
        tts_trim_silence,
        true
    );
    private_nh.param(
        "tts_trim_silence_threshold",
        tts_trim_silence_threshold,
        200
    );
    private_nh.param(
        "tts_trim_keep_ms",
        tts_trim_keep_ms,
        20.0
    );
    private_nh.param(
        "tts_crossfade_ms",
        tts_crossfade_ms,
        8.0
    );
    private_nh.param(
        "tts_playback_speed",
        tts_playback_speed,
        1.50
    );

    private_nh.param(
        "simulation_result_timeout",
        simulation_result_timeout,
        120.0
    );

    simulation_result_timeout =
        max(
            1.0,
            simulation_result_timeout
        );

    ROS_INFO(
        "仿真任务稳定性超时：%.1fs；"
        "超时无结果将自动视为完成",
        simulation_result_timeout
    );

    tts_trim_silence_threshold =
        max(0, min(32767, tts_trim_silence_threshold));
    tts_trim_keep_ms =
        max(0.0, tts_trim_keep_ms);
    tts_crossfade_ms =
        max(0.0, tts_crossfade_ms);

    if (tts_playback_speed < 0.50) {
        ROS_WARN(
            "tts_playback_speed=%.2f 太小，自动修正为 0.50",
            tts_playback_speed
        );
        tts_playback_speed = 0.50;
    } else if (tts_playback_speed > 2.00) {
        ROS_WARN(
            "tts_playback_speed=%.2f 太大，自动修正为 2.00",
            tts_playback_speed
        );
        tts_playback_speed = 2.00;
    }

    ROS_INFO(
        "TTS 连续播放参数：trim=%s，threshold=%d，"
        "keep=%.1fms，crossfade=%.1fms，speed=%.2fx",
        tts_trim_silence ? "true" : "false",
        tts_trim_silence_threshold,
        tts_trim_keep_ms,
        tts_crossfade_ms,
        tts_playback_speed
    );

    // ============================================================
    // 二维码参数
    // ============================================================
    loadQrParameters(private_nh);

    // ============================================================
    // ROS 服务
    // ============================================================
    semantic_client =
        nh.serviceClient<ucarmain2026::GetTaskSemantics>(
            "/get_task_semantics"
        );

    qr_client =
        nh.serviceClient<qr_01::qr_code>(
            "qr_detect"
        );

    classifier_client =
        nh.serviceClient<ucarmain2026::ItemClassify>(
            "/get_item_classification"
        );

    // ============================================================
    // 语音唤醒 + 同流 PCM
    // ============================================================
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

    // 禁用 speech_command_node 旧版固定时长写盘。
    // speech_command_node 只负责唤醒和持续发布同流 PCM；
    // 真正录音由本程序启动 vad_record.py 完成。
    nh.setParam(
        "/speech_command/internal_task_recording",
        false
    );

    MoveBaseClient ac("move_base", true);

    ROS_INFO(
        "等待 Spark 语义服务 /get_task_semantics..."
    );
    semantic_client.waitForExistence();

    if (!ros::ok()) {
        stopVadProcess();
        return 1;
    }

    // 防止 Spark 服务读取上一轮比赛遗留录音。
    remove(AUDIO_FILE);
    remove(vad_status_file.c_str());

    current_state = WAIT_WAKEUP;
    wakeup_received = false;

    ROS_INFO(
        "国赛总控节点已启动！请说“小飞小飞”唤醒..."
    );
    ROS_INFO(
        "VAD 参数：min=%.1fs，silence=%.1fs，"
        "max=%.1fs，tail=%.2fs，backend=%s，"
        "重录尾音保护=%.2fs",
        vad_min_seconds,
        vad_silence_seconds,
        vad_max_seconds,
        vad_tail_seconds,
        vad_backend.c_str(),
        retry_pcm_guard_seconds
    );

    ros::Rate rate(20);

    while (
        ros::ok()
        && current_state != ALL_FINISHED
    ) {
        ros::spinOnce();

        switch (current_state) {

            // ====================================================
            // 1. 唤醒
            // ====================================================
            case WAIT_WAKEUP:
            {
                if (wakeup_received) {
                    awake_sub.shutdown();

                    // awakeCallback 已经在同一条 PCM 流上启动 VAD，
                    // 所以这里只切状态，不重新开启录音。
                    current_state = RECORDING;
                }
                break;
            }

            // ====================================================
            // 2. VAD 动态录音
            // ====================================================
            case RECORDING:
            {
                const VadPollResult vad_result =
                    pollVadProcess();

                if (
                    vad_result
                    == VAD_RECORDING_SUCCEEDED
                ) {
                    current_state =
                        SEMANTIC_PARSING;
                } else if (
                    vad_result
                        == VAD_RECORDING_FAILED
                    || vad_result
                        == VAD_NOT_RUNNING
                ) {
                    ROS_WARN(
                        "录音失败，播放提示音后直接重新录制..."
                    );

                    if (!startRetryRecording()) {
                        ROS_ERROR(
                            "无法继续任务录音，结束本次任务"
                        );
                        current_state =
                            ALL_FINISHED;
                    }
                }
                break;
            }

            // ====================================================
            // 3. Spark语义解析
            //
            // 成功以后不再直接进入二维码，
            // 而是先执行新增的国赛坡道阶段。
            // ====================================================
            case SEMANTIC_PARSING:
            {
                ucarmain2026::GetTaskSemantics
                    srv_task;

                if (
                    semantic_client.call(srv_task)
                    && srv_task.response.success
                ) {
                    target_real =
                        srv_task.response.target_real;
                    target_sim =
                        srv_task.response.target_sim;

                    ROS_INFO(
                        "语义解析成功！实体区=[%s]，仿真区=[%s]",
                        target_real.c_str(),
                        target_sim.c_str()
                    );

                    const bool real_valid =
                        target_real == "food"
                        || target_real == "daily"
                        || target_real
                            == "electronic";

                    const bool sim_valid =
                        target_sim == "food"
                        || target_sim == "daily"
                        || target_sim
                            == "electronic";

                    if (
                        !real_valid
                        || !sim_valid
                        || target_real == target_sim
                    ) {
                        ROS_WARN(
                            "语义服务虽然返回success，但类别非法："
                            "real=[%s] sim=[%s]；"
                            "播放提示音后重新录制",
                            target_real.c_str(),
                            target_sim.c_str()
                        );

                        target_real.clear();
                        target_sim.clear();

                        if (
                            startRetryRecording()
                        ) {
                            current_state =
                                RECORDING;
                        } else {
                            ROS_ERROR(
                                "无法重新启动VAD，结束本次任务"
                            );
                            current_state =
                                ALL_FINISHED;
                        }
                        break;
                    }

                    // 语义已经最终确认。
                    // 后续不再需要麦克风，立即结束 speech_command_node，
                    // 释放音频设备和后台资源。
                    stopSpeechCommandNodeFast();

                    current_state =
                        SLOPE_STAGE;
                } else {
                    ROS_WARN(
                        "解析失败！播放提示音后直接重新录制..."
                    );

                    if (
                        startRetryRecording()
                    ) {
                        current_state =
                            RECORDING;
                    } else {
                        ROS_ERROR(
                            "无法重新启动 VAD，结束本次任务"
                        );
                        current_state =
                            ALL_FINISHED;
                    }
                }
                break;
            }

            // ====================================================
            // 4. 国赛扫码前坡道
            //
            // 这里严格执行当前已验证 race_2 的坡道流程。
            // 成功后才允许进入二维码观察点导航。
            // ====================================================
            case SLOPE_STAGE:
            {
                ROS_WARN(
                    "语音任务确认完成，开始国赛坡道阶段："
                    "实体区=%s，仿真区=%s",
                    target_real.c_str(),
                    target_sim.c_str()
                );

                NationalSlopeStage slope_stage;

                if (!slope_stage.run()) {
                    ROS_ERROR(
                        "国赛坡道阶段失败，"
                        "禁止继续二维码和后续任务"
                    );
                    current_state =
                        ALL_FINISHED;
                    break;
                }

                ROS_WARN(
                    "国赛坡道阶段完成，"
                    "开始二维码识别流程"
                );

                current_state = NAVIGATING;
                break;
            }

            case NAVIGATING:
            {
                ac.waitForServer();

                if (valid_qr_count >= 3) {
                    stopQrCamera();
                    cleanupQrHttpSession();
                    ROS_INFO("已收集 3 个有效二维码，进入分类阶段");
                    current_state = ITEM_CLASSIFYING;
                    break;
                }

                if (qr_waypoint_idx >= static_cast<int>(qr_wp_x.size())) {
                    if (qr_scan_round + 1 < qr_max_scan_rounds) {
                        qr_scan_round++;
                        qr_waypoint_idx = 0;
                        qr_view_mode = QR_VIEW_NORMAL;
                        qr_decoded_any_at_current_view = false;
                        qr_url_failures_this_round.clear();

                        ROS_WARN(
                            "第 %d 轮扫码结束，目前仅收集 %d/3 个二维码；"
                            "开始第 %d 轮兜底扫描，每点 %.2f 秒",
                            qr_scan_round,
                            valid_qr_count,
                            qr_scan_round + 1,
                            currentQrScanTimeout()
                        );
                        break;
                    }

                    stopQrCamera();
                    cleanupQrHttpSession();

                    ROS_ERROR(
                        "二维码扫码失败：完成 %d 轮观察后仅获得 %d/3 个有效结果；"
                        "启用固定三类别物品兜底，不再终止比赛流程。",
                        qr_max_scan_rounds,
                        valid_qr_count
                    );

                    // V20稳定性兜底：
                    // 一旦完整扫码流程最终没有得到3个有效二维码，
                    // 为确保 food / daily / electronic 三个类别一定都存在，
                    // 不继续使用不完整候选，而统一替换为三个已准备好音频的
                    // 固定中文物品。
                    //
                    // 食品       -> 牛奶
                    // 日用品     -> 水杯
                    // 电子产品   -> 显示器
                    target_qr_1 = "牛奶";
                    target_qr_2 = "水杯";
                    target_qr_3 = "显示器";
                    valid_qr_count = 3;

                    ROS_WARN(
                        "二维码兜底候选已设置："
                        "[牛奶, 水杯, 显示器]；"
                        "继续使用原ItemClassify服务按本轮实体/仿真类别选择目标。"
                    );

                    current_state = ITEM_CLASSIFYING;
                    break;
                }

                const double target_qr_yaw = currentQrViewYaw();

                if (qr_view_mode == QR_VIEW_NORMAL) {
                    ROS_INFO(
                        "第 %d/%d 轮：前往二维码观察点 %d/%zu "
                        "[x=%.2f, y=%.2f, yaw=%.2f]",
                        qr_scan_round + 1,
                        qr_max_scan_rounds,
                        qr_waypoint_idx + 1,
                        qr_wp_x.size(),
                        qr_wp_x[qr_waypoint_idx],
                        qr_wp_y[qr_waypoint_idx],
                        target_qr_yaw
                    );
                } else {
                    ROS_INFO(
                        "观察点 %d 视觉识别 fallback：保持位置 [x=%.2f, y=%.2f]，"
                        "%s %.1f°，目标 yaw=%.2f",
                        qr_waypoint_idx + 1,
                        qr_wp_x[qr_waypoint_idx],
                        qr_wp_y[qr_waypoint_idx],
                        qr_view_mode == QR_VIEW_LEFT ? "左转" : "右转",
                        qr_fallback_yaw_deg,
                        target_qr_yaw
                    );
                }

                if (
                    !go_destination(
                        qr_wp_x[qr_waypoint_idx],
                        qr_wp_y[qr_waypoint_idx],
                        target_qr_yaw,
                        ac
                    )
                ) {
                    if (qr_view_mode == QR_VIEW_NORMAL) {
                        ROS_WARN(
                            "无法到达二维码观察点 %d，直接尝试下一个点",
                            qr_waypoint_idx + 1
                        );
                        advanceQrWaypoint();
                    } else if (qr_view_mode == QR_VIEW_LEFT) {
                        ROS_WARN(
                            "观察点 %d 左转 fallback 到位失败，继续尝试右转 fallback",
                            qr_waypoint_idx + 1
                        );
                        qr_view_mode = QR_VIEW_RIGHT;
                    } else {
                        ROS_WARN(
                            "观察点 %d 右转 fallback 到位失败，放弃该观察点",
                            qr_waypoint_idx + 1
                        );
                        advanceQrWaypoint();
                    }
                    break;
                }

                // 相机在整个二维码阶段只打开一次；移动到后续观察点时保持开启。
                if (!ensureQrCameraReady()) {
                    ROS_ERROR(
                        "二维码摄像头无法启动，本观察点跳过；下一点将再次尝试"
                    );
                    advanceQrWaypoint();
                    break;
                }

                if (!flushQrCamera()) {
                    ROS_WARN(
                        "观察点 %d 到位后缓存清理失败，仍尝试读取实时画面",
                        qr_waypoint_idx + 1
                    );
                }

                // 失败 URL 只在当前观察点内限次；到新观察点后允许再次尝试。
                qr_url_failures_this_round.clear();
                scan_start_time = ros::Time::now();
                qr_found_item_at_current_waypoint = false;
                qr_decoded_any_at_current_view = false;
                qr_post_success_deadline = ros::Time(0);
                ROS_INFO(
                    "观察点 %d [%s] 开始扫码，本视角视觉驻留上限 %.2f 秒，当前已收集 %d/3",
                    qr_waypoint_idx + 1,
                    currentQrViewName(),
                    currentQrScanTimeout(),
                    valid_qr_count
                );
                current_state = QR_SCANNING;
                break;
            }

            case QR_SCANNING:
            {
                if (valid_qr_count >= 3) {
                    stopQrCamera();
                    cleanupQrHttpSession();
                    ROS_INFO("成功收集 3 个有效二维码，立即进入分类阶段");
                    current_state = ITEM_CLASSIFYING;
                    break;
                }

                const ros::Time now = ros::Time::now();
                const double elapsed =
                    (now - scan_start_time).toSec();

                // 官方场景是一面墙一个二维码：一旦本墙已成功获得有效内容，
                // 只再保留一个很短的追加窗口。若同帧或紧邻几帧还有其他二维码，
                // 仍会继续处理；否则窗口到期后立即前往下一观察点。
                if (
                    qr_found_item_at_current_waypoint
                    && now >= qr_post_success_deadline
                ) {
                    ROS_INFO(
                        "观察点 %d 已获得有效二维码，追加扫描 %.2f 秒结束；"
                        "立即前往下一观察点",
                        qr_waypoint_idx + 1,
                        max(0.0, qr_post_success_scan_seconds)
                    );
                    advanceQrWaypoint();
                    current_state = NAVIGATING;
                    break;
                }

                if (elapsed > currentQrScanTimeout()) {
                    // 只有当前整个视角一张二维码都没有解出来，才认为是视觉失败。
                    // 如果解出了旧二维码或 HTTP 后续失败，说明相机/ZBar已经看到了码，
                    // 左右转并不能解决网络问题，因此不触发角度 fallback。
                    if (qr_decoded_any_at_current_view) {
                        ROS_INFO(
                            "观察点 %d [%s] 扫描结束：本视角已成功解出二维码，"
                            "但未新增有效物品；不触发角度 fallback，前往下一观察点",
                            qr_waypoint_idx + 1,
                            currentQrViewName()
                        );
                        advanceQrWaypoint();
                        current_state = NAVIGATING;
                        break;
                    }

                    if (qr_view_mode == QR_VIEW_NORMAL) {
                        ROS_WARN(
                            "观察点 %d 正常朝向 %.2f 秒内完全未解出二维码；"
                            "启动视觉 fallback：原地左转 %.1f°",
                            qr_waypoint_idx + 1,
                            currentQrScanTimeout(),
                            qr_fallback_yaw_deg
                        );
                        qr_view_mode = QR_VIEW_LEFT;
                        current_state = NAVIGATING;
                        break;
                    }

                    if (qr_view_mode == QR_VIEW_LEFT) {
                        ROS_WARN(
                            "观察点 %d 左转 %.1f° 后仍完全未解出二维码；"
                            "继续尝试原朝向右转 %.1f°",
                            qr_waypoint_idx + 1,
                            qr_fallback_yaw_deg,
                            qr_fallback_yaw_deg
                        );
                        qr_view_mode = QR_VIEW_RIGHT;
                        current_state = NAVIGATING;
                        break;
                    }

                    ROS_WARN(
                        "观察点 %d 右转 %.1f° 后仍完全未解出二维码；"
                        "左右视角 fallback 均失败，前往下一观察点",
                        qr_waypoint_idx + 1,
                        qr_fallback_yaw_deg
                    );
                    advanceQrWaypoint();
                    current_state = NAVIGATING;
                    break;
                }

                qr_01::qr_code srv;
                srv.request.command = 1;

                if (!qr_client.call(srv)) {
                    ROS_WARN_THROTTLE(1.0, "二维码识别服务调用失败");
                    break;
                }

                if (srv.response.result.empty()) {
                    break;
                }

                const bool found_new_item =
                    processQrServerResponse(srv.response.result);

                if (valid_qr_count >= 3) {
                    stopQrCamera();
                    ROS_INFO("成功收集 3 个有效二维码，立即进入分类阶段");
                    current_state = ITEM_CLASSIFYING;
                    break;
                }

                if (found_new_item) {
                    ROS_INFO(
                        "当前观察点获得新二维码；再观察 %.2f 秒确认是否还有其他二维码，"
                        "当前 %d/3",
                        max(0.0, qr_post_success_scan_seconds),
                        valid_qr_count
                    );
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
                    const string classified_real_item =
                        classify_srv.response.real_item;
                    const string classified_sim_item =
                        classify_srv.response.sim_item;

                    // 分类服务通常必须返回中文播报名。
                    // 唯一允许的英文字符例外是精确名称“U盘”；
                    // 其他英文/中英混合名称仍禁止进入TTS并重新请求分类。
                    if (
                        !isChineseTtsItemText(classified_real_item)
                        || !isChineseTtsItemText(classified_sim_item)
                    ) {
                        ROS_ERROR(
                            "分类服务返回了非法TTS物品名："
                            "real=[%s] sim=[%s]；"
                            "普通物品必须为中文，唯一英文字符例外为“U盘”；"
                            "拒绝进入语音合成，重新请求分类",
                            classified_real_item.c_str(),
                            classified_sim_item.c_str()
                        );
                        ros::Duration(0.5).sleep();
                        break;
                    }

                    final_real_item = classified_real_item;
                    final_sim_item = classified_sim_item;

                    ROS_INFO(
                        "分类成功：实体区中文播报名=[%s]，"
                        "仿真区中文播报名=[%s]",
                        final_real_item.c_str(),
                        final_sim_item.c_str()
                    );

                    current_state = TTS_BROADCASTING;
                } else {
                    ROS_WARN(
                        "物品分类服务本次失败，2秒后重新请求"
                    );
                    ros::Duration(2.0).sleep();
                }
                break;
            }

            case TTS_BROADCASTING:
            {
                if (!prepareItemAudios(final_real_item, final_sim_item)) {
                    ROS_ERROR("物品音频准备失败，终止后续完整比赛流程");
                    current_state = ALL_FINISHED;
                    break;
                }

                CategoryAudioTexts real_audio_texts;
                CategoryAudioTexts sim_audio_texts;

                if (
                    !getCategoryAudioTexts(target_real, real_audio_texts)
                    || !getCategoryAudioTexts(target_sim, sim_audio_texts)
                ) {
                    ROS_ERROR("类别固定音频映射失败，终止后续完整比赛流程");
                    current_state = ALL_FINISHED;
                    break;
                }

                // 只播二维码识别后的第一部分。
                if (!playInitialTaskBroadcast(
                        final_real_item,
                        final_sim_item,
                        real_audio_texts,
                        sim_audio_texts
                    )) {
                    ROS_ERROR("第一阶段播报失败；继续执行比赛流程");
                }

                // 按要求把局部规划器终点接管距离改为0.30m。
                const string goal_threshold_param =
                    "/move_base/MyPlanner/goal_dist_threshold";

                ros::param::set(goal_threshold_param, 0.30);

                double confirmed_goal_threshold = -1.0;
                ros::param::get(
                    goal_threshold_param,
                    confirmed_goal_threshold
                );

                ROS_WARN(
                    "已设置 %s = %.3f",
                    goal_threshold_param.c_str(),
                    confirmed_goal_threshold
                );

                // 先到用户要求的 -90° 衔接姿态。
                ROS_WARN(
                    "前往找板衔接点 (1.25,4.30,-90°)"
                );

                if (!go_destination(
                        1.25,
                        4.30,
                        -0.5 * 3.14159265358979323846,
                        ac
                    )) {
                    ROS_ERROR(
                        "无法到达 (1.25,4.30,-90°)，终止找板流程"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                // V14.9内部起点调整更新为(1.25,4.30,0°)。
                bool real_broadcast_done = false;

                const auto real_docked_callback =
                    [&]() {
                        if (real_broadcast_done) {
                            return;
                        }

                        real_broadcast_done = true;

                        if (!playRealDockedBroadcast(
                                final_real_item,
                                real_audio_texts
                            )) {
                            ROS_ERROR(
                                "现实目标已停靠，但中间播报失败；继续后续流程"
                            );
                        }
                    };

                ROS_WARN(
                    "进入 V14.9 找板：现实类别=%s，仿真类别=%s；"
                    "先执行 (1.25,4.30,0°) 姿态调整",
                    target_real.c_str(),
                    target_sim.c_str()
                );

                TargetPatrolDocking patrol(
                    target_real,
                    target_sim,
                    real_docked_callback
                );

                const bool patrol_success = patrol.run();

                // 找板阶段结束后再次明确停车。
                patrol.stopAndHold();

                if (!patrol_success) {
                    ROS_ERROR(
                        "V14.9 找板阶段发生真实执行故障"
                        "（导航/摄像头/停靠控制失败）；"
                        "结束本次流程"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                const bool real_target_docked =
                    patrol.realTargetDocked();

                const bool simulation_target_docked =
                    patrol.simulationTargetDocked();

                // 防御性补播只能在“现实目标确实已经停靠”时触发。
                // 若现实目标最终没有找到，绝不能伪造现实停靠播报。
                if (
                    real_target_docked
                    && !real_broadcast_done
                ) {
                    ROS_WARN(
                        "现实目标确实已停靠，但即时播报未触发；"
                        "现补播一次"
                    );
                    real_docked_callback();
                } else if (!real_target_docked) {
                    ROS_WARN(
                        "现实目标最终未完成实体停靠，"
                        "跳过‘已将现实物品放入加工车间’播报。"
                    );
                }

                if (
                    real_target_docked
                    && simulation_target_docked
                ) {
                    ROS_INFO(
                        "实体找板阶段：现实/仿真目标均已停靠，"
                        "按正常流程进入仿真任务。"
                    );
                } else if (
                    !real_target_docked
                    && simulation_target_docked
                ) {
                    ROS_WARN(
                        "实体找板阶段：仅仿真目标已停靠；"
                        "现实目标已放弃，继续进入仿真任务。"
                    );
                } else if (
                    real_target_docked
                    && !simulation_target_docked
                ) {
                    ROS_WARN(
                        "实体找板阶段：现实目标已停靠但未找到仿真目标；"
                        "保持当前位置，直接进入仿真任务。"
                    );
                } else {
                    ROS_ERROR(
                        "实体找板阶段：两个目标均未完成实体停靠；"
                        "保持当前位置，直接进入仿真任务。"
                    );
                }

                // 无论实体找板最终属于上述哪一种正常兜底结果，
                // 都发送仿真目标大类并进入仿真任务。
                if (!sendSimulationTargetAndWait(target_sim)) {
                    ROS_ERROR(
                        "仿真目标发送/等待被 ROS 关闭中断，无法最终播报"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                // 只要仿真端有任意非空返回，无论成功失败都正常播。
                if (!playSimulationFinishedBroadcast(
                        final_sim_item,
                        sim_audio_texts
                    )) {
                    ROS_ERROR("收到仿真返回，但最终播报失败");
                }

                // 仿真最终播报完成后进入红绿灯与最终巡线。
                current_state = TRAFFIC_LIGHT_CONTROL;
                break;
            }

            case TRAFFIC_LIGHT_CONTROL:
            {
                // 红绿灯观察点固定为 (2.50, 2.60)，朝向保持既定 -90°。
                ROS_WARN(
                    "进入最终红绿灯阶段：前往观察点 "
                    "(2.50, 2.60, -90°)"
                );

                if (!go_destination(
                        2.50,
                        2.60,
                        -0.5 * 3.14159265358979323846,
                        ac
                    )) {
                    ROS_ERROR(
                        "无法到达红绿灯观察点 "
                        "(2.50, 2.60, -90°)，结束比赛流程"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                ROS_WARN(
                    "已到达红绿灯观察点；"
                    "开始切换detect2026到YOLOv5三分类RKNN模型，"
                    "未确认有效方向前持续停车"
                );

                TrafficLightRecognizer traffic_light;

                if (!traffic_light.initialize()) {
                    ROS_ERROR(
                        "红绿灯RKNN初始化失败，结束比赛流程"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                // 确认方向后释放detect2026摄像头，再同步执行对应巡线服务。
                if (!traffic_light.run()) {
                    ROS_ERROR(
                        "红绿灯识别或最终巡线服务执行失败，"
                        "不播放任务完成音频"
                    );
                    current_state = ALL_FINISHED;
                    break;
                }

                ROS_WARN(
                    "最终巡线已完成，以 %.2f 倍速播放任务完成音频：%s",
                    TASK_COMPLETE_PLAYBACK_SPEED,
                    TASK_COMPLETE_AUDIO
                );

                if (!playWavAtSpeed(
                        TASK_COMPLETE_AUDIO,
                        "任务完成提示音",
                        TASK_COMPLETE_PLAYBACK_SPEED
                    )) {
                    ROS_ERROR(
                        "最终巡线已完成，但任务完成.wav的1.5倍速播放失败；"
                        "比赛流程仍正常结束"
                    );
                }

                current_state = ALL_FINISHED;
                break;
            }

            case ALL_FINISHED:
                break;

            default:
                ROS_ERROR(
                    "国赛总控进入未知状态：%d",
                    static_cast<int>(
                        current_state
                    )
                );
                current_state = ALL_FINISHED;
                break;
        }

        rate.sleep();
    }

    // ============================================================
    // 退出清理
    // ============================================================
    // 防御性清理：正常二维码成功/失败退出时已经释放 easy handle；
    // 若流程被其它异常路径提前结束，这里仍保证 libcurl 资源释放。
    cleanupQrCurlGlobal();

    stopVadProcess();
    stopQrCamera();

    ROS_INFO(
        "国赛完整流程结束："
        "语音唤醒/语义、坡道、二维码、分类、"
        "1.5倍速离线TTS、V14.9找板停靠、"
        "仿真通信、最终播报、RKNN红绿灯、巡线及任务完成音频均已结束。"
    );

    return ros::ok() ? 0 : 1;
}