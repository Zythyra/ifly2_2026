/**
 * @file qr_client_test.cpp
 * @brief 2026 智能车比赛：四观察点二维码扫描链路测试
 *        支持同帧多二维码、相机持续开启、两轮兜底与 HTTP 有限重试。
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>

#include <algorithm>
#include <clocale>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

enum State {
    NAVIGATING,
    SCANNING,
    FINISHED,
    FAILED
};
State current_state = NAVIGATING;

string target_result_1 = "未扫到1";
string target_result_2 = "未扫到2";
string target_result_3 = "未扫到3";
int valid_count = 0;

ros::ServiceClient qr_client;
int qr_waypoint_idx = 0;
int qr_scan_round = 0;
ros::Time scan_start_time;
ros::Time qr_post_success_deadline;
bool qr_found_item_at_current_waypoint = false;

vector<double> qr_wp_x = {1.0, 0.75, 0.5, 0.75};
vector<double> qr_wp_y = {5.25, 5.0, 5.25, 5.5};
vector<double> qr_wp_yaw = {3.14, 1.57, 0.0, -1.57};

double qr_scan_timeout = 1.2;
double qr_retry_scan_timeout = 2.0;
double qr_camera_warmup = 0.7;
double qr_post_success_scan_seconds = 0.15;
double qr_http_timeout = 2.0;
double qr_http_retry_delay = 0.15;
int qr_max_scan_rounds = 2;
int qr_http_retry_count = 2;

bool qr_camera_started = false;
vector<string> successful_qr_urls;
map<string, int> qr_url_failures_this_round;

// ================= 离线语音测试配置 =================
const char* const TASK_AUDIO_SCRIPT =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/"
    "scripts/generate_task_audios.py";
const char* const TASK_AUDIO_2 =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/2.wav";
const char* const TASK_AUDIO_3 =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/3.wav";
const char* const TASK_AUDIO_4 =
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/audios/4.wav";

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

bool generateQrTestAudios(
    const string& text2,
    const string& text3,
    const string& text4
) {
    const string command =
        "python3 " + shellQuote(TASK_AUDIO_SCRIPT) + " "
        + shellQuote(text2) + " "
        + shellQuote(text3) + " "
        + shellQuote(text4);

    ROS_INFO("正在生成二维码测试语音...");
    const int result = system(command.c_str());
    if (result != 0) {
        ROS_ERROR("二维码测试语音生成失败，system 返回值：%d", result);
        return false;
    }
    return true;
}

bool playAudio(const string& audio_file, const string& description) {
    ROS_INFO("播放%s：%s", description.c_str(), audio_file.c_str());
    const string command = "aplay -q " + shellQuote(audio_file);
    const int result = system(command.c_str());
    if (result != 0) {
        ROS_WARN("%s播放失败，system 返回值：%d", description.c_str(), result);
        return false;
    }
    return true;
}

// ================= 导航 =================
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

// ================= HTTP / JSON =================
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
        const long timeout_ms = static_cast<long>(
            max(0.1, qr_http_timeout) * 1000.0
        );
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);
        curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        const CURLcode result = curl_easy_perform(curl);
        if (result != CURLE_OK) {
            ROS_WARN("二维码地址请求失败：%s", curl_easy_strerror(result));
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
    return json_str.substr(start_quote + 1, end_quote - start_quote - 1);
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

bool urlAlreadyAccepted(const string& url) {
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

        if (
            qr_client.call(srv)
            && (
                srv.response.result.empty()
                || !startsWith(srv.response.result, "ERROR:")
            )
        ) {
            qr_camera_started = true;
            ROS_INFO("二维码摄像头已开启，预热 %.2f 秒", qr_camera_warmup);
            ros::Duration(max(0.0, qr_camera_warmup)).sleep();
            return true;
        }

        ROS_WARN(
            "二维码摄像头开启失败，第 %d/2 次：%s",
            attempt,
            srv.response.result.c_str()
        );
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

    ROS_WARN("清理摄像头缓存失败，尝试重新初始化：%s", srv.response.result.c_str());
    stopQrCamera();
    if (!ensureQrCameraReady()) {
        return false;
    }

    srv.request.command = -3;
    return qr_client.call(srv)
        && !startsWith(srv.response.result, "ERROR:");
}

double currentScanTimeout() {
    return qr_scan_round == 0
        ? qr_scan_timeout
        : qr_retry_scan_timeout;
}

bool acceptItem(const string& url, const string& item) {
    if (item.empty() || urlAlreadyAccepted(url) || valid_count >= 3) {
        return false;
    }

    successful_qr_urls.push_back(url);
    if (valid_count == 0) {
        target_result_1 = item;
    } else if (valid_count == 1) {
        target_result_2 = item;
    } else {
        target_result_3 = item;
    }
    valid_count++;

    ROS_INFO("录入第 %d 个有效二维码：%s", valid_count, item.c_str());
    return true;
}

bool processUrl(const string& url) {
    if (url.empty() || urlAlreadyAccepted(url)) {
        return false;
    }

    const auto failed = qr_url_failures_this_round.find(url);
    if (
        failed != qr_url_failures_this_round.end()
        && failed->second >= qr_http_retry_count
    ) {
        return false;
    }

    const int max_attempts = max(1, qr_http_retry_count);
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        ROS_INFO("请求二维码链接，第 %d/%d 次：%s", attempt, max_attempts, url.c_str());

        const string json = httpGet(url);
        const int code = extractCode(json);
        const string item = code == 200
            ? trimCopy(extractResult(json))
            : "";

        if (code == 200 && !item.empty()) {
            qr_url_failures_this_round.erase(url);
            return acceptItem(url, item);
        }

        if (code == 200) {
            ROS_WARN("二维码接口返回 code=200，但 result 为空");
        } else if (code == 400) {
            ROS_WARN("二维码接口返回 code=400，本轮进行有限重试");
        } else {
            ROS_WARN("二维码接口请求异常，code=%d", code);
        }

        if (attempt < max_attempts) {
            ros::Duration(max(0.0, qr_http_retry_delay)).sleep();
        }
    }

    qr_url_failures_this_round[url] = max_attempts;
    return false;
}

bool processQrResponse(const string& raw_result) {
    if (raw_result.empty()) {
        return false;
    }

    if (startsWith(raw_result, "ERROR:")) {
        ROS_WARN("二维码服务返回相机错误：%s", raw_result.c_str());
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

    const ros::WallTime http_start = ros::WallTime::now();
    bool found_new = false;
    for (const string& url : urls) {
        if (valid_count >= 3) {
            break;
        }
        if (processUrl(url)) {
            found_new = true;
        }
    }

    const double http_elapsed =
        (ros::WallTime::now() - http_start).toSec();
    if (http_elapsed > 0.001) {
        // HTTP 时间不计入当前观察点的视觉驻留时间。
        scan_start_time += ros::Duration(http_elapsed);
    }

    if (found_new) {
        qr_found_item_at_current_waypoint = true;
        qr_post_success_deadline =
            ros::Time::now()
            + ros::Duration(max(0.0, qr_post_success_scan_seconds));
    }
    return found_new;
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
        ROS_ERROR("二维码观察点参数长度不一致，恢复默认四观察点");
        qr_wp_x = default_x;
        qr_wp_y = default_y;
        qr_wp_yaw = default_yaw;
    }

    private_nh.param("qr_scan_timeout", qr_scan_timeout, 1.2);
    private_nh.param("qr_retry_scan_timeout", qr_retry_scan_timeout, 2.0);
    private_nh.param("qr_camera_warmup", qr_camera_warmup, 0.7);
    private_nh.param("qr_post_success_scan_seconds", qr_post_success_scan_seconds, 0.15);
    private_nh.param("qr_http_timeout", qr_http_timeout, 2.0);
    private_nh.param("qr_http_retry_delay", qr_http_retry_delay, 0.15);
    private_nh.param("qr_max_scan_rounds", qr_max_scan_rounds, 2);
    private_nh.param("qr_http_retry_count", qr_http_retry_count, 2);

    qr_max_scan_rounds = max(1, qr_max_scan_rounds);
    qr_http_retry_count = max(1, qr_http_retry_count);
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "qr_client_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    bool enable_offline_tts = true;
    bool play_offline_tts = true;
    private_nh.param("enable_offline_tts", enable_offline_tts, true);
    private_nh.param("play_offline_tts", play_offline_tts, true);
    loadQrParameters(private_nh);

    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");

    MoveBaseClient ac("move_base", true);
    ROS_INFO("等待 move_base 服务启动...");
    ac.waitForServer();
    ROS_INFO("move_base 服务已连接，开始二维码观察点扫描测试");

    ROS_INFO(
        "二维码参数：观察点=%zu，首轮=%.2fs/点，兜底=%.2fs/点，"
        "单墙成功后追加扫描=%.2fs，最大轮数=%d",
        qr_wp_x.size(),
        qr_scan_timeout,
        qr_retry_scan_timeout,
        qr_post_success_scan_seconds,
        qr_max_scan_rounds
    );

    ros::Rate rate(20);
    while (
        ros::ok()
        && current_state != FINISHED
        && current_state != FAILED
    ) {
        ros::spinOnce();

        switch (current_state) {
            case NAVIGATING:
            {
                if (valid_count >= 3) {
                    stopQrCamera();
                    current_state = FINISHED;
                    break;
                }

                if (qr_waypoint_idx >= static_cast<int>(qr_wp_x.size())) {
                    if (qr_scan_round + 1 < qr_max_scan_rounds) {
                        qr_scan_round++;
                        qr_waypoint_idx = 0;
                        qr_url_failures_this_round.clear();
                        ROS_WARN(
                            "第 %d 轮结束，仅收集 %d/3；开始第 %d 轮兜底扫描",
                            qr_scan_round,
                            valid_count,
                            qr_scan_round + 1
                        );
                        break;
                    }

                    stopQrCamera();
                    ROS_ERROR(
                        "完成 %d 轮后仍只有 %d/3 个有效二维码，测试失败",
                        qr_max_scan_rounds,
                        valid_count
                    );
                    current_state = FAILED;
                    break;
                }

                ROS_INFO(
                    "第 %d/%d 轮，前往观察点 %d/%zu：[%.2f, %.2f, %.2f]",
                    qr_scan_round + 1,
                    qr_max_scan_rounds,
                    qr_waypoint_idx + 1,
                    qr_wp_x.size(),
                    qr_wp_x[qr_waypoint_idx],
                    qr_wp_y[qr_waypoint_idx],
                    qr_wp_yaw[qr_waypoint_idx]
                );

                if (!go_destination(
                        qr_wp_x[qr_waypoint_idx],
                        qr_wp_y[qr_waypoint_idx],
                        qr_wp_yaw[qr_waypoint_idx],
                        ac
                    )) {
                    ROS_WARN("观察点 %d 到达失败，跳到下一点", qr_waypoint_idx + 1);
                    qr_waypoint_idx++;
                    break;
                }

                if (!ensureQrCameraReady()) {
                    ROS_ERROR("摄像头启动失败，本观察点跳过");
                    qr_waypoint_idx++;
                    break;
                }

                if (!flushQrCamera()) {
                    ROS_WARN("缓存清理失败，仍继续尝试实时扫码");
                }

                // 失败 URL 只在当前观察点内限次；到新观察点后允许再次尝试。
                qr_url_failures_this_round.clear();
                scan_start_time = ros::Time::now();
                qr_found_item_at_current_waypoint = false;
                qr_post_success_deadline = ros::Time(0);
                ROS_INFO(
                    "观察点 %d 开始扫码，视觉驻留 %.2f 秒，当前 %d/3",
                    qr_waypoint_idx + 1,
                    currentScanTimeout(),
                    valid_count
                );
                current_state = SCANNING;
                break;
            }

            case SCANNING:
            {
                if (valid_count >= 3) {
                    stopQrCamera();
                    current_state = FINISHED;
                    break;
                }

                const ros::Time now = ros::Time::now();
                if (
                    qr_found_item_at_current_waypoint
                    && now >= qr_post_success_deadline
                ) {
                    ROS_INFO(
                        "观察点 %d 已获得有效二维码，追加扫描 %.2f 秒结束；立即前往下一点",
                        qr_waypoint_idx + 1,
                        max(0.0, qr_post_success_scan_seconds)
                    );
                    qr_waypoint_idx++;
                    current_state = NAVIGATING;
                    break;
                }

                if (
                    (now - scan_start_time).toSec()
                    > currentScanTimeout()
                ) {
                    ROS_INFO(
                        "观察点 %d 扫描结束，当前 %d/3",
                        qr_waypoint_idx + 1,
                        valid_count
                    );
                    qr_waypoint_idx++;
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

                const bool found_new = processQrResponse(srv.response.result);
                if (valid_count >= 3) {
                    stopQrCamera();
                    current_state = FINISHED;
                    break;
                }

                if (found_new) {
                    ROS_INFO("当前点获得新二维码；再观察 %.2f 秒确认是否还有其他二维码，当前 %d/3", max(0.0, qr_post_success_scan_seconds), valid_count);
                }
                break;
            }

            case FINISHED:
            case FAILED:
                break;
        }

        rate.sleep();
    }

    stopQrCamera();

    ROS_INFO("========== 二维码扫描测试结束 ==========");
    ROS_INFO("结果1：%s", target_result_1.c_str());
    ROS_INFO("结果2：%s", target_result_2.c_str());
    ROS_INFO("结果3：%s", target_result_3.c_str());

    if (current_state != FINISHED || valid_count < 3) {
        ROS_ERROR("没有收集满 3 个有效二维码，不执行后续语音测试");
        return 2;
    }

    if (!enable_offline_tts) {
        ROS_INFO("~enable_offline_tts=false，跳过离线语音测试");
        return 0;
    }

    const string text2 = "结果一，" + target_result_1 + "。";
    const string text3 = "结果二，" + target_result_2 + "。";
    const string text4 = "结果三，" + target_result_3 + "。";

    if (!generateQrTestAudios(text2, text3, text4)) {
        ROS_ERROR("扫码结果已经保留，但离线语音链路测试失败");
        return 1;
    }

    if (!play_offline_tts) {
        ROS_INFO("~play_offline_tts=false，WAV 已生成但不播放");
        return 0;
    }

    const string full_test_broadcast = text2 + text3 + text4;
    ROS_INFO("完整测试播报内容：%s", full_test_broadcast.c_str());

    playAudio(TASK_AUDIO_2, "第 1 个二维码测试结果");
    playAudio(TASK_AUDIO_3, "第 2 个二维码测试结果");
    playAudio(TASK_AUDIO_4, "第 3 个二维码测试结果");

    ROS_INFO("二维码扫描与离线语音链路测试全部完成");
    return 0;
}
