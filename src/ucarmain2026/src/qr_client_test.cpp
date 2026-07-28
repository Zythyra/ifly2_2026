/**
 * @file qr_client_test.cpp
 * @brief 盲盒抓取二维码测试节点
 *        （定点四向扫码 + Ekho 离线语音链路测试）
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>

#include <algorithm>
#include <clocale>
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

// ================= 全局状态机 =================
enum State { NAVIGATING, SCANNING, FINISHED };
State currentState = NAVIGATING;

// 二维码内容存储变量
string target_result_1 = "未扫到1";
string target_result_2 = "未扫到2";
string target_result_3 = "未扫到3";
int valid_count = 0;

// 查重黑名单与扫码控制
vector<string> scanned_urls;
ros::ServiceClient qr_client;
int qr_waypoint_idx = 0;
ros::Time scan_start_time;

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

/**
 * @brief 把任意 UTF-8 文本安全地作为一个 shell 参数传给 system()。
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

    ROS_INFO("正在使用 Ekho 离线生成二维码测试语音...");
    const int result = system(command.c_str());
    if (result != 0) {
        ROS_ERROR(
            "二维码测试语音生成失败，system 返回值：%d",
            result
        );
        return false;
    }

    ROS_INFO("二维码测试语音生成成功");
    return true;
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

// ================= 导航 Action 辅助函数 =================
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

// ================= JSON 与 HTTP 解析辅助 =================
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

// ================= 主函数 =================
int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "qr_client_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    bool enable_offline_tts = true;
    bool play_offline_tts = true;
    private_nh.param(
        "enable_offline_tts",
        enable_offline_tts,
        true
    );
    private_nh.param(
        "play_offline_tts",
        play_offline_tts,
        true
    );

    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");
    qr_01::qr_code srv;

    MoveBaseClient ac("move_base", true);
    ROS_INFO("等待 move_base 服务启动...");
    ac.waitForServer();
    ROS_INFO("move_base 服务已连接！开始定点扫码测试...");

    ros::Rate rate(10);

    // 定点扫码的四个预设位置
    const double qr_wp_x[4] =
        {1.0, 0.75, 0.5, 0.75};
    const double qr_wp_y[4] =
        {5.25, 5.0, 5.25, 5.5};
    const double qr_wp_yaw[4] =
        {3.14, 1.57, 0.0, -1.57};

    while (ros::ok() && currentState != FINISHED) {
        ros::spinOnce();

        switch (currentState) {
            case NAVIGATING:
            {
                if (qr_waypoint_idx >= 4) {
                    ROS_WARN(
                        "4 个预设扫码点已全部遍历完毕！结束扫码阶段。"
                    );
                    currentState = FINISHED;
                    break;
                }

                ROS_INFO(
                    "正在前往第 %d 个定点扫码位置："
                    "[X:%.2f, Y:%.2f, Yaw:%.2f]...",
                    qr_waypoint_idx + 1,
                    qr_wp_x[qr_waypoint_idx],
                    qr_wp_y[qr_waypoint_idx],
                    qr_wp_yaw[qr_waypoint_idx]
                );

                if (
                    go_destination(
                        qr_wp_x[qr_waypoint_idx],
                        qr_wp_y[qr_waypoint_idx],
                        qr_wp_yaw[qr_waypoint_idx],
                        ac
                    )
                ) {
                    ROS_INFO(
                        "已到达第 %d 个位置，开启相机准备识别...",
                        qr_waypoint_idx + 1
                    );

                    srv.request.command = -1;
                    qr_client.call(srv);
                    ros::Duration(1.0).sleep();
                    srv.request.command = -3;
                    qr_client.call(srv);

                    scan_start_time = ros::Time::now();
                    currentState = SCANNING;
                } else {
                    ROS_WARN(
                        "无法到达第 %d 个位置，直接尝试下一个点...",
                        qr_waypoint_idx + 1
                    );
                    qr_waypoint_idx++;
                }
                break;
            }

            case SCANNING:
            {
                if (
                    (ros::Time::now() - scan_start_time).toSec()
                    > 2.0
                ) {
                    ROS_INFO(
                        "当前位置驻留 2 秒结束，未发现新二维码，"
                        "前往下一个观察点..."
                    );
                    srv.request.command = -2;
                    qr_client.call(srv);
                    qr_waypoint_idx++;
                    currentState = NAVIGATING;
                    break;
                }

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

                        ROS_INFO(
                            "静止捕获新二维码，发起系统请求：%s",
                            captured_url.c_str()
                        );
                        const string json = httpGet(captured_url);
                        const int code = extractCode(json);

                        if (code == 200) {
                            const string res_text =
                                extractResult(json);
                            if (valid_count == 0) {
                                target_result_1 = res_text;
                            } else if (valid_count == 1) {
                                target_result_2 = res_text;
                            } else if (valid_count == 2) {
                                target_result_3 = res_text;
                            }
                            valid_count++;

                            ROS_INFO(
                                "录入第 %d 个有效内容：%s",
                                valid_count,
                                res_text.c_str()
                            );

                            if (valid_count >= 3) {
                                ROS_INFO(
                                    "成功收集满 3 个有效二维码内容！"
                                );
                                currentState = FINISHED;
                            } else {
                                ROS_INFO(
                                    "当前点已获取目标，提前前往下一个定点..."
                                );
                                srv.request.command = -2;
                                qr_client.call(srv);
                                qr_waypoint_idx++;
                                currentState = NAVIGATING;
                            }
                        } else if (code == 400) {
                            ROS_WARN("该物品为无效干扰项，忽略...");
                        } else {
                            ROS_WARN(
                                "网络异常或返回为空（code=%d），"
                                "移出黑名单继续原位重试...",
                                code
                            );
                            scanned_urls.pop_back();
                        }
                    }
                }
                break;
            }

            case FINISHED:
                break;
        }

        rate.sleep();
    }

    srv.request.command = -2;
    qr_client.call(srv);

    ROS_INFO("========== 盲盒扫描阶段测试结束 ==========");
    ROS_INFO("提取到的三个有效物品：");
    ROS_INFO("1：%s", target_result_1.c_str());
    ROS_INFO("2：%s", target_result_2.c_str());
    ROS_INFO("3：%s", target_result_3.c_str());

    if (!enable_offline_tts) {
        ROS_INFO(
            "参数 ~enable_offline_tts=false，跳过离线语音测试"
        );
        return 0;
    }

    const string text2 =
        "结果一，" + target_result_1 + "。";
    const string text3 =
        "结果二，" + target_result_2 + "。";
    const string text4 =
        "结果三，" + target_result_3 + "。";

    if (!generateQrTestAudios(text2, text3, text4)) {
        ROS_ERROR(
            "扫码结果已经保留，但离线语音链路测试失败"
        );
        return 1;
    }

    if (!play_offline_tts) {
        ROS_INFO(
            "参数 ~play_offline_tts=false，WAV 已生成但不播放"
        );
        return 0;
    }

    playAudio(TASK_AUDIO_2, "第 1 个二维码测试结果");
    playAudio(TASK_AUDIO_3, "第 2 个二维码测试结果");
    playAudio(TASK_AUDIO_4, "第 3 个二维码测试结果");

    ROS_INFO("二维码扫描与 Ekho 离线语音链路测试全部完成");
    return 0;
}