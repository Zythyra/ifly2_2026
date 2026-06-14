/**
 * @file qr_client_test.cpp
 * @brief 盲盒抓取二维码测试节点 - 定点四向扫码版 (2秒极速+提前结束机制)
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

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

// ================= 导航 Action 辅助函数 =================
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

// ================= JSON 与 HTTP 解析辅助 =================
size_t WriteCallback(void* contents, size_t size, size_t nmemb, string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

string httpGet(string url) {
    CURL* curl;
    CURLcode res;
    string readBuffer;
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L); // 2秒网络超时
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

// ================= 主函数 =================
int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "qr_client_test_node");
    ros::NodeHandle nh;

    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");
    qr_01::qr_code srv;

    MoveBaseClient ac("move_base", true);
    ROS_INFO("⏳ 等待 move_base 服务启动...");
    ac.waitForServer();
    ROS_INFO("✅ move_base 服务已连接！开始定点扫码测试...");

    ros::Rate rate(10); 
    
    // 定点扫码的四个预设位置
    double qr_wp_x[4]   = {1.0,  0.75, 0.5,  0.75};
    double qr_wp_y[4]   = {5.25, 5.0,  5.25, 5.5};
    double qr_wp_yaw[4] = {3.14,  1.57, 0.0, -1.57};

    while (ros::ok() && currentState != FINISHED) {
        ros::spinOnce();

        switch (currentState) {
            case NAVIGATING:
            {
                if (qr_waypoint_idx >= 4) {
                    ROS_WARN("🏁 4个预设扫码点已全部遍历完毕！结束扫码阶段。");
                    currentState = FINISHED;
                    break;
                }

                ROS_INFO("==================================================");
                ROS_INFO("🚗 正在前往第 %d 个定点扫码位置: [X:%.2f, Y:%.2f, Yaw:%.2f]...", 
                         qr_waypoint_idx + 1, qr_wp_x[qr_waypoint_idx], qr_wp_y[qr_waypoint_idx], qr_wp_yaw[qr_waypoint_idx]);
                         
                if (go_destination(qr_wp_x[qr_waypoint_idx], qr_wp_y[qr_waypoint_idx], qr_wp_yaw[qr_waypoint_idx], ac)) {
                    ROS_INFO("✔️ 已到达第 %d 个位置，开启相机准备识别...", qr_waypoint_idx + 1);
                    
                    srv.request.command = -1; qr_client.call(srv); 
                    ros::Duration(1.0).sleep(); 
                    srv.request.command = -3; qr_client.call(srv); 
                    
                    scan_start_time = ros::Time::now();
                    currentState = SCANNING;
                } else {
                    ROS_WARN("⚠️ 无法到达第 %d 个位置，直接尝试下一个点...", qr_waypoint_idx + 1);
                    qr_waypoint_idx++;
                }
                break;
            }

            case SCANNING:
            {
                // 【修改 1】：驻留时间改为 2.0 秒
                if ((ros::Time::now() - scan_start_time).toSec() > 2.0) {
                    ROS_INFO("⏱️ 当前位置驻留 2 秒结束，未发现新二维码，前往下一个观察点...");
                    srv.request.command = -2; qr_client.call(srv); // 关机省资源
                    qr_waypoint_idx++;
                    currentState = NAVIGATING;
                    break;
                }

                srv.request.command = 1;
                if (qr_client.call(srv) && !srv.response.result.empty()) {
                    string raw_res = srv.response.result;
                    size_t split_pos = raw_res.find("|");
                    string captured_url = (split_pos != string::npos) ? raw_res.substr(split_pos + 1) : raw_res;
                    
                    if (find(scanned_urls.begin(), scanned_urls.end(), captured_url) == scanned_urls.end()) {
                        scanned_urls.push_back(captured_url); 
                        
                        ROS_INFO("🌐 静止捕获新二维码，发起系统请求: %s", captured_url.c_str());
                        string json = httpGet(captured_url);
                        int code = extractCode(json);
                        
                        if (code == 200) {
                            string res_text = extractResult(json);
                            if (valid_count == 0) target_result_1 = res_text;
                            else if (valid_count == 1) target_result_2 = res_text;
                            else if (valid_count == 2) target_result_3 = res_text;
                            valid_count++;
                            ROS_INFO("✅ 录入第 %d 个有效内容(200): %s", valid_count, res_text.c_str());
                            
                            // 【修改 2】：如果成功拿到 200 数据，立刻去下一个点！
                            if (valid_count >= 3) {
                                ROS_INFO("🎉 成功收集满 3 个有效二维码内容！");
                                currentState = FINISHED;
                            } else {
                                ROS_INFO("⏩ 当前点已成功获取目标，无需再等！提前前往下一个定点...");
                                srv.request.command = -2; qr_client.call(srv); // 关机省资源
                                qr_waypoint_idx++; // 累加索引前往下一个点
                                currentState = NAVIGATING;
                            }
                        } else if (code == 400) {
                            ROS_WARN("🗑️ 该物品为无效干扰项(400)，忽略...");
                        } else {
                            ROS_WARN("📶 网络异常或返回为空 (code=%d)，移出黑名单继续原位重试...", code);
                            scanned_urls.pop_back(); 
                        }
                    }
                }
                break;
            }
        }
        rate.sleep();
    }

    srv.request.command = -2; 
    qr_client.call(srv);
    
    ROS_INFO("========== 🎉 盲盒扫描阶段测试结束！==========");
    ROS_INFO("提取到的三个有效物品：");
    ROS_INFO("1: %s", target_result_1.c_str());
    ROS_INFO("2: %s", target_result_2.c_str());
    ROS_INFO("3: %s", target_result_3.c_str());

    return 0;
}