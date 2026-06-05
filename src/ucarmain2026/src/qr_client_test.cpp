/**
 * @file qr_client_test.cpp
 * @brief 盲盒抓取二维码客户端 - 包含视觉伺服对准与 HTTP 死磕机制
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// ================= 全局状态机 =================
// ROTATING: 寻目标旋转 | ALIGNING: PID居中对准 | FETCHING: 网络请求 | FINISHED: 任务完成
enum State { ROTATING, ALIGNING, FETCHING, FINISHED };
State currentState = ROTATING;

// 绝对里程计累加变量
double last_yaw = 0.0;
double total_rotated = 0.0; 
bool odom_initialized = false; 

// 二维码内容存储变量
std::string target_result_1 = "";
std::string target_result_2 = "";
std::string target_result_3 = "";
int valid_count = 0; // 仅记录有效 (code=200) 的数量

// 查重黑名单，扫过且处理完毕的 URL 才会进黑名单
std::vector<std::string> scanned_urls; 
ros::ServiceClient qr_client;
std::string captured_url = ""; 
int lost_count = 0; // 对准时防丢计数器

// ================= JSON 与 HTTP 解析 =================
int extractCode(const std::string& json_str) {
    size_t key_pos = json_str.find("\"code\"");
    if (key_pos == std::string::npos) return -1;
    size_t colon_pos = json_str.find(":", key_pos);
    if (colon_pos == std::string::npos) return -1;
    try { return std::stoi(json_str.substr(colon_pos + 1)); } catch (...) { return -1; }
}

std::string extractResult(const std::string& json_str) {
    size_t key_pos = json_str.find("\"result\"");
    if (key_pos == std::string::npos) return "";
    size_t start_quote = json_str.find("\"", key_pos + 8); 
    if (start_quote == std::string::npos) return "";
    size_t end_quote = json_str.find("\"", start_quote + 1);
    if (end_quote == std::string::npos) return "";
    return json_str.substr(start_quote + 1, end_quote - start_quote - 1);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    if (!odom_initialized) { last_yaw = yaw; odom_initialized = true; return; }
    
    double delta = yaw - last_yaw;
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;
    
    total_rotated += delta; 
    last_yaw = yaw;
}

size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

std::string httpGet(std::string url) {
    CURL* curl; CURLcode res; std::string readBuffer;
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L); 
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    return readBuffer;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    ros::init(argc, argv, "competition_final_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");

    ROS_INFO("等待二维码视觉服务端启动...");
    qr_client.waitForExistence();

    while (ros::ok() && !odom_initialized) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("底盘就绪，开始执行盲盒抓取任务！");

    qr_01::qr_code srv;
    srv.request.command = -1; // 开机预热
    qr_client.call(srv);
    ros::Duration(2.0).sleep(); 
    srv.request.command = -3; // 清理缓存
    qr_client.call(srv); 

    ros::Rate rate(20); 

    while (ros::ok() && currentState != FINISHED) {
        ros::spinOnce(); 
        geometry_msgs::Twist vel;
        double rotated_deg = std::abs(total_rotated) * 180.0 / M_PI;

        switch (currentState) {
            // ================= 阶段 1：巡航寻找 =================
            case ROTATING:
                if (rotated_deg >= 360.0) { currentState = FINISHED; break; }
                vel.angular.z = 0.4; 
                
                srv.request.command = 1;
                if (qr_client.call(srv) && !srv.response.result.empty()) {
                    std::string raw_res = srv.response.result;
                    size_t split_pos = raw_res.find("|");
                    std::string temp_url = (split_pos != std::string::npos) ? raw_res.substr(split_pos + 1) : raw_res;
                    
                    if (std::find(scanned_urls.begin(), scanned_urls.end(), temp_url) != scanned_urls.end()) {
                        break; // 已经在黑名单，直接无视
                    }

                    ROS_INFO("👀 发现新目标，进入视觉伺服对准模式...");
                    captured_url = temp_url;
                    lost_count = 0; 
                    currentState = ALIGNING; // 切换到对准模式
                }
                break;

            // ================= 阶段 2：PID 视觉居中对准 =================
            case ALIGNING:
                srv.request.command = 1;
                if (qr_client.call(srv) && !srv.response.result.empty()) {
                    lost_count = 0; 
                    std::string raw_res = srv.response.result;
                    size_t split_pos = raw_res.find("|");
                    
                    int x_coord = 320; // 默认画面中央
                    if (split_pos != std::string::npos) {
                        try { x_coord = std::stoi(raw_res.substr(0, split_pos)); } 
                        catch (...) { x_coord = 320; }
                        // 更新一下 URL，防止对准过程中扫到了旁边的码
                        captured_url = raw_res.substr(split_pos + 1); 
                    }

                    // 计算像素误差 (中心坐标是320)
                    int error = 320 - x_coord;
                    
                    // 误差小于 15 像素，视为对准完毕
                    if (std::abs(error) < 15) {
                        ROS_INFO("🎯 二维码已完美居中！刹车准备获取数据。");
                        vel.angular.z = 0.0;
                        cmd_pub.publish(vel);
                        ros::Duration(0.5).sleep(); // 停稳
                        currentState = FETCHING;
                    } else {
                        // P 控制器动态调节旋转速度
                        double Kp = 0.0025; 
                        vel.angular.z = Kp * error;
                        
                        // 速度限幅，防止转太快画面糊了，也防止过冲
                        if (vel.angular.z > 0) vel.angular.z = std::max(std::min(vel.angular.z, 0.4), 0.05);
                        else vel.angular.z = std::min(std::max(vel.angular.z, -0.4), -0.05);
                    }
                } else {
                    lost_count++;
                    if (lost_count > 10) { // 连续0.5秒丢失目标
                        ROS_WARN("⚠️ 目标在对准时丢失！恢复巡航搜索...");
                        currentState = ROTATING;
                    }
                    vel.angular.z = 0.0; // 丢失期间先原地别动
                }
                break;

            // ================= 阶段 3：HTTP 数据抓取 (死磕模式) =================
            case FETCHING:
            {
                ROS_INFO("🌐 向系统请求数据: %s", captured_url.c_str());
                std::string json = httpGet(captured_url);
                int code = extractCode(json);
                
                // 【核心逻辑】：只有拿到明确的 200 或 400，才视为处理完毕，否则一直死磕
                if (code == 200 || code == 400) {
                    scanned_urls.push_back(captured_url); // 封锁这个 URL，不再理它
                    
                    if (code == 200) {
                        std::string res_text = extractResult(json);
                        if (valid_count == 0) target_result_1 = res_text;
                        else if (valid_count == 1) target_result_2 = res_text;
                        else if (valid_count == 2) target_result_3 = res_text;
                        valid_count++;
                        ROS_INFO("✅ 获得有效物品(200): %s", res_text.c_str());
                    } else {
                        ROS_WARN("🗑️ 获得无效物品(400)，抛弃继续寻找。");
                    }
                    
                    if (valid_count >= 3) {
                        currentState = FINISHED;
                    } else {
                        ROS_INFO("🔄 寻找下一个目标...");
                        currentState = ROTATING;
                    }
                } else {
                    // 没有拿到 200/400 (网络超时、报错等)，状态不切换！
                    // 会在下一个 rate.sleep() 后，继续执行 FETCHING 发起新的请求
                    ROS_WARN("📶 网络卡顿或返回异常 (code=%d)，正在原位重试请求...", code);
                }
                break;
            }
        }

        // 仅在寻找和对准时，发布速度指令；FETCHING 时小车必须保持静止
        if (currentState == ROTATING || currentState == ALIGNING) {
            cmd_pub.publish(vel);
        }
        rate.sleep();
    }

    srv.request.command = -2; 
    qr_client.call(srv);
    
    ROS_INFO("========== 🎉 盲盒扫描阶段闭环完成！==========");
    ROS_INFO("提取到的三个有效目标变量如下：");
    ROS_INFO("目标一: %s", target_result_1.c_str());
    ROS_INFO("目标二: %s", target_result_2.c_str());
    ROS_INFO("目标三: %s", target_result_3.c_str());

    return 0;
}