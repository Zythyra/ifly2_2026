/**

1. 去年的 Aruco 只能返回 int32 的数字ID，无法处理今年赛题动态下发的网址字符串。重构为 Zbar 算法，并将 ROS Service 返回值改为 string 类型。
2. 为什么里程计不用 current_yaw - start_yaw，不用 move_base
初始底盘轻微抖动会导致 current_yaw 越过 -pi，瞬间计算出转了359度导致任务秒退。
且AMCL的粒子滤波重采样会在原地旋转时发生“数值跳变”。
使用底层 /odom 进行微小变化量绝对累加计算，无视跳变，绝对精准。
3. 不用PID 对准，改为“发现即刹车”
ROS Topic 通信有延迟，刹车后获取的 X 坐标是历史残影，导致小车“幽灵转身”。
且 Zbar 算法对镜头畸变敏感，能识别出二维码时，往往已经处于画面中间偏优区域。
4. 加 scanned_urls 黑名单查重机制？
抓取完毕后，强行休眠旋转甩开二维码会触发底盘“看门狗”，导致底层强制断联。
不休眠的话，又会把同一个二维码连刷3遍。
引入黑名单。扫过就当没看见，依靠 20Hz 的控制循环匀速转走。

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

// 全局状态机
enum State { ROTATING, FETCHING, FINISHED };
State currentState = ROTATING;

// 绝对里程计累加变量
double last_yaw = 0.0;
double total_rotated = 0.0; 
bool odom_initialized = false; 

//二维码内容存储变量
std::string target_result_1 = "";
std::string target_result_2 = "";
std::string target_result_3 = "";
int valid_count = 0; // 仅记录有效 (code=200) 的数量

// 查重黑名单，防止同一个二维码反复刷
std::vector<std::string> scanned_urls; 
ros::ServiceClient qr_client;
std::string captured_url = ""; 

//JSON 解析
int extractCode(const std::string& json_str) {
    size_t key_pos = json_str.find("\"code\"");
    if (key_pos == std::string::npos) return -1;
    
    size_t colon_pos = json_str.find(":", key_pos);
    if (colon_pos == std::string::npos) return -1;
    
    try {
        return std::stoi(json_str.substr(colon_pos + 1));
    } catch (...) {
        return -1;
    }
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


// 里程计回调：计算累加角度，防止 -pi 到 pi 跳变秒退
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // 强制等待第一帧数据，防止拿 0.0 去算初始偏差
    if (!odom_initialized) {
        last_yaw = yaw;
        odom_initialized = true;
        return;
    }
    
    //处理旋转跨越 180度时的数值突变
    double delta = yaw - last_yaw;
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;
    
    total_rotated += delta; 
    last_yaw = yaw;
}

// CURL 回调函数：将网页返回的数据流写入 std::string
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// HTTP GET 请求封装
std::string httpGet(std::string url) {
    CURL* curl; CURLcode res; std::string readBuffer;
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        //超时设置2秒。防止场馆局域网卡顿导致小车死等
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L); 
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    return readBuffer;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); // 解决终端中文乱码
    ros::init(argc, argv, "competition_final_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    qr_client = nh.serviceClient<qr_01::qr_code>("qr_detect");

    ROS_INFO("等待二维码视觉服务端启动...");
    qr_client.waitForExistence();

    // 必须堵塞等待底盘硬件上线
    while (ros::ok() && !odom_initialized) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("底盘就绪，开始执行盲盒抓取任务！");

    qr_01::qr_code srv;
    srv.request.command = -1; // -1: 打开相机预热
    qr_client.call(srv);
    ros::Duration(2.0).sleep(); 

    srv.request.command = -3; // -3: 清理缓存陈旧画面
    qr_client.call(srv); 

    // 以20Hz 频率循环
    ros::Rate rate(20); 

    while (ros::ok() && currentState != FINISHED) {
        ros::spinOnce(); // 每次循环必刷新回调
        geometry_msgs::Twist vel;
        
        // 取绝对值，防止旋转方向(左转正，右转负)干扰判断
        double rotated_deg = std::abs(total_rotated) * 180.0 / M_PI;

        switch (currentState) {
            case ROTATING:
                // 为了容错，赛题 270 度，我们设为 275 度，确保能扫完侧边
                if (rotated_deg >= 275.0) {
                    currentState = FINISHED;
                    break;
                }
                vel.angular.z = 0.4; // 0.4 rad/s，配合刹车滑行刚好居中
                
                srv.request.command = 1;
                if (qr_client.call(srv) && !srv.response.result.empty()) {
                    
                    // 剥离可能遗留的 X 坐标前缀 (兼容 "320|http://..." 格式)
                    std::string raw_res = srv.response.result;
                    size_t split_pos = raw_res.find("|");
                    std::string temp_url = (split_pos != std::string::npos) ? raw_res.substr(split_pos + 1) : raw_res;
                    
                    // 黑名单查重
                    bool is_duplicate = false;
                    for (size_t i = 0; i < scanned_urls.size(); i++) {
                        if (scanned_urls[i] == temp_url) {
                            is_duplicate = true;
                            break;
                        }
                    }

                    if (is_duplicate) {
                        // 使用 break
                        // 若使用 continue 会跳过下面的 publish 和 sleep，导致底层收不到指令停转并占满 CPU，然后导致小车卡死
                        break; 
                    } else {
                        // 这是一个全新的目标
                        ROS_INFO("发现新目标，立即刹车锁定");
                        vel.angular.z = 0.0;
                        cmd_pub.publish(vel); // 立刻停车
                        ros::Duration(0.5).sleep(); // 等待物理减震器平稳
                        
                        captured_url = temp_url;
                        scanned_urls.push_back(captured_url); // 马上存入黑名单
                        currentState = FETCHING;
                    }
                }
                break;

            case FETCHING:
                ROS_INFO_STREAM("向系统请求: " << captured_url);
                std::string json = httpGet(captured_url);
                
                // 解析 JSON 并校验 code确定有效
                int code = extractCode(json);
                if (code == 200) {
                    std::string res_text = extractResult(json);
                    
                    // 依次存入独立变量
                    if (valid_count == 0) target_result_1 = res_text;
                    else if (valid_count == 1) target_result_2 = res_text;
                    else if (valid_count == 2) target_result_3 = res_text;
                    
                    valid_count++;
                    ROS_INFO("有效数据(200)，存入目标 %d: %s", valid_count, res_text.c_str());
                } else {
                    // code == 400 或者请求失败
                    ROS_WARN("无效数据(code=%d)，已跳过该内容，继续寻找。", code);
                }
                
                // 满 3 个收工
                if (valid_count >= 3) {
                    currentState = FINISHED;
                } else {
                    currentState = ROTATING;
                    ROS_INFO("继续逆时针平滑搜索...");

                }
                break;
        }

        // 只要在旋转状态，每秒下发 20 次指令
        if (currentState == ROTATING) {
            cmd_pub.publish(vel);
        }
        rate.sleep();
    }

    srv.request.command = -2; // 安全关闭相机，释放算力
    qr_client.call(srv);
    
    ROS_INFO("========== 🎉 比赛流程彻底结束！==========");
    ROS_INFO("提取到的三个有效目标变量如下：");
    ROS_INFO("变量 target_result_1 = %s", target_result_1.c_str());
    ROS_INFO("变量 target_result_2 = %s", target_result_2.c_str());
    ROS_INFO("变量 target_result_3 = %s", target_result_3.c_str());

    return 0;
}