#include <ros/ros.h>
#include <qr_01/qr_code.h>
#include <curl/curl.h>
#include <string>

// --- 定义一个供 curl 使用的回调函数，用于接收网页返回的数据 ---
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// --- 封装一个发起 HTTP GET 请求的函数 ---
std::string httpGet(const std::string& url) {
    CURL* curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        
        // 极其重要：设置超时时间为 3 秒！防止赛场网络卡顿导致小车死机
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L); 
        
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        
        if(res != CURLE_OK) {
            ROS_ERROR("向打分服务器请求失败: %s", curl_easy_strerror(res));
            return "";
        }
    }
    return readBuffer;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 解决中文乱码
    ros::init(argc, argv, "qr_client_test_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<qr_01::qr_code>("qr_detect");
    
    ROS_INFO("等待服务端启动...");
    client.waitForExistence();
    ROS_INFO("连接成功！开始扫描流程...");

    qr_01::qr_code srv;

    srv.request.command = -1; // 预热
    client.call(srv);
    ros::Duration(2.0).sleep();

    srv.request.command = -3; // 清缓存
    client.call(srv);

    bool found = false;
    for (int i = 0; i < 20; i++) {
        srv.request.command = 1;
        if (client.call(srv)) {
            // 如果成功拿到字符串
            if (!srv.response.result.empty() && srv.response.result.find("ERROR") == std::string::npos) {
                
                std::string qr_url = srv.response.result;
                ROS_INFO_STREAM("🎉 成功拿到二维码网址: " << qr_url);
                
                // ================= 发起网络请求 =================
                ROS_INFO("正在向 y9000p 发起 HTTP 请求...");
                std::string json_result = httpGet(qr_url);
                
                if (!json_result.empty()) {
                    ROS_INFO_STREAM("✅ 成功获取网页 JSON 内容:\n" << json_result);
                    // 接下来你就可以把这个 json_result 交给底盘导航去判断去哪个区了
                } else {
                    ROS_WARN("拿到了网址，但请求网页 JSON 失败！请检查电脑端 Python 服务是否开启。");
                }
                // ===============================================

                found = true;
                break;
            }
        }
        ros::Duration(0.1).sleep();
    }

    if (!found) ROS_WARN("未能在画面中找到二维码。");

    srv.request.command = -2; // 释放
    client.call(srv);

    return 0;
}