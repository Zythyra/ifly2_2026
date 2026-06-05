/**
 * @file pure_detect_test.cpp
 * @brief 纯净版目标检测测试节点 - 已修复摄像头未初始化 (NoneType) 的 Bug
 */

#include <ros/ros.h>
#include <ros_nanodet/detect_result_srv.h> 
#include <vector>
#include <string>

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); 
    
    ros::init(argc, argv, "pure_detect_test_node");
    ros::NodeHandle nh;

    ros::ServiceClient detect_client = nh.serviceClient<ros_nanodet::detect_result_srv>("nanodet_detect");

    ROS_INFO("========== 纯净版目标检测测试启动 ==========");
    ROS_INFO("正在等待 nanodet_detect 视觉服务端启动...");
    
    detect_client.waitForExistence();
    ROS_INFO("✔️ 服务端已连接！");

    // =========================================================
    // 【新增核心修复】：先发送 -1 指令，唤醒并打开底层物理摄像头
    // =========================================================
    ros_nanodet::detect_result_srv srv_open;
    srv_open.request.detect_start = -1;
    if (detect_client.call(srv_open)) {
        ROS_INFO("📸 已发送打开摄像头指令 (-1)，等待摄像头预热...");
    } else {
        ROS_ERROR("❌ 发送打开摄像头指令失败！");
    }
    // 给摄像头两秒钟的物理启动和曝光预热时间
    ros::Duration(2.0).sleep(); 

    // 发送一次 -3 指令清理刚开机时的历史缓存脏帧
    ros_nanodet::detect_result_srv srv_clear;
    srv_clear.request.detect_start = -3;
    detect_client.call(srv_clear);
    ros::Duration(0.5).sleep();

    // 严格对应 YAML 的类别
    std::vector<std::string> class_names = {
        "daily_workshop", "food_workshop", "electronics_workshop"
    };

    ros::Rate rate(10); 
    ROS_INFO("🚀 开始持续请求检测画面...");

    while (ros::ok()) {
        ros_nanodet::detect_result_srv srv;
        srv.request.detect_start = 1; 

        if (detect_client.call(srv)) {
            int obj_count = srv.response.class_name.size(); 
            
            if (obj_count > 0) {
                for (int i = 0; i < obj_count; i++) {
                    int id = srv.response.class_name[i];
                    std::string name = (id >= 0 && id < class_names.size()) ? class_names[id] : "未知类别";
                    
                    ROS_INFO("👀 发现目标: [%s] | 类别ID: %d | 框坐标: 左上(%d, %d) -> 右下(%d, %d)", 
                             name.c_str(), 
                             id, 
                             srv.response.x0[i], 
                             srv.response.y0[i],
                             srv.response.x1[i],
                             srv.response.y1[i]);
                }
            } else {
                ROS_INFO_THROTTLE(2.0, "视野中空旷，未发现这三类目标...");
            }

        } else {
            ROS_WARN_THROTTLE(2.0, "⚠️ 调用检测服务失败，请检查 Python 服务端是否报错...");
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 退出时安全关闭摄像头
    ros_nanodet::detect_result_srv srv_close;
    srv_close.request.detect_start = -2;
    detect_client.call(srv_close);

    ROS_INFO("========== 测试结束 ==========");
    return 0;
}