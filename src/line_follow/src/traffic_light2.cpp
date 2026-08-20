/*
 * 讯飞2026 YOLOv5红绿灯方向识别（避障路线）
 *
 * 模型类别：
 *   0 -> 左转
 *   1 -> 右转
 *   2 -> 直行
 *
 * 工作流程：
 *   1. 调用 /nanodet_detect，command=-10，切换detect2026到红绿灯模型
 *   2. command=-1 打开 /dev/video0
 *   3. command=-3 清理缓存
 *   4. 循环 command=10 做YOLOv5三分类RKNN检测
 *   5. 连续 stable_frames 帧方向一致后确认
 *   6. 释放摄像头
 *   7. 调用对应巡线服务：
 *        class 0 LEFT     -> /line2o_left
 *        class 1 RIGHT    -> /line2o_right
 *        class 2 STRAIGHT -> /lineo_right
 *
 * 注意：
 *   - 红灯不作为单独类别。没有检测到有效的0/1/2时始终停车。
 *   - C++不再直接打开摄像头，摄像头统一由detect2026.py管理。
 *   - detect2026交通模型返回结果已按置信度从高到低排序，
 *     因此若同一帧存在多个候选，本节点优先使用第一个有效检测。
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include "line_follow/line_follow.h"
#include "ros_nanodet/detect_result_srv.h"

#include <algorithm>
#include <locale.h>
#include <string>

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
          pnh_("~"),
          route_started_(false),
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
            "YOLOv5红绿灯节点启动："
            "模型映射 0=左转，1=右转，2=直行；"
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

    void run()
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
        // YOLOv5模型固定映射：
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
            "请求detect2026切换到红绿灯三分类RKNN模型...");

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
            "红绿灯三分类RKNN模型已就绪");

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

            publishStop();
            ros::shutdown();
            return;
        }

        ROS_INFO(
            "巡线服务%s执行结束",
            service_name);

        ros::shutdown();
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
    bool camera_opened_;
    bool traffic_mode_ready_;

    int stable_count_;
    Direction last_direction_;
};

int main(
    int argc,
    char **argv)
{
    setlocale(
        LC_ALL,
        "");

    ros::init(
        argc,
        argv,
        "traffic_light");

    TrafficLightRecognizer node;

    if (!node.initialize())
    {
        ROS_FATAL(
            "红绿灯RKNN节点初始化失败");
        return 1;
    }

    node.run();

    return 0;
}