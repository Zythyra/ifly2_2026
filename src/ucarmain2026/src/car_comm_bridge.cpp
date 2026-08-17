#include <ros/ros.h>
#include <std_msgs/String.h>

#include <errno.h>
#include <locale.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class CarCommBridge
{
public:
    CarCommBridge()
        : pnh_("~"),
          socket_fd_(-1),
          running_(true),
          next_seq_id_(1),
          task_active_(false),
          waiting_target_ack_(false),
          current_seq_(0),
          target_retry_count_(0),
          have_last_completed_seq_(false),
          last_completed_seq_(0)
    {
        pnh_.param<std::string>("server_host",
                                server_host_,
                                "gazebo-pc.local");
        pnh_.param("server_port", server_port_, 5000);
        pnh_.param("reconnect_interval",
                   reconnect_interval_,
                   1.0);
        pnh_.param("ack_timeout",
                   ack_timeout_,
                   0.5);

        if (ack_timeout_ < 0.05)
        {
            ROS_WARN("ack_timeout=%.3f 太小，自动修正为 0.05 s",
                     ack_timeout_);
            ack_timeout_ = 0.05;
        }

        if (reconnect_interval_ < 0.1)
        {
            ROS_WARN("reconnect_interval=%.3f 太小，自动修正为 0.1 s",
                     reconnect_interval_);
            reconnect_interval_ = 0.1;
        }

        detected_target_sub_ = nh_.subscribe(
            "/detected_target", 10,
            &CarCommBridge::detectedTargetCallback, this);

        task_finished_pub_ = nh_.advertise<std_msgs::String>(
            "/car_task_finished", 10);

        retry_timer_ = nh_.createWallTimer(
            ros::WallDuration(0.05),
            &CarCommBridge::retryTimerCallback,
            this);

        network_thread_ = std::thread(
            &CarCommBridge::networkLoop, this);

        ROS_INFO("小车通信节点启动：server=%s:%d，ACK超时=%.2f s",
                 server_host_.c_str(),
                 server_port_,
                 ack_timeout_);
    }

    ~CarCommBridge()
    {
        running_ = false;

        {
            std::lock_guard<std::mutex> lock(socket_mutex_);
            if (socket_fd_ >= 0)
            {
                shutdown(socket_fd_, SHUT_RDWR);
            }
        }

        if (network_thread_.joinable())
        {
            network_thread_.join();
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (socket_fd_ >= 0)
        {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }

private:
    static std::vector<std::string> split(const std::string &text, char delimiter)
    {
        std::vector<std::string> parts;
        size_t start = 0;

        while (true)
        {
            const size_t pos = text.find(delimiter, start);

            if (pos == std::string::npos)
            {
                parts.push_back(text.substr(start));
                break;
            }

            parts.push_back(text.substr(start, pos - start));
            start = pos + 1;
        }

        return parts;
    }

    static bool parseSeq(const std::string &text, uint64_t &seq)
    {
        try
        {
            size_t used = 0;
            const unsigned long long value = std::stoull(text, &used);

            if (used != text.size())
            {
                return false;
            }

            seq = static_cast<uint64_t>(value);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    void networkLoop()
    {
        while (running_ && ros::ok())
        {
            const int fd = connectToServer();

            if (fd < 0)
            {
                ROS_WARN("连接仿真端失败，%.1f 秒后重试",
                         reconnect_interval_);
                sleepReconnectInterval();
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                socket_fd_ = fd;
            }

            ROS_INFO("已连接仿真端 %s:%d",
                     server_host_.c_str(),
                     server_port_);

            resendPendingTargetAfterReconnect();

            receiveLoop(fd);

            {
                std::lock_guard<std::mutex> lock(socket_mutex_);

                if (socket_fd_ == fd)
                {
                    close(socket_fd_);
                    socket_fd_ = -1;
                }
            }

            if (running_ && ros::ok())
            {
                ROS_WARN("与仿真端连接断开，准备重新连接");
                sleepReconnectInterval();
            }
        }
    }

    int connectToServer()
    {
        struct addrinfo hints;
        struct addrinfo *result = nullptr;

        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        const std::string port_text = std::to_string(server_port_);

        const int ret = getaddrinfo(
            server_host_.c_str(),
            port_text.c_str(),
            &hints,
            &result);

        if (ret != 0)
        {
            ROS_WARN("无法解析主机名 %s：%s",
                     server_host_.c_str(),
                     gai_strerror(ret));
            return -1;
        }

        int connected_fd = -1;

        for (struct addrinfo *rp = result;
             rp != nullptr;
             rp = rp->ai_next)
        {
            const int fd = socket(
                rp->ai_family,
                rp->ai_socktype,
                rp->ai_protocol);

            if (fd < 0)
            {
                continue;
            }

            if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
            {
                connected_fd = fd;
                break;
            }

            close(fd);
        }

        freeaddrinfo(result);
        return connected_fd;
    }

    void receiveLoop(int fd)
    {
        std::string cache;
        char buffer[1024];

        while (running_ && ros::ok())
        {
            const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);

            if (n > 0)
            {
                cache.append(buffer, static_cast<size_t>(n));

                size_t pos = std::string::npos;
                while ((pos = cache.find('\n')) != std::string::npos)
                {
                    std::string line = cache.substr(0, pos);
                    cache.erase(0, pos + 1);

                    if (!line.empty() && line.back() == '\r')
                    {
                        line.pop_back();
                    }

                    handleLine(line);
                }
            }
            else if (n == 0)
            {
                break;
            }
            else
            {
                if (errno == EINTR)
                {
                    continue;
                }

                ROS_WARN("TCP 接收失败: %s", strerror(errno));
                break;
            }
        }
    }

    void handleLine(const std::string &line)
    {
        const std::vector<std::string> parts = split(line, '|');

        if (parts.empty())
        {
            return;
        }

        if (parts[0] == "ACK_TARGET")
        {
            handleTargetAck(parts);
            return;
        }

        if (parts[0] == "RESULT")
        {
            handleResult(parts);
            return;
        }

        ROS_WARN("收到未知 TCP 消息：[%s]", line.c_str());
    }

    void handleTargetAck(const std::vector<std::string> &parts)
    {
        if (parts.size() != 2)
        {
            ROS_WARN("ACK_TARGET 格式错误");
            return;
        }

        uint64_t seq = 0;
        if (!parseSeq(parts[1], seq))
        {
            ROS_WARN("ACK_TARGET seq_id 非法：[%s]", parts[1].c_str());
            return;
        }

        bool accepted = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (task_active_ &&
                waiting_target_ack_ &&
                seq == current_seq_)
            {
                waiting_target_ack_ = false;
                accepted = true;
            }
        }

        if (accepted)
        {
            ROS_INFO("收到 ACK_TARGET，seq=%llu，目标任务已被仿真端确认接收",
                     static_cast<unsigned long long>(seq));
        }
        else
        {
            ROS_WARN("收到非当前任务的 ACK_TARGET，seq=%llu，忽略",
                     static_cast<unsigned long long>(seq));
        }
    }

    void handleResult(const std::vector<std::string> &parts)
    {
        if (parts.size() != 3)
        {
            ROS_WARN("RESULT 格式错误：应为 RESULT|seq|status");
            return;
        }

        uint64_t seq = 0;
        if (!parseSeq(parts[1], seq))
        {
            ROS_WARN("RESULT seq_id 非法：[%s]", parts[1].c_str());
            return;
        }

        const std::string status = parts[2];
        if (status.empty())
        {
            ROS_WARN("RESULT 状态为空，忽略");
            return;
        }

        bool publish_result = false;
        bool ack_result = false;
        bool duplicate_result = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (task_active_ && seq == current_seq_)
            {
                publish_result = true;
                ack_result = true;

                task_active_ = false;
                waiting_target_ack_ = false;

                have_last_completed_seq_ = true;
                last_completed_seq_ = seq;
            }
            else if (have_last_completed_seq_ &&
                     seq == last_completed_seq_)
            {
                // ACK_RESULT 可能丢失，仿真端会重复发送 RESULT。
                // 此时只重新 ACK，绝不能再次发布 ROS 完成消息。
                ack_result = true;
                duplicate_result = true;
            }
        }

        if (!ack_result)
        {
            ROS_WARN("收到未知任务 RESULT，seq=%llu，当前不确认",
                     static_cast<unsigned long long>(seq));
            return;
        }

        const std::string ack =
            "ACK_RESULT|" + std::to_string(seq) + "\n";

        if (sendLine(ack))
        {
            if (duplicate_result)
            {
                ROS_INFO("收到重复 RESULT，seq=%llu，仅重新发送 ACK_RESULT",
                         static_cast<unsigned long long>(seq));
            }
            else
            {
                ROS_INFO("已发送 ACK_RESULT，seq=%llu",
                         static_cast<unsigned long long>(seq));
            }
        }

        if (publish_result)
        {
            std_msgs::String msg;
            msg.data = status;
            task_finished_pub_.publish(msg);

            ROS_INFO("TCP -> ROS：任务 seq=%llu 结果 [%s]，已发布 /car_task_finished",
                     static_cast<unsigned long long>(seq),
                     status.c_str());
        }
    }

    void detectedTargetCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data.empty())
        {
            ROS_WARN("/detected_target 内容为空，忽略");
            return;
        }

        uint64_t seq = 0;
        std::string line;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (task_active_)
            {
                ROS_WARN("当前任务 seq=%llu 尚未结束，新目标 [%s] 被忽略",
                         static_cast<unsigned long long>(current_seq_),
                         msg->data.c_str());
                return;
            }

            seq = next_seq_id_++;
            current_seq_ = seq;
            current_target_ = msg->data;
            task_active_ = true;
            waiting_target_ack_ = true;
            target_retry_count_ = 0;
            last_target_send_time_ = ros::WallTime::now();

            line = buildTargetLine(current_seq_, current_target_);
        }

        if (sendLine(line))
        {
            ROS_INFO("ROS -> TCP：发送 TARGET seq=%llu，类别 [%s]",
                     static_cast<unsigned long long>(seq),
                     msg->data.c_str());
        }
        else
        {
            ROS_WARN("TARGET seq=%llu 暂未发送成功，将由超时重发机制继续尝试",
                     static_cast<unsigned long long>(seq));
        }
    }

    void retryTimerCallback(const ros::WallTimerEvent &)
    {
        std::string line;
        uint64_t seq = 0;
        int retry_count = 0;
        bool need_retry = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (!task_active_ || !waiting_target_ack_)
            {
                return;
            }

            const double elapsed =
                (ros::WallTime::now() - last_target_send_time_).toSec();

            if (elapsed < ack_timeout_)
            {
                return;
            }

            seq = current_seq_;
            line = buildTargetLine(current_seq_, current_target_);

            ++target_retry_count_;
            retry_count = target_retry_count_;
            last_target_send_time_ = ros::WallTime::now();
            need_retry = true;
        }

        if (!need_retry)
        {
            return;
        }

        if (sendLine(line))
        {
            ROS_WARN("ACK_TARGET 超时，重发 TARGET seq=%llu，第 %d 次重试",
                     static_cast<unsigned long long>(seq),
                     retry_count);
        }
        else
        {
            ROS_WARN("TARGET seq=%llu 第 %d 次重试时连接不可用，等待恢复",
                     static_cast<unsigned long long>(seq),
                     retry_count);
        }
    }

    void resendPendingTargetAfterReconnect()
    {
        std::string line;
        uint64_t seq = 0;
        bool need_send = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (task_active_ && waiting_target_ack_)
            {
                seq = current_seq_;
                line = buildTargetLine(current_seq_, current_target_);
                last_target_send_time_ = ros::WallTime::now();
                ++target_retry_count_;
                need_send = true;
            }
        }

        if (need_send && sendLine(line))
        {
            ROS_WARN("连接恢复，立即重发待确认 TARGET seq=%llu",
                     static_cast<unsigned long long>(seq));
        }
    }

    static std::string buildTargetLine(uint64_t seq,
                                       const std::string &target)
    {
        return "TARGET|" +
               std::to_string(seq) +
               "|" +
               target +
               "\n";
    }

    bool sendLine(const std::string &data)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (socket_fd_ < 0)
        {
            return false;
        }

        size_t sent = 0;

        while (sent < data.size())
        {
            const ssize_t n = send(
                socket_fd_,
                data.data() + sent,
                data.size() - sent,
                MSG_NOSIGNAL);

            if (n <= 0)
            {
                ROS_WARN("TCP 发送失败: %s", strerror(errno));
                return false;
            }

            sent += static_cast<size_t>(n);
        }

        return true;
    }

    void sleepReconnectInterval()
    {
        int milliseconds =
            static_cast<int>(reconnect_interval_ * 1000.0);

        if (milliseconds < 100)
        {
            milliseconds = 100;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(milliseconds));
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber detected_target_sub_;
    ros::Publisher task_finished_pub_;
    ros::WallTimer retry_timer_;

    std::string server_host_;
    int server_port_;
    double reconnect_interval_;
    double ack_timeout_;

    int socket_fd_;
    std::atomic<bool> running_;
    std::thread network_thread_;
    std::mutex socket_mutex_;

    std::mutex state_mutex_;

    uint64_t next_seq_id_;

    bool task_active_;
    bool waiting_target_ack_;

    uint64_t current_seq_;
    std::string current_target_;

    ros::WallTime last_target_send_time_;
    int target_retry_count_;

    bool have_last_completed_seq_;
    uint64_t last_completed_seq_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "car_comm_bridge");

    CarCommBridge bridge;

    ros::spin();
    return 0;
}