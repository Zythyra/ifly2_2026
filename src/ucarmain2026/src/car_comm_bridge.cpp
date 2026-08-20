/*
 * 讯飞2026 仿真-实车通信 Bridge V3
 * 小车端 TCP Client
 *
 * V3 自愈增强：
 * 1. 保留原 TARGET / ACK_TARGET / RESULT / ACK_RESULT + seq_id 幂等协议。
 * 2. 应用层 PING/PONG 心跳，默认 1s 一次，3s 无任何接收数据即主动断链。
 * 3. ACK_TARGET 连续 3 次重试后仍无 ACK，不再无限重发到“僵尸 socket”，
 *    主动 shutdown 当前连接，重新建立 TCP。
 * 4. send() 一旦明确失败，立即 shutdown + close 当前 socket，唤醒 recv()。
 * 5. Linux TCP SO_KEEPALIVE + TCP_KEEPIDLE/TCP_KEEPINTVL/TCP_KEEPCNT
 *    + TCP_USER_TIMEOUT 作为应用层心跳之外的第二层保险。
 * 6. 重连地址顺序：
 *      上次成功连接的数字 IP（最快，且不依赖 mDNS）
 *      -> server_host（默认 gazebo-pc.local）
 *      -> server_fallback_host（可配置固定 IP）
 *    因此即使比赛中 .local 临时失效，只要之前成功连接过一次，
 *    仍可直接使用缓存 IP 自动恢复。
 * 7. connect 使用非阻塞超时，避免失效 IP 长时间卡住重连线程。
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <errno.h>
#include <fcntl.h>
#include <locale.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <set>
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
          last_completed_seq_(0),
          last_rx_ms_(0),
          last_ping_send_ms_(0),
          heartbeat_seq_(1)
    {
        pnh_.param<std::string>("server_host",
                                server_host_,
                                "gazebo-pc.local");

        // 可选固定 IP 兜底。
        // 不知道比赛现场 gazebo-pc 的最终固定 IP，因此默认留空；
        // 但只要首次通过 .local 成功连接，本节点会自动缓存数字 IP，
        // 后续重连优先使用缓存 IP，不依赖 mDNS。
        pnh_.param<std::string>("server_fallback_host",
                                server_fallback_host_,
                                "");

        pnh_.param("server_port",
                   server_port_,
                   5000);

        pnh_.param("reconnect_interval",
                   reconnect_interval_,
                   1.0);

        pnh_.param("connect_timeout",
                   connect_timeout_,
                   1.5);

        pnh_.param("ack_timeout",
                   ack_timeout_,
                   0.5);

        pnh_.param("max_ack_retries_before_reconnect",
                   max_ack_retries_before_reconnect_,
                   3);

        pnh_.param("heartbeat_interval",
                   heartbeat_interval_,
                   1.0);

        pnh_.param("heartbeat_timeout",
                   heartbeat_timeout_,
                   3.0);

        pnh_.param("tcp_keepidle",
                   tcp_keepidle_,
                   2);

        pnh_.param("tcp_keepintvl",
                   tcp_keepintvl_,
                   1);

        pnh_.param("tcp_keepcnt",
                   tcp_keepcnt_,
                   3);

        pnh_.param("tcp_user_timeout_ms",
                   tcp_user_timeout_ms_,
                   4000);

        sanitizeParameters();

        detected_target_sub_ = nh_.subscribe(
            "/detected_target", 10,
            &CarCommBridge::detectedTargetCallback, this);

        task_finished_pub_ = nh_.advertise<std_msgs::String>(
            "/car_task_finished", 10);

        retry_timer_ = nh_.createWallTimer(
            ros::WallDuration(0.05),
            &CarCommBridge::retryTimerCallback,
            this);

        health_timer_ = nh_.createWallTimer(
            ros::WallDuration(0.20),
            &CarCommBridge::healthTimerCallback,
            this);

        network_thread_ = std::thread(
            &CarCommBridge::networkLoop, this);

        ROS_INFO(
            "小车通信V3启动：server=%s:%d，fallback=[%s]，"
            "ACK=%.2fs×%d，heartbeat=%.1fs/%.1fs，"
            "TCP_USER_TIMEOUT=%dms",
            server_host_.c_str(),
            server_port_,
            server_fallback_host_.empty()
                ? "未配置"
                : server_fallback_host_.c_str(),
            ack_timeout_,
            max_ack_retries_before_reconnect_,
            heartbeat_interval_,
            heartbeat_timeout_,
            tcp_user_timeout_ms_);
    }

    ~CarCommBridge()
    {
        running_ = false;
        forceDisconnect("节点退出", false);

        if (network_thread_.joinable())
        {
            network_thread_.join();
        }
    }

private:
    static int64_t steadyNowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void sanitizeParameters()
    {
        if (ack_timeout_ < 0.05)
        {
            ROS_WARN("ack_timeout=%.3f 太小，自动修正为 0.05s",
                     ack_timeout_);
            ack_timeout_ = 0.05;
        }

        if (reconnect_interval_ < 0.1)
        {
            ROS_WARN("reconnect_interval=%.3f 太小，自动修正为 0.1s",
                     reconnect_interval_);
            reconnect_interval_ = 0.1;
        }

        if (connect_timeout_ < 0.2)
        {
            ROS_WARN("connect_timeout=%.3f 太小，自动修正为 0.2s",
                     connect_timeout_);
            connect_timeout_ = 0.2;
        }

        if (max_ack_retries_before_reconnect_ < 1)
        {
            ROS_WARN("max_ack_retries_before_reconnect=%d 非法，自动修正为1",
                     max_ack_retries_before_reconnect_);
            max_ack_retries_before_reconnect_ = 1;
        }

        if (heartbeat_interval_ < 0.2)
        {
            heartbeat_interval_ = 0.2;
        }

        if (heartbeat_timeout_ < heartbeat_interval_ * 2.0)
        {
            ROS_WARN(
                "heartbeat_timeout=%.2f 太小，自动修正为 %.2fs",
                heartbeat_timeout_,
                heartbeat_interval_ * 2.0);
            heartbeat_timeout_ = heartbeat_interval_ * 2.0;
        }

        tcp_keepidle_ = std::max(1, tcp_keepidle_);
        tcp_keepintvl_ = std::max(1, tcp_keepintvl_);
        tcp_keepcnt_ = std::max(1, tcp_keepcnt_);
        tcp_user_timeout_ms_ = std::max(1000, tcp_user_timeout_ms_);
    }

    static std::vector<std::string> split(const std::string &text,
                                          char delimiter)
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
            const unsigned long long value =
                std::stoull(text, &used);

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
            std::string connected_ip;
            std::string connected_via;

            const int fd = connectToServer(
                connected_ip,
                connected_via);

            if (fd < 0)
            {
                ROS_WARN(
                    "连接仿真端失败，%.1fs后重试",
                    reconnect_interval_);
                sleepReconnectInterval();
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(socket_mutex_);
                socket_fd_ = fd;
            }

            last_successful_ip_ = connected_ip;

            const int64_t now_ms = steadyNowMs();
            last_rx_ms_.store(now_ms);
            last_ping_send_ms_.store(0);

            ROS_INFO(
                "已连接仿真端：via=[%s]，peer_ip=%s，port=%d",
                connected_via.c_str(),
                connected_ip.c_str(),
                server_port_);

            resendPendingTargetAfterReconnect();

            receiveLoop(fd);

            closeSocketIfCurrent(fd);

            if (running_ && ros::ok())
            {
                ROS_WARN(
                    "与仿真端连接断开，准备重新连接；"
                    "上次成功IP=[%s]",
                    last_successful_ip_.empty()
                        ? "无"
                        : last_successful_ip_.c_str());

                sleepReconnectInterval();
            }
        }
    }

    int connectToServer(std::string &connected_ip,
                        std::string &connected_via)
    {
        std::vector<std::string> candidates;
        std::set<std::string> seen;

        auto appendCandidate =
            [&](const std::string &host)
        {
            if (!host.empty() &&
                seen.insert(host).second)
            {
                candidates.push_back(host);
            }
        };

        // 重连优先上次实际成功的数字IP，直接绕过mDNS。
        appendCandidate(last_successful_ip_);
        appendCandidate(server_host_);
        appendCandidate(server_fallback_host_);

        for (const std::string &host : candidates)
        {
            std::string peer_ip;
            const int fd =
                connectToHost(host, peer_ip);

            if (fd >= 0)
            {
                connected_ip = peer_ip;
                connected_via = host;
                return fd;
            }
        }

        return -1;
    }

    int connectToHost(const std::string &host,
                      std::string &peer_ip)
    {
        struct addrinfo hints;
        struct addrinfo *result = nullptr;

        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        const std::string port_text =
            std::to_string(server_port_);

        const int ret = getaddrinfo(
            host.c_str(),
            port_text.c_str(),
            &hints,
            &result);

        if (ret != 0)
        {
            ROS_WARN(
                "无法解析/使用地址 [%s]：%s",
                host.c_str(),
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

            if (!connectWithTimeout(
                    fd,
                    rp->ai_addr,
                    rp->ai_addrlen,
                    connect_timeout_))
            {
                close(fd);
                continue;
            }

            configureConnectedSocket(fd);

            char host_buffer[NI_MAXHOST] = {0};

            const int name_ret = getnameinfo(
                rp->ai_addr,
                rp->ai_addrlen,
                host_buffer,
                sizeof(host_buffer),
                nullptr,
                0,
                NI_NUMERICHOST);

            if (name_ret == 0)
            {
                peer_ip = host_buffer;
            }
            else
            {
                peer_ip = host;
            }

            connected_fd = fd;
            break;
        }

        freeaddrinfo(result);

        if (connected_fd < 0)
        {
            ROS_WARN(
                "连接地址 [%s]:%d 失败",
                host.c_str(),
                server_port_);
        }

        return connected_fd;
    }

    static bool connectWithTimeout(
        int fd,
        const struct sockaddr *addr,
        socklen_t addrlen,
        double timeout_sec)
    {
        const int old_flags =
            fcntl(fd, F_GETFL, 0);

        if (old_flags < 0)
        {
            return false;
        }

        if (fcntl(fd,
                  F_SETFL,
                  old_flags | O_NONBLOCK) < 0)
        {
            return false;
        }

        int ret = connect(fd, addr, addrlen);

        if (ret == 0)
        {
            fcntl(fd, F_SETFL, old_flags);
            return true;
        }

        if (errno != EINPROGRESS)
        {
            fcntl(fd, F_SETFL, old_flags);
            return false;
        }

        struct pollfd pfd;
        std::memset(&pfd, 0, sizeof(pfd));
        pfd.fd = fd;
        pfd.events = POLLOUT;

        const int timeout_ms =
            std::max(
                1,
                static_cast<int>(
                    timeout_sec * 1000.0));

        ret = poll(&pfd, 1, timeout_ms);

        if (ret <= 0)
        {
            fcntl(fd, F_SETFL, old_flags);
            return false;
        }

        int socket_error = 0;
        socklen_t error_len =
            sizeof(socket_error);

        if (getsockopt(
                fd,
                SOL_SOCKET,
                SO_ERROR,
                &socket_error,
                &error_len) < 0)
        {
            fcntl(fd, F_SETFL, old_flags);
            return false;
        }

        fcntl(fd, F_SETFL, old_flags);

        return socket_error == 0;
    }

    void configureConnectedSocket(int fd)
    {
        int one = 1;

        if (setsockopt(
                fd,
                SOL_SOCKET,
                SO_KEEPALIVE,
                &one,
                sizeof(one)) < 0)
        {
            ROS_WARN(
                "设置 SO_KEEPALIVE 失败：%s",
                strerror(errno));
        }

        if (setsockopt(
                fd,
                IPPROTO_TCP,
                TCP_NODELAY,
                &one,
                sizeof(one)) < 0)
        {
            ROS_WARN(
                "设置 TCP_NODELAY 失败：%s",
                strerror(errno));
        }

        int value = tcp_keepidle_;
        if (setsockopt(
                fd,
                IPPROTO_TCP,
                TCP_KEEPIDLE,
                &value,
                sizeof(value)) < 0)
        {
            ROS_WARN(
                "设置 TCP_KEEPIDLE 失败：%s",
                strerror(errno));
        }

        value = tcp_keepintvl_;
        if (setsockopt(
                fd,
                IPPROTO_TCP,
                TCP_KEEPINTVL,
                &value,
                sizeof(value)) < 0)
        {
            ROS_WARN(
                "设置 TCP_KEEPINTVL 失败：%s",
                strerror(errno));
        }

        value = tcp_keepcnt_;
        if (setsockopt(
                fd,
                IPPROTO_TCP,
                TCP_KEEPCNT,
                &value,
                sizeof(value)) < 0)
        {
            ROS_WARN(
                "设置 TCP_KEEPCNT 失败：%s",
                strerror(errno));
        }

#ifdef TCP_USER_TIMEOUT
        value = tcp_user_timeout_ms_;
        if (setsockopt(
                fd,
                IPPROTO_TCP,
                TCP_USER_TIMEOUT,
                &value,
                sizeof(value)) < 0)
        {
            ROS_WARN(
                "设置 TCP_USER_TIMEOUT 失败：%s",
                strerror(errno));
        }
#endif
    }

    void receiveLoop(int fd)
    {
        std::string cache;
        char buffer[1024];

        while (running_ && ros::ok())
        {
            const ssize_t n =
                recv(fd,
                     buffer,
                     sizeof(buffer),
                     0);

            if (n > 0)
            {
                last_rx_ms_.store(
                    steadyNowMs());

                cache.append(
                    buffer,
                    static_cast<size_t>(n));

                size_t pos = std::string::npos;

                while (
                    (pos = cache.find('\n'))
                    != std::string::npos)
                {
                    std::string line =
                        cache.substr(0, pos);

                    cache.erase(
                        0,
                        pos + 1);

                    if (!line.empty() &&
                        line.back() == '\r')
                    {
                        line.pop_back();
                    }

                    handleLine(line);
                }
            }
            else if (n == 0)
            {
                ROS_WARN(
                    "仿真端主动关闭TCP连接");
                break;
            }
            else
            {
                if (errno == EINTR)
                {
                    continue;
                }

                // forceDisconnect() 会 shutdown/close，
                // 此时 EBADF/ENOTCONN/ECONNRESET 都属于预期退出。
                if (running_ && ros::ok())
                {
                    ROS_WARN(
                        "TCP接收结束：%s",
                        strerror(errno));
                }
                break;
            }
        }
    }

    void handleLine(const std::string &line)
    {
        const std::vector<std::string> parts =
            split(line, '|');

        if (parts.empty())
        {
            return;
        }

        if (parts[0] == "PING")
        {
            handlePing(parts);
            return;
        }

        if (parts[0] == "PONG")
        {
            handlePong(parts);
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

        ROS_WARN(
            "收到未知TCP消息：[%s]",
            line.c_str());
    }

    void handlePing(
        const std::vector<std::string> &parts)
    {
        if (parts.size() != 2)
        {
            ROS_WARN("PING格式错误");
            return;
        }

        const std::string pong =
            "PONG|" + parts[1] + "\n";

        if (!sendLine(pong))
        {
            ROS_WARN(
                "回复PONG失败，连接已进入重连流程");
        }
    }

    void handlePong(
        const std::vector<std::string> &parts)
    {
        if (parts.size() != 2)
        {
            ROS_WARN("PONG格式错误");
            return;
        }

        ROS_DEBUG_THROTTLE(
            5.0,
            "收到仿真端PONG");
    }

    void handleTargetAck(
        const std::vector<std::string> &parts)
    {
        if (parts.size() != 2)
        {
            ROS_WARN("ACK_TARGET格式错误");
            return;
        }

        uint64_t seq = 0;

        if (!parseSeq(parts[1], seq))
        {
            ROS_WARN(
                "ACK_TARGET seq_id非法：[%s]",
                parts[1].c_str());
            return;
        }

        bool accepted = false;

        {
            std::lock_guard<std::mutex>
                lock(state_mutex_);

            if (task_active_ &&
                waiting_target_ack_ &&
                seq == current_seq_)
            {
                waiting_target_ack_ = false;
                target_retry_count_ = 0;
                accepted = true;
            }
        }

        if (accepted)
        {
            ROS_INFO(
                "收到ACK_TARGET，seq=%llu，"
                "目标任务已被仿真端确认接收",
                static_cast<unsigned long long>(seq));
        }
        else
        {
            ROS_WARN(
                "收到非当前任务的ACK_TARGET，"
                "seq=%llu，忽略",
                static_cast<unsigned long long>(seq));
        }
    }

    void handleResult(
        const std::vector<std::string> &parts)
    {
        if (parts.size() != 3)
        {
            ROS_WARN(
                "RESULT格式错误："
                "应为 RESULT|seq|status");
            return;
        }

        uint64_t seq = 0;

        if (!parseSeq(parts[1], seq))
        {
            ROS_WARN(
                "RESULT seq_id非法：[%s]",
                parts[1].c_str());
            return;
        }

        const std::string status = parts[2];

        if (status.empty())
        {
            ROS_WARN("RESULT状态为空，忽略");
            return;
        }

        bool publish_result = false;
        bool ack_result = false;
        bool duplicate_result = false;

        {
            std::lock_guard<std::mutex>
                lock(state_mutex_);

            if (task_active_ &&
                seq == current_seq_)
            {
                publish_result = true;
                ack_result = true;

                task_active_ = false;
                waiting_target_ack_ = false;

                have_last_completed_seq_ = true;
                last_completed_seq_ = seq;
            }
            else if (
                have_last_completed_seq_ &&
                seq == last_completed_seq_)
            {
                // ACK_RESULT可能丢失。
                // 对重复RESULT只重新ACK，
                // 绝不能重复发布ROS完成消息。
                ack_result = true;
                duplicate_result = true;
            }
        }

        if (!ack_result)
        {
            ROS_WARN(
                "收到未知任务RESULT，seq=%llu，当前不确认",
                static_cast<unsigned long long>(seq));
            return;
        }

        const std::string ack =
            "ACK_RESULT|" +
            std::to_string(seq) +
            "\n";

        if (sendLine(ack))
        {
            if (duplicate_result)
            {
                ROS_INFO(
                    "收到重复RESULT，seq=%llu，"
                    "仅重新发送ACK_RESULT",
                    static_cast<unsigned long long>(seq));
            }
            else
            {
                ROS_INFO(
                    "已发送ACK_RESULT，seq=%llu",
                    static_cast<unsigned long long>(seq));
            }
        }

        if (publish_result)
        {
            std_msgs::String msg;
            msg.data = status;
            task_finished_pub_.publish(msg);

            ROS_INFO(
                "TCP -> ROS：任务seq=%llu结果[%s]，"
                "已发布/car_task_finished",
                static_cast<unsigned long long>(seq),
                status.c_str());
        }
    }

    void detectedTargetCallback(
        const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data.empty())
        {
            ROS_WARN(
                "/detected_target内容为空，忽略");
            return;
        }

        uint64_t seq = 0;
        std::string line;

        {
            std::lock_guard<std::mutex>
                lock(state_mutex_);

            if (task_active_)
            {
                ROS_WARN(
                    "当前任务seq=%llu尚未结束，"
                    "新目标[%s]被忽略",
                    static_cast<unsigned long long>(
                        current_seq_),
                    msg->data.c_str());
                return;
            }

            seq = next_seq_id_++;
            current_seq_ = seq;
            current_target_ = msg->data;
            task_active_ = true;
            waiting_target_ack_ = true;
            target_retry_count_ = 0;
            last_target_send_time_ =
                ros::WallTime::now();

            line = buildTargetLine(
                current_seq_,
                current_target_);
        }

        if (sendLine(line))
        {
            ROS_INFO(
                "ROS -> TCP：发送TARGET seq=%llu，类别[%s]",
                static_cast<unsigned long long>(seq),
                msg->data.c_str());
        }
        else
        {
            ROS_WARN(
                "TARGET seq=%llu暂未发送成功，"
                "等待连接恢复后自动重发",
                static_cast<unsigned long long>(seq));
        }
    }

    void retryTimerCallback(
        const ros::WallTimerEvent &)
    {
        // 已断线时不消耗ACK重试次数，
        // 等networkLoop重连后resendPendingTargetAfterReconnect()。
        if (!isConnected())
        {
            return;
        }

        std::string line;
        uint64_t seq = 0;
        int retry_count = 0;
        bool force_reconnect = false;

        {
            std::lock_guard<std::mutex>
                lock(state_mutex_);

            if (!task_active_ ||
                !waiting_target_ack_)
            {
                return;
            }

            const double elapsed =
                (ros::WallTime::now() -
                 last_target_send_time_)
                    .toSec();

            if (elapsed < ack_timeout_)
            {
                return;
            }

            // 已经完成N次“重发”，又经过一个完整ACK窗口仍没收到，
            // 说明旧连接高度可疑，直接废弃。
            if (target_retry_count_ >=
                max_ack_retries_before_reconnect_)
            {
                seq = current_seq_;
                force_reconnect = true;

                // 防止timer在socket真正退出前重复触发同一条日志。
                last_target_send_time_ =
                    ros::WallTime::now();
            }
            else
            {
                seq = current_seq_;

                line = buildTargetLine(
                    current_seq_,
                    current_target_);

                ++target_retry_count_;
                retry_count =
                    target_retry_count_;

                last_target_send_time_ =
                    ros::WallTime::now();
            }
        }

        if (force_reconnect)
        {
            ROS_ERROR(
                "ACK_TARGET连续%d次重试仍无响应，"
                "判定当前TCP可能僵死；"
                "主动断链并重连，seq=%llu",
                max_ack_retries_before_reconnect_,
                static_cast<unsigned long long>(seq));

            forceDisconnect(
                "ACK_TARGET连续超时");
            return;
        }

        if (sendLine(line))
        {
            ROS_WARN(
                "ACK_TARGET超时，重发TARGET seq=%llu，"
                "第%d/%d次",
                static_cast<unsigned long long>(seq),
                retry_count,
                max_ack_retries_before_reconnect_);
        }
        else
        {
            ROS_WARN(
                "TARGET seq=%llu第%d次重试时连接不可用，"
                "等待自动重连",
                static_cast<unsigned long long>(seq),
                retry_count);
        }
    }

    void healthTimerCallback(
        const ros::WallTimerEvent &)
    {
        if (!isConnected())
        {
            return;
        }

        const int64_t now_ms =
            steadyNowMs();

        const int64_t last_rx =
            last_rx_ms_.load();

        if (last_rx > 0)
        {
            const double silent_sec =
                static_cast<double>(
                    now_ms - last_rx) /
                1000.0;

            if (silent_sec >
                heartbeat_timeout_)
            {
                ROS_ERROR(
                    "TCP心跳判死：连续%.2fs未收到仿真端任何数据"
                    "（阈值%.2fs），主动重连",
                    silent_sec,
                    heartbeat_timeout_);

                forceDisconnect(
                    "应用层心跳超时");
                return;
            }
        }

        const int64_t last_ping =
            last_ping_send_ms_.load();

        if (last_ping == 0 ||
            (now_ms - last_ping) >=
                static_cast<int64_t>(
                    heartbeat_interval_ *
                    1000.0))
        {
            const uint64_t seq =
                heartbeat_seq_.fetch_add(1);

            const std::string ping =
                "PING|" +
                std::to_string(seq) +
                "\n";

            // 先记时间，避免send失败和timer并发造成疯狂重复。
            last_ping_send_ms_.store(
                now_ms);

            if (!sendLine(ping))
            {
                ROS_WARN(
                    "发送PING失败，"
                    "连接已进入自动重连流程");
            }
        }
    }

    void resendPendingTargetAfterReconnect()
    {
        std::string line;
        uint64_t seq = 0;
        bool need_send = false;

        {
            std::lock_guard<std::mutex>
                lock(state_mutex_);

            if (task_active_ &&
                waiting_target_ack_)
            {
                seq = current_seq_;

                line = buildTargetLine(
                    current_seq_,
                    current_target_);

                // 新连接重新获得完整的ACK重试预算。
                target_retry_count_ = 0;
                last_target_send_time_ =
                    ros::WallTime::now();

                need_send = true;
            }
        }

        if (need_send)
        {
            if (sendLine(line))
            {
                ROS_WARN(
                    "连接恢复，立即重发待确认TARGET seq=%llu",
                    static_cast<unsigned long long>(seq));
            }
            else
            {
                ROS_WARN(
                    "连接刚恢复但TARGET seq=%llu重发失败，"
                    "将再次进入重连",
                    static_cast<unsigned long long>(seq));
            }
        }
    }

    static std::string buildTargetLine(
        uint64_t seq,
        const std::string &target)
    {
        return
            "TARGET|" +
            std::to_string(seq) +
            "|" +
            target +
            "\n";
    }

    bool sendLine(const std::string &data)
    {
        std::lock_guard<std::mutex>
            lock(socket_mutex_);

        if (socket_fd_ < 0)
        {
            return false;
        }

        const int fd = socket_fd_;
        size_t sent = 0;

        while (sent < data.size())
        {
            const ssize_t n = send(
                fd,
                data.data() + sent,
                data.size() - sent,
                MSG_NOSIGNAL);

            if (n <= 0)
            {
                const int saved_errno =
                    errno;

                ROS_WARN(
                    "TCP发送失败：%s；"
                    "立即废弃当前socket并重连",
                    strerror(saved_errno));

                invalidateSocketLocked(fd);

                return false;
            }

            sent +=
                static_cast<size_t>(n);
        }

        return true;
    }

    bool isConnected()
    {
        std::lock_guard<std::mutex>
            lock(socket_mutex_);

        return socket_fd_ >= 0;
    }

    void forceDisconnect(
        const std::string &reason,
        bool print_log = true)
    {
        std::lock_guard<std::mutex>
            lock(socket_mutex_);

        if (socket_fd_ < 0)
        {
            return;
        }

        const int fd = socket_fd_;

        if (print_log)
        {
            ROS_WARN(
                "主动废弃TCP连接：%s",
                reason.c_str());
        }

        invalidateSocketLocked(fd);
    }

    void invalidateSocketLocked(int expected_fd)
    {
        if (socket_fd_ != expected_fd ||
            expected_fd < 0)
        {
            return;
        }

        socket_fd_ = -1;

        shutdown(
            expected_fd,
            SHUT_RDWR);

        close(expected_fd);
    }

    void closeSocketIfCurrent(int fd)
    {
        std::lock_guard<std::mutex>
            lock(socket_mutex_);

        if (socket_fd_ == fd)
        {
            socket_fd_ = -1;
            shutdown(fd, SHUT_RDWR);
            close(fd);
        }
    }

    void sleepReconnectInterval()
    {
        int milliseconds =
            static_cast<int>(
                reconnect_interval_ *
                1000.0);

        milliseconds =
            std::max(
                100,
                milliseconds);

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                milliseconds));
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber detected_target_sub_;
    ros::Publisher task_finished_pub_;
    ros::WallTimer retry_timer_;
    ros::WallTimer health_timer_;

    std::string server_host_;
    std::string server_fallback_host_;
    std::string last_successful_ip_;

    int server_port_;
    double reconnect_interval_;
    double connect_timeout_;
    double ack_timeout_;
    int max_ack_retries_before_reconnect_;

    double heartbeat_interval_;
    double heartbeat_timeout_;

    int tcp_keepidle_;
    int tcp_keepintvl_;
    int tcp_keepcnt_;
    int tcp_user_timeout_ms_;

    int socket_fd_;
    std::atomic<bool> running_;
    std::thread network_thread_;
    std::mutex socket_mutex_;

    std::atomic<int64_t> last_rx_ms_;
    std::atomic<int64_t> last_ping_send_ms_;
    std::atomic<uint64_t> heartbeat_seq_;

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

    ros::init(
        argc,
        argv,
        "car_comm_bridge");

    CarCommBridge bridge;

    ros::spin();
    return 0;
}