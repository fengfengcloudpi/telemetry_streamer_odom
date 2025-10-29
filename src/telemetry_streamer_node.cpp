#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <cstring>   // std::memset
#include <cmath>     // std::atan2
#include <stdexcept> // std::runtime_error
#include <algorithm> // std::max

using namespace std::chrono_literals;

TelemetryStreamerNode::TelemetryStreamerNode(const FullConfig &cfg)
: rclcpp::Node("telemetry_streamer_odom"),
  cfg_(cfg)
{
    // === 1. 初始化 UDP ===
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0)
    {
        RCLCPP_FATAL(get_logger(), "Failed to create UDP socket");
        throw std::runtime_error("socket()");
    }

    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port   = htons(cfg_.net.port);

    if (::inet_pton(AF_INET, cfg_.net.dest_ip.c_str(), &dest_addr_.sin_addr) != 1)
    {
        RCLCPP_FATAL(get_logger(), "Bad dest ip: %s", cfg_.net.dest_ip.c_str());
        throw std::runtime_error("dest ip");
    }

    base_tick_ms_ = cfg_.net.base_tick_ms;
    if (base_tick_ms_ <= 0)
        base_tick_ms_ = 10;  // fallback

    // === 2. 订阅 /odom ===
    // 这里沿用了你原本写死的 "/odom" 话题和 QoS=10
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(odom_cache_.mtx);
            odom_cache_.last = *msg;
            odom_cache_.has  = true;
        }
    );

    // === 3. 根据配置构建运行时 stream 列表 ===
    // 你当前实现里只真正处理 topic == "/odom" 的流
    for (auto &s : cfg_.streams)
    {
        if (!s.enable)
            continue;
        if (s.topic != "/odom")
            continue;  // 目前只支持里程计

        StreamRuntime rt;
        rt.spec = &s;

        // 全局 tick 调度参数：
        // step = period_ms / base_tick_ms
        // offset = phase_ms / base_tick_ms
        rt.step   = std::max(1, s.period_ms / base_tick_ms_);
        rt.offset = std::max(0, s.phase_ms  / base_tick_ms_);

        // 模板版本设为1（固定）
        rt.template_ver = 1;

        runtimes_.push_back(rt);
    }

    // === 4. 全局定时器 ===
    // 每 base_tick_ms_ 触发一次 onTick()
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(base_tick_ms_),
        std::bind(&TelemetryStreamerNode::onTick, this)
    );

    RCLCPP_INFO(
        get_logger(),
        "telemetry_streamer_odom started. base_tick_ms=%d",
        base_tick_ms_
    );
}

TelemetryStreamerNode::~TelemetryStreamerNode()
{
    if (sock_fd_ >= 0)
    {
        ::close(sock_fd_);
    }
}

void TelemetryStreamerNode::onTick()
{
    // 计数加一
    tick_count_++;

    // 获取当前时间戳（微秒）
    const auto now = this->now();
    uint64_t ts_usec =
        static_cast<uint64_t>(now.seconds()) * 1000000ULL +
        static_cast<uint64_t>(now.nanoseconds() % 1000000000ULL) / 1000ULL;

    // 遍历已启用的每条 stream，判断这一次 tick 是否应该发送
    for (auto &rt : runtimes_)
    {
        const auto &s = *rt.spec;

        // 如果当前 tick 不在该 stream 的发送时序上，就跳过
        if ( (tick_count_ % rt.step) != rt.offset )
        {
            continue;
        }

        // 1) 从缓存的里程计中提取 float 槽
        std::vector<float> floats = extract_odom_floats(s);

        // 2) 使用你的协议打包
        auto pkt = build_stream_frame(
            s.id,               // stream id
            rt.template_ver,    // 模板版本
            ts_usec,            // 时间戳(微秒)
            seq_counter_++,     // 递增序号
            floats              // payload (float数组)
        );

        // 3) UDP 发送
        ssize_t sent = ::sendto(
            sock_fd_,
            pkt.bytes.data(),
            pkt.bytes.size(),
            0,
            reinterpret_cast<sockaddr*>(&dest_addr_),
            sizeof(dest_addr_)
        );
        (void)sent; // 需要的话你可以在这里加错误检查或调试日志
    }
}

std::vector<float> TelemetryStreamerNode::extract_odom_floats(const StreamSpec &s)
{
    // 先准备好固定长度的 float 槽，全部初始化 0.0f
    std::vector<float> out;
    out.resize(s.n_floats, 0.0f);

    // 把缓存的 odom 复制出来（带锁）
    nav_msgs::msg::Odometry msg_copy;
    {
        std::lock_guard<std::mutex> lk(odom_cache_.mtx);
        if (!odom_cache_.has)
        {
            // 还没收到任何 /odom，就保持全0
            return out;
        }
        msg_copy = odom_cache_.last;
    }

    // 计算 yaw（假设在平面上运动）
    float yaw = 0.0f;
    {
        const auto &q = msg_copy.pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    }

    // 遍历配置里的映射表，把对应字段塞进 out[m.index]
    for (const auto &m : s.mappings)
    {
        if (m.kind != "float")
            continue;
        if (m.index < 0 || m.index >= s.n_floats)
            continue;

        float value = 0.0f;

        // 这些 if-else 分支来自你原本的硬编码映射：
        // "pose.pose.position.x"
        // "pose.pose.position.y"
        // "pose.pose.orientation.yaw"
        // "twist.twist.linear.x"
        // "twist.twist.linear.y"
        // "twist.twist.angular.z"
        if (m.path == "pose.pose.position.x")
        {
            value = static_cast<float>(msg_copy.pose.pose.position.x);
        }
        else if (m.path == "pose.pose.position.y")
        {
            value = static_cast<float>(msg_copy.pose.pose.position.y);
        }
        else if (m.path == "pose.pose.orientation.yaw")
        {
            value = yaw;
        }
        else if (m.path == "twist.twist.linear.x")
        {
            value = static_cast<float>(msg_copy.twist.twist.linear.x);
        }
        else if (m.path == "twist.twist.linear.y")
        {
            value = static_cast<float>(msg_copy.twist.twist.linear.y);
        }
        else if (m.path == "twist.twist.angular.z")
        {
            value = static_cast<float>(msg_copy.twist.twist.angular.z);
        }

        out[m.index] = value;
    }

    return out;
}
