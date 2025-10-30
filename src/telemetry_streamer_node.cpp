#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
#include "telemetry_streamer_odom/odom_fields.hpp"
#include "telemetry_streamer_odom/topic_extractors.hpp"
#include "telemetry_streamer_odom/field_kind_dispatch.hpp"

#include <chrono>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std::chrono_literals;

// ===================== 构造函数 =====================

TelemetryStreamerNode::TelemetryStreamerNode(const FullConfig &cfg)
: rclcpp::Node("telemetry_streamer_odom"), cfg_(cfg)
{
    // 1) 初始化 UDP socket
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        RCLCPP_FATAL(get_logger(), "Failed to create UDP socket");
        throw std::runtime_error("socket creation failed");
    }

    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port   = htons(cfg_.net.port);
    ::inet_pton(AF_INET, cfg_.net.dest_ip.c_str(), &dest_addr_.sin_addr);

    base_tick_ms_ = cfg_.net.base_tick_ms > 0 ? cfg_.net.base_tick_ms : 10;

    // 2) 动态订阅各 topic
    for (auto &s : cfg_.streams)
    {
        if (!s.enable) continue;

        if (s.topic == "/odom") {
            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                s.topic, 10,
                [this](nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lk(odom_cache_.mtx);
                    odom_cache_.last = *msg;
                    odom_cache_.has  = true;
                });
        }
        // future: /imu /gps ...

        StreamRuntime rt;
        rt.spec = &s;
        rt.step   = std::max(1, s.period_ms / base_tick_ms_);
        rt.offset = std::max(0, s.phase_ms  / base_tick_ms_);
        rt.template_ver = 1;
        runtimes_.push_back(rt);
    }

    // 3) 定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(base_tick_ms_),
        std::bind(&TelemetryStreamerNode::onTick, this));

    RCLCPP_INFO(get_logger(),
                "telemetry_streamer_odom started. base_tick_ms=%d", base_tick_ms_);
}

// ===================== 析构函数 =====================

TelemetryStreamerNode::~TelemetryStreamerNode()
{
    if (sock_fd_ >= 0) ::close(sock_fd_);
}

// ===================== 拷贝 Odom 缓存 =====================

nav_msgs::msg::Odometry TelemetryStreamerNode::copyOdom()
{
    std::lock_guard<std::mutex> lk(odom_cache_.mtx);
    return odom_cache_.last;
}

// ===================== 提取函数 =====================

std::vector<float> TelemetryStreamerNode::extract_odom_floats(const StreamSpec &s)
{
    std::vector<float> out(s.n_floats, 0.0f);

    for (const auto &m : s.mappings)
    {
        auto kind_it = FIELD_KIND_MAP.find(m.kind);
        if (kind_it == FIELD_KIND_MAP.end()) {
            RCLCPP_ERROR(this->get_logger(), "Unknown field kind: %s", m.kind.c_str());
            continue;
        }
        kind_it->second(this, s, m, out);
    }

    return out;
}

// adapter for topic table
std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s)
{
    return self->extract_odom_floats(s);
}

// ===================== 定时调度 =====================

void TelemetryStreamerNode::onTick()
{
    tick_count_++;

    const auto now = this->now();
    uint64_t ts_usec = (uint64_t)now.seconds() * 1'000'000ULL
                     + (uint64_t)(now.nanoseconds() % 1'000'000'000ULL) / 1000ULL;

    for (auto &rt : runtimes_) {
        const auto &s = *rt.spec;
        if ((tick_count_ % rt.step) != rt.offset) continue;

        // topic → extractor
        auto it = TOPIC_EXTRACTOR_MAP.find(s.topic);
        if (it == TOPIC_EXTRACTOR_MAP.end()) {
            RCLCPP_ERROR(get_logger(), "No extractor for topic: %s", s.topic.c_str());
            continue;
        }

        std::vector<float> floats = it->second(this, s);

        // UDP 打包并发送
        auto pkt = build_stream_frame(
            s.id, rt.template_ver, ts_usec, seq_counter_++, floats);

        ::sendto(sock_fd_,
                 pkt.bytes.data(), pkt.bytes.size(),
                 0,
                 reinterpret_cast<sockaddr*>(&dest_addr_),
                 sizeof(dest_addr_));
    }
}
