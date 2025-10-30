#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
#include "telemetry_streamer_odom/odom_fields.hpp"
#include "telemetry_streamer_odom/imu_fields.hpp"           // ✅ 新增
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

// ---------------- 构造 ----------------
TelemetryStreamerNode::TelemetryStreamerNode(const FullConfig &cfg)
: rclcpp::Node("telemetry_streamer_odom"), cfg_(cfg)
{
    // UDP
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

    // 动态订阅
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
        } else if (s.topic == "/imu") {  // ✅ 新增
            sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                s.topic, 50,  // IMU 通常更高频
                [this](sensor_msgs::msg::Imu::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lk(imu_cache_.mtx);
                    imu_cache_.last = *msg;
                    imu_cache_.has  = true;
                });
        }

        StreamRuntime rt;
        rt.spec = &s;
        rt.step   = std::max(1, s.period_ms / base_tick_ms_);
        rt.offset = std::max(0, s.phase_ms  / base_tick_ms_);
        if (rt.step > 0) rt.offset %= rt.step; // 小健壮性
        rt.template_ver = 1;
        runtimes_.push_back(rt);
    }

    // 定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(base_tick_ms_),
        std::bind(&TelemetryStreamerNode::onTick, this));

    RCLCPP_INFO(get_logger(), "telemetry_streamer_odom started. base_tick_ms=%d", base_tick_ms_);
}

// ---------------- 析构 ----------------
TelemetryStreamerNode::~TelemetryStreamerNode()
{
    if (sock_fd_ >= 0) ::close(sock_fd_);
}

// ---------------- 缓存复制 ----------------
nav_msgs::msg::Odometry TelemetryStreamerNode::copyOdom()
{
    std::lock_guard<std::mutex> lk(odom_cache_.mtx);
    return odom_cache_.last;
}

sensor_msgs::msg::Imu TelemetryStreamerNode::copyImu()     // ✅ 新增
{
    std::lock_guard<std::mutex> lk(imu_cache_.mtx);
    return imu_cache_.last;
}

// ---------------- ODOM 提取 ----------------
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

// 适配器
std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s)
{
    return self->extract_odom_floats(s);
}

// ---------------- IMU 提取（新） ----------------
std::vector<float> TelemetryStreamerNode::extract_imu_floats(const StreamSpec &s)
{
    std::vector<float> out(s.n_floats, 0.0f);

    // 复制 IMU
    auto msg_copy = copyImu();
    {
        std::lock_guard<std::mutex> lk(imu_cache_.mtx);
        if (!imu_cache_.has) return out;
    }

    // 计算 RPY（右手系）
    const auto &q = msg_copy.orientation;
    // roll (x-axis rotation)
    float sinr_cosp = 2.f * (q.w*q.x + q.y*q.z);
    float cosr_cosp = 1.f - 2.f * (q.x*q.x + q.y*q.y);
    float roll  = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    float sinp = 2.f * (q.w*q.y - q.z*q.x);
    float pitch;
    if (std::abs(sinp) >= 1.f) pitch = std::copysign(static_cast<float>(M_PI)/2.f, sinp);
    else                       pitch = std::asin(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2.f * (q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.f - 2.f * (q.y*q.y + q.z*q.z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    ImuAngles ang{roll, pitch, yaw};

    // 按映射填充
    for (const auto &m : s.mappings)
    {
        if (m.kind != "float") continue; // 现阶段只发 float
        if (m.index < 0 || m.index >= s.n_floats) continue;

        auto it = IMU_FIELD_MAP.find(m.path);
        if (it != IMU_FIELD_MAP.end()) {
            out[m.index] = it->second(msg_copy, ang);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown IMU field path: %s", m.path.c_str());
        }
    }
    return out;
}

// 适配器
std::vector<float> extract_imu(TelemetryStreamerNode* self, const StreamSpec& s)
{
    return self->extract_imu_floats(s);
}

// ---------------- 定时调度 ----------------
void TelemetryStreamerNode::onTick()
{
    tick_count_++;

    const auto now = this->now();
    uint64_t ts_usec = (uint64_t)now.seconds() * 1'000'000ULL
                     + (uint64_t)(now.nanoseconds() % 1'000'000'000ULL) / 1000ULL;

    for (auto &rt : runtimes_) {
        const auto &s = *rt.spec;
        if ((tick_count_ % rt.step) != rt.offset) continue;

        // 话题 → 提取
        auto it = TOPIC_EXTRACTOR_MAP.find(s.topic);
        if (it == TOPIC_EXTRACTOR_MAP.end()) {
            RCLCPP_ERROR(get_logger(), "No extractor for topic: %s", s.topic.c_str());
            continue;
        }

        std::vector<float> floats = it->second(this, s);

        auto pkt = build_stream_frame(
            s.id, rt.template_ver, ts_usec, seq_counter_++, floats);

        ::sendto(sock_fd_,
                 pkt.bytes.data(), pkt.bytes.size(),
                 0, reinterpret_cast<sockaddr*>(&dest_addr_),
                 sizeof(dest_addr_));
    }
}
