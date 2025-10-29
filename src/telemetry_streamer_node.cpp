#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "telemetry_streamer_odom/config.hpp"
#include "telemetry_streamer_odom/protocol.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

// 缓存最新的 /odom
struct OdomCache {
    std::mutex mtx;
    nav_msgs::msg::Odometry last;
    bool has = false;
};

// 每个要发送的stream在运行时的调度信息
struct StreamRuntime {
    const StreamSpec* spec;
    int step;         // period_ms / base_tick_ms
    int offset;       // phase_ms / base_tick_ms
    uint16_t template_ver;
};

class TelemetryStreamerNode : public rclcpp::Node {
public:
    TelemetryStreamerNode(const FullConfig& cfg)
    : Node("telemetry_streamer_odom"), cfg_(cfg)
    {
        // 打开UDP
        sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to create UDP socket");
            throw std::runtime_error("socket()");
        }
        std::memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port   = htons(cfg_.net.port);
        if (::inet_pton(AF_INET, cfg_.net.dest_ip.c_str(), &dest_addr_.sin_addr) != 1) {
            RCLCPP_FATAL(get_logger(), "Bad dest ip");
            throw std::runtime_error("dest ip");
        }

        base_tick_ms_ = cfg_.net.base_tick_ms;
        if (base_tick_ms_ <= 0) base_tick_ms_ = 10;

        // 订阅 /odom
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg){
                std::lock_guard<std::mutex> lk(odom_cache_.mtx);
                odom_cache_.last = *msg;
                odom_cache_.has = true;
            }
        );

        // 只关心启用的 stream（我们假定只有一个：odom）
        for (auto &s : cfg_.streams) {
            if (!s.enable) continue;
            if (s.topic != "/odom") continue; // 本版本只支持odometry

            StreamRuntime rt;
            rt.spec = &s;

            // 全局tick调度: step = period_ms / base_tick_ms
            rt.step   = std::max(1, s.period_ms / base_tick_ms_);
            rt.offset = std::max(0, s.phase_ms  / base_tick_ms_);

            // 模板版本号，这里先写1
            rt.template_ver = 1;
            runtimes_.push_back(rt);
        }

        // 全局tick定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(base_tick_ms_),
            std::bind(&TelemetryStreamerNode::onTick, this)
        );

        RCLCPP_INFO(get_logger(), "telemetry_streamer_odom started. base_tick_ms=%d", base_tick_ms_);
    }

    ~TelemetryStreamerNode() override {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
        }
    }

private:

    void onTick() {
        // 全局tick计数+1
        tick_count_++;

        // 时间戳（微秒）
        const auto now = this->now();
        uint64_t ts_usec =
            static_cast<uint64_t>(now.seconds()) * 1000000ULL +
            static_cast<uint64_t>(now.nanoseconds() % 1000000000ULL) / 1000ULL;

        // 对每个运行时stream，检查这个tick是不是要发
        for (auto &rt : runtimes_) {
            const auto &s = *rt.spec;
            if ( (tick_count_ % rt.step) != rt.offset ) {
                continue; // 本tick不该发
            }

            // 从缓存提取floats
            std::vector<float> floats = extract_odom_floats(s);

            // 打包成我们协议的帧
            auto pkt = build_stream_frame(
                s.id,
                rt.template_ver,
                ts_usec,
                seq_counter_++,
                floats
            );

            // UDP 发送
            ssize_t sent = ::sendto(
                sock_fd_,
                pkt.bytes.data(),
                pkt.bytes.size(),
                0,
                reinterpret_cast<sockaddr*>(&dest_addr_),
                sizeof(dest_addr_)
            );
            (void)sent; // 可以加错误处理/日志
        }
    }

    // 根据 XML <map> 配置，把 odom 里的值装进float槽
    std::vector<float> extract_odom_floats(const StreamSpec& s) {
        std::vector<float> out;
        out.resize(s.n_floats, 0.0f);

        std::lock_guard<std::mutex> lk(odom_cache_.mtx);
        if (!odom_cache_.has) {
            return out;
        }
        const auto &msg = odom_cache_.last;

        // 计算 yaw（基于四元数，假设机器人主要在平面运动）
        float yaw = 0.0f;
        {
            const auto &q = msg.pose.pose.orientation;
            double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
            double cosy_cosp = 1.0-2.0*(q.y*q.y + q.z*q.z);
            yaw = static_cast<float>( std::atan2(siny_cosp, cosy_cosp) );
        }

        // 遍历 XML 的映射
        for (auto &m : s.mappings) {
            if (m.kind != "float") continue;
            if (m.index < 0 || m.index >= s.n_floats) continue;

            float value = 0.0f;

            // 这里是硬编码字段 -> 值 的映射。
            // 后续可以用“getter表”替代这些 if。
            if (m.path == "pose.pose.position.x") {
                value = static_cast<float>(msg.pose.pose.position.x);
            } else if (m.path == "pose.pose.position.y") {
                value = static_cast<float>(msg.pose.pose.position.y);
            } else if (m.path == "pose.pose.orientation.yaw") {
                value = yaw;
            } else if (m.path == "twist.twist.linear.x") {
                value = static_cast<float>(msg.twist.twist.linear.x);
            } else if (m.path == "twist.twist.linear.y") {
                value = static_cast<float>(msg.twist.twist.linear.y);
            } else if (m.path == "twist.twist.angular.z") {
                value = static_cast<float>(msg.twist.twist.angular.z);
            }

            out[m.index] = value;
        }

        return out;
    }

private:
    FullConfig cfg_;

    int sock_fd_{-1};
    sockaddr_in dest_addr_{};

    int base_tick_ms_{10};
    uint64_t tick_count_{0};
    uint32_t seq_counter_{0};

    rclcpp::TimerBase::SharedPtr timer_;

    OdomCache odom_cache_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    std::vector<StreamRuntime> runtimes_;
};
