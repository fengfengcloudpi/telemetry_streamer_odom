#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "telemetry_streamer_odom/config.hpp"
#include "telemetry_streamer_odom/protocol.hpp"

#include <mutex>
#include <vector>
#include <cstdint>
#include <netinet/in.h>  // sockaddr_in

// 说明：
// 这个类负责：
//  - 订阅 /odom
//  - 周期性按照配置打包遥测帧
//  - 通过 UDP 发送
//
// 依赖的结构体 FullConfig / StreamSpec / StreamMapEntry / NetworkSpec
// 已经在 config.hpp 中定义了。:contentReference[oaicite:1]{index=1}
class TelemetryStreamerNode : public rclcpp::Node
{
public:
    // 构造：接收完整配置（含网络、streams 映射等）
    explicit TelemetryStreamerNode(const FullConfig &cfg);

    // 析构：关闭 UDP socket
    ~TelemetryStreamerNode() override;

private:
    // ===== 内部辅助结构 =====

    // 缓存最近一次里程计消息，带互斥锁
    struct OdomCache {
        std::mutex mtx;
        nav_msgs::msg::Odometry last;
        bool has = false;
    };

    // 运行时的每条遥测流（对应 FullConfig::streams[i]）
    // step/offset 用于决定该流在哪些 tick 上发送
    struct StreamRuntime {
        const StreamSpec *spec;   // 指向配置里的 stream
        int step;                 // period_ms / base_tick_ms
        int offset;               // phase_ms / base_tick_ms
        uint16_t template_ver;    // 协议模板版本(目前写死=1)
    };

    // ===== 定时器触发函数 =====
    // 全局 tick，每 base_tick_ms_ 触发一次
    void onTick();

    // 根据配置把 /odom 的字段拷到 float 槽位里，生成该流要发的浮点数组
    std::vector<float> extract_odom_floats(const StreamSpec &s);

    // ===== 成员变量 =====

    FullConfig cfg_;

    // UDP发送
    int sock_fd_{-1};
    sockaddr_in dest_addr_{};  // 目标IP/端口

    // 定时调度
    int base_tick_ms_{10};
    uint64_t tick_count_{0};     // 全局tick计数
    uint32_t seq_counter_{0};    // 发包自增序号

    rclcpp::TimerBase::SharedPtr timer_;

    // /odom缓存和订阅
    OdomCache odom_cache_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // 所有启用的流在运行期的调度信息
    std::vector<StreamRuntime> runtimes_;
};
