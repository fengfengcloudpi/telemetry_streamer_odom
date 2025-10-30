#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>

// =================== 配置结构体 ===================
// 这些来自你的 config.hpp，如果你已有就保持一致

struct FieldMapping {
    std::string kind;   // "float" 等
    int index;
    std::string path;
};

struct StreamSpec {
    int id;
    std::string name;
    std::string topic;
    bool enable;
    int period_ms;
    int phase_ms;
    int n_floats;
    std::vector<FieldMapping> mappings;
};

struct NetworkSpec {
    std::string dest_ip;
    int port;
    int base_tick_ms;
};

struct FullConfig {
    NetworkSpec net;
    std::vector<StreamSpec> streams;
};

// =================== 运行时结构 ===================

struct StreamRuntime {
    const StreamSpec* spec = nullptr;
    int step = 1;
    int offset = 0;
    int template_ver = 1;
};

struct OdomCache {
    std::mutex mtx;
    nav_msgs::msg::Odometry last{};
    bool has = false;
};

// =================== 主节点类 ===================

class TelemetryStreamerNode : public rclcpp::Node
{
public:
    explicit TelemetryStreamerNode(const FullConfig &cfg);
    ~TelemetryStreamerNode();

    // ========== 新增声明 ==========
    // 线程安全复制缓存
    nav_msgs::msg::Odometry copyOdom();

    // /odom 专用字段提取函数
    std::vector<float> extract_odom_floats(const StreamSpec &s);

    // 定时调度
    void onTick();

private:
    // ===== UDP 网络 =====
    int sock_fd_ = -1;
    struct sockaddr_in dest_addr_{};
    int base_tick_ms_ = 10;
    uint64_t seq_counter_ = 0;
    uint64_t tick_count_ = 0;

    // ===== ROS 订阅 =====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // ===== 缓存与配置 =====
    OdomCache odom_cache_;
    FullConfig cfg_;

    // ===== 定时器与运行时状态 =====
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<StreamRuntime> runtimes_;
};

// =================== 外部接口声明 ===================
// 让 topic_extractors.cpp 可以调用
std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s);
