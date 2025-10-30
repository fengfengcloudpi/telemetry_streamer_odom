#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>          // ✅ 新增
#include <vector>
#include <string>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>

// ---------------- 配置结构（保持你现有定义即可） ----------------
struct FieldMapping {
    std::string kind;   // "float", ...
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

// ---------------- 运行时结构 ----------------
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

// ✅ 新增：IMU 缓存
struct ImuCache {
    std::mutex mtx;
    sensor_msgs::msg::Imu last{};
    bool has = false;
};

// ---------------- 主节点类 ----------------
class TelemetryStreamerNode : public rclcpp::Node
{
public:
    explicit TelemetryStreamerNode(const FullConfig &cfg);
    ~TelemetryStreamerNode();

    // 拷贝缓存
    nav_msgs::msg::Odometry copyOdom();
    sensor_msgs::msg::Imu   copyImu();    // ✅ 新增

    // 字段提取
    std::vector<float> extract_odom_floats(const StreamSpec &s);
    std::vector<float> extract_imu_floats (const StreamSpec &s);  // ✅ 新增

    // 定时调度
    void onTick();

private:
    // UDP
    int sock_fd_ = -1;
    struct sockaddr_in dest_addr_{};
    int base_tick_ms_ = 10;
    uint64_t seq_counter_ = 0;
    uint64_t tick_count_ = 0;

    // 订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     sub_imu_;   // ✅ 新增

    // 缓存与配置
    OdomCache odom_cache_;
    ImuCache  imu_cache_;  // ✅ 新增
    FullConfig cfg_;

    // 定时器与运行时
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<StreamRuntime> runtimes_;
};

// 供 topic 表调用
std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s);
std::vector<float> extract_imu (TelemetryStreamerNode* self, const StreamSpec& s); // ✅ 新增
