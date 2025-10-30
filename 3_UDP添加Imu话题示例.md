帮我添加imu数据的配置发送

###### ChatGPT 说：

好，我把 **IMU 数据的配置与发送**完整接入到你当前的“查表映射 + 话题抽象”框架里。你可以直接拷贝这些文件/片段进工程。改动点非常集中：

- 新增：`imu_fields.hpp`
- 头文件新增 IMU 缓存、订阅、提取函数声明
- `telemetry_streamer_node.cpp` 增加 `/imu` 订阅与 `extract_imu_floats`
- `topic_extractors.cpp` 注册 `/imu` 的 extractor
- `CMakeLists.txt` 加上 `sensor_msgs`

下面给你**完整代码**与**最小必要改动**。

------

# 1) 新增：IMU 字段映射表

**`include/telemetry_streamer_odom/imu_fields.hpp`**

```
#pragma once
#include <unordered_map>
#include <functional>
#include <string>
#include <sensor_msgs/msg/imu.hpp>

using ImuMsg = sensor_msgs::msg::Imu;

// 说明：roll/pitch/yaw 由外部提前计算后传入 getter 的第二个参数（此处复用 yaw 参数位）
// 约定：第二个参数传入的是 yaw，roll/pitch 在回调闭包内捕获，见 extract_imu_floats 实现
struct ImuAngles {
  float roll{0.f}, pitch{0.f}, yaw{0.f};
};

static const std::unordered_map<std::string, std::function<float(const ImuMsg&, const ImuAngles&)>> IMU_FIELD_MAP = {
    // Orientation (RPY from quaternion)
    {"orientation.roll",  [](const ImuMsg&, const ImuAngles& ang){ return ang.roll; }},
    {"orientation.pitch", [](const ImuMsg&, const ImuAngles& ang){ return ang.pitch; }},
    {"orientation.yaw",   [](const ImuMsg&, const ImuAngles& ang){ return ang.yaw; }},

    // Angular velocity
    {"gyro.x", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.angular_velocity.x); }},
    {"gyro.y", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.angular_velocity.y); }},
    {"gyro.z", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.angular_velocity.z); }},

    // Linear acceleration
    {"accel.x", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.linear_acceleration.x); }},
    {"accel.y", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.linear_acceleration.y); }},
    {"accel.z", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.linear_acceleration.z); }},
};
```

可扩展的 path 约定如下：

- 姿态角：`orientation.roll|pitch|yaw`
- 角速度：`gyro.x|y|z`
- 线加速度：`accel.x|y|z`

------

# 2) 修改：`telemetry_streamer_node.hpp` 增加 IMU 支持

把下面内容加入你的头文件。如果你直接用我这份“合并版”，替换即可。

**`include/telemetry_streamer_odom/telemetry_streamer_node.hpp`**

```
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
```

------

# 3) 修改：`telemetry_streamer_node.cpp` 增加订阅与提取

**`src/telemetry_streamer_node.cpp`** 关键改动如下。你可用下面整段直接覆盖你的 cpp（包含之前的 Odom 修改与 IMU 新增）：

```
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
```

------

# 4) 修改：注册 IMU Extractor

**`src/topic_extractors.cpp`** 更新为：

```
#include "telemetry_streamer_odom/topic_extractors.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"

std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s);
std::vector<float> extract_imu (TelemetryStreamerNode* self, const StreamSpec& s); // ✅

const std::unordered_map<std::string, ExtractFunc> TOPIC_EXTRACTOR_MAP = {
    {"/odom", extract_odom},
    {"/imu",  extract_imu},   // ✅ 已接入
    // {"/gps",  extract_gps}, // 未来接入
};
```

------

# 5) 修改：CMakeLists.txt 增加 sensor_msgs

把 `sensor_msgs` 加到 `find_package` 与依赖里。

**`CMakeLists.txt`** 相关段：

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # ✅ 新增

# ...

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs            # ✅ 新增
)
```

------

# 6) XML 配置示例

```
<streams>

  <!-- ODOM -->
  <stream topic="/odom" enable="true" period_ms="50" phase_ms="0" id="1" name="odom_stream" n_floats="5">
    <mapping kind="float" index="0" path="pose.x"/>
    <mapping kind="float" index="1" path="pose.y"/>
    <mapping kind="float" index="2" path="orientation.yaw"/>
    <mapping kind="float" index="3" path="twist.linear.x"/>
    <mapping kind="float" index="4" path="twist.angular.z"/>
  </stream>

  <!-- IMU -->
  <stream topic="/imu" enable="true" period_ms="20" phase_ms="0" id="2" name="imu_stream" n_floats="6">
    <mapping kind="float" index="0" path="orientation.roll"/>
    <mapping kind="float" index="1" path="orientation.pitch"/>
    <mapping kind="float" index="2" path="orientation.yaw"/>
    <mapping kind="float" index="3" path="gyro.x"/>
    <mapping kind="float" index="4" path="gyro.y"/>
    <mapping kind="float" index="5" path="accel.z"/>
  </stream>

</streams>
```

------

## 小结与建议

- 你现在已拥有 **多话题、多字段、按类型分派** 的完整链路。
- Odom 与 IMU 都通过查表扩展字段，不再修改核心逻辑。
- 后续如果要支持 `/gps`，只需要：
  1. 新建 `gps_fields.hpp`
  2. 在 `telemetry_streamer_node.hpp/.cpp` 加 `GpsCache`、订阅与 `extract_gps_floats`
  3. 在 `topic_extractors.cpp` 注册 `{"/gps", extract_gps}`

需要我顺手把 **`package.xml`** 模板也给你一份吗？你就能一键 build。