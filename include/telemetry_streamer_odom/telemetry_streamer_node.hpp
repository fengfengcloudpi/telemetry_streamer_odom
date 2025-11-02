// telemetry_streamer_node.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <utility>   // pair
#include <cmath>     // floor/ceil/llround
#include "telemetry_streamer_odom/config.hpp"
#include "telemetry_streamer_odom/field_kind_dispatch.hpp"

namespace telemetry_streamer_odom {

// 若仓库已有类似 RuntimeEntry/字段名，请保留原名，这里仅示例
struct RuntimeEntry {
  const StreamSpec* spec{nullptr};
  uint32_t step{1};        // 基于 base_tick_ms_ 的步长
  uint32_t offset{0};      // 相位（0..step-1）
  uint16_t template_ver{1};
};

class TelemetryStreamerNode : public rclcpp::Node {
public:
  explicit TelemetryStreamerNode(const FullConfig& cfg);

  void set_base_tick_ms(uint32_t ms);

  nav_msgs::msg::Odometry copyOdom();
  sensor_msgs::msg::Imu   copyImu();

  StreamBuffers extract_buffers(const StreamSpec& s);
  void publish_stream(const StreamSpec& spec, const StreamBuffers& bufs);

private:
  void on_tick();
  void build_runtimes_();

  // —— 扫描周期折算（与仓库“扫描周期”一致的对齐点）——
  enum class RoundingPolicy { Floor, Ceil, Nearest };
  std::pair<uint32_t,uint32_t> compute_step_offset_scan_(
      uint32_t period_ms,   // 每路扫描周期（优先 scan_period_ms）
      uint32_t phase_ms,    // 每路扫描相位（优先 scan_phase_ms）
      uint32_t base_tick_ms,// 扫描基准拍
      RoundingPolicy policy // 取整策略：默认 Floor = 偏快不偏慢（与仓库原习惯一致）
  ) const;

  static uint32_t clamp_step_min1_(uint32_t v) { return v == 0 ? 1u : v; }
  static uint32_t normalize_phase_(uint32_t off, uint32_t step) { return (step==0)?0:(off%step); }

private:
  FullConfig config_;
  std::vector<RuntimeEntry> runtimes_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t base_tick_ms_{50};    // 扫描基准拍（默认 20Hz）
  uint64_t tick_count_{0};
  uint32_t seq_counter_{0};

  int sock_fd_{-1};
  sockaddr_in dest_addr_{};

  std::mutex odom_mutex_;
  std::mutex imu_mutex_;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::Imu   last_imu_;
};

} // namespace telemetry_streamer_odom
