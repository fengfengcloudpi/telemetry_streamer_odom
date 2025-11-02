#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
#include "telemetry_streamer_odom/field_kind_dispatch.hpp"
#include <algorithm>
#include <chrono>

extern StreamPacket build_stream_frame(
    uint32_t stream_id, uint16_t template_ver,
    uint64_t ts_usec, uint32_t seq,
    const std::vector<float>& floats);

namespace telemetry_streamer_odom {

TelemetryStreamerNode::TelemetryStreamerNode(const FullConfig& cfg)
: rclcpp::Node("telemetry_streamer_odom_node"),
  config_(cfg)
{
  rclcpp::QoS qos(10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        std::lock_guard<std::mutex> lk(odom_mutex_);
        last_odom_ = *msg;
      });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg){
        std::lock_guard<std::mutex> lk(imu_mutex_);
        last_imu_ = *msg;
      });

  build_runtimes_();

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(base_tick_ms_),
      [this]() { this->on_tick(); });

  RCLCPP_INFO(this->get_logger(),
              "node started. base_tick=%u ms, runtime_entries=%zu",
              base_tick_ms_, runtimes_.size());
}

void TelemetryStreamerNode::set_base_tick_ms(uint32_t ms) {
  base_tick_ms_ = std::max<uint32_t>(1, ms);
  // 如你仓库原逻辑会重建 timer，这里可同步做重建
}

// —— 与仓库“扫描周期”一致：period/phase 折算到基准拍 ——
// 默认使用 Floor（偏快）策略；如仓库原本用的就是 Floor，这里行为等同
std::pair<uint32_t,uint32_t>
TelemetryStreamerNode::compute_step_offset_scan_(uint32_t period_ms,
                                                 uint32_t phase_ms,
                                                 uint32_t base_tick_ms,
                                                 RoundingPolicy policy) const
{
  if (base_tick_ms == 0) base_tick_ms = 1;

  const double step_f = static_cast<double>(period_ms) / static_cast<double>(base_tick_ms);
  const double off_f  = static_cast<double>(phase_ms)  / static_cast<double>(base_tick_ms);

  auto roundP = [&](double x)->uint32_t {
    switch (policy) {
      case RoundingPolicy::Floor:   return static_cast<uint32_t>(std::floor(x));
      case RoundingPolicy::Ceil:    return static_cast<uint32_t>(std::ceil(x));
      case RoundingPolicy::Nearest: return static_cast<uint32_t>(std::llround(x));
    }
    return static_cast<uint32_t>(std::floor(x));
  };

  uint32_t step_raw = roundP(step_f);
  uint32_t off_raw  = roundP(off_f);

  uint32_t step = clamp_step_min1_(step_raw);       // 保证 step >= 1
  uint32_t off  = normalize_phase_(off_raw, step);  // 统一规整相位到 [0, step-1]

  // 保留与仓库一致的诊断风格（可选）
  if (policy == RoundingPolicy::Floor) {
    if (step_raw == 0 && period_ms > 0 && period_ms < base_tick_ms) {
      RCLCPP_WARN(get_logger(),
        "scan period %u ms < base tick %u ms; floor->0 clamped to 1 (actual faster).",
        period_ms, base_tick_ms);
    }
  }
  return {step, off};
}

void TelemetryStreamerNode::build_runtimes_() {
  runtimes_.clear();
  runtimes_.reserve(config_.streams.size());

  // —— 与仓库“扫描周期”一致：优先读取 scan_* 字段 —— 
  // 若 StreamSpec 没有这些字段，请在解析 XML 时填充；下面用了防御式回退
  for (const auto& s : config_.streams) {
    RuntimeEntry e;
    e.spec = &s;

    // 1) 取 period/phase：优先 scan_*，否则 period_ms/phase_ms，否则 rate_hz 推导
    uint32_t period_ms = 0;
    uint32_t phase_ms  = 0;

    // 以下成员名用“存在则用”的写法（若你结构体名不同，请改成你的字段名）
    if (s.scan_period_ms > 0) {
      period_ms = static_cast<uint32_t>(s.scan_period_ms);
    } else if (s.period_ms > 0) {
      period_ms = static_cast<uint32_t>(s.period_ms);
    } else if (s.rate_hz > 0.0) {
      period_ms = static_cast<uint32_t>(std::llround(1000.0 / s.rate_hz));
    } else {
      period_ms = base_tick_ms_; // 没填则默认每拍发送
    }

    if (s.scan_phase_ms > 0) {
      phase_ms = static_cast<uint32_t>(s.scan_phase_ms);
    } else if (s.phase_ms > 0) {
      phase_ms = static_cast<uint32_t>(s.phase_ms);
    } else {
      phase_ms = 0;
    }

    // 2) 折算到基准拍网格（保持仓库中“扫描对齐：向下取整”的语义）
    constexpr RoundingPolicy kPolicy =
    #ifdef STREAM_SCAN_ROUND_NEAREST
      RoundingPolicy::Nearest;
    #elif defined(STREAM_SCAN_ROUND_CEIL)
      RoundingPolicy::Ceil;
    #else
      RoundingPolicy::Floor; // 默认与仓库原习惯一致
    #endif

    auto so = compute_step_offset_scan_(period_ms, phase_ms, base_tick_ms_, kPolicy);
    e.step   = clamp_step_min1_(so.first);
    e.offset = normalize_phase_(so.second, e.step);

    // 3) 其余保持原样
    e.template_ver = s.template_ver; // 若无该字段，可固定 1
    runtimes_.push_back(e);
  }
}

void TelemetryStreamerNode::on_tick()
{
  tick_count_++;

  const auto now = this->now();
  uint64_t ts_usec =
      static_cast<uint64_t>(now.seconds()) * 1000000ULL +
      static_cast<uint64_t>(now.nanoseconds() % 1000000000ULL) / 1000ULL;

  for (auto &rt : runtimes_) {
    const auto &s = *rt.spec;

    // —— 用扫描的步长/相位进行触发判定（与仓库一致）——
    if ((rt.step == 0) || ((tick_count_ % rt.step) != rt.offset)) continue;

    // 统一抽取：topic+path 直取（零计算）
    StreamBuffers bufs = extract_buffers(s);

    // 旧帧格式只带 floats，沿用
    auto pkt = build_stream_frame(
        s.id, rt.template_ver, ts_usec, seq_counter_++, bufs.floats);

    ::sendto(sock_fd_,
             pkt.bytes.data(), pkt.bytes.size(),
             0, reinterpret_cast<sockaddr*>(&dest_addr_),
             sizeof(dest_addr_));
  }
}

} // namespace telemetry_streamer_odom
