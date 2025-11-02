#pragma once
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

namespace telemetry_streamer_odom {

class TelemetryStreamerNode;

struct TopicOps {
  std::function<bool(const std::string&)> has_float_path;
  std::function<bool(const std::string&)> has_int_path;
  std::function<float(TelemetryStreamerNode*, const std::string&)> read_float;
  std::function<int  (TelemetryStreamerNode*, const std::string&)> read_int;
};

extern const std::unordered_map<std::string, TopicOps> TOPIC_REGISTRY;

} // namespace telemetry_streamer_odom
