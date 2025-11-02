#include "telemetry_streamer_odom/field_resolver.hpp"
#include "telemetry_streamer_odom/topic_registry.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"

namespace telemetry_streamer_odom {

std::optional<FloatAccessor>
resolve_float_accessor(const std::string& topic, const std::string& path) {
  auto it = TOPIC_REGISTRY.find(topic);
  if (it == TOPIC_REGISTRY.end()) return std::nullopt;
  const TopicOps& ops = it->second;
  if (!ops.has_float_path(path)) return std::nullopt;
  return [path, &ops](TelemetryStreamerNode* self) -> float {
    return ops.read_float(self, path);
  };
}

std::optional<IntAccessor>
resolve_int_accessor(const std::string& topic, const std::string& path) {
  auto it = TOPIC_REGISTRY.find(topic);
  if (it == TOPIC_REGISTRY.end()) return std::nullopt;
  const TopicOps& ops = it->second;
  if (!ops.has_int_path(path)) return std::nullopt;
  return [path, &ops](TelemetryStreamerNode* self) -> int {
    return ops.read_int(self, path);
  };
}

} // namespace telemetry_streamer_odom
