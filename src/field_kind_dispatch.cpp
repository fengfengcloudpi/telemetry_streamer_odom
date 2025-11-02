#include "telemetry_streamer_odom/field_kind_dispatch.hpp"
#include "telemetry_streamer_odom/field_resolver.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace telemetry_streamer_odom {

static void handle_float(TelemetryStreamerNode* self, const StreamSpec& s,
                         const StreamMapEntry& m, StreamBuffers& bufs) {
  if (m.index < 0 || m.index >= static_cast<int>(bufs.floats.size())) return;
  auto acc = resolve_float_accessor(s.topic, m.path);
  if (!acc) {
    RCLCPP_WARN(self->get_logger(), "No float accessor for topic=%s path=%s",
                s.topic.c_str(), m.path.c_str());
    return;
  }
  bufs.floats[m.index] = (*acc)(self);
}

static void handle_int(TelemetryStreamerNode* self, const StreamSpec& s,
                       const StreamMapEntry& m, StreamBuffers& bufs) {
  if (m.index < 0 || m.index >= static_cast<int>(bufs.ints.size())) return;
  auto acc = resolve_int_accessor(s.topic, m.path);
  if (!acc) {
    RCLCPP_WARN(self->get_logger(), "No int accessor for topic=%s path=%s",
                s.topic.c_str(), m.path.c_str());
    return;
  }
  bufs.ints[m.index] = (*acc)(self);
}

const std::unordered_map<std::string, FieldKindHandler> FIELD_KIND_MAP = {
  {"float", handle_float},
  {"int",   handle_int},
  // 以后需要可扩: {"bool", handle_bool}, {"double", ...}
};

} // namespace telemetry_streamer_odom
