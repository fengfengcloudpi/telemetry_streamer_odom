#pragma once
#include <unordered_map>
#include <string>
#include <vector>
#include "telemetry_streamer_odom/config.hpp" // 提供 StreamSpec / StreamMapEntry

namespace telemetry_streamer_odom {

class TelemetryStreamerNode;


using FieldKindHandler =
  void(*)(TelemetryStreamerNode*, const StreamSpec&, const StreamMapEntry&, StreamBuffers&);

extern const std::unordered_map<std::string, FieldKindHandler> FIELD_KIND_MAP;

} // namespace telemetry_streamer_odom
