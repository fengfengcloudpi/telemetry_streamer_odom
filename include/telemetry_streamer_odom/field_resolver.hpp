#pragma once
#include <functional>
#include <optional>
#include <string>

namespace telemetry_streamer_odom {

class TelemetryStreamerNode;

using FloatAccessor = std::function<float(TelemetryStreamerNode*)>;
using IntAccessor   = std::function<int  (TelemetryStreamerNode*)>;

std::optional<FloatAccessor>
resolve_float_accessor(const std::string& topic, const std::string& path);

std::optional<IntAccessor>
resolve_int_accessor(const std::string& topic, const std::string& path);

} // namespace telemetry_streamer_odom
