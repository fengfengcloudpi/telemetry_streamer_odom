#include "telemetry_streamer_odom/field_kind_dispatch.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
#include "telemetry_streamer_odom/odom_fields.hpp"
#include <cmath>

// float 类型字段提取
static void handle_float(
    TelemetryStreamerNode* self,
    const StreamSpec& s,
    const FieldMapping& m,
    std::vector<float>& out)
{
    if (m.index < 0 || m.index >= s.n_floats) return;

    auto it = ODOM_FIELD_MAP.find(m.path);
    if (it == ODOM_FIELD_MAP.end()) {
        RCLCPP_WARN(self->get_logger(), "Unknown float field path: %s", m.path.c_str());
        return;
    }

    auto msg_copy = self->copyOdom();  // 从缓存获取 odom

    const auto &q = msg_copy.pose.pose.orientation;
    float yaw = std::atan2(
        2.0f * (q.w*q.z + q.x*q.y),
        1.0f - 2.0f * (q.y*q.y + q.z*q.z)
    );

    out[m.index] = it->second(msg_copy, yaw);
}

// 可扩展 int / bool 类型
static void handle_int(...) {}
static void handle_bool(...) {}

const std::unordered_map<std::string, FieldKindHandler> FIELD_KIND_MAP = {
    {"float", handle_float},
    // {"int",   handle_int},
    // {"bool",  handle_bool},
};
