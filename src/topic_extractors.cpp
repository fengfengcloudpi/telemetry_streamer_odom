#include "telemetry_streamer_odom/topic_extractors.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"

std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s);
std::vector<float> extract_imu (TelemetryStreamerNode* self, const StreamSpec& s); // ✅

const std::unordered_map<std::string, ExtractFunc> TOPIC_EXTRACTOR_MAP = {
    {"/odom", extract_odom},
    {"/imu",  extract_imu},   // ✅ 已接入
    // {"/gps",  extract_gps}, // 未来接入
};
