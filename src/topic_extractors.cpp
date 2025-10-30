#include "telemetry_streamer_odom/topic_extractors.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"

// 前向声明
std::vector<float> extract_odom(TelemetryStreamerNode* self, const StreamSpec& s);

// 未来可扩展
std::vector<float> extract_imu(TelemetryStreamerNode*, const StreamSpec&) { return {}; }
std::vector<float> extract_gps(TelemetryStreamerNode*, const StreamSpec&) { return {}; }

const std::unordered_map<std::string, ExtractFunc> TOPIC_EXTRACTOR_MAP = {
    {"/odom", extract_odom},
    {"/imu",  extract_imu},
    {"/gps",  extract_gps},
};
