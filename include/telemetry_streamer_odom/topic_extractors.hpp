#pragma once
#include <unordered_map>
#include <functional>
#include <string>
#include <vector>

struct StreamSpec;
class TelemetryStreamerNode;

// 话题 → 提取函数
using ExtractFunc = std::function<std::vector<float>(TelemetryStreamerNode*, const StreamSpec&)>;

extern const std::unordered_map<std::string, ExtractFunc> TOPIC_EXTRACTOR_MAP;
