#pragma once
#include <unordered_map>
#include <functional>
#include <string>
#include <vector>

struct StreamSpec;
struct StreamMapEntry;
class TelemetryStreamerNode;

// kind = "float" | "int" | "bool"
using FieldKindHandler = std::function<void(
    TelemetryStreamerNode* self,
    const StreamSpec& spec,
    const StreamMapEntry& m,
    std::vector<float>& out
)>;

extern const std::unordered_map<std::string, FieldKindHandler> FIELD_KIND_MAP;
