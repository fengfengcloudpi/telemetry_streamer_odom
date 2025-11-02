#pragma once
#include <string>
#include <vector>
#include <cstdint>

struct StreamMapEntry {
    std::string kind;   // "float"
    int index;          // 槽位下标
    std::string path;   // 例如 "pose.pose.position.x"
};

struct StreamSpec {
    uint16_t id;
    std::string name;
    std::string topic;
    std::string type;
    int period_ms;
    int phase_ms;
    bool enable;

    int n_ints;
    int n_floats;
    int n_strs;

    std::vector<StreamMapEntry> mappings;
};

struct NetworkSpec {
    std::string bind_ip;
    std::string dest_ip;
    uint16_t port;
    int base_tick_ms;
    int mtu;
};

struct FullConfig {
    NetworkSpec net;
    std::vector<StreamSpec> streams;
};

struct StreamBuffers {
  std::vector<float> floats;
  std::vector<int>   ints;
};

bool load_config_from_xml(const std::string& path, FullConfig& out_cfg);
