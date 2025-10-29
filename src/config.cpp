#include "telemetry_streamer_odom/config.hpp"
#include <tinyxml2.h>
#include <iostream>

using namespace tinyxml2;

bool load_config_from_xml(const std::string& path, FullConfig& out_cfg) {
    XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != XML_SUCCESS) {
        std::cerr << "Failed to load XML: " << path << "\n";
        return false;
    }

    auto* root = doc.FirstChildElement("streams");
    if (!root) {
        std::cerr << "No <streams>\n";
        return false;
    }

    // network
    auto* net = root->FirstChildElement("network");
    if (!net) {
        std::cerr << "No <network>\n";
        return false;
    }
    NetworkSpec ns{};
    ns.bind_ip      = net->Attribute("bind_ip")      ? net->Attribute("bind_ip")      : "0.0.0.0";
    ns.dest_ip      = net->Attribute("dest_ip")      ? net->Attribute("dest_ip")      : "127.0.0.1";
    ns.port         = static_cast<uint16_t>( net->UnsignedAttribute("port", 50200) );
    ns.base_tick_ms = net->IntAttribute("base_tick_ms", 10);
    ns.mtu          = net->IntAttribute("mtu", 1200);
    out_cfg.net = ns;

    // streams
    for (auto* se = root->FirstChildElement("stream"); se; se = se->NextSiblingElement("stream")) {
        StreamSpec ss{};

        ss.id        = static_cast<uint16_t>( se->UnsignedAttribute("id", 0) );
        ss.name      = se->Attribute("name")  ? se->Attribute("name")  : "";
        ss.topic     = se->Attribute("topic") ? se->Attribute("topic") : "";
        ss.type      = se->Attribute("type")  ? se->Attribute("type")  : "";
        ss.period_ms = se->IntAttribute("period_ms", 100);
        ss.phase_ms  = se->IntAttribute("phase_ms", 0);
        ss.enable    = (std::string(se->Attribute("enable")?se->Attribute("enable"):"true") == "true");

        auto* slots = se->FirstChildElement("slots");
        if (slots) {
            ss.n_ints   = slots->IntAttribute("ints", 0);
            ss.n_floats = slots->IntAttribute("floats", 0);
            ss.n_strs   = slots->IntAttribute("strings", 0);
        } else {
            ss.n_ints = ss.n_floats = ss.n_strs = 0;
        }

        for (auto* me = se->FirstChildElement("map"); me; me = me->NextSiblingElement("map")) {
            StreamMapEntry m{};
            m.kind  = me->Attribute("kind")  ? me->Attribute("kind")  : "";
            m.index = me->IntAttribute("index", 0);
            m.path  = me->Attribute("path")  ? me->Attribute("path")  : "";
            ss.mappings.push_back(m);
        }

        out_cfg.streams.push_back(ss);
    }

    return true;
}
