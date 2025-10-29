#ifndef TELEMETRY_STREAMER_NODE_HPP
#define TELEMETRY_STREAMER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "telemetry_streamer_odom/config.hpp"

class TelemetryStreamerNode : public rclcpp::Node {
public:
    TelemetryStreamerNode(const FullConfig& cfg);
    ~TelemetryStreamerNode() override;
    void onTick();

private:
    FullConfig cfg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // TELEMETRY_STREAMER_NODE_HPP
