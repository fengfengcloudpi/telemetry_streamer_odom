#include "telemetry_streamer_odom/topic_registry.hpp"
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
#include "telemetry_streamer_odom/odom_fields.hpp"
#include "telemetry_streamer_odom/imu_fields.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace telemetry_streamer_odom {

using Node = TelemetryStreamerNode;

template<typename MsgT,
         typename FloatGetterMap,
         typename IntGetterMap,
         typename CopyFn>
static TopicOps make_topic_ops(const FloatGetterMap& fmap,
                               const IntGetterMap&   imap,
                               CopyFn copy_msg) {
  TopicOps ops;
  ops.has_float_path = [&fmap](const std::string& path) {
    return fmap.find(path) != fmap.end();
  };
  ops.has_int_path = [&imap](const std::string& path) {
    return imap.find(path) != imap.end();
  };
  ops.read_float = [&fmap, copy_msg](Node* self, const std::string& path) -> float {
    auto it = fmap.find(path);
    if (it == fmap.end()) return 0.0f;
    MsgT msg = copy_msg(self);
    return it->second(msg);
  };
  ops.read_int = [&imap, copy_msg](Node* self, const std::string& path) -> int {
    auto it = imap.find(path);
    if (it == imap.end()) return 0;
    MsgT msg = copy_msg(self);
    return it->second(msg);
  };
  return ops;
}

const std::unordered_map<std::string, TopicOps> TOPIC_REGISTRY = {
  {
    "/odom",
    make_topic_ops<nav_msgs::msg::Odometry>(
      ODOM_FLOAT_GETTERS,
      ODOM_INT_GETTERS,
      [](Node* self){ return self->copyOdom(); }
    )
  },
  {
    "/imu",
    make_topic_ops<sensor_msgs::msg::Imu>(
      IMU_FLOAT_GETTERS,
      IMU_INT_GETTERS,
      [](Node* self){ return self->copyImu(); }
    )
  },
  // 将来新增 topic，只需在此添加一条
};

} // namespace telemetry_streamer_odom
