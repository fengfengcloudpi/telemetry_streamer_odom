#pragma once
#include <unordered_map>
#include <functional>
#include <string>
#include <sensor_msgs/msg/imu.hpp>

using ImuMsg = sensor_msgs::msg::Imu;

// 说明：roll/pitch/yaw 由外部提前计算后传入 getter 的第二个参数（此处复用 yaw 参数位）
// 约定：第二个参数传入的是 yaw，roll/pitch 在回调闭包内捕获，见 extract_imu_floats 实现
struct ImuAngles {
  float roll{0.f}, pitch{0.f}, yaw{0.f};
};

static const std::unordered_map<std::string, std::function<float(const ImuMsg&, const ImuAngles&)>> IMU_FIELD_MAP = {
    // Orientation (RPY from quaternion)
    {"orientation.roll",  [](const ImuMsg&, const ImuAngles& ang){ return ang.roll; }},
    {"orientation.pitch", [](const ImuMsg&, const ImuAngles& ang){ return ang.pitch; }},
    {"orientation.yaw",   [](const ImuMsg&, const ImuAngles& ang){ return ang.yaw; }},

    // Angular velocity
    {"gyro.x", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.angular_velocity.x); }},
    {"gyro.y", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.angular_velocity.y); }},
    {"gyro.z", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.angular_velocity.z); }},

    // Linear acceleration
    {"accel.x", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.linear_acceleration.x); }},
    {"accel.y", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.linear_acceleration.y); }},
    {"accel.z", [](const ImuMsg& m, const ImuAngles&){ return static_cast<float>(m.linear_acceleration.z); }},
};
