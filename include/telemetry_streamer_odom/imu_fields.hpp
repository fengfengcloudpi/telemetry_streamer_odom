#pragma once
#include <functional>
#include <string>
#include <unordered_map>
#include <sensor_msgs/msg/imu.hpp>

namespace telemetry_streamer_odom {

// 直接读取的 Getter 类型（不做任何额外计算）
using ImuFloatGetter = std::function<float(const sensor_msgs::msg::Imu&)>;
using ImuIntGetter   = std::function<int  (const sensor_msgs::msg::Imu&)>;

// 直接在头文件里定义，C++14 支持
inline const std::unordered_map<std::string, ImuFloatGetter> IMU_FLOAT_GETTERS = {
    // 角速度
    {"angular_velocity.x", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.angular_velocity.x); }},
    {"angular_velocity.y", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.angular_velocity.y); }},
    {"angular_velocity.z", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.angular_velocity.z); }},
    // 线加速度
    {"linear_acceleration.x", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.linear_acceleration.x); }},
    {"linear_acceleration.y", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.linear_acceleration.y); }},
    {"linear_acceleration.z", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.linear_acceleration.z); }},
    // 四元数（直接分量）
    {"orientation.x", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.orientation.x); }},
    {"orientation.y", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.orientation.y); }},
    {"orientation.z", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.orientation.z); }},
    {"orientation.w", [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.orientation.w); }},
    // 如需协方差，可按需解注释并在 XML 中使用相同 key
    // {"angular_velocity_cov[0]",  [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.angular_velocity_covariance[0]); }},
    // {"linear_acceleration_cov[0]",[](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.linear_acceleration_covariance[0]); }},
    // {"orientation_cov[0]",       [](const sensor_msgs::msg::Imu& m){ return static_cast<float>(m.orientation_covariance[0]); }},
};

inline const std::unordered_map<std::string, ImuIntGetter> IMU_INT_GETTERS = {
    {"header.seq", [](const sensor_msgs::msg::Imu& m){ return static_cast<int>(m.header.seq); }},
};

} // namespace telemetry_streamer_odom
