#pragma once
#include <functional>
#include <string>
#include <unordered_map>
#include <nav_msgs/msg/odometry.hpp>

namespace telemetry_streamer_odom {

// 直接读取的 Getter 类型（不做任何额外计算）
using OdomFloatGetter = std::function<float(const nav_msgs::msg::Odometry&)>;
using OdomIntGetter   = std::function<int  (const nav_msgs::msg::Odometry&)>;

// 直接在头文件里定义，C++14 支持
inline const std::unordered_map<std::string, OdomFloatGetter> ODOM_FLOAT_GETTERS = {
    // 位置
    {"pose.x", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.position.x); }},
    {"pose.y", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.position.y); }},
    {"pose.z", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.position.z); }},

    // 四元数（直接分量）
    {"orientation.x", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.orientation.x); }},
    {"orientation.y", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.orientation.y); }},
    {"orientation.z", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.orientation.z); }},
    {"orientation.w", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.pose.pose.orientation.w); }},

    // 线速度
    {"twist.linear.x",  [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.twist.twist.linear.x); }},
    {"twist.linear.y",  [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.twist.twist.linear.y); }},
    {"twist.linear.z",  [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.twist.twist.linear.z); }},

    // 角速度
    {"twist.angular.x", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.twist.twist.angular.x); }},
    {"twist.angular.y", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.twist.twist.angular.y); }},
    {"twist.angular.z", [](const nav_msgs::msg::Odometry& m){ return static_cast<float>(m.twist.twist.angular.z); }},
};

inline const std::unordered_map<std::string, OdomIntGetter> ODOM_INT_GETTERS = {
    {"header.seq", [](const nav_msgs::msg::Odometry& m){ return static_cast<int>(m.header.seq); }},
};

} // namespace telemetry_streamer_odom
