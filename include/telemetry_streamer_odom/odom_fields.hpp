#pragma once
#include <unordered_map>
#include <functional>
#include <string>
#include <nav_msgs/msg/odometry.hpp>

using OdomMsg = nav_msgs::msg::Odometry;

// Odom 话题的字段映射表：path -> getter(msg, yaw)
static const std::unordered_map<std::string, std::function<float(const OdomMsg&, float)>> ODOM_FIELD_MAP = {
    // --- 位置 ---
    {"pose.x", [](const OdomMsg& m, float){ return m.pose.pose.position.x; }},
    {"pose.y", [](const OdomMsg& m, float){ return m.pose.pose.position.y; }},
    {"pose.z", [](const OdomMsg& m, float){ return m.pose.pose.position.z; }},

    // --- 朝向 ---
    {"orientation.yaw", [](const OdomMsg&, float yaw){ return yaw; }},

    // --- 线速度 ---
    {"twist.linear.x", [](const OdomMsg& m, float){ return m.twist.twist.linear.x; }},
    {"twist.linear.y", [](const OdomMsg& m, float){ return m.twist.twist.linear.y; }},
    {"twist.linear.z", [](const OdomMsg& m, float){ return m.twist.twist.linear.z; }},

    // --- 角速度 ---
    {"twist.angular.x", [](const OdomMsg& m, float){ return m.twist.twist.angular.x; }},
    {"twist.angular.y", [](const OdomMsg& m, float){ return m.twist.twist.angular.y; }},
    {"twist.angular.z", [](const OdomMsg& m, float){ return m.twist.twist.angular.z; }},
};
