cmake_minimum_required(VERSION 3.5)
project(telemetry_streamer_odom)

# 默认编译设置
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

add_library(telemetry_streamer_odom_lib
    src/config.cpp
    src/telemetry_streamer_node.cpp
)
target_include_directories(telemetry_streamer_odom_lib PUBLIC
    include
)
ament_target_dependencies(telemetry_streamer_odom_lib
    rclcpp nav_msgs tinyxml2
)

add_executable(telemetry_streamer_odom_node
    src/main.cpp
)
target_link_libraries(telemetry_streamer_odom_node
    telemetry_streamer_odom_lib
)
ament_target_dependencies(telemetry_streamer_odom_node
    rclcpp nav_msgs tinyxml2
)

install(TARGETS
    telemetry_streamer_odom_lib
    telemetry_streamer_odom_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
