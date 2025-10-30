cmake_minimum_required(VERSION 3.8)
project(telemetry_streamer_odom)

### 1. 基础依赖（ROS2）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 👇 注意：这里我们并没有使用 find_package(tinyxml2 REQUIRED)
# 因为你的环境没有 tinyxml2Config.cmake
# 我们将直接手动链接系统库 libtinyxml2.so，而不是依赖 tinyxml2::tinyxml2

### 2. 可执行文件：把所有 .cpp 都放进来
add_executable(telemetry_streamer_odom_node
    src/main.cpp
    src/config.cpp
    src/telemetry_streamer_node.cpp
    src/field_kind_dispatch.cpp
    src/topic_extractors.cpp
)

### 3. ament 依赖（除了 tinyxml2 以外）
ament_target_dependencies(telemetry_streamer_odom_node
    rclcpp
    nav_msgs
    ament_index_cpp
)

### 4. 头文件搜索路径
target_include_directories(telemetry_streamer_odom_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

### 5. 手动链接系统 tinyxml2 库
# 这一步假设你已经在系统里安装了 libtinyxml2-dev
# - 头文件 tinyxml2.h 通常在 /usr/include
# - 库文件 libtinyxml2.so 通常在 /usr/lib/x86_64-linux-gnu
#
# 我们不使用 tinyxml2::tinyxml2 这个 target 名，而是直接告诉链接器 -ltinyxml2
target_link_libraries(telemetry_streamer_odom_node
    tinyxml2
)

### 6. 安装可执行文件（供 ros2 run 调用）
install(TARGETS telemetry_streamer_odom_node
    DESTINATION lib/${PROJECT_NAME}
)

### 7. 安装头文件（让别的包也能 include 我们的头）
install(DIRECTORY include/
    DESTINATION include/
)

### 8. 安装运行时配置（例如 XML 配置文件）
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

### 9. 完成 ament 包导出
ament_package()
