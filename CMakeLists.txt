cmake_minimum_required(VERSION 3.8)
project(telemetry_streamer_odom)

# 1. 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tinyxml2 REQUIRED) 
# 如果你的 build_stream_frame(...) 在单独的 protocol.cpp 里，
# 也要把它加进下面的源文件列表
# 同理，如果你还有别的 .cpp 文件（比如 utils.cpp），也要一起列出来

add_executable(telemetry_streamer_odom_node
    src/main.cpp
    src/config.cpp
    src/telemetry_streamer_node.cpp
    # 如果你有这个文件就保留；如果没有就删掉这行
)

# 2. 让这个可执行文件链接正确的依赖
ament_target_dependencies(telemetry_streamer_odom_node
    rclcpp
    nav_msgs
    ament_index_cpp
    tinyxml2               
)

# 3. 头文件路径 (include/ 目录里是我们的 .hpp)
target_include_directories(telemetry_streamer_odom_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(telemetry_streamer_odom_node
    tinyxml2               #有的环境下这句是必须的
)
# 4. 安装可执行文件：colcon build 之后可以 ros2 run
install(TARGETS telemetry_streamer_odom_node
    DESTINATION lib/${PROJECT_NAME}
)

# 5. 安装头文件（让别的包也能 include/telemetry_streamer_odom/...）
install(DIRECTORY include/
    DESTINATION include/
)

# 6. 安装配置文件 (如果你有 config/*.xml 之类的运行时文件)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
