cmake_minimum_required(VERSION 3.5)
project(telemetry_streamer_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# -------- TinyXML2 robust detection --------
# 1) ROS vendor（推荐在ROS环境）
find_package(tinyxml2_vendor QUIET)

# 2) 若可能，直接找上游 CONFIG 包
find_package(tinyxml2 QUIET CONFIG)

set(TINYXML2_TARGET "")
set(TINYXML2_INCLUDE_DIRS "")
set(TINYXML2_LIBS "")

if(TARGET tinyxml2::tinyxml2)
  # 常见于 tinyxml2_vendor 或部分发行版
  set(TINYXML2_TARGET tinyxml2::tinyxml2)
elseif(TARGET TinyXML2::TinyXML2)
  # 常见于系统包 libtinyxml2-dev
  set(TINYXML2_TARGET TinyXML2::TinyXML2)
endif()

# 3) 如果没有 CMake target，走 pkg-config 或手工查找
if(NOT TINYXML2_TARGET)
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(TINYXML2_PC tinyxml2 QUIET)
  endif()

  if(TINYXML2_PC_FOUND)
    set(TINYXML2_INCLUDE_DIRS ${TINYXML2_PC_INCLUDE_DIRS})
    set(TINYXML2_LIBS ${TINYXML2_PC_LINK_LIBRARIES})
  else()
    # 最后兜底：find_library / find_path
    find_library(TINYXML2_LIB NAMES tinyxml2)
    find_path(TINYXML2_INC NAMES tinyxml2.h PATH_SUFFIXES tinyxml2)
    if(TINYXML2_LIB AND TINYXML2_INC)
      set(TINYXML2_INCLUDE_DIRS ${TINYXML2_INC})
      set(TINYXML2_LIBS ${TINYXML2_LIB})
    else()
      message(FATAL_ERROR
        "Could not find TinyXML2 via vendor, CONFIG, pkg-config, or find_library.\n"
        "Try:\n"
        "  - rosdep install --from-paths src -i -y (to get ros-humble-tinyxml2-vendor)\n"
        "  - or: sudo apt install libtinyxml2-dev\n"
        "Then clean build: rm -rf build install log && colcon build"
      )
    endif()
  endif()
endif()

# -------- Library --------
add_library(${PROJECT_NAME}_lib
  src/config.cpp
  src/telemetry_streamer_node.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
  ${TINYXML2_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  ament_index_cpp
  # 注意：不要把 tinyxml2 放这里；它不是 ament 包
)

# 链接 tinyxml2（优先用 target，否则用库路径）
if(TINYXML2_TARGET)
  target_link_libraries(${PROJECT_NAME}_lib ${TINYXML2_TARGET})
else()
  target_link_libraries(${PROJECT_NAME}_lib ${TINYXML2_LIBS})
endif()

# -------- Executable --------
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${TINYXML2_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)
if(TINYXML2_TARGET)
  target_link_libraries(${PROJECT_NAME}_node ${TINYXML2_TARGET})
else()
  target_link_libraries(${PROJECT_NAME}_node ${TINYXML2_LIBS})
endif()

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  ament_index_cpp
)

# -------- Install --------
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
