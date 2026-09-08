cmake_minimum_required(VERSION 3.16)
project(rm_armor_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 导出编译数据库，供 clangd / VSCode 智能提示使用（与 01/02 保持一致）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 复用题1 的检测器库（纯 CMake 工程，用 add_subdirectory 引入）
add_subdirectory(../01_detector 01_detector EXCLUDE_FROM_ALL)

# P0：视频 -> detector -> 发布标注图像话题
add_executable(armor_video_node src/armor_video_node.cpp)
ament_target_dependencies(armor_video_node rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(armor_video_node armor_detector ${OpenCV_LIBS})

install(TARGETS armor_video_node DESTINATION lib/${PROJECT_NAME})

ament_package()
