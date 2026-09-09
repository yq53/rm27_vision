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
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rm_interfaces REQUIRED)

# 复用题2 工程（内部引入 01_detector 的 armor_detector，并提供 armor_ekf）
add_subdirectory(../02_tracker 02_tracker EXCLUDE_FROM_ALL)

# 相机接入抽象（video/ip/hik + camera.yaml 可配；hik 需 USE_HIK_SDK=ON）
add_subdirectory(../04_camera 04_camera EXCLUDE_FROM_ALL)

# P0 教学示例（仅 detector）：视频 -> detector -> 发布标注图像话题
add_executable(armor_video_node src/armor_video_node.cpp)
ament_target_dependencies(armor_video_node rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(armor_video_node armor_detector ${OpenCV_LIBS})

# 主节点（完整链路）：detect -> PnP -> EKF -> 发布单条 /armor/state + 标注图 /armor/annotated
add_executable(armor_tracker_node src/armor_tracker_node.cpp)
ament_target_dependencies(
  armor_tracker_node rclcpp sensor_msgs std_msgs cv_bridge rm_interfaces
)
target_link_libraries(armor_tracker_node armor_detector armor_ekf camera_source ${OpenCV_LIBS})

install(TARGETS armor_video_node armor_tracker_node DESTINATION lib/${PROJECT_NAME})

ament_package()
