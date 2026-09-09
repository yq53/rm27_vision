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

# 海康 MVS SDK 接入（camera.yaml / hik 源）：默认 OFF，普通构建不引入 MVS/yaml 依赖。
# 现场/本机启用：colcon build --cmake-args -DUSE_HIK_SDK=ON
option(USE_HIK_SDK "启用海康 MVS SDK(hik 相机源 + camera.yaml)" OFF)
set(MVS_SDK_DIR "/opt/MVS" CACHE PATH "海康 MVS SDK 安装目录")
if(USE_HIK_SDK)
  if(NOT EXISTS "${MVS_SDK_DIR}/include/MvCameraControl.h")
    message(FATAL_ERROR "找不到 ${MVS_SDK_DIR}/include/MvCameraControl.h，请先安装 MVS 或 -DMVS_SDK_DIR=...")
  endif()
  find_package(yaml-cpp REQUIRED)
  message(STATUS "USE_HIK_SDK=ON: MVS SDK = ${MVS_SDK_DIR}")
endif()

# 复用题2 工程（内部引入 01_detector 的 armor_detector，并提供 armor_ekf）
add_subdirectory(../02_tracker 02_tracker EXCLUDE_FROM_ALL)

# P0 教学示例（仅 detector）：视频 -> detector -> 发布标注图像话题
add_executable(armor_video_node src/armor_video_node.cpp)
ament_target_dependencies(armor_video_node rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(armor_video_node armor_detector ${OpenCV_LIBS})

# 主节点（完整链路）：detect -> PnP -> EKF -> 发布单条 /armor/state + 标注图 /armor/annotated
# image_source.cpp：海康专属代码在文件内 #ifdef RM_USE_HIK_SDK 段，开关关闭时为空实现
add_executable(armor_tracker_node src/armor_tracker_node.cpp src/image_source.cpp)
ament_target_dependencies(
  armor_tracker_node rclcpp sensor_msgs std_msgs cv_bridge rm_interfaces
)
target_link_libraries(armor_tracker_node armor_detector armor_ekf ${OpenCV_LIBS})
if(USE_HIK_SDK)
  target_compile_definitions(armor_tracker_node PRIVATE RM_USE_HIK_SDK=1)
  target_include_directories(armor_tracker_node PRIVATE ${MVS_SDK_DIR}/include)
  target_link_directories(armor_tracker_node PRIVATE ${MVS_SDK_DIR}/lib/64)
  target_link_libraries(armor_tracker_node MvCameraControl yaml-cpp)
endif()

install(TARGETS armor_video_node armor_tracker_node DESTINATION lib/${PROJECT_NAME})

ament_package()
