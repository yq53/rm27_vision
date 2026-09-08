cmake_minimum_required(VERSION 3.16)
project(armor_tracker LANGUAGES CXX)

# 统一使用 C++17，并导出编译数据库（供 clangd / VSCode 智能提示使用）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

# 引入题1 的检测器库（EXCLUDE_FROM_ALL：只编库，不默认编它的 armor_demo）
add_subdirectory(../01_detector 01_detector EXCLUDE_FROM_ALL)

# L2 学习工具：正投影（3D -> 2D）演示，把 L1 的相机模型"看"见
add_executable(projection_demo src/projection_demo.cpp)
target_link_libraries(projection_demo PRIVATE ${OpenCV_LIBS})

# L3：让 YOLO 框长出四角点（v1：bbox 角点近似），打通 01_detector -> 02_tracker
add_executable(corner_demo src/corner_demo.cpp)
target_link_libraries(corner_demo PRIVATE armor_detector ${OpenCV_LIBS})

# L4：PnP 闭环（A 合成验证 + B 视频叠加测距）
add_executable(pnp_demo src/pnp_demo.cpp)
target_link_libraries(pnp_demo PRIVATE armor_detector ${OpenCV_LIBS})

# L7：ArmorEKF 库（头文件在 include/，与 01_detector 同款规范）
add_library(armor_ekf STATIC src/armor_ekf.cpp)
target_include_directories(armor_ekf PUBLIC include)
target_link_libraries(armor_ekf PUBLIC ${OpenCV_LIBS})

# L7：tracker_demo（PnP -> EKF 平滑，原/滤对比）
add_executable(tracker_demo src/tracker_demo.cpp)
target_link_libraries(tracker_demo PRIVATE armor_detector armor_ekf ${OpenCV_LIBS})

# L8：框内灯条精定位库 + 演示（v2 的 CV 半边）
add_library(light_bar_detector STATIC src/light_bar_detector.cpp)
target_include_directories(light_bar_detector PUBLIC include)
target_link_libraries(light_bar_detector PUBLIC ${OpenCV_LIBS})

add_executable(lightbar_demo src/lightbar_demo.cpp)
target_link_libraries(lightbar_demo PRIVATE armor_detector light_bar_detector ${OpenCV_LIBS})
