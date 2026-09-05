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
