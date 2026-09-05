cmake_minimum_required(VERSION 3.16)
project(armor_tracker LANGUAGES CXX)

# 统一使用 C++17，并导出编译数据库（供 clangd / VSCode 智能提示使用）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

# L2 学习工具：正投影（3D -> 2D）演示，把 L1 的相机模型"看"见
add_executable(projection_demo src/projection_demo.cpp)
target_link_libraries(projection_demo PRIVATE ${OpenCV_LIBS})
