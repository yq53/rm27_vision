cmake_minimum_required(VERSION 3.16)
project(armor_detector LANGUAGES CXX)

# 统一使用 C++17，并导出编译数据库（供 clangd / VSCode 智能提示使用）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

# 检测器静态库：题1 的正式交付物，
# 题2 tracker、题3 ROS2 节点都会链接这个库复用 detect()
add_library(armor_detector STATIC
    src/armor_detector.cpp
)
target_include_directories(armor_detector PUBLIC include)
target_link_libraries(armor_detector PUBLIC ${OpenCV_LIBS})

# 题1 演示程序：读视频 -> 逐帧 detect -> 画框 -> 保存结果视频
add_executable(armor_demo src/demo_main.cpp)
target_link_libraries(armor_demo PRIVATE armor_detector)
