cmake_minimum_required(VERSION 3.16)
project(camera_source LANGUAGES CXX)

# 统一使用 C++17，并导出编译数据库（供 clangd / VSCode 智能提示使用）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# 海康 MVS SDK 后端：默认 OFF（无设备/无 SDK 也能整体编译运行）。
# 现场接入：cmake -DUSE_HIK_SDK=ON -DMVS_INCLUDE_DIR=... -DMVS_LIB=... 
option(USE_HIK_SDK "启用海康 MVS SDK 后端（现场有 SDK/相机时打开）" OFF)

add_library(camera_source STATIC src/camera_source.cpp)
target_include_directories(camera_source PUBLIC include)
target_link_libraries(camera_source PUBLIC ${OpenCV_LIBS} yaml-cpp)

if(USE_HIK_SDK)
    # 需要调用方通过 -DMVS_INCLUDE_DIR/-DMVS_LIB 指到 MVS 的 include 与库文件
    if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIB)
        message(FATAL_ERROR "USE_HIK_SDK=ON 时必须提供 -DMVS_INCLUDE_DIR=... -DMVS_LIB=...")
    endif()
    target_compile_definitions(camera_source PUBLIC RM_USE_HIK_SDK=1)
    target_include_directories(camera_source PUBLIC ${MVS_INCLUDE_DIR})
    target_link_libraries(camera_source PUBLIC ${MVS_LIB})
    message(STATUS "海康 HIK SDK 后端已启用")
endif()

# 探针 demo：camera_probe [camera.yaml] [最大帧数]
add_executable(camera_probe src/camera_probe.cpp)
target_link_libraries(camera_probe PRIVATE camera_source)
