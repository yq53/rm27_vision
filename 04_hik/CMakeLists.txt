cmake_minimum_required(VERSION 3.16)
project(hik_probe LANGUAGES CXX)

# 统一 C++17 + 导出编译数据库（clangd / VSCode 提示用，和 01/02 一致）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ========== 找到海康 MVS SDK ==========
# 概念①：让编译器"看见"海康的头文件和库文件，需要告诉 CMake 三样东西：
#   1) 头文件目录（include）：里面 MvCameraControl.h 声明了所有 MV_CC_* 函数
#   2) 库目录（lib/64）：里面放着编译好的 libMvCameraControl.so
#   3) 库名（MvCameraControl）：链接时告诉链接器"我要用这个库"
# 库的物理文件名是 libMvCameraControl.so，链接时去掉前缀 lib 和后缀 .so 写 MvCameraControl。
set(MVS_SDK_DIR "/opt/MVS" CACHE PATH "海康 MVS SDK 安装目录（装到别处时用 -DMVS_SDK_DIR=... 覆盖）")
set(MVS_INCLUDE_DIR "${MVS_SDK_DIR}/include")
set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/64")

# 提前检查：头文件在不在？不在说明 SDK 没装或路径不对，直接报错而不是编译到一半才懵
if(NOT EXISTS "${MVS_INCLUDE_DIR}/MvCameraControl.h")
    message(FATAL_ERROR "找不到 ${MVS_INCLUDE_DIR}/MvCameraControl.h，请先安装 MVS SDK 或 -DMVS_SDK_DIR=...")
endif()

add_executable(hik_probe src/hik_probe.cpp)
target_include_directories(hik_probe PRIVATE ${MVS_INCLUDE_DIR})
target_link_directories(hik_probe PRIVATE ${MVS_LIB_DIR})
target_link_libraries(hik_probe PRIVATE MvCameraControl)

# B 步：按序列号打开相机 + 设曝光/增益
add_executable(hik_open src/hik_open.cpp)
target_include_directories(hik_open PRIVATE ${MVS_INCLUDE_DIR})
target_link_directories(hik_open PRIVATE ${MVS_LIB_DIR})
target_link_libraries(hik_open PRIVATE MvCameraControl)

# 运行提示：程序运行时需要能"找到" .so（动态库是运行时才加载的）
#   cmake 之外执行: export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
message(STATUS "MVS SDK: include=${MVS_INCLUDE_DIR}  lib=${MVS_LIB_DIR}")
