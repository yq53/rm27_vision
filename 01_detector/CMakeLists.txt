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

# 四关键点检测器库（灯条端点输出，供 PnP 直接使用）
# 后端选择：默认 cv2.dnn（零额外依赖）；ONNX Runtime 后端需显式开启：
#   cmake -S 01_detector -B build -DUSE_ONNXRUNTIME=ON -DONNXRUNTIME_DIR=<ort 根目录>
option(USE_ONNXRUNTIME "四关键点检测器使用 ONNX Runtime 后端" OFF)
set(ONNXRUNTIME_DIR "" CACHE PATH "ONNX Runtime 解压目录（含 include/ 与 lib/）")

add_library(armor_pose_detector STATIC
    src/armor_pose_detector.cpp
)
target_include_directories(armor_pose_detector PUBLIC include)
target_link_libraries(armor_pose_detector PUBLIC ${OpenCV_LIBS})

if(USE_ONNXRUNTIME)
  if(NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
    message(FATAL_ERROR "USE_ONNXRUNTIME=ON 需要 -DONNXRUNTIME_DIR=...（含 include/ 与 lib/）")
  endif()
  target_compile_definitions(armor_pose_detector PUBLIC RM_USE_ONNXRUNTIME=1)
  target_include_directories(armor_pose_detector PUBLIC ${ONNXRUNTIME_DIR}/include)
  target_link_directories(armor_pose_detector PUBLIC ${ONNXRUNTIME_DIR}/lib)
  target_link_libraries(armor_pose_detector PUBLIC onnxruntime)
  # 让可执行文件在运行时也能找到 libonnxruntime.so
  target_link_options(armor_pose_detector PUBLIC "-Wl,-rpath,${ONNXRUNTIME_DIR}/lib")
  message(STATUS "armor_pose_detector 使用 ONNX Runtime: ${ONNXRUNTIME_DIR}")
endif()

# 题1 演示程序：读视频 -> 逐帧 detect -> 画框 -> 保存结果视频
add_executable(armor_demo src/demo_main.cpp)
target_link_libraries(armor_demo PRIVATE armor_detector)
