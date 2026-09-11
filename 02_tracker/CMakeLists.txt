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

# PnP 共用库：位姿解算 + 重投影误差/yaw 指标（评测与后续角点升级共用）
add_library(armor_pnp STATIC src/armor_pnp.cpp)
target_include_directories(armor_pnp PUBLIC include)
target_link_libraries(armor_pnp PUBLIC ${OpenCV_LIBS})

# 角点精修库（v2）：ROI 内灯条检测 + PCA 端点 + 亮度梯度修正
add_library(armor_corner STATIC src/armor_corner.cpp)
target_include_directories(armor_corner PUBLIC include)
target_link_libraries(armor_corner PUBLIC ${OpenCV_LIBS})

# 评测工具：detector -> 角点(bbox/refine) -> PnP -> EKF，输出逐帧 CSV + 汇总指标
add_executable(eval_demo src/eval_demo.cpp)
target_link_libraries(eval_demo PRIVATE armor_detector armor_pose_detector armor_ekf armor_pnp armor_corner ${OpenCV_LIBS})

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
