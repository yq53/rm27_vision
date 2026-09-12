#!/usr/bin/env bash
# [工具] 一键配置环境 + 构建（**本脚本不启动任何程序**；启动一律交给 launch 文件）
#
# 用法（在仓库根目录，或任意目录——脚本会自己定位仓库根）：
#   bash scripts/setup.sh            # 配置环境 + 构建（自动探测可选的 ONNX Runtime / MVS SDK）
#   bash scripts/setup.sh --clean    # 先删掉 build/ install/ log/ 再重新构建
#   ORT_DIR=/path/to/onnxruntime bash scripts/setup.sh    # 手工指定 ORT 位置
#
# 它做三件事：
#   1) source ROS2 环境，并设好本工程需要的环境变量（RMW / ROS_LOG_DIR）；
#   2) 构建：题1/题2 的普通 CMake 工程 + 题3 的 ROS2 包 + 04_hik 学习 demo，
#      能探到 ONNX Runtime 就顺手把 pose 检测器通路一起编进去（探到 MVS SDK 就编上海康源）；
#   3) 打印构建结果与"下一步跑什么"。
#
# 构建完就能用（仍在仓库根目录执行）：
#   ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true print_state:=true
#   → 视频源 data/demo.avi + bbox 检测器 + rqt 画面 + 终端状态数字
#
#   POSE_MODEL=models/third_party/Infantry-v8n/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx
#   ros2 launch rm_armor_visualization armor_tracker.launch.py detector:=pose pose_model_path:="$POSE_MODEL" use_rqt:=true
#   → 同样素材，换成四关键点检测器
set -eo pipefail   # 注意：不能用 -u —— ROS2 的 setup.bash 引用了未定义变量

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

DO_CLEAN=0
if [[ "${1:-}" == "--clean" ]]; then DO_CLEAN=1; fi

echo "[setup] 仓库根目录：$REPO"

# ---------- 1) 环境 ----------
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[setup] 找不到 /opt/ros/humble/setup.bash —— 请先安装 ROS2 Humble" >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

export ROS_LOG_DIR="$REPO/.roslog"   # ~/.ros 不可写时也能落日志
mkdir -p "$ROS_LOG_DIR"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"   # 跨 RMW 传大图像会掉帧
echo "[setup] ROS2 环境就绪：ROS_LOG_DIR=$ROS_LOG_DIR  RMW=$RMW_IMPLEMENTATION"
if [[ "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]]; then
    echo "[setup] 提示：当前 RMW 不是 rmw_fastrtps_cpp。单节点无所谓；但发布端与订阅端 RMW 不一致时，"
    echo "        /armor/annotated 这类大图像会明显掉帧（launch 内部固定用 fastrtps，不受这里影响）。"
fi

# ---------- 2) 探测可选依赖 ----------
# CMAKE_ARGS：给 colcon（题3 的 ROS 包）；PLAIN_ARGS：给题1/题2 的普通 CMake 工程。
# 两边都要带 ORT 开关：题2 的 eval_demo（离线评测基础设施）用 pose 跑 A/B，靠的就是这个开关；
# 只给 colcon 的话，clone 者编出来的 eval_demo 会在 detector=pose 时直接抛异常。
CMAKE_ARGS=()
PLAIN_ARGS=()

# 在多个常见位置找 ONNX Runtime（解压即用的目录，特征文件是 include/onnxruntime_cxx_api.h）
if [[ -z "${ORT_DIR:-}" ]]; then
    for candidate in "$REPO/.models_ext/onnxruntime" "$REPO/../.models_ext/onnxruntime" \
             "$REPO/../../.models_ext/onnxruntime" "$REPO/third_party/onnxruntime" \
             "/opt/ros/${ROS_DISTRO:-humble}/opt/onnxruntime_vendor" \
             /opt/onnxruntime /usr/local/onnxruntime \
             "$HOME"/onnxruntime-linux-x64-* "$HOME"/onnxruntime*; do
        if [[ -f "$candidate/include/onnxruntime_cxx_api.h" ]]; then
            ORT_DIR="$candidate"
            break
        fi
    done
fi
if [[ -n "${ORT_DIR:-}" && -f "$ORT_DIR/include/onnxruntime_cxx_api.h" ]]; then
    echo "[setup] 检测到 ONNX Runtime：$ORT_DIR  → 启用四关键点检测器（detector:=pose）"
    CMAKE_ARGS+=(-DUSE_ONNXRUNTIME=ON "-DONNXRUNTIME_DIR=$ORT_DIR")
    PLAIN_ARGS+=(-DUSE_ONNXRUNTIME=ON "-DONNXRUNTIME_DIR=$ORT_DIR")
else
    echo "[setup] 未检测到 ONNX Runtime → 只构建 bbox 通路（detector:=pose 将不可用）"
    echo "        · 首选（装了 ROS2 就有源，一条命令）："
    echo "              sudo apt install ros-${ROS_DISTRO:-humble}-onnxruntime-vendor"
    echo "        · 备选（离线 / 没有 ROS 源）：下载 onnxruntime-linux-x64-*.tgz 解压后指过来"
    echo "              ORT_DIR=/你的解压目录 bash scripts/setup.sh"
    echo "              （下载页 https://github.com/microsoft/onnxruntime/releases）"
    echo "        ⚠ 不启用 ORT 时，README 第三步的 ②（pose）会直接报错退出；①（bbox）不受影响。"
fi

if [[ -f /opt/MVS/include/MvCameraControl.h ]]; then
    echo "[setup] 检测到海康 MVS SDK（/opt/MVS） → 启用 source:=hik 图像源"
    CMAKE_ARGS+=(-DUSE_HIK_SDK=ON)
fi

# ---------- 3) 构建 ----------
if [[ "$DO_CLEAN" == "1" ]]; then
    echo "[setup] --clean：删除 build/ install/ log/"
    rm -rf build install log
fi

JOBS="$(nproc)"
echo "[setup] 构建题1（01_detector）…"
cmake -S 01_detector -B build/01_detector -DCMAKE_BUILD_TYPE=Release "${PLAIN_ARGS[@]}"
cmake --build build/01_detector -j"$JOBS"

echo "[setup] 构建题2（02_tracker）…"
cmake -S 02_tracker -B build/02_tracker -DCMAKE_BUILD_TYPE=Release "${PLAIN_ARGS[@]}"
cmake --build build/02_tracker -j"$JOBS"

echo "[setup] 构建题3（ROS2 包 rm_armor_visualization）…"
if [[ ${#CMAKE_ARGS[@]} -gt 0 ]]; then
    colcon build --packages-up-to rm_armor_visualization --cmake-args "${CMAKE_ARGS[@]}"
else
    colcon build --packages-up-to rm_armor_visualization
fi

if [[ -f /opt/MVS/include/MvCameraControl.h ]]; then
    echo "[setup] 构建 04_hik 学习 demo…"
    cmake -S 04_hik -B build/04_hik
    cmake --build build/04_hik -j"$JOBS"
fi

# ---------- 4) 收尾：只提示，不启动 ----------
source install/setup.bash
echo
echo "[setup] 构建完成，开关状态："
grep -E "USE_ONNXRUNTIME|USE_HIK_SDK" build/rm_armor_visualization/CMakeCache.txt || true
echo
echo "[setup] 下一步（本脚本不启动任何程序，启动交给 launch）："
echo
echo "  # 题1：视频检测（普通可执行文件，不走 ROS）"
echo "  ./build/01_detector/armor_demo data/demo.avi models/armor_yolov8n.onnx 240"
echo
echo "  # 题3：视频源 data/demo.avi + bbox 检测器 + rqt + 终端状态数字"
echo "  ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true print_state:=true"
echo
echo "  # 题3：同样素材，换成四关键点检测器（需上面显示 USE_ONNXRUNTIME=ON）"
echo "  ros2 launch rm_armor_visualization armor_tracker.launch.py \\"
echo "      detector:=pose \\"
echo "      pose_model_path:=models/third_party/Infantry-v8n/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx \\"
echo "      use_rqt:=true"
echo
echo "  # 手机/海康相机：见 README「题3 → 真实相机」"
