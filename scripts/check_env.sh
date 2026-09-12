#!/usr/bin/env bash
# [工具] 环境自检：检查这台机器是否具备运行本工程所需的依赖，缺什么就告诉你装什么。
#
# 用法（在仓库根目录，或任意目录——脚本会自己定位仓库根）：
#   bash scripts/check_env.sh
#
# 退出码：0 = 必需项全齐（可选项缺失只警告）；1 = 有必需项缺失。
# 它只做检查，不改动系统、不安装任何东西。
set -eo pipefail   # 不能用 -u：ROS2 的 setup.bash 引用了未定义变量

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

OK_N=0; WARN_N=0; FAIL_N=0
ok()   { printf '  \033[32m[ OK ]\033[0m %s\n' "$1"; OK_N=$((OK_N + 1)); }
warn() { printf '  \033[33m[警告]\033[0m %s\n' "$1"; WARN_N=$((WARN_N + 1)); }
fail() { printf '  \033[31m[缺失]\033[0m %s\n' "$1"; FAIL_N=$((FAIL_N + 1)); }
hint() { printf '         └─ %s\n' "$1"; }
title() { printf '\n\033[1m%s\033[0m\n' "$1"; }

printf '\033[1m== rm27_vision 环境自检 ==\033[0m  （仓库：%s）\n' "$REPO"
printf '只做检查，不会安装或修改任何东西。\n'

# ---------------------------------------------------------------- 1. 系统与工具链
title '[1/6] 系统与编译工具链'

if [[ -r /etc/os-release ]]; then
    . /etc/os-release
    if [[ "${VERSION_ID:-}" == "22.04" ]]; then
        ok "系统 ${PRETTY_NAME}"
    else
        warn "系统是 ${PRETTY_NAME:-未知}，本工程在 Ubuntu 22.04 + ROS2 Humble 上验证；
       换版本时 ROS2 发行版名也要跟着换（apt 包名里的 humble → 你的发行版）"
    fi
else
    warn "读不到 /etc/os-release，无法判断系统版本"
fi

for tool in g++ clang++ cmake make pkg-config; do
    if command -v "$tool" >/dev/null 2>&1; then
        ok "$tool（$(command -v "$tool")）"
    else
        case "$tool" in
        g++ | clang++ | make) hint "sudo apt install build-essential" ;;
        cmake) hint "sudo apt install cmake   # 需要 ≥ 3.16" ;;
        pkg-config) hint "sudo apt install pkg-config" ;;
        esac
        # 编译器二者有其一即可，另一种缺失不算错
        if [[ "$tool" == "g++" || "$tool" == "clang++" ]]; then
            warn "$tool 未找到"
        else
            fail "$tool 未找到"
        fi
    fi
done
command -v g++ >/dev/null 2>&1 || command -v clang++ >/dev/null 2>&1 \
    || fail "g++ / clang++ 都没有——没有 C++ 编译器无法构建"

# ---------------------------------------------------------------- 2. ROS2 与软件包
title '[2/6] ROS2 Humble 与所需软件包（题3 需要）'

ROS_SETUP=""
for candidate in /opt/ros/humble/setup.bash "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; do
    if [[ -f "$candidate" ]]; then ROS_SETUP="$candidate"; break; fi
done

if [[ -z "$ROS_SETUP" ]]; then
    fail "找不到 /opt/ros/humble/setup.bash —— 未安装 ROS2 Humble"
    hint "安装步骤见 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    hint "装完再跑一次本脚本；只想验证题1/题2 的话可以先不装（它们是纯 CMake 工程）"
else
    ok "ROS2：$ROS_SETUP"
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    if command -v ros2 >/dev/null 2>&1; then
        ok "ros2 CLI 可用"
    else
        fail "ros2 CLI 不可用（source 了 $ROS_SETUP 之后仍找不到 ros2）"
    fi
    # 逐个检查本工程依赖的 ROS2 包
    for pkg in rclcpp sensor_msgs std_msgs geometry_msgs cv_bridge; do
        if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
            ok "ROS2 包 $pkg"
        else
            fail "ROS2 包 $pkg 缺失"
            hint "sudo apt install ros-humble-$pkg"
        fi
    done
    # rqt 只在 use_rqt:=true 时需要
    if ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
        ok "ROS2 包 rqt_image_view（launch 的 use_rqt:=true 用）"
    else
        warn "ROS2 包 rqt_image_view 缺失 —— 不影响节点运行，只是 rqt_image_view 看不了图"
        hint "sudo apt install ros-humble-rqt-image-view"
    fi
fi

if command -v colcon >/dev/null 2>&1; then
    ok "colcon（题3 构建用）"
else
    fail "colcon 未安装"
    hint "sudo apt install python3-colcon-common-extensions"
fi

# ---------------------------------------------------------------- 3. OpenCV
title '[3/6] OpenCV（题1/题2 需要，题3 也需要）'

if pkg-config --exists opencv4 2>/dev/null; then
    ok "OpenCV C++：$(pkg-config --modversion opencv4)"
else
    fail "找不到 OpenCV4 的 pkg-config 信息（opencv4.pc）"
    hint "sudo apt install libopencv-dev"
fi

# ---------------------------------------------------------------- 4. 可选：ONNX Runtime
title '[4/6] 可选依赖：ONNX Runtime（只影响 detector:=pose）'

ORT_FOUND=""
if [[ -n "${ORT_DIR:-}" && -f "$ORT_DIR/include/onnxruntime_cxx_api.h" ]]; then
    ORT_FOUND="$ORT_DIR"
fi
if [[ -z "$ORT_FOUND" ]]; then
    for candidate in "$REPO/.models_ext/onnxruntime" "$REPO/../.models_ext/onnxruntime" \
             "$REPO/../../.models_ext/onnxruntime" "$REPO/third_party/onnxruntime" \
             "/opt/ros/${ROS_DISTRO:-humble}/opt/onnxruntime_vendor" \
             /opt/onnxruntime /usr/local/onnxruntime \
             "$HOME"/onnxruntime-linux-x64-* "$HOME"/onnxruntime*; do
        if [[ -f "$candidate/include/onnxruntime_cxx_api.h" ]]; then ORT_FOUND="$candidate"; break; fi
    done
fi

if [[ -n "$ORT_FOUND" ]]; then
    ORT_FOUND="$(readlink -f "$ORT_FOUND")"   # 归一化，避免输出里出现 ../ 这种相对路径
    ok "ONNX Runtime：$ORT_FOUND"
    [[ -f "$ORT_FOUND/lib/libonnxruntime.so" ]] \
        && ok "libonnxruntime.so 存在" \
        || warn "有头文件但没找到 $ORT_FOUND/lib/libonnxruntime.so，构建可能失败"
    hint "构建时用：ORT_DIR=$ORT_FOUND bash scripts/setup.sh"
else
    warn "未找到 ONNX Runtime —— detector:=pose 不可用（bbox 通路完全不受影响）"
    hint "首选（装了 ROS2 就有源，一条命令）：sudo apt install ros-${ROS_DISTRO:-humble}-onnxruntime-vendor"
    hint "备选（离线 / 没有 ROS 源）：取 onnxruntime-linux-x64-*.tgz 解压，再 ORT_DIR=/解压目录 bash scripts/setup.sh"
    hint "注意：pip install onnxruntime 是 Python 包，C++ 链接不了，别用那个"
fi

# ---------------------------------------------------------------- 5. 可选：海康 MVS SDK
title '[5/6] 可选依赖：海康 MVS SDK（只影响 source:=hik）'

MVS_DIR="${MVS_SDK_DIR:-/opt/MVS}"
if [[ -f "$MVS_DIR/include/MvCameraControl.h" ]]; then
    ok "MVS SDK 头文件：$MVS_DIR/include/MvCameraControl.h"
    if [[ -f "$MVS_DIR/lib/64/libMvCameraControl.so" ]]; then
        ok "MVS 动态库：$MVS_DIR/lib/64/libMvCameraControl.so"
    else
        warn "没找到 $MVS_DIR/lib/64/libMvCameraControl.so"
    fi
    if pkg-config --exists yaml-cpp 2>/dev/null; then
        ok "yaml-cpp（读 data/camera.yaml 用）"
    else
        fail "yaml-cpp 未安装 —— 启用 MVS 后构建会失败"
        hint "sudo apt install libyaml-cpp-dev"
    fi
    hint "本机没有海康相机也不影响构建：只是运行时枚举不到设备"
else
    warn "未找到 MVS SDK（$MVS_DIR/include/MvCameraControl.h）—— source:=hik 不可用"
    hint "需要海康相机时：装 MVS 客户端（含 SDK）后重跑本脚本；"
    hint "或用 -DMVS_SDK_DIR=/你的/MVS 指定非默认安装位置"
fi

# ---------------------------------------------------------------- 6. 仓库自带素材
title '[6/6] 仓库自带素材与模型'

for f in data/demo.avi data/camera.yaml \
         models/armor_yolov8n.onnx \
         models/third_party/Infantry-v8n/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx; do
    if [[ -f "$f" ]]; then ok "$f"; else fail "$f 缺失（应从 git 里带出来，检查是否 clone 完整）"; fi
done

# 日志目录可写性（~/.ros 只读会报 spdlog 错）
if [[ ! -w "$HOME/.ros" && -e "$HOME/.ros" ]]; then
    warn "$HOME/.ros 不可写 —— 运行节点会报 'Read-only file system'"
    hint 'export ROS_LOG_DIR=$PWD/.roslog'
fi

# ---------------------------------------------------------------- 总结与下一步
title '== 自检结论 =='
printf '  必需项通过：%d    警告：%d    缺失：%d\n' "$OK_N" "$WARN_N" "$FAIL_N"

if [[ "$FAIL_N" -gt 0 ]]; then
    printf '\n\033[31m有必需项缺失，请按上面的提示安装后重跑本脚本。\033[0m\n'
    exit 1
fi

printf '\n\033[32m必需项齐全，可以开始构建。\033[0m\n\n'
printf '下一步（按上面探测结果）：\n'
if [[ -n "$ORT_FOUND" ]]; then
    printf '  bash scripts/setup.sh            # 会自动启用 ONNX Runtime → detector:=pose 可用\n'
else
    printf '  bash scripts/setup.sh            # 只有 bbox 通路；想用 pose 先按 [4/6] 装 ONNX Runtime\n'
fi
printf '\n构建完成后（启动一律走 launch，本脚本不启动任何程序）：\n'
printf '  source /opt/ros/humble/setup.bash && source install/setup.bash\n'
printf '  ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true print_state:=true\n'
printf '\n更多命令（两种素材 × 两种检测器）见 README「快速验证（题3）」。\n'
