#!/usr/bin/env bash
# [工具] 一键启动可视化演示：准备好环境（ROS2 环境 / RMW / 日志目录），再启动主节点与 rqt。
#
# 用法（在仓库根目录，或任意目录——脚本会自己定位仓库根）：
#   bash scripts/demo.sh                        # 视频源 data/demo.avi + bbox 检测器 + rqt（默认）
#   bash scripts/demo.sh pose                   # 四关键点检测器（需已按 README「构建 C」启用 ONNX Runtime）
#   bash scripts/demo.sh ip 192.168.1.10:8080   # 手机 IP Webcam（会自动补 http:// 与 /video 后缀）
#   bash scripts/demo.sh hik                    # 海康相机（读 data/camera.yaml 里的 serial_number）
#   bash scripts/demo.sh pose no_rqt            # 加 no_rqt 则只起节点，不弹 rqt（位置随意）
#
# 等价的手工命令（不使用本脚本）见 README「题3 → 一键启动（推荐）」与「手动分终端」。
set -eo pipefail   # 注意：不能用 -u —— ROS2 的 setup.bash 引用了未定义变量，会直接报错退出

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

MODE="${1:-bbox}"
if [[ $# -gt 0 ]]; then shift; fi

# no_rqt 允许出现在任意位置（各模式第 2 个参数含义不同：ip 模式那里是地址）
USE_RQT=1
REST=()
for arg in "$@"; do
    if [[ "$arg" == "no_rqt" ]]; then
        USE_RQT=0
    else
        REST+=("$arg")
    fi
done
if [[ ${#REST[@]} -gt 0 ]]; then set -- "${REST[@]}"; else set --; fi

# ---- 环境准备（这几步就是"手工版"的开头几行）----
if [[ ! -f install/setup.bash ]]; then
    echo "[demo.sh] 还没构建过。请先执行：" >&2
    echo "  source /opt/ros/humble/setup.bash && colcon build --packages-up-to rm_armor_visualization" >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
export ROS_LOG_DIR="$REPO/.roslog"          # ~/.ros 不可写时也能落日志
mkdir -p "$ROS_LOG_DIR"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"   # 跨 RMW 传大图会掉帧

if [[ "$USE_RQT" == "1" ]] && ! ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
    echo "[demo.sh] 未找到 rqt_image_view，已自动改为不启动 rqt。" >&2
    echo "           安装：sudo apt install ros-humble-rqt-image-view" >&2
    USE_RQT=0
fi

# ---- 模式 -> launch 参数 ----
LAUNCH_ARGS=()
case "$MODE" in
    bbox | pose)
        LAUNCH_ARGS+=("detector:=$MODE")
        if [[ "$MODE" == "pose" ]]; then
            POSE_MODEL="models/third_party/Infantry-v8n/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx"
            if [[ ! -f "$POSE_MODEL" ]]; then
                echo "[demo.sh] 找不到四关键点模型：$POSE_MODEL" >&2
                exit 1
            fi
            LAUNCH_ARGS+=("pose_model_path:=$POSE_MODEL")
        fi
        ;;
    ip)
        URL="${1:-}"
        if [[ -z "$URL" ]]; then
            echo "[demo.sh] 用法：bash scripts/demo.sh ip <地址:端口>   例如 ip 192.168.1.10:8080" >&2
            exit 1
        fi
        shift
        [[ "$URL" == http* ]] || URL="http://$URL"      # 补协议
        [[ "$URL" == */video ]] || URL="$URL/video"     # 补 /video（裸地址是网页，打不开）
        LAUNCH_ARGS+=("source:=ip" "ip_url:=$URL")      # 手机端务必横屏，否则 PnP 出镜像假解
        ;;
    hik)
        LAUNCH_ARGS+=("source:=hik")
        ;;
    *)
        echo "[demo.sh] 未知模式：$MODE（可选 bbox / pose / ip / hik）" >&2
        exit 1
        ;;
esac

if [[ "$USE_RQT" == "1" ]]; then LAUNCH_ARGS+=("use_rqt:=true"); fi

echo "[demo.sh] 模式=$MODE  rqt=$USE_RQT"
echo "[demo.sh] ros2 launch rm_armor_visualization armor_tracker.launch.py ${LAUNCH_ARGS[*]} $*"
exec ros2 launch rm_armor_visualization armor_tracker.launch.py "${LAUNCH_ARGS[@]}" "$@"
