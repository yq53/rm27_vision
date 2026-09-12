"""一键启动 armor_tracker_node（可选 rqt 可视化）。

用法（在仓库根目录）:
    ros2 launch rm_armor_visualization armor_tracker.launch.py                     # 视频源(默认)
    ros2 launch rm_armor_visualization armor_tracker.launch.py source:=ip ip_url:=http://<手机IP>:8080/video
    ros2 launch rm_armor_visualization armor_tracker.launch.py source:=hik          # 现场海康(读 camera.yaml)
    ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true        # 附带 rqt 看图
    ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true print_state:=true  # 图 + 终端里的状态数字
    ros2 launch rm_armor_visualization armor_tracker.launch.py repo_root:=$HOME/xxx/rm27_vision   # 任意目录启动

说明:
  - source 是"单一入口"参数，launch 把它翻译成节点真实参数（video_path / camera_config）；
  - repo_root 用于把 data/、models/ 等相对路径拼成绝对路径，默认 "." = 当前目录；
  - 环境变量（RMW_IMPLEMENTATION、hik 的 LD_LIBRARY_PATH）由本 launch 设置，
    仅对本次启动的进程生效，不写入 ~/.bashrc。
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    source = LaunchConfiguration("source").perform(context)
    repo_root = LaunchConfiguration("repo_root")

    if source not in ("video", "ip", "hik"):
        raise RuntimeError(
            f"source 必须是 video / ip / hik 之一，当前为 '{source}'"
        )

    actions = []

    # ① 环境变量：RMW 用 FastDDS（本机 CycloneDDS 传大图像不稳）
    actions.append(SetEnvironmentVariable("RMW_IMPLEMENTATION", LaunchConfiguration("rmw")))

    # ② hik 模式：把 MVS 库目录前置到 LD_LIBRARY_PATH（进程级，不改 .bashrc）
    if source == "hik":
        actions.append(
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    LaunchConfiguration("mvs_lib_dir"),
                    ":",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            )
        )

    # ③ source -> 节点参数 的映射（node 本身只认 video_path / camera_config）
    if source == "hik":
        source_param = {
            "camera_config": PathJoinSubstitution(
                [repo_root, LaunchConfiguration("camera_config")]
            )
        }
        source_desc = "hik (读 camera.yaml)"
    elif source == "ip":
        ip_url = LaunchConfiguration("ip_url").perform(context)
        if not ip_url:
            raise RuntimeError("source:=ip 时必须提供 ip_url，例如 ip_url:=http://10.0.0.5:8080/video")
        source_param = {"video_path": LaunchConfiguration("ip_url")}  # URL 不做路径拼接
        source_desc = f"ip ({ip_url})"
    else:
        source_param = {
            "video_path": PathJoinSubstitution([repo_root, LaunchConfiguration("video_path")])
        }
        source_desc = "video (本地文件)"

    params = dict(source_param)
    params["model_path"] = PathJoinSubstitution([repo_root, LaunchConfiguration("model_path")])
    params["detector"] = LaunchConfiguration("detector")
    if LaunchConfiguration("pose_model_path").perform(context):
        params["pose_model_path"] = PathJoinSubstitution(
            [repo_root, LaunchConfiguration("pose_model_path")]
        )

    actions.append(LogInfo(msg=f"[armor_tracker.launch] 图像源 = {source_desc}"))
    tracker_node = Node(
        package="rm_armor_visualization",
        executable="armor_tracker_node",
        name="armor_tracker_node",
        output="screen",
        parameters=[params],
    )
    actions.append(tracker_node)
    # 主节点一旦退出（例如 detector:=pose 但构建没启用 ONNX Runtime，或相机打不开），
    # 就让整个 launch 跟着退出——否则 rqt 窗口会空着，看起来像"启动了但没图像"，
    # 很难意识到节点其实已经死了。（Node 本身没有 required 参数，用事件处理器实现）
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=tracker_node,
                on_exit=[
                    LogInfo(
                        msg="[armor_tracker.launch] 主节点已退出，launch 一并退出。"
                        "常见原因：detector:=pose 但构建时未启用 ONNX Runtime；"
                        "或视频/模型/相机路径不对 —— 具体原因见上面节点打印的 ERROR。"
                    ),
                    EmitEvent(event=Shutdown(reason="主节点已退出")),
                ],
            )
        )
    )

    # ④ 可选：rqt 查看标注图
    actions.append(
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            arguments=["/armor/annotated"],
            condition=IfCondition(LaunchConfiguration("use_rqt")),
            output="screen",
        )
    )
    # ⑤ 可选：把 /armor/state 打到终端（省掉第二个终端跑 ros2 topic echo）
    actions.append(
        Node(
            package="rm_armor_visualization",
            executable="armor_state_printer",
            name="armor_state_printer",
            condition=IfCondition(LaunchConfiguration("print_state")),
            output="screen",
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("source", default_value="video", description="图像源: video | ip | hik"),
            DeclareLaunchArgument("video_path", default_value="data/demo.avi", description="source:=video 时的视频路径"),
            DeclareLaunchArgument("ip_url", default_value="", description="source:=ip 时的流地址(如 http://<IP>:8080/video)"),
            DeclareLaunchArgument("camera_config", default_value="data/camera.yaml", description="source:=hik 时的 yaml 配置"),
            DeclareLaunchArgument("model_path", default_value="models/armor_yolov8n.onnx", description="bbox 检测器模型路径"),
            DeclareLaunchArgument("detector", default_value="bbox", description="检测器: bbox | pose（四关键点）"),
            DeclareLaunchArgument("pose_model_path", default_value="", description="detector:=pose 时的四关键点模型路径"),
            DeclareLaunchArgument("repo_root", default_value=".", description="仓库根目录(相对路径的基准, 可给绝对路径)"),
            DeclareLaunchArgument("use_rqt", default_value="false", description="是否附带启动 rqt_image_view"),
            DeclareLaunchArgument("print_state", default_value="false", description="是否附带启动 armor_state_printer(把 /armor/state 打到终端)"),
            DeclareLaunchArgument("rmw", default_value="rmw_fastrtps_cpp", description="RMW 实现(本机需 FastDDS)"),
            DeclareLaunchArgument("mvs_lib_dir", default_value="/opt/MVS/lib/64", description="source:=hik 时的 MVS 库目录"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
