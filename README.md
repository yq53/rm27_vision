# rm27_vision — 崇实战队 27 赛季算法补录（视觉方向）

视觉方向三小题：装甲板识别（detector）→ 装甲板跟踪（tracker）→ 仿真/相机接入与可视化。
本仓库按题目分目录组织，每部分可独立构建运行。

```
rm27_vision/
├── models/             # 已导出的 ONNX 模型（训练工程见工作区 trainning/）
├── data/               # 测试素材（demo.avi：自瞄演示视频，1440x1080@30fps）
├── 01_detector/        # 题1：YOLOv8n 装甲板识别器（静态库 + 演示程序）
├── 02_tracker/         # 题2：装甲板跟踪器（进行中）
├── 03_visualization/   # 题3：ROS2 接入与可视化（进行中）
├── docs/
│   ├── notes.md        # 学习笔记（原理问答，供面试复习）
│   └── screenshots/    # 运行效果截图
└── results/            # 运行输出（已被 gitignore）
```

## 构建

```bash
# 在仓库根目录
cmake -S 01_detector -B build/01_detector -DCMAKE_BUILD_TYPE=Release
cmake --build build/01_detector -j
```

## 题1：装甲板识别器（detector）

方案：YOLOv8n 神经网络（自训，CPU 可用 OpenCV DNN 推理，无需 onnxruntime）。

- 模型：`models/armor_yolov8n.onnx`
  （训练自工作区 `trainning/`，exp1-3，单类别 `armour`，mAP50≈0.97；训练数据为自录视频抽帧标注）
- 库：`01_detector`（`rm_vision::ArmorDetector`，接口 `detect(frame) -> vector<Armor>`）
- 演示程序：`armor_demo`（读视频 → 逐帧检测 → 画框 → 存结果视频）

### 运行

```bash
./build/01_detector/armor_demo data/demo.avi                        # 全部帧（默认阈值 0.35/0.45）
./build/01_detector/armor_demo data/demo.avi models/armor_yolov8n.onnx 240   # 只跑前 240 帧
```

参数依次为：`<视频路径> [模型路径] [最大帧数]`，均可省略使用默认值
（`data/demo.avi`、`models/armor_yolov8n.onnx`、全部帧）。

输出：`results/detector_demo.avi`（标注视频）、`results/preview_*.png`（每 120 帧一张预览）。

> 注意：程序约定在**仓库根目录**下运行（默认相对路径 `data/`、`models/`）。

### 效果展示

全量 687 帧实测：**498 帧检出装甲板（72.5%）**，CPU 平均 **54.5 ms/帧（≈18 FPS）**。

| demo.avi 第 240 帧（检出 1 个） | demo.avi 第 600 帧（检出 2 个） |
|---|---|
| ![第240帧](docs/screenshots/detector_preview_0240.png) | ![第600帧](docs/screenshots/detector_preview_0600.png) |

部分帧因装甲板过小/模糊/镜头切换而漏检，属单帧检测的正常局限，题2 tracker 将结合时序信息弥补。

### 已知瑕疵与后续

- 暗光下模型可能把一个装甲板的两个灯条误检成两个装甲（两灯条框 IoU≈0，NMS 不会合并；根因在模型层）。
- 单帧漏检 → 交给题2 tracker 用跨帧信息处理。

---

## 题3：ROS2 接入与可视化（P0 已通）

把题1 detector 包装成 ROS2 节点：视频/相机帧 → 检测 → 发布标注图像话题。

```bash
# 构建（仓库根目录）
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_armor_visualization

# 终端1：运行节点（视频当图像源，循环播放）
source install/setup.bash
ros2 run rm_armor_visualization armor_video_node --ros-args -p video_path:=data/demo.avi

# 终端2：可视化（直接带话题参数，避免手动选择）
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /armor/annotated
```

> 沙箱环境提示：若 `~/.ros` 只读导致日志失败，先 `export ROS_LOG_DIR=$PWD/.roslog`。
> 参数：`video_path`（默认 data/demo.avi）、`model_path`（默认 models/armor_yolov8n.onnx）、`loop`（默认 true）。

> ⚠️ 环境备注：本机默认 RMW 为 CycloneDDS，传输大图像消息不稳定（大量 "message lost"，rqt 无画面）。
> **请使用 FastDDS**：`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（建议写入 `~/.bashrc`）。

---

## 题2 / 题3 完成状态（2026-09-08）

- **题2 tracker**：✅ v1 完成（detect→PnP→EKF 平滑，离线 `tracker_demo`：抖动 0.042→0.020 m）；v2 灯条精定位为探索原型（实验记录见 `docs/notes.md` 第 17 节）。
- **题3 ROS2 接入**：✅ P0/P1 完成——主节点发布**单条自定义状态** `/armor/state` + 标注图 `/armor/annotated`；真实相机（手机 IP Webcam）已验证。

### 题3 主节点运行（完整链路，推荐）

```bash
# 构建
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_armor_visualization
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 本机需 FastDDS（见下）

# 终端1：视频源 demo
ros2 run rm_armor_visualization armor_tracker_node --ros-args -p video_path:=data/demo.avi

# 终端1'：真实相机（手机 IP Webcam，地址以 App 显示为准，必须横屏！）
ros2 run rm_armor_visualization armor_tracker_node \
    --ros-args -p video_path:=http://<手机IP>:8080/video

# 终端2：查看
ros2 topic echo /armor/state --once
ros2 run rqt_image_view rqt_image_view /armor/annotated
```

**说明**：
- `rm_interfaces`：独立接口包，定义 `ArmorState.msg`（header+position+velocity+distance）。
- 节点角色：`armor_tracker_node`=主节点（完整链路）；`armor_video_node`=P0 学习示例（仅 detector）。
- **内参为演示级近似**（按分辨率 + 假定 HFOV≈72° 推导，未标定）→ 距离量级可信、绝对精度需棋盘标定；手机**必须横屏**（竖屏因旋转元数据会产生镜像假解）。
- 证据截图：`docs/screenshots/`（如有）。
