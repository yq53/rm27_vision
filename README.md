# rm27_vision — 崇实战队 27 赛季算法补录（视觉方向）

视觉方向三小题：**装甲板识别（detector）→ 装甲板跟踪（tracker）→ 接入仿真/真实相机 + 可视化**。
本仓库按题分目录组织，每部分可独立构建、运行、复现；学习过程与原理问答见 `docs/notes.md`（知识记录 + 工作日志）。

**三题完成状态（2026-09-09 复盘确认）**

| 题目 | 交付 | 状态 |
|---|---|---|
| 题1 detector | `01_detector/`：YOLOv8n 自训模型 + 静态库 + 演示程序 | ✅ 完成（含效果截图） |
| 题2 tracker | `02_tracker/`：PnP + 常速卡尔曼（`armor_ekf`）+ 平滑演示 | ✅ v1 完成；v2 灯条精定位=探索原型（冻结） |
| 题3 ROS2 接入 | `03_visualization/` + `rm_interfaces/`：完整链路节点 + 自定义消息 + 可视化 | ✅ 完成（含真实相机验证证据） |

## 目录结构

```
rm27_vision/
├── models/               # ONNX 模型（训练工程在工作区 trainning/，不入仓库）
├── data/                 # 测试素材 demo.avi + camera.yaml（相机配置，现场只改 serial_number）

├── 01_detector/          # 题1：装甲板识别器（armor_detector 库 + armor_demo 演示）
├── 02_tracker/           # 题2：装甲板跟踪器（armor_ekf 库 + 学习用 demo）
├── 03_visualization/     # 题3：ROS2 主节点 armor_tracker_node + P0 示例 armor_video_node
├── 04_hik/              # Hik SDK 学习 demo（probe/open/grab，不参与主构建）
├── rm_interfaces/        # 自定义消息接口包（ArmorState.msg）
├── docs/
│   ├── notes.md          # 学习笔记（原理问答 + 踩坑日志 + 工作记录）
│   └── screenshots/      # 运行效果截图（题1 预览 + 题3 真实相机证据）
└── results/              # 运行输出（生成物不入库，唯一例外=证据录屏 real_camera_*.mkv）
```

## 环境与前置

- Ubuntu 22.04 + ROS2 Humble + OpenCV 4.x + C++17（clang/GCC 均可）；ROS2 节点另需 `cv_bridge`。
- **RMW 必读**：本机默认 CycloneDDS 传大图像消息不稳定（rqt 无画面）→ 请用 FastDDS：
  `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（建议写入 `~/.bashrc`）。
- 程序约定在**仓库根目录**下运行（默认相对路径 `data/`、`models/`）。
- 若 `~/.ros` 只读导致日志报错：`export ROS_LOG_DIR=$PWD/.roslog`。

## 构建

```bash
# A. 题1 / 题2（普通 CMake，OpenCV）
cmake -S 01_detector -B build/01_detector -DCMAKE_BUILD_TYPE=Release && cmake --build build/01_detector -j
cmake -S 02_tracker -B build/02_tracker -DCMAKE_BUILD_TYPE=Release && cmake --build build/02_tracker -j

# B. 题3（ROS2 colcon；rm_interfaces 会被自动带上）
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_armor_visualization
source install/setup.bash
```

> 02_tracker / 03_visualization 通过 `add_subdirectory` 内部复用 `01_detector`，无需单独安装。

---

## 题1：装甲板识别器（detector）

方案：**YOLOv8n 神经网络（自训）**，CPU 上用 OpenCV DNN 推理（无需 onnxruntime / GPU）。
训练数据为自己录制的比赛/演示视频抽帧标注（协议：`demo.avi` 只当测试集，**永不进训练**，详见 notes §16）。

- 模型：`models/armor_yolov8n.onnx`（单类别 `armour`，mAP50≈0.97；`_dark` 为暗化增强微调实验模型，负结果存档，勿用）
- 库：`01_detector` → `rm_vision::ArmorDetector`，接口 `detect(frame) -> vector<Armor>`（阈值 0.35 / NMS 0.45，可调）
- 演示：`armor_demo`（读视频 → 逐帧检测 → 画框 → 存结果）

### 运行

```bash
./build/01_detector/armor_demo                                          # 全部帧（默认参数）
./build/01_detector/armor_demo data/demo.avi models/armor_yolov8n.onnx 240   # 指定只跑前 240 帧
```

参数：`<视频路径> [模型路径] [最大帧数]`，均可省略。输出 `results/detector_demo.avi` + `results/preview_*.png`（每 120 帧一张）。

### 效果与指标

全量 687 帧实测：**498 帧检出装甲板（72.5%）**，CPU 平均 **54.5 ms/帧（≈18 FPS）**。

| demo.avi 第 240 帧（检出 1 个） | demo.avi 第 600 帧（检出 2 个） |
|---|---|
| ![第240帧](docs/screenshots/detector_preview_0240.png) | ![第600帧](docs/screenshots/detector_preview_0600.png) |

### 已知局限（诚实声明）

- 暗光下可能把一个装甲板的两个灯条误检成两个装甲（两框 IoU≈0，NMS 不合并；根因在模型层，见 notes §15）。
- 过小/模糊/镜头切换会漏检——单帧检测的正常局限，正是题2 tracker 用跨帧信息弥补的场景。

---

## 题2：装甲板跟踪器（tracker）

方案（v1，社区主流 PnP + 滤波路线）：YOLO 框 → 四角点 → **PnP** 解板心 3D 位姿 → **常速卡尔曼滤波**平滑。
滤波为"EKF 框架、当前模型线性故退化为 KF"（测量=PnP 解出的 3D 位置；漏检帧只 predict 不 update）。
后端不限：若将来测量换成像素角点/状态加角度，只需把常量 F/H 换成雅可比（类骨架已预留，见 `armor_ekf.hpp`）。

- 库：`02_tracker` → `rm_tracker::ArmorEKF`（6D 状态 pos+vel，`init/predict/update`，Q/R 可调）
- 演示：`tracker_demo`（绿=原始 PnP 距离、黄=EKF+速度）、`pnp_demo`（PnP 合成闭环 + 真实数据叠加）、
  `corner_demo`（框→四角点）、`projection_demo`（3D→2D 正投影教学）

### 运行

```bash
# 平滑演示（抖动对比 + 距离/速度叠加）
./build/02_tracker/tracker_demo [视频] [模型] [最大帧数]

# PnP 闭环验证（A 段合成：干净数据 dist/yaw 误差≈0；B 段真实数据叠加测距）
./build/02_tracker/pnp_demo [视频] [模型] [最大帧数]
```

### 指标与证据

- `tracker_demo` 300 帧：相邻帧距离跳动 **raw 0.042 → EKF 0.020 m（抖减半）**；
  平均距离 raw 0.654 vs EKF 0.668 m（平滑不掉真值）。输出 `results/tracker_demo.avi`。
- `pnp_demo` A 段合成闭环：干净数据距离/偏航解算误差≈0，重投影≈0 px（数字可复现，见 notes §11.6）。
- PnP 防伪解三层：IPPE 多解 + r₃.z>0 破镜像 + 重投影误差>10px 拒绝；物理闸门（z>0.2 且 dist<20m）兜底。

### v2 探索（已冻结为原型）

框内**灯条精定位**（灯条端点 → 更准板角 → PnP 更准，顺带修题1 双灯条误检）：
`light_bar_detector` 库 + `lightbar_demo` 已实现配对/合并逻辑，但 demo.avi 极端偏航下配对率仅 ~7%，
继续投入需真机/定向数据支撑 → 冻结，设计决策与三级容错方案记录在 notes §15。

### 已知局限（v1 简化，理由见 notes §14.3）

- 单目标：每帧挑**面积最大**的板（≈最近）→ 两块板交替最大时会跳目标（多目标=MOT：数据关联+每目标一个滤波器，未做）。
- 无装甲数字识别、无整车位姿估计、无目标预测（均属 v2 之后的加分项）。

---

## 题3：ROS2 接入与可视化

架构：图像源（视频/真实相机）→ detector → PnP → EKF → **单条自定义状态** `/armor/state` + 标注图 `/armor/annotated`。

```
03_visualization/
├── armor_tracker_node   # 主节点（完整链路：detect→PnP→EKF→ArmorState + 标注图）
├── armor_video_node     # P0 教学示例（仅 detector → 标注图，保留作对比学习）
rm_interfaces/
└── msg/ArmorState.msg   # std_msgs/Header header; Point position; Vector3 velocity; float64 distance
```

### 一键启动（推荐，launch）

```bash
cd <仓库根目录> && source install/setup.bash

# 默认：视频源(data/demo.avi) + 主节点
ros2 launch rm_armor_visualization armor_tracker.launch.py

# 手机 IP Webcam（横屏；ip_url 必须带 /video）
ros2 launch rm_armor_visualization armor_tracker.launch.py source:=ip ip_url:=http://<手机IP>:8080/video

# 现场海康（改 data/camera.yaml 的 serial_number；需先用 USE_HIK_SDK=ON 构建）
ros2 launch rm_armor_visualization armor_tracker.launch.py source:=hik

# 附带 rqt 看图
ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true

# 不在仓库根目录时：用 repo_root 指定绝对路径
ros2 launch rm_armor_visualization armor_tracker.launch.py repo_root:=$HOME/my_project/ws_exam/rm27_vision
```

- `source` 是唯一入口参数（video/ip/hik），launch 自动映射成节点的 `video_path` / `camera_config`；
- launch 会自动设置 `RMW_IMPLEMENTATION`（及 hik 模式的 `LD_LIBRARY_PATH`），**仅对本次启动的进程生效，不写入 ~/.bashrc**；
- 其余参数：`video_path` `ip_url` `camera_config` `model_path` `repo_root`(默认 `.`) `use_rqt` `rmw` `mvs_lib_dir`。
- 已实测（2026-09-10）：默认 video / 异地启动+`repo_root` / `source:=ip` 缺参报错 / `source:=hik` 无相机报错 / `use_rqt:=true` 弹窗看图 ✅。

### 运行（完整链路·手动分终端）



```bash
# 构建
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_armor_visualization
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 本机必用 FastDDS（见上）

# 终端1：视频源（循环播放）
ros2 run rm_armor_visualization armor_tracker_node --ros-args -p video_path:=data/demo.avi

# 终端1''：海康相机（现场：改 data/camera.yaml 的 serial_number 即可）
#   需先以 USE_HIK_SDK=ON 构建：
#   colcon build --packages-up-to rm_armor_visualization --cmake-args -DUSE_HIK_SDK=ON
#   运行时: export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
ros2 run rm_armor_visualization armor_tracker_node --ros-args -p camera_config:=data/camera.yaml

# 终端1'：真实相机（手机 IP Webcam——必须横屏！地址以 App 显示为准）
ros2 run rm_armor_visualization armor_tracker_node \
    --ros-args -p video_path:=http://<手机IP>:8080/video

# 终端2：查看状态与标注图
ros2 topic echo /armor/state --once
ros2 run rqt_image_view rqt_image_view /armor/annotated
```

> 手机流地址**必须带 `/video`**（裸地址是网页，VideoCapture 打不开）；竖屏会产生 90° 旋转元数据 → PnP 镜像假解（z<0），**务必横屏**。
> `armor_video_node` 用法相同，仅发布标注图（参数 `video_path/model_path/loop`，默认 loop=true）。

### 真实相机验证证据（2026-09-09）

![手机 IP Webcam 实拍屏幕 + rqt 实时标注](docs/screenshots/phone_rqt.png)

- 截图：`docs/screenshots/phone_rqt.png`（检测框锁定目标，`d=1.05m v=(0.5,0.2,1.3)` 实时输出）；
- 录屏：`results/real_camera_2026-09-09.mkv`（24 s：手机实拍 → detect → PnP → EKF → 可视化全程）；
- 内容决策：靶面用 `data/demo.avi`（**非训练集、光照差、角度极端**）回放——苛刻条件下仍持续锁定，
  比理想素材更能证明真实鲁棒性；d≈0.8~1.1 m 与手机到屏幕距离同量级（内参为近似，见下）。

### 已知局限

- 内参为**演示级近似**：按分辨率 + 假定 HFOV≈72° 推导（`computeK`），未棋盘标定 → 距离量级可信、绝对精度需标定。
- 可视化为 rqt 2D 标注（满足"可视化界面"要求）；未接仿真器（题面"仿真/真实相机"二选一，走通真实相机路线；河科仿真器为后续可选项）。
- ROS2 踩坑（RMW/QoS/自定义消息/横屏）逐条记录在 notes §17.4。

---

## FAQ（环境与常见问题）

| 现象 | 原因 / 解法 |
|---|---|
| rqt 无画面 / topic 大量 message lost | CycloneDDS 大图像不稳 → `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| rqt 打开后是渐变占位图 | 手动下拉常没真正订阅 → 直接带话题：`rqt_image_view rqt_image_view /armor/annotated` |
| 手机流"Stream ends prematurely" | URL 少了 `/video` 后缀 |
| 检测框在但 z<0 / 距离乱跳 | 手机竖屏（旋转元数据）→ 横屏解决 |
| 距离绝对值和真实差一截 | 内参未标定（HFOV≈72° 近似）→ 只论量级；标定见 notes §8.11 |

## 参考资料对照

考核题面给的开源参考（自瞄能力链：识别模型 → 部署框架 → 仿真 → 整车系统）中，
本工程学习时对照阅读了：同济 SuperPower `sp_vision_25`（灯条端点 3D 建模、板尺寸 135×125/230×127、灯条 56mm 出处——本地 `reference/` 有源码）、
深大/河科 keypoint 类识别模型概念、河科视觉仿真器（内参精确已知的思路）。
踩坑、设计决策、负结果的完整记录都在 `docs/notes.md`。

## 修订记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | 2026-09-08 | 三题初稿运行说明 + 证据截图 |
| **v2.0** | 2026-09-09 | 复盘后重构：三题并列结构、状态总览表、补题2 运行/指标、FAQ、参考资料对照；修复录屏死链（.gitignore 白名单） |
