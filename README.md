# rm27_vision — 崇实战队 27 赛季算法组补录考核（视觉方向）

视觉方向三小题的完整实现：**装甲板识别（detector）→ 装甲板跟踪（tracker）→ 接入真实相机 + 可视化**。
按题分目录组织，每部分可独立构建、运行、复现；学习过程、原理问答、踩坑与负结果的完整记录见
`docs/notes.md`（知识记录 + 工作日志，21 章）。

- 代码：C++17 + OpenCV（题1/题2 为普通 CMake，题3 为 ROS2 Humble ament/colcon）
- 素材：`data/demo.avi`（687 帧，自录，**只作测试集、永不进训练**）
- 评测：`eval_demo` 统一口径离线评测（逐帧 CSV + 汇总），所有指标数字可复现

## 考核要求对照

| 题 | 题面要求 | 本仓库交付 | 状态 | 证据 |
|---|---|---|---|---|
| 题1 detector | 识别装甲板；传统视觉或神经网络方案均可，不要求高鲁棒性 | `01_detector/`：自训 YOLOv8n bbox 检测器（默认）+ 深大四关键点检测器（可选）两种实现，同一接口 | ✅ | 687 帧检出 **72.49%（bbox）/ 90.25%（关键点）**；`docs/screenshots/` |
| 题2 tracker | 单板跟踪或整车估计；PnP+EKF、ESEKF、MCSKF、因子图等后端不限 | `02_tracker/`：PnP（IPPE 多解 + 破镜像 + 双重闸门）+ 常速卡尔曼；另附离线评测基础设施 | ✅ | `results/eval_*_summary.txt`（逐帧 CSV 可复算）；平滑抖动 **0.042→0.020 m** |
| 题3 接入与可视化 | 可接入仿真、真实相机验证，并有可视化界面（OpenGL/QT、web、foxglove、rerun 等） | `03_visualization/` + `rm_interfaces/`：图像源抽象（视频 / 网络流 / 海康工业相机）+ 完整链路 ROS2 节点 + 自定义状态话题 + rqt 2D 标注 | ✅ 真实相机验证 | `results/real_camera_2026-09-09.mkv`、`docs/screenshots/phone_rqt.png` |
| 导航方向 | 运动控制 / A-B / 路径规划（4 小题） | 未做（题面注明视觉与导航不强求全部完成，本仓库聚焦视觉方向） | — | — |

> 三题通过同一套「2D 点 → PnP → EKF」主链路串起来：题1 决定 2D 点从哪来，题2 决定怎么解与怎么平滑，
> 题3 决定数据从哪来、结果给谁看。这也是本工程的组织方式——**一题一目录，但共享同一份核心库**。

## 目录结构

```
rm27_vision/
├── models/                    # ONNX 模型（自训 bbox 模型；第三方权重不入库，见「许可说明」）
├── data/                      # demo.avi 测试素材 + camera.yaml 相机配置（现场只改 serial_number）
│
├── 01_detector/               # 题1：检测器
│   ├── include/armor_detector/        #   rm_vision::ArmorDetector（bbox）
│   ├── include/armor_pose_detector/   #   rm_vision::ArmorPoseDetector（四关键点，可选 ORT 后端）
│   ├── src/                           #   两个库实现 + demo_main
│   └── scripts/convert_fp16_to_fp32.py
├── 02_tracker/                # 题2：跟踪器 + 评测基础设施
│   ├── include/armor_pnp/             #   PnP 共用解算（位姿 + 重投影误差 + yaw）
│   ├── include/armor_ekf/             #   rm_tracker::ArmorEKF（6D pos+vel）
│   ├── include/armor_corner/          #   v2 角点精修原型（负结果，默认关闭）
│   ├── include/armor_tracker/         #   v2 灯条精定位原型（负结果，已冻结）
│   ├── src/{projection,corner,pnp,tracker,lightbar}_demo.cpp
│   ├── src/eval_demo.cpp              #   离线评测主程序
│   └── scripts/compare_eval.py        #   多 tag 汇总对照表
├── 03_visualization/          # 题3：ROS2 接入与可视化
│   ├── src/armor_tracker_node.cpp     #   主节点（完整链路）
│   ├── src/armor_video_node.cpp       #   P0 教学示例（仅 detector → 标注图）
│   ├── src/image_source.{hpp,cpp}     #   图像源抽象（video / ip / hik）
│   └── launch/armor_tracker.launch.py #   一键启动
├── 04_hik/                    # 海康 MVS SDK 学习 demo（probe/open/grab，独立构建，不参与主构建）
├── rm_interfaces/             # 自定义消息接口包（ArmorState.msg）
├── docs/
│   ├── notes.md               # 学习笔记（21 章：原理问答 + 踩坑 + 负结果 + 工作记录）
│   └── screenshots/           # 运行效果截图（题1 预览 + 题3 真实相机证据）
└── results/                   # 运行输出（生成物不入库；唯一例外=证据录屏 real_camera_2026-09-09.mkv）
```

## 环境与依赖

**必备**

- Ubuntu 22.04 · ROS2 Humble · OpenCV 4.x · C++17（clang / GCC 均可）· CMake ≥ 3.16
- 题3 另需 `cv_bridge`（随 ROS2 desktop 安装）

**可选（按需开启，默认全关，不开也能构建）**

| 依赖 | 用途 | 开启方式 |
|---|---|---|
| ONNX Runtime ≥ 1.16（C++，解压即用） | 四关键点模型后端（`detector:=pose`） | `-DUSE_ONNXRUNTIME=ON -DONNXRUNTIME_DIR=<ort 根目录>` |
| 海康 MVS SDK 5.0.2（`/opt/MVS`） | 海康工业相机图像源（`source:=hik`） | `-DUSE_HIK_SDK=ON`（可加 `-DMVS_SDK_DIR=...`） |
| `rqt_image_view` | 查看标注图 | `sudo apt install ros-humble-rqt-image-view` |

**运行约定**

- 程序约定在**仓库根目录**下运行（默认相对路径 `data/`、`models/`）；不在根目录时用 launch 的 `repo_root:=绝对路径`。
- **RMW 必读**：本机默认 CycloneDDS 传大图像消息不稳定（rqt 无画面）→ 用 FastDDS：
  `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（launch 已自动设置，仅对本次启动生效）。
- 若 `~/.ros` 只读导致日志报错：`export ROS_LOG_DIR=$PWD/.roslog`。
- ⚠️ 本文档命令中的示例值（路径 / IP）**请替换成你自己的**。不要直接粘贴带尖括号的占位符——
  bash 会把 `<` 当成输入重定向而报语法错误。正确做法是先设变量，例如：

  ```bash
  ORT_DIR=$HOME/onnxruntime-linux-x64-1.23.2
  POSE_MODEL=$HOME/models/Infantry-v8n-fp16.onnx
  ```

## 构建

```bash
# A. 题1 / 题2：普通 CMake，只需 OpenCV
cmake -S 01_detector -B build/01_detector -DCMAKE_BUILD_TYPE=Release && cmake --build build/01_detector -j
cmake -S 02_tracker  -B build/02_tracker  -DCMAKE_BUILD_TYPE=Release && cmake --build build/02_tracker  -j

# B. 题3：ROS2 colcon（默认配置；不引入 ORT / MVS 依赖）
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_armor_visualization
source install/setup.bash

# C. 额外启用四关键点模型（ONNX Runtime 后端）
colcon build --packages-up-to rm_armor_visualization \
    --cmake-args -DUSE_ONNXRUNTIME=ON -DONNXRUNTIME_DIR="$ORT_DIR"

# D. 额外启用海康相机（MVS SDK）
colcon build --packages-up-to rm_armor_visualization --cmake-args -DUSE_HIK_SDK=ON

# E. 04_hik 学习 demo（独立工程）
cmake -S 04_hik -B build/04_hik && cmake --build build/04_hik -j
```

- 02_tracker / 03_visualization 通过 `add_subdirectory` 内部复用 `01_detector`，无需单独安装。
- C 与 D 可同时给（同一行写两个 `-D`）；两个开关都是 CMake `option()`，默认 OFF——
  **没装 ONNX Runtime 或 MVS SDK 的人也能完整构建并跑通默认链路**。

---

## 题1：装甲板识别器（detector）

两个检测器实现同一个概念接口（`detect(frame) -> 一组带 rect 的目标`），可按需切换，互不影响：

- `rm_vision::ArmorDetector`：轴对齐 bbox，角点由框四角近似（**默认**）
- `rm_vision::ArmorPoseDetector`：4 个灯条端点，可直接用于 PnP，无需框角近似（**可选**）

### 方案 A：自训 YOLOv8n bbox 检测器（默认）

CPU 上用 OpenCV DNN 推理（**无需 onnxruntime、无需 GPU**）。训练数据为自己录制的比赛/演示视频抽帧标注，
协议：`data/demo.avi` 只当测试集、**永不进训练**（域差实验与协议见 notes §16）。

- 模型：`models/armor_yolov8n.onnx`（单类别 `armour`，640×640 输入，验证集 mAP50≈0.97）
- `models/armor_yolov8n_dark.onnx`：暗化增强微调实验模型，**负结果存档，勿用**（mAP50 保持 0.974 但跨域无提升，见 §16）
- 接口：`01_detector/include/armor_detector/`，阈值 0.35 / NMS 0.45（可调）
- 演示：`armor_demo`（读视频 → 逐帧检测 → 画框 → 存结果视频）

```bash
./build/01_detector/armor_demo                                          # 默认参数跑全部帧
./build/01_detector/armor_demo data/demo.avi models/armor_yolov8n.onnx 240   # 只跑前 240 帧
```

参数：`<视频路径> [模型路径] [最大帧数]`，均可省略。输出 `results/detector_demo.avi` + `results/preview_*.png`（每 120 帧一张）。

**效果与指标**：全量 687 帧，**498 帧检出装甲板（72.49%）**；`armor_demo` 端到端（含画框与写视频）
平均 **54.5 ms/帧 ≈ 18 FPS**；同口径下纯 `detect()` 耗时 **37.96 ms**（见题2 评测表）。

| demo.avi 第 240 帧（1 个目标） | demo.avi 第 600 帧（2 个目标） |
|---|---|
| ![第240帧](docs/screenshots/detector_preview_0240.png) | ![第600帧](docs/screenshots/detector_preview_0600.png) |

**已知局限（诚实声明）**

- 暗光下可能把一个装甲板的两个灯条误检成两个装甲（两框 IoU≈0，NMS 不合并；根因在模型层，见 notes §15）。
- 过小 / 模糊 / 镜头切换会漏检——单帧检测的正常局限，正是题2 tracker 用跨帧信息弥补的场景。

### 方案 B：四关键点模型（可选，直出灯条端点）

模型直接回归 4 个**灯条端点**，绕过"bbox 四角近似"这一步，从根上提高 PnP 的角点质量。

- **来源与许可**：深大 RobotPilots 开源的 `Infantry-v8n`（YOLOv8n-Pose 重设计头）。
  仓库标称 MIT，但**ONNX 元数据标注 AGPL-3.0（Ultralytics）** → 本仓库**不提交该权重**，请自行下载。
- **输入几何**：`480×640`（4:3，与相机/素材同比例 → letterbox 退化为纯缩放，无灰边）；
  对比自训 bbox 模型的 `640×640`（4:3 素材需上下各补 80 行灰边，画布 25% 是废像素）。
- **输出布局**（实测确认，`21 × 6300`）：`row4..12` = 9 个类别分数，`row13..20` = 4 个关键点 (x,y)，
  无逐点置信度；框由 `boundingRect(4 点)` 推出。
- **关键点语义**：4 个点是**两根灯条的端点，不是板四角** → PnP 物体模型必须用 `135mm × 56mm`
  （`barEndObjectPoints`），索引映射 `kp0/kp3/kp2/kp1 → TL/TR/BR/BL`。
  用错的代价：板四角模型重投影 8.09 px、反向绕向 25.64 px，正确为 **1.09 px**（详见 notes §20.2）。
- **推理后端**：该导出图**在 OpenCV 4.x 的 cv2.dnn 上不可用**（入口 Cast 节点解析失败；转 fp32 后改为
  解码段 `NaryEltwise` 广播断言失败）→ 必须用 **ONNX Runtime** 构建（构建方式见上文 C）。
  注意：ORT 使用**原始导出件**，`scripts/convert_fp16_to_fp32.py` 的产物只适用于 cv2.dnn。
  未启用 ORT 的构建若被要求 `detector:=pose`，会在构造检测器时**明确抛异常并提示重新构建**，不会静默失效。

运行方式见「题3 → 检测器选择」；A/B 的量化对比见「题2 → 687 帧实测」。

---

## 题2：装甲板跟踪器（tracker）

### v1 主链路：PnP + 卡尔曼

```
bbox/关键点 → 2D 点(4) → PnP → 板心 3D 位置 → 卡尔曼滤波 → 状态输出
```

- **PnP**（`rm_tracker::solveArmorPose`，IPPE 多解）三层防伪解 + 一层物理兜底：
  1. `SOLVEPNP_IPPE` 出多个候选解；
  2. 优先保留 `r₃.z > 0`（板正面朝相机）的候选，破镜像解；
  3. 候选中取重投影误差最小者，且**误差 > 10 px 直接拒绝**；
  4. 物理闸门：`z > 0.2 m 且 dist < 20 m`，否则视为伪解丢弃。
  输出 `PoseResult{tvec, reproj_err_px, yaw_deg}`——后两项供评测量化角点质量。
- **滤波**（`rm_tracker::ArmorEKF`）：6D 状态（位置 + 速度），常速模型。
  当前模型线性，故 EKF 退化为 KF；漏检帧只 `predict` 不 `update`。
  骨架按 EKF 写（常量 F/H 换成雅可比即可换测量模型），后端不受限。
- **演示**：`tracker_demo`（绿=原始 PnP、黄=EKF，看平滑效果）、`pnp_demo`（A 段合成闭环 + B 段真实数据测距）、
  `corner_demo`（框 → 四角点）、`projection_demo`（3D→2D 正投影教学）。

```bash
./build/02_tracker/tracker_demo [视频] [模型] [最大帧数]   # 平滑演示：抖动对比 + 距离/速度叠加
./build/02_tracker/pnp_demo     [视频] [模型] [最大帧数]   # PnP 闭环验证（合成段误差≈0）
```

**指标**：`tracker_demo` 300 帧，相邻帧距离跳动 **raw 0.042 → EKF 0.020 m（抖动减半）**，
平均距离 raw 0.654 vs EKF 0.668 m（平滑不掉真值）；`pnp_demo` A 段合成数据距离/偏航误差≈0、重投影≈0 px
（推导见 notes §11.6）。

### 离线评测基础设施（`eval_demo`）

所有"改进是否有效"的结论都建立在这套同口径评测上——**口径不变才可比**。

```bash
./build/02_tracker/eval_demo <视频> <模型> <最大帧数> <tag> <corner_mode:bbox|refine> <detector:bbox|pose>
python3 02_tracker/scripts/compare_eval.py baseline pose      # 多 tag 并排对照表
```

（参数顺序固定；`pose` 是**第 6 个**参数，`<corner_mode>` 是第 5 个，传错会拿 pose 模型去构造 bbox 检测器。）

- 产物：`results/eval_<tag>.csv`（逐帧）+ `results/eval_<tag>_summary.txt`（汇总）
- 指标：检出率、PnP 通过率、重投影误差、原始/EKF 距离与抖动、检测耗时、偏航角
- **口径定义**（notes §20.4）：
  - `jitter` = **相邻且帧号连续**的有效帧之间的距离差分均值 `|d_i − d_{i−1}|`，样本数 `n` 一并输出
    （跨漏检空档不计、首帧不计——避免把"漏检造成的跳变"算成抖动）；
  - `NaN` = "本帧无有效值"（与真实的 0 区分）；`numeric_limits::max()` 作比较哨兵、`quiet_NaN()` 作缺失标记，二者不混用；
  - `refined` 列在 bbox 模式表示"角点精修成功"，在 pose 模式表示"使用了模型端点"；
  - 检测器阈值与物体模型配对在 `eval_demo` 与主节点中**保持一致**（pose 0.5 / bbox 0.35；pose 用灯条端点模型），
    因此评测数字描述的就是节点行为。

### 687 帧实测：bbox 基线 vs 四关键点（同一 EKF、同一口径）

`data/demo.avi` 687 帧，唯一变量是"2D 点从哪来"：

| 指标 | bbox 基线 | 四关键点 | Δ |
|---|---|---|---|
| 检出帧 | 498（72.49%） | **620（90.25%）** | +17.76 pt |
| PnP 通过 | 415（60.41%） | **604（87.92%）** | +27.51 pt |
| 重投影误差（mean / median） | 4.11 / 3.15 px | **1.19 / 0.82 px** | −2.92 px |
| 距离（raw / EKF） | 0.894 / 1.058 m | 0.964 / 0.946 m | — |
| 抖动 raw | 0.0853 m | **0.0322 m** | −62% |
| 抖动 EKF | 0.0555 m | **0.0219 m** | −60% |
| 偏航角 mean\|·\| | 18.28° | 13.27° | −5.0° |
| 检测耗时 | 37.96 ms | **26.81 ms** | −11.1 ms |

原始数据：`results/eval_baseline_summary.txt`、`results/eval_pose_summary.txt`（逐帧 CSV 在 `results/eval_*.csv`）。

### v2 探索与负结果（两条路都主动放弃）

两条"用传统视觉拿到更准角点"的路线都做了、测了、否掉了，代码留在仓库里作为决策证据：

| 路线 | 做法 | 结果 | 处置 |
|---|---|---|---|
| `light_bar_detector` + `lightbar_demo`（notes §15） | 框内阈值 + 连通块 + 灯条配对拼板 | demo.avi **配对率仅 7%**：板大多大角度侧转，两灯条在画面里几乎重叠成一块 | **冻结**（设计决策与三级容错方案留档） |
| `armor_corner`（notes §21.1） | ROI → Otsu/轮廓 → PCA 主轴取端点 → 亮度梯度修正 → 四边形合理性闸门 | 中间档闸门下仅 **0.4% 的检出帧**精修成功（`bars_mean≈0.92`，大偏航常只可见**单根**灯条）；严格档 0% | **默认关闭**，不接入主节点 |

- `armor_corner` 的实测截图（ROI / 二值图 / 叠加）需 `RM_CORNER_DEBUG=1` 才会输出到 `results/corner_dbg_*.png`，**默认关闭**。
- **它不影响交付效果**：主节点根本不链接 `armor_corner`（grep 零引用），唯一入口是 `eval_demo` 的
  `corner_mode=refine` 且**默认值为 `bbox`**。保留它是为了留下"为什么转向神经网络"的可复现证据。
- 结论直接成为方案 B 的动机：既然传统法在极端偏航失效，就用网络直出灯条端点。

### 已知局限（v1 简化）

- **单目标**：每帧挑**面积最大**的板（≈最近）。两块板交替最大时会跳目标——多目标跟踪需要数据关联 +
  每目标一个滤波器，未做（理由见 notes §14.3）。
- 无装甲数字识别、无整车位姿估计、无目标预测；内参未标定（见题3 局限）。

---

## 题3：ROS2 接入与可视化

### 架构

```
图像源(视频 / 网络流 / 海康相机) → detector(bbox | pose) → PnP → EKF
        │                                                        │
        └──────────────► /armor/annotated (标注图)  ◄────────────┘
                          /armor/state (自定义状态)
```

| 文件 | 职责 |
|---|---|
| `03_visualization/src/armor_tracker_node.cpp` | 主节点：完整链路，发布 `/armor/state` + `/armor/annotated` |
| `03_visualization/src/image_source.{hpp,cpp}` | 图像源抽象（`ImageSource` 接口 + 工厂）；海康代码在文件内 `#ifdef RM_USE_HIK_SDK` 段 |
| `03_visualization/src/armor_video_node.cpp` | P0 教学示例：仅 detector → 标注图，保留作对比学习 |
| `rm_interfaces/msg/ArmorState.msg` | `std_msgs/Header header; Point position; Vector3 velocity; float64 distance` |
| `03_visualization/launch/armor_tracker.launch.py` | 一键启动（含环境变量自动化） |

### 一键启动（推荐）

```bash
cd <仓库根目录> && source install/setup.bash

# 默认：视频源(data/demo.avi) + 主节点
ros2 launch rm_armor_visualization armor_tracker.launch.py

# 手机 IP Webcam（必须横屏；ip_url 必须带 /video）
ros2 launch rm_armor_visualization armor_tracker.launch.py source:=ip ip_url:=http://192.168.1.10:8080/video

# 现场海康（先改 data/camera.yaml 的 serial_number；需以 USE_HIK_SDK=ON 构建）
ros2 launch rm_armor_visualization armor_tracker.launch.py source:=hik

# 附带 rqt 看图
ros2 launch rm_armor_visualization armor_tracker.launch.py use_rqt:=true

# 不在仓库根目录时：用 repo_root 指定绝对路径
ros2 launch rm_armor_visualization armor_tracker.launch.py repo_root:=$HOME/my_project/ws_exam/rm27_vision
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `source` | `video` | 唯一入口参数：`video` / `ip` / `hik`，launch 映射成节点的 `video_path` / `camera_config` |
| `video_path` | `data/demo.avi` | `source:=video` 时的视频路径 |
| `ip_url` | 空 | `source:=ip` 时的流地址（缺参会直接报错） |
| `camera_config` | `data/camera.yaml` | `source:=hik` 时的 yaml 配置 |
| `model_path` | `models/armor_yolov8n.onnx` | bbox 检测器模型 |
| `detector` | `bbox` | `bbox` / `pose` |
| `pose_model_path` | 空 | `detector:=pose` 时必填 |
| `repo_root` | `.` | 相对路径基准，可给绝对路径 |
| `use_rqt` / `rmw` / `mvs_lib_dir` | `false` / `rmw_fastrtps_cpp` / `/opt/MVS/lib/64` | 可视化 / RMW / MVS 库目录 |

已实测（2026-09-10）：默认 video ✅ / 异地启动 + `repo_root` ✅ / `source:=ip` 缺参报错 ✅ /
`source:=hik` 无相机报错 ✅ / `use_rqt:=true` 弹窗看图 ✅。

### 手动分终端（完整链路）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_armor_visualization && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 终端1：视频源（循环播放）
ros2 run rm_armor_visualization armor_tracker_node --ros-args -p video_path:=data/demo.avi

# 终端1'：手机 IP Webcam（必须横屏，地址以 App 显示为准）
ros2 run rm_armor_visualization armor_tracker_node \
    --ros-args -p video_path:=http://192.168.1.10:8080/video

# 终端2：查看状态与标注图
ros2 topic echo /armor/state --once
ros2 run rqt_image_view rqt_image_view /armor/annotated
```

> 手机流地址**必须带 `/video`**（裸地址是网页，VideoCapture 打不开）；竖屏会产生 90° 旋转元数据 →
> PnP 镜像假解（z<0），**务必横屏**。`armor_video_node` 用法相同，仅发布标注图（默认 `loop=true`）。

### 检测器选择（`detector:=bbox|pose`）

默认仍用自训 bbox 模型；切到四关键点模型（模型说明见「题1 方案 B」）：

```bash
ORT_DIR=$HOME/onnxruntime-linux-x64-1.23.2                       # ORT 解压目录（构建时 -DONNXRUNTIME_DIR 同一个）
POSE_MODEL=$HOME/models/Infantry-v8n-fp16-20260726.onnx          # 四关键点原始导出件

ros2 run rm_armor_visualization armor_tracker_node --ros-args \
    -p video_path:=data/demo.avi -p detector:=pose -p pose_model_path:="$POSE_MODEL"
# 或
ros2 launch rm_armor_visualization armor_tracker.launch.py detector:=pose pose_model_path:="$POSE_MODEL"
```

切换只影响"2D 点从哪来"：节点内部两条路都归到同一组变量（`target_rect` / `target_corners` / `has_target`），
之后的 PnP → EKF → 发布**完全共用**。

### 真实相机验证证据（2026-09-09）

![手机 IP Webcam 实拍屏幕 + rqt 实时标注](docs/screenshots/phone_rqt.png)

- 截图 `docs/screenshots/phone_rqt.png`：检测框锁定目标，`d=1.05m v=(0.5,0.2,1.3)` 实时输出；
- 录屏 `results/real_camera_2026-09-09.mkv`（24 s）：手机实拍 → detect → PnP → EKF → 可视化全程；
- 内容决策：靶面用 `data/demo.avi`（**非训练集、光照差、角度极端**）回放——苛刻条件下仍持续锁定，
  比理想素材更能证明真实鲁棒性；`d≈0.8~1.1 m` 与手机到屏幕的实际距离同量级（内参为近似，见局限）。

### 可视化

- **rqt 2D 标注**（当前交付形态，满足题面"可视化界面"要求）：`/armor/annotated` 话题上叠加检测框、
  距离与速度文字；`/armor/state` 可 `ros2 topic echo` 或接 PlotJuggler。
- **Foxglove**：自定义消息 + 标准 `sensor_msgs/Image` 可直接被 Foxglove 订阅做 3D 位姿与曲线显示；
  P0 阶段曾用它做过 `/armor/state → 3D 位姿` 的展示节点，因不属于题面必需、且要动稳定文件而**回档移除**
  （notes §17.4），需要时可作为扩展重新接上。
- 未接仿真器：题面"仿真 / 真实相机"二选一，本工程走通**真实相机**路线；河科视觉仿真器为后续可选项。

### 已知局限

- **内参为演示级近似**：按分辨率 + 假定水平 FOV≈72° 推导（`computeK`），未做棋盘格标定 →
  距离量级可信、绝对精度需标定（标定方案见 notes §8.11）。
- **可视化带宽**：1440×1080×3 ≈ 4.67 MB/帧，30 Hz ≈ 140 MB/s，本机 rqt 订阅端吃力（表现为"卡"）；
  曾实现"限流 + 缩放"两参数，按要求回档——真机相机流阶段再处理（notes §20.7）。
- 单目标、无数字识别、无整车位姿（同题2 局限）。

---

## 04_hik：海康 MVS SDK 学习 demo

独立工程（不参与主构建），三步递进，把 SDK 的使用拆开学会：

| 程序 | 内容 |
|---|---|
| `hik_probe` | 枚举设备，打印型号/序列号（第一个 MV_CC_* 调用） |
| `hik_open` | 按序列号打开相机，关闭自动曝光并设曝光/增益 |
| `hik_grab` | 取一帧，把 SDK 缓冲转成 OpenCV BGR（`cv::Mat`） |

```bash
cmake -S 04_hik -B build/04_hik && cmake --build build/04_hik -j
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH     # 运行期找 .so 必需
./build/04_hik/hik_probe
```

主节点用的是封装后的版本（`image_source.cpp` 的 `#ifdef RM_USE_HIK_SDK` 段 + `data/camera.yaml`）：
**现场展示只需改 `data/camera.yaml` 里的 `serial_number`，代码不用动**。踩坑记录见 notes §19。

---

## 证据与复现索引

| 想验证什么 | 看这里 | 怎么复现 |
|---|---|---|
| 题1 检出效果 | `docs/screenshots/detector_preview_*.png` | `./build/01_detector/armor_demo` |
| 检测器 A/B 全量指标 | `results/eval_*_summary.txt` | `eval_demo` + `compare_eval.py`（见题2） |
| 题2 平滑效果 | `results/tracker_demo.avi` | `./build/02_tracker/tracker_demo` |
| 题2 PnP 闭环正确性 | notes §11.6（数字可复算） | `./build/02_tracker/pnp_demo` |
| ② 负结果原始证据 | `results/corner_*`（需 `RM_CORNER_DEBUG=1`） | `eval_demo ... refine` |
| 题3 真实相机验证 | `results/real_camera_2026-09-09.mkv`、`docs/screenshots/phone_rqt.png` | `source:=ip` 或 `video_path:=http://...` |
| 原理与踩坑全过程 | `docs/notes.md`（21 章） | 按目录读；§20（四关键点与评测）、§21（负结果全录 + 考核对照）是本轮核心 |

## FAQ

| 现象 | 原因 / 解法 |
|---|---|
| rqt 无画面 / topic 大量 message lost | CycloneDDS 大图像不稳 → `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| rqt 打开后是渐变占位图 | 手动下拉常没真正订阅 → 直接带话题：`rqt_image_view rqt_image_view /armor/annotated` |
| 手机流 "Stream ends prematurely" | URL 少了 `/video` 后缀 |
| 检测框在但 z<0 / 距离乱跳 | 手机竖屏（旋转元数据）→ 横屏解决 |
| 距离绝对值和真实差一截 | 内参未标定（HFOV≈72° 近似）→ 只论量级；标定见 notes §8.11 |
| `detector:=pose` 时报 "需要 ONNX Runtime 后端" | 该构建未开 ORT → 用 `--cmake-args -DUSE_ONNXRUNTIME=ON -DONNXRUNTIME_DIR="$ORT_DIR"` 重新 colcon build |
| ORT 报 float16/float32 类型不匹配 | 用了 `convert_fp16_to_fp32.py` 的产物 → ORT 必须用**原始导出件** |
| `source:=hik` 报找不到相机 | `data/camera.yaml` 的 `serial_number` 未填 / 相机未上电；构建需 `-DUSE_HIK_SDK=ON` |

## 参考资料对照

考核题面给出的是完整的自瞄能力链（识别模型 → 部署框架 → 仿真 → 整车系统）。本工程学习时对照阅读了：

- **同济 SuperPower `sp_vision_25`**：灯条端点 3D 建模、板尺寸 135×125 / 230×127、灯条长度 56mm 的出处
  （本地 `reference/` 有源码）——本工程 `barEndObjectPoints` 的物体模型直接来自这里；
- **深大 RobotPilots 26 赛季视觉模型**：四关键点（Pose）思路与模型——本工程方案 B 的模型来源；
- **河科 Actor&Thinker 视觉仿真器**：内参精确已知的仿真思路（作为"仿真接入"的后续可选项）；
- 上科大十等星 26 赛季自瞄教程：概念与工程结构的对照。

踩坑、设计决策、两条 v2 负结果的完整记录都在 `docs/notes.md`。

## 许可说明

- **代码**：MIT（见 `LICENSE`，与 `rm_interfaces`、`03_visualization` 的 `package.xml` 声明一致）。
- **权重**：本仓库提交的自训 ONNX 模型由 Ultralytics YOLOv8 训练链产出，其 ONNX 元数据标注
  `AGPL-3.0`；代码许可与权重许可**不是一回事**，若需再分发权重请自行确认许可条件。
  深大 `Infantry-v8n` 权重**未入库**，需自行下载。

## 修订记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | 2026-09-08 | 三题初稿运行说明 + 证据截图 |
| v2.0 | 2026-09-09 | 复盘后重构：三题并列结构、状态总览表、补题2 运行/指标、FAQ、参考资料对照；修复录屏死链（.gitignore 白名单） |
| **v3.0** | 2026-09-11 | 按考核题面重写全文：补「考核要求对照」表；新增题2 评测基础设施（口径 + 687 帧 A/B 表）与两条 v2 负结果；题1 补四关键点检测器（输入几何/关键点语义/后端要求）；题3 补检测器选择、真实相机与可视化路线、海康相机；新增 04_hik 说明、证据与复现索引、许可说明 |
