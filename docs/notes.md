# rm27_vision 归纳笔记（notes）

> **这份文档是什么**：按**主题**归纳本工程的结论与约定——系统由什么组成、关键常数与接口、评测口径、
> 踩坑速查、负结果与局限。**每条结论后面都标了它在过程日志里的出处**（`log.md §X.Y`）。
>
> **三份文档的分工**
>
> | 文档 | 回答什么 | 什么时候看 |
> |---|---|---|
> | `README.md` | 怎么跑、该看到什么、指标是多少 | 想跑起来 / 想核对数字 |
> | **本文件 `notes.md`** | 系统由什么组成、关键约定是什么（**主题归纳**） | 想快速建立整体认识 |
> | [`log.md`](log.md) | 为什么这么做、当时怎么想、试错过程（**过程日志，23 章，章节号稳定**） | 想溯源推导与踩坑细节 |
>
> 三者的交叉验证结论见 `log.md §22`。

---

## 1. 系统架构与数据流

```
图像源(video / ip / hik) → 检测器(bbox | pose) → PnP → EKF ─┬─→ /armor/state        自定义 ArmorState
                                                             └─→ /armor/annotated   sensor_msgs/Image（SensorDataQoS）
```

- 目录职责与依赖方向：`03_visualization` → `02_tracker` → `01_detector`（通过 `add_subdirectory` 内部复用，不单独安装）
  → `log.md §17`
- **两种检测器被"抹平"的位置**：节点构造时二选一（`detector_` / `pose_detector_` 只有一个非空），
  每帧把结果统一塞进 `target_rect` / `target_corners` / `has_target` 三个变量，之后的 PnP → EKF → 发布**完全共用**
  → `log.md §20.6`
- **必需文件 vs 可删文件**：判据是"删掉它还能不能起主节点并在 rqt 看到结果"，清单见 `README.md`「文件清单」
- 启动入口只有 `ros2 launch`；`scripts/setup.sh` 只做"环境 + 构建" → `README.md`「快速开始」

## 2. 关键接口与常数

| 量 | 值 | 出处 |
|---|---|---|
| 装甲板尺寸（小 / 大） | 135×125 mm / 230×127 mm | `log.md §8` |
| 灯条长度（标称 / 本素材实测最优） | 56 mm / ≈52 mm | `log.md §20.2` |
| 四关键点的物体模型 | 135×56 mm（`barEndObjectPoints`） | `log.md §20.2` |
| 自训 bbox 模型输入 | 640×640（4:3 素材需上下各补 80 行灰边） | `log.md §5.2` |
| letterbox 填充灰值 | 114 | `log.md §5.2` |
| 四关键点模型输入 / 输出 | `480×640` / `[1, 21, 6300]` | `log.md §20.1`、`§20.2` |
| 输出布局 | `row4..12` = 9 个类别分数；`row13..20` = 4 个关键点 (x,y)；无逐点置信度 | `log.md §20.2` |
| 关键点索引映射 | `kp0/kp3/kp2/kp1 → TL/TR/BR/BL`（4 点是**灯条端点**，不是板四角） | `log.md §20.2` |
| 检测阈值 / NMS | bbox 0.35 / pose 0.5；NMS 均为 0.45 | `log.md §20.4` |
| PnP 拒绝条件 | 平均重投影 > 10 px；物理闸门 `z > 0.2 m 且 dist < 20 m` | `log.md §11` |
| 滤波状态 | 6D：位置 + 速度（常速模型） | `log.md §14` |
| 内参 | `fx = (w/2)/tan(36°)`，即 HFOV≈72° 近似（**未标定**） | `log.md §8.5`–`§8.11` |
| 评测素材 | `data/demo.avi`，687 帧 @30 fps | `log.md §20.5` |

## 3. 两种检测器：语义、后端、依赖

| | 方案 A：bbox（默认） | 方案 B：四关键点（推荐） |
|---|---|---|
| 模型来源 | **本仓库自训**（自录素材抽帧标注，量小） | **深圳大学 RobotPilots 26 赛季开源**（见 README 文末「参考仓库与教程」第 1 条） |
| 输出 | 轴对齐 bbox（角点 = 框四角近似） | 4 个灯条端点（可直接用于 PnP） |
| 推理后端 | OpenCV DNN（零额外依赖） | **必须 ONNX Runtime**（OpenCV 4.x 的 cv2.dnn 加载不了该图；python cv2 5.0 实测可跑） |
| 687 帧检出 | 72.49% | **90.25%** |
| 重投影 | 4.11 px | **1.19 px** |
| 结论 | 保留作基线；**想看最好效果请用 B** | 推荐 |
| 出处 | `log.md §3`、`§16` | `log.md §20.1`–`§20.5` |

**换模型必须重新验证两件事**（否则不报错、只静默出错）：
① 输出布局（`row13..20` 是否为关键点）；② 4 个点的语义与顺序（是否灯条端点、绕向）。
验证方法：用候选物体模型做 PnP，看重投影误差是否显著变小 → `log.md §20.2`。

## 4. 位姿解算与滤波

- **PnP**（`rm_tracker::solveArmorPose`）：`SOLVEPNP_IPPE` 出多解 → 优先保留 `r₃.z > 0`（破镜像）→
  取重投影最小者且 >10 px 拒绝 → 物理闸门。输出 `PoseResult{tvec, reproj_err_px, yaw_deg}`，
  后两项供评测量化角点质量 → `log.md §11`
- **EKF**（`rm_tracker::ArmorEKF`）：6D 常速；当前模型线性，故实际退化为 KF；漏检帧只 `predict` 不 `update`；
  骨架按 EKF 写，换测量模型只需把常量 F/H 换成雅可比 → `log.md §13`、`§14`
- **单目标**：每帧挑面积最大的板（≈最近），两块板交替最大时会跳目标；多目标需要数据关联，未做
  → `log.md §14.3`
- ⚠️ **已知维护风险**：解算逻辑存在**两份**实现——主节点内的 `solveArmorPosition` 与库 `armor_pnp.cpp`，
  二者当前逐行一致（评测数字因此能代表节点行为），但**改动时必须同步** → `log.md §20.6`

## 5. 评测口径与 687 帧结果

- 工具：`eval_demo <视频> <模型> <最大帧数> <tag> <corner_mode> <detector>` + `compare_eval.py`；
  **口径不变才可比** → `log.md §20.4`
- 关键定义：
  - `jitter` = **相邻且帧号连续**的有效帧之间的距离差分均值（跨漏检空档不计、首帧不计），样本数 `n` 一并输出；
  - `NaN` = "本帧无有效值"（与真实的 0 区分）；`numeric_limits::max()` 作比较哨兵、`quiet_NaN()` 作缺失标记，二者不混用；
  - `refined` 在 bbox 模式 = "角点精修成功"，在 pose 模式 = "使用了模型端点"（**语义不同，别混读**）
  → `log.md §20.4`
- 结果（687 帧，同一 EKF、同一口径；完整表见 README）：

  | 指标 | bbox | pose |
  |---|---|---|
  | 检出帧 | 498（72.49%） | 620（90.25%） |
  | PnP 通过 | 415（60.41%） | 604（87.92%） |
  | 重投影 mean / median | 4.11 / 3.15 px | 1.19 / 0.82 px |
  | 抖动 raw / ekf | 0.0853 / 0.0555 m | 0.0322 / 0.0219 m |
  | 偏航角 mean\|·\| | 18.28° | 13.27° |
  | 检测耗时 | 37.96 ms | 26.81 ms |

  → `log.md §20.5`
- 入库的原始汇总：`results/eval_baseline_summary.txt`、`results/eval_pose_summary.txt`（可自行重跑复现）

## 6. 踩坑速查

| 症状 | 原因 / 解法 | 出处 |
|---|---|---|
| cv2.dnn 加载 fp16 模型报 `unknown input 'graph_input_cast_0'` | 入口 Cast 节点未被 importer 建模；转 fp32 后又卡在 `NaryEltwise` | `log.md §20.3` |
| ORT 报 float16/float32 类型不匹配 | 误用了 `convert_fp16_to_fp32.py` 的产物；ORT 必须用**原始导出件** | `log.md §20.3` |
| `detector:=pose` 启动即抛"需要 ONNX Runtime 后端" | 该构建未开 ORT；用 `-DUSE_ONNXRUNTIME=ON -DONNXRUNTIME_DIR=…` 重建 | `log.md §20.3` |
| 改了 `03_visualization` 源码，`ros2 run` 行为却没变 | `cmake --build` **不更新 `install/`**；必须 `colcon build` | 本轮实测（README FAQ） |
| 自写订阅者收不到 `/armor/annotated` | 该话题是 **SensorDataQoS（BEST_EFFORT）**，订阅端要匹配 | 本轮实测（README FAQ） |
| `source:=hik` 节点退出 255，但 `ros2 launch` 返回 0 | launch 不透传节点退出码；**以节点日志为准** | `log.md §19.3` |
| 某命令像卡住不结束 | 帧数参数传了非数字：只有 `armor_demo` 会报错，其余 `atoi` 静默变 0 = 跑全片 | 本轮实测（README FAQ） |
| `eval_demo` 第 5/6 参数写错却不报错 | 该程序不校验取值，会静默按 bbox 跑 | 本轮实测 |
| rqt 无画面 / 大量 message lost | 跨 RMW 传大图像掉帧（≈26 Hz → ≈1.9 Hz）；统一 FastDDS | `log.md §17.4` |
| `[ros2run]: Aborted` / spdlog `Read-only file system` | `~/.ros` 不可写 → `export ROS_LOG_DIR=$PWD/.roslog` | `log.md §17.4` |
| 手机流 "Stream ends prematurely" | URL 少了 `/video` 后缀 | `log.md §17.4` |
| 检测框在但 z<0 / 距离乱跳 | 手机竖屏产生 90° 旋转元数据 → 镜像假解，务必横屏 | `log.md §17.4` |
| 海康程序报找不到 `libMvCameraControl.so` | 一般**不需要**设 `LD_LIBRARY_PATH`（CMake 已写入 RUNPATH）；只有换环境才要 | `log.md §19.3` |
| 设了 `RM_CORNER_DEBUG=0` 也落盘调试图 | 源码判断是 `getenv() != nullptr`，**传任意值都触发** | 本轮实测（README） |
| `scripts/setup.sh` 探不到 ONNX Runtime | 用 `ORT_DIR=/你的/onnxruntime bash scripts/setup.sh` 指定 | `README.md`「配置环境与构建」 |

## 7. 负结果与已知局限

**三条负结果**（都做了、测了、主动放弃；代码与证据留在仓库里）：

| 路线 | 结果 | 处置 | 出处 |
|---|---|---|---|
| `light_bar_detector`：框内阈值 + 连通块 + 灯条配对 | demo.avi **配对率仅 7%**（大角度侧转时两灯条在画面里几乎重叠） | 冻结 | `log.md §15` |
| `armor_corner`：ROI + PCA 主轴端点 + 亮度梯度修正 | 中间档闸门下仅 **0.4%** 检出帧精修成功（`bars_mean≈0.92`，常只见单根灯条）；严格档 0% | 默认关闭，不接入主节点 | `log.md §21.1` |
| `armor_yolov8n_dark`：暗化增强微调 | mAP50 保持 0.974 但跨域无提升 | 负结果存档，勿用 | `log.md §16` |

**已知局限**：单目标（无数据关联）；内参未标定（HFOV≈72° 近似，距离只保证量级）；
可视化带宽（1440×1080×30 Hz ≈ 140 MB/s，限流方案已按要求回档）；无数字识别、无整车位姿
→ `log.md §20.7`、`§21.2`

**未做清单**（按价值排序，其中前两项**已按要求暂缓**）：④ PnP 降自由度（固定 pitch/roll 只解 yaw）、
棋盘格内参标定、R/Q 噪声建模升级、多目标/整车 EKF、装甲数字识别、仿真接入 → `log.md §21.2`

## 8. 术语表

| 术语 | 含义 |
|---|---|
| **灯条端点** | 装甲板两根发光灯条的上下端点；四关键点模型输出的 4 个点就是它（**不是板四角**） |
| **板四角** | 装甲板物理外框的四角（135×125 mm 矩形的角）；bbox 方案用它，靠检测框近似 |
| **bbox 方案 / pose 方案** | 2D 点的两种来源：检测框四角 / 模型直出的灯条端点 |
| **重投影误差** | 用解出的位姿把 3D 模型点投回图像，与观测到的 2D 点的平均像素距离——**角点质量的直接度量** |
| **yaw（本工程口径）** | 板法线相对"正对相机"的水平偏角 `atan2(n.x, n.z)`，正对≈0 |
| **jitter（抖动）** | 相邻有效帧之间距离测量值的差分均值（米），衡量稳定性 |
| **有效帧** | 该帧 PnP 成功解出位姿（计入 `pnp_ok`），否则记 NaN |
| **IPPE** | 一种面向平面目标的 PnP 解法，会给出多个候选解，需要额外规则挑解 |
| **letterbox** | 等比缩放 + 填灰边，使图像适配网络输入尺寸（本项目用 114 灰） |
| **refined** | eval 的一列：bbox 模式=角点精修成功；pose 模式=使用了模型端点 |
| **单板跟踪** | 只跟踪一块板（本工程：每帧面积最大者），不做多目标数据关联 |

更细的术语解释（含当时的学习顺序）见 `log.md §7`。

## 9. 文档地图与交叉验证

- 想**跑起来看效果** → `README.md`（顶部「快速开始」与「快速验证（题3）」）
- 想**核对数字** → `README.md`「687 帧实测」+ `results/eval_*_summary.txt`（已入库）
- 想**快速建立整体认识** → 本文件
- 想**溯源推导 / 看踩坑过程 / 看负结果原始记录** → [`log.md`](log.md)（23 章，章节号稳定）

**三重交叉验证**（README ↔ notes ↔ log）：

- README 里每个论断都能在 `log.md` 或本文件找到依据（逐条对照表见 `log.md §22.1`）；
- 本文件每条结论都标了 `log.md §X.Y` 出处；
- `log.md` 里影响交付的结论都已在 README 反映；未反映的部分**只有学习过程类内容**（刻意分工，不是缺漏）
  → `log.md §22.2`、`§22.5`
