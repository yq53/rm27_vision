# rm27_vision 学习笔记（视觉方向考核）

> 本文件记录开发过程中的讲解与问答，用于复习与面试准备。
> 仓库：`rm27_vision` · 考核截止：2026-09-16 22:00

---

## 0. 目录

1. 考核背景与总体路线
2. 环境备忘
3. 题1：装甲板识别器（现状）
4. 代码问答笔记（重点，面试速查）
5. 自测清单
6. 下一步

---

## 1. 考核背景与总体路线

考核 = 一个自瞄项目的拆解，视觉方向三小题：

| 题 | 内容 | 状态 |
|---|---|---|
| 题1 | 装甲板识别器 detector（传统视觉或神经网络均可） | ✅ 已跑通 |
| 题2 | 装甲板跟踪器 tracker（PnP+EKF 等，后端不限） | ⏳ 未开始 |
| 题3 | 接入仿真/真实相机 + 可视化界面 | ⏳ 未开始 |

**语言路线**：全程 C++（ROS2 + OpenCV C++），与战队 clang + VSCode 环境一致。

**仓库结构**：

```
rm27_vision/
├── models/            # ONNX 模型（题1 神经网络方案）
├── data/              # 测试素材 demo.avi
├── 01_detector/       # 题1：检测器（静态库 + 演示程序）
├── 02_tracker/        # 题2（占位）
├── 03_visualization/  # 题3（占位，ROS2）
├── docs/              # 笔记（本文件）
└── results/           # 运行输出（gitignore）
```

---

## 2. 环境备忘

- Ubuntu 22.04 · ROS2 Humble · OpenCV 4.13(dev) · clang 14 · CMake 3.22
- **clang 链接修复**：机器装了 gcc-12 但没有配套的 `libstdc++-12-dev`，clang 自动选择了 gcc-12 目录作为工具链，导致 `找不到 -lstdc++`。
  → 解决：`sudo apt install -y libstdc++-12-dev`（无需任何代码改动）。
- 构建命令（在**仓库根目录**运行，代码默认相对路径 `data/`、`models/`）：

```bash
cmake -S 01_detector -B build/01_detector -DCMAKE_BUILD_TYPE=Release
cmake --build build/01_detector -j
./build/01_detector/armor_demo data/demo.avi            # 全部帧
./build/01_detector/armor_demo data/demo.avi models/armor_yolov8n.onnx 240  # 只跑前 240 帧
```

---

## 3. 题1：装甲板识别器（现状）

**方案**：YOLOv8n 神经网络（自训模型 + OpenCV DNN 推理，无需 onnxruntime）。

- 模型：`models/armor_yolov8n.onnx`（来自工作区 `trainning/` 的 exp1-3，单类别 `armour`，mAP50≈0.97）
- 数据：自录视频抽帧（65 张）→ 手工标注 → 52 train / 13 val
- 结构：检测核心封装成**静态库 `armor_detector`**，对外接口只有 `detect(frame) → vector<Armor>`；`armor_demo` 负责视频循环 + 画框 + 存结果。

**实测（demo.avi，687 帧全量，CPU）**：

| 指标 | 值 |
|---|---|
| 检出装甲板帧 | 498/687 (72.5%)，共 737 次检出 |
| 平均耗时 | 51.2 ms/帧 ≈ 19.5 FPS |

**已知瑕疵**：暗光下模型会把一个装甲板的两个灯条误检成两个装甲。
→ 原因：两个灯条框几乎不重叠（IoU≈0），NMS 不会合并它们；问题在"模型把灯条当成了目标"这一层，与后处理无关。留待后续优化（调阈值 / 灯条配对后处理 / 换模型）。

---

## 4. 代码问答笔记

### 4.1 `#pragma once`（头文件保护）

- `#include` 本质 = 把整个文件复制粘贴到当前文件。
- 同一个头文件被间接包含两次 → 类被定义两遍 → **重复定义编译错误**。
- `#pragma once`：编译器记住该文件，第二次遇到直接跳过（等价传统宏保护 `#ifndef ... #endif`，clang/GCC/MSVC 都支持，RM 开源项目通用写法）。
- `pragma` = 给编译器的指令。

### 4.2 为什么构造函数前要加 `explicit`

- `explicit` 禁止**隐式转换**，只对有意义的单参数构造函数生效；`ArmorDetector(std::string)` 正是单参数。
- 不加时允许：`ArmorDetector d = some_string;` 或把 `std::string` 悄悄传给"期望 `ArmorDetector`"的函数——对一个加载十几 MB 模型的重型对象来说几乎一定是笔误。
- 加 `explicit` 后这类写法编译报错，逼你显式构造。
- **原则：单参数构造函数默认都加 `explicit`，除非真的想允许隐式转换。**

### 4.3 声明 vs 定义（头文件 vs .cpp）

- 头文件：**声明** —— 只告诉编译器"有这个函数、参数、返回类型"，不写函数体（对外暴露接口"能干什么"）。
- .cpp：**定义** —— `ArmorDetector::detect(...) { ... }` 写函数体（实现细节"怎么干"）。
- `类名::` 前缀 = "我在实现类里声明过的那个函数"。
- 好处：使用者看不到实现细节；改实现只重编译一个 .cpp；链接器负责把调用和实现对上。

### 4.4 为什么 letterbox 是 private + static 的成员函数

- **private**：它是"喂模型前怎么做预处理"的**内部实现细节**，外部只需要 `detect()`；藏起来减少接口面、防止误用。
- **static（静态成员函数）** = 没有 `this`、不依赖对象状态，纯"参数进→结果出"。
  - 判断标准：**用不用成员变量？** letterbox 只碰参数和 `kInputSize`（类级常量）→ 可以 static；`detect` 要用 `net_`、阈值 → 必须普通成员函数。
- **为什么不写成类外自由函数？**
  1. 它用到类的 **private** 常量 `kInputSize`——搬出去要么把 640 泄漏成全局，要么硬编码 640，未来换模型输入尺寸时两处不一致的 bug 源；
  2. 它是"这个类喂模型的方式"，属于类的私有实现，绑在一起语义最清楚。
- **补充**：如果某个纯工具函数不依赖任何类私有成员、且只在一个 .cpp 内用，工程上更常见的做法是放**匿名命名空间**（该文件私有自由函数）。"类内 static private" 和"匿名命名空间"都是规范写法，按耦合度选。

### 4.5 `namespace rm_vision { ... }`

- 命名空间 = 给符号**分组**，防止**名字冲突**（不同库都可能有 `Armor`/`letterbox`）。
- 命名空间内所有名字的真实全名带前缀：`rm_vision::Armor`、`rm_vision::ArmorDetector`。
- Armor 和 ArmorDetector 同属检测器模块（输入/输出类型）→ 一起放进 `rm_vision`；以后 tracker 等也进同一命名空间。
- 使用侧：`rm_vision::ArmorDetector d(...)` 完整限定 / `using rm_vision::ArmorDetector;` / `using namespace rm_vision;`。
- **头文件里禁止 `using namespace`**（`#include` 会把它扩散污染所有包含者）。

### 4.6 置信度阈值 & NMS（重点）

**置信度阈值 `conf_threshold_`（0.35）**
- 网络对每个候选框输出"是装甲板"的概率，低于阈值的候选框直接当噪声丢弃。
- 调高 → 误检↓（precision↑）但漏检↑；调低 → 相反。

**NMS = Non-Maximum Suppression（非极大值抑制）**
- 背景：YOLO 对同一个目标会输出**大量高度重叠的候选框**，需要只留最好的一个。
- 重叠判据 **IoU（交并比）**：

  ```
  IoU = 交集面积 ÷ 并集面积
  ```

  - 分母是**并集**（两框总覆盖），不是任一框面积——同一个框被大框包住，除以小框是 1.0、除以并集可能只有 0.3，并集分母才是公平的 0~1 度量。
- NMS 是**贪心算法**，不是"互超阈值的都删"：
  1. 按置信度从高到低排序；
  2. 取最高分框进结果（"王者"）；
  3. 删掉其余框中与它 **IoU > nms_threshold** 的（视为同一目标）；
  4. 剩余框中再取最高分当新"王者"，重复到没有框。
  - 名字含义：*抑制掉非局部最大值的框*。
- **调参方向**：`nms_threshold` 调大 → 删得少 → 同目标可能残留多框；调小 → 删得狠 → 前后贴近的不同目标可能被误删。注意它比较的是**框与框的重叠**，不是分数——别和置信度阈值搞混。
- 回到实测瑕疵：灯条两框 IoU≈0 < 0.45 → 被当两个目标。NMS 工作正常，误检根源在模型本身。

### 4.7 命名规范与 `kInputSize`

- Google C++ 风格：
  - **常量**：`k` 前缀 + 大驼峰 —— `kInputSize`（k = constant，InputSize = 输入尺寸）
  - **成员变量**：尾下划线 —— `conf_threshold_`（或 `m_` 前缀，二选一统一）
  - **普通变量/函数**：snake_case —— `model_path`、`letterbox`
- `static constexpr int kInputSize = 640;` 为什么是 static？
  - **类级常量**，不属于某个对象（10 个对象不需要 10 份 640）；
  - `constexpr` = 编译期常量，用到的地方直接被替换成 640，不占运行期存储；
  - 输入尺寸是**模型本身固定属性**（训练时定死），与实例无关 → 类级常量最合适，放 private 保护起来。

---

## 5. 自测清单（面试速查）

- [ ] 能解释 `#pragma once` 和宏保护的等价关系
- [ ] 能说出 `explicit` 解决什么问题、什么时候必须加
- [ ] 能区分声明和定义，解释为什么头文件不放实现
- [ ] 能判断一个成员函数该不该加 static（"用不用成员变量？"）
- [ ] 能说出 static 成员函数 vs 类外自由函数 vs 匿名命名空间的取舍
- [ ] 能画出 IoU 公式并解释为什么分母是并集
- [ ] 能手写/口述 NMS 贪心流程，说清两个阈值各自控制什么
- [ ] 能解释 `rm_vision::` 前缀的意义、头文件为什么禁止 using namespace
- [ ] 能说明 Google 风格下常量/成员/局部变量的命名差异

---

## 6. 下一步

- [ ] 通读 `01_detector/src/armor_detector.cpp`（letterbox 数学、blobFromImage、ONNX 输出的 5 个数、坐标还原）—— 题1 核心逻辑
- [ ] 题2 预告：tracker 需要**装甲板四角点**做 PnP，而 YOLO 只给轴对齐包围框 → 需要框内找灯条/角点或换方案
- [ ] 提交规范：代码每完成一个里程碑 commit 一次，README 记录运行方式
