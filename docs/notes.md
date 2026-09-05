# rm27_vision 学习笔记（视觉方向考核）

> 本文件记录开发过程中的讲解与问答，用于复习与面试准备。
> 仓库：`rm27_vision` · 考核截止：2026-09-16 22:00

---

## 0. 目录

1. 考核背景与总体路线
2. 环境备忘
3. 题1：装甲板识别器（现状）
4. 代码问答笔记（重点，面试速查）
5. armor_detector.cpp 实现问答（题1 核心）
6. demo_main.cpp 问答与异常处理（题1 入口）
7. 自测清单
8. 下一步
9. 题2 tracker：术语表与学习路线（进行中）
10. L1 坐标系与针孔相机模型（题2 数学地基）
11. L2 正投影演示（题2 数学地基·代码版）
12. L3 四角点 v1：corner_demo（题2 进行中）

---

## 1. 考核背景与总体路线

考核 = 一个自瞄项目的拆解，视觉方向三小题：

| 题 | 内容 | 状态 |
|---|---|---|
| 题1 | 装甲板识别器 detector（传统视觉或神经网络均可） | ✅ 已跑通 |
| 题2 | 装甲板跟踪器 tracker（PnP+EKF 等，后端不限） | 🔄 进行中（v1→v2 分步） |
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
| 平均耗时 | 54.5 ms/帧 ≈ 18 FPS（最终复跑） |

**已知瑕疵**：暗光下模型会把一个装甲板的两个灯条误检成两个装甲。
→ 原因：两个灯条框几乎不重叠（IoU≈0），NMS 不会合并它们；问题在"模型把灯条当成了目标"这一层，与后处理无关。留待后续优化（调阈值 / 灯条配对后处理 / 换模型）。

**题1 收尾（已完成）**：
- README 增加效果截图（`docs/screenshots/`）与完整运行说明
- 全量复跑确认最终数据（见上表）。调参规律验证：置信度阈值若升到 0.6，帧命中率从 73.3% 降到约 22%——阈值↑→漏检↑，默认 0.35 是权衡结果（改阈值需在代码里调，未做命令行参数）

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
- 重叠判据 **IoU（交并比）**：设两框区域为 $A$、$B$，则

  $$ IoU = \frac{|A \cap B|}{|A \cup B|} $$

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

## 5. armor_detector.cpp 实现问答（题1 核心）

### 5.1 backend 与 target（推理后端与目标设备）

- 模型（ONNX）= 图纸：描述有哪些算子、怎么连。
- **Backend** = 由谁执行这些算子（执行引擎）：`DNN_BACKEND_OPENCV`（OpenCV 内置实现）、`DNN_BACKEND_INFERENCE_ENGINE`（OpenVINO）、`DNN_BACKEND_CUDA`（cuDNN）、`DNN_BACKEND_OPENCL` 等。
- **Target** = 在哪块硬件上算：`DNN_TARGET_CPU` / `DNN_TARGET_CUDA` / `DNN_TARGET_OPENCL` 等。
- 一句话：**backend 决定"用什么软件去算"，target 决定"在哪块硬件上算"**；backend 必须支持所选的 target。
- 比喻：模型=施工图纸，backend=请哪家施工队，target=工地（CPU 地块 / GPU 地块）。
- `readNetFromONNX` 只解析；真正执行在 `net_.forward()` 时按 backend/target 进行。`setPreferable*` 是"偏好"，OpenCV 没编译对应引擎时会回退/报错。

### 5.2 letterbox 的 pad / pad_w / pad_h

- 流程：原图按长边等比缩到 640（1440×1080 → 640×480）→ 宽/高不够 640 的方向补**灰色边框**让画面居中。
- `pad_w` = 左右每侧补的灰边像素（本例=0）；`pad_h` = 上下每侧补的灰边像素（本例=(640−480)/2=80）。
- 它们是 letterbox 的**输出参数**（引用带出），detect 坐标还原时要"减 pad 再除 ratio"。
- 灰边值 114 与训练时 Ultralytics 的 letterbox 一致——推理必须和训练同分布。

### 5.3 `static_cast<float>` 与整数除法陷阱

- `int / int` = 整数除法，结果截断：`640 / 1440 == 0`（不是 0.44）→ ratio 变 0 检测全废。
- `static_cast<float>(kInputSize)` 把一个 int 显式转成 float，使除法变成浮点除法。
- 用 static_cast 而非 C 风格 `(float)x`：编译期类型检查 + 代码里显眼可搜索。
- 口诀：**想要小数结果，先让至少一个操作数变成浮点**。

### 5.4 blob 是什么

- blob（借用 Caffe 术语）= 喂给网络的输入张量。
- `blobFromImage` 把图像重排成 **NCHW 四维连续 float 数组**：N=1（batch）、C=3（通道分离：先整排 R 再整排 G 再整排 B）、H=W=640；数值 0~255 → 0~1；BGR→RGB（swapRB=true）。
- 一句话：把人能看的图，变成"网络指定的内存排布 + 类型 + 数值范围"的数字块。

### 5.5 为什么 `ratio` 不能加 constexpr

- `constexpr` = **编译期**常量；`ratio` 的值取决于**运行期**才读到的图片尺寸，天生不可能是 constexpr。
- 且该行后面要被 letterbox 通过**引用改写**（输出参数），也不是 const。
- constexpr 不是性能优化工具：这种一帧一次的初始化开销可忽略，编译器自己会优化。**先跑通、用 profiler 找热点再优化**；51ms/帧里 99% 是 forward。

### 5.6 const 加在哪（规则 + 实例）

- 规则：**初始化之后不再改变的量就加 const**（防误改、表达意图、助优化）。
- 例子：`input/blob/output`（只读传给下一级）、`dim1/num_boxes`（循环上界）、每轮循环里的 `cx/cy/w/h`。
- 不能加 const：被引用改写的输出参数（`ratio/pad_w/pad_h`）、循环内被更新的量（`max_score`）。
- 习惯：先默认 const，需要改时再摘——摘 const 是状态在变化的信号。

### 5.7 `output.ptr<float>()`

- `cv::Mat::ptr<T>()` 是**成员函数模板**：返回指向矩阵数据首元素的 `T*`，用于把 Mat 当 C 数组直接快速遍历（避开 `.at()` 的检查开销）。
- `output` 是 const 时返回 `const float*`（const 重载）。
- 推理输出内部就是连续一维 float 数组 + 维度元信息 `output.size[]`，ptr 只是把这个事实暴露出来。

### 5.8 `vector::reserve`

- `size()` = 当前元素数；`capacity()` = 已分配容量。
- `push_back` 在 `size==capacity` 时要**扩容搬家**（重新分配+搬元素），vector 默认翻倍扩容。
- `reserve(n)` = 提前把容量扩到 n，后续 push_back 不触发扩容——**明确知道数量级时的标准优化**（如 `boxes.reserve(num_boxes)`、`armors.reserve(indices.size())`）。不改 size。

### 5.9 YOLO 输出里的 C 与 N

- 输出 = 二维矩阵，**C（特征维）= 4 + 类别数**：每框前 4 个数 `cx,cy,w,h`（640 输入图像素单位），第 4 个起是各类别得分（单类时 C=5，第 4 个数即置信度）。
- **N（候选框数）**：YOLO 在多尺度网格每个位置都预测框，640 输入 = 80²+40²+20² = **8400** 个候选框，绝大多数是背景垃圾框 → 靠 conf 阈值 + NMS 过滤。
- 实际行列数受输入尺寸影响，但"每候选框 C 个数"结构不变。

### 5.10 data 内存结构 + detect 全流程

**channel-major 排布 `(1, C=5, N=8400)`**（本模型导出形式），把它想成 **5 行 × 8400 列**的表格，每列是一个候选框：

```
行0 cx / 行1 cy / 行2 w / 行3 h / 行4 conf
```

内存按行连续（row-major）→ 第 f 行第 i 列偏移 = `f * num_boxes + i`：
```cpp
cx = data[0 * num_boxes + i];  cy = data[1 * num_boxes + i];
w  = data[2 * num_boxes + i];  h  = data[3 * num_boxes + i];
```
"前 num_boxes 个数是同一类数据" —— 正确：前 N 个全是 cx，接着 N 个全是 cy……

**转置排布兼容**：部分导出工具给 `(1, N, C)`（每行一个框的 5 个数）。代码比较 `dim1/dim2` 大小自动判断（5<8400 → channel-major），这就是第 78~82 行的作用——比 trainning/main.cpp 写死一种更健壮。

**detect 全流程（对照行号）**：
1. 预处理（54~67）：letterbox 缩放+灰边（拿到 ratio/pad 逆变换参数）→ blobFromImage 转 NCHW。
2. 前向（70~72）：`setInput` → `forward()` → `ptr<float>()` 拿裸数据。
3. 解析循环（91~122）：逐候选框取 cx,cy,w,h + 各类别分数取最大；低于 conf 阈值 continue。
4. 坐标还原（109~117）：**与 letterbox 互逆**——`原图 = (画布坐标 − pad) ÷ ratio`：
   $$ x = \frac{c_x - w/2 - pad_w}{ratio}, \qquad y = \frac{c_y - h/2 - pad_h}{ratio}, \qquad bw = \frac{w}{ratio}, \quad bh = \frac{h}{ratio} $$
   再 `rect &= Rect(0,0,cols,rows)` 裁越界（`&` 被重载为矩形求交）。
5. 组装 + NMS（119~131）：阈值过滤后的框 push 进三个平行 vector（reserve 预留）→ `NMSBoxes` 收敛重叠框（indices 是保留框的下标）→ 组装成 `Armor` 返回。

**数值例子**：原图 1440×1080 → ratio≈0.444，缩到 640×480，pad_h=80。画布检出中心(400,300)宽高(200,100) → 原图 x=(400−100−0)/0.444≈675、y=(300−50−80)/0.444≈383。

### 5.11 `class_id` 的作用（单类时的"有意义死代码"）

- 语义上，检测结果该回答三件事：**在哪（rect）+ 是什么（class_id）+ 多确定（confidence）**，三者构成完整结果。
- 目前单类永远为 0，demo 也不读它——但**题2 会真正用到**：PnP 解算时不同装甲板（1 号小装甲 / 大装甲 / 前哨站）物理尺寸不同，3D 模型要按类别选，届时靠 `class_id` 决定用哪套尺寸。
- **已改进**：解析循环现在记录 argmax（`best_class = f - 4`），不再是硬编码 0——单类行为不变，语义从"写死"变成"真的算出了得分最高的类"。
- 反方观点（可聊）：若确定永远单类，按 YAGNI 原则可删。"为未来预留 vs 不过度设计"是工程平衡题。

### 5.12 `cx / cy` 的 `c` = center（中心）

- YOLO 用**中心点 + 宽高**表示框（训练/损失/解码围绕中心编码），不是"左上角 + 宽高"。
- 转成 OpenCV 画框坐标：左上角 = `(cx − w/2, cy − h/2)`（还原到原图时再叠加减 pad、除 ratio）。

### 5.13 两种内存排布（下标公式的来由）

两种导出的内存结构**确实不同**，不是同一种排布换写法：

| 排布 | 维度 | 含义 | 取"第 i 框第 f 特征" |
|---|---|---|---|
| channel-major | `(1, C, N)` | 行 = 特征：前 N 个全是所有框的 cx，再 N 个 cy…… | `data[f * num_boxes + i]` |
| (1, N, C) | `(1, N, C)` | 行 = 一个框（每 C 个一组） | `data[i * num_features + f]` |

- 本项目模型导出是 channel-major；部分导出器/版本会输出转置版。
- 代码先比较 `size[1]/size[2]` 判断排布再选对应公式——**同一份逻辑兼容两种**；写死一种的话换模型就出乱码框。
- `data[i * num_features + 0]` 里的 `+0`：与 `+1/+2/+3` 对称可读，纯可读性取舍，不是 bug。

### 5.14 单类时 max_score 循环是否多余

- **功能上**：单类（C=5）循环只跑一次 f=4，等价直接取 `data[4 * num_boxes + i]`，确实可以省。
- **设计上**：它是多类解析的通用骨架——K 类时第 4~4+K−1 行是各类别分数，必须遍历取 max 并记录 argmax。
- 权衡：保留通用循环（换多类模型不用改）vs 最简写法（当下最清晰）。
- 本项目保留并顺手升级为 argmax 版本（见 5.11），单类行为不变。

---

## 6. demo_main.cpp 问答与异常处理（题1 入口）

### 6.1 demo_main 的职责与整体串联

- demo_main 是**胶水层**：不含检测算法（在库 armor_detector 里），只组织"一场检测演示"。
- 流程六步：① 解析命令行参数（默认值 + argc 防越界）→ ② 打开视频（失败 cerr + return -1）→ ③ 加载检测器 + 调阈值（失败抛异常，try-catch 接住）→ ④ 准备输出（建 results/、读视频属性、开 VideoWriter）→ ⑤ 主循环（read → detect → 画框 → 存视频/预览/进度）→ ⑥ 汇总（chrono 计时 + 打印命中率等）。
- 设计思想：**算法归算法（库）、流程归流程（main）**。题3 接 ROS2 时只写新节点调用库，库一行不改。

### 6.2 命令行参数：`argv/argc` 与 `atoi → stoi`

- `argv[0]` 是程序名，`argv[1]` 起才是参数；`argc` = 参数个数。`(argc > N) ? argv[N] : 默认值` 防止访问不存在的 argv（越界）。
- `atoi`（ASCII to Integer）：C 风格字符串→int，**遇非数字静默返回 0、不报错**。
- `stoi`：更现代，**遇非数字抛 `std::invalid_argument`**（比静默更安全，但异常必须被接住，否则照样崩）。配套写法：
  ```cpp
  int max_frames = 0;
  if (argc > 3) {
      try {
          max_frames = std::stoi(argv[3]);
      } catch (const std::invalid_argument&) {
          std::cerr << "[ERROR] Invalid max_frames argument: " << argv[3] << std::endl;
          return -1;
      }
  }
  ```

### 6.3 `std::cerr` ≠ 抛异常

| | `std::cerr` | 异常 throw |
|---|---|---|
| 本质 | 标准错误输出流（写 stderr） | 错误处理机制（抛对象给上层） |
| 作用 | 打印文字 | 传递"错误对象" |
| 后续 | 程序继续 | 没人 catch 就终止 |

- 惯例：正常日志用 `std::cout`（stdout），错误用 `std::cerr`（stderr，重定向时不丢失）。
- 本代码用"cerr 打印 + return 非0"处理可恢复错误（打不开视频）；"throw"留给构造函数这类"对象无法构造"的严重情况。

### 6.4 `namespace fs = std::filesystem;`（命名空间别名）

- 给已存在的命名空间起短名，`fs::path` ≡ `std::filesystem::path`。
- `fs::path` 把路径当类型：跨平台分隔符自动处理、支持 `result_dir / "xxx.avi"` 运算符拼接（比字符串拼安全）。
- 与第 4.5 节区分：那是在"定义"命名空间，这里是"引用"（起别名）。

### 6.5 准备输出块（VideoWriter / fourcc）

- 输出参数全部从**视频文件属性**读（不写死）：`capture.get(CAP_PROP_FRAME_WIDTH/HEIGHT/FPS)`；fps 读到 0（无字段）时退化为 30（防御性设计）。
- 为什么读：输出视频尺寸/帧率必须和输入一致，否则播放速度/比例错误。
- `VideoWriter` = VideoCapture 的反向类：把一帧帧 Mat 编码成视频文件。构造函数 4 参数：文件名 / fourcc / fps / Size。
- **fourcc** = Four Character Code：4 个字符标识一种编码器，`fourcc('M','J','P','G')` = Motion JPEG。`.avi` 配 MJPG / `.mp4` 配 mp4v，后缀要和编码器匹配。MJPG 兼容性好但文件大（帧内压缩）。
- `isOpened()` 检查：系统不支持编码器时 writer 为空，直接 write 会崩 → 先查，失败就警告跳过存视频。

### 6.6 主循环（统计变量与关键写法）

- `capture.read(frame)`：读一帧 + 返回 bool（读到 true / 结尾 false），比 `>>` 显式。
- `frames_with_armor` = 有检出的**帧数**（帧级命中率，一帧多个也+1）；`total_detections` = 检出**总个数**。两者语义不同。
- 先判 `!armors.empty()` 再计数：否则会把"没检出的帧"也计进命中帧。空检测结果是正常情况不是错误。
- `frame_id % 120 == 0`：每 120 帧存预览 PNG。
- `cv::format("preview_%04d.png", frame_id)`：OpenCV 版 sprintf；`%04d` = 至少 4 位补零 → 文件名字典序=时间序。
- `std::max(frame_id, 1)` 防除零（视频一帧都没读到也能安全打印）。

### 6.7 chrono 计时

- `steady_clock`：单调时钟，只增不减、不受系统时间修改影响，专用于测间隔。
- `end - start` = duration（默认纳秒，单位是编译期属性）。
- `std::chrono::duration<double, std::milli>(...)`：模板转换——"以 double 表示的毫秒 duration"，单位换算编译期完成，不会写错比例。
- `.count()`：取出裸数值 → `double elapsed_ms`。
- 设计动机：duration 类型自带单位，加减换算由编译器检查，杜绝单位写错。

### 6.8 异常接收端语法（catch 形参）

- **throw 抛的是对象**：`throw std::runtime_error("...")` 会构造一个临时对象抛出去；catch 像函数收参一样接它。
- `catch (const std::invalid_argument&)`：只接 invalid_argument 类型；`const &` = 只读引用（不复制、禁止改）；**没写参数名** = 不需要访问该对象（错误信息自己打印）。
- `catch (const std::exception& e)`：接**所有标准异常**（exception 是所有标准异常的基类，`runtime_error`/`invalid_argument` 都派生自它）；`e.what()` 取出异常自带的描述文字。
- **为什么 catch 用引用不用值**：按值接收会**对象切片**（派生类被砍成基类再复制），多态丢失、what() 拿到不完整信息。规范写法永远是 `catch (const T&)`。
- **catch 顺序**：同一 try 下，派生类（具体）写在前面、基类（兜底）写在最后，否则基类会截胡。

### 6.9 已应用的修复（异常处理落地）

1. **模型加载 try-catch + unique_ptr**：构造函数可能 throw，原来没人接。修复：
   ```cpp
   std::unique_ptr<ArmorDetector> detector;   // 指针在 try 外声明
   try {
       detector = std::make_unique<ArmorDetector>(model_path);
   } catch (const std::exception& e) {
       std::cerr << "[ERROR] Failed to load model: " << model_path << std::endl;
       std::cerr << "        reason: " << e.what() << std::endl;
       return -1;
   }
   ```
   - 为什么 unique_ptr：对象要活过整个 main（主循环要用），而 try 块内局部对象块结束即销毁 → 堆上构造 + 智能指针管理生命周期，无需手写 delete。
   - 调用从 `detector.detect()` 变 `detector->detect()`。
2. **stoi 防崩**（见 6.2）：非法帧数参数得到友好报错而非崩溃。
3. 验证：错误模型路径 → 打印 `reason: Can't read ONNX file...` 后退出；`abc` 参数 → 打印 `Invalid max_frames argument` 后退出。
4. 小知识：`return -1` 在 shell 显示为 255（-1 按无符号取模）；"非 0 即失败"语义不变，想直观可 `return EXIT_FAILURE`。

---

## 7. 自测清单（面试速查）

- [ ] 能解释 `#pragma once` 和宏保护的等价关系
- [ ] 能说出 `explicit` 解决什么问题、什么时候必须加
- [ ] 能区分声明和定义，解释为什么头文件不放实现
- [ ] 能判断一个成员函数该不该加 static（"用不用成员变量？"）
- [ ] 能说出 static 成员函数 vs 类外自由函数 vs 匿名命名空间的取舍
- [ ] 能画出 IoU 公式并解释为什么分母是并集
- [ ] 能手写/口述 NMS 贪心流程，说清两个阈值各自控制什么
- [ ] 能解释 `rm_vision::` 前缀的意义、头文件为什么禁止 using namespace
- [ ] 能说明 Google 风格下常量/成员/局部变量的命名差异
- [ ] 能区分 backend 和 target，并说出本项目的设置（OPENCV + CPU）
- [ ] 能解释整数除法陷阱和 static_cast 的作用
- [ ] 能画出 YOLO 输出表格（5 行 × N 列）并解释 C/N 含义
- [ ] 能口述 letterbox（缩放+pad）与坐标还原（减 pad 除 ratio）的互逆关系
- [ ] 能解释 blob 的 NCHW 排布、swapRB、1/255 各做什么
- [ ] 能说出 `Mat::ptr<T>()`、`vector::reserve`、`rect &= ...` 各自干什么
- [ ] 能区分 `atoi` 与 `stoi`（静默返回 vs 抛异常），并写出配套 try-catch
- [ ] 能说清 `std::cout` / `std::cerr` / 抛异常三者区别
- [ ] 能解释 VideoWriter 4 参数与 fourcc 含义、为什么后缀要匹配编码器
- [ ] 能说清 `frames_with_armor` 与 `total_detections` 的语义差异
- [ ] 能解释 `steady_clock` + `duration<double,milli>` + `.count()` 的计时原理
- [ ] 能解释异常捕获为何用 `const T&`（切片/多态）、catch 顺序规则
- [ ] 能说明 unique_ptr 解决"对象要活过 try 块"的原理
- [ ] 能解释 `class_id` 的用途、为什么解析要算 argmax（而非硬编码 0）
- [ ] 能说明 YOLO 用中心点表示框（cx/cy），以及如何换算成左上角
- [ ] 能对照两种输出排布讲清两个下标公式（`f*N+i` vs `i*C+f`）

---

## 8. 下一步

- [x] 通读 `01_detector/src/armor_detector.cpp`（letterbox 数学、blobFromImage、ONNX 输出的 5 个数、坐标还原）—— 题1 核心逻辑（问答见第 5 节）
- [x] 通读 `01_detector/src/demo_main.cpp`（视频循环、VideoWriter、drawArmors、FPS 统计）—— 问答见第 6 节
- [x] 题1 收尾：README 附图（docs/screenshots/）+ 全量复跑（代码保持收尾前状态，未加阈值参数）
- [ ] 题2 预告：tracker 需要**装甲板四角点**做 PnP，而 YOLO 只给轴对齐包围框 → 需要框内找灯条/角点或换方案
- [ ] 提交规范：代码每完成一个里程碑 commit 一次，README 记录运行方式

---

## 9. 题2 tracker：术语表与学习路线（进行中）

### 9.1 自瞄链路四问（tracker 的位置）

| 问题 | 谁回答 | 输出 |
|---|---|---|
| ① 画面哪里有装甲板？ | 题1 detector（✅） | 2D 包围框 |
| ② 它在三维空间哪里、角度多少？ | PnP | 3D 位置 + 姿态 |
| ③ 往哪跑？噪声/丢帧怎么办？ | EKF | 平滑状态 + 预测 |
| ④ 瞄准哪、何时开火？ | 云台控制/弹道解算 | （范围外） |

题2 = 让系统从"看得见"(2D) 升级到"知道在哪里、怎么动、接下来在哪"(3D 状态估计)。

### 9.2 术语（大白话版）

- **PnP**：已知 n 个 3D 点(装甲板四角，尺寸已知) + 对应 2D 像素 + 相机内参 → 反推相机看物体的位置/角度。直觉：看一扇尺寸已知的门，能估出它多远、自己正对它多少度。
- **为什么光有 PnP 不够**：逐帧独立 → ①抖(检测噪声放大) ②瞎(没有速度,无法提前量) ③断(漏帧即无输出)。
- **EKF**：预测(运动模型 $F$) + 更新(测量 $z$)，按卡尔曼增益 $K$ 融合，输出比任一单独估计都稳。五步循环（记熟）：
  $$
  \begin{aligned}
  \text{预测:} \quad \bar{x} &= F x, & \bar{P} &= F P F^{\mathsf T} + Q \\
  \text{更新:} \quad K &= \bar{P} H^{\mathsf T} \left( H \bar{P} H^{\mathsf T} + R \right)^{-1}, &
  x &= \bar{x} + K (z - H \bar{x}), & P &= (I - K H)\, \bar{P}
  \end{aligned}
  $$
- **单板 vs 整车**：单板=每块板独立跟踪；整车=4 块板建模为同一刚体(车)，切板状态不跳变（进阶）。
- **后端不限**：ESEKF/MCSKF/因子图是更高级方案；EKF 教科书级、能讲透即可交卷。

### 9.3 三个前置输入问题（决定代码结构）

1. 四角点从哪来？YOLO 只给轴对齐框 → v1 用框角近似，v2 升级框内灯条精定位
2. 相机内参？demo.avi 无内参 → v1 假设值，题3 真实相机/标定板替换
3. 装甲板 3D 尺寸？已确认（见下），来源与待核事项见笔记

**装甲板官方尺寸（v2 3D 建模用，来源：官方规范手册图 3-16/3-17 人工读数，2026-09-03）**

| 常量 | 小装甲板 | 大装甲板 | 来源 |
|---|---|---|---|
| 板宽（受攻击面） | 135 mm | 230 mm | 官方图人工读数（同济常量 135/230 一致 ✅） |
| 板高 | 125 mm | 127 mm | 官方图人工读数（同济注释 126 为两者近似） |
| 灯条长 | 56 mm（参考） | 56 mm（参考） | 同济代码常量；官方图未标注灯条尺寸，待实测复核 |

- 官方"示意图"只标板宽板高，无灯条尺寸；同济 56mm 为实战测量/旧版图纸值，v2 实现时若需延长灯条还原板角，板高应按类型取 125/127，延长系数逐类计算
- 参考实现：同济 `solver.cpp` 用"两灯条端点"(宽=板宽、竖=灯条长)做 3D 点建模；`detector.cpp` 检测到灯条后按系数延长成板角像素点

### 9.4 学习路线（v1→v2，单板为主）

- L1 坐标系与针孔相机模型、内参 K（含线代/概率/微积分速补）← 当前
- L2 装甲板 3D 建模 + 正投影（3D→2D）小工具，把相机模型"看"见
- L3 四角点 v1：bbox 角点近似，改数据结构
- L4 PnP：cv::solvePnP + 距离合理性验证
- L5 概率统计速补：高斯/方差/协方差
- L6 EKF 设计 + 实现（常速模型、位置测量）
- L7 tracker 工程化 + 演示（丢帧模拟、平滑对比、轨迹绘制）
- L8（v2）框内灯条精定位四角点；可选整车味多板关联

### 9.5 数学补课地图

- 线代：向量/矩阵乘法/线性变换/旋转矩阵/齐次坐标（L1 起步）
- 概率：高斯分布、方差/协方差、独立性（L5 用到）
- 微积分：导数 → 雅可比矩阵（EKF 线性化用到，届时补）

---

## 10. L1 坐标系与针孔相机模型（题2 数学地基）

### 10.1 本课核心：一条翻译链

$$ P_{板} \xrightarrow{[R \mid t]\,(外参,\,未知)} P_{相机}=(X,Y,Z) \xrightarrow{\div Z\,(透视)} (x_{norm},y_{norm}) \xrightarrow{K\,(内参,\,已知)} (u,v) $$

题1 得到的是最右端 $(u,v)$（检测框）；题2 的 PnP 要反解出 $[R \mid t]$（相机怎么看这块板）。

### 10.2 坐标系

| 坐标系 | 原点 | 说明 |
|---|---|---|
| 板坐标系 | 装甲板中心 | 四角 3D 坐标是常量：$(\pm w/2,\ \pm h/2,\ 0)$ |
| 相机坐标系 | 相机光心 | 算距离/角度用 |
| 像素坐标系 | 图像左上角 | 题1 输出 $(u,v)$ |

### 10.3 线代速补（够用即可）

- 点 = 列向量；矩阵×向量 = 线性变换（拉伸/旋转/切变，不能平移）。
- 旋转矩阵性质：$R^{-1} = R^{\mathsf T}$（转回去 = 反着转），后面推导常用。

### 10.4 针孔模型（透视）

相似三角形：像的大小 $y \propto \dfrac{Y}{Z}$，即"近大远小"。

- 第一步（除以深度）：$x_{norm}=\dfrac{X}{Z},\quad y_{norm}=\dfrac{Y}{Z}$（把物体放到离相机 1m 的虚拟平面上看）。
- 第二步（内参）：$u = f_x\, x_{norm} + c_x,\quad v = f_y\, y_{norm} + c_y$。
- $f_x,f_y$：焦距的像素表达（大 = 长焦 = 视野窄）；$c_x,c_y$：主点 ≈ 图像中心。

### 10.5 内参矩阵 $K$

$$ K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} $$

demo.avi 无真实内参 → v1 假设 $f_x=f_y=1000,\ c_x=720,\ c_y=540$（1440×1080）；题3 标定后替换。

### 10.6 齐次坐标与外参（为什么平移能写成矩阵乘）

- 平移不是线性变换，3×3 矩阵做不了 → 给点加一维 1（齐次坐标），旋转+平移合成 3×4 矩阵 $[R \mid t]$：

$$ P_{相机} = [R \mid t]\, P_{板} $$

- **本课最重要公式**（把 10.5/10.6 合并）：

$$ \text{像素} \sim K\,[R \mid t]\, P_{板} $$

未知量只有 $[R \mid t]$ → PnP 反解它。

### 10.7 数值例子（建立"米→像素"手感）

板中心在相机系 $(0.3,\,-0.1,\,3.0)$ m，$K$ 如上：
> 约定：相机坐标取 **X 右、Y 下、Z 前**（OpenCV 惯例），故 $Y=-0.1$ 表示点在光轴**上方**，投影 $v \approx 507 < c_y$，出现在画面中心上方（偏右偏上）。

$$ x_{norm}=\frac{0.3}{3.0}=0.10,\quad y_{norm}=\frac{-0.1}{3.0}\approx -0.0333 $$

$$ u = 1000\times 0.10 + 720 = 820, \qquad v = 1000\times(-0.0333)+540 \approx 507 $$

手感：3m 远处 X 方向 0.3m ≈ 像素差 100px（除以 Z 再乘 fx 的比例感），日后粗验 PnP 结果用。

### 10.8 L1 检查点

1. 用同一 $K$ 投影 $(0.6,\,0.0,\,4.0)$，求 $(u,v)$；
2. 若 $f_x=f_y=2000$，同一点投到哪？$f_x$ 变大意味着什么？
3. 齐次坐标最后一维"1"的作用？
4. $K[R \mid t]P_{板}$ 中 PnP 要求解的未知量是哪个？

### 10.9 L1 问答补充（概念澄清，面试用）

- **外参 $[R\mid t]$**：$R$($3\times3$) 与 $t$($3\times1$) 水平增广成 $3\times4$，表示"先旋转再平移"的**刚体变换**；完整齐次形式为 $\begin{bmatrix}R & t \\ 0\,0\,0 & 1\end{bmatrix}$（可连续相乘）。$t$ 的前三分量 = 板原点在相机系的位置。
- **转置的几何意义**：对**旋转矩阵**（正交矩阵，$R^{\mathsf T}R=I$）成立 $R^{-1}=R^{\mathsf T}$ = "拧回去"；对一般矩阵不成立（转置 ≠ 反向）。
- **两大层坐标系**（理解链路的骨架）：
  - 3D 真实世界（米）：板系 →(外参)→ 相机系，变换保真不丢信息；
  - 2D 图像世界（像素）：除以 $Z$（丢深度）→ 归一化 → $K$ → 像素。
  - **投影的本质 = 丢掉深度**：图像只记录"方向"，不记录距离——1m 远 (0.3,0.1) 与 10m 远 (3,1) 投到同一像素。PnP 靠"已知板物理尺寸"把深度反推回来。
  - 深度 $Z$ = 沿光轴方向的坐标，**不是**到光心的直线距离 $r=\sqrt{X^2+Y^2+Z^2}$。
  - $y=f\cdot\dfrac{Y}{Z}$：$Y$、$f$ 为常量 → $y\propto\dfrac{1}{Z}$，反比例 = 近大远小。
- **焦距**：针孔模型中 = 光心到成像平面距离；像素版 $f_x=f_{mm}\times s$（$s$ = 每毫米像素数）。
- **光轴**：过光心、垂直成像面，即相机系 $Z$ 轴；**主点**：光轴与成像面交点 $(c_x,c_y)$。
- **归一化坐标** $x_{norm}=X/Z=\tan(\text{与光轴的水平夹角})$："以深度为尺量横向偏移"→ 无量纲方向；角度相同 → 像素列相同。
- **K 如何"合成两步"**（易错点）：K 矩阵乘只含"乘 $f$ 加 $c$"，**除以 $Z$ 来自齐次坐标末尾除以第三维**。K 第三行 $(0,0,1)$ 把 $Z$ 护送到第三维，于是
  $$(KP_c)=\begin{pmatrix}f_xX+c_xZ\\ f_yY+c_yZ\\ Z\end{pmatrix}\ \xrightarrow{\ \div Z\ }\ u=f_x\tfrac{X}{Z}+c_x,\ v=f_y\tfrac{Y}{Z}+c_y$$
  这就是公式用 $\sim$（相差比例因子）的原因；"普通相等"写法为 $u=\dfrac{(KP_c)_1}{(KP_c)_3}$。
- **内参从哪来 / 不知道怎么办**：真实开发用棋盘格标定板拍 10+ 张 → `cv::calibrateCamera` 得 $K$；demo.avi 无内参 → v1 假设 $f_x{=}f_y{=}1000$（后果：**绝对距离尺度错，但 yaw/pitch 角度基本准**——角度来自画面比值）；仿真器（如河科）内参精确已知 → 题3 可验证绝对精度。**不需要问出题人**，标定是自瞄开发者的标准动作。

### 10.10 L1 检查点参考答案

1. $x_{norm}=0.6/4=0.15$ → $u=1000\times0.15+720=870$，$v=540$ → $(870,540)$ ✅
2. $u=2000\times0.15+720=1020$ → $(1020,540)$；$f_x=f_{mm}\times s$ 变大 = 长焦(视野窄)或传感器密度高 → 同一角度占更多像素、远板在画面中更大（"拉近"感）；不是"画质更高"。
3. 补"1"三重作用：① 维度匹配（3×4 才能乘 4×1）；② 使平移进入矩阵（仿射变换）；③ 尺度等价语义——支持末尾"除以第三维"表达透视。
4. $[R\mid t]$ 待 PnP 反解；$K$ 已知（假设/标定）；$P_{板}$ 已知（官方尺寸）。

### 10.11 相机来源备忘（2026-09-03 确认）

- **无 USB 相机**。题3 候选：① 笔记本内置摄像头（当前容器 `/dev/video*` 无设备，需 `ls /dev/video*` 确认可见）；② 手机当摄像头：IP Webcam（Android，HTTP 流 `http://<手机IP>:8080/video`，`VideoCapture` 可直接打开 URL，容器内可用）/ DroidCam 无线模式；③ 仿真（河科 RM 视觉仿真器，内参精确已知）。
- 任何新相机接入前需**棋盘格标定**拿自己的 $K$（把 L1 内参用起来）。

---

## 11. L2 正投影演示（题2 数学地基·代码版）

### 11.1 本课位置与交付物

- L1 是公式，L2 把公式变成**可运行、可验证**的正投影工具——它同时是 L4 PnP 的"标准答案生成器"（PnP 反解的验证基准）。
- 交付：`02_tracker/` 工程（`projection_demo`）→ `results/projection_static.png`（近/远板对照）、`results/projection_motion.avi`（6s 动画）、终端自检数字。

### 11.2 代码全景

| 函数 | 对应 L1 概念 |
|---|---|
| `rotateYaw(p,yaw)` | 绕板系 Y 轴旋转 $R_y(\theta)$ |
| `plateToCamera(p,yaw,t)` | 外参 $P_{cam}=R_y\,p+t$ |
| `projectToPixel(p,px)` | 透视除法 + 内参：$u=f_x X/Z+c_x$（手写，不调现成函数） |
| `plateCorners()` | 板系 3D 点：$(\pm hw,\pm hh,0)$，原点在板中心 |
| `drawPlate`/`drawCrosshair`/`printPlateInfo` | 画图与验证辅助 |

main 三步：① 自检（复算 L1 检查点数字）→ ② 静态场景（近/远板同方向角）→ ③ 动画（横移 + 转身）。

### 11.3 数据流主线（正投影 = L4 的"正向尺子"）

```
板系4角(±hw,±hh,0) --plateToCamera(外参)--> 相机系(X,Y,Z)
  --projectToPixel(÷Z, K)--> 像素(u,v) --drawPlate--> 画面四边形
```

### 11.4 验证证据（可复现的数字）

| 验证点 | 结果 |
|---|---|
| 投影公式正确 | 自检 (870,540)、(820,506.7) 与手算一致 |
| 图像只记方向 | 近/远板中心都是 u=820 |
| 近大远小 | 板宽像素 45.0 → 22.5（距离翻倍减半） |
| 转身压缩 | yaw ±23° 时板宽明显变窄 → bbox 不够用的伏笔 |

### 11.5 L2 问答笔记

- **匿名命名空间 `namespace{}`**：文件私有工具（内部链接），只在本 .cpp 可见、防跨文件重名；对比 `namespace rm_vision`（公开 API，跨文件共享）。私有工具进匿名、公开接口进有名。
- **裸 `{}` 局部作用域**：变量出块即死；何时用：① 名字复用/防冲突 ② 让 RAII 资源提前释放 ③ 表达"一个自包含步骤" ④ switch 内声明。原则：作用域越小，"作案面积"越小。
- **`project` 动词义 = 投影**（名词才是项目）；`projectToPixel` 强调"投影动作"。
- **返回值 vs 异常**：返回值可选（忘查最坏是旧值，不崩），异常强制（没人 catch 直接终止）；"点在相机后方"属预期内常态 → 用 bool 返回让调用方跳过，不抛异常。
- **`scene`** = 场景成品图（背景 + 所有图层）；**BGR**：`Scalar(30,30,30)` 是深灰底（OpenCV 颜色顺序 BGR！）。
- **crosshair**：cross×hair（细丝）→ 十字准星，源自瞄准镜十字丝。
- **`t` = translation 平移向量**（板原点在相机系的位置），不是点；与 R 合为 $[R\mid t]$。
- **yaw/pitch/roll**：绕 Y/X/Z 轴（云台语境 Y 向上）；demo 用 yaw 因为"演示转身 + 对应自瞄主战场"。
- **Ry 不是"R 的 y 分量"而是绕 Y 轴旋转的完整矩阵**；三个基本旋转矩阵是背下来的（"绕谁谁不变"）。用 Ry 是因为我们把板坐标系的"上"定义在 Y 轴（板宽沿 X、高沿 Y）→ "转身绕竖轴"= 绕 Y。**轴的选择 = 运动设计 × 坐标系朝向的约定**。
- **`hw/hh` = half width/height**（板中心原点 → 角坐标天然是"半"）。
- **drawPlate 的 pts/px**：pt=point、px=pixel；把 3D 角投影成 2D 再画——**故意丢 Z**（投影后 Z 用完即弃，绘图只认 2D）。
- **printPlateInfo**：打印到**终端(stdout)**不是图片；取 4 角 min/max u,v 得外接矩形中心/宽高，供手算核对；`printf` = C 语言格式化打印（C++ 用 `<cstdio>` 的 `std::printf`）。
- **kFrames=180** → 30fps 下 6s（`@` = at）；只在一处用的常量就近放块内（作用域最小化）。
- **phase = 相位**：$2\pi i/kFrames$ 是动画周期角度；x、yaw 用 sin 是为了平滑 + 周期整取可循环播放；yaw 频率翻倍让"转身压缩"效果多出现。
- **snprintf**：s=string（写缓冲区）、n=size（限长防溢出），对比 printf（写终端、无界）。
- **writer.release()**：析构会自动释放，功能上可省；但显式写保证"封口(file 写完) → 才打印 saved"，顺序语义更严谨。

### 11.6 L3 预告

真实检测给的是**轴对齐外接框**，板转身时 bbox 与板四边形不重合 → 需要"从框长出四角点"：v1 框角近似 → 打通 PnP；v2 灯条精定位升级。

---

## 12. L3 四角点 v1：corner_demo（进度：L1-L3 ✅，L4 进行中）

### 12.1 为什么需要"四角点"

- L4 PnP 需要"3D 点 ↔ 2D 点"的一一对应；题1 YOLO 只给轴对齐框（cv::Rect）。
- bbox ≠ 板的四边形：板转身（透视压缩）时框角 ≠ 板角。v1 先接受"框角 ≈ 板角"（板近正对时误差小），v2 再上灯条精定位。

### 12.2 顺序约定（L4 的命根子）

- 2D 角点顺序：**0 TL(左上) → 1 TR → 2 BR → 3 BL（顺时针）**；
- 板坐标系建模：**X 右、Y 下、Z 前**（与相机同向）→ 正对相机的板 R≈I，便于验证；
- 原因：`solvePnP` 的 3D 点列表与 2D 点列表**按下标一一对应**，顺序错 → 姿态乱码。

### 12.3 代码结构（02 第一次链接 01）

```cmake
add_subdirectory(../01_detector 01_detector EXCLUDE_FROM_ALL)
target_link_libraries(corner_demo PRIVATE armor_detector ...)
```

- 检测逻辑**无第二份**（corner_demo 全文无 letterbox/blob/NMS，只调 `detect()`）；
- L3 真正新增：`rectToCorners`（框→4 有序角点）、`drawCorners`（0-3 彩色圆点）、顺序约定。

### 12.4 外壳重复的观察（Rule of Three）

- `demo_main.cpp`(158 行) 与 `corner_demo.cpp`(163 行) 约半以上行完全一致——重复的是**演示外壳**（参数/开视频/主循环/统计），不是检测代码；
- **两层复用**：库层（检测逻辑）✅ 无重复；外壳层 ❌ 暂时复制；
- 工程经验：**重复两次忍、第三次才抽象**（Rule of Three）；L4 若出第三个视频 demo，再抽共享 runner（回调式"每帧做什么"）；
- 更长远的理由：题3 用 ROS2 后"视频循环"变成"图像话题回调"，这层外壳活不到题3，不值得提前抽象。

### 12.5 验证

- corner_demo 240 帧：63.3% 帧有装甲、179 组四角点、54.6ms/帧；
- 视觉验证点：看板转身的帧，"框角"与"板真角"的偏差 → v2 动机。
