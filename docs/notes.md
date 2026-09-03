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
6. 自测清单
7. 下一步

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
   ```
   x = (cx − w/2 − pad_w) / ratio
   y = (cy − h/2 − pad_h) / ratio
   bw = w / ratio;  bh = h / ratio
   ```
   再 `rect &= Rect(0,0,cols,rows)` 裁越界（`&` 被重载为矩形求交）。
5. 组装 + NMS（119~131）：阈值过滤后的框 push 进三个平行 vector（reserve 预留）→ `NMSBoxes` 收敛重叠框（indices 是保留框的下标）→ 组装成 `Armor` 返回。

**数值例子**：原图 1440×1080 → ratio≈0.444，缩到 640×480，pad_h=80。画布检出中心(400,300)宽高(200,100) → 原图 x=(400−100−0)/0.444≈675、y=(300−50−80)/0.444≈383。

---

## 6. 自测清单（面试速查）

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

---

## 7. 下一步

- [ ] 通读 `01_detector/src/armor_detector.cpp`（letterbox 数学、blobFromImage、ONNX 输出的 5 个数、坐标还原）—— 题1 核心逻辑（问答见第 5 节）
- [ ] 通读 `01_detector/src/demo_main.cpp`（视频循环、VideoWriter、drawArmors、FPS 统计）
- [ ] 题2 预告：tracker 需要**装甲板四角点**做 PnP，而 YOLO 只给轴对齐包围框 → 需要框内找灯条/角点或换方案
- [ ] 提交规范：代码每完成一个里程碑 commit 一次，README 记录运行方式
