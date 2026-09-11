# Infantry-v8n —— 第三方权重（非本仓库原创）

本目录下的 ONNX 权重来自 **深圳大学 RobotPilots 战队**在 RoboMaster 社区开源的
「RM2026-视觉模型统一部署库与识别模型开源」（该开源项目亦被本次考核题面列为参考资料之一）。

本工程用它作为**题1 的第二种检测器**（`detector:=pose`，四关键点直出灯条端点），
默认检测器仍是我们自训的 bbox 模型，见仓库根 `README.md`「题1 → 方案 B」。

## 文件与校验

| 文件 | 大小 | sha256 |
|---|---|---|
| `Infantry-v8n-fp16-20260726-D1.8w-B16.onnx` | 6,429,469 B | `8b735d0e68faf330e351e353be9f792954512cfaf64f054123c466f755a6fe1e` |

```bash
sha256sum models/third_party/Infantry-v8n/Infantry-v8n-fp16-20260726-D1.8w-B16.onnx
```

## 模型元信息（读自 ONNX 元数据）

| 项 | 值 |
|---|---|
| `task` | `pose` |
| `imgsz` | `[480, 640]`（4:3，与相机/素材同比例） |
| `stride` / `batch` | `32` / `1` |
| 输入 | `images`：`[1, 3, 480, 640]`，**FLOAT** |
| 输出 | `output0`：`[1, 21, 6300]`，**FLOAT** |
| 类别数 | 9（`class0` … `class8`） |
| 许可 | `AGPL-3.0 License (https://ultralytics.com/license)` |

### 两个容易踩的点

**① 文件名叫 fp16，但对外的张量类型是 fp32。** fp16 只在图内部：入口 `graph_input_cast0`
把 fp32 输入转 fp16 参与计算，出口 `graph_output_cast0` 再转回 fp32。因此：

- ONNX Runtime 直接用**这份原始导出件**（C++ 侧 `CreateTensor<float>` / `GetTensorData<float>()` 是对的）；
- `01_detector/scripts/convert_fp16_to_fp32.py` 的产物**只适用于 cv2.dnn**，ORT 用它反而会报类型不匹配。

**② 这份导出件不能随手替换。** `ArmorPoseDetector::decode()` 按本文件的输出布局写死
（`row4..12` = 9 个类别分数，`row13..20` = 4 个关键点 `(x,y)`），且 4 个点的语义是
**两根灯条的端点、不是板四角**，索引映射为 `kp0/kp3/kp2/kp1 → TL/TR/BR/BL`。
换成别的导出件（哪怕同一网络、不同日期/后端）都必须**重新验证这两件事**，
否则不会报错、只会静默出错（典型症状：PnP 重投影误差异常变大）。验证方法见
`docs/notes.md` §20.2；布局与语义的完整证据也在那一节。

## 许可

- 本权重**不是本仓库原创**，其标注许可为 **AGPL-3.0**（Ultralytics）。
- 本仓库代码为 **MIT**（见仓库根 `LICENSE`），但**不对本目录下的权重做 MIT 声明**；
  二次分发或商业使用请遵循其自身许可（Ultralytics 对 AGPL 模型用于闭源商业场景要求企业许可）。
- 仓库根 `README.md`「许可说明」按内容分层列明了三类内容各自的许可。
