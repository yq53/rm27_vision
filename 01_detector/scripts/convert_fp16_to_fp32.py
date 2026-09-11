#!/usr/bin/env python3
"""把 fp16 ONNX 转成纯 fp32 ONNX（供 OpenCV DNN 使用）。

背景：第三方四关键点模型常以 fp16 导出（Ultralytics half=True），OpenCV 的 ONNX
解析器无法加载（Cast 节点报 "unknown input ... of node /model.0/conv/Conv"）。

适用对象：**仅 cv2.dnn 后端**。
  - 若使用 ONNX Runtime 后端（USE_ONNXRUNTIME=ON），**请直接用原始 fp16 模型**：
    ORT 原生支持 fp16，而本脚本产物会残留 float16 类型声明，ORT 会报类型不匹配。
本脚本做两件事：
  1) 所有 FLOAT16 初始化器 -> FLOAT32
  2) 删除所有 Cast 节点，并把其消费者重连到 Cast 的输入（等价于去掉类型转换）

用法：
    python3 convert_fp16_to_fp32.py <输入 fp16.onnx> <输出 fp32.onnx>

依赖：pip install onnx
"""

import sys

import onnx
from onnx import TensorProto, numpy_helper


def convert(src: str, dst: str) -> None:
    model = onnx.load(src)

    # 1) fp16 权重 -> fp32
    casted = 0
    for init in model.graph.initializer:
        if init.data_type == TensorProto.FLOAT16:
            array = numpy_helper.to_array(init).astype("float32")
            init.CopyFrom(numpy_helper.from_array(array, init.name))
            casted += 1

    # 2) 删除 Cast 节点，记录 输出名 -> 输入名 的重定向
    redirect = {}
    kept = []
    for node in model.graph.node:
        if node.op_type == "Cast":
            redirect[node.output[0]] = node.input[0]
        else:
            kept.append(node)
    del model.graph.node[:]
    model.graph.node.extend(kept)

    def resolve(name: str) -> str:
        while name in redirect:
            name = redirect[name]
        return name

    for node in model.graph.node:
        for i, inp in enumerate(node.input):
            if inp in redirect:
                node.input[i] = resolve(inp)
    for out in model.graph.output:
        if out.name in redirect:
            out.name = resolve(out.name)

    onnx.checker.check_model(model)
    onnx.save(model, dst)
    print(f"fp16 初始化器 -> fp32: {casted} 个; 删除 Cast 节点: {len(redirect)} 个")
    print(f"已保存: {dst}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    convert(sys.argv[1], sys.argv[2])
