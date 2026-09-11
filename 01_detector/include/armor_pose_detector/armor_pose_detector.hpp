#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

// ArmorPoseDetector：装甲板"四关键点"模型（YOLOv8-Pose 系）的推理封装
//
// 与 ArmorDetector（bbox 检测）并列的第二种检测器实现，互不影响：
//   - ArmorDetector      输出轴对齐 bbox（角点靠 bbox 四角近似）
//   - ArmorPoseDetector  输出 4 个灯条端点（可直接用于 PnP，无需 bbox 近似）
//
// 模型输出布局（已实测确认，21 x N，N=6300）：
//   row[4..12]  9 个类别分数
//   row[13..20] 4 个关键点 (x, y) —— 灯条端点（无逐点置信度）
//   其余行未使用
// 关键点语义与顺序（实验验证：用"灯条端点矩形 135x56mm"做 PnP，重投影 1.09px；
// 反向绕向 25.6px，板四角模型 8.09px）：
//   原始索引 0 = 左灯条上端, 3 = 右灯条上端, 2 = 右灯条下端, 1 = 左灯条下端
//   本类输出统一重排为 TL, TR, BR, BL，便于直接与 PnP 物体点对应。
//
// 后端：
//   默认 cv2.dnn —— 零额外依赖，但本仓库使用的四关键点模型在其上不可用，两种精度各挂一处：
//     原始模型：入口 Cast 节点（graph_input_cast_0）使 OpenCV 4.x 的 ONNX importer 解析失败
//     转 fp32 后：Cast 问题消失，改为 decode 段 NaryEltwise 广播断言失败
//   RM_USE_ONNXRUNTIME 编译开关 —— 使用 ONNX Runtime C++，本模型的可行后端
//   模型文件：ORT 用原始导出件（对外声明 fp32 输入/输出，内部以 fp16 计算）；
//             scripts/convert_fp16_to_fp32.py 的产物只适用于 cv2.dnn

namespace rm_vision {

// ArmorPose结构体
struct ArmorPose {
    cv::Rect rect;                    // = 4 个灯条端点的外接矩形
    float confidence;                 // 置信度
    int class_id;                     // 类别 id
    std::array<cv::Point2f, 4> kpts;  // 灯条端点，顺序 TL, TR, BR, BL
};

class ArmorPoseDetector {
public:
    explicit ArmorPoseDetector(std::string model_path);
    ~ArmorPoseDetector();

    // detect函数
    std::vector<ArmorPose> detect(const cv::Mat& frame);

    // 设置置信度阈值
    void setConfidenceThreshold(float threshold) {
        conf_threshold_ = threshold;
    }

    // 设置NMS阈值
    void setNmsThreshold(float threshold) {
        nms_threshold_ = threshold;
    }

private:
    // letterbox：保持宽高比缩放到 640x480，其余填灰（114）
    static cv::Mat letterbox(const cv::Mat& src, float& scale, int& pad_x, int& pad_y);

    // 从 (1, C, N) 或 (1, N, C) 输出张量解码为检测结果
    std::vector<ArmorPose> decode(
        const float* data,
        int dim1,
        int dim2,
        int frame_w,
        int frame_h,
        float scale,
        int pad_x,
        int pad_y
    ) const;

    std::string model_path_;
    float conf_threshold_ = 0.5f;
    float nms_threshold_ = 0.45f;

#ifdef RM_USE_ONNXRUNTIME
    struct OrtImpl;                // 隐藏 ORT 头文件，避免污染使用者
    std::unique_ptr<OrtImpl> ort_; // ONNX Runtime 后端
#endif

    static constexpr int kInW = 640;
    static constexpr int kInH = 480;
    static constexpr int kNumKpts = 4;
};

} // namespace rm_vision
