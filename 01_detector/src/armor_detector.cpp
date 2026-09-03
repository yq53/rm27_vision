#include "armor_detector/armor_detector.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rm_vision {

// 构造函数(加载模型)实现
ArmorDetector::ArmorDetector(std::string model_path):
    net_(cv::dnn::readNetFromONNX(std::move(model_path))) {
    
    // 检测加载是否成功
    if (net_.empty()) {
        throw std::runtime_error("Failed to load ONNX model");
    }
    
    // 推理后端：OpenCV 自带引擎 + CPU。
    // 若 OpenCV 编译了 CUDA 且有显卡，可换成 DNN_TARGET_CUDA 提速。
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

// letterbox函数实现
cv::Mat ArmorDetector::letterbox(const cv::Mat& src, float& ratio, int& pad_w, int& pad_h) {
    const int src_w = src.cols;
    const int src_h = src.rows;

    // 长边恰好缩放到 640，短边按同一比例缩放（图像不会变形）
    ratio = static_cast<float>(kInputSize) / std::max(src_w, src_h);
    const int new_w = cvRound(src_w * ratio);
    const int new_h = cvRound(src_h * ratio);

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));

    // 计算灰边 padding，让缩放后的图像居于 640x640 正中
    pad_w = (kInputSize - new_w) / 2;
    pad_h = (kInputSize - new_h) / 2;

    // 灰底 + 把缩放图贴到中间；灰度值 114 与训练时保持一致
    cv::Mat padded(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(pad_w, pad_h, new_w, new_h)));
    return padded;
}

// detect函数实现
std::vector<Armor> ArmorDetector::detect(const cv::Mat& frame) {
    std::vector<Armor> armors;
    if (frame.empty()) {
        return armors;
    }

    // ---- 1. 预处理：letterbox -> blob(NCHW, RGB, 归一化到 0~1) ----
    float ratio = 0.0f;
    int pad_w = 0;
    int pad_h = 0;
    const cv::Mat input = letterbox(frame, ratio, pad_w, pad_h);

    const cv::Mat blob = cv::dnn::blobFromImage(
        input,
        1.0 / 255.0, // 像素值映射到 [0,1]
        cv::Size(kInputSize, kInputSize), // 输出尺寸（已 letterbox，不再裁剪）
        cv::Scalar(), // 不减均值
        true, // swapRB: BGR -> RGB（训练时是 RGB）
        false // 不做 crop
    );

    // ---- 2. 前向推理 ----
    net_.setInput(blob);
    const cv::Mat output = net_.forward();
    const float* data = output.ptr<float>();

    // ---- 3. 解析输出 ----
    // YOLOv8 的导出结果有两种排布：(1, C, N) 或 (1, N, C)
    // 这里 C = 4 个坐标 + 类别数 = 5，N = 候选框数量。
    // 通过比较第 1、2 维的大小自动判断是哪种排布，保证鲁棒。
    const int dim1 = output.size[1];
    const int dim2 = output.size[2];
    const bool channel_major = dim1 < dim2;
    const int num_features = channel_major ? dim1 : dim2; // C
    const int num_boxes = channel_major ? dim2 : dim1; // N

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    boxes.reserve(num_boxes);
    scores.reserve(num_boxes);
    class_ids.reserve(num_boxes);

    for (int i = 0; i < num_boxes; ++i) {
        // 取第 i 个框的 (cx, cy, w, h)
        const float cx = channel_major ? data[0 * num_boxes + i] : data[i * num_features + 0];
        const float cy = channel_major ? data[1 * num_boxes + i] : data[i * num_features + 1];
        const float w = channel_major ? data[2 * num_boxes + i] : data[i * num_features + 2];
        const float h = channel_major ? data[3 * num_boxes + i] : data[i * num_features + 3];

        // 单类别：索引 4 就是置信度；多类别时应取各类别分数的最大值
        float max_score = 0.0f;
        for (int f = 4; f < num_features; ++f) {
            const float score =
                channel_major ? data[f * num_boxes + i] : data[i * num_features + f];
            max_score = std::max(max_score, score);
        }
        if (max_score < conf_threshold_) {
            continue;
        }

        // 把 640x640 坐标系还原到原图：先减 padding 再除以 ratio
        const int x = cvRound((cx - w / 2 - pad_w) / ratio);
        const int y = cvRound((cy - h / 2 - pad_h) / ratio);
        const int bw = cvRound(w / ratio);
        const int bh = cvRound(h / ratio);

        // 裁到图像范围内（避免越界）
        cv::Rect rect(x, y, bw, bh);
        rect &= cv::Rect(0, 0, frame.cols, frame.rows);

        boxes.push_back(rect);
        scores.push_back(max_score);
        class_ids.push_back(0);
    }

    // ---- 4. NMS：同一装甲板的重叠框只留置信度最高的一个 ----
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, conf_threshold_, nms_threshold_, indices);

    armors.reserve(indices.size());
    for (const int idx: indices) {
        armors.push_back(Armor { boxes[idx], scores[idx], class_ids[idx] });
    }
    return armors;
}

} // namespace rm_vision
