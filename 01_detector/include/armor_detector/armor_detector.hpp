#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rm_vision {

// 装甲板检测结果
struct Armor {
    cv::Rect rect; // 原图像素坐标系下的包围框
    float confidence; // 置信度 (0~1)
    int class_id; // 类别 id（本工程只有 0: armour）
};

// 装甲板检测器类声明
class ArmorDetector {
public:
    // 构造函数
    explicit ArmorDetector(std::string model_path);

    // 对一帧图像做检测，返回所有装甲板（已做阈值过滤 + NMS）。
    // 注意不是 const：cv::dnn::Net 内部有状态，forward 会修改它
    std::vector<Armor> detect(const cv::Mat& frame);

    void setConfidenceThreshold(float threshold) {
        conf_threshold_ = threshold;
    }
    void setNmsThreshold(float threshold) {
        nms_threshold_ = threshold;
    }

private:
    // letterbox 预处理：保持宽高比缩放到 kInputSize，剩余区域填灰
    // 返回处理后的图，并通过引用参数带回还原坐标所需的信息
    static cv::Mat letterbox(const cv::Mat& src, float& ratio, int& pad_w, int& pad_h);

    cv::dnn::Net net_;
    float conf_threshold_ = 0.35f;
    float nms_threshold_ = 0.45f;

    // 训练时固定的输入尺寸，推理必须保持一致
    static constexpr int kInputSize = 640;
};

} // namespace rm_vision
