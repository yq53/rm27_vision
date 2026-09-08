#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace rm_tracker {

// L8：框内（ROI）灯条精定位 —— "NN 粗修给框，CV 精修找灯条"分工的 CV 半边。
//
// 输入：YOLO 框裁出的 ROI（通常含 1~2 根灯条）
// 输出：detectBars 找到的候选灯条；pairArmor 把两根配成一块装甲板
//
// 判据设计参考同济 sp_vision_25 detector.cpp：
//   灯条三连：接近竖直(angle_error) + 细长(ratio) + 足够长(length)
//   配对约束：长度相近 + 竖直范围重叠 + 水平间距合理

struct LightBar {
    cv::Point2f center;
    cv::Point2f top; // 长边上端点（y 较小）
    cv::Point2f bottom; // 长边下端点
    float length_px = 0.f;
    float ratio = 0.f; // 长/短（细长条 > 1）
    float angle_error_deg = 0.f; // 与竖直方向的偏差（0=竖直）
};

class LightBarDetector {
public:
    struct Params {
        double threshold = 120.0; // 二值化阈值（暗场灯条亮）
        double min_length_px = 18.0; // 灯条最小像素长
        double min_ratio = 1.8; // 长/短 最小（太胖不是灯条）
        double max_ratio = 15.0; // 长/短 最大
        double max_angle_deg = 35.0; // 与竖直最大偏角
        double min_contour_area = 40.0;
        // 配对约束
        double max_len_diff_ratio = 0.5; // 两灯条长度差 / 较短者
        double min_gap_len_ratio = 0.3; // 水平间距 / 较短灯条长
        double max_gap_len_ratio = 4.0;
    };

    explicit LightBarDetector(const Params& params);
    LightBarDetector(); // 用默认 Params（实现在 .cpp，规避 clang 的 NSDMI 默认实参限制）

    // 在单通道 ROI（灰度/通道差分）里找灯条候选，按 center.x 升序返回
    std::vector<LightBar> detectBars(const cv::Mat& single_channel_roi) const;

    // 把两根灯条配对成一块板；成功输出四角点（ROI 坐标，顺序：左顶、右顶、右底、左底）
    bool
    pairArmor(const LightBar& left, const LightBar& right, std::vector<cv::Point2f>& corners_out)
        const;

    const Params& params() const {
        return params_;
    }

private:
    Params params_;
};

} // namespace rm_tracker
