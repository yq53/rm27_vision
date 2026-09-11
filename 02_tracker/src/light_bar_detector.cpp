// [负结果] v2 灯条精定位原型（demo.avi 配对率仅 7%，已冻结）—— 仅 lightbar_demo 引用
#include "armor_tracker/light_bar_detector.hpp"

#include <algorithm>
#include <cmath>

namespace rm_tracker {

LightBarDetector::LightBarDetector(): params_() {}

LightBarDetector::LightBarDetector(const Params& params): params_(params) {}

std::vector<LightBar> LightBarDetector::detectBars(const cv::Mat& single_channel_roi) const {
    std::vector<LightBar> bars;
    if (single_channel_roi.empty()) {
        return bars;
    }

    // 二值化：灯条是亮区（暗场视频里亮条最突出）
    cv::Mat binary;
    cv::threshold(single_channel_roi, binary, params_.threshold, 255, cv::THRESH_BINARY);

    // 闭运算：把灯条内部可能的小断裂连起来（竖直小核）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 5));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour: contours) {
        if (cv::contourArea(contour) < params_.min_contour_area) {
            continue;
        }
        const cv::RotatedRect rect = cv::minAreaRect(contour);

        // 用外接矩形的相邻边长找"长边方向"（不依赖 OpenCV angle 的坑人约定）
        cv::Point2f pts[4];
        rect.points(pts);
        const cv::Point2f e0 = pts[1] - pts[0];
        const cv::Point2f e1 = pts[2] - pts[1];
        const float l0 = cv::norm(e0);
        const float l1 = cv::norm(e1);
        if (l0 < 1e-3f || l1 < 1e-3f) {
            continue;
        }
        const bool e0_long = l0 >= l1;
        const float long_len = e0_long ? l0 : l1;
        const float short_len = e0_long ? l1 : l0;
        const cv::Point2f long_dir = (e0_long ? e0 : e1) / long_len;

        // 与竖直方向的夹角：|cos| = |dir.y|
        const float angle_deg =
            std::acos(std::min(1.0f, std::abs(long_dir.y))) * 180.0f / static_cast<float>(CV_PI);

        const float ratio = long_len / std::max(short_len, 1e-3f);
        if (ratio < params_.min_ratio || ratio > params_.max_ratio) {
            continue;
        }
        if (long_len < params_.min_length_px) {
            continue;
        }
        if (angle_deg > params_.max_angle_deg) {
            continue;
        }

        // 上下端点：长边中点沿长边方向各取一半；y 小者为 top
        const cv::Point2f center = rect.center;
        const cv::Point2f cand_top = center - long_dir * (long_len / 2.0f);
        const cv::Point2f cand_bot = center + long_dir * (long_len / 2.0f);
        const bool top_is_first = cand_top.y <= cand_bot.y;

        LightBar bar;
        bar.center = center;
        bar.top = top_is_first ? cand_top : cand_bot;
        bar.bottom = top_is_first ? cand_bot : cand_top;
        bar.length_px = long_len;
        bar.ratio = ratio;
        bar.angle_error_deg = angle_deg;
        bars.push_back(bar);
    }

    std::sort(bars.begin(), bars.end(), [](const LightBar& a, const LightBar& b) {
        return a.center.x < b.center.x;
    });
    return bars;
}

bool LightBarDetector::pairArmor(
    const LightBar& left,
    const LightBar& right,
    std::vector<cv::Point2f>& corners_out
) const {
    // 长度相近
    const float shorter = std::min(left.length_px, right.length_px);
    const float len_diff = std::abs(left.length_px - right.length_px);
    if (shorter < 1e-3f || len_diff / shorter > params_.max_len_diff_ratio) {
        return false;
    }

    // 竖直范围有重叠（两灯条应大致同高）
    const float overlap =
        std::min(left.bottom.y, right.bottom.y) - std::max(left.top.y, right.top.y);
    if (overlap < 0.4f * shorter) {
        return false;
    }

    // 水平间距合理：两灯条不能叠在一起，也不能远得像两块板
    const float gap = std::abs(right.center.x - left.center.x);
    if (gap / shorter < params_.min_gap_len_ratio || gap / shorter > params_.max_gap_len_ratio) {
        return false;
    }

    // 顺序：左顶、右顶、右底、左底（配合方案 B 的 object points）
    corners_out = { left.top, right.top, right.bottom, left.bottom };
    return true;
}

} // namespace rm_tracker
