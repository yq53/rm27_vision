#pragma once

// armor_corner：局部角点精修（v2）
//
// 思路（参考社区 24 中南方案 / 上科大 2026 方案的做法）：
//   1) 用检测框（YOLO bbox）裁出 ROI —— 把搜索范围限制在板附近，避免全图灯条配对失败；
//   2) ROI 内二值化找亮灯条轮廓（灯条是画面里最亮的竖向结构）；
//   3) 对每个灯条轮廓做 PCA 取主方向，沿主方向投影取两端点 → 得到灯条端点；
//   4) 在端点附近沿主方向搜索亮度梯度最大处，修正"二值化阈值漂移"带来的端点偏差；
//   5) 左右灯条各两个端点拼成板四角（TL/TR/BR/BL，与 plateObjectPoints 一一对应）。
//
// 找不到足够灯条时回退为 bbox 四角（refined=false），保证不劣化。

#include <vector>

#include <opencv2/core.hpp>

namespace rm_tracker {

struct CornerRefineResult {
    std::vector<cv::Point2d> corners; // TL, TR, BR, BL
    bool refined = false;             // true = 由灯条精修得到；false = 回退 bbox 四角
    int bars_found = 0;               // 参与拼角的灯条数（0/1/2）
};

// bgr：原图；bbox：检测框；expand_ratio：ROI 相对 bbox 的外扩比例
CornerRefineResult refineArmorCorners(const cv::Mat& bgr, const cv::Rect& bbox, double expand_ratio = 0.30);

} // namespace rm_tracker
