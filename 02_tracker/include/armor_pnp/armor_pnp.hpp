#pragma once

// armor_pnp：装甲板 PnP 位姿解算的共用实现（评测/演示/节点可复用）
//
// 与 03_visualization 主节点中的解算逻辑一致：IPPE 多解 + r3.z>0 破镜像 +
// 重投影误差阈值 + 物理闸门(z>0.2, dist<20m)。此处额外返回重投影误差与 yaw，
// 供评测脚本量化角点质量。

#include <vector>

#include <opencv2/core.hpp>

namespace rm_tracker {

// 单次解算结果
struct PoseResult {
    cv::Mat tvec;         // 3x1，板心在相机坐标系的位置 (m)
    double reproj_err_px; // 四个角点的平均重投影误差 (px)
    double yaw_deg;       // 板法线相对"正对相机"的水平偏角 (deg)，正对≈0
};

// 板坐标系角点（顺序 TL/TR/BR/BL，与 rectToCorners 对应）
std::vector<cv::Point3d> plateObjectPoints(double plate_w_m = 0.135, double plate_h_m = 0.125);

// 灯条端点物体模型（顺序 TL/TR/BR/BL）：
// 宽 = 两块灯条外侧间距（≈装甲板宽 135mm），高 = 灯条长度（标称 56mm，本素材实测最优 ≈52mm）
// 说明：四关键点模型输出的 4 个点是"灯条端点"，用它做 PnP 时物体模型必须用本函数；
//       实测重投影 1.09px（135x56），而用板四角模型 8.09px、反向绕向 25.6px。
std::vector<cv::Point3d> barEndObjectPoints(double armor_w_m = 0.135, double bar_len_m = 0.056);

// 轴对齐 bbox -> 四角点（v1 角点近似）
std::vector<cv::Point2d> rectToCorners(const cv::Rect& rect);

// 内参近似：按当前分辨率 + 假定水平 FOV≈72° 推导（与主节点一致，未标定）
cv::Mat computeK(int width, int height);

// 解算板位姿；成功返回 true 并填充 out
bool solveArmorPose(
    const std::vector<cv::Point3d>& object,
    const std::vector<cv::Point2d>& image,
    const cv::Mat& K,
    PoseResult& out
);

} // namespace rm_tracker
