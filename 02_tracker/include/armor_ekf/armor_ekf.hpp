#pragma once

#include <opencv2/opencv.hpp>

namespace rm_tracker {

// L7：装甲板跟踪用的卡尔曼滤波（EKF 框架，当前模型线性 → 退化为 KF）。
//
// 状态 x = (x, y, z, vx, vy, vz)ᵀ  —— 板心位置 + 速度（相机坐标系，单位 m/m/s）
// 运动模型：常速（位置 += 速度*dt），过程噪声只"推方差不推均值"
// 测量模型：z = H x（H 只取位置 3 维），z 由 PnP 每帧解出（"测量 = PnP 位姿"）
//
// 设计为"通用 EKF 形态"：将来若把测量换成像素角点、或状态里加角度，
// 只需把常量 F_/H_ 换成"由当前状态计算雅可比"的函数，类骨架不用改。
class ArmorEKF {
public:
    ArmorEKF();

    // 用第一个 PnP 位姿初始化状态（速度置 0，位置用测量，P 给初值）
    void init(const cv::Mat& z3x1);

    // 预测一步（无论有没有测量，每帧都调用；漏检帧 = 只预测不更新）
    void predict(double dt);

    // 有测量时更新（z3x1 = PnP 解出的板心位置 (x,y,z)ᵀ）
    void update(const cv::Mat& z3x1);

    bool initialized() const {
        return initialized_;
    }

    // 6x1 状态（调试/绘图用）
    const cv::Mat& state() const {
        return x_;
    }

    // 6x6 协方差（调试/绘图用）
    const cv::Mat& covariance() const {
        return P_;
    }

    // ---- 调参口（L7 结尾的"Q/R 手感练习"用） ----
    void setMeasurementNoiseStd(double pos_sigma_m); // R = diag(sigma²)*3
    void setProcessAccelVariance(double q); // 模型没抓到的加速度方差 (m/s²)²

private:
    // 由 dt 构造常速运动矩阵 F(6x6) 与过程噪声 Q(6x6)
    cv::Mat buildF(double dt) const;
    cv::Mat buildQ(double dt) const;

    cv::Mat H_; // 3x6 测量矩阵（只取位置）
    cv::Mat R_; // 3x3 测量噪声的协方差矩阵（v ~ N(0, R)）
    double q_; // 过程加速度方差 (m/s²)²

    cv::Mat P_; // 6x6 状态协方差
    cv::Mat x_; // 6x1 状态均值

    bool initialized_ = false;
};

} // namespace rm_tracker
