// [核心] 常速卡尔曼滤波（6D 状态：位置 + 速度）—— 主节点依赖，删除则无法启动
#include "armor_ekf/armor_ekf.hpp"

namespace rm_tracker {

namespace {
    constexpr double kInitPosVar = 0.05 * 0.05; // 初值位置方差 (m²)
    constexpr double kInitVelVar = 4.0; // 初值速度方差 (m/s)²：先不知道速度，给大
} // namespace

ArmorEKF::ArmorEKF() {
    // 测量矩阵：H = [I3 | 0]，从 6 维状态里抽位置 3 维
    H_ = cv::Mat::zeros(3, 6, CV_64F);
    H_.at<double>(0, 0) = 1.0;
    H_.at<double>(1, 1) = 1.0;
    H_.at<double>(2, 2) = 1.0;

    // 默认测量噪声：PnP 位置抖动量级 ≈ 0.02 m
    setMeasurementNoiseStd(0.02);
    // 默认过程噪声：敌方加速度特征量级 ~2 m/s² → 方差 4 (m/s²)²
    setProcessAccelVariance(4.0 );

    x_ = cv::Mat::zeros(6, 1, CV_64F);
    P_ = cv::Mat::eye(6, 6, CV_64F);
}

// 设置测量噪声协方差R
void ArmorEKF::setMeasurementNoiseStd(double pos_sigma_m) {
    R_ = cv::Mat::eye(3, 3, CV_64F) * (pos_sigma_m * pos_sigma_m);  // 假设各变量间独立(协方差为0)
}

// 设置过程噪声q
void ArmorEKF::setProcessAccelVariance(double q) {
    q_ = q;
}

// 构建运动矩阵F
cv::Mat ArmorEKF::buildF(double dt) const {
    // 常速模型：x' = x + v*dt
    cv::Mat F = cv::Mat::eye(6, 6, CV_64F);
    F.at<double>(0, 3) = dt;
    F.at<double>(1, 4) = dt;
    F.at<double>(2, 5) = dt;
    return F;
}

// 构建过程噪声协方差Q
cv::Mat ArmorEKF::buildQ(double dt) const {
    // 随机游走加速度模型的过程噪声（离散化后）：
    // 位置块 (dt⁴/4)*q*I，位置-速度块 (dt³/2)*q*I，速度块 dt²*q*I
    const double q11 = dt * dt * dt * dt / 4.0 * q_;
    const double q12 = dt * dt * dt / 2.0 * q_;
    const double q22 = dt * dt * q_;
    cv::Mat Q = cv::Mat::zeros(6, 6, CV_64F);
    for (int i = 0; i < 3; ++i) {
        Q.at<double>(i, i) = q11;
        Q.at<double>(i, i + 3) = q12;
        Q.at<double>(i + 3, i) = q12;
        Q.at<double>(i + 3, i + 3) = q22;
    }
    return Q;
}

// 初始化KF
void ArmorEKF::init(const cv::Mat& z3x1) {
    x_ = cv::Mat::zeros(6, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        x_.at<double>(i) = z3x1.at<double>(i); // 位置用首个测量
    }

    P_ = cv::Mat::eye(6, 6, CV_64F);
    for (int i = 0; i < 3; ++i) {
        P_.at<double>(i, i) = kInitPosVar;
        P_.at<double>(i + 3, i + 3) = kInitVelVar;
    }
    initialized_ = true;
}

// 预测
void ArmorEKF::predict(double dt) {
    if (!initialized_) {
        return;
    }
    const cv::Mat F = buildF(dt);
    const cv::Mat Q = buildQ(dt);
    x_ = F * x_; // 均值：只被模型推
    P_ = F * P_ * F.t() + Q; // 协方差：被 F 夹两下 + 过程噪声（变"更没底"）
}

// 更新 
void ArmorEKF::update(const cv::Mat& z3x1) {
    if (!initialized_) {
        return;
    }
    const cv::Mat y = z3x1 - H_ * x_; // 残差：测量 - 预测测量
    const cv::Mat S = H_ * P_ * H_.t() + R_; // 新息协方差（测量空间总不确定）
    const cv::Mat K = P_ * H_.t() * S.inv(); // 卡尔曼增益（融合权重）
    x_ = x_ + K * y; // 均值修正
    P_ = P_ - K * H_ * P_; // = (I - K*H)*P：不确定变小
}

} // namespace rm_tracker
