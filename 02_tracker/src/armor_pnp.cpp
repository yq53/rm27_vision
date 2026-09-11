#include "armor_pnp/armor_pnp.hpp"

#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>

namespace rm_tracker {

// 计算板角点
std::vector<cv::Point3d> plateObjectPoints(double plate_w_m, double plate_h_m) {
    const double hw = plate_w_m / 2.0;
    const double hh = plate_h_m / 2.0;
    return { { -hw, -hh, 0 }, { hw, -hh, 0 }, { hw, hh, 0 }, { -hw, hh, 0 } };
}

// 计算灯条角点
std::vector<cv::Point3d> barEndObjectPoints(double armor_w_m, double bar_len_m) {
    // 与 plateObjectPoints 同为 TL/TR/BR/BL 顺序，只是高度取灯条长度
    return plateObjectPoints(armor_w_m, bar_len_m);
}

// 取矩形框角点
std::vector<cv::Point2d> rectToCorners(const cv::Rect& rect) {
    const double x0 = rect.x;
    const double y0 = rect.y;
    const double x1 = rect.x + rect.width;
    const double y1 = rect.y + rect.height;
    return { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
}

// 构造内参K
cv::Mat computeK(int width, int height) {
    const double fx = width / (2.0 * std::tan(36.0 * CV_PI / 180.0));
    return (cv::Mat_<double>(3, 3) << fx, 0, width / 2.0, 0, fx, height / 2.0, 0, 0, 1);
}

// 解算装甲板位姿
bool solveArmorPose(
    const std::vector<cv::Point3d>& object,
    const std::vector<cv::Point2d>& image,
    const cv::Mat& K,
    PoseResult& out
) {
    // 误差统计函数
    auto reproj_err = [&](const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2d> proj;
        cv::projectPoints(object, rvec, tvec, K, cv::noArray(), proj);
        double sum = 0.0;
        for (size_t i = 0; i < image.size(); ++i) {
            const cv::Point2d d = proj[i] - image[i];
            sum += std::sqrt(d.x * d.x + d.y * d.y);
        }
        return sum / static_cast<double>(image.size());
    };

    try {
        std::vector<cv::Mat> rvecs, tvecs;
        cv::solvePnPGeneric(
            object, image, K, cv::noArray(), rvecs, tvecs, false, cv::SOLVEPNP_IPPE
        );
        if (rvecs.empty()) {
            return false;
        }

        // 破镜像：优先保留 r3.z>0（板正面朝相机）的候选
        bool any_front = false;
        for (const auto& rv: rvecs) {
            cv::Mat rmat;
            cv::Rodrigues(rv, rmat);
            any_front = any_front || (rmat.at<double>(2, 2) > 0.0);
        }

        // 比较获得最佳外参
        double best = std::numeric_limits<double>::max();
        int best_idx = -1;
        for (size_t k = 0; k < rvecs.size(); ++k) {
            cv::Mat rmat;
            cv::Rodrigues(rvecs[k], rmat);
            if (any_front && rmat.at<double>(2, 2) <= 0.0) {
                continue;
            }
            const double err = reproj_err(rvecs[k], tvecs[k]);
            if (err < best) {
                best = err;
                best_idx = static_cast<int>(k);
            }
        }
        if (best_idx < 0 || best > 10.0) {
            return false;
        }

        // 物理闸门：板在相机前方且距离合理
        const cv::Mat& tv = tvecs[best_idx];
        const double z = tv.at<double>(2);
        const double dist = std::sqrt(
            tv.at<double>(0) * tv.at<double>(0) + tv.at<double>(1) * tv.at<double>(1) + z * z
        );
        if (!(z > 0.2 && dist < 20.0)) {
            return false;
        }

        // 板法线 = R * (0,0,1)（第三列）。平面点 z=0 经 solvePnP 解出的 R 使法线指向场景内侧，
        // 即正对相机时 n≈(0,0,1)，故 yaw = atan2(n.x, n.z)：正对≈0，板向左右偏转则增大。
        cv::Mat rmat;
        cv::Rodrigues(rvecs[best_idx], rmat);
        const double nx = rmat.at<double>(0, 2);
        const double nz = rmat.at<double>(2, 2);

        out.tvec = tv.clone();
        out.reproj_err_px = best;
        out.yaw_deg = std::atan2(nx, nz) * 180.0 / CV_PI;
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

} // namespace rm_tracker
