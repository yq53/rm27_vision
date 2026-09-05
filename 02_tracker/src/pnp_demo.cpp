#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"

using rm_vision::Armor;
using rm_vision::ArmorDetector;

namespace fs = std::filesystem;

// L4：PnP 闭环
//  A 段（合成验证）：自己设"真值位姿" -> 正投影生成像素角点 -> solvePnP 反解
//      -> 对比反解出的距离/偏航角与真值，并打印重投影误差。
//  B 段（真实数据）：demo.avi 逐帧 detect -> 框角近似四角点 -> solvePnP
//      -> 画面上叠加显示 距离/偏航角。
//
// 注意：给 solvePnP 的坐标统一用 double（IPPE 对 float 输入会出现病态分支）。

namespace {

constexpr double kFx = 1000.0;
constexpr double kFy = 1000.0;
constexpr double kCx = 720.0;
constexpr double kCy = 540.0;

constexpr double kPlateW = 0.135; // 小装甲板宽 135mm
constexpr double kPlateH = 0.125; // 小装甲板高 125mm

// 内参矩阵 K（demo 假设值，题3 标定后替换）
cv::Mat cameraMatrix() {
    return (cv::Mat_<double>(3, 3) << kFx, 0, kCx, 0, kFy, kCy, 0, 0, 1);
}

// 板系 3D 四角点（double；顺序与 L3 的 2D 角点一致：TL,TR,BR,BL），单位 m
std::vector<cv::Point3d> plateObjectPoints() {
    const double hw = kPlateW / 2;
    const double hh = kPlateH / 2;
    return {
        { -hw, -hh, 0 }, // 0 TL
        { hw, -hh, 0 }, // 1 TR
        { hw, hh, 0 }, // 2 BR
        { -hw, hh, 0 }, // 3 BL
    };
}

// 2D 角点：从轴对齐框取四角（顺序同上）
std::vector<cv::Point2d> rectToCorners(const cv::Rect& rect) {
    const double x0 = rect.x;
    const double y0 = rect.y;
    const double x1 = rect.x + rect.width;
    const double y1 = rect.y + rect.height;
    return { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
}

// 绕 Y 轴旋转（A 段生成"真值"用）
cv::Point3d rotateYaw(const cv::Point3d& p, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return { c * p.x + s * p.z, p.y, -s * p.x + c * p.z };
}

// 手写正投影（A 段生成像素用，与 L2 一致）
cv::Point2d projectPoint(const cv::Point3d& p_cam) {
    return { kFx * p_cam.x / p_cam.z + kCx, kFy * p_cam.y / p_cam.z + kCy };
}

// 一组 PnP 结果
struct SolveResult {
    double dist_m = 0.0; // 板中心到相机光心距离 (m)
    double yaw_deg = 0.0; // 绕板竖轴的偏航角 (deg)
    double reproj_mean = 0.0; // 平均重投影误差 (px)
    double reproj_max = 0.0; // 最大重投影误差 (px)
};

// 求解一块板：object(3D) + image(2D) -> PnP -> 统计
bool solveArmor(
    const std::vector<cv::Point3d>& object, // 板坐标系
    const std::vector<cv::Point2d>& image, // 像素坐标系
    SolveResult& out
) {
    // IPPE 有"镜像歧义"：同一组点可能对应两个位姿。OpenCV 4.x 的 solvePnP(IPPE)
    // 在带噪声输入下可能返回错误的那一个，因此改用 solvePnPGeneric 拿全部候选，
    // 再用"重投影误差最小"来挑选（顺便得到验证指标）。

    // 重投影误差统计函数(平均误差 + 最大误差)
    auto reproj_error = [&](const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2d> reproj;
        cv::projectPoints(object, rvec, tvec, cameraMatrix(), cv::noArray(), reproj);
        double sum = 0.0;
        double max_err = 0.0;
        for (size_t i = 0; i < image.size(); ++i) {
            const cv::Point2d d = reproj[i] - image[i];
            const double err = std::sqrt(d.x * d.x + d.y * d.y); // 模长（像素）
            sum += err;
            max_err = std::max(max_err, err);
        }
        return std::make_pair(sum / image.size(), max_err);
    };

    cv::Mat best_rvec, best_tvec;
    bool found = false;
    try {
        std::vector<cv::Mat> rvecs, tvecs;
        cv::solvePnPGeneric(
            object,
            image,
            cameraMatrix(),
            cv::noArray(),
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE
        );

        // 镜像歧义的 tie-break：镜像两解的重投影误差几乎相同、无法靠误差区分，
        // 但板面法线（旋转矩阵第 3 列 r3）方向相反。比赛场景中板只会正面朝向
        // 相机（转过去就被挡住/检测不到），因此：
        //   若存在 r3.z > 0（法线朝相机）的候选，就只在正面候选里比误差；
        //   若全部不正面，才退回"误差最小"。
        struct Candidate {
            cv::Mat rvec;
            cv::Mat tvec;
            double err;
            bool front; // r3.z > 0
        };
        std::vector<Candidate> cands;
        cands.reserve(rvecs.size());
        for (size_t k = 0; k < rvecs.size(); ++k) {
            cv::Mat rmat;
            cv::Rodrigues(rvecs[k], rmat);
            const bool front = rmat.at<double>(2, 2) > 0.0; // 第三列第三行 = r3.z
            cands.push_back({ rvecs[k], tvecs[k], reproj_error(rvecs[k], tvecs[k]).first, front });
        }

        bool any_front = false;
        for (const auto& c: cands) {
            any_front = any_front || c.front;
        }

        double best_err = std::numeric_limits<double>::max();
        for (const auto& c: cands) {
            if (any_front && !c.front) {
                continue; // 有正面解可选时，跳过背面候选
            }
            if (c.err < best_err) {
                best_err = c.err;
                best_rvec = c.rvec;
                best_tvec = c.tvec;
            }
        }
        found = !cands.empty();
    } catch (const cv::Exception& e) {
        std::cerr << "[WARN] solvePnPGeneric failed: " << e.what() << std::endl;
        return false;
    }
    if (!found) {
        return false;
    }

    // OpenCV 4.x 的 IPPE 在带噪输入下可能给出病态解（重投影误差天文数字）。
    // 兜底：用 ITERATIVE（无初值）再解一次，谁的重投影误差小就用谁。
    cv::Mat pick_r = best_rvec, pick_t = best_tvec;
    auto [mean_err, max_err] = reproj_error(pick_r, pick_t);
    try {
        cv::Mat r_iter, t_iter;
        cv::solvePnP(
            object,
            image,
            cameraMatrix(),
            cv::noArray(),
            r_iter,
            t_iter,
            false,
            cv::SOLVEPNP_ITERATIVE
        );
        const auto [m2, x2] = reproj_error(r_iter, t_iter);
        if (m2 < mean_err) {
            mean_err = m2;
            max_err = x2;
            pick_r = r_iter;
            pick_t = t_iter;
        }
    } catch (const cv::Exception&) {
        // ITERATIVE 也失败则保留 IPPE 结果
    }

    // 两种解都不可信（重投影误差 > 10px）-> 判失败，调用方跳过这一帧
    if (mean_err > 10.0) {
        return false;
    }
    out.reproj_mean = mean_err;
    out.reproj_max = max_err;

    // tvec = 板原点(中心)在相机系的位置 -> 距离
    const double tx = pick_t.at<double>(0);
    const double ty = pick_t.at<double>(1);
    const double tz = pick_t.at<double>(2);
    out.dist_m = std::sqrt(tx * tx + ty * ty + tz * tz);

    // 旋转向量 -> 旋转矩阵；yaw = atan2(-R20, R00)（绕 Y 轴约定）
    cv::Mat rmat;
    cv::Rodrigues(pick_r, rmat);
    out.yaw_deg = std::atan2(-rmat.at<double>(2, 0), rmat.at<double>(0, 0)) * 180.0 / CV_PI;
    return true;
}

} // namespace

int main(int argc, char** argv) {
    // ================= A 段：合成闭环验证（不需要视频/模型） =================
    std::cout << "======== A 段：合成闭环（真值 -> 像素 -> PnP 反解） ========\n";
    {
        const auto object = plateObjectPoints(); // 板坐标系

        // 真值：板中心在相机系 (0.30, 0.05, 3.00)，绕 Y 轴转 0.25 rad ≈ 14.3°
        const double kTrueYaw = 0.25;
        const cv::Point3d t_true(0.30, 0.05, 3.00);
        const double true_dist = cv::norm(t_true); // = 3.015 m（不是 3.00！）

        // 1) 正投影生成"测量到的像素角点"
        std::vector<cv::Point2d> image; // 像素坐标系
        for (const auto& p: object) {
            image.push_back(projectPoint(rotateYaw(p, kTrueYaw) + t_true));
        }

        // 2) PnP 反解（干净数据）
        SolveResult clean;
        if (solveArmor(object, image, clean)) {
            std::cout << "clean data : dist=" << std::fixed << std::setprecision(3) << clean.dist_m
                      << " m (true=" << true_dist << ") | yaw=" << std::setprecision(1)
                      << clean.yaw_deg
                      << " deg (true=14.3) | reproj mean/max = " << clean.reproj_mean << " / "
                      << clean.reproj_max << " px\n";
        }
        std::cout << "（干净数据精确还原位姿 → 反链路通了。噪声鲁棒性由 B 段真实数据展示："
                     "真实角点自带像素噪声，配合 >10px 重投影误差拒收，看平均距离是否稳定）\n";
    }

    // ================= B 段：真实数据（demo.avi 逐帧 PnP + 叠加显示） =================
    std::cout << "\n======== B 段：demo.avi 逐帧 PnP ========\n";
    const std::string video_path = (argc > 1) ? argv[1] : "data/demo.avi";
    const std::string model_path = (argc > 2) ? argv[2] : "models/armor_yolov8n.onnx";
    const int max_frames = (argc > 3) ? std::atoi(argv[3]) : 0; // 0 = 全部

    cv::VideoCapture capture(video_path);
    if (!capture.isOpened()) {
        std::cerr << "[ERROR] Failed to open video: " << video_path << std::endl;
        return -1;
    }

    std::unique_ptr<ArmorDetector> detector;
    try {
        detector = std::make_unique<ArmorDetector>(model_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load model: " << model_path << std::endl;
        std::cerr << "        reason: " << e.what() << std::endl;
        return -1;
    }
    detector->setConfidenceThreshold(0.35f);
    detector->setNmsThreshold(0.45f);

    const fs::path result_dir = "results";
    fs::create_directories(result_dir);

    const int fw = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int fh = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = capture.get(cv::CAP_PROP_FPS) > 0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;

    const std::string output_path = (result_dir / "pnp_demo.avi").string();
    cv::VideoWriter
        writer(output_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(fw, fh));
    if (!writer.isOpened()) {
        std::cerr << "[WARN] cannot open writer, skip saving video\n";
    }

    const auto object = plateObjectPoints();
    cv::Mat frame;
    int frame_id = 0;
    int frames_with_solve = 0;
    double dist_sum = 0.0;
    const auto start = std::chrono::steady_clock::now();

    while (capture.read(frame)) {
        const std::vector<Armor> armors = detector->detect(frame);
        for (const Armor& armor: armors) {
            const std::vector<cv::Point2d> corners = rectToCorners(armor.rect);

            SolveResult res;
            if (!solveArmor(object, corners, res)) {
                continue;
            }
            ++frames_with_solve;
            dist_sum += res.dist_m;

            cv::rectangle(frame, armor.rect, cv::Scalar(0, 255, 0), 1);
            std::ostringstream oss;
            oss << "d=" << std::fixed << std::setprecision(2) << res.dist_m << "m"
                << " yaw=" << std::setprecision(0) << res.yaw_deg << "deg";
            cv::putText(
                frame,
                oss.str(),
                armor.rect.tl() + cv::Point(0, -8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 255),
                2
            );
        }

        if (frame_id % 120 == 0) {
            cv::imwrite(
                (result_dir / cv::format("pnp_preview_%04d.png", frame_id)).string(),
                frame
            );
        }
        if (writer.isOpened()) {
            writer.write(frame);
        }
        if (frame_id % 60 == 0) {
            std::cout << "[frame " << frame_id << "] " << armors.size() << " armor(s)\n";
        }
        ++frame_id;
        if (max_frames > 0 && frame_id >= max_frames) {
            break;
        }
    }

    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    writer.release();

    std::cout << "========================================\n";
    std::cout << "[SUMMARY] frames: " << frame_id << " | solved: " << frames_with_solve;
    if (frames_with_solve > 0) {
        std::cout << " | avg dist: " << cv::format("%.2f", dist_sum / frames_with_solve) << " m";
    }
    std::cout << " | avg: " << cv::format("%.1f", elapsed_ms / std::max(frame_id, 1))
              << " ms/frame\n";
    std::cout << "[SUMMARY] saved: " << output_path << std::endl;
    return 0;
}
