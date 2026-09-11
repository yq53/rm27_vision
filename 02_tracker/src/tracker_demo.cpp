// [教学] L7：PnP → EKF 平滑对比演示（原始 vs 滤波）—— 删除不影响主节点
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"
#include "armor_ekf/armor_ekf.hpp"

using rm_tracker::ArmorEKF;
using rm_vision::Armor;
using rm_vision::ArmorDetector;

namespace fs = std::filesystem;

// L7：tracker_demo —— 把 PnP 位姿喂给 ArmorEKF，量化对比"滤波前/后"
// 数据流：demo.avi → detect → 角点 → PnP(z=板心位置) → EKF(平滑+速度) → 对比输出
//
// 简化约定（v1 单目标）：一帧有多个检测时，只跟踪"面积最大"的那块板。

namespace {

constexpr double kFx = 1000.0;
constexpr double kFy = 1000.0;
constexpr double kCx = 720.0;
constexpr double kCy = 540.0;
constexpr double kPlateW = 0.135; // 小装甲板宽
constexpr double kPlateH = 0.125; // 小装甲板高

// 内参
cv::Mat cameraMatrix() {
    return (cv::Mat_<double>(3, 3) << kFx, 0, kCx, 0, kFy, kCy, 0, 0, 1);
}

// 板坐标系
std::vector<cv::Point3d> plateObjectPoints() {
    const double hw = kPlateW / 2;
    const double hh = kPlateH / 2;
    return { { -hw, -hh, 0 }, { hw, -hh, 0 }, { hw, hh, 0 }, { -hw, hh, 0 } }; // TL TR BR BL
}

// 获取四角点
std::vector<cv::Point2d> rectToCorners(const cv::Rect& rect) {
    const double x0 = rect.x;
    const double y0 = rect.y;
    const double x1 = rect.x + rect.width;
    const double y1 = rect.y + rect.height;
    return { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
}

// 单块板 PnP 求板心位置 z = (x,y,z)ᵀ（3x1）。
// 复用 pnp_demo 打磨过的稳健路径：Generic 全候选 + 正面约束 + 最小重投影误差。
bool solveArmorPosition(
    const std::vector<cv::Point3d>& object,
    const std::vector<cv::Point2d>& image,
    cv::Mat& z3x1
) {
    auto reproj_error = [&](const cv::Mat& rvec, const cv::Mat& tvec) {
        std::vector<cv::Point2d> reproj;
        cv::projectPoints(object, rvec, tvec, cameraMatrix(), cv::noArray(), reproj);
        double sum = 0.0;
        for (size_t i = 0; i < image.size(); ++i) {
            const cv::Point2d d = reproj[i] - image[i];
            sum += std::sqrt(d.x * d.x + d.y * d.y);
        }
        return sum / image.size();
    };

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
        if (rvecs.empty()) {
            return false;
        }
        // 挑"重投影误差最小"；若存在正面解(r3.z>0)则只在正面解里挑
        bool any_front = false;
        for (const auto& rv: rvecs) {
            cv::Mat rmat;
            cv::Rodrigues(rv, rmat);
            any_front = any_front || (rmat.at<double>(2, 2) > 0.0);
        }
        double best = std::numeric_limits<double>::max();
        int best_idx = -1;
        for (size_t k = 0; k < rvecs.size(); ++k) {
            cv::Mat rmat;
            cv::Rodrigues(rvecs[k], rmat);
            if (any_front && rmat.at<double>(2, 2) <= 0.0) {
                continue;
            }
            const double err = reproj_error(rvecs[k], tvecs[k]);
            if (err < best) {
                best = err;
                best_idx = static_cast<int>(k);
            }
        }
        if (best_idx < 0 || best > 10.0) {
            return false; // 误差闸门：解不可信
        }
        z3x1 = tvecs[best_idx].clone();
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}

// 计算一段序列相邻帧距离差的平均幅度（"抖动"的量化指标，越小越平）
double meanAdjacentDelta(const std::vector<double>& v) {
    if (v.size() < 2) {
        return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 1; i < v.size(); ++i) {
        sum += std::abs(v[i] - v[i - 1]);
    }
    return sum / (v.size() - 1);
}

double norm3(const cv::Mat& m) {
    return std::sqrt(
        m.at<double>(0) * m.at<double>(0) + m.at<double>(1) * m.at<double>(1)
        + m.at<double>(2) * m.at<double>(2)
    );
}

} // namespace

int main(int argc, char** argv) {
    const std::string video_path = (argc > 1) ? argv[1] : "data/demo.avi";
    const std::string model_path = (argc > 2) ? argv[2] : "models/armor_yolov8n.onnx";
    const int max_frames = (argc > 3) ? std::atoi(argv[3]) : 0;

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
    const double dt = 1.0 / fps;

    const std::string output_path = (result_dir / "tracker_demo.avi").string();
    cv::VideoWriter
        writer(output_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(fw, fh));
    if (!writer.isOpened()) {
        std::cerr << "[WARN] cannot open writer, skip saving video\n";
    }

    const auto object = plateObjectPoints();
    ArmorEKF ekf;

    cv::Mat frame;
    int frame_id = 0;
    int frames_detected = 0;
    int frames_predicted_only = 0;
    std::vector<double> raw_dists; // 每帧 PnP 原始距离（有检测才记）
    std::vector<double> filt_dists; // 滤波后距离（初始化后每帧记）

    const auto start = std::chrono::steady_clock::now();
    while (capture.read(frame)) {
        const std::vector<Armor> armors = detector->detect(frame);

        // 每帧先预测（无论有无检测）——漏检帧 = 只预测不更新
        ekf.predict(dt);

        // 挑面积最大的装甲作为跟踪目标（v1 单目标简化）
        double max_area = 0.0;
        const Armor* target = nullptr;
        for (const Armor& a: armors) {
            const double area = a.rect.area();
            if (area > max_area) {
                max_area = area;
                target = &a;
            }
        }

        if (target != nullptr) {
            ++frames_detected;
            const std::vector<cv::Point2d> corners = rectToCorners(target->rect);
            cv::Mat z3x1;
            if (solveArmorPosition(object, corners, z3x1)) {
                if (!ekf.initialized()) {
                    ekf.init(z3x1); // 首帧初始化
                } else {
                    ekf.update(z3x1);
                }
                raw_dists.push_back(norm3(z3x1));
                // 画面：画框 + 原始距离
                cv::rectangle(frame, target->rect, cv::Scalar(0, 255, 0), 2);
                cv::putText(
                    frame,
                    cv::format("raw d=%.2fm", norm3(z3x1)),
                    target->rect.tl() + cv::Point(0, -10),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(0, 255, 0),
                    2
                );
            }
        } else {
            ++frames_predicted_only; // 漏检：EKF 已 predict，无 update
        }

        if (ekf.initialized()) {
            filt_dists.push_back(norm3(ekf.state()));
            // 画面：滤波距离（黄） + 速度（画在板中心附近）
            const cv::Point org(fw / 2 - 220, fh - 60);
            cv::putText(
                frame,
                cv::format(
                    "EKF d=%.2fm  v=(%.1f,%.1f,%.1f)m/s",
                    norm3(ekf.state()),
                    ekf.state().at<double>(3),
                    ekf.state().at<double>(4),
                    ekf.state().at<double>(5)
                ),
                org,
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 220, 255),
                2
            );
        }

        if (frame_id % 60 == 0) {
            std::cout << "[frame " << frame_id << "] det=" << (target != nullptr)
                      << " raw=" << (raw_dists.empty() ? -1.0 : raw_dists.back())
                      << " filt=" << (filt_dists.empty() ? -1.0 : filt_dists.back()) << std::endl;
        }
        if (writer.isOpened()) {
            writer.write(frame);
        }
        ++frame_id;
        if (max_frames > 0 && frame_id >= max_frames) {
            break;
        }
    }
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    writer.release();

    // ---- 量化评估 ----
    std::cout << "========================================\n";
    std::cout << "[SUMMARY] frames: " << frame_id << " | detected: " << frames_detected
              << " | predict-only(漏检): " << frames_predicted_only << std::endl;
    std::cout << "[SUMMARY] 距离相邻帧平均跳动  raw: "
              << cv::format("%.3f", meanAdjacentDelta(raw_dists))
              << " m  vs  EKF: " << cv::format("%.3f", meanAdjacentDelta(filt_dists)) << " m"
              << std::endl;
    std::cout << "[SUMMARY] 平均距离             raw: "
              << cv::format(
                     "%.3f",
                     std::accumulate(raw_dists.begin(), raw_dists.end(), 0.0)
                         / std::max<int>(1, (int)raw_dists.size())
                 )
              << "  EKF: "
              << cv::format(
                     "%.3f",
                     std::accumulate(filt_dists.begin(), filt_dists.end(), 0.0)
                         / std::max<int>(1, (int)filt_dists.size())
                 )
              << std::endl;
    std::cout << "[SUMMARY] avg time: " << cv::format("%.1f", elapsed_ms / std::max(frame_id, 1))
              << " ms/frame | saved: " << output_path << std::endl;
    return 0;
}
