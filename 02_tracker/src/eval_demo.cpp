// eval_demo：离线评测当前链路（detector -> 角点 -> PnP -> EKF）
//
// 用法：eval_demo [视频] [模型] [最大帧数(0=全部)] [tag] [corner_mode: bbox|refine] [detector: bbox|pose]
// 输出：results/eval_<tag>.csv（逐帧）+ results/eval_<tag>_summary.txt（汇总）
//
// 两种检测器：
//   detector=bbox  自训 bbox 模型（ArmorDetector）：角点 = bbox 四角（或用 armor_corner 精修）
//   detector=pose  四关键点模型（ArmorPoseDetector）：角点 = 灯条端点，物体模型用 barEndObjectPoints
//
// 指标定义：
//   frames_with_armor 检出率 = 至少检出 1 块板的帧 / 总帧
//   pnp_ok 率               = PnP 解算通过的帧 / 总帧
//   reproj_px              角点平均重投影误差
//   dist_raw / dist_ekf    板心距离（原始 PnP / EKF 滤波后）
//   jitter                 相邻帧距离跳动的平均绝对值 |d_i - d_{i-1}|（只统计相邻两帧都有效的）

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "armor_corner/armor_corner.hpp"
#include "armor_detector/armor_detector.hpp"
#include "armor_ekf/armor_ekf.hpp"
#include "armor_pnp/armor_pnp.hpp"
#include "armor_pose_detector/armor_pose_detector.hpp"

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

bool valid(double v) {
    return !std::isnan(v);
}

// 相邻帧跳动：只统计"当前帧与上一帧都有效且帧号相邻"的差分
struct JitterStat {
    double mean_abs = kNan;
    double std = kNan;
    int count = 0;
};

// 计算抖动
JitterStat jitterOf(const std::vector<double>& series) {
    // 计算差值
    std::vector<double> diffs;
    int last_idx = -1;
    double last_val = 0.0;
    for (size_t i = 0; i < series.size(); ++i) {
        if (!valid(series[i])) {
            continue;
        }
        // 保证前后两帧都有图像
        if (last_idx >= 0 && static_cast<int>(i) == last_idx + 1) {
            diffs.push_back(std::fabs(series[i] - last_val));
        }
        last_idx = static_cast<int>(i);
        last_val = series[i];
    }
    
    // 计算统计量
    JitterStat st;
    if (diffs.empty()) {
        return st;
    }
    const double mean = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size(); // 均值
    double var = 0.0;
    for (const double d: diffs) {
        var += (d - mean) * (d - mean); // 方差
    }
    var /= diffs.size();
    st.mean_abs = mean;
    st.std = std::sqrt(var);
    st.count = static_cast<int>(diffs.size());
    return st;
}

// 计算均值
double meanOf(const std::vector<double>& series) {
    double sum = 0.0;
    int n = 0;
    for (const double v: series) {
        if (valid(v)) {
            sum += v;
            ++n;
        }
    }
    return n > 0 ? sum / n : kNan;
}

// 计算中值
double medianOf(std::vector<double> series) {
    series.erase(std::remove_if(series.begin(), series.end(), [](double v) { return !valid(v); }),
                 series.end());
    if (series.empty()) {
        return kNan;
    }
    std::sort(series.begin(), series.end());
    const size_t n = series.size();
    return (n % 2 == 1) ? series[n / 2] : 0.5 * (series[n / 2 - 1] + series[n / 2]);
}

std::vector<cv::Point2d> bboxCorners(const cv::Rect& rect) {
    return { { static_cast<double>(rect.x), static_cast<double>(rect.y) },
             { static_cast<double>(rect.x + rect.width), static_cast<double>(rect.y) },
             { static_cast<double>(rect.x + rect.width), static_cast<double>(rect.y + rect.height) },
             { static_cast<double>(rect.x), static_cast<double>(rect.y + rect.height) } };
}

} // namespace

int main(int argc, char** argv) {
    const std::string video_path = (argc > 1) ? argv[1] : "data/demo.avi";
    const std::string model_path = (argc > 2) ? argv[2] : "models/armor_yolov8n.onnx";
    const int max_frames = (argc > 3) ? std::atoi(argv[3]) : 0;
    const std::string tag = (argc > 4) ? argv[4] : "baseline";
    const std::string corner_mode = (argc > 5) ? argv[5] : "bbox";  // bbox | refine
    const std::string detector_mode = (argc > 6) ? argv[6] : "bbox"; // bbox | pose

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 打不开视频: " << video_path << std::endl;
        return 1;
    }
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) {
        fps = 30.0;
    }
    const double dt = 1.0 / fps;

    std::unique_ptr<rm_vision::ArmorDetector> bbox_detector;
    std::unique_ptr<rm_vision::ArmorPoseDetector> pose_detector;
    if (detector_mode == "pose") {
        pose_detector = std::make_unique<rm_vision::ArmorPoseDetector>(model_path);
        pose_detector->setConfidenceThreshold(0.5f);
        pose_detector->setNmsThreshold(0.45f);
    } else {
        bbox_detector = std::make_unique<rm_vision::ArmorDetector>(model_path);
        bbox_detector->setConfidenceThreshold(0.35f);
        bbox_detector->setNmsThreshold(0.45f);
    }
    rm_tracker::ArmorEKF ekf;

    std::vector<double> dist_raw, dist_ekf, reproj, yaw, detect_ms, pnp_ms;
    std::vector<int> n_armor;

    // 计数器
    int frames = 0;             // 总处理帧数
    int frames_with_armor = 0;  // 检出计数
    int pnp_ok_count = 0;       // PnP通过计数
    int refined_count = 0;      // bbox 模式下精修成功 / pose 模式下使用灯条端点的帧数
    long bars_sum = 0;          // 参与拼角的灯条计数

    // 创建文件输出流
    std::filesystem::create_directories("results");
    std::ofstream csv("results/eval_" + tag + ".csv");
    csv << "frame,n_armor,target_area,pnp_ok,refined,reproj_px,dist_raw,yaw_deg,dist_ekf,"
           "detect_ms,pnp_ms\n";

    // 主循环
    cv::Mat frame;
    while (cap.read(frame)) {
        if (max_frames > 0 && frames >= max_frames) {
            break;
        }

        std::vector<cv::Point2d> corners;
        cv::Rect target_rect;
        int target_area = 0;
        int target_count = 0; // 本帧检出目标数（bbox: 检测框数 / pose: 关键点目标数）
        bool corner_refined = false;

        // === 获取检测目标 ===
        const auto t0 = std::chrono::steady_clock::now();
        // pose模型
        if (detector_mode == "pose") {
            const auto dets = pose_detector->detect(frame);
            target_count = static_cast<int>(dets.size());   // 检测到几个目标

            // 最大面积筛选
            const rm_vision::ArmorPose* target = nullptr;   
            int best_area = 0;
            for (const auto& d: dets) {
                if (d.rect.area() > best_area) {
                    best_area = d.rect.area();
                    target = &d;
                }
            }
            if (target != nullptr) {
                target_rect = target->rect;
                target_area = best_area;
                corners.assign(target->kpts.begin(), target->kpts.end());
                corner_refined = true; // 直接用灯条端点，无需 bbox 近似
                ++refined_count;
            }
        } else {    // bbox模型
            const auto dets = bbox_detector->detect(frame);
            target_count = static_cast<int>(dets.size());

            // 最大面积筛选
            const rm_vision::Armor* target = nullptr;
            double best_area = 0.0;
            for (const auto& d: dets) {
                if (d.rect.area() > best_area) {
                    best_area = d.rect.area();
                    target = &d;
                }
            }
            if (target != nullptr) {
                target_rect = target->rect;
                target_area = static_cast<int>(best_area);
                if (corner_mode == "refine") {
                    const auto cr = rm_tracker::refineArmorCorners(frame, target->rect);
                    corners = cr.corners;
                    corner_refined = cr.refined;
                    bars_sum += cr.bars_found;
                    if (cr.refined) {
                        ++refined_count;
                    }
                } else {
                    corners = bboxCorners(target->rect);
                }
            }
        }
        const auto t1 = std::chrono::steady_clock::now();

        rm_tracker::PoseResult pose;
        bool ok = false;
        if (!corners.empty()) {
            // 获取ROI角点
            const std::vector<cv::Point3d> object =
                (detector_mode == "pose") ? rm_tracker::barEndObjectPoints()
                                          : rm_tracker::plateObjectPoints();
            const cv::Mat K = rm_tracker::computeK(frame.cols, frame.rows); // 内参
            ok = rm_tracker::solveArmorPose(object, corners, K, pose);  // PnP求解pose
        }
        const auto t2 = std::chrono::steady_clock::now();

        // === ekf预测更新 ===
        ekf.predict(dt);
        double d_raw = kNan;
        double yaw_deg = kNan;
        double err_px = kNan;
        if (ok) {
            const cv::Mat& p = pose.tvec;   // PnP平移向量
            d_raw = std::sqrt(
                p.at<double>(0) * p.at<double>(0) + p.at<double>(1) * p.at<double>(1)
                + p.at<double>(2) * p.at<double>(2)
            );  // PnP距离
            yaw_deg = pose.yaw_deg;         // PnP yaw
            err_px = pose.reproj_err_px;    // 平均误差
            if (!ekf.initialized()) {
                ekf.init(pose.tvec);
            } else {
                ekf.update(pose.tvec);
            }
            ++pnp_ok_count;
        }

        double d_ekf = kNan;
        if (ekf.initialized()) {
            const cv::Mat& x = ekf.state();
            d_ekf = std::sqrt(
                x.at<double>(0) * x.at<double>(0) + x.at<double>(1) * x.at<double>(1)
                + x.at<double>(2) * x.at<double>(2)
            );
        }

        const double d_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double p_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

        n_armor.push_back(target_count);
        if (!corners.empty()) {
            ++frames_with_armor;
        }
        dist_raw.push_back(d_raw);
        dist_ekf.push_back(d_ekf);
        reproj.push_back(err_px);
        yaw.push_back(yaw_deg);
        detect_ms.push_back(d_ms);
        pnp_ms.push_back(p_ms);

        // 写入
        csv << frames << ',' << target_count << ',' << target_area << ','
            << (ok ? 1 : 0) << ',' << (corner_refined ? 1 : 0) << ','
            << (valid(err_px) ? std::to_string(err_px) : "") << ','
            << (valid(d_raw) ? std::to_string(d_raw) : "") << ','
            << (valid(yaw_deg) ? std::to_string(yaw_deg) : "") << ','
            << (valid(d_ekf) ? std::to_string(d_ekf) : "") << ',' << d_ms << ',' << p_ms << '\n';
        ++frames;
    }

    const JitterStat j_raw = jitterOf(dist_raw);
    const JitterStat j_ekf = jitterOf(dist_ekf);

    // 将数据推入文字缓冲区
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(4);   // 数字保留小数点后4位
    out << "=== eval tag=" << tag << " ===\n";
    out << "video=" << video_path << "  model=" << model_path << "  fps=" << fps
        << "  detector=" << detector_mode << "  corner_mode=" << corner_mode << "\n";
    out << "frames=" << frames << "\n";
    out << "frames_with_armor=" << frames_with_armor << "  rate="
        << (frames ? 100.0 * frames_with_armor / frames : 0.0) << "%\n";
    out << "pnp_ok=" << pnp_ok_count << "  rate=" << (frames ? 100.0 * pnp_ok_count / frames : 0.0)
        << "%\n";
    out << "refined_frames=" << refined_count << "  rate="
        << (frames_with_armor ? 100.0 * refined_count / frames_with_armor : 0.0) << "%(占检出)";
    if (detector_mode == "bbox" && corner_mode == "refine") {
        out << "  bars_mean="
            << (frames_with_armor ? static_cast<double>(bars_sum) / frames_with_armor : 0.0);
    } else if (detector_mode == "bbox") {
        out << "  bars_mean=NA(未启用精修, 未统计灯条)";
    } else {
        out << "  bars_mean=NA(pose: 端点由模型直接给出, 不统计灯条数)";
    }
    out << "\n";
    out << "reproj_px mean=" << meanOf(reproj) << "  median=" << medianOf(reproj) << "\n";
    out << "dist_raw mean=" << meanOf(dist_raw) << "  dist_ekf mean=" << meanOf(dist_ekf) << "\n";
    out << "jitter_raw mean_abs=" << j_raw.mean_abs << " std=" << j_raw.std << " n=" << j_raw.count
        << "\n";
    out << "jitter_ekf mean_abs=" << j_ekf.mean_abs << " std=" << j_ekf.std << " n=" << j_ekf.count
        << "\n";
    out << "yaw_deg mean_abs=";
    {
        std::vector<double> abs_yaw;
        for (const double v: yaw) {
            if (valid(v)) {
                abs_yaw.push_back(std::fabs(v));
            }
        }
        out << meanOf(abs_yaw);
    }
    out << "\n";
    out << "time detect_ms=" << meanOf(detect_ms) << "  pnp_ms=" << meanOf(pnp_ms) << "\n";

    // 从缓冲区中取值
    std::cout << out.str(); // 流向屏幕
    std::ofstream summary("results/eval_" + tag + "_summary.txt");
    summary << out.str();   // 存档进_summary.txt
    std::cout << "已写出 results/eval_" << tag << ".csv 与 _summary.txt" << std::endl;
    return 0;
}
