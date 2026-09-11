// [教学] L8：灯条精定位演示（负结果的演示部分）—— 删除不影响主节点
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"
#include "armor_tracker/light_bar_detector.hpp"

using rm_tracker::LightBar;
using rm_tracker::LightBarDetector;
using rm_vision::Armor;
using rm_vision::ArmorDetector;

namespace fs = std::filesystem;

// L8 演示：对每个 YOLO 框做"框内灯条精定位 + 配对"
// 可视化：绿框=YOLO 框；红短线=检测到的灯条长边；黄四角=配对成功的装甲四角点。
// 统计：框总数、配对成功数 —— 直观看出"结构校验"拦掉了多少单灯条误检。

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
        return -1;
    }
    detector->setConfidenceThreshold(0.35f);
    detector->setNmsThreshold(0.45f);

    LightBarDetector light_bar_detector;

    const fs::path result_dir = "results";
    fs::create_directories(result_dir);
    const int fw = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int fh = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = capture.get(cv::CAP_PROP_FPS) > 0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;

    const std::string output_path = (result_dir / "lightbar_demo.avi").string();
    cv::VideoWriter
        writer(output_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(fw, fh));

    cv::Mat frame;
    int frame_id = 0;
    int total_boxes = 0;
    int paired_boxes = 0;

    const auto start = std::chrono::steady_clock::now();
    while (capture.read(frame)) {
        const std::vector<Armor> armors = detector->detect(frame);

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        for (const Armor& armor: armors) {
            ++total_boxes;
            cv::rectangle(frame, armor.rect, cv::Scalar(0, 255, 0), 1); // YOLO 框

            // 框外扩一点点，避免灯条恰好压在框边上被裁掉
            cv::Rect roi = armor.rect;
            const int pad = std::max(4, roi.width / 10);
            roi.x = std::max(0, roi.x - pad);
            roi.y = std::max(0, roi.y - pad);
            roi.width = std::min(fw - roi.x, roi.width + 2 * pad);
            roi.height = std::min(fh - roi.y, roi.height + 2 * pad);
            if (roi.width <= 0 || roi.height <= 0) {
                continue;
            }

            const cv::Mat roi_gray = gray(roi);
            const std::vector<LightBar> bars = light_bar_detector.detectBars(roi_gray);

            // 画每根灯条的长边（红）
            for (const LightBar& bar: bars) {
                cv::line(
                    frame,
                    bar.top + cv::Point2f(roi.x, roi.y),
                    bar.bottom + cv::Point2f(roi.x, roi.y),
                    cv::Scalar(0, 0, 255),
                    2
                );
            }

            // 配对：按 x 顺序取相邻两根尝试
            for (size_t i = 0; i + 1 < bars.size(); ++i) {
                std::vector<cv::Point2f> corners;
                if (!light_bar_detector.pairArmor(bars[i], bars[i + 1], corners)) {
                    continue;
                }
                ++paired_boxes;
                std::vector<cv::Point> poly;
                for (const auto& c: corners) {
                    poly.push_back(c + cv::Point2f(roi.x, roi.y));
                }
                for (int k = 0; k < 4; ++k) {
                    cv::line(frame, poly[k], poly[(k + 1) % 4], cv::Scalar(0, 220, 255), 2);
                }
                break; // v1：每框只取最佳一对
            }
        }

        if (writer.isOpened()) {
            writer.write(frame);
        }
        if (frame_id % 60 == 0) {
            std::cout << "[frame " << frame_id << "] boxes=" << armors.size() << std::endl;
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
    std::cout << "[SUMMARY] frames: " << frame_id << " | total boxes: " << total_boxes
              << " | paired(结构校验通过): " << paired_boxes << " ("
              << (total_boxes ? cv::format("%.0f", 100.0 * paired_boxes / total_boxes) : "0")
              << "%)\n";
    std::cout << "[SUMMARY] avg: " << cv::format("%.1f", elapsed_ms / std::max(frame_id, 1))
              << " ms/frame | saved: " << output_path << std::endl;
    return 0;
}
