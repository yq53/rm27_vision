#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"

using rm_vision::Armor;
using rm_vision::ArmorDetector;

namespace fs = std::filesystem;

// 在帧上画出检测框和置信度（演示程序专用，检测器库本身不含绘图）
void drawArmors(cv::Mat& frame, const std::vector<Armor>& armors) {
    for (const Armor& armor: armors) {
        cv::rectangle(frame, armor.rect, cv::Scalar(0, 255, 0), 2);

        const std::string label = cv::format("armour %.2f", armor.confidence);
        cv::putText(
            frame,
            label,
            armor.rect.tl() + cv::Point(0, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2
        );
    }
}

int main(int argc, char** argv) {
    // 路径初始化
    const std::string video_path = (argc > 1) ? argv[1] : "data/demo.avi";
    const std::string model_path = (argc > 2) ? argv[2] : "models/armor_yolov8n.onnx";
    const int max_frames = (argc > 3) ? std::atoi(argv[3]) : 0; // 0 = 处理全部帧

    // 视频捕获
    cv::VideoCapture capture(video_path);
    if (!capture.isOpened()) {
        std::cerr << "[ERROR] Failed to open video: " << video_path << std::endl;
        return -1;
    }

    // ---- 加载检测器（模型路径错误时会抛异常） ----
    ArmorDetector detector(model_path);
    detector.setConfidenceThreshold(0.35f);
    detector.setNmsThreshold(0.45f);
    std::cout << "[INFO] Model loaded: " << model_path << std::endl;

    // ---- 准备输出 ----
    const fs::path result_dir = "results";
    fs::create_directories(result_dir);

    const int frame_width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int frame_height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = capture.get(cv::CAP_PROP_FPS) > 0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;

    const std::string output_path = (result_dir / "detector_demo.avi").string();
    cv::VideoWriter writer(
        output_path,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        cv::Size(frame_width, frame_height)
    );
    if (!writer.isOpened()) {
        std::cerr << "[WARN] Failed to open writer, skip saving video: " << output_path
                  << std::endl;
    }

    // ---- 逐帧检测主循环 ----
    cv::Mat frame;
    int frame_id = 0;
    int frames_with_armor = 0;
    int total_detections = 0;
    const auto start_time = std::chrono::steady_clock::now();

    while (capture.read(frame)) {
        const std::vector<Armor> armors = detector.detect(frame);
        if (!armors.empty()) {
            ++frames_with_armor;
            total_detections += static_cast<int>(armors.size());
        }

        drawArmors(frame, armors);

        // 每隔 120 帧存一张带标注的预览图，方便快速肉眼检查
        if (frame_id % 120 == 0) {
            const std::string preview_path =
                (result_dir / cv::format("preview_%04d.png", frame_id)).string();
            cv::imwrite(preview_path, frame);
        }

        if (writer.isOpened()) {
            writer.write(frame);
        }

        if (frame_id % 60 == 0) {
            std::cout << "[frame " << frame_id << "] " << armors.size() << " armor(s) detected"
                      << std::endl;
        }
        ++frame_id;

        if (max_frames > 0 && frame_id >= max_frames) {
            break;
        }
    }

    const auto end_time = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // ---- 汇总 ----
    std::cout << "========================================" << std::endl;
    std::cout << "[SUMMARY] processed frames : " << frame_id << std::endl;
    std::cout << "[SUMMARY] frames w/ armor   : " << frames_with_armor << " ("
              << cv::format("%.1f", 100.0 * frames_with_armor / std::max(frame_id, 1)) << "%)"
              << std::endl;
    std::cout << "[SUMMARY] total detections  : " << total_detections << std::endl;
    std::cout << "[SUMMARY] avg time per frame: "
              << cv::format("%.1f", elapsed_ms / std::max(frame_id, 1)) << " ms" << std::endl;
    if (writer.isOpened()) {
        std::cout << "[SUMMARY] result video saved: " << output_path << std::endl;
    }
    return 0;
}
