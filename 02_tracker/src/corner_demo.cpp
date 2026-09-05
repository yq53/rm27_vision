#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "armor_detector/armor_detector.hpp"

using rm_vision::Armor;
using rm_vision::ArmorDetector;

namespace fs = std::filesystem;

// L3：让 YOLO 框长出"四角点"（v1：框角近似）
//
// 为什么需要四角点？L4 的 PnP 需要"3D 点 <-> 2D 像素点"的对应关系，
// 而题1 的 YOLO 只给了轴对齐外接框（rect）。v1 先近似：
//   板的四角 ≈ 框的四个角（板子接近正对相机时误差小）。
//
// 四角点顺序约定（重要，L4 的 3D 点必须按同一顺序给 solvePnP）：
//   0 左上 TL -> 1 右上 TR -> 2 右下 BR -> 3 左下 BL（顺时针）。
// 后续板坐标系建模为 X 右、Y 下、Z 前（与相机坐标系同向），
// 正对相机时旋转近似单位阵，方便验证。

namespace {

// 从轴对齐框取出 4 个角点（顺序 TL, TR, BR, BL）
std::vector<cv::Point2f> rectToCorners(const cv::Rect& rect) {
    const float x0 = static_cast<float>(rect.x);
    const float y0 = static_cast<float>(rect.y);
    const float x1 = static_cast<float>(rect.x + rect.width);
    const float y1 = static_cast<float>(rect.y + rect.height);
    return { { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } };
}

// 把 4 个角点按编号画出来，方便肉眼看顺序
void drawCorners(cv::Mat& frame, const std::vector<cv::Point2f>& corners) {
    // 顺序对应的颜色：TL 红、TR 绿、BR 蓝、BL 黄
    const cv::Scalar kColors[4] = {
        { 0, 0, 255 }, // 0 TL red
        { 0, 255, 0 }, // 1 TR green
        { 255, 0, 0 }, // 2 BR blue
        { 0, 255, 255 }, // 3 BL yellow
    };
    for (int i = 0; i < 4; ++i) {
        cv::circle(frame, corners[i], 5, kColors[i], -1);
        cv::putText(
            frame,
            std::to_string(i),
            corners[i] + cv::Point2f(7, -7),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            kColors[i],
            2
        );
    }
}

} // namespace

int main(int argc, char** argv) {
    // 路径初始化
    const std::string video_path = (argc > 1) ? argv[1] : "data/demo.avi";
    const std::string model_path = (argc > 2) ? argv[2] : "models/armor_yolov8n.onnx";
    const int max_frames = (argc > 3) ? std::atoi(argv[3]) : 0; // 0 = 全部(atoi格式)

    // 视频捕获
    cv::VideoCapture capture(video_path);
    if (!capture.isOpened()) {
        std::cerr << "[ERROR] Failed to open video: " << video_path << std::endl;
        return -1;
    }

    // 初始化detector
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

    // 准备输出
    const fs::path result_dir = "results";
    fs::create_directories(result_dir);

    // 获取输入视频信息(保证输入与输出的属性一致)
    const int frame_width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int frame_height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = capture.get(cv::CAP_PROP_FPS) > 0 ? capture.get(cv::CAP_PROP_FPS) : 30.0;

    const std::string output_path = (result_dir / "corner_demo.avi").string();

    // 创建videowriter
    cv::VideoWriter writer(
        output_path,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        cv::Size(frame_width, frame_height)
    );
    if (!writer.isOpened()) {
        std::cerr << "[WARN] cannot open writer, skip saving video" << std::endl;
    }

    cv::Mat frame;
    int frame_id = 0;
    int frames_with_armor = 0;
    int total_corners = 0;
    const auto start = std::chrono::steady_clock::now();

    while (capture.read(frame)) {
        const std::vector<Armor> armors = detector->detect(frame);
        if (!armors.empty()) {
            ++frames_with_armor;
        }

        // 每个检测框 -> 4 角点 -> 画出来
        for (const Armor& armor: armors) {
            const std::vector<cv::Point2f> corners = rectToCorners(armor.rect);
            total_corners += static_cast<int>(corners.size());

            cv::rectangle(frame, armor.rect, cv::Scalar(0, 255, 0), 1); // 原框画细一点
            drawCorners(frame, corners);
        }

        if (frame_id % 120 == 0) {
            const std::string preview =
                (result_dir / cv::format("corner_preview_%04d.png", frame_id)).string();
            cv::imwrite(preview, frame);
        }
        if (writer.isOpened()) {
            writer.write(frame);
        }
        if (frame_id % 60 == 0) {
            std::cout << "[frame " << frame_id << "] " << armors.size() << " armor(s)" << std::endl;
        }
        ++frame_id;
        if (max_frames > 0 && frame_id >= max_frames) {
            break;
        }
    }

    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    writer.release();

    std::cout << "========================================" << std::endl;
    std::cout << "[SUMMARY] frames : " << frame_id << " | with armor: " << frames_with_armor << " ("
              << cv::format("%.1f", 100.0 * frames_with_armor / std::max(frame_id, 1)) << "%)"
              << std::endl;
    std::cout << "[SUMMARY] corner quads drawn: " << total_corners / 4 << std::endl;
    std::cout << "[SUMMARY] avg: " << cv::format("%.1f", elapsed_ms / std::max(frame_id, 1))
              << " ms/frame" << std::endl;
    std::cout << "[SUMMARY] saved: " << output_path << std::endl;
    return 0;
}
