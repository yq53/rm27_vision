#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "camera_source/camera_source.hpp"

// 探针 demo：camera_probe [camera.yaml] [最大帧数]
// 读配置 -> 打开相机 -> 逐帧读取并统计 -> 保存首帧到 results/camera_probe.png
int main(int argc, char** argv) {
    const std::string yaml_path = (argc > 1) ? argv[1] : "data/camera.yaml";
    const int max_frames = (argc > 2) ? std::stoi(argv[2]) : 60;

    rm_camera::CameraConfig cfg;
    try {
        cfg = rm_camera::loadCameraConfig(yaml_path);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 读取 yaml 失败: " << e.what() << std::endl;
        return 1;
    }

    std::printf(
        "配置: backend=%s  serial=%s  %dx%d  exposure_auto=%d gain_auto=%d\n",
        cfg.backend.c_str(),
        cfg.serial_number.empty() ? "-" : cfg.serial_number.c_str(),
        cfg.width,
        cfg.height,
        (int)cfg.exposure_auto,
        (int)cfg.gain_auto
    );

    std::unique_ptr<rm_camera::CameraSource> cam;
    try {
        cam = rm_camera::openCamera(cfg);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 打开相机失败: " << e.what() << std::endl;
        return 1;
    }
    if (!cam || !cam->isOpened()) {
        std::cerr << "[ERROR] 相机未能打开（backend=" << cfg.backend << "）" << std::endl;
        return 1;
    }

    int n = 0;
    double sum_ms = 0.0;
    cv::Mat first;
    const auto t0 = std::chrono::steady_clock::now();
    cv::Mat frame;
    while (n < max_frames && cam->read(frame)) {
        if (n == 0) {
            first = frame.clone();
            std::printf("首帧尺寸: %dx%d\n", frame.cols, frame.rows);
        }
        ++n;
        if (n % 20 == 0) {
            const auto dt = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0
            ).count();
            sum_ms += dt;
            std::printf("已读 %d 帧，最近 20 帧用时 %.0f ms\n", n, dt);
        }
    }
    const auto total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (n > 0) {
        std::printf("共读 %d 帧，总用时 %.2f s（平均 %.1f ms/帧）\n", n, total, total * 1000.0 / n);
        cv::imwrite("results/camera_probe.png", first);
        std::printf("首帧已存: results/camera_probe.png\n");
    }
    return 0;
}
