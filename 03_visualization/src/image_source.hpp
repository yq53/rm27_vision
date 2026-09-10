#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

// 图像源抽象（D 步接口版，2026-09-10）
//
// 主节点只面对 ImageSource（纯 C++ 多态），不认识"海康/文件"这些具体源；
// 具体实现与编译开关全部收在 image_source.cpp 内，不泄漏到主节点。

namespace rm_vision {

// 与 data/camera.yaml 对应的配置（backend: "video" | "hik"）
struct SourceConfig {
    std::string backend = "video";
    std::string video_path = "data/demo.avi";
    bool loop = true; // video 源：读到尾回卷
    std::string serial_number; // hik 源：现场只改这个
    double exposure_time_us = 2000.0;
    double gain = 8.0;
};

// 读取 camera.yaml。默认构建(未开 USE_HIK_SDK)不支持，会抛异常说明。
SourceConfig loadSourceConfig(const std::string& yaml_path);

// 图像源统一接口：拿一帧 BGR 图 + 状态查询
class ImageSource {
public:
    virtual ~ImageSource() = default;
    virtual bool read(cv::Mat& out) = 0; // 成功写 out(BGR)，失败返回 false
    virtual bool isOpened() const = 0;
    virtual double fpsHint() const = 0; // 节拍参考（video 读文件帧率；hik 按 30）
    // 约定：实现应返回 > 0；但主节点仍会做一次 fps<=0 兜底（防御性，避免 1/fps=inf）
};

// 工厂：按 cfg.backend 返回 VideoSource / HikSource。
// backend=hik 但未以 USE_HIK_SDK 编译时会抛异常说明。
std::unique_ptr<ImageSource> createImageSource(const SourceConfig& cfg);

} // namespace rm_vision
