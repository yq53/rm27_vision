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

    // hik 源的"格式"相关，名字对齐战队自家项目 config/*.yaml 的写法。
    // 分两组：前三个写给相机（留空 = 不动相机、用它自己的默认值），最后一个管我们自己的输出。
    std::string pixel_format;  // 例 BayerRG8 / Mono8 / RGB8Packed（海康默认常是 Bayer 原始格式）
    std::string adc_bit_depth; // 例 Bits_8（钉死 8bit；10/12bit 会变成 BayerRG10/12，处理方式不同）
    std::string trigger_mode;  // 例 Off（连续采集；On 而没有触发信号时一帧都收不到）
    std::string format = "bgr"; // 交给下游的像素格式；本工程下游（检测器 / cv_bridge）按 BGR8 处理
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
