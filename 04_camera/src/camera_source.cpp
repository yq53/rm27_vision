#include "camera_source/camera_source.hpp"

#include <stdexcept>
#include <string>

#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

namespace rm_camera {

namespace {

// 从 yaml 节点安全读取可选字段：节点存在才覆盖默认值
template <typename T>
void assignIfPresent(const YAML::Node& node, const char* key, T& dst) {
    if (node[key]) {
        dst = node[key].as<T>();
    }
}

// 后端一：文件/设备/网络流都走 cv::VideoCapture，只是 URL 与 loop 语义不同
class VideoCaptureSource: public CameraSource {
public:
    explicit VideoCaptureSource(std::string source, bool loop):
        loop_(loop) {
        cap_.open(source);
    }

    bool isOpened() const override {
        return cap_.isOpened();
    }

    bool read(cv::Mat& out) override {
        if (!cap_.read(out)) {
            // 文件到尾：可选回卷再读一次（网络流/相机 read 阻塞失败则直接返回 false）
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                return cap_.read(out);
            }
            return false;
        }
        return true;
    }

    double get(int prop_id) const override {
        return cap_.get(prop_id);
    }

    bool set(int prop_id, double value) override {
        return cap_.set(prop_id, value);
    }

private:
    cv::VideoCapture cap_;
    bool loop_ = true;
};

#ifdef RM_USE_HIK_SDK
// 后端三：海康 MVS SDK（编译期开关 USE_HIK_SDK=ON 才编译）
// 现场接入步骤（TODO 现场执行）：
//   1) 安装 MVS SDK 并保证头文件/库路径可达（cmake 时 -DMVS_INCLUDE_DIR=... -DMVS_LIB=...）
//   2) MV_CC_Initialize() -> 按 serial_number 枚举/打开设备 -> 设置宽高/曝光/增益
//   3) MV_CC_StartGrabbing() 后每帧取图转 cv::Mat(BGR) 返回
class HikCameraSource: public CameraSource {
public:
    explicit HikCameraSource(const CameraConfig& cfg) {
        // TODO: 用 cfg.serial_number/width/height/exposure_auto/exposure_time_us/gain_auto/gain
        //       打开海康设备并应用参数。先占位抛错，保证未接 SDK 也能整体编译。
        throw std::runtime_error(
            "HikCameraSource: USE_HIK_SDK=ON 但接入代码尚未在无设备环境实现（现场填）");
    }
    bool isOpened() const override { return false; }
    bool read(cv::Mat&) override { return false; }
    double get(int) const override { return -1; }
    bool set(int, double) override { return false; }
};
#endif // RM_USE_HIK_SDK

} // namespace

CameraConfig loadCameraConfig(const std::string& yaml_path) {
    CameraConfig cfg;
    const YAML::Node root = YAML::LoadFile(yaml_path); // 文件不存在会抛异常，由调用方捕获
    assignIfPresent(root, "backend", cfg.backend);
    assignIfPresent(root, "video_path", cfg.video_path);
    assignIfPresent(root, "ip_url", cfg.ip_url);
    assignIfPresent(root, "loop", cfg.loop);
    assignIfPresent(root, "serial_number", cfg.serial_number);
    assignIfPresent(root, "width", cfg.width);
    assignIfPresent(root, "height", cfg.height);
    assignIfPresent(root, "exposure_auto", cfg.exposure_auto);
    assignIfPresent(root, "exposure_time_us", cfg.exposure_time_us);
    assignIfPresent(root, "gain_auto", cfg.gain_auto);
    assignIfPresent(root, "gain", cfg.gain);
    return cfg;
}

std::unique_ptr<CameraSource> openCamera(const CameraConfig& cfg) {
    if (cfg.backend == "video") {
        return std::make_unique<VideoCaptureSource>(cfg.video_path, cfg.loop);
    }
    if (cfg.backend == "ip") {
        if (cfg.ip_url.empty()) {
            throw std::runtime_error("backend=ip 但 ip_url 为空");
        }
        return std::make_unique<VideoCaptureSource>(cfg.ip_url, /*loop=*/false);
    }
    if (cfg.backend == "hik") {
#ifdef RM_USE_HIK_SDK
        return std::make_unique<HikCameraSource>(cfg);
#else
        throw std::runtime_error(
            "backend=hik 需要编译期开关 USE_HIK_SDK=ON（现场装 MVS SDK 后再开）");
#endif
    }
    throw std::runtime_error("未知 backend: " + cfg.backend + "（可选 video/ip/hik）");
}

} // namespace rm_camera
