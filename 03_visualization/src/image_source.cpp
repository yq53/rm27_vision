// image_source.cpp：图像源实现与工厂（D 步接口版）
//
// 结构：
//   [无开关段] VideoSource（cv::VideoCapture）—— 永远编译，覆盖旧用法
//   [#ifdef RM_USE_HIK_SDK 段] yaml 读取 + HikSource（MVS）—— 开关护住海康专属代码
// 这样默认构建零 MVS/yaml 依赖，主节点零预处理分支。

#include "image_source.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifdef RM_USE_HIK_SDK
#include <yaml-cpp/yaml.h>
#include "MvCameraControl.h"
#endif

namespace rm_vision {

namespace {

// ---------- video 源：包装 cv::VideoCapture（文件 / IP 流 / 普通相机） ----------
class VideoSource: public ImageSource {
public:
    explicit VideoSource(const SourceConfig& cfg): loop_(cfg.loop) {
        cap_.open(cfg.video_path);
    }

    bool isOpened() const override {
        return cap_.isOpened();
    }

    bool read(cv::Mat& out) override {
        if (cap_.read(out)) {
            return true;
        }
        if (loop_) { // 文件到尾回卷（保持旧行为）
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return cap_.read(out);
        }
        return false;
    }

    double fpsHint() const override {
        const double fps = cap_.get(cv::CAP_PROP_FPS);
        return fps > 0.0 ? fps : 30.0;
    }

private:
    cv::VideoCapture cap_;
    bool loop_ = true;
};

#ifdef RM_USE_HIK_SDK
// ---------- hik 源：海康 MVS SDK（仅 USE_HIK_SDK=ON 时编译） ----------
namespace {

void printRet(const char* what, int ret) {
    if (ret != MV_OK) {
        std::printf("[hik_source][%s] 失败，错误码 = 0x%X\n", what, ret);
    }
}

std::string serialOf(const MV_CC_DEVICE_INFO* dev) {
    if (dev == nullptr) {
        return "";
    }
    if (dev->nTLayerType == MV_GIGE_DEVICE) {
        return std::string(
            reinterpret_cast<const char*>(dev->SpecialInfo.stGigEInfo.chSerialNumber)
        );
    }
    if (dev->nTLayerType == MV_USB_DEVICE) {
        return std::string(
            reinterpret_cast<const char*>(dev->SpecialInfo.stUsb3VInfo.chSerialNumber)
        );
    }
    return "";
}

} // namespace

class HikSource: public ImageSource {
public:
    // 构造即打开：失败会打印原因并抛异常（由工厂/节点捕获）
    explicit HikSource(const SourceConfig& cfg) {
        if (MV_CC_Initialize() != MV_OK) {
            throw std::runtime_error("hik: MV_CC_Initialize 失败");
        }

        MV_CC_DEVICE_INFO_LIST list;
        std::memset(&list, 0, sizeof(list));
        if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list) != MV_OK
            || list.nDeviceNum == 0) {
            std::printf("[hik_source] 未发现相机（检查网线/USB 与相机电源）\n");
            MV_CC_Finalize();
            throw std::runtime_error("hik: 未发现相机(serial=" + cfg.serial_number + ")");
        }

        int idx = -1;
        for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
            const std::string sn = serialOf(list.pDeviceInfo[i]);
            std::printf("[hik_source] 在线设备 %u: %s\n", i, sn.c_str());
            if (sn == cfg.serial_number) {
                idx = static_cast<int>(i);
            }
        }
        if (idx < 0) {
            std::printf("[hik_source] 未找到序列号 %s（对照 MVS 客户端实际序列号）\n",
                        cfg.serial_number.c_str());
            MV_CC_Finalize();
            throw std::runtime_error("hik: 未找到序列号 " + cfg.serial_number);
        }

        if (MV_CC_CreateHandle(&handle_, list.pDeviceInfo[idx]) != MV_OK) {
            handle_ = nullptr;
            MV_CC_Finalize();
            throw std::runtime_error("hik: MV_CC_CreateHandle 失败");
        }
        if (MV_CC_OpenDevice(handle_) != MV_OK) {
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            MV_CC_Finalize();
            throw std::runtime_error("hik: MV_CC_OpenDevice 失败(可能被占用)");
        }

        // 关自动曝光 + 固定曝光/增益（现场按 MVS 客户端与光线调整）
        MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", cfg.exposure_time_us);
        printRet("ExposureTime", ret);
        ret = MV_CC_SetFloatValue(handle_, "Gain", cfg.gain);
        printRet("Gain", ret);

        if (MV_CC_StartGrabbing(handle_) != MV_OK) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            MV_CC_Finalize();
            throw std::runtime_error("hik: MV_CC_StartGrabbing 失败");
        }
        grabbing_ = true;
        std::printf("[hik_source] 已打开序列号 %s（曝光 %.0fus, 增益 %.1f）\n",
                    cfg.serial_number.c_str(), cfg.exposure_time_us, cfg.gain);
    }

    ~HikSource() override {
        close();
    }

    bool isOpened() const override {
        return handle_ != nullptr && grabbing_;
    }

    bool read(cv::Mat& out) override {
        if (!isOpened()) {
            return false;
        }
        MV_FRAME_OUT frame_out;
        std::memset(&frame_out, 0, sizeof(frame_out));
        const int ret = MV_CC_GetImageBuffer(handle_, &frame_out, 1000);
        if (ret != MV_OK) {
            printRet("GetImageBuffer", ret);
            return false;
        }
        const auto& info = frame_out.stFrameInfo;
        bool ok = false;
        if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
            cv::Mat raw(info.nHeight, info.nWidth, CV_8UC3, frame_out.pBufAddr);
            raw.copyTo(out); // 拷贝出 SDK 缓冲区
            ok = true;
        } else if (info.enPixelType == PixelType_Gvsp_Mono8) {
            cv::Mat gray(info.nHeight, info.nWidth, CV_8UC1, frame_out.pBufAddr);
            cv::cvtColor(gray, out, cv::COLOR_GRAY2BGR);
            ok = true;
        } else {
            std::printf("[hik_source] 不支持的像素格式 %d，跳过本帧\n",
                        static_cast<int>(info.enPixelType));
        }
        MV_CC_FreeImageBuffer(handle_, &frame_out); // 归还缓冲
        return ok;
    }

    double fpsHint() const override {
        return 30.0;
    }

private:
    void close() {
        if (handle_ == nullptr) {
            return;
        }
        if (grabbing_) {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        MV_CC_Finalize();
        std::printf("[hik_source] 已关闭\n");
    }

    void* handle_ = nullptr;
    bool grabbing_ = false;
};
#endif // RM_USE_HIK_SDK

} // namespace

SourceConfig loadSourceConfig(const std::string& yaml_path) {
#ifdef RM_USE_HIK_SDK
    SourceConfig cfg;
    const YAML::Node root = YAML::LoadFile(yaml_path); // 文件/语法错误抛异常
    if (root["backend"]) {
        cfg.backend = root["backend"].as<std::string>();
    }
    if (root["video_path"]) {
        cfg.video_path = root["video_path"].as<std::string>();
    }
    if (root["loop"]) {
        cfg.loop = root["loop"].as<bool>();
    }
    if (root["serial_number"]) {
        cfg.serial_number = root["serial_number"].as<std::string>();
    }
    if (root["exposure_time_us"]) {
        cfg.exposure_time_us = root["exposure_time_us"].as<double>();
    }
    if (root["gain"]) {
        cfg.gain = root["gain"].as<double>();
    }
    return cfg;
#else
    (void)yaml_path;
    throw std::runtime_error(
        "camera_config 需要以 -DUSE_HIK_SDK=ON 构建本包（见 README）");
#endif
}

std::unique_ptr<ImageSource> createImageSource(const SourceConfig& cfg) {
    if (cfg.backend == "hik") {
#ifdef RM_USE_HIK_SDK
        return std::make_unique<HikSource>(cfg); // 打开失败会抛异常
#else
        throw std::runtime_error("backend=hik 需要以 -DUSE_HIK_SDK=ON 构建本包（见 README）");
#endif
    }
    if (cfg.backend != "video") {
        throw std::runtime_error("camera.yaml 的 backend 必须是 hik 或 video");
    }
    return std::make_unique<VideoSource>(cfg);
}

} // namespace rm_vision
