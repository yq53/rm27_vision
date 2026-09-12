// [核心] 图像源抽象实现与工厂（video / ip / hik；hik 段由 RM_USE_HIK_SDK 开关控制）
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

    // 判断VideoCapture是否启动
    bool isOpened() const override {
        return cap_.isOpened();
    }

    // VideoCapture读取帧
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

    // 获取源视频帧率
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

// 错误码翻译
void printRet(const char* what, int ret) {
    if (ret != MV_OK) {
        std::printf("[hik_source][%s] 失败，错误码 = 0x%X\n", what, ret);
    }
}

// 获取序列号
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

// 像素格式 -> 可读名字（只列常见几种，其余返回"未知格式"）
const char* pixelTypeName(MvGvspPixelType type) {
    switch (type) {
    case PixelType_Gvsp_BGR8_Packed: return "BGR8_Packed";
    case PixelType_Gvsp_RGB8_Packed: return "RGB8_Packed";
    case PixelType_Gvsp_Mono8:       return "Mono8";
    case PixelType_Gvsp_BayerRG8:    return "BayerRG8";
    case PixelType_Gvsp_BayerGR8:    return "BayerGR8";
    case PixelType_Gvsp_BayerGB8:    return "BayerGB8";
    case PixelType_Gvsp_BayerBG8:    return "BayerBG8";
    default:                         return "未知格式";
    }
}

// 把 相机流 的一帧转成 BGR；返回 false 表示这个格式我们不认识（调用方跳过该帧）
bool frameToBgr(const MV_FRAME_OUT& frame_out, cv::Mat& out) {
    const auto& info = frame_out.stFrameInfo;   // 图像信息
    const int w = static_cast<int>(info.nWidth);    // width
    const int h = static_cast<int>(info.nHeight);   // height
    unsigned char* data = frame_out.pBufAddr;       // 图像指针地址

    // 像素格式检测，并处理输出成BGR
    switch (info.enPixelType) {
    case PixelType_Gvsp_BGR8_Packed: { // 相机直接给 BGR：零转换
        cv::Mat raw(h, w, CV_8UC3, data);
        raw.copyTo(out); // 拷贝出 SDK 缓冲区
        return true;
    }
    case PixelType_Gvsp_RGB8_Packed: { // 相机给 RGB：换个通道顺序
        cv::Mat raw(h, w, CV_8UC3, data);
        cv::cvtColor(raw, out, cv::COLOR_RGB2BGR);
        return true;
    }
    case PixelType_Gvsp_Mono8: { // 灰度：复制成三通道
        cv::Mat raw(h, w, CV_8UC1, data);
        cv::cvtColor(raw, out, cv::COLOR_GRAY2BGR);
        return true;
    }
    case PixelType_Gvsp_BayerRG8: { // Bayer 原始格式（海康常见默认，第一行 R G）：去马赛克
        cv::Mat raw(h, w, CV_8UC1, data);
        cv::cvtColor(raw, out, cv::COLOR_BayerBG2BGR);
        return true;
    }
    case PixelType_Gvsp_BayerGR8: {
        cv::Mat raw(h, w, CV_8UC1, data);
        cv::cvtColor(raw, out, cv::COLOR_BayerGB2BGR);
        return true;
    }
    case PixelType_Gvsp_BayerGB8: {
        cv::Mat raw(h, w, CV_8UC1, data);
        cv::cvtColor(raw, out, cv::COLOR_BayerGR2BGR);
        return true;
    }
    case PixelType_Gvsp_BayerBG8: {
        cv::Mat raw(h, w, CV_8UC1, data);
        cv::cvtColor(raw, out, cv::COLOR_BayerRG2BGR);
        return true;
    }
    default:
        return false;
    }
}

} // namespace

class HikSource: public ImageSource {
public:
    // 构造即打开：失败会打印原因并抛异常（由工厂/节点捕获）
    explicit HikSource(const SourceConfig& cfg) {
        // 初始化机械视觉相机控制系统
        if (MV_CC_Initialize() != MV_OK) {
            throw std::runtime_error("hik: MV_CC_Initialize 失败");
        }

        // 获取设备列表
        MV_CC_DEVICE_INFO_LIST list;
        std::memset(&list, 0, sizeof(list));
        if (MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list) != MV_OK
            || list.nDeviceNum == 0) {
            std::printf("[hik_source] 未发现相机（检查网线/USB 与相机电源）\n");
            MV_CC_Finalize();
            throw std::runtime_error("hik: 未发现相机(serial=" + cfg.serial_number + ")");
        }

        // 遍历搜索目标设备序列号
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

        // 创建handle并链接设备
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

        // 发送请求
        if (!cfg.pixel_format.empty()) {
            const int r =
                MV_CC_SetEnumValueByString(handle_, "PixelFormat", cfg.pixel_format.c_str());
            std::printf(
                "[hik_source] 请求像素格式 %s：%s\n",
                cfg.pixel_format.c_str(),
                r == MV_OK ? "成功" : "该相机不支持，将按它实际输出的格式做转换"
            );
        }
        if (!cfg.adc_bit_depth.empty()) {
            const int r =
                MV_CC_SetEnumValueByString(handle_, "ADCBitDepth", cfg.adc_bit_depth.c_str());
            if (r != MV_OK) {
                std::printf(
                    "[hik_source] 设置 ADCBitDepth=%s 失败（0x%X）：若输出变成 BayerRG10/12 请检查此项\n",
                    cfg.adc_bit_depth.c_str(),
                    r
                );
            }
        }
        if (!cfg.trigger_mode.empty()) {
            const int r =
                MV_CC_SetEnumValueByString(handle_, "TriggerMode", cfg.trigger_mode.c_str());
            if (r != MV_OK) {
                std::printf(
                    "[hik_source] 设置 TriggerMode=%s 失败（0x%X）：若一帧都收不到请检查此项\n",
                    cfg.trigger_mode.c_str(),
                    r
                );
            }
        }
        if (cfg.format != "bgr") {
            std::printf(
                "[hik_source] 注意 format=%s：本工程下游（检测器 / cv_bridge）按 BGR8 处理，"
                "这里仍输出 bgr\n",
                cfg.format.c_str()
            );
        }

        // 捕获相机流，推入buffer
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

    // 设备是否开启
    bool isOpened() const override {
        return (handle_ != nullptr) && grabbing_;
    }

    // 相机流读取函数
    bool read(cv::Mat& out) override {
        if (!isOpened()) {
            return false;
        }
        MV_FRAME_OUT frame_out;
        std::memset(&frame_out, 0, sizeof(frame_out));

        // 从buffer中取帧，最多等待1000ms(1s)
        const int ret = MV_CC_GetImageBuffer(handle_, &frame_out, 1000);
        if (ret != MV_OK) {
            printRet("GetImageBuffer", ret);
            return false;
        }

        // 相机实际输出的格式只报一次，方便现场确认
        const auto& info = frame_out.stFrameInfo;
        if (!first_frame_reported_) {
            std::printf(
                "[hik_source] 相机实际输出格式：%s（0x%08X）\n",
                pixelTypeName(info.enPixelType),
                static_cast<unsigned>(info.enPixelType)
            );
            first_frame_reported_ = true;
        }

        const bool ok = frameToBgr(frame_out, out);

        // 转化错误，未知输入格式
        if (!ok) {
            // 前几帧各报一次，之后每 300 帧报一次，避免刷屏
            ++skipped_frames_;
            if (skipped_frames_ <= 3 || skipped_frames_ % 300 == 0) {
                std::printf(
                    "[hik_source] 不支持的像素格式 %s（0x%08X），累计跳过 %d 帧；"
                    "可在 data/camera.yaml 设置 pixel_format，或检查相机是否被设成 10/12bit 输出\n",
                    pixelTypeName(info.enPixelType),
                    static_cast<unsigned>(info.enPixelType),
                    skipped_frames_
                );
            }
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
    bool first_frame_reported_ = false; // 首帧报一次"相机实际输出格式"
    int skipped_frames_ = 0;            // 累计跳过的帧数（格式不认识时）
};
#endif // RM_USE_HIK_SDK

} // namespace

// 加载yaml文件
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
    if (root["pixel_format"]) {
        cfg.pixel_format = root["pixel_format"].as<std::string>();
    }
    if (root["adc_bit_depth"]) {
        cfg.adc_bit_depth = root["adc_bit_depth"].as<std::string>();
    }
    if (root["trigger_mode"]) {
        cfg.trigger_mode = root["trigger_mode"].as<std::string>();
    }
    if (root["format"]) {
        cfg.format = root["format"].as<std::string>();
    }
    return cfg;
#else
    (void)yaml_path;
    throw std::runtime_error(
        "camera_config 需要以 -DUSE_HIK_SDK=ON 构建本包（见 README）");
#endif
}

// 
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
