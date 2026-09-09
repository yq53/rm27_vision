#pragma once

// camera_source：统一相机接入抽象（P0a，2026-09-10 起）
//
// 目的：现场演示时"只改 camera.yaml 的序列号即可换源"。
// 后端：
//   video  cv::VideoCapture 读本地视频文件（可 loop，用于无相机自测）
//   ip     cv::VideoCapture 读网络流（如手机 IP Webcam http://<ip>:8080/video）
//   hik    海康 MVS SDK（需编译期开关 USE_HIK_SDK=ON，默认 OFF；现场有 SDK 再开）
//
// 用法：
//   CameraConfig cfg = loadCameraConfig("data/camera.yaml");
//   auto cam = openCamera(cfg);
//   cam->read(frame); ...

#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace rm_camera {

// 与 data/camera.yaml 一一对应的配置结构
struct CameraConfig {
    std::string backend; // "video" | "ip" | "hik"
    // video / ip 共用（ip 时 video_path 存 URL 亦可，分开更直白）
    std::string video_path = "data/demo.avi";
    std::string ip_url = "";
    bool loop = true; // 仅文件类源生效：读到尾自动回卷
    // hik
    std::string serial_number = ""; // 现场只改这里
    // 通用图像参数
    int width = 1280;
    int height = 720;
    // 曝光/增益（hik 后端生效；video/ip 忽略）
    bool exposure_auto = true;
    double exposure_time_us = 2000.0; // 手动曝光时长(us)
    bool gain_auto = true;
    double gain = 8.0;
};

// 读取 yaml 配置；字段可缺省（有默认值）
CameraConfig loadCameraConfig(const std::string& yaml_path);

// 统一的相机句柄接口（所有后端一致的最小面）
class CameraSource {
public:
    virtual ~CameraSource() = default;

    virtual bool isOpened() const = 0;
    // 读一帧到 out；文件源按 loop 决定是否回卷
    virtual bool read(cv::Mat& out) = 0;
    // 透传 OpenCV 属性（hik 后端实现成自身参数查询/设置）
    virtual double get(int prop_id) const = 0;
    virtual bool set(int prop_id, double value) = 0;
};

// 工厂：按 cfg.backend 构造对应实现；hik 未编译时返回 nullptr 并输出原因
std::unique_ptr<CameraSource> openCamera(const CameraConfig& cfg);

} // namespace rm_camera
