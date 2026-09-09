// hik_grab：取流并转成 OpenCV BGR 图（C 步）
//
// 用法：hik_grab [序列号] [取帧数]
//   流程 = B 步(枚举->点名->打开->设参) + StartGrabbing -> 循环取帧 -> 转 BGR -> 存图。
// 像素格式处理：相机输出 BGR8 直接包成 Mat；输出 Mono8 先包成灰度再用 cvtColor 转 BGR。
// 无相机环境：与 hik_open 相同，枚举 0 台即提示退出（编译与流程验证在本机完成，
// 真机取流留待现场）。

#include <cstdio>
#include <cstring>
#include <string>

#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"

namespace {

void printRet(const char* what, int ret) {
    if (ret != MV_OK) {
        std::printf("[%s] 失败，错误码 = 0x%X\n", what, ret);
    } else {
        std::printf("[%s] 成功\n", what);
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

const char* pixelName(unsigned int type) {
    if (type == PixelType_Gvsp_BGR8_Packed) {
        return "BGR8";
    }
    if (type == PixelType_Gvsp_Mono8) {
        return "Mono8";
    }
    return "(其他)";
}

} // namespace

int main(int argc, char** argv) {
    const std::string want = (argc > 1) ? argv[1] : "000000000000";
    const int want_frames = (argc > 2) ? std::stoi(argv[2]) : 30;

    int ret = MV_CC_Initialize();
    printRet("MV_CC_Initialize", ret);
    if (ret != MV_OK) {
        return 1;
    }

    MV_CC_DEVICE_INFO_LIST list;
    std::memset(&list, 0, sizeof(list));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
    printRet("MV_CC_EnumDevices", ret);
    if (ret != MV_OK) {
        MV_CC_Finalize();
        return 1;
    }

    std::printf("在线设备 %u 台，目标序列号：%s\n", list.nDeviceNum, want.c_str());
    if (list.nDeviceNum == 0) {
        std::printf("未发现相机（本机无相机属预期）；接上相机后重跑即可。\n");
        MV_CC_Finalize();
        return 0;
    }

    int idx = -1;
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        const std::string sn = serialOf(list.pDeviceInfo[i]);
        std::printf("  [%u] 序列号 %s\n", i, sn.empty() ? "(未知)" : sn.c_str());
        if (idx < 0 && sn == want) {
            idx = static_cast<int>(i);
        }
    }
    if (idx < 0) {
        std::printf("[错误] 未找到序列号为 %s 的相机。\n", want.c_str());
        MV_CC_Finalize();
        return 1;
    }

    void* handle = nullptr;
    ret = MV_CC_CreateHandle(&handle, list.pDeviceInfo[idx]);
    printRet("MV_CC_CreateHandle", ret);
    if (ret != MV_OK) {
        MV_CC_Finalize();
        return 1;
    }
    ret = MV_CC_OpenDevice(handle);
    printRet("MV_CC_OpenDevice", ret);
    if (ret != MV_OK) {
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return 1;
    }

    ret = MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    ret = MV_CC_SetFloatValue(handle, "ExposureTime", 2000.0);
    printRet("设曝光", ret);

    // 开始出流（相机开始持续送帧）
    ret = MV_CC_StartGrabbing(handle);
    printRet("MV_CC_StartGrabbing", ret);
    if (ret != MV_OK) {
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return 1;
    }

    int got = 0;
    for (int i = 0; i < want_frames; ++i) {
        MV_FRAME_OUT frame_out;
        std::memset(&frame_out, 0, sizeof(frame_out));
        // 阻塞最多 1000ms 等一帧；帧数据由 SDK 管理，用完必须 FreeImageBuffer 归还
        ret = MV_CC_GetImageBuffer(handle, &frame_out, 1000);
        if (ret != MV_OK) {
            std::printf("[取帧 %d] 超时或失败，错误码 = 0x%X\n", i, ret);
            continue;
        }

        const auto& info = frame_out.stFrameInfo;
        std::printf(
            "帧 %d: %ux%u 像素格式=%s(%d)\n",
            i,
            info.nWidth,
            info.nHeight,
            pixelName(info.enPixelType),
            static_cast<int>(info.enPixelType)
        );

        // 原始数据只包不拷贝（零成本）；需要保存/继续处理时再 clone
        cv::Mat frame;
        if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
            frame = cv::Mat(info.nHeight, info.nWidth, CV_8UC3, frame_out.pBufAddr[0]);
        } else if (info.enPixelType == PixelType_Gvsp_Mono8) {
            cv::Mat gray(info.nHeight, info.nWidth, CV_8UC1, frame_out.pBufAddr[0]);
            cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);
        } else {
            std::printf("  不支持的像素格式，跳过本帧\n");
            MV_CC_FreeImageBuffer(handle, &frame_out);
            continue;
        }

        if (got == 0) {
            cv::imwrite("results/hik_grab_0000.png", frame);
            std::printf("  首帧已存 results/hik_grab_0000.png\n");
        }
        ++got;
        MV_CC_FreeImageBuffer(handle, &frame_out); // 归还缓冲区给 SDK
    }
    std::printf("取帧完成：成功 %d/%d 帧。\n", got, want_frames);

    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    return 0;
}
