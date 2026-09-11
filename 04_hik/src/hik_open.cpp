// [教学] 海康 MVS SDK 学习 demo · B 步：按序列号打开相机 + 设曝光/增益
// hik_open：按序列号打开相机并设置曝光/增益（B 步）
//
// 用法：hik_open [序列号]
//   不传参时默认 "000000000000"（用于演示"找不到目标相机"的报错路径）。
// 无相机环境下：枚举到 0 台即说明并退出，属预期结果。

#include <cstdio>
#include <cstring>
#include <string>

#include "MvCameraControl.h"

namespace {

void printRet(const char* what, int ret) {
    if (ret != MV_OK) {
        std::printf("[%s] 失败，错误码 = 0x%X\n", what, ret);
    } else {
        std::printf("[%s] 成功\n", what);
    }
}

// 返回一台设备信息的序列号（GigE 与 USB3 字段位置不同；字段是 unsigned char[]）
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

int main(int argc, char** argv) {
    const std::string want = (argc > 1) ? argv[1] : "000000000000";

    // 初始化
    int ret = MV_CC_Initialize();
    printRet("MV_CC_Initialize", ret);
    if (ret != MV_OK) {
        return 1;
    }

    // 获取设备列表
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
        std::printf("未发现相机（本机无相机属预期）；接上相机后重跑本程序即可。\n");
        MV_CC_Finalize();
        return 0;
    }

    // 遍历设备列表，找到匹配的序列号
    int idx = -1;
    for (unsigned int i = 0; i < list.nDeviceNum; ++i) {
        const std::string sn = serialOf(list.pDeviceInfo[i]);
        std::printf("  [%u] 序列号 %s\n", i, sn.empty() ? "(未知)" : sn.c_str());
        if (idx < 0 && sn == want) {
            idx = static_cast<int>(i);
        }
    }
    if (idx < 0) {
        std::printf("[错误] 未找到序列号为 %s 的相机（对照 MVS 客户端里的实际序列号）。\n", want.c_str());
        MV_CC_Finalize();
        return 1;
    }

    // 创建句柄并连接上相机
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

    // 关闭自动曝光、设固定曝光/增益（数值仅为演示，现场按环境调）
    ret = MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    printRet("关闭自动曝光", ret);
    ret = MV_CC_SetFloatValue(handle, "ExposureTime", 2000.0);
    printRet("ExposureTime=2000us", ret);
    ret = MV_CC_SetFloatValue(handle, "Gain", 8.0);
    printRet("Gain=8", ret);

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();
    std::printf("B 步流程结束：枚举->点名->打开->设参->关闭（本机无相机，仅验证流程与报错路径）。\n");
    return 0;
}
