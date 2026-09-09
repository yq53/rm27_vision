// hik_probe：第一个真正调用海康 MVS SDK 的程序（第 A 步，2026-09-10）
//
// 目标：把"SDK 环境"跑通——能编译、能链接、能调用 API。
// 无相机时它打印"发现 0 台"并干净退出，这正是本步要验证的（见 main 末尾）。
//
// 本文件刻意短小、注释密集：学的是"调用 SDK 的骨架长什么样"。

#include <cstdio>
#include <cstring>

#include "MvCameraControl.h" // 海康相机控制接口：所有 MV_CC_* 函数都声明在这里

namespace {

// 把 SDK 返回码翻译成能看懂的报错（正式写法常查 MvErrorDefine.h）
void printRet(const char* what, int ret) {
    if (ret != MV_OK) {
        std::printf("[%s] 失败，错误码 = 0x%X（MV_OK=0 才是成功）\n", what, ret);
    } else {
        std::printf("[%s] 成功\n", what);
    }
}

// 从设备信息结构里取出序列号
void printSerial(const MV_CC_DEVICE_INFO* dev) {
    if (dev == nullptr) {
        return;
    }
    if (dev->nTLayerType == MV_GIGE_DEVICE) {
        std::printf("  - [GigE 网口相机] 序列号 = %s\n", dev->SpecialInfo.stGigEInfo.chSerialNumber);
    } else if (dev->nTLayerType == MV_USB_DEVICE) {
        std::printf("  - [USB3 相机] 序列号 = %s\n", dev->SpecialInfo.stUsb3VInfo.chSerialNumber);
    } else {
        std::printf("  - [其他类型 0x%X]\n", dev->nTLayerType);
    }
}

} // namespace

int main() {
    // SDK初始化
    std::printf("== 1) 初始化 SDK ==\n");
    int ret = MV_CC_Initialize();
    printRet("MV_CC_Initialize", ret);
    if (ret != MV_OK) {
        return 1;
    }

    // 输出检测到的所有相机
    std::printf("== 2) 枚举相机 ==\n");
    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list)); // 先清零，避免残留垃圾数据
    // 第一个参数是"我要找哪种相机"：网口(GigE) + USB3 都要
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    printRet("MV_CC_EnumDevices", ret);
    if (ret != MV_OK) {
        MV_CC_Finalize();
        return 1;
    }

    std::printf("发现 %u 台相机：\n", device_list.nDeviceNum);
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        printSerial(device_list.pDeviceInfo[i]);
    }

    std::printf("== 3) 释放 SDK ==\n");
    ret = MV_CC_Finalize();
    printRet("MV_CC_Finalize", ret);

    // 无相机时的预期结果：发现 0 台、三步全部"成功"退出。
    // 这一步在家里的价值：把"环境/编译/链接"和"相机问题"彻底分开——
    //   能跑到这行 = 环境没问题，现场接上相机后同一份代码自然会枚举出 1 台。
    return 0;
}
