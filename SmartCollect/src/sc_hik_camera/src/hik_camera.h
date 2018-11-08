#ifndef __HIK_CAMERA_ARM_H__
#define __HIK_CAMERA_ARM_H__

#include <stdlib.h>
#include <vector>
// #include <boost/thread/thread.hpp>
#include <signal.h>
#include <sstream>
// #include <boost/algorithm/string.hpp>
#include <time.h>
#include <assert.h>
#include <opencv2/opencv.hpp>

#include "include/MvCameraControl.h"

class HikCamera {
public:
    HikCamera();
    ~HikCamera();
    void Run();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

    int err_;
    std::vector<void *> handles_;
    MV_CC_DEVICE_INFO_LIST deviceList_;

    void EnumGigeDevices();
    void PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void OpenConfigDevices();
    void ConfigDevices(size_t index);
    void RegisterCB();
    void DoClean();
    void PressEnterToExit();
};

#endif

