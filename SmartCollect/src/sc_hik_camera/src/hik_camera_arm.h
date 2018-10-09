#ifndef __HIK_CAMERA_ARM_H__
#define __HIK_CAMERA_ARM_H__

#include <stdlib.h>
#include "MvCameraControl.h"

class HikCamera {
public:
    HikCamera();
    ~HikCamera();
    void run();


private:
    static void __stdcall imageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

    int err_;
    void *handle_;
    MV_CC_DEVICE_INFO_LIST deviceList_;

    void enumGigeDevices();
    void printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void open_configDevices();
    void configDevices(size_t index);
    void registerCB();
    void doClean();
    void PressEnterToExit();
};

#endif

