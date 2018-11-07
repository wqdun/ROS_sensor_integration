#ifndef __HIK_CAMERA_ARM_H__
#define __HIK_CAMERA_ARM_H__

#include <stdlib.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>

#include "MvCameraControl.h"
#include "../../../sc_lib_public_tools/src/thread_pool.h"
#include "../../../sc_lib_public_tools/src/tools_no_ros.h"
#include "image_task.h"


class HikCamera {
public:
    HikCamera(bool *_isNodeRunning);
    ~HikCamera();
    void Run();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static void ConvertSaveImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
    static void Convert2Mat(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser, double time);
    static void SaveImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser, double _unixTime);
    static bool *s_pIsCameraRunning_;
    static threadpool<ImageTask> s_threadPool_;


    int err_;
    std::vector<void *> handles_;
    MV_CC_DEVICE_INFO_LIST deviceList_;
    boost::shared_ptr<boost::thread> pThread_;

    void EnumGigeDevices();
    void PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void OpenConfigDevices();
    void ConfigDevices(size_t index);
    void RegisterCB();
    void DoClean();
    void PressEnterToExit();
};

#endif

