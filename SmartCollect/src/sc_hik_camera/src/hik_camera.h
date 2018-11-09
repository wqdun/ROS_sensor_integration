#ifndef __HIK_CAMERA_ARM_H__
#define __HIK_CAMERA_ARM_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
// #include <stdlib.h>
#include <vector>
// #include <boost/thread/thread.hpp>
// #include <signal.h>
// #include <sstream>
// #include <boost/algorithm/string.hpp>
// #include <time.h>
#include <assert.h>
#include <mutex>

#include <opencv2/opencv.hpp>
#include "include/MvCameraControl.h"

class HikCamera {
public:
    HikCamera(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~HikCamera();
    void Run();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static void Convert2Mat(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo);

    static std::mutex s_matImageMutex_;
    static MV_CC_DEVICE_INFO_LIST s_deviceList_;
    static std::vector<void *> s_handles_;

    ros::NodeHandle nh_;
    int err_;
    std::vector<void *> handles_;
    std::vector<image_transport::Publisher> pubImages_;


    void EnumGigeDevices();
    int Get1stByteIP(size_t index);
    void OpenConfigDevices(size_t index);
    void ConfigDevices(size_t index, void *_handle);
    void SetAdvertiseHandle(size_t, int);

    void RegisterCB();
    void DoClean();
    void PressEnterToExit();
};

#endif

