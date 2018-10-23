#ifndef __HIK_CAMERA_ARM_H__
#define __HIK_CAMERA_ARM_H__

#include <stdlib.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>

#include "MvCameraControl.h"
#include "serial_reader.h"
#include "system.h"

class HikCamera {
public:
    HikCamera(bool &_isNodeRunning);
    ~HikCamera();
    void Run();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static void ConvertSaveImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
    static void Convert2Mat(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser, double time);
    static void IMUProc(const std::deque<slamProtocol_t> &tenIMUMeasurements, vinssystem &mSystem);
    static cv::Mat ImageProc(cv::Mat srcImage, double header, vinssystem* mpSystem,pair<double,vector<Box>> onebox);

    static MV_CC_PIXEL_CONVERT_PARAM s_convertParam_;
    static boost::shared_ptr<SerialReader> s_pSerialReader_;
    static vinssystem system_;

    int err_;
    std::vector<void *> handles_;
    MV_CC_DEVICE_INFO_LIST deviceList_;
    boost::shared_ptr<boost::thread> pThread_;
    bool &isCameraRunning_;

    void EnumGigeDevices();
    void PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void OpenConfigDevices();
    void ConfigDevices(size_t index);
    void RegisterCB();
    void DoClean();
    void PressEnterToExit();
};

#endif

