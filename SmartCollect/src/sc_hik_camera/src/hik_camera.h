#ifndef __HIK_CAMERA_ARM_H__
#define __HIK_CAMERA_ARM_H__

#include <stdlib.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <time.h>

#include "MvCameraControl.h"
#include "serial_reader.h"
#include "system.h"
#include "data_input.h"
#include "data_output.h"
#include "classification.h"
#include "../../sc_lib_public_tools/src/locker.h"

typedef struct {
    cv::Mat matImage;
    double header;
    std::deque<slamProtocol_t> slams;
} mat2SlamProtocols_t;

class HikCamera {
public:
    HikCamera(bool &_isNodeRunning);
    ~HikCamera();
    void Run();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static void ConvertSaveImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
    static void Convert2Mat(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);

    static MV_CC_PIXEL_CONVERT_PARAM s_convertParam_;
    static boost::shared_ptr<SerialReader> s_pSerialReader_;
    static std::deque<mat2SlamProtocols_t> s_mat2Slams_;
    static mutex_locker s_mutexLocker_;
    static vinssystem system_;

    int err_;
    std::vector<void *> handles_;
    MV_CC_DEVICE_INFO_LIST deviceList_;
    boost::shared_ptr<boost::thread> pThread_;
    bool &isCameraRunning_;
    double lastinputtime;

    void EnumGigeDevices();
    void PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    void OpenConfigDevices();
    void ConfigDevices(size_t index);
    void RegisterCB();
    void DoClean();
    void PressEnterToExit();
    void IMUProc(const std::deque<slamProtocol_t> &tenIMUMeasurements, vinssystem &mSystem);
    cv::Mat ImageProc(cv::Mat srcImage, double header, vinssystem* mpSystem,pair<double,vector<Box>> onebox, int queue_length = 0);

};

#endif

