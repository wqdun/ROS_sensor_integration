#ifndef __SINGLE_CAMERA_H__
#define __SINGLE_CAMERA_H__

#include <string>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <deque>
#include <sys/time.h>
#include "include/MvCameraControl.h"

typedef struct {
    double header;
    size_t cameraIndex;
    std::string cameraIP;
    size_t frameNum;
    cv::Mat matImage;
    unsigned char *pDataImage;
} time2Mat_t;

class SingleCamera {
public:
    static std::mutex s_matImageMutex_;
    static std::deque<time2Mat_t> s_time2Mat_;

    SingleCamera();
    ~SingleCamera();

    void SetCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    std::string GetCameraIP();
    void *GetHandle();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

    void SetHandle(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void SetIndex_Ip(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void StartCamera();
    void ConfigDevices();
    size_t GetCameraIndex();

    void *cameraHandle_;
    // e.g., "6666"
    std::string cameraIP_;
    size_t cameraIndex_;
};

#endif

