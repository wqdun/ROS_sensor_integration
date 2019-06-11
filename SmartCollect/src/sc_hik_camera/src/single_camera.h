#ifndef __SINGLE_CAMERA_H__
#define __SINGLE_CAMERA_H__

#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <sys/time.h>
#include <cv_bridge/cv_bridge.h>
#include "include/MvCameraControl.h"

typedef union {
    uint32_t uint32Data[2];
    uint64_t uint64Data;
} uint32TOuint64_t;

class HikCameraManager;
class SingleCamera {
public:
    SingleCamera(HikCameraManager *pManager);
    ~SingleCamera();
    void SetCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    std::string GetCameraIP();
    int GetCameraID();
    void *GetHandle();
    void PublishImage();
    void AdaptNightMode();
    void CheckAndRestartCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void LogDeviceStatus();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static HikCameraManager *s_pManager_;

    enum FrameSpecInfoSelector {
        Timestamp = 0,
        Gain,
        Exposure,
        BrightnessInfo,
        WhiteBalance,
        Framecounter,
        ExtTriggerCount,
        LineInputOutput,
        ROIPosition
    };

    void SetHandle(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void SetIndex_Ip(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void SetImagePath();
    bool StartCamera();
    void ConfigDevices();
    size_t GetCameraIndex();
    void SetAdvertiseTopic();
    void PublishCamFps();
    void EnterNightMode();
    void ExitNightMode();
    void TurnOnFrameSpecInfo(FrameSpecInfoSelector frameSpecInfoSelector);
    bool RestartCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);

    void *cameraHandle_;
    // e.g., "6666"
    std::string cameraIP_;
    int cameraID_;
    size_t cameraIndex_;
    std::string imagePath_;
    std::mutex mat2PubMutex_;
    double lastDeviceTimeStamp_;
    ros::NodeHandle nh_;
    image_transport::Publisher pubImage_;
    cv::Mat mat2Pub_;
    ros::Publisher pubCamSpeed_;
    double imageFreq_;
    bool isNightMode_;
    double currentImageExposureTime_;
    bool isCallbackOK_;
    int cameraStartupCounter_;
};

#endif

