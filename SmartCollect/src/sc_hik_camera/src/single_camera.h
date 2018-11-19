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

class HikCameraManager;
class SingleCamera {
public:
    SingleCamera(HikCameraManager *pManager);
    ~SingleCamera();
    void SetCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    std::string GetCameraIP();
    void *GetHandle();
    void PublishImageAndFreq();


private:
    static void __stdcall ImageCB(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static HikCameraManager *s_pManager_;

    void SetHandle(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void SetIndex_Ip(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index);
    void StartCamera();
    void ConfigDevices();
    size_t GetCameraIndex();
    void SetAdvertiseTopic();

    void *cameraHandle_;
    // e.g., "6666"
    std::string cameraIP_;
    size_t cameraIndex_;
    std::mutex mat2PubMutex_;
    double lastDeviceTimeStamp_;
    ros::NodeHandle nh_;
    image_transport::Publisher pubImage_;
    cv::Mat mat2Pub_;
    ros::Publisher pubCamSpeed_;
    double imageFreq_;
};

#endif

