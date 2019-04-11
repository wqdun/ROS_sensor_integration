#ifndef __SINGLE_PTGREY_CAMERA_H__
#define __SINGLE_PTGREY_CAMERA_H__

#include <sys/time.h>
#include <glog/logging.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "FlyCapture2.h"

class PtgreyCameraManager;
class SinglePtgreyCamera {
public:
    SinglePtgreyCamera(PtgreyCameraManager *pManager);
    ~SinglePtgreyCamera();

    bool Run();
    void InitCamera(int _index, const std::string &_rawdataDir);
    void PublishImage();
    void PublishImageFreq();


private:
    static void XferCallBack(FlyCapture2::Image *pImage, const void *_pSinglePtgreyCamera);
    static PtgreyCameraManager *s_pManager_;
    static bool ConvertImage(cv::Mat* matImage, FlyCapture2::Image* image);
    void SetImagePath();
    void SetCameraProperties();

    FlyCapture2::Error error_;
    FlyCapture2::GigECamera *pCamera_;
    cv::Mat mat2Pub_;
    std::mutex mat2PubMutex_;
    image_transport::Publisher pubImage_;
    FlyCapture2::Image inImage_;
    FlyCapture2::Image convertedImage_;
    std::string imagePath_;
    double lastImageTimeStampInSeconds;
    unsigned int imageTimeStampInSecondsTimes128;

    ros::NodeHandle nh_;
    ros::Publisher pubCamSpeed_;
    double imageFreq_;
};

#endif

