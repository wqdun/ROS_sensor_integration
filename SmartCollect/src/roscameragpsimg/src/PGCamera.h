#ifndef __PGCAMERA_H
#define __PGCAMERA_H

#include <FlyCapture2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <sys/stat.h>

#include "sc_msgs/imu5651.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/include/rapidjson/document.h"
#include "../../sc_lib_public_tools/include/rapidjson/prettywriter.h"
#include "../../sc_lib_public_tools/include/rapidjson/ostreamwrapper.h"
#include "../../sc_lib_public_tools/include/rapidjson/istreamwrapper.h"

using namespace std;
using namespace cv;
using namespace FlyCapture2;

class Cameras;

class CPGCamera
{
public:
    CPGCamera(int _index, const std::string &_rawdataDir);
    ~CPGCamera(void);

    bool ReleaseCamera();
    bool IsCameraConnected();
    bool SetCameragain(int8_t _camGain);
    bool StartCapture();
    bool StopCapture();
    void Grab();
    bool FireSoftwareTrigger();
    void DestoryDeliverImageCallback();

    bool isSaveImg_;
    double camFps_;
    FlyCapture2::Image convertedImage_;
    GigECamera *m_pCamera;
    FlyCapture2::Error error_;
    unsigned int m_nCamNum;
    FlyCapture2::PGRGuid m_guidCam;

    Cameras *pCamerasDaddy_;
    std::string imgSavePath_;
    int cameraId_;


private:
    FlyCapture2::BusManager m_busMgr;
    // calculate fps
    double lastBeginTime_;
    // update image ROS message
    int updateFreq_;
    Mat img_;
    Mat matImageDown_;
    image_transport::Publisher pub;

    static void XferCallBack(Image *pImage, const void *pCallBackData);
    void setImgSaveDir(GigECamera *pGigECamera);
    int GetIdCamera(const std::string &ipaddress);
    int getID4IP(const std::string &_ip);
    void logErrorTrace(FlyCapture2::Error error);
    double CalcFps(double nowTime);
    void setImageBuffer(GigECamera *_pGigECamera);
    void setTriggerMode(GigECamera *__pGigECamera);
    bool convertImage(cv::Mat* matImage, Image* image);
};

#endif
