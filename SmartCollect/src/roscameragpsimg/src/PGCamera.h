#pragma once

#include "FlyCapture2.h"
#include "FlyCapture2GUI.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/include/rapidjson/document.h"
#include "../../sc_lib_public_tools/include/rapidjson/prettywriter.h"
#include "../../sc_lib_public_tools/include/rapidjson/ostreamwrapper.h"
#include "../../sc_lib_public_tools/include/rapidjson/istreamwrapper.h"

#include <std_msgs/Float64.h>
#include "sc_msgs/imu5651.h"
#include <sys/stat.h>

using namespace std;
using namespace cv;

enum e_Grab_OpsType
{
    e_Grab_SetExposure,
    e_Grab_Grab,
    e_Grab_Freeze,
    e_Grab_Init
};

// camera operate information
struct  tag_GrabInfo
{
    int  nCamID;
    int  nImgID;
    double dbExposure;
    bool bFlipY;
    int nFPN;
    /*-----------------------------------------------------------*/
    bool m_bGrabFinish;
    /*-----------------------------------------------------------*/
    e_Grab_OpsType eOpsType;
    void Set(tag_GrabInfo Src);
    void Get(tag_GrabInfo &Dst);

    tag_GrabInfo()
    {
        nCamID  = 0;
        nImgID  = 0;
        dbExposure= 28;
        nFPN    = 0;
        bFlipY  = false;
        eOpsType = e_Grab_SetExposure;
        /*-----------------------------------------------------------*/
        m_bGrabFinish = false;
        /*-----------------------------------------------------------*/
    }
};

struct  tag_CamInfo
{
    double dbGain;
    int nLineRate;
    bool bNeedFilp;
    int  nSerialIndex;
    /*-------------------------------------------------------------*/
    bool bonOff;
    int nMode;
    int nParameter;
    int nSource;
    int nPolarity;
    int nROIX;
    int nROIY;
    int nROIWidth;
    int nROIHeigth;
    int nCamLink;
    int nFlip;
    int nRestore;
    /*-------------------------------------------------------------*/
    tag_CamInfo()
    {
        bNeedFilp   = false;
        nLineRate   = 6000;
        dbGain      = 1.0;
        nSerialIndex = 5;
        /*----------------------------------------*/
        bonOff      = false;
        nMode       = 0;
        nParameter  = 0;
        nSource     = 0;
        nPolarity   = 1;
        nROIX       = 0;
        nROIY       = 0;
        nROIWidth   = 2448;
        nROIHeigth  = 2048;
        nCamLink    = 2;
        nFlip       = 0;
        /*----------------------------------------*/
    }
};

using namespace FlyCapture2;

enum eCameraFormatType
{
    eMono,      // PIXEL_FORMAT_MONO8
    eRGB8,      // PIXEL_FORMAT_RGB8
    eRaw8,      // PIXEL_FORMAT_RAW8
};

enum eCameraPortType
{
    eGigE,
    e1394,
    eUSB3
};

typedef void(*ImageGrabbedCallBack)(void* , void*, void*);

class CPGCamera
{
public:
    CPGCamera(ros::NodeHandle& nh, int _index);
    ~CPGCamera(void);
    bool InitCamera(int _CameraID);
    bool NewCamera(int _CameraID);
    bool ReleaseCamera();
    bool DisConnectCamera();
    bool IsCameraConnected();

    bool IsCameraExist();
    bool CameraConnect();
    bool SetCameraParam();
    bool SetCameragain(int8_t _camGain);
    bool StartCapture();
    bool StopCapture();
    void Grab();
    bool FireSoftwareTrigger();
    void DestoryDeliverImageCallback();
    void ImageRecieve(cv::Mat matImage);
    cv::Mat GetmatImage(void);
    /*-------------------------------------------
    @function:
              convert the format of the image
    @author:
              liuyusen@navinfo.com
    @date:
              2018-01-02
    @input:
              Mat *matImage == output image point which is Mat
              Image *image  == input  image point which is Image
    @output:
              bool == tell the result of the convert function
    -------------------------------------------*/
    bool ConvertImage(cv::Mat* matImage,Image* image);
private:
    std::string imgSavePath_;
    FlyCapture2::BusManager m_busMgr;

    // calculate fps
    double lastBeginTime_;
    double camFps_;

    // update image ROS message
    int updateFreq_;



    static void XferCallBack(Image *pImage, const void *pCallBackData);
    static bool sIsSaveImg_;
    static std::string sRawdataPath_;

    Mat matImageDown_;
    Mat img_;
    FlyCapture2::Image convertedImage_;

    void setImgSaveDir(GigECamera *pGigECamera);
    int GetIdCamera(const std::string &ipaddress);
    int getID4IP(const std::string &_ip);
    void logErrorTrace(FlyCapture2::Error error);
    double CalcFps(double nowTime);


public:
    static void SetIsSaveImg(int8_t _isRecord);
    static void SetRawdataPath(const std::string &_rawdataPath);

    double TimeStamptoDouble(FlyCapture2::TimeStamp *timeStamp);
    GigECamera *m_pCamera;
    unsigned int m_CameraID;
    bool m_bLongEdge;
    FlyCapture2::Error error;
    unsigned int m_nCamNum;
    FlyCapture2::PGRGuid m_guidCam;
    FlyCapture2::TriggerMode m_triggerModeCam;
    Property m_propertyCam;

    FlyCapture2::Image m_imgRawBuffer;
    int m_nBufferWidth;
    int m_nBufferHeight;

    GigEImageSettings m_GigimageSettings;
    GigEImageSettingsInfo m_GigimageSettingInfo;
    FC2Config m_fc2Config;
    tag_CamInfo CamInfo;

    // camera parameter
    eCameraFormatType m_eFormatType;
    eCameraPortType m_ePortType;

    // capture control
    bool m_bStartedCapture;
    // set gain
    Property pProp;

    Image imgConvertOut;
    ros::NodeHandle mnh;
    // image publisher
    image_transport::Publisher  pub;
    // imu publisher
    sc_msgs::imu5651 msg_imu_string;
    ros::Publisher pub_imu_string;
    // cam speed
    std_msgs::Float64 msg_cam_speed;
    ros::Publisher pub_cam_speed;
};


