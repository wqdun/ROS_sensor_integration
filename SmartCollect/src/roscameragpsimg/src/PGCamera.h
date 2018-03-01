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
#include<ros/ros.h>
#include<image_transport/image_transport.h>
//#define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

#include <std_msgs/Float64.h>
#include "roscameragpsimg/imu5651.h"

using namespace std;
using namespace cv;



enum e_Grab_OpsType
{
	e_Grab_SetExposure,
	e_Grab_Grab,
	e_Grab_Freeze,
	e_Grab_Init
};

//相机操作信息
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
		nCamID	= 0;
		nImgID	= 0;
		dbExposure= 28;
		nFPN	= 0;
		bFlipY	= false;
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
	int	 nSerialIndex;
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
		bNeedFilp	= false;
		nLineRate	= 6000;
		dbGain		= 1.0;
		nSerialIndex = 5;
		/*----------------------------------------*/
		bonOff		= false;
		nMode		= 0;
		nParameter	= 0;
		nSource		= 0;
		nPolarity	= 1;
		nROIX		= 0;
		nROIY		= 0;
		nROIWidth	= 2448;
		nROIHeigth	= 2048;
		nCamLink	= 2;
		nFlip		= 0;
		/*----------------------------------------*/
	}
};
using namespace FlyCapture2;

//////////////////////////////////////////////////////////////////////////////


enum eCameraFormatType
{
	eMono,		// PIXEL_FORMAT_MONO8
	eRGB8,		// PIXEL_FORMAT_RGB8
	eRaw8,		// PIXEL_FORMAT_RAW8
};

//
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
	CPGCamera(int nBufWidth, int nBufHeight, ros::NodeHandle& mnh);
	~CPGCamera(void);
	//init camera
	bool InitCamera(int m_CameraID);
	//new camera
	bool NewCamera(int m_CameraID);
	//releasecamera
	bool ReleaseCamera();
	//diconnectcamera
	bool DisConnectCamera();
	//iscameraConnected
	bool IsCameraConnected();

	//IsCameraExist
	bool IsCameraExist();
	//CameraConnect
	bool CameraConnect();
	//SetCameraParam
	bool SetCameraParam();
	//SetCameraGain
	bool SetCameragain();
	//StartCapture
	bool StartCapture();
	//StopCapture
	bool StopCapture();
	//Grab
	void Grab();
    //FireSoftwareTrigger
	bool FireSoftwareTrigger();
	//RegisterDeliverImageCallback
	void RegisterDeliverImageCallback(ImageGrabbedCallBack _callback, void *_Owner, void *_Control);
	void DestoryDeliverImageCallback();
    //ImageRecieve
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
	static void XferCallBack(Image* pImage, const void *pCallBackData);

public:
    double TimeStamptoDouble(FlyCapture2::TimeStamp *timeStamp);
	GigECamera *m_pCamera;
	unsigned int m_CameraID;		//相机ID号
	bool m_bLongEdge;				//TRUE:长边  FALSE:短边

	FlyCapture2::Error error;					//错误信息
	FlyCapture2::BusManager m_busMgr;			//总线信息管理
	unsigned int m_nCamNum;			//相机个数
	FlyCapture2::PGRGuid m_guidCam;				//相机GUID标识
	FlyCapture2::TriggerMode m_triggerModeCam;	//相机触发模式类
	Property m_propertyCam;			//相机参数设置类
	FlyCapture2::Image m_imgRawBuffer;			//原始采集图像Buffer


	//图像Buffer大小
	int m_nBufferWidth;
	int m_nBufferHeight;

	//GigE接口设置
	GigEImageSettings	m_GigimageSettings;
	GigEImageSettingsInfo m_GigimageSettingInfo;
	FC2Config			m_fc2Config;
	tag_CamInfo CamInfo;
	//相机参数
	eCameraFormatType m_eFormatType;
	eCameraPortType m_ePortType;

	ImageGrabbedCallBack m_GrabImgCallBack;

	//捕获控制
	bool m_bStartedCapture;
	//setgain
	Property pProp;

  //图像Image;
  Image imgConvertOut;
  //Mat imgs;
  //nh
  ros::NodeHandle mnh;
  //image publisher
  image_transport::Publisher  pub;
  //imu publisher
  roscameragpsimg::imu5651 msg_imu_string;
  ros::Publisher pub_imu_string;
  //cam speed
  std_msgs::Float64 msg_cam_speed;
  ros::Publisher    pub_cam_speed;

};

