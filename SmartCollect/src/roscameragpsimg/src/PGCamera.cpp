#include "PGCamera.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <FlyCapture2.h>
#include "lock.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

CMutex mymutex;
//time control
extern vector<string> parsed_data;
extern string global_gps ;

//set gain
extern float cam_gain;


CPGCamera::CPGCamera(ros::NodeHandle& nh, int _index):
    img_(1200 , 1920 , CV_8UC3 , Scalar::all(0) )
{
    lastBeginTime_ = camFps_ = -1.;
    updateFreq_ = -1;

    mnh = nh;
    image_transport::ImageTransport it(mnh);
    pub = it.advertise("camera/image", 1);

    pub_imu_string = mnh.advertise<sc_msgs::imu5651>("imu_string", 1);
    pub_cam_speed  = mnh.advertise<std_msgs::Float64>("cam_speed", 1);

    m_pCamera = NULL;

    m_CameraID = 0;
    m_bStartedCapture = false;

    (void)InitCamera(_index);
}

CPGCamera::~CPGCamera(void)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    delete m_pCamera;
}

bool CPGCamera::InitCamera(int _CameraIndex)
{
    LOG(INFO) << __FUNCTION__ << " start.";

    error = m_busMgr.GetCameraFromIndex(_CameraIndex, &m_guidCam);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        exit(1);
    }

    m_pCamera = new GigECamera();
    if (!m_pCamera)
    {
        LOG(WARNING) << "Failed to new GigECamera.";
        exit(1);
    }

    error = m_pCamera->Connect(&m_guidCam);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        delete m_pCamera;
        exit(1);
    }

    (void)setImgSaveDir(m_pCamera);

    // error = m_pCamera->RestoreFromMemoryChannel(1);
    // if ( error != PGRERROR_OK )
    // {
    //     logErrorTrace(error);
    //     exit(1);
    // }

    return true;
}



void CPGCamera::setImgSaveDir(GigECamera *pGigECamera) {
    LOG(INFO) << __FUNCTION__ << " start.";

    CameraInfo camInfo;
    error = pGigECamera->GetCameraInfo(&camInfo);
    ostringstream ipAddress;
    ipAddress << (unsigned int)camInfo.ipAddress.octets[0] << "." << (unsigned int)camInfo.ipAddress.octets[1] << "." << (unsigned int)camInfo.ipAddress.octets[2] << "." << (unsigned int)camInfo.ipAddress.octets[3];
    LOG(INFO) << "IP address - " << ipAddress.str();

    int cameraId = GetIdCamera(ipAddress.str() );

    imgSavePath_ = sRawdataPath_ + "/camera"+ to_string(cameraId) + "/";
    (void)mkdir(imgSavePath_.c_str(), S_IRWXU);
}

int CPGCamera::GetIdCamera(const std::string &ipaddress)
{
    using namespace rapidjson;
    const std::string configFile("/opt/smartc/config/camera_config.json");
    if(0 != access(configFile.c_str(), 0) ) {
        LOG(ERROR) << configFile << " does not exist.";
        return -1;
    }

    ifstream ifs(configFile);
    IStreamWrapper isw(ifs);
    Document doc;
    doc.ParseStream(isw);
    if(doc.HasParseError() ) {
        LOG(ERROR) << "Failed to parse " << configFile << ", GetParseError: " << doc.GetParseError();
        return -1;
    }
    if(!doc.HasMember(ipaddress.c_str()))
    {
        LOG(ERROR) << "I can't find " << ipaddress << " in " << configFile;
        exit(1);
    }
    const int cameraId = doc[ipaddress.c_str()].GetInt();
    LOG(INFO) << ipaddress << "'s ID: " << cameraId;

    return cameraId;
}


bool CPGCamera::IsCameraExist()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    CameraInfo camInfo[128];
    unsigned int numCamInfo = 0;

    for(size_t i = 0; i < 10; ++i)
    {
        numCamInfo = 128;
        error = BusManager::DiscoverGigECameras(camInfo, &numCamInfo);
        LOG(INFO) << "I discover " << numCamInfo << " camera.";
        if(error != PGRERROR_OK)
        {
            LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
            continue;
        }

        if(numCamInfo >= 1)
        {
            LOG(INFO) << "I discover camera successfully.";
            break;
        }

        error = m_busMgr.RescanBus();
        if(error != PGRERROR_OK)
        {
            LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        }
    }
    if(numCamInfo < 1)
    {
        LOG(ERROR) << "Failed to discover camera, num: " << numCamInfo;
        exit(-1);
    }

    // for(size_t i = 0; i < 10; ++i)
    // {
    //     error = m_busMgr.GetNumOfCameras(&m_nCamNum);
    //     LOG(INFO) << "I GetNumOfCameras: " << m_nCamNum << " cameras.";
    //     if(error != PGRERROR_OK)
    //     {
    //         LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
    //         continue;
    //     }

    //     if(m_nCamNum >= 1)
    //     {
    //         LOG(INFO) << "I GetNumOfCameras successfully.";
    //         break;
    //     }

    //     error = m_busMgr.RescanBus();
    //     if(error != PGRERROR_OK)
    //     {
    //         LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
    //     }
    // }
    // if(m_nCamNum < 1)
    // {
    //     LOG(ERROR) << "Failed to GetNumOfCameras, num: " << m_nCamNum;
    //     exit(-1);
    // }

    return true;
}

bool CPGCamera::CameraConnect()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        LOG(WARNING) << "m_pCamera is NULL.";
        return false;
    }
    error = m_pCamera->Connect(&m_guidCam);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }

    error = m_pCamera->RestoreFromMemoryChannel(1);
    if ( error != PGRERROR_OK )
    {
        logErrorTrace(error);
        return false;
    }

    return true;
}

void CPGCamera::SetIsSaveImg(int8_t _isRecord)
{
    LOG(INFO) << __FUNCTION__ << " start, _isRecord: " << (int32_t)_isRecord;
    CPGCamera::sIsSaveImg_ = _isRecord;
}

void CPGCamera::SetRawdataPath(const std::string &_rawdataPath) {
    LOG(INFO) << __FUNCTION__ << " start, _rawdataPath: " << _rawdataPath;
    CPGCamera::sRawdataPath_ = _rawdataPath;
}


bool CPGCamera::SetCameragain(int8_t _camGain)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    error = ((GigECamera *)m_pCamera)->GetProperty(&pProp);
    if(error != PGRERROR_OK)
    {
        LOG(INFO) << "SetCameragain: GetProperty.";
        logErrorTrace(error);
    }
    pProp.type = GAIN;
    pProp.absControl = true;
    pProp.onePush = false;
    pProp.onOff = true;
    pProp.autoManualMode = false;
    pProp.absValue = (float)_camGain;
    error = ((GigECamera *)m_pCamera)->SetProperty(&pProp );
    if (error != PGRERROR_OK)
    {
        LOG(INFO) << "pProp.absValue: " << pProp.absValue;
        LOG(INFO) << "SetCameragain: SetProperty.";
        logErrorTrace(error);
    }
    LOG(INFO) << __FUNCTION__ << " end.";
    return true;
}

bool CPGCamera::SetCameraParam()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    LOG(INFO) << "Set camera parameter.";
    error = ((GigECamera *)m_pCamera)->GetGigEImageSettings(&m_GigimageSettings);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }
    m_GigimageSettings.pixelFormat = PIXEL_FORMAT_MONO8;
    m_GigimageSettings.offsetX = 0;
    m_GigimageSettings.offsetY = 0;
    m_GigimageSettings.width = 2448;
    m_GigimageSettings.height = 2048;

    error = m_pCamera->SetGigEImageSettings(&m_GigimageSettings);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }

    LOG(INFO) << "Set camera trigger mode: external.";
    error = m_pCamera->GetTriggerMode(&m_triggerModeCam);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }
    m_triggerModeCam.onOff = true;
    m_triggerModeCam.mode = 1;
    m_triggerModeCam.parameter = 0;
    m_triggerModeCam.source = 0;
    error = m_pCamera->SetTriggerMode(&m_triggerModeCam);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }
    return true;
}

bool CPGCamera::DisConnectCamera()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }
    error = m_pCamera->Disconnect();
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }
    return true;
}

bool CPGCamera::ReleaseCamera()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    bool bRelease = true;
    bool bConnect = true;

    bConnect = m_pCamera->IsConnected();
    if (bConnect == true)
    {
        StopCapture();
        DisConnectCamera();
    }
    delete m_pCamera;
    m_pCamera = NULL;

    return true;
}

void CPGCamera::logErrorTrace(FlyCapture2::Error error) {
    LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
}

bool CPGCamera::IsCameraConnected()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera)
    {
        return m_pCamera->IsConnected();
    }

    return false;
}

bool CPGCamera::StartCapture()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    error = m_pCamera->StartCapture(XferCallBack, this);
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }

    return true;
}

bool CPGCamera::StopCapture()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    error = m_pCamera->StopCapture();
    if (error != PGRERROR_OK)
    {
        logErrorTrace(error);
        return false;
    }
    return true;
}

double CPGCamera::CalcFps(double nowTime) {
    LOG(INFO) << __FUNCTION__ << " start.";

    if(lastBeginTime_ < 0) {
        LOG(INFO) << "1st time grab image, no calculate fps, pPGCamera->lastBeginTime_： " << lastBeginTime_;
        lastBeginTime_ = nowTime;
        return -1;
    }

    double camFps = 1.0 / (nowTime - lastBeginTime_);
    lastBeginTime_ = nowTime;
    return camFps;
}

void CPGCamera::XferCallBack(Image *pImage, const void *_pPGCamera)
{
    LOG(INFO) << __FUNCTION__ << " start with address: " << _pPGCamera;
    CPGCamera *pPGCamera = (CPGCamera*)_pPGCamera;

    FlyCapture2::Error _error;

    int bits = pImage->GetBitsPerPixel();
    LOG_FIRST_N(INFO, 20) << "BitsPerPixel: " << bits;

    double beginTime = ros::Time::now().toSec();
    pPGCamera->camFps_ = pPGCamera->CalcFps(beginTime);
    LOG(INFO) << "camFps_: " << pPGCamera->camFps_;

    ++(pPGCamera->updateFreq_);
    (pPGCamera->updateFreq_) %= 2;

    string seconds_str = std::to_string(beginTime);

    //time of gps
    string gps_str = "--";

    int ret = mymutex.Lock();
    DLOG(INFO) << "trylock result: " << ret;
    if(ret != 0)
    {
        DLOG(WARNING) << "Failed to trylock, result: " << ret;
        return ;
    }

    bool isGpsTime = false;
    if(parsed_data.size() >= 17 && global_gps.size() > 2 && global_gps.size() < 20)
    {
        gps_str = global_gps;
        isGpsTime = true;
    }
    else
    {
        LOG(WARNING) << "GPSweek time error: " << global_gps << "; using system time: " << seconds_str;
        gps_str = seconds_str;
        isGpsTime = false;
    }
    int retUnlock = mymutex.Unlock();
    DLOG(INFO) << "unlock result: " << retUnlock;

    double gps_str_db = 0.0;
    stringstream ss;
    ss << gps_str;
    ss >> gps_str_db;
    gps_str_db = fmod(gps_str_db, 3600 * 24);
    gps_str = std::to_string(gps_str_db);
    gps_str = isGpsTime? gps_str: "sys" + gps_str;
    DLOG(INFO) << "time_str: " << gps_str;
    int format_int = 0;
    switch(format_int)
    {
        // jpg
        case 0:
        {
            string gps_str_jpg(pPGCamera->imgSavePath_ + gps_str + ".jpg");
            LOG(INFO) << "I am gonna save in " << gps_str_jpg;
            const char* pathout_gps_jpg_c = gps_str_jpg.c_str();

            _error = pImage->Convert(PIXEL_FORMAT_BGR, &(pPGCamera->convertedImage_) );
            if(_error != PGRERROR_OK)
            {
                LOG(WARNING) << "pImage->Convert error.";
                break;
            }

            if(sIsSaveImg_)
            {
                FlyCapture2::JPEGOption jpgoption;
                _error = pPGCamera->convertedImage_.Save(pathout_gps_jpg_c, &jpgoption);
                if(_error != PGRERROR_OK)
                {
                    LOG(WARNING) << "Save image jpg error.";
                    break;
                }
            }
            if(0 != pPGCamera->updateFreq_)
            {
                break;
            }

            bool isConverted = pPGCamera->ConvertImage(&(pPGCamera->img_), &(pPGCamera->convertedImage_) );
            cv::resize(pPGCamera->img_, pPGCamera->matImageDown_, cv::Size(1920 / 5, 1200 / 5));
            LOG(INFO) << "Publish a image, matConvert state: " << isConverted;
            if(isConverted)
            {
                sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pPGCamera->matImageDown_).toImageMsg();
                if(NULL == msg_img)
                {
                    LOG(WARNING) << "Failed to transfer to sensor_msgs::Image, msg_img is null.";
                    break;
                }
                pPGCamera->pub.publish(msg_img);
            }
            break;
        }
        default:
        {
            LOG(WARNING)<<"the format is not surported!";
            break;
        }
    }

    pPGCamera->msg_cam_speed.data = pPGCamera->camFps_;
    pPGCamera->pub_cam_speed.publish(pPGCamera->msg_cam_speed);

    LOG(INFO) << __FUNCTION__ << " end with " << " address: " << _pPGCamera;
    return;
}

double CPGCamera::TimeStamptoDouble(FlyCapture2::TimeStamp *timeStamp)
{
    return (double)timeStamp->cycleSeconds + (((double)timeStamp->cycleCount+((double)timeStamp->cycleOffset/3072.0))/8000.0);
}

void CPGCamera::ImageRecieve(cv::Mat matImage)
{

}

cv::Mat CPGCamera::GetmatImage(void)
{

}

bool CPGCamera::ConvertImage(cv::Mat* matImage, Image* image)
{
    int rows = matImage->rows;
    int cols = matImage->cols;

    int step = matImage->step;
    uchar* dst_data = (uchar*)matImage->data;

    int height = image->GetRows();
    int width  = image->GetCols();

    int bits  = image->GetBitsPerPixel();
    int stride = image->GetStride();

    uchar* src_data = (uchar*)image->GetData();

    if(height!=rows || width != cols)
    {
        return false;
    }
    for(int i = 0 ; i < rows; i++)
    {
        for(int j = 0 ; j < 3*cols ; j=j+3)
        {
            dst_data[i*step+j]=src_data[i*stride+j];
            dst_data[i*step+j+1]=src_data[i*stride+j+1];
            dst_data[i*step+j+2]=src_data[i*stride+j+2];
        }
    }
    return true;
}
