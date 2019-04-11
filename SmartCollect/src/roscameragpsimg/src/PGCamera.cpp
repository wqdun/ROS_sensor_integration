#include "PGCamera.h"
#include "camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

CPGCamera::CPGCamera(int _index, const std::string &_rawdataDir):
    img_(1200 , 1920 , CV_8UC3 , Scalar::all(0) )
{
    lastBeginTime_ = camFps_ = -1.;
    updateFreq_ = -1;
    isSaveImg_ = false;

    error_ = m_busMgr.GetCameraFromIndex(_index, &m_guidCam);
    if(error_ != PGRERROR_OK)
    {
        logErrorTrace(error_);
        exit(1);
    }

    m_pCamera = new GigECamera();
    if(!m_pCamera)
    {
        LOG(ERROR) << "Failed to new GigECamera.";
        exit(1);
    }

    error_ = m_pCamera->Connect(&m_guidCam);
    if(error_ != PGRERROR_OK)
    {
        logErrorTrace(error_);
        delete m_pCamera;
        exit(1);
    }

    imgSavePath_ = _rawdataDir;
    (void)setImgSaveDir(m_pCamera);
    (void)setImageBuffer(m_pCamera);
}

CPGCamera::~CPGCamera(void)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    delete m_pCamera;
}

void CPGCamera::setImgSaveDir(GigECamera *pGigECamera) {
    LOG(INFO) << __FUNCTION__ << " start.";

    CameraInfo camInfo;
    error_ = pGigECamera->GetCameraInfo(&camInfo);
    ostringstream ipAddress;
    ipAddress << (unsigned int)camInfo.ipAddress.octets[0] << "." << (unsigned int)camInfo.ipAddress.octets[1] << "." << (unsigned int)camInfo.ipAddress.octets[2] << "." << (unsigned int)camInfo.ipAddress.octets[3];
    LOG(INFO) << "IP address - " << ipAddress.str();

    cameraId_ = GetIdCamera(ipAddress.str() );
    if(0 == cameraId_)
    {
        imgSavePath_ += "/Image/";
    }
    else
    {
        imgSavePath_ += "/Image/panoramas/";
    }

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    if(0 == cameraId_) {
        pub = it.advertise("camera/image", 0);
    }
    else {
        pub = it.advertise("camera/image" + std::to_string(cameraId_), 0);
    }

    LOG(INFO) << "imgSavePath_: " << imgSavePath_;
    (void)mkdir(imgSavePath_.c_str(), S_IRWXO);
}

void CPGCamera::setImageBuffer(GigECamera *_pGigECamera) {
    LOG(INFO) << __FUNCTION__ << " start.";

    FC2Config m_fc2Config;
    _pGigECamera->GetConfiguration(&m_fc2Config);
    // Buffer: 200 images
    m_fc2Config.numBuffers = 200;
    m_fc2Config.highPerformanceRetrieveBuffer = true;
    m_fc2Config.grabMode = BUFFER_FRAMES;
    m_fc2Config.grabTimeout = TIMEOUT_INFINITE;
    error_ = _pGigECamera->SetConfiguration(&m_fc2Config);
    if(error_ != PGRERROR_OK)
    {
        logErrorTrace(error_);
        delete _pGigECamera;
        exit(9);
    }
}

void CPGCamera::setTriggerMode(GigECamera *__pGigECamera) {
    LOG(INFO) << __FUNCTION__ << " start.";

    // Check for external trigger support
    TriggerModeInfo triggerModeInfo;
    error_ = __pGigECamera->GetTriggerModeInfo( &triggerModeInfo );
    if (error_ != PGRERROR_OK)
    {
        logErrorTrace( error_ );
        exit(10);
    }

    if(!triggerModeInfo.present)
    {
        LOG(ERROR) << "Camera does not support external trigger! Exiting...";
        exit(11);
    }

    TriggerMode triggerMode;
    error_ = __pGigECamera->GetTriggerMode( &triggerMode );
    if (error_ != PGRERROR_OK)
    {
        logErrorTrace( error_ );
        exit(12);
    }

    triggerMode.onOff = false;
    triggerMode.mode = 0;
    triggerMode.parameter = 0;
    // Triggering the camera externally using source 0.
    triggerMode.source = 0;
    error_ = __pGigECamera->SetTriggerMode( &triggerMode );
    if (error_ != PGRERROR_OK)
    {
        logErrorTrace( error_ );
        exit(13);
    }
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

bool CPGCamera::SetCameragain(int8_t _camGain)
{
    LOG(INFO) << __FUNCTION__ << " start.";

    Property gainProperty;
    gainProperty.type = GAIN;
    error_ = m_pCamera->GetProperty(&gainProperty);
    if(error_ != PGRERROR_OK)
    {
        LOG(INFO) << "Failed to GetProperty.";
        logErrorTrace(error_);
        return false;
    }
    LOG(INFO) << "gainProperty: " << gainProperty.absValue;

    gainProperty.type = GAIN;
    gainProperty.absControl = true;
    gainProperty.onePush = false;
    gainProperty.onOff = true;
    gainProperty.autoManualMode = false;
    gainProperty.absValue = (float)_camGain;
    error_ = m_pCamera->SetProperty(&gainProperty);
    if (error_ != PGRERROR_OK)
    {
        LOG(WARNING) << "gainProperty.absValue: " << gainProperty.absValue;
        LOG(WARNING) << "Failed to SetProperty.";
        logErrorTrace(error_);
        return false;
    }
    LOG(INFO) << __FUNCTION__ << " end.";
    return true;
}

void CPGCamera::logErrorTrace(FlyCapture2::Error _error) {
    LOG(ERROR) << _error.GetFilename() << ":" << _error.GetLine() << ":" << _error.GetDescription();
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

    error_ = m_pCamera->StartCapture(XferCallBack, this);
    if (error_ != PGRERROR_OK)
    {
        logErrorTrace(error_);
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

    error_ = m_pCamera->StopCapture();
    if (error_ != PGRERROR_OK)
    {
        LOG(WARNING) << "Failed to StopCapture, cameraId_: " << cameraId_;
        logErrorTrace(error_);
        return false;
    }
    return true;
}

double CPGCamera::CalcFps(double nowTime) {
    LOG(INFO) << __FUNCTION__ << " start.";

    if(lastBeginTime_ < 0) {
        LOG(INFO) << "1st time grab image, no calculate fps, pPGCamera->lastBeginTime_: " << lastBeginTime_;
        lastBeginTime_ = nowTime;
        return -1;
    }

    double camFps = 1.0 / (nowTime - lastBeginTime_);
    lastBeginTime_ = nowTime;
    return camFps;
}

void CPGCamera::XferCallBack(Image *pImage, const void *_pPGCamera)
{
    CPGCamera *pPGCamera = (CPGCamera*)_pPGCamera;
    LOG(INFO) << __FUNCTION__ << " start with cameraId_: " << pPGCamera->cameraId_;

    double gpsWeekTimeCorrected = pPGCamera->pCamerasDaddy_->gpsWeekTimeCorrected_;
    bool isGpsTimeValid = pPGCamera->pCamerasDaddy_->isGpsTimeValid_;

    LOG(INFO) << "gpsWeekTimeCorrected: " << std::fixed << gpsWeekTimeCorrected;
    gpsWeekTimeCorrected = fmod(gpsWeekTimeCorrected, 3600 * 24);

    double beginTime = ros::Time::now().toSec();
    pPGCamera->camFps_ = pPGCamera->CalcFps(beginTime);
    DLOG(INFO) << "camFps_: " << pPGCamera->camFps_;

    std::string jpgFile(pPGCamera->imgSavePath_);
    if(isGpsTimeValid) {
        if(0 == pPGCamera->cameraId_) {
            jpgFile += (std::to_string(gpsWeekTimeCorrected) + ".jpg");
        }
        else {
            jpgFile += (std::to_string(gpsWeekTimeCorrected) + "-" + std::to_string(pPGCamera->cameraId_) + ".jpg");
        }
    }
    else {
        jpgFile += ("sys" + std::to_string(gpsWeekTimeCorrected) + "-" + std::to_string(pPGCamera->cameraId_) + ".jpg");
    }

    FlyCapture2::Error flyError;
    flyError = pImage->Convert(PIXEL_FORMAT_BGR, &(pPGCamera->convertedImage_) );
    if(flyError != PGRERROR_OK)
    {
        LOG(WARNING) << "Failed to pImage->Convert " << jpgFile;
        pPGCamera->logErrorTrace(flyError);
        return;
    }

    if(!(pPGCamera->isSaveImg_) ) {
        LOG(INFO) << "No save " << jpgFile;
    }
    else {
        LOG(INFO) << "I am gonna save " << jpgFile;
        FlyCapture2::JPEGOption jpgoption;
        flyError = pPGCamera->convertedImage_.Save(jpgFile.c_str(), &jpgoption);
        if(flyError != PGRERROR_OK) {
            LOG(WARNING) << "Failed to save " << jpgFile;
            pPGCamera->logErrorTrace(flyError);
            return;
        }
    }

    ++(pPGCamera->updateFreq_);
    if(0 == pPGCamera->cameraId_) {
        pPGCamera->updateFreq_ %= 8;
    }
    else {
        pPGCamera->updateFreq_ %= 2;
    }

    if(0 == pPGCamera->updateFreq_) {
        LOG(INFO) << "Gonna publish " << jpgFile;
        bool isConvert2Mat = pPGCamera->convertImage(&(pPGCamera->img_), &(pPGCamera->convertedImage_) );
        if(!isConvert2Mat) {
            LOG(ERROR) << "Failed to convertImage " << jpgFile;
            return;
        }
        Mat matImageDown;
        cv::resize(pPGCamera->img_, matImageDown, cv::Size(1920 / 5, 1200 / 5));
        sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matImageDown).toImageMsg();
        if(NULL == msg_img)
        {
            LOG(WARNING) << "Failed to transfer to sensor_msgs::Image, msg_img is null.";
        }
        else
        {
            pPGCamera->pub.publish(msg_img);
        }
    }

    LOG(INFO) << __FUNCTION__ << " end with cameraId_: " << pPGCamera->cameraId_;
    return;
}

// @function:
//           convert the format of the image
// @author:
//           liuyusen@navinfo.com
// @date:
//           2018-01-02
// @input:
//           Mat *matImage == output image point which is Mat
//           Image *image  == input  image point which is Image
// @output:
//           bool == tell the result of the convert function
bool CPGCamera::convertImage(cv::Mat* matImage, Image* image)
{
    int rows = matImage->rows;
    int cols = matImage->cols;
    int step = matImage->step;
    uchar* dst_data = (uchar*)matImage->data;

    int height = image->GetRows();
    int width  = image->GetCols();
    int stride = image->GetStride();
    uchar* src_data = (uchar*)image->GetData();

    if(height != rows || width != cols)
    {
        LOG(WARNING) << "height(" << height << ") != rows(" << rows << "); " << "width(" << width << ") != cols(" << cols << ").";
        return false;
    }
    int cols_3 = 3 * cols;
    for(int i = 0; i < rows; ++i)
    {
        int i_step = i * step;
        int i_stride = i * stride;
        for(int j = 0 ; j < cols_3; j += 3)
        {
            dst_data[i_step + j] = src_data[i_stride + j];
            dst_data[i_step + j + 1] = src_data[i_stride + j + 1];
            dst_data[i_step + j + 2] = src_data[i_stride + j + 2];
        }
    }
    return true;
}


