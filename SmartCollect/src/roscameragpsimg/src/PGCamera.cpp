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
    isSaveImg_ = true;

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
        imgSavePath_ += "/Image/master/";
    }
    else
    {
        imgSavePath_ += "/Image/";
    }
    LOG(INFO) << "imgSavePath_: " << imgSavePath_;
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

bool CPGCamera::SetCameragain(int8_t _camGain)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    error_ = ((GigECamera *)m_pCamera)->GetProperty(&pProp);
    if(error_ != PGRERROR_OK)
    {
        LOG(INFO) << "SetCameragain: GetProperty.";
        logErrorTrace(error_);
    }
    pProp.type = GAIN;
    pProp.absControl = true;
    pProp.onePush = false;
    pProp.onOff = true;
    pProp.autoManualMode = false;
    pProp.absValue = (float)_camGain;
    error_ = ((GigECamera *)m_pCamera)->SetProperty(&pProp );
    if (error_ != PGRERROR_OK)
    {
        LOG(INFO) << "pProp.absValue: " << pProp.absValue;
        LOG(INFO) << "SetCameragain: SetProperty.";
        logErrorTrace(error_);
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
        logErrorTrace(error_);
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

    double gpsWeekTimeCorrected = pPGCamera->pCamerasDaddy_->gpsWeekTimeCorrected_;
    bool isGpsTimeValid = pPGCamera->pCamerasDaddy_->isGpsTimeValid_;

    LOG(INFO) << "gpsWeekTimeCorrected: " << std::fixed << gpsWeekTimeCorrected;
    gpsWeekTimeCorrected = fmod(gpsWeekTimeCorrected, 3600 * 24);

    double beginTime = ros::Time::now().toSec();
    pPGCamera->camFps_ = pPGCamera->CalcFps(beginTime);
    DLOG(INFO) << "camFps_: " << pPGCamera->camFps_;

    std::string jpgFile(pPGCamera->imgSavePath_);
    if(isGpsTimeValid) {
        jpgFile += (std::to_string(gpsWeekTimeCorrected) + "-" + std::to_string(pPGCamera->cameraId_) + ".jpg");
    }
    else {
        jpgFile += ("sys" + std::to_string(gpsWeekTimeCorrected) + "-" + std::to_string(pPGCamera->cameraId_) + ".jpg");
    }

    FlyCapture2::Error flyError = pImage->Convert(PIXEL_FORMAT_BGR, &(pPGCamera->convertedImage_) );
    if(flyError != PGRERROR_OK)
    {
        LOG(WARNING) << "Failed to pImage->Convert " << jpgFile;
        return;
    }

    if(!(pPGCamera->isSaveImg_) )
    {
        LOG(INFO) << "No save " << jpgFile;
        return;
    }
    LOG(INFO) << "I am gonna save " << jpgFile;
    flyError = pPGCamera->convertedImage_.Save(jpgFile.c_str(), &(pPGCamera->jpgoption_) );
    if(flyError != PGRERROR_OK)
    {
        LOG(WARNING) << "Failed to save " << jpgFile;
        pPGCamera->logErrorTrace(flyError);
        return;
    }

    LOG(INFO) << __FUNCTION__ << " end with " << " address: " << _pPGCamera;
    return;
}


