#include "single_camera.h"
#include "hik_camera_manager.h"

HikCameraManager* SingleCamera::s_pManager_;

SingleCamera::SingleCamera(HikCameraManager *pManager):
    mat2Pub_(1200, 1920, CV_8UC3, cv::Scalar::all(0) ),
    cameraHandle_(NULL),
    cameraStartupCounter_(-1),
    isCallbackOK_(false)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    s_pManager_ = pManager;
    imageFreq_ = 0;
    isNightMode_ = false;
    currentImageExposureTime_ = 0;
}

SingleCamera::~SingleCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    err = MV_CC_StopGrabbing(cameraHandle_);
    if(MV_OK != err) {
        LOG(ERROR) << "MV_CC_StopGrabbing failed, err: " << err;
        exit(1);
    }

    err = MV_CC_CloseDevice(cameraHandle_);
    if(MV_OK != err) {
        LOG(ERROR) << "MV_CC_CloseDevice failed, err: " << err;
        exit(1);
    }

    err = MV_CC_DestroyHandle(cameraHandle_);
    if(MV_OK != err) {
        LOG(ERROR) << "MV_CC_DestroyHandle failed, err: " << err;
        exit(1);
    }
}

void SingleCamera::SetCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";

    SetIndex_Ip(deviceInfoList, index);
    SetHandle(deviceInfoList, index);
    SetAdvertiseTopic();
    SetImagePath();

    (void)StartCamera();
}

void SingleCamera::SetImagePath() {
    LOG(INFO) << __FUNCTION__ << " start.";
    if("6666" == cameraIP_) {
        imagePath_ = (s_pManager_->rawDataPath_) + "/Image/";
        LOG(INFO) << "Save camera " << cameraIP_ << " in " << imagePath_;
    }
    else {
        imagePath_ = (s_pManager_->rawDataPath_) + "/Image/panoramas/";
        LOG(INFO) << "Save camera " << cameraIP_ << " in " << imagePath_;
    }
}

void SingleCamera::SetAdvertiseTopic() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pubCamSpeed_ = nh_.advertise<std_msgs::Float64>("cam_speed" + cameraIP_, 10);

    image_transport::ImageTransport it(nh_);
    pubImage_ = it.advertise("camera/image" + cameraIP_, 10);

    return;
}

void SingleCamera::PublishImage() {
    DLOG(INFO) << __FUNCTION__ << " start.";

    cv::Mat imageResized;
    mat2PubMutex_.lock();
    cv::resize(mat2Pub_, imageResized, cv::Size(mat2Pub_.cols / 10, mat2Pub_.rows / 10));
    mat2PubMutex_.unlock();
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageResized).toImageMsg();
    pubImage_.publish(imgMsg);

    return;
}

void SingleCamera::SetIndex_Ip(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    cameraIndex_ = index;

    MV_CC_DEVICE_INFO* pDeviceInfo = deviceInfoList.pDeviceInfo[index];
    assert(pDeviceInfo);
    assert(pDeviceInfo->nTLayerType == MV_GIGE_DEVICE);

    int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    const std::string ip = std::to_string(nIp1) + std::to_string(nIp2) + std::to_string(nIp3) + std::to_string(nIp4);
    cameraIP_ = ip;
    cameraID_ = nIp1 - 5;

    LOG(INFO) << "Device Model Name: " << pDeviceInfo->SpecialInfo.stGigEInfo.chModelName << "; CurrentIp: " << cameraIP_ << "; UserDefinedName: " << pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
}

void SingleCamera::SetHandle(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_CC_CreateHandle(&cameraHandle_, deviceInfoList.pDeviceInfo[index]);
    assert(MV_OK == err);
}

bool SingleCamera::StartCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    err = MV_CC_OpenDevice(cameraHandle_);
    if (MV_OK != err) {
        LOG(ERROR) << "Failed to MV_CC_OpenDevice.";
        return false;
    }

    (void)ConfigDevices();
    err = MV_CC_RegisterImageCallBackEx(cameraHandle_, ImageCB, this);
    assert(MV_OK == err);

    err = MV_CC_StartGrabbing(cameraHandle_);
    assert(MV_OK == err);

    return true;
}

void SingleCamera::CheckAndRestartCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index) {
    DLOG(INFO) << __FUNCTION__ << " start, cameraStartupCounter_: " << cameraStartupCounter_;

    const int WAITING_FOR_STARTUP_COUNTER = 3;
    if (cameraStartupCounter_ < WAITING_FOR_STARTUP_COUNTER) {
        ++cameraStartupCounter_;
        LOG(INFO) << "cameraStartupCounter_: " << cameraStartupCounter_;
        return;
    }

    LOG_FIRST_N(INFO, 1) << "Gonna monitor cameras status.";
    if (!isCallbackOK_) {
        if (RestartCamera(deviceInfoList, index)) {
            cameraStartupCounter_ = 0;
        } // else do nothing
    } // else do nothing

    isCallbackOK_ = false;
}

void SingleCamera::LogDeviceStatus() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    MVCC_INTVALUE currentDeviceLinkSpeed = {};
    err = MV_CC_GetIntValue(cameraHandle_, "DeviceLinkSpeed", &currentDeviceLinkSpeed);
    assert(MV_OK == err);

    LOG(INFO) << "Get DeviceLinkSpeed: " << currentDeviceLinkSpeed.nCurValue;
    return;
}

void __stdcall SingleCamera::ImageCB(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *_pSingleCamera) {
    struct timeval now;
    gettimeofday(&now, NULL);
    const double unixTime = now.tv_sec + now.tv_usec / 1000000.;
    const double unixTimeMinusGpsTime = s_pManager_->unixTimeMinusGpsTime_;
    LOG_EVERY_N(INFO, 50) << "unixTimeMinusGpsTime: " << std::fixed << unixTimeMinusGpsTime;
    const double gpsTime = unixTime - unixTimeMinusGpsTime;

    SingleCamera *pSingleCamera = static_cast<SingleCamera *>(_pSingleCamera);
    pSingleCamera->isCallbackOK_ = true;

    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }

    const int _cameraID(pSingleCamera->GetCameraID());
    LOG_EVERY_N(INFO, 20) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: "
        << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight
        << " from " << _cameraID
        << "; ExposureTime: " << pFrameInfo->fExposureTime
        << "; fGain: " << pFrameInfo->fGain;

    pSingleCamera->currentImageExposureTime_ = pFrameInfo->fExposureTime;

    void *handle = pSingleCamera->GetHandle();
    unsigned char *pDataForRGB = (unsigned char *)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    assert(pDataForRGB);
    MV_CC_PIXEL_CONVERT_PARAM convertParam = {0};
    convertParam.nWidth = pFrameInfo->nWidth;
    convertParam.nHeight = pFrameInfo->nHeight;
    convertParam.pSrcData = pData;
    convertParam.nSrcDataLen = pFrameInfo->nFrameLen;
    convertParam.enSrcPixelType = pFrameInfo->enPixelType;
    convertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convertParam.pDstBuffer = pDataForRGB;
    convertParam.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048;
    int err = MV_CC_ConvertPixelType(handle, &convertParam);
    if(MV_OK != err) {
        LOG(ERROR) << "Failed to MV_CC_ConvertPixelType; err: " << err;
        return;
    }

    cv::Mat mat2Save(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
    memcpy(mat2Save.data, pDataForRGB, pFrameInfo->nWidth * pFrameInfo->nHeight * 3);
    pSingleCamera->mat2PubMutex_.lock();
    pSingleCamera->mat2Pub_ = mat2Save;
    pSingleCamera->mat2PubMutex_.unlock();
    if(pDataForRGB) {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }

    uint32TOuint64_t uint32TOuint64;
    uint32TOuint64.uint32Data[0] = pFrameInfo->nDevTimeStampLow;
    uint32TOuint64.uint32Data[1] = pFrameInfo->nDevTimeStampHigh;
    const double deviceTimeStamp = static_cast<double>(uint32TOuint64.uint64Data) / 100000000.;
    DLOG(INFO) << std::fixed << "deviceTimeStamp: " << deviceTimeStamp;

    pSingleCamera->imageFreq_ = 1 / (deviceTimeStamp - pSingleCamera->lastDeviceTimeStamp_);
    pSingleCamera->lastDeviceTimeStamp_ = deviceTimeStamp;
    LOG_EVERY_N(INFO, 50) << "pSingleCamera->imageFreq_: " << pSingleCamera->imageFreq_;
    pSingleCamera->PublishCamFps();

    const double gpsTimeDaySec = fmod(gpsTime, (3600 * 24));
    const double triggerTimeDaySec = gpsTimeDaySec; // - 0.075;
    LOG_FIRST_N(INFO, 1) << "MV_CH089_10GC camera callback delay is ~75 ms.";

    char triggerTimeDaySecCstr[20];
    sprintf(triggerTimeDaySecCstr, "%012.6f", triggerTimeDaySec);

    const std::string picFileName(pSingleCamera->imagePath_ + triggerTimeDaySecCstr + "_" + std::to_string(pFrameInfo->nFrameNum) + "_" + std::to_string(deviceTimeStamp) + "_" + std::to_string(_cameraID) + ".jpg");
    DLOG(INFO) << "picFileName: " << picFileName;

    if (unixTimeMinusGpsTime < 0) {
        LOG(WARNING) << "No save image before unixTimeMinusGpsTime be updated: " << unixTimeMinusGpsTime;
        goto ByeBye;
    }

    if(s_pManager_->isSaveImg_) {
        SaveImageTask *pSaveImageTask = new SaveImageTask(mat2Save, *pFrameInfo, picFileName);
        s_pManager_->threadPool_.append_task(pSaveImageTask);
    }

ByeBye:
    DLOG_EVERY_N(INFO, 20) << __FUNCTION__ << " end.";
}

void SingleCamera::PublishCamFps() {
    std_msgs::Float64 msgImageFreq;
    msgImageFreq.data = imageFreq_;
    pubCamSpeed_.publish(msgImageFreq);
}

std::string SingleCamera::GetCameraIP() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return cameraIP_;
}

int SingleCamera::GetCameraID() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return cameraID_;
}

size_t SingleCamera::GetCameraIndex() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return cameraIndex_;
}

void *SingleCamera::GetHandle() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return cameraHandle_;
}

void SingleCamera::AdaptNightMode() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    if (currentImageExposureTime_ > 14999) {
        if (!isNightMode_) {
            EnterNightMode();
            isNightMode_ = true;
        }
        // else nothing
    }
    else {
        if (isNightMode_) {
            ExitNightMode();
            isNightMode_ = false;
        }
        // else nothing
    }
}

void SingleCamera::EnterNightMode() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    err = MV_CC_SetEnumValue(cameraHandle_, "GainAuto", 0);
    assert(MV_OK == err);

    err = MV_CC_SetFloatValue(cameraHandle_, "Gain", 20);
    assert(MV_OK == err);
}

void SingleCamera::ExitNightMode() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;
    int configValue1 = -1;
    int configValue2 = -1;

    MVCC_ENUMVALUE currentGainMode = {0};
    err = MV_CC_GetGainMode(cameraHandle_, &currentGainMode);
    assert(MV_OK == err);
    err = MV_CC_SetEnumValue(cameraHandle_, "GainAuto", 2);
    assert(MV_OK == err);
    LOG(INFO) << "GainAuto: " << currentGainMode.nCurValue << " --> 2";
    MVCC_FLOATVALUE currentAutoGainLowerLimit = {0};
    err = MV_CC_GetFloatValue(cameraHandle_, "AutoGainLowerLimit", &currentAutoGainLowerLimit);
    assert(MV_OK == err);
    MVCC_FLOATVALUE currentAutoGainUpperLimit = {0};
    err = MV_CC_GetFloatValue(cameraHandle_, "AutoGainUpperLimit", &currentAutoGainUpperLimit);
    assert(MV_OK == err);
    err = MV_CC_SetFloatValue(cameraHandle_, "AutoGainLowerLimit", 0);
    err = MV_CC_SetFloatValue(cameraHandle_, "AutoGainUpperLimit", 20);
    assert(MV_OK == err);
    LOG(INFO) << "AutoGain [" << currentAutoGainLowerLimit.fCurValue << ", " << currentAutoGainUpperLimit.fCurValue << "] --> [0, 20]";

    return;
}

void SingleCamera::ConfigDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;
    const std::string hikConfig("/opt/smartc/config/hik_config.yaml");
    cv::FileStorage fs(hikConfig, cv::FileStorage::READ);
    int configValue1 = -1;
    int configValue2 = -1;

    bool currentAcquisitionFrameRateEnable = false;
    err = MV_CC_GetBoolValue(cameraHandle_, "AcquisitionFrameRateEnable", &currentAcquisitionFrameRateEnable);
    assert(MV_OK == err);
    fs["AcquisitionFrameRateEnable"] >> configValue1;
    err = MV_CC_SetBoolValue(cameraHandle_, "AcquisitionFrameRateEnable", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Current AcquisitionFrameRateEnable: " << currentAcquisitionFrameRateEnable << "-->" << configValue1;

    fs["TriggerMode"] >> configValue1;
    err = MV_CC_SetEnumValue(cameraHandle_, "TriggerMode", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Set trigger mode: " << configValue1;
    fs["TriggerSource"] >> configValue1;
    err = MV_CC_SetEnumValue(cameraHandle_, "TriggerSource", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Set TriggerSource: " << configValue1;
    fs["TriggerActivation"] >> configValue1;
    err = MV_CC_SetEnumValue(cameraHandle_, "TriggerActivation", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Set TriggerActivation: " << configValue1;

    bool currentTriggerCacheEnable = false;
    err = MV_CC_GetBoolValue(cameraHandle_, "TriggerCacheEnable", &currentTriggerCacheEnable);
    assert(MV_OK == err);
    fs["TriggerCacheEnable"] >> configValue1;
    err = MV_CC_SetBoolValue(cameraHandle_, "TriggerCacheEnable", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Current TriggerCacheEnable: " << currentTriggerCacheEnable << "-->" << configValue1;

    MVCC_INTVALUE currentDeviceLinkSpeed = {};
    err = MV_CC_GetIntValue(cameraHandle_, "DeviceLinkSpeed", &currentDeviceLinkSpeed);
    assert(MV_OK == err);
    LOG(INFO) << "Get DeviceLinkSpeed: " << currentDeviceLinkSpeed.nCurValue;

    const int nPacketSize = MV_CC_GetOptimalPacketSize(cameraHandle_);
    LOG(INFO) << "Detection network optimal package size, nPacketSize: " << nPacketSize;
    assert(nPacketSize > 0);
    err = MV_CC_SetIntValue(cameraHandle_, "GevSCPSPacketSize", nPacketSize);
    assert(MV_OK == err);

    MVCC_INTVALUE currentGevSCPD = {};
    err = MV_CC_GetIntValue(cameraHandle_, "GevSCPD", &currentGevSCPD);
    assert(MV_OK == err);
    fs["GevSCPD"] >> configValue1;
    err = MV_CC_SetIntValue(cameraHandle_, "GevSCPD", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Set GevSCPD: " << currentGevSCPD.nCurValue << " --> " << configValue1;

    bool currentGevPAUSEFrameReception = false;
    err = MV_CC_GetBoolValue(cameraHandle_, "GevPAUSEFrameReception", &currentGevPAUSEFrameReception);
    assert(MV_OK == err);
    fs["GevPAUSEFrameReception"] >> configValue1;
    err = MV_CC_SetBoolValue(cameraHandle_, "GevPAUSEFrameReception", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Set GevPAUSEFrameReception: " << currentGevPAUSEFrameReception << "-->" << configValue1;

    MVCC_ENUMVALUE currentBinningHorizontal = {0};
    err = MV_CC_GetEnumValue(cameraHandle_, "BinningHorizontal", &currentBinningHorizontal);
    assert(MV_OK == err);
    MVCC_ENUMVALUE currentBinningVertical = {0};
    err = MV_CC_GetEnumValue(cameraHandle_, "BinningVertical", &currentBinningVertical);
    assert(MV_OK == err);
    fs["BinningHorizontal"] >> configValue1;
    err = MV_CC_SetEnumValue(cameraHandle_, "BinningHorizontal", configValue1);
    assert(MV_OK == err);
    fs["BinningVertical"] >> configValue2;
    err = MV_CC_SetEnumValue(cameraHandle_, "BinningVertical", configValue2);
    assert(MV_OK == err);
    LOG(INFO) << "Current binning: " << currentBinningHorizontal.nCurValue << "*" << currentBinningVertical.nCurValue << "-->" << configValue1 << "*" << configValue2;

    MVCC_ENUMVALUE currentPixel = {0};
    err = MV_CC_GetPixelFormat(cameraHandle_, &currentPixel);
    assert(MV_OK == err);
    // only PixelType_Gvsp_BayerRG8 can support [4096 * 2160] * 10 fps
    unsigned int pixel = PixelType_Gvsp_BayerRG8;
    err = MV_CC_SetPixelFormat(cameraHandle_, pixel);
    assert(MV_OK == err);
    LOG(INFO) << "PixelFormat: " << currentPixel.nCurValue << " --> " << pixel;

    ExitNightMode();

    MVCC_ENUMVALUE currentExposureAuto = {0};
    err = MV_CC_GetEnumValue(cameraHandle_, "ExposureAuto", &currentExposureAuto);
    assert(MV_OK == err);
    fs["ExposureAuto"] >> configValue1;
    err = MV_CC_SetEnumValue(cameraHandle_, "ExposureAuto", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "ExposureAuto: " << currentExposureAuto.nCurValue << " --> " << configValue1;
    MVCC_INTVALUE currentAutoExposureTimeLowerLimit = {0};
    MVCC_INTVALUE currentAutoExposureTimeUpperLimit = {0};
    err = MV_CC_GetIntValue(cameraHandle_, "AutoExposureTimeLowerLimit", &currentAutoExposureTimeLowerLimit);
    err = MV_CC_GetIntValue(cameraHandle_, "AutoExposureTimeUpperLimit", &currentAutoExposureTimeUpperLimit);
    assert(MV_OK == err);
    fs["AutoExposureTimeLowerLimit"] >> configValue1;
    fs["AutoExposureTimeUpperLimit"] >> configValue2;
    err = MV_CC_SetIntValue(cameraHandle_, "AutoExposureTimeLowerLimit", configValue1);
    err = MV_CC_SetIntValue(cameraHandle_, "AutoExposureTimeUpperLimit", configValue2);
    assert(MV_OK == err);
    LOG(INFO) << "AutoExposureTime [" << currentAutoExposureTimeLowerLimit.nCurValue << ", " << currentAutoExposureTimeUpperLimit.nCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    MVCC_INTVALUE currentLineDebouncerTime = {}; // LineDebouncerTime
    err = MV_CC_GetIntValue(cameraHandle_, "LineDebouncerTime", &currentLineDebouncerTime);
    assert(MV_OK == err);
    fs["LineDebouncerTime"] >> configValue1;
    err = MV_CC_SetIntValue(cameraHandle_, "LineDebouncerTime", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "Set LineDebouncerTime: " << currentLineDebouncerTime.nCurValue << " --> " << configValue1;

    TurnOnFrameSpecInfo(FrameSpecInfoSelector::Timestamp);
    TurnOnFrameSpecInfo(FrameSpecInfoSelector::Gain);
    TurnOnFrameSpecInfo(FrameSpecInfoSelector::Exposure);
    TurnOnFrameSpecInfo(FrameSpecInfoSelector::BrightnessInfo);

    fs.release();
    return;
}

bool SingleCamera::RestartCamera(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    if (cameraHandle_) {
        err = MV_CC_CloseDevice(cameraHandle_);
        (MV_OK == err)? (LOG(INFO) << "Success."): (LOG(ERROR) << "Failed: " << err);
        err = MV_CC_DestroyHandle(cameraHandle_);
        (MV_OK == err)? (LOG(INFO) << "Success."): (LOG(ERROR) << "Failed: " << err);
        cameraHandle_ = NULL;
    }

    SetHandle(deviceInfoList, index);
    if (!StartCamera()) {
        LOG(ERROR) << "Failed to StartCamera.";
        err = MV_CC_DestroyHandle(cameraHandle_);
        (MV_OK == err)? (LOG(INFO) << "Success."): (LOG(ERROR) << "Failed: " << err);
        cameraHandle_ = NULL;
        return false;
    }

    LOG(INFO) << "StartCamera successfully.";
    return true;
}

void SingleCamera::TurnOnFrameSpecInfo(FrameSpecInfoSelector frameSpecInfoSelector) {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    err = MV_CC_SetEnumValue(cameraHandle_, "FrameSpecInfoSelector", frameSpecInfoSelector);
    assert(MV_OK == err);
    bool isOnBefore = false;
    err = MV_CC_GetBoolValue(cameraHandle_, "FrameSpecInfo", &isOnBefore);
    assert(MV_OK == err);
    err = MV_CC_SetBoolValue(cameraHandle_, "FrameSpecInfo", true);
    assert(MV_OK == err);
    bool isOnNow = false;
    err = MV_CC_GetBoolValue(cameraHandle_, "FrameSpecInfo", &isOnNow);
    LOG(INFO) << "Set FrameSpecInfo: " << isOnBefore << "-->" << isOnNow;
}
