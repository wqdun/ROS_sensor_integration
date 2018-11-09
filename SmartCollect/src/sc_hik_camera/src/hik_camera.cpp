#include "hik_camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

MV_CC_DEVICE_INFO_LIST HikCamera::s_deviceList_;
std::mutex HikCamera::s_matImageMutex_;
std::vector<void *> HikCamera::s_handles_;
std::deque<time2Mat_t> HikCamera::s_time2Mat_;

HikCamera::HikCamera(ros::NodeHandle nh, ros::NodeHandle private_nh):
    threadPool_(10, 20)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_OK;
    nh_ = nh;
    pubImages_.clear();
    memset(&s_deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    threadPool_.start();
}

HikCamera::~HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)DoClean();

    LOG(INFO) << __FUNCTION__ << " end.";
}

void HikCamera::OpenConfigDevices(size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";

    void *handle = NULL;
    err_ = MV_CC_CreateHandle(&handle, s_deviceList_.pDeviceInfo[index]);
    assert(MV_OK == err_);
    s_handles_.push_back(handle);
    // then s_handles_[index] == handle

    err_ = MV_CC_OpenDevice(handle);
    assert(MV_OK == err_);

    (void)ConfigDevices(index, handle);

    // err_ = MV_CC_RegisterImageCallBackEx(handle, ImageCB, &index);
    // 回掉函数中不能传入栈变量，core
    LOG(INFO) << index;
    assert(MV_OK == err_);

    err_ = MV_CC_StartGrabbing(handle);
    assert(MV_OK == err_);
}

int HikCamera::Get1stByteIP(size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    MV_CC_DEVICE_INFO* pDeviceInfo = s_deviceList_.pDeviceInfo[index];
    assert(pDeviceInfo);
    assert(pDeviceInfo->nTLayerType == MV_GIGE_DEVICE);

    int nIp1 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    LOG(INFO) << "Device Model Name: " << pDeviceInfo->SpecialInfo.stGigEInfo.chModelName;
    LOG(INFO) << "CurrentIp: " << nIp1 << nIp2 << nIp3<< nIp4;
    LOG(INFO) << "UserDefinedName: " << pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;

    pDeviceInfo->nReserved[0] = nIp1;
    return nIp1;
}

void HikCamera::SetAdvertiseHandle(size_t index, int _1stByteIP) {
    LOG(INFO) << __FUNCTION__ << " start.";
    image_transport::Publisher pub;
    image_transport::ImageTransport it(nh_);
    pub = it.advertise("camera/image" + std::to_string(_1stByteIP), 0);

    pubImages_.push_back(pub);
    return;
}

void HikCamera::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_CC_EnumDevices(MV_GIGE_DEVICE, &s_deviceList_);
    assert(MV_OK == err_);
    LOG(INFO) << "Find " << s_deviceList_.nDeviceNum << " Devices.";

    for(size_t i = 0; i < s_deviceList_.nDeviceNum; ++i) {
        int _1stByteIP = Get1stByteIP(i);
        (void)SetAdvertiseHandle(i, _1stByteIP);
        (void)OpenConfigDevices(i);
    }
    // PressEnterToExit();
    while(1) {
        LOG(INFO) << "while start.";

        s_matImageMutex_.lock();
        LOG(INFO) << "s_time2Mat_.size(): " << s_time2Mat_.size();
        if(s_time2Mat_.empty()) {
            LOG_EVERY_N(INFO, 20) << "s_time2Mat_.empty()";
            s_matImageMutex_.unlock();
            usleep(500000);
            continue;
        }
        time2Mat_t _time2Mat = s_time2Mat_.front();
        s_time2Mat_.pop_front();
        s_matImageMutex_.unlock();

        const size_t _1stByteIP = 6;//s_deviceList_.pDeviceInfo[_time2Mat.cameraIndex]->nReserved[0];
        SaveImageTask *pSaveImageTask = new SaveImageTask(_time2Mat.header, _1stByteIP, _time2Mat.matImage);
        threadPool_.append_task(pSaveImageTask);

        // cv::Mat _matImage = ;
        // cv::resize(pPGCamera->img_, matImageDown, cv::Size(1920 / 5, 1200 / 5));
        // sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matImageDown).toImageMsg();

        // pubImage_.publish();
    }
}

void __stdcall HikCamera::ImageCB(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pIndex) {
    struct timeval now;
    gettimeofday(&now, NULL);
    const double unixTime = now.tv_sec + now.tv_usec / 1000000.;

    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }

    int err = MV_OK;
    const size_t _cameraIndex = *(size_t *)pIndex;
    LOG(INFO) << "camera index: " << _cameraIndex;

    void *handle = s_handles_[_cameraIndex];
    LOG(INFO) << "camera index: " << _cameraIndex;
    MV_CC_DEVICE_INFO* pDeviceInfo = s_deviceList_.pDeviceInfo[_cameraIndex];
    LOG(INFO) << "camera index: " << _cameraIndex << "; pDeviceInfo->nReserved[0]: " << pDeviceInfo->nReserved[0];
    LOG(INFO) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: " << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight;

    unsigned char *pDataForRGB = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    assert(pDataForRGB);

    MV_CC_PIXEL_CONVERT_PARAM convertParam = {0};
    convertParam.nWidth = pFrameInfo->nWidth;
    convertParam.nHeight = pFrameInfo->nHeight;
    convertParam.pSrcData = pData;
    convertParam.nSrcDataLen = pFrameInfo->nFrameLen;
    convertParam.enSrcPixelType = pFrameInfo->enPixelType;
    convertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    convertParam.pDstBuffer = pDataForRGB;
    convertParam.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048;
    err = MV_CC_ConvertPixelType(handle, &convertParam);
    if(MV_OK != err) {
        LOG(ERROR) << "Failed to MV_CC_ConvertPixelType; err: " << err;
        return;
    }
    LOG(INFO) << "ConvertPixelType " << pFrameInfo->enPixelType << " --> " << PixelType_Gvsp_RGB8_Packed;

    cv::Mat matRGB(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
    memcpy(matRGB.data, pDataForRGB, pFrameInfo->nWidth * pFrameInfo->nHeight * 3);
    if(pDataForRGB) {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }

    time2Mat_t time2Mat;
    time2Mat.header = unixTime;
    time2Mat.cameraIndex = _cameraIndex;
    time2Mat.matImage = matRGB;

    s_matImageMutex_.lock();
    s_time2Mat_.emplace_back(time2Mat);
    LOG(INFO) << s_time2Mat_.size() << " s_time2Mat_";
    s_matImageMutex_.unlock();

    LOG(INFO) << __FUNCTION__ << " end.";
}

void HikCamera::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

void HikCamera::DoClean() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.stop();
    for(const auto &handle: s_handles_) {
        err_ = MV_CC_StopGrabbing(handle);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_StopGrabbing failed, err_: " << err_;
            exit(1);
        }

        err_ = MV_CC_CloseDevice(handle);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_CloseDevice failed, err_: " << err_;
            exit(1);
        }

        err_ = MV_CC_DestroyHandle(handle);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_DestroyHandle failed, err_: " << err_;
            exit(1);
        }
    }
}


void HikCamera::ConfigDevices(size_t index, void *_handle) {
    LOG(INFO) << __FUNCTION__ << " start.";
    const std::string hikConfig("/opt/smartc/config/hik_config.yaml");
    cv::FileStorage fs(hikConfig, cv::FileStorage::READ);
    int configValue1 = -1;
    int configValue2 = -1;

    fs["TriggerMode"] >> configValue1;
    err_ = MV_CC_SetEnumValue(_handle, "TriggerMode", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "Set trigger mode: " << configValue1;
    fs["TriggerSource"] >> configValue1;
    err_ = MV_CC_SetEnumValue(_handle, "TriggerSource", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "Set TriggerSource: " << configValue1;
    fs["TriggerActivation"] >> configValue1;
    err_ = MV_CC_SetEnumValue(_handle, "TriggerActivation", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "Set TriggerActivation: " << configValue1;

    const int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
    LOG(INFO) << "Detection network optimal package size, nPacketSize: " << nPacketSize;
    assert(nPacketSize > 0);
    err_ = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
    assert(MV_OK == err_);

    MVCC_ENUMVALUE currentBinningHorizontal = {0};
    err_ = MV_CC_GetEnumValue(_handle, "BinningHorizontal", &currentBinningHorizontal);
    assert(MV_OK == err_);
    MVCC_ENUMVALUE currentBinningVertical = {0};
    err_ = MV_CC_GetEnumValue(_handle, "BinningVertical", &currentBinningVertical);
    assert(MV_OK == err_);
    fs["BinningHorizontal"] >> configValue1;
    err_ = MV_CC_SetEnumValue(_handle, "BinningHorizontal", configValue1);
    assert(MV_OK == err_);
    fs["BinningVertical"] >> configValue2;
    err_ = MV_CC_SetEnumValue(_handle, "BinningVertical", configValue2);
    assert(MV_OK == err_);
    LOG(INFO) << "Current binning: " << currentBinningHorizontal.nCurValue << "*" << currentBinningVertical.nCurValue << "-->" << configValue1 << "*" << configValue2;

    MVCC_ENUMVALUE currentPixel = {0};
    err_ = MV_CC_GetPixelFormat(_handle, &currentPixel);
    assert(MV_OK == err_);
    // only PixelType_Gvsp_BayerRG8 can support [4096 * 2160] * 10 fps
    unsigned int pixel = PixelType_Gvsp_BayerRG8;
    err_ = MV_CC_SetPixelFormat(_handle, pixel);
    assert(MV_OK == err_);
    LOG(INFO) << "PixelFormat: " << currentPixel.nCurValue << " --> " << pixel;

    MVCC_ENUMVALUE currentGainMode = {0};
    err_ = MV_CC_GetGainMode(_handle, &currentGainMode);
    assert(MV_OK == err_);
    fs["GainAuto"] >> configValue1;
    err_ = MV_CC_SetEnumValue(_handle, "GainAuto", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "GainAuto: " << currentGainMode.nCurValue << " --> " << configValue1;
    MVCC_FLOATVALUE currentAutoGainLowerLimit = {0};
    err_ = MV_CC_GetFloatValue(_handle, "AutoGainLowerLimit", &currentAutoGainLowerLimit);
    assert(MV_OK == err_);
    MVCC_FLOATVALUE currentAutoGainUpperLimit = {0};
    err_ = MV_CC_GetFloatValue(_handle, "AutoGainUpperLimit", &currentAutoGainUpperLimit);
    assert(MV_OK == err_);
    fs["AutoGainLowerLimit"] >> configValue1;
    fs["AutoGainUpperLimit"] >> configValue2;
    err_ = MV_CC_SetFloatValue(_handle, "AutoGainLowerLimit", configValue1);
    err_ = MV_CC_SetFloatValue(_handle, "AutoGainUpperLimit", configValue2);
    assert(MV_OK == err_);
    LOG(INFO) << "AutoGain [" << currentAutoGainLowerLimit.fCurValue << ", " << currentAutoGainUpperLimit.fCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    MVCC_ENUMVALUE currentExposureAuto = {0};
    err_ = MV_CC_GetEnumValue(_handle, "ExposureAuto", &currentExposureAuto);
    assert(MV_OK == err_);
    fs["ExposureAuto"] >> configValue1;
    err_ = MV_CC_SetEnumValue(_handle, "ExposureAuto", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "ExposureAuto: " << currentExposureAuto.nCurValue << " --> " << configValue1;
    MVCC_INTVALUE currentAutoExposureTimeLowerLimit = {0};
    MVCC_INTVALUE currentAutoExposureTimeUpperLimit = {0};
    err_ = MV_CC_GetIntValue(_handle, "AutoExposureTimeLowerLimit", &currentAutoExposureTimeLowerLimit);
    err_ = MV_CC_GetIntValue(_handle, "AutoExposureTimeUpperLimit", &currentAutoExposureTimeUpperLimit);
    assert(MV_OK == err_);
    fs["AutoExposureTimeLowerLimit"] >> configValue1;
    fs["AutoExposureTimeUpperLimit"] >> configValue2;
    err_ = MV_CC_SetIntValue(_handle, "AutoExposureTimeLowerLimit", configValue1);
    err_ = MV_CC_SetIntValue(_handle, "AutoExposureTimeUpperLimit", configValue2);
    assert(MV_OK == err_);
    LOG(INFO) << "AutoExposureTime [" << currentAutoExposureTimeLowerLimit.nCurValue << ", " << currentAutoExposureTimeUpperLimit.nCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    fs.release();
    return;
}

