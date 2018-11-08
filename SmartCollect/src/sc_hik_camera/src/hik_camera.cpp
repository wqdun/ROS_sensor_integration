#include "hik_camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

HikCamera::HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_OK;
    handles_.clear();
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

HikCamera::~HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)DoClean();

    LOG(INFO) << __FUNCTION__ << " end.";
}

void HikCamera::EnumGigeDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_CC_EnumDevices(MV_GIGE_DEVICE, &deviceList_);
    assert(MV_OK == err_);

    LOG(INFO) << "Find " << deviceList_.nDeviceNum << " Devices.";
    assert(deviceList_.nDeviceNum > 0);

    for(size_t i = 0; i < deviceList_.nDeviceNum; ++i) {
        LOG(INFO) << "device: " << i;
        MV_CC_DEVICE_INFO* pDeviceInfo = deviceList_.pDeviceInfo[i];
        assert(pDeviceInfo);

        (void)PrintDeviceInfo(pDeviceInfo);
    }
}

void HikCamera::OpenConfigDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(size_t i = 0; i < deviceList_.nDeviceNum; ++i) {
        void *_handle = NULL;
        err_ = MV_CC_CreateHandle(&_handle, deviceList_.pDeviceInfo[i]);
        assert(MV_OK == err_);

        err_ = MV_CC_OpenDevice(_handle);
        assert(MV_OK == err_);

        handles_.push_back(_handle);
        (void)ConfigDevices(i);
    }
}

void HikCamera::ConfigDevices(size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    void *handle = handles_[index];
    const std::string hikConfig("/opt/smartc/config/hik_config.yaml");
    cv::FileStorage fs(hikConfig, cv::FileStorage::READ);
    int configValue1 = -1;
    int configValue2 = -1;

    fs["TriggerMode"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "TriggerMode", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "Set trigger mode: " << configValue1;
    fs["TriggerSource"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "TriggerSource", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "Set TriggerSource: " << configValue1;
    fs["TriggerActivation"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "TriggerActivation", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "Set TriggerActivation: " << configValue1;

    const int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
    LOG(INFO) << "Detection network optimal package size, nPacketSize: " << nPacketSize;
    assert(nPacketSize > 0);
    err_ = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
    assert(MV_OK == err_);

    MVCC_ENUMVALUE currentBinningHorizontal = {0};
    err_ = MV_CC_GetEnumValue(handle, "BinningHorizontal", &currentBinningHorizontal);
    assert(MV_OK == err_);
    MVCC_ENUMVALUE currentBinningVertical = {0};
    err_ = MV_CC_GetEnumValue(handle, "BinningVertical", &currentBinningVertical);
    assert(MV_OK == err_);
    fs["BinningHorizontal"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "BinningHorizontal", configValue1);
    assert(MV_OK == err_);
    fs["BinningVertical"] >> configValue2;
    err_ = MV_CC_SetEnumValue(handle, "BinningVertical", configValue2);
    assert(MV_OK == err_);
    LOG(INFO) << "Current binning: " << currentBinningHorizontal.nCurValue << "*" << currentBinningVertical.nCurValue << "-->" << configValue1 << "*" << configValue2;

    MVCC_ENUMVALUE currentPixel = {0};
    err_ = MV_CC_GetPixelFormat(handle, &currentPixel);
    assert(MV_OK == err_);

    unsigned int pixel = PixelType_Gvsp_RGB8_Packed;
    err_ = MV_CC_SetPixelFormat(handle, pixel);
    assert(MV_OK == err_);
    LOG(INFO) << "PixelFormat: " << currentPixel.nCurValue << " --> " << pixel;

    MVCC_FLOATVALUE currentFrameRate = {0};
    err_ = MV_CC_GetFrameRate(handle, &currentFrameRate);
    assert(MV_OK == err_);

    const float frameRate = 10;
    err_ = MV_CC_SetFrameRate(handle, frameRate);
    assert(MV_OK == err_);
    LOG(INFO) << "Frame rate: " << currentFrameRate.fCurValue << " --> " << frameRate;

    MVCC_ENUMVALUE currentGainMode = {0};
    err_ = MV_CC_GetGainMode(handle, &currentGainMode);
    assert(MV_OK == err_);
    fs["GainAuto"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "GainAuto", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "GainAuto: " << currentGainMode.nCurValue << " --> " << configValue1;

    MVCC_FLOATVALUE currentAutoGainLowerLimit = {0};
    err_ = MV_CC_GetFloatValue(handle, "AutoGainLowerLimit", &currentAutoGainLowerLimit);
    assert(MV_OK == err_);
    MVCC_FLOATVALUE currentAutoGainUpperLimit = {0};
    err_ = MV_CC_GetFloatValue(handle, "AutoGainUpperLimit", &currentAutoGainUpperLimit);
    assert(MV_OK == err_);
    fs["AutoGainLowerLimit"] >> configValue1;
    fs["AutoGainUpperLimit"] >> configValue2;
    err_ = MV_CC_SetFloatValue(handle, "AutoGainLowerLimit", configValue1);
    err_ = MV_CC_SetFloatValue(handle, "AutoGainUpperLimit", configValue2);
    assert(MV_OK == err_);
    LOG(INFO) << "AutoGain [" << currentAutoGainLowerLimit.fCurValue << ", " << currentAutoGainUpperLimit.fCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    MVCC_ENUMVALUE currentExposureAuto = {0};
    err_ = MV_CC_GetEnumValue(handle, "ExposureAuto", &currentExposureAuto);
    assert(MV_OK == err_);
    fs["ExposureAuto"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "ExposureAuto", configValue1);
    assert(MV_OK == err_);
    LOG(INFO) << "ExposureAuto: " << currentExposureAuto.nCurValue << " --> " << configValue1;

    MVCC_INTVALUE currentAutoExposureTimeLowerLimit = {0};
    MVCC_INTVALUE currentAutoExposureTimeUpperLimit = {0};
    err_ = MV_CC_GetIntValue(handle, "AutoExposureTimeLowerLimit", &currentAutoExposureTimeLowerLimit);
    err_ = MV_CC_GetIntValue(handle, "AutoExposureTimeUpperLimit", &currentAutoExposureTimeUpperLimit);
    assert(MV_OK == err_);

    fs["AutoExposureTimeLowerLimit"] >> configValue1;
    fs["AutoExposureTimeUpperLimit"] >> configValue2;
    err_ = MV_CC_SetIntValue(handle, "AutoExposureTimeLowerLimit", configValue1);
    err_ = MV_CC_SetIntValue(handle, "AutoExposureTimeUpperLimit", configValue2);
    assert(MV_OK == err_);

    LOG(INFO) << "AutoExposureTime [" << currentAutoExposureTimeLowerLimit.nCurValue << ", " << currentAutoExposureTimeUpperLimit.nCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    fs.release();
}

void HikCamera::RegisterCB() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(const auto &handle: handles_) {
        err_ = MV_CC_RegisterImageCallBackEx(handle, ImageCB, handle);
        assert(MV_OK == err_);

        err_ = MV_CC_StartGrabbing(handle);
        assert(MV_OK == err_);
    }
}

void HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
    LOG(INFO) << __FUNCTION__ << " start.";
    assert(pstMVDevInfo);
    assert(pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE);

    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    LOG(INFO) << "Device Model Name: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName;
    LOG(INFO) << "CurrentIp: " << nIp1 << nIp2 << nIp3<< nIp4;
    LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName;

    return;
}

void HikCamera::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)EnumGigeDevices();
    (void)OpenConfigDevices();
    (void)RegisterCB();
    PressEnterToExit();
    while(0) {
        LOG(INFO) << "while start.";
        // s_mutexLocker_.mutex_lock();
        // std::deque<mat2SlamProtocols_t> _mat2Slams = s_mat2Slams_;
        // LOG(INFO) << "dequeue size" << s_mat2Slams_.size();
        // if(s_mat2Slams_.empty()) {
        //     LOG(INFO) << "s_mat2Slams_.empty()";
        //     s_mutexLocker_.mutex_unlock();
        //     usleep(50000);
        //     continue;
        // }
        // s_mat2Slams_.pop_front();
        // s_mutexLocker_.mutex_unlock();

        // LOG(INFO) << "inputImage start.";
        // std::vector<Box> emptybox;
        // pair<double,vector<Box>> emptyonebox;
        // double unixTime = _mat2Slams.front().header;
        // emptyonebox.first = unixTime;
        // emptyonebox.second = emptybox;

        // (void)IMUProc(_mat2Slams.front().slams, system_);
        // LOG(INFO) << "IMUProc end.";

        // int queue_length = _mat2Slams.size();
        // (void)ImageProc(_mat2Slams.front().matImage, unixTime, &system_, emptyonebox, queue_length);
        // LOG(INFO) << "inputImage end.";
    }
}

void HikCamera::DoClean() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(const auto &handle: handles_) {
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

void HikCamera::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

void __stdcall HikCamera::ImageCB(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }

    LOG(INFO) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: " << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight;
    // (void)Convert2Mat(pData, pFrameInfo, pUser);
}

// void HikCamera::Convert2Mat(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
//     DLOG(INFO) << __FUNCTION__ << " start.";

//     struct timeval now;
//     gettimeofday(&now, NULL);
//     const double unixTime = now.tv_sec + now.tv_usec / 1000000.;

//     cv::Mat matImage(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
//     memcpy(matImage.data, pData, pFrameInfo->nWidth * pFrameInfo->nHeight * 3);
//     cv::Mat matBGR(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
//     cv::cvtColor(matImage, matBGR, cv::COLOR_RGB2BGR);

//     mat2SlamProtocols_t mat2Slams;
//     mat2Slams.matImage = matBGR;
//     mat2Slams.header = unixTime;

//     s_pSerialReader_->slam10DatasMutex_.lock();
//     DLOG(INFO) << s_pSerialReader_->slam10Datas_.size();
//     mat2Slams.slams = s_pSerialReader_->slam10Datas_;
//     s_pSerialReader_->slam10DatasMutex_.unlock();

//     s_mutexLocker_.mutex_lock();
//     s_mat2Slams_.emplace_back(mat2Slams);
//     s_mutexLocker_.mutex_unlock();

//     DLOG(INFO) << __FUNCTION__ << " end.";
// }
