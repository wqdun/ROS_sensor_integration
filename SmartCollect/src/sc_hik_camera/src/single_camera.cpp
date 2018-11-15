#include "single_camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

std::deque<time2Mat_t> SingleCamera::s_time2Mat_;
std::mutex SingleCamera::s_matImageMutex_;
SingleCamera::SingleCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
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
    StartCamera();
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

    LOG(INFO) << "Device Model Name: " << pDeviceInfo->SpecialInfo.stGigEInfo.chModelName << "; CurrentIp: " << cameraIP_ << "; UserDefinedName: " << pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
}

void SingleCamera::SetHandle(const MV_CC_DEVICE_INFO_LIST &deviceInfoList, size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    void *handle = NULL;
    int err = MV_CC_CreateHandle(&handle, deviceInfoList.pDeviceInfo[index]);
    assert(MV_OK == err);

    cameraHandle_ = handle;
}

void SingleCamera::StartCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;

    err = MV_CC_OpenDevice(cameraHandle_);
    assert(MV_OK == err);

    (void)ConfigDevices();

    err = MV_CC_RegisterImageCallBackEx(cameraHandle_, ImageCB, this);
    assert(MV_OK == err);

    err = MV_CC_StartGrabbing(cameraHandle_);
    assert(MV_OK == err);
}

void __stdcall SingleCamera::ImageCB(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *_pSingleCamera) {
    struct timeval now;
    gettimeofday(&now, NULL);
    const double unixTime = now.tv_sec + now.tv_usec / 1000000.;

    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }
    LOG(INFO) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: " << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight;
    int err = MV_OK;

    SingleCamera *pSingleCamera = static_cast<SingleCamera *>(_pSingleCamera);
    void *handle = pSingleCamera->GetHandle();

    unsigned char *pDataForRGB = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
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
    err = MV_CC_ConvertPixelType(handle, &convertParam);
    if(MV_OK != err) {
        LOG(ERROR) << "Failed to MV_CC_ConvertPixelType; err: " << err;
        return;
    }
    LOG_FIRST_N(INFO, 1) << "ConvertPixelType " << pFrameInfo->enPixelType << " --> " << PixelType_Gvsp_BGR8_Packed;

    cv::Mat matBGR(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
    memcpy(matBGR.data, pDataForRGB, pFrameInfo->nWidth * pFrameInfo->nHeight * 3);
    if(pDataForRGB) {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }

    const std::string _cameraIP = pSingleCamera->GetCameraIP();
    const size_t _cameraIndex = pSingleCamera->GetCameraIndex();
    LOG_FIRST_N(INFO, 20) << "_cameraIP: " << _cameraIP << "; _cameraIndex: " << _cameraIndex;

    time2Mat_t time2Mat;
    time2Mat.header = unixTime;
    time2Mat.cameraIP = _cameraIP;
    time2Mat.cameraIndex = _cameraIndex;
    time2Mat.frameNum = pFrameInfo->nFrameNum;
    time2Mat.matImage = matBGR;

    s_matImageMutex_.lock();
    s_time2Mat_.emplace_back(time2Mat);
    LOG(INFO) << s_time2Mat_.size() << " s_time2Mat_";
    s_matImageMutex_.unlock();

    LOG(INFO) << __FUNCTION__ << " end.";
}

std::string SingleCamera::GetCameraIP() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return cameraIP_;
}

size_t SingleCamera::GetCameraIndex() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return cameraIndex_;
}

void *SingleCamera::GetHandle() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return cameraHandle_;
}


void SingleCamera::ConfigDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;
    const std::string hikConfig("/opt/smartc/config/hik_config.yaml");
    cv::FileStorage fs(hikConfig, cv::FileStorage::READ);
    int configValue1 = -1;
    int configValue2 = -1;

    bool currentAcquisitionFrameRateEnable = {0};
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

    const int nPacketSize = MV_CC_GetOptimalPacketSize(cameraHandle_);
    LOG(INFO) << "Detection network optimal package size, nPacketSize: " << nPacketSize;
    assert(nPacketSize > 0);
    err = MV_CC_SetIntValue(cameraHandle_, "GevSCPSPacketSize", nPacketSize);
    assert(MV_OK == err);

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

    MVCC_ENUMVALUE currentGainMode = {0};
    err = MV_CC_GetGainMode(cameraHandle_, &currentGainMode);
    assert(MV_OK == err);
    fs["GainAuto"] >> configValue1;
    err = MV_CC_SetEnumValue(cameraHandle_, "GainAuto", configValue1);
    assert(MV_OK == err);
    LOG(INFO) << "GainAuto: " << currentGainMode.nCurValue << " --> " << configValue1;
    MVCC_FLOATVALUE currentAutoGainLowerLimit = {0};
    err = MV_CC_GetFloatValue(cameraHandle_, "AutoGainLowerLimit", &currentAutoGainLowerLimit);
    assert(MV_OK == err);
    MVCC_FLOATVALUE currentAutoGainUpperLimit = {0};
    err = MV_CC_GetFloatValue(cameraHandle_, "AutoGainUpperLimit", &currentAutoGainUpperLimit);
    assert(MV_OK == err);
    fs["AutoGainLowerLimit"] >> configValue1;
    fs["AutoGainUpperLimit"] >> configValue2;
    err = MV_CC_SetFloatValue(cameraHandle_, "AutoGainLowerLimit", configValue1);
    err = MV_CC_SetFloatValue(cameraHandle_, "AutoGainUpperLimit", configValue2);
    assert(MV_OK == err);
    LOG(INFO) << "AutoGain [" << currentAutoGainLowerLimit.fCurValue << ", " << currentAutoGainUpperLimit.fCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

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

    fs.release();
    return;
}


