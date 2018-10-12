#include "hik_camera_arm.h"

// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

MV_CC_PIXEL_CONVERT_PARAM HikCamera::s_convertParam_ = {0};

HikCamera::HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_OK;
    handles_.clear();
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

HikCamera::~HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)DoClean();
}

void HikCamera::EnumGigeDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_CC_EnumDevices(MV_GIGE_DEVICE, &deviceList_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_EnumDevices failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "Find " << deviceList_.nDeviceNum << " Devices.";
    if(deviceList_.nDeviceNum <= 0) {
        LOG(WARNING) << "Find No Devices.";
        exit(1);
    }

    for(int i = 0; i < deviceList_.nDeviceNum; ++i) {
        LOG(INFO) << "device: " << i;
        MV_CC_DEVICE_INFO* pDeviceInfo = deviceList_.pDeviceInfo[i];
        if(!pDeviceInfo) {
            LOG(ERROR) << "NULL == pDeviceInfo";
            exit(1);
        }
        (void)PrintDeviceInfo(pDeviceInfo);
    }
}

void HikCamera::OpenConfigDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(size_t i = 0; i < deviceList_.nDeviceNum; ++i) {
        void *_handle = NULL;
        err_ = MV_CC_CreateHandle(&_handle, deviceList_.pDeviceInfo[i]);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_CreateHandle failed, err_: " << err_ << "; i: " << i;
            exit(1);
        }

        err_ = MV_CC_OpenDevice(_handle);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_OpenDevice failed, err_: " << err_ << "; i: " << i;
            exit(1);
        }

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
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_SetTriggerMode failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "Set trigger mode: " << configValue1;

    if(deviceList_.pDeviceInfo[index]->nTLayerType != MV_GIGE_DEVICE) {
        LOG(INFO) << "It only works for the GigE camera.";
        return;
    }
    const int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
    LOG(INFO) << "Detection network optimal package size, nPacketSize: " << nPacketSize;
    if(nPacketSize <= 0) {
        LOG(ERROR) << "Get Packet Size failed, nPacketSize: " << nPacketSize;
        exit(1);
    }
    err_ = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
    if(err_ != MV_OK) {
        LOG(ERROR) << "Set Packet Size failed, err_: " << err_;
        exit(1);
    }

    MVCC_ENUMVALUE currentBinningHorizontal = {0};
    err_ = MV_CC_GetEnumValue(handle, "BinningHorizontal", &currentBinningHorizontal);
    if(MV_OK != err_) {
        LOG(ERROR) << "Get BinningHorizontal failed, err_: " << err_;
        exit(1);
    }
    MVCC_ENUMVALUE currentBinningVertical = {0};
    err_ = MV_CC_GetEnumValue(handle, "BinningVertical", &currentBinningVertical);
    if(MV_OK != err_) {
        LOG(ERROR) << "Get BinningVertical failed, err_: " << err_;
        exit(1);
    }
    fs["BinningHorizontal"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "BinningHorizontal", configValue1);
    if(MV_OK != err_) {
        LOG(ERROR) << "Set BinningHorizontal failed, err_: " << err_;
        exit(1);
    }
    fs["BinningVertical"] >> configValue2;
    err_ = MV_CC_SetEnumValue(handle, "BinningVertical", configValue2);
    if(MV_OK != err_) {
        LOG(ERROR) << "Set BinningHorizontal failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "Current binning: " << currentBinningHorizontal.nCurValue << "*" << currentBinningVertical.nCurValue << "-->" << configValue1 << "*" << configValue2;

    MVCC_ENUMVALUE currentPixel = {0};
    err_ = MV_CC_GetPixelFormat(handle, &currentPixel);
    if(err_ != MV_OK) {
        LOG(ERROR) << "MV_CC_GetPixelFormat failed, err_: " << err_;
        exit(1);
    }
    unsigned int pixel = PixelType_Gvsp_RGB8_Packed;
    err_ = MV_CC_SetPixelFormat(handle, pixel);
    if(err_ != MV_OK) {
        LOG(ERROR) << "MV_CC_SetPixelFormat failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "PixelFormat: " << currentPixel.nCurValue << " --> " << pixel;

    MVCC_FLOATVALUE currentFrameRate = {0};
    err_ = MV_CC_GetFrameRate(handle, &currentFrameRate);
    if(err_ != MV_OK) {
        LOG(ERROR) << "GetFrameRate failed, err_: " << err_;
        exit(1);
    }
    const float frameRate = 10;
    err_ = MV_CC_SetFrameRate(handle, frameRate);
    if(err_ != MV_OK) {
        LOG(ERROR) << "SetFrameRate failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "Frame rate: " << currentFrameRate.fCurValue << " --> " << frameRate;

    MVCC_ENUMVALUE currentGainMode = {0};
    err_ = MV_CC_GetGainMode(handle, &currentGainMode);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_GetGainMode failed, err_: " << err_;
        exit(1);
    }
    fs["GainAuto"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "GainAuto", configValue1);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_SetTriggerMode failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "GainAuto: " << currentGainMode.nCurValue << " --> " << configValue1;

    MVCC_FLOATVALUE currentAutoGainLowerLimit = {0};
    err_ = MV_CC_GetFloatValue(handle, "AutoGainLowerLimit", &currentAutoGainLowerLimit);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_GetFloatValue failed, err_: " << err_;
        exit(1);
    }
    MVCC_FLOATVALUE currentAutoGainUpperLimit = {0};
    err_ = MV_CC_GetFloatValue(handle, "AutoGainUpperLimit", &currentAutoGainUpperLimit);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_GetFloatValue failed, err_: " << err_;
        exit(1);
    }
    fs["AutoGainLowerLimit"] >> configValue1;
    fs["AutoGainUpperLimit"] >> configValue2;
    err_ = MV_CC_SetFloatValue(handle, "AutoGainLowerLimit", configValue1);
    err_ = MV_CC_SetFloatValue(handle, "AutoGainUpperLimit", configValue2);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_SetFloatValue failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "AutoGain [" << currentAutoGainLowerLimit.fCurValue << ", " << currentAutoGainUpperLimit.fCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    MVCC_ENUMVALUE currentExposureAuto = {0};
    err_ = MV_CC_GetEnumValue(handle, "ExposureAuto", &currentExposureAuto);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_GetEnumValue failed, err_: " << err_;
        exit(1);
    }
    fs["ExposureAuto"] >> configValue1;
    err_ = MV_CC_SetEnumValue(handle, "ExposureAuto", configValue1);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_SetEnumValue failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "ExposureAuto: " << currentExposureAuto.nCurValue << " --> " << configValue1;

    MVCC_INTVALUE currentAutoExposureTimeLowerLimit = {0};
    MVCC_INTVALUE currentAutoExposureTimeUpperLimit = {0};
    err_ = MV_CC_GetIntValue(handle, "AutoExposureTimeLowerLimit", &currentAutoExposureTimeLowerLimit);
    err_ = MV_CC_GetIntValue(handle, "AutoExposureTimeUpperLimit", &currentAutoExposureTimeUpperLimit);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_GetIntValue failed, err_: " << err_;
        exit(1);
    }
    fs["AutoExposureTimeLowerLimit"] >> configValue1;
    fs["AutoExposureTimeUpperLimit"] >> configValue2;
    err_ = MV_CC_SetIntValue(handle, "AutoExposureTimeLowerLimit", configValue1);
    err_ = MV_CC_SetIntValue(handle, "AutoExposureTimeUpperLimit", configValue2);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_SetIntValue failed, err_: " << err_;
        exit(1);
    }
    LOG(INFO) << "AutoExposureTime [" << currentAutoExposureTimeLowerLimit.nCurValue << ", " << currentAutoExposureTimeUpperLimit.nCurValue << "] --> [" << configValue1 << ", " << configValue2 << "]";

    fs.release();
}

void HikCamera::RegisterCB() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(const auto &handle: handles_) {
        err_ = MV_CC_RegisterImageCallBackEx(handle, ImageCB, handle);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_RegisterImageCallBackEx failed, err_: " << err_;
            exit(1);
        }

        err_ = MV_CC_StartGrabbing(handle);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_StartGrabbing failed, err_: " << err_;
            exit(1);
        }
    }
}

void HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
    LOG(INFO) << __FUNCTION__ << " start.";
    if(!pstMVDevInfo) {
        LOG(ERROR) << "The Pointer of pstMVDevInfo is NULL!";
        exit(1);
    }
    if(pstMVDevInfo->nTLayerType != MV_GIGE_DEVICE) {
        LOG(ERROR) << "Not support: " << pstMVDevInfo->nTLayerType;
        exit(1);
    }

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
    (void)PressEnterToExit();
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
    // (void)ConvertSaveImage(pData, pFrameInfo, pUser);
    (void)Convert2Mat(pData, pFrameInfo, pUser);

}

void HikCamera::Convert2Mat(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;
    void *handle = pUser;

    // unsigned char *pDataForRGB = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    // if(!pDataForRGB) {
    //     LOG(ERROR) << "Failed to allocate memory for pDataForRGB.";
    //     exit(-1);
    // }
    // s_convertParam_.nWidth = pFrameInfo->nWidth;
    // s_convertParam_.nHeight = pFrameInfo->nHeight;
    // s_convertParam_.pSrcData = pData;
    // s_convertParam_.nSrcDataLen = pFrameInfo->nFrameLen;
    // s_convertParam_.enSrcPixelType = pFrameInfo->enPixelType;
    // s_convertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    // s_convertParam_.pDstBuffer = pDataForRGB;
    // s_convertParam_.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048;
    // DLOG(INFO) << __FUNCTION__;
    // err = MV_CC_ConvertPixelType(handle, &s_convertParam_);
    // if(MV_OK != err) {
    //     LOG(ERROR) << "MV_CC_ConvertPixelType failed, err: " << err;
    //     return;
    // }
    // LOG(INFO) << "ConvertPixelType " << pFrameInfo->enPixelType << " --> " << PixelType_Gvsp_RGB8_Packed;

    cv::Mat matImage(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
    memcpy(matImage.data, pData, pFrameInfo->nWidth * pFrameInfo->nHeight * 3);
    // if(pDataForRGB) {
    //     free(pDataForRGB);
    //     pDataForRGB = NULL;
    // }
    cv::Mat matBGR(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
    cv::cvtColor(matImage, matBGR, cv::COLOR_RGB2BGR);
    cv::imshow("matBGR", matBGR);
    DLOG(INFO) << "cvtColor end.";
    // cv::imwrite("/tmp/image.jpg", matBGR);
    cv::waitKey(1);
    DLOG(INFO) << __FUNCTION__ << " end.";
}

// void HikCamera::ConvertSaveImage(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {
//     LOG(INFO) << __FUNCTION__ << " start.";
//     int err = MV_OK;
//     void *handle = pUser;

//     unsigned char *pDataForRGB = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
//     if(!pDataForRGB) {
//         LOG(ERROR) << "Failed to allocate memory for pDataForRGB.";
//         exit(-1);
//     }

//     s_convertParam_.nWidth = pFrameInfo->nWidth;
//     s_convertParam_.nHeight = pFrameInfo->nHeight;
//     s_convertParam_.pSrcData = pData;
//     s_convertParam_.nSrcDataLen = pFrameInfo->nFrameLen;
//     s_convertParam_.enSrcPixelType = pFrameInfo->enPixelType;
//     s_convertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
//     s_convertParam_.pDstBuffer = pDataForRGB;
//     s_convertParam_.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048;

//     err = MV_CC_ConvertPixelType(handle, &s_convertParam_);
//     if(MV_OK != err) {
//         LOG(ERROR) << "MV_CC_ConvertPixelType fail! err: " << err;
//         return;
//     }
//     LOG(INFO) << "ConvertPixelType " << pFrameInfo->enPixelType << " --> " << PixelType_Gvsp_RGB8_Packed;

//     // Output image format is JPEG; Compress the uncompressed image
//     tjhandle tjInstance = tjInitCompress();
//     if(NULL == tjInstance) {
//         LOG(ERROR) << "initializing compressor.";
//         exit(-1);
//     }
//     unsigned char *jpegBuf = NULL;
//     unsigned long jpegSize = 0;
//     const int outQual = 80;
//     const int pixelFormat = TJPF_RGB;
//     const int outSubsamp = TJSAMP_444;
//     const int flags = 0;
//     if(tjCompress2(tjInstance, pDataForRGB, pFrameInfo->nWidth, 0, pFrameInfo->nHeight, pixelFormat, &jpegBuf, &jpegSize, outSubsamp, outQual, flags) < 0) {
//         LOG(ERROR) << "compressing image.";
//         exit(-1);
//     }
//     if(pDataForRGB) {
//         free(pDataForRGB);
//         pDataForRGB = NULL;
//     }
//     tjDestroy(tjInstance);
//     tjInstance = NULL;

//     std::string imageName("/tmp/" + std::to_string(pFrameInfo->nFrameNum) + ".jpg");
//     FILE *jpegFile = fopen(imageName.c_str(), "wb");
//     if(!jpegFile) {
//         LOG(ERROR) << "Opening output file.";
//         exit(-1);
//     }
//     if(fwrite(jpegBuf, jpegSize, 1, jpegFile) < 1) {
//         LOG(ERROR) << "Writing output file.";
//         exit(-1);
//     }
//     fclose(jpegFile);
//     jpegFile = NULL;
//     tjFree(jpegBuf);
//     jpegBuf = NULL;

//     LOG(INFO) << "Save " << imageName;
//     return;
// }
