#include "hik_camera_arm.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
// #define __USE_HIK_API_SAVING_JPG__

HikCamera::HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_OK;
    handle_ = NULL;
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

HikCamera::~HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)doClean();
}

void HikCamera::enumGigeDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_CC_EnumDevices(MV_GIGE_DEVICE, &deviceList_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_EnumDevices failed, err_: " << err_;
        exit(1);
    }
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
        (void)printDeviceInfo(pDeviceInfo);
    }
}

void HikCamera::open_configDevices() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(size_t i = 0; i < deviceList_.nDeviceNum; ++i) {
        err_ = MV_CC_CreateHandle(&handle_, deviceList_.pDeviceInfo[i]);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_CreateHandle failed, err_: " << err_ << "; i: " << i;
            exit(1);
        }

        err_ = MV_CC_OpenDevice(handle_);
        if(MV_OK != err_) {
            LOG(ERROR) << "MV_CC_OpenDevice failed, err_: " << err_ << "; i: " << i;
            exit(1);
        }

        (void)configDevices(i);
    }
}

void HikCamera::configDevices(size_t index) {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Set trigger mode as off.";
    err_ = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_SetTriggerMode failed, err_: " << err_;
        exit(1);
    }

    LOG(INFO) << "Detection network optimal package size.";
    if(deviceList_.pDeviceInfo[index]->nTLayerType != MV_GIGE_DEVICE) {
        LOG(INFO) << "It only works for the GigE camera.";
        return;
    }
    int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
    LOG(INFO) << "Detection network optimal package size, nPacketSize: " << nPacketSize;
    if(nPacketSize <= 0) {
        LOG(ERROR) << "Get Packet Size failed, nPacketSize: " << nPacketSize;
        exit(1);
    }
    err_ = MV_CC_SetIntValue(handle_, "GevSCPSPacketSize", nPacketSize);
    if(err_ != MV_OK) {
        LOG(ERROR) << "Set Packet Size failed, err_: " << err_;
        exit(1);
    }
}

void HikCamera::registerCB() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_CC_RegisterImageCallBackEx(handle_, imageCB, handle_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_RegisterImageCallBackEx failed, err_: " << err_;
        exit(1);
    }

    err_ = MV_CC_StartGrabbing(handle_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_StartGrabbing failed, err_: " << err_;
        exit(1);
    }
}

void HikCamera::printDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
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

void HikCamera::run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)enumGigeDevices();
    (void)open_configDevices();
    (void)registerCB();
    (void)PressEnterToExit();
}


void HikCamera::doClean() {
    LOG(INFO) << __FUNCTION__ << " start.";
    err_ = MV_CC_StopGrabbing(handle_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_StopGrabbing failed, err_: " << err_;
        exit(1);
    }

    err_ = MV_CC_CloseDevice(handle_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_CloseDevice failed, err_: " << err_;
        exit(1);
    }

    err_ = MV_CC_DestroyHandle(handle_);
    if(MV_OK != err_) {
        LOG(ERROR) << "MV_CC_DestroyHandle failed, err_: " << err_;
        exit(1);
    }
    handle_ = NULL;
}

void HikCamera::PressEnterToExit(void)
{
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

void __stdcall HikCamera::imageCB(unsigned char *pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }
    LOG(INFO) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: " << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight;
}

