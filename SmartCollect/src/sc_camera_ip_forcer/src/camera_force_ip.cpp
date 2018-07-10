#include "camera_force_ip.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>


CameraForceIp::CameraForceIp() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

CameraForceIp::~CameraForceIp() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

int CameraForceIp::doForceIp() {
    LOG(INFO) << __FUNCTION__ << " start.";

    if(!forceIpFor(3) ) {
        LOG(ERROR) << "Failed to force IP.";
        return -1;
    }

    if(!refreshCamerasFor(3) ) {
        LOG(ERROR) << "Failed to refresh Cameras.";
        return -2;
    }

    int cameraNum = -1;
    if( (cameraNum = getCamerasInfo() ) < 0) {
        LOG(ERROR) << "Failed to get Cameras.";
        return -3;
    }

    LOG(INFO) << "Got " << cameraNum  << " cameras.";
    return cameraNum;
}

int CameraForceIp::getCamerasInfo() {
    LOG(INFO) << __FUNCTION__ << " start.";

    unsigned int numCameras = 0;
    error_ = busMgr_.GetNumOfCameras(&numCameras);
    if(error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "Error getting number of cameras.";
        logErrorTrace(error_);
        return -1;
    }

    return numCameras;
}

bool CameraForceIp::refreshCamerasFor(size_t tryTimes) {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(size_t i = 0; i < tryTimes; ++i) {
        error_ = busMgr_.RescanBus();
        if(error_ != FlyCapture2::PGRERROR_OK) {
            LOG(ERROR) << "Error rescanning bus.";
            logErrorTrace(error_);
            continue;
        }

        LOG(INFO) << "Rescanning bus successfully.";
        return true;
    }

    return false;
}

bool CameraForceIp::forceIpFor(size_t tryTimes) {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(size_t i = 0; i < tryTimes; ++i) {
        error_ = FlyCapture2::BusManager::ForceAllIPAddressesAutomatically();
        if(error_ != FlyCapture2::PGRERROR_OK) {
            LOG(ERROR) << "Error forcing IP addresses.";
            logErrorTrace(error_);
            sleep(5);
            continue;
        }

        LOG(INFO) << "Forcing IP addresses successfully.";
        sleep(5);
        return true;
    }

    return false;
}

void CameraForceIp::logErrorTrace(FlyCapture2::Error error) {
    LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
}

