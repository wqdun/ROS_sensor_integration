#ifndef __CAMERA_FORCE_IP_H__
#define __CAMERA_FORCE_IP_H__

#include "FlyCapture2.h"
#include "Error.h"

class CameraForceIp {
public:
    CameraForceIp();
    ~CameraForceIp();

    int doForceIp();


private:
    FlyCapture2::Error error_;
    FlyCapture2::BusManager busMgr_;

    bool forceIpFor(size_t tryTimes);
    bool refreshCamerasFor(size_t tryTimes);
    int getCamerasInfo();
    void logErrorTrace(FlyCapture2::Error error);
};

#endif

