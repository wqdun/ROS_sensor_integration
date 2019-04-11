#ifndef __CAMERA_FORCE_IP_H__
#define __CAMERA_FORCE_IP_H__

#include <fcntl.h>
#include "FlyCapture2.h"
#include "Error.h"

class CameraForceIp {
public:
    CameraForceIp();
    ~CameraForceIp();

    int getCamerasInfo();
    int doForceIp();


private:
    FlyCapture2::Error error_;
    FlyCapture2::BusManager busMgr_;

    bool forceIpFor(size_t tryTimes);
    bool refreshCamerasFor(size_t tryTimes);
    void logErrorTrace(FlyCapture2::Error error);
};

#endif

