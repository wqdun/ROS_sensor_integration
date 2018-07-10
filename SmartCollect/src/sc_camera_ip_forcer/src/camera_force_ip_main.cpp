#include "camera_force_ip.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    FLAGS_log_dir = "/opt/smartc/log/";
    google::InitGoogleLogging(argv[0]);

    CameraForceIp cameraForceIper;

    const size_t TRY_TIMES = 3;
    const int CAMERA_NUM = 6;
    int cameraNum = -1;
    for(size_t i = 0; i < TRY_TIMES; ++i) {
        cameraNum = cameraForceIper.doForceIp();
        if(cameraNum != CAMERA_NUM) {
            LOG(WARNING) << "i: " << i << "; got " << cameraNum << " cameras.";
            continue;
        }

        LOG(INFO) << "i: " << i << "; got " << cameraNum << " cameras successfully.";
        return cameraNum;
    }

    LOG(ERROR) << "Got only " << cameraNum << " cameras.";
    return cameraNum;
}

