#include "camera_force_ip.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    CameraForceIp cameraForceIper;
    return cameraForceIper.doForceIp();
}