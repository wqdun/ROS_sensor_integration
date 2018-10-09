#include "hik_camera_arm.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";

    HikCamera hikCamera;
    hikCamera.run();
    LOG(INFO) << "Returning...";
    return 0;
}

