#include "hik_camera_arm.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>


bool isNodeRunning = true;
void CtrlCHandler(int signo) {
    LOG(INFO) << "Receive a Control+C.";
    isNodeRunning = false;
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    // signal(SIGINT, CtrlCHandler);

    HikCamera hikCamera(isNodeRunning);
    hikCamera.Run();
    LOG(INFO) << "Returning...";
    return 0;
}

