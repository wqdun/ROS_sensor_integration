#include "hik_camera_arm.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>


bool isNodeRunning = true;
void CtrlCHandler(int signo) {
    LOG(INFO) << "Receive a Control+C.";
    isNodeRunning = !isNodeRunning;
}

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    signal(SIGINT, CtrlCHandler);

    HikCamera hikCamera(&isNodeRunning);
    hikCamera.Run();
    LOG(INFO) << "Returning...";
    return 0;
}

// sudo g++ -std=c++11 -o hik_camera_arm_node hik_camera_arm_node.cpp hik_camera_arm.cpp  -I/opt/mvs/include  -L/opt/mvs/lib/arm64/ -lMvCameraControl -L/opt/libjpeg-turbo/lib64 -lglog -lopencv_core -lopencv_highgui -lopencv_imgproc -Wl,-rpath,/opt/mvs/lib/arm64/ -lboost_system -lboost_thread -lopencv_imgcodecs


