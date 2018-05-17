#include "camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << "parameters.";
    if(4 != argc) {
        LOG(INFO) << "usage: exec image_format path_save_image path_save_imu.";
        return -1;
    }

    ros::init(argc, argv, "image_publisher");

    Cameras camerasor(ros::NodeHandle(), ros::NodeHandle("~") );
    camerasor.imgFormat_ = argv[1];
    camerasor.rawdataPath_ = argv[2];
    LOG(INFO) << "imgFormat_: " << camerasor.imgFormat_ << "; rawdataPath_: " << camerasor.rawdataPath_;

    camerasor.run();

    return 0;
}