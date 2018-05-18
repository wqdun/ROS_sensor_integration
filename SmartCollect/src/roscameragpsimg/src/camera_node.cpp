#include "camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if(3 != argc) {
        LOG(INFO) << "usage: exec image_format path_rawdata.";
        return -1;
    }

    ros::init(argc, argv, "image_publisher");

    const std::string imgFormat(argv[1]);
    const std::string rawPath(argv[2]);
    LOG(INFO) << "imgFormat: " << imgFormat << "; rawPath: " << rawPath;

    Cameras camerasor(ros::NodeHandle(), ros::NodeHandle("~"), imgFormat, rawPath);
    camerasor.run();

    return 0;
}