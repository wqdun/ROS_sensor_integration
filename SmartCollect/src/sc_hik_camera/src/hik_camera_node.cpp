#include "hik_camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";

    ros::init(argc, argv, "hik_camera_node");
    HikCamera hikCamera(ros::NodeHandle(), ros::NodeHandle("~"));
    hikCamera.Run();
    LOG(INFO) << "Returning...";
    return 0;
}
