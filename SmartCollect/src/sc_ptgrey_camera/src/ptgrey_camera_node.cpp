#include <ros/ros.h>
#include "ptgrey_camera_manager.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if(argc < 2) {
        LOG(ERROR) << "usage: exec path_rawdata.";
        return -1;
    }

    ros::init(argc, argv, "ptgrey_camera_node");
    const std::string rawPath(argv[1]);
    LOG(INFO) << "rawPath: " << rawPath;
    PtgreyCameraManager ptgreyCameraBoss(rawPath);
    ptgreyCameraBoss.Run();
    LOG(INFO) << "Returning...";
    return 0;
}
