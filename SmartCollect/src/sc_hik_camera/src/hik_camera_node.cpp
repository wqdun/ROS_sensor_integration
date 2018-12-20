#include "hik_camera_manager.h"

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if(argc < 2) {
        LOG(ERROR) << "usage: exec path_rawdata.";
        return -1;
    }

    ros::init(argc, argv, "hik_camera_node");
    const std::string rawPath(argv[1]);
    LOG(INFO) << "rawPath: " << rawPath;
    HikCameraManager hikCameraBoss(rawPath);
    hikCameraBoss.Run();
    LOG(INFO) << "Returning...";
    return 0;
}
