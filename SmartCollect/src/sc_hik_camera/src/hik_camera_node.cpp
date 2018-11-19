#include "hik_camera_manager.h"

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";

    ros::init(argc, argv, "hik_camera_node");
    HikCameraManager hikCameraBoss;
    hikCameraBoss.Run();
    LOG(INFO) << "Returning...";
    return 0;
}
