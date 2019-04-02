#include "comm_timer.h"

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 2) {
        LOG(ERROR) << "./a.out /dev/ttyUSBx...";
        return 1;
    }

    ros::init(argc, argv, "rt_imu_node");
    const std::string serialDevice(argv[1]);
    CommTimer commTimer(serialDevice);
    commTimer.Run();
    return 0;
}


