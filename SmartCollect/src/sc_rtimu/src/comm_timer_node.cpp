#include "comm_timer.h"

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 3) {
        LOG(ERROR) << "./a.out /dev/ttyUSBx /Path/to/IMU/";
        return 1;
    }

    ros::init(argc, argv, "rt_imu_node");
    const std::string serialDevice(argv[1]);
    const std::string imuPath(argv[2]);
    CommTimer commTimer(serialDevice, imuPath);
    commTimer.Run();
    return 0;
}


