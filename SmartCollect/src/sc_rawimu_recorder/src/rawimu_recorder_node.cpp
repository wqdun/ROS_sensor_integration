#include "rawimu_recorder.h"

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 4) {
        LOG(ERROR) << "./a.out /dev/ttyUSBx BaudRate RawdataImuPath.";
        return 1;
    }
    ros::init(argc, argv, "rawimu_recorder_node");

    const std::string serialDevice(argv[1]);
    const int BAUD_RATE = public_tools::ToolsNoRos::string2int(argv[2]);
    const std::string rawdataImuPath(argv[3]);

    RawimuRecorder rawimuRecorder(serialDevice, rawdataImuPath);
    rawimuRecorder.Run(BAUD_RATE);
    return 0;
}


