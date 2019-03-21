#include "rawimu_recorder.h"

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 3) {
        LOG(ERROR) << "./a.out /dev/ttyUSBx RawdataImuPath.";
        return 1;
    }
    ros::init(argc, argv, "rawimu_recorder_node");

    const std::string serialDevice(argv[1]);
    const std::string rawdataImuPath(argv[2]);

    RawimuRecorder rawimuRecorder(serialDevice, rawdataImuPath);
    rawimuRecorder.Run();
    return 0;
}


