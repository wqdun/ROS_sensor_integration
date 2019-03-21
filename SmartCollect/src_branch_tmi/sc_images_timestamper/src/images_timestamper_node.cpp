#include "images_timestamper.h"

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 3) {
        LOG(ERROR) << "./a.out /dev/ttyUSB* /path/2/Rawdata/IMU/...";
        return 1;
    }

    ros::init(argc, argv, "sc_images_timestamper_node");
    const std::string serialDevice(argv[1]);
    const std::string imuPath(argv[2]);
    ImagesTimestamper imagesTimestamper(serialDevice, imuPath);
    imagesTimestamper.Read();
    return 0;
}


