#include <glog/logging.h>
#include "serial_factory.h"

int main(int argc, char const *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 5) {
        LOG(ERROR) << "./a.out a1 /dev/ttyUSBx BaudRate ImuPath";
        return 1;
    }
    ros::init(argc, argv, "rtimu_node");

    SerialFactory serialFactory;
    BaseSerial *pSerial = serialFactory.CreateSerial(argv[1]);

    pSerial->SetSerialDevice(argv[2]);
    pSerial->SetBaudRate(argv[3]);
    pSerial->SetOutputPath(argv[4]);

    pSerial->Run();
    return 0;
}


