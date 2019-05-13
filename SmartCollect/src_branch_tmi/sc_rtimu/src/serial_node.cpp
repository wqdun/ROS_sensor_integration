#include <glog/logging.h>

#include "serial_factory.h"

int main(int argc, char const *argv[]) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 2) {
        LOG(ERROR) << "./a.out /dev/ttyUSBx...";
        return 1;
    }

    SerialFactory serialFactory;
    BaseSerial *pSerial = serialFactory.CreateSerial("a1");

    pSerial->SetSerialDevice();
    pSerial->SetBaudRate();
    pSerial->SetOutputPath();

    pSerial->Run();
    return 0;
}


