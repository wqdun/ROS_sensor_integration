#include "serial_reader.h"

int main(int argc, char const *argv[]) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if (argc < 2) {
        LOG(ERROR) << "./a.out /dev/ttyUSBx...";
        return 1;
    }

    const std::string serialDevice(argv[1]);
    SerialReader serialReader(serialDevice);
    serialReader.Write();
    // serialReader.Read();
    return 0;
}


