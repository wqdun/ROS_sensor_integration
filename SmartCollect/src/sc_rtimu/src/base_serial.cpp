#include "base_serial.h"

void BaseSerial::SetSerialDevice(const std::string &serialDevice) {
    LOG(INFO) << __FUNCTION__ << " start.";
    serialDevice_ = serialDevice;
}

void BaseSerial::SetBaudRate(const std::string &baudRate) {
    LOG(INFO) << __FUNCTION__ << " start.";
    baudRate_ = public_tools::ToolsNoRos::string2int(baudRate);
}

void BaseSerial::SetOutputPath(const std::string &outputPath) {
    LOG(INFO) << __FUNCTION__ << " start.";
    outputPath_ = outputPath;

    std::string imuFileNamePrefix("");
    (void)public_tools::PublicTools::generateFileName(outputPath_, imuFileNamePrefix);
    rtImuFile_ = outputPath_ + imuFileNamePrefix + "_rt_track.txt";
}

double BaseSerial::FillerDeque(const std::deque<double> &aDeque) {
    DLOG(INFO) << __FUNCTION__ << " start, aDeque.size(): " << aDeque.size();
    return (*std::min_element(aDeque.cbegin(), aDeque.cend()));
}

