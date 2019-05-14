#ifndef __BASE_SERIAL_H__
#define __BASE_SERIAL_H__

#include <glog/logging.h>
#include <string>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"

class BaseSerial {
public:
    void SetSerialDevice(const std::string &serialDevice);
    void SetBaudRate(const std::string &baudRate);
    void SetOutputPath(const std::string &outputPath);

    virtual void Run() = 0;


protected:
    std::string serialDevice_;
    int baudRate_;
    std::string outputPath_;
    std::string rtImuFile_;
};

#endif
