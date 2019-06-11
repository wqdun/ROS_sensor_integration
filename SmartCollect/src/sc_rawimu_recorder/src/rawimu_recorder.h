#ifndef __RAWIMU_RECORDER_H__
#define __RAWIMU_RECORDER_H__

#include <glog/logging.h>
#include <ros/ros.h>
#include <fcntl.h>
#include "sc_msgs/Novatel.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

class RawimuRecorder {
public:
    RawimuRecorder(const std::string &_serialName, const std::string &_imuPath);
    ~RawimuRecorder();
    int Read();
    int Write();
    int Run(int baudrate);


private:
    int fd_;
    std::string serialName_;
    std::string imuPath_;
    sc_msgs::Novatel novatelMsg_;

    void Record2File(const std::string &rawInsFile);
    void WriteSerial();
    void ReadSerial();
    void ParseFrame(const std::string &_frame, size_t _headerLength);
    void ParsePsrdop(const std::string &psrdopFrame, size_t _headerLength);
    void ParseBestgnsspos(const std::string &bestgnssposFrame, size_t _headerLength);
    void ParseInspvax(const std::string &inspvaxFrame, size_t _headerLength);
    void ParseRawimu(const std::string &inspvaxFrame, size_t _headerLength);

    void CheckSum(const std::string &__frame);
    unsigned long CRC32Value(int i);
    unsigned long CalculateBlockCRC32(const std::string &___frame);
    bool IsCheckSumWrong(const std::string &__frame);
};


#endif
