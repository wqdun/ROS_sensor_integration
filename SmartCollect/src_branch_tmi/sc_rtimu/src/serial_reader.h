#ifndef __SERIAL_READER_H__
#define __SERIAL_READER_H__

#include <glog/logging.h>
#include <ros/ros.h>
#include <deque>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/thread/thread.hpp>
#include <mutex>
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "sc_msgs/Novatel.h"

typedef union {
    unsigned char uCharData[2];
    unsigned short uShortData;
} uchar2Ushort_t;

typedef union {
    unsigned char uCharData[4];
    int32_t int32Data;
} uchar2int32_t;

typedef union {
    unsigned char uCharData[4];
    float floatData;
} uchar2Float_t;

typedef union {
    unsigned char uCharData[8];
    double doubleData;
} uchar2Double_t;


class SerialReader {
public:
    SerialReader(const std::string &_serialName, const std::string &_imuPath = "");
    ~SerialReader();
    int Read();
    int Write();
    void PublishMsg();
    double GetUnixTimeMinusGpsTime();


private:
    int fd_;
    std::string serialName_;
    sc_msgs::Novatel novatelMsg_;
    std::string rtImuFile_;
    ros::NodeHandle nh_;
    ros::Publisher pubNovatelMsg_;
    double unixTimeMinusGpsTime_;

    void WriteSerial();
    void ReadSerial();
    void ParseFrame(const std::string &_frame, size_t _headerLength, double &gpsTime2update);
    void ParsePsrdop(const std::string &psrdopFrame, size_t _headerLength);
    void ParseBestgnsspos(const std::string &bestgnssposFrame, size_t _headerLength);
    void ParseInspvax(const std::string &inspvaxFrame, size_t _headerLength);
    void WriteRtImuFile();
    void ParseRawimu(const std::string &inspvaxFrame, size_t _headerLength, double &gpsTime2update);
    void CheckSum(const std::string &__frame);
    unsigned long CRC32Value(int i);
    unsigned long CalculateBlockCRC32(const std::string &___frame);
    bool IsCheckSumWrong(const std::string &__frame);
    double CalcUnixTimeMinusGpsTime(double gpsWeekSec);
};


#endif