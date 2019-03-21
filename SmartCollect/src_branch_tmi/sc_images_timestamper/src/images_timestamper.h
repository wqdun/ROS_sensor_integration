#ifndef __IMAGES_TIMESTAMPER_H__
#define __IMAGES_TIMESTAMPER_H__

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
    uint32_t uint32Data;
} uchar2Uint32_t;

typedef union {
    unsigned char uCharData[8];
    int64_t int64Data;
} uchar2int64_t;

typedef union {
    unsigned char uCharData[4];
    float floatData;
} uchar2Float_t;

typedef union {
    unsigned char uCharData[8];
    double doubleData;
} uchar2Double_t;


class ImagesTimestamper {
public:
    ImagesTimestamper(const std::string &_serialName, const std::string &_imuPath = "");
    ~ImagesTimestamper();
    int Read();
    int Write();


private:
    int fd_;
    std::string serialName_;
    std::string imagesTimestampFile_;

    void WriteSerial();
    void ReadSerial();
    void ParseFrame(const std::string &_frame);
    void ParsePsrdop(const std::string &psrdopFrame, size_t _headerLength);
    void ParseBestgnsspos(const std::string &bestgnssposFrame, size_t _headerLength);
    void ParseInspvax(const std::string &inspvaxFrame, size_t _headerLength);
    void WriteRtImuFile();
    void ParseRawimu(const std::string &inspvaxFrame, size_t _headerLength, double &gpsTime2update);
    bool IsCheckSumWrong(const std::string &__frame);
    double CalcUnixTimeMinusGpsTime(double gpsWeekSec);
};

#endif
