#ifndef __SERIAL_READER_H__
#define __SERIAL_READER_H__

#include <deque>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "../../sc_lib_public_tools/src/tools_no_ros.h"

typedef struct {
    double unixTime;
    double gpsTime;

    double gyroX;
    double gyroY;
    double gyroZ;

    double accX;
    double accY;
    double accZ;

    double lat;
    double lon;
    double hei;

    double encoder_v;
} slamProtocol_t;

class SerialReader {
public:
    SerialReader();
    ~SerialReader();
    void Run();

    bool isGonnaRun_;
    slamProtocol_t slamData_;
    std::deque<slamProtocol_t> slam10Datas_;


private:
    int fd_;

    void GetPositionFromGpfpd(const std::string &gpfpd, std::string &position);
    void WriteSerial();
    void ReadSerial();
    void Parse2SlamData(const std::string &_slamProtocol);
};


#endif
