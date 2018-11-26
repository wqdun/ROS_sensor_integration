#ifndef __SERIAL_READER_H__
#define __SERIAL_READER_H__

#include <glog/logging.h>
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
#include "sc_msgs/Novatel.h"

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

    double east;
    double north;

    double encoder_v;
    double yaw;
    double pitch;
    double roll;
} slamProtocol_t;

typedef union {
   unsigned char uCharData[2];
   unsigned short uShortData;
} uchar2Ushort_t;

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
    SerialReader();
    ~SerialReader();
    int Run();

    bool isSerialRunning_;
    slamProtocol_t slamData_;
    std::deque<slamProtocol_t> slam10Datas_;
    // boost::shared_ptr<CanParser> pCanParser_;
    std::mutex slam10DatasMutex_;


private:
    int fd_;
    sc_msgs::Novatel novatelMsg_;
    // boost::shared_ptr<boost::thread> pCanParserThread_;

    void GetPositionFromGpfpd(const std::string &gpfpd, std::string &position);
    void WriteSerial();
    void ReadSerial();
    void Parse2SlamData(const std::string &_slamProtocol);
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
