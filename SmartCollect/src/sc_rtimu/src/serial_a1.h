#ifndef __SERIAL_A1_H__
#define __SERIAL_A1_H__

#include <fcntl.h>
#include "sc_msgs/Novatel.h"
#include "base_serial.h"

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


class SerialA1: public BaseSerial {
public:
    virtual ~SerialA1() {
        LOG(INFO) << "Goodbye SerialA1..";
    }
    void Run();


private:
    int fd_;
    sc_msgs::Novatel novatelMsg_;
    ros::NodeHandle nh_;
    ros::Publisher pubNovatelMsg_;

    int WriteSerial();
    void ReadSerial();

    void PublishMsg();

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
};

#endif
