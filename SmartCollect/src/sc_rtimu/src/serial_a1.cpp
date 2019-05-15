#include "serial_a1.h"

void SerialA1::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pubNovatelMsg_ = nh_.advertise<sc_msgs::Novatel>("sc_novatel", 1);

    fd_ = open(serialName_.c_str(), O_RDWR | O_NOCTTY);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open device, try change permission...";
        exit(1);
    }
    if(public_tools::ToolsNoRos::SetSerialOption(fd_, baudRate_, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        exit(1);
    }

    WriteSerial();
    ReadSerial();

    close(fd_);
}

void SerialA1::PublishMsg() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pubNovatelMsg_.publish(novatelMsg_);
}

int SerialA1::WriteSerial() {
    LOG(INFO) << __FUNCTION__ << " start.";

    int err = -1;
    const std::vector<std::string> cmds = {
        // "freset\r",
        "unlogall usb1\r",
        "unlogall usb2\r",
        "unlogall usb3\r",
        "log usb1 rangecmpb ontime 1\r",
        "log usb1 rawephemb onchanged\r",
        "log usb1 gloephemerisb onchanged\r",
        "log usb1 bdsephemerisb onchanged\r",
        "log usb1 rawimub onnew\r",
        "log usb2 bestgnssposb ontime 1\r",
        "log usb2 psrdopb onchanged\r",
        "log usb2 inspvaxb ontime 1\r",
    };
    for(const auto &data2Send: cmds) {
        err = write(fd_, data2Send.c_str(), data2Send.size() );
        usleep(10000);
        // err = write(fd_, data2Send.c_str(), data2Send.size() );
        if(err < 0) {
            LOG(ERROR) << "Failed to send " << data2Send << ", err: " << err;
            exit(1);
        }
        LOG(INFO) << "Send " << data2Send << " successfully, bytes: " << err;
    }

    return 0;
}

void SerialA1::ReadSerial() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const char HEADER_0 = 0xaa;
    const char HEADER_1 = 0x44;
    const char HEADER_2_LONG_FRAME = 0x12;
    const char HEADER_2_SHORT_FRAME = 0x13;
    const size_t CHECKSUM_LENGTH = 4;
    const size_t BUFFER_SIZE = 1000;
    unsigned char buf[BUFFER_SIZE];
    std::string framesBuf("");
    int err = tcflush(fd_, TCIOFLUSH);
    LOG(INFO) << "tcflush: " << err;
    err = tcflush(fd_, TCIFLUSH);
    LOG(INFO) << "tcflush: " << err;
    err = tcflush(fd_, TCOFLUSH);
    LOG(INFO) << "tcflush: " << err;
    while(1) {
        bzero(buf, BUFFER_SIZE);
        int nread = read(fd_, buf, BUFFER_SIZE);
        LOG_EVERY_N(INFO, 50) << "nread: " << nread;
        if(nread <= 0) {
            continue;
        }

        for(int i = 0; i < nread; ++i) {
            framesBuf += buf[i];
        }

        if(framesBuf.size() < 100) {
            continue;
        }

        struct timeval now;
        gettimeofday(&now, NULL);
        double unixTimeWhenIGetTheFrame = now.tv_sec + now.tv_usec / 1000000.;
#ifndef NDEBUG
        for(size_t i = 0; i < framesBuf.size(); ++i) {
            LOG(INFO) << i << ":" << std::hex << (int)framesBuf[i];
        }
#endif
        LOG(INFO) << "framesBuf.size(): " << framesBuf.size();
        size_t save4NextFrameIndex = framesBuf.size();
        double latestGpsTime = -1;
        for(size_t bufIndex = 0; bufIndex < framesBuf.size() - 8; ++bufIndex) {
            if((HEADER_0 == framesBuf[bufIndex]) && (HEADER_1 == framesBuf[bufIndex + 1]) && (HEADER_2_LONG_FRAME == framesBuf[bufIndex + 2])) {
                const unsigned char headerLength = framesBuf[bufIndex + 3];
                const unsigned char messageLength = framesBuf[bufIndex + 8];
                const size_t frameLength = headerLength + messageLength + CHECKSUM_LENGTH;
                DLOG(INFO) << "bufIndex: " << bufIndex << "; frameLength: " << frameLength << "; headerLength: " << (int)headerLength << "; messageLength: " << (int)messageLength;
                if( (bufIndex + frameLength) > framesBuf.size() ) {
                    LOG(INFO) << "The last frame is not complete: " << bufIndex << ":" << frameLength;
                    save4NextFrameIndex = bufIndex;
                    break;
                }

                const std::string aFrame(framesBuf.substr(bufIndex, frameLength));
                ParseFrame(aFrame, headerLength, latestGpsTime);
                DLOG(INFO) << "Avoid unnecessary loop: " << bufIndex << ":" << frameLength;
                bufIndex += frameLength;
                --bufIndex;
            } // else continue
        }
        if(latestGpsTime > 0) {
            DLOG(INFO) << "GPS time updated: " << latestGpsTime;
            novatelMsg_.seconds_into_week = latestGpsTime;
        }

        unixTimeMinusGpsTimeQueue_.push_back(unixTimeWhenIGetTheFrame - novatelMsg_.GPS_week_sec);
        if (unixTimeMinusGpsTimeQueue_.size() > 50) {
            unixTimeMinusGpsTimeQueue_.pop_front();
        }
        novatelMsg_.unix_time_minus_gps_time = FillerDeque(unixTimeMinusGpsTimeQueue_);

        DLOG(INFO) << "novatelMsg_.seconds_into_week: " << std::fixed << novatelMsg_.seconds_into_week;
        framesBuf.erase(0, save4NextFrameIndex);

        PublishMsg();
    }
}

void SerialA1::ParseFrame(const std::string &_frame, size_t _headerLength, double &gpsTime2update) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
#ifndef NDEBUG
    for(auto &c: _frame) {
        LOG(INFO) << std::hex << (int)c;
    }
#endif
    if(IsCheckSumWrong(_frame)) {
        LOG(ERROR) << "CheckSumWrong.";
        return;
    }

    assert(_frame.size() > 5);
    const unsigned short MESSAGE_ID_PSRDOP = 174;
    const unsigned short MESSAGE_ID_BESTGNSSPOS = 1429;
    const unsigned short MESSAGE_ID_INSPVAX = 1465;
    const unsigned short MESSAGE_ID_RAWIMU = 268;

    uchar2Ushort_t uchar2Ushort;
    uchar2Ushort.uCharData[0] = _frame[4];
    uchar2Ushort.uCharData[1] = _frame[5];
    unsigned short messageID = uchar2Ushort.uShortData;
    LOG_EVERY_N(INFO, 50) << "messageID: " << messageID;
    switch(messageID) {
    case MESSAGE_ID_PSRDOP:
        ParsePsrdop(_frame, _headerLength);
        break;
    case MESSAGE_ID_BESTGNSSPOS:
        ParseBestgnsspos(_frame, _headerLength);
        break;
    case MESSAGE_ID_INSPVAX:
        ParseInspvax(_frame, _headerLength);
        break;
    case MESSAGE_ID_RAWIMU:
        ParseRawimu(_frame, _headerLength, gpsTime2update);
        break;
    default:
        LOG(ERROR) << "Unhandled messageID: " << messageID;
    }
}

void SerialA1::ParseRawimu(const std::string &inspvaxFrame, size_t _headerLength, double &_gpsTime2update) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;

    uchar2Double_t uchar2Double;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 4 + i];
    }
    _gpsTime2update = uchar2Double.doubleData;
    DLOG(INFO) << std::fixed << "_gpsTime2update: " << _gpsTime2update;
}

double SerialA1::GetUnixTimeMinusGpsTime() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return unixTimeMinusGpsTime_;
}

double SerialA1::CalcUnixTimeMinusGpsTime(double gpsWeekSec) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    struct timeval now;
    gettimeofday(&now, NULL);
    double unixTime = now.tv_sec + now.tv_usec / 1000000.;

    return (unixTime - gpsWeekSec);
}

void SerialA1::ParseInspvax(const std::string &inspvaxFrame, size_t _headerLength) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;

    uchar2int32_t uchar2int32;
    uchar2Double_t uchar2Double;

    for(size_t i = 0; i < 4; ++i) {
        uchar2int32.uCharData[i] = inspvaxFrame[16 + i];
    }
    novatelMsg_.GPS_week_sec =  static_cast<double>(uchar2int32.int32Data) / 1000.;

    for(size_t i = 0; i < 4; ++i) {
        uchar2int32.uCharData[i] = inspvaxFrame[_headerLength + i];
    }
    novatelMsg_.ins_status = uchar2int32.int32Data;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 8 + i];
    }
    novatelMsg_.latitude = uchar2Double.doubleData;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 16 + i];
    }
    novatelMsg_.longitude = uchar2Double.doubleData;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 24 + i];
    }
    novatelMsg_.height = uchar2Double.doubleData;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 36 + i];
    }
    novatelMsg_.north_vel = uchar2Double.doubleData;
    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 44 + i];
    }
    novatelMsg_.east_vel = uchar2Double.doubleData;
    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 52 + i];
    }
    novatelMsg_.up_vel = uchar2Double.doubleData;
    novatelMsg_.abs_vel = sqrt(novatelMsg_.north_vel * novatelMsg_.north_vel + novatelMsg_.east_vel * novatelMsg_.east_vel + novatelMsg_.up_vel * novatelMsg_.up_vel);

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 60 + i];
    }
    novatelMsg_.roll = uchar2Double.doubleData;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 68 + i];
    }
    novatelMsg_.pitch = uchar2Double.doubleData;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 76 + i];
    }
    novatelMsg_.azimuth = uchar2Double.doubleData;

    uchar2Float_t uchar2Float;
    for(size_t i = 0; i < 4; ++i) {
        uchar2Float.uCharData[i] = inspvaxFrame[_headerLength + 116 + i];
    }
    novatelMsg_.azimuth_deviation = uchar2Float.floatData;

    (void)WriteRtImuFile();
}

void SerialA1::WriteRtImuFile() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    std::fstream file(rtImuFile_, std::ios::out | std::ios::app);
    if(!file) {
        LOG(ERROR) << "Failed to open " << rtImuFile_;
        return;
    }

    file << std::fixed
         << novatelMsg_.GPS_week_sec << ","
         << novatelMsg_.azimuth << ","
         << novatelMsg_.pitch << ","
         << novatelMsg_.roll << ","
         << novatelMsg_.latitude << ","
         << novatelMsg_.longitude << ","
         << novatelMsg_.height << ","
         << novatelMsg_.east_vel << ","
         << novatelMsg_.north_vel << ","
         << novatelMsg_.up_vel << ","
         << static_cast<int>(novatelMsg_.sv_num) << "\n";

    file.close();
    return;
}

void SerialA1::ParseBestgnsspos(const std::string &bestgnssposFrame, size_t _headerLength) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
    assert(bestgnssposFrame.size() > _headerLength + 64);
    unsigned char _svNum = bestgnssposFrame[_headerLength + 64];
    novatelMsg_.sv_num = _svNum;
    LOG(INFO) << "novatelMsg_.sv_num: " << (int)novatelMsg_.sv_num;
}

void SerialA1::ParsePsrdop(const std::string &psrdopFrame, size_t _headerLength) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
    assert(psrdopFrame.size() > _headerLength + 11);
    uchar2Float_t uchar2Float;
    for(size_t i = 0; i < 4; ++i) {
        uchar2Float.uCharData[i] = psrdopFrame[_headerLength + 8 + i];
    }
    novatelMsg_.hdop = uchar2Float.floatData;
    LOG(INFO) << novatelMsg_.hdop;
}

bool SerialA1::IsCheckSumWrong(const std::string &__frame) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return CalculateBlockCRC32(__frame);
}

unsigned long SerialA1::CRC32Value(int i) {
    const unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for(j = 8 ; j > 0; j--) {
        if(ulCRC & 1) {
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
        }
        else {
            ulCRC >>= 1;
        }
    }
    return ulCRC;
}

unsigned long SerialA1::CalculateBlockCRC32(const std::string &___frame) {
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    unsigned long ulCount = ___frame.size();
    for(auto &c: ___frame) {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC ^ static_cast<unsigned char>(c)) & 0xff);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return ulCRC;
}
