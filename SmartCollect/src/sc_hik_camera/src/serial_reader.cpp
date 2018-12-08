#include "serial_reader.h"

SerialReader::SerialReader(const std::string &_serialName, const std::string &_imuPath) {
    LOG(INFO) << __FUNCTION__ << " start.";
    serialName_ = _serialName;

    std::string imuFileNamePrefix("");
    (void)public_tools::PublicTools::generateFileName(_imuPath, imuFileNamePrefix);
    rtImuFile_ = _imuPath + imuFileNamePrefix + "_rt_track.txt";

    pubNovatelMsg_ = nh_.advertise<sc_msgs::Novatel>("sc_novatel", 10);
}

SerialReader::~SerialReader() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

int SerialReader::Read() {
    LOG(INFO) << __FUNCTION__ << " start.";

    fd_ = open(serialName_.c_str(), O_RDWR | O_NOCTTY);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open " << serialName_ << ", try change permission...";
        return -1;
    }

    if(public_tools::ToolsNoRos::SetSerialOption(fd_, 230400, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        return -1;
    }

    (void)ReadSerial();

    close(fd_);
    return 0;
}

int SerialReader::Write() {
    LOG(INFO) << __FUNCTION__ << " start.";

    fd_ = open(serialName_.c_str(), O_RDWR | O_NOCTTY);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open device, try change permission...";
        return -1;
    }

    if(public_tools::ToolsNoRos::SetSerialOption(fd_, 230400, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        return -1;
    }

    (void)WriteSerial();

    close(fd_);
    return 0;
}

void SerialReader::PublishMsg() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pubNovatelMsg_.publish(novatelMsg_);
}

void SerialReader::WriteSerial() {
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
        // sleep
        err = write(fd_, data2Send.c_str(), data2Send.size() );
        err = write(fd_, data2Send.c_str(), data2Send.size() );
        if(err < 0) {
            LOG(ERROR) << "Failed to send " << data2Send << ", err: " << err;
            exit(1);
        }
        LOG(INFO) << "Send " << data2Send << " successfully, bytes: " << err;
    }
}

void SerialReader::ReadSerial() {
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
        DLOG(INFO) << "novatelMsg_.seconds_into_week: " << std::fixed << novatelMsg_.seconds_into_week;
        framesBuf.erase(0, save4NextFrameIndex);
    }
}

void SerialReader::ParseFrame(const std::string &_frame, size_t _headerLength, double &gpsTime2update) {
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

void SerialReader::ParseRawimu(const std::string &inspvaxFrame, size_t _headerLength, double &_gpsTime2update) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;

    uchar2Double_t uchar2Double;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 4 + i];
    }
    _gpsTime2update = uchar2Double.doubleData;
    DLOG(INFO) << std::fixed << "_gpsTime2update: " << _gpsTime2update;
}

double SerialReader::GetGpsTime() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return novatelMsg_.seconds_into_week;
}

double SerialReader::GetUnixTimeMinusGpsTime() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return unixTimeMinusGpsTime_;
}

double SerialReader::CalcUnixTimeMinusGpsTime(double gpsWeekSec) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    struct timeval now;
    gettimeofday(&now, NULL);
    double unixTime = now.tv_sec + now.tv_usec / 1000000.;

    return (unixTime - gpsWeekSec);
}

void SerialReader::ParseInspvax(const std::string &inspvaxFrame, size_t _headerLength) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;

    uchar2int32_t uchar2int32;
    uchar2Double_t uchar2Double;

    for(size_t i = 0; i < 4; ++i) {
        uchar2int32.uCharData[i] = inspvaxFrame[16 + i];
    }
    novatelMsg_.GPS_week_sec =  static_cast<double>(uchar2int32.int32Data) / 1000.;
    unixTimeMinusGpsTime_ = CalcUnixTimeMinusGpsTime(novatelMsg_.GPS_week_sec);
    DLOG(INFO) << "unixTimeMinusGpsTime_: " << std::fixed << unixTimeMinusGpsTime_;

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

void SerialReader::WriteRtImuFile() {
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

void SerialReader::ParseBestgnsspos(const std::string &bestgnssposFrame, size_t _headerLength) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
    assert(bestgnssposFrame.size() > _headerLength + 64);
    unsigned char _svNum = bestgnssposFrame[_headerLength + 64];
    novatelMsg_.sv_num = _svNum;
    LOG(INFO) << "novatelMsg_.sv_num: " << (int)novatelMsg_.sv_num;
}

void SerialReader::ParsePsrdop(const std::string &psrdopFrame, size_t _headerLength) {
    DLOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
    assert(psrdopFrame.size() > _headerLength + 11);
    uchar2Float_t uchar2Float;
    for(size_t i = 0; i < 4; ++i) {
        uchar2Float.uCharData[i] = psrdopFrame[_headerLength + 8 + i];
    }
    novatelMsg_.hdop = uchar2Float.floatData;
    LOG(INFO) << novatelMsg_.hdop;
}

bool SerialReader::IsCheckSumWrong(const std::string &__frame) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return CalculateBlockCRC32(__frame);
}

unsigned long SerialReader::CRC32Value(int i) {
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

unsigned long SerialReader::CalculateBlockCRC32(const std::string &___frame) {
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

