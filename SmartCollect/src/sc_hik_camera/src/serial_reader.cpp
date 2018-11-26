#include "serial_reader.h"

SerialReader::SerialReader() {
    LOG(INFO) << __FUNCTION__ << " start.";
    isSerialRunning_ = true;
    slam10Datas_.resize(50);

    // pCanParser_.reset(new CanParser() );

    for(auto &slamData: slam10Datas_) {
        slamData = {0};
    }
}

SerialReader::~SerialReader() {
    LOG(INFO) << __FUNCTION__ << " start.";
    close(fd_);
    // pCanParser_->isCanParserRunning_ = false;
    // pCanParserThread_->join();
}

int SerialReader::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    // pCanParserThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CanParser::Run, pCanParser_) ) );

    fd_ = open("/dev/ttyUSB0", O_RDWR);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open device, try change permission...";
        return -1;
    }

    if(public_tools::ToolsNoRos::setSerialOption(fd_, 230400, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        return -1;
    }

    // (void)WriteSerial();
    (void)ReadSerial();
    return 0;
}

void SerialReader::WriteSerial() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = -1;
    const std::vector<std::string> cmds = {
        "$cmd,output,com0,null*ff",
        "$cmd,output,com0,gpfpd,0.01*ff",
        "$cmd,output,com0,gtimu,0.01*ff",
    };
    for(const auto &data2Send: cmds) {
        err = write(fd_, data2Send.c_str(), data2Send.size() );
        if(err < 0) {
            LOG(ERROR) << "Failed to send " << data2Send << ", err: " << err;
            exit(1);
        }
        LOG(INFO) << "Send " << data2Send << " successfully, bytes: " << err;
    }
}

void SerialReader::GetPositionFromGpfpd(const std::string &gpfpd, std::string &position) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    std::vector<std::string> gpfpdParsed;
    boost::split(gpfpdParsed, gpfpd, boost::is_any_of( ",*" ) );
    if(18 != gpfpdParsed.size() ) {
        LOG(ERROR) << "Error parsing " << gpfpd << gpfpdParsed.size();
        return;
    }
    position = gpfpdParsed[7] + "," + gpfpdParsed[8] + "," + gpfpdParsed[9];
    return;
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

    while(isSerialRunning_) {
        bzero(buf, BUFFER_SIZE);
        int nread = read(fd_, buf, BUFFER_SIZE);
        if(nread <= 0) {
            LOG_EVERY_N(INFO, 10) << "nread: " << nread;
            continue;
        }

        for(int i = 0; i < nread; ++i) {
            framesBuf += buf[i];
        }

        if(framesBuf.size() < 200) {
            continue;
        }
#ifndef NDEBUG
        for(size_t i = 0; i < framesBuf.size(); ++i) {
            LOG(INFO) << i << ":" << std::hex << (int)framesBuf[i];
        }
#endif
        LOG(INFO) << "framesBuf.size(): " << framesBuf.size();
        size_t save4NextFrameIndex = framesBuf.size();
        for(size_t bufIndex = 0; bufIndex < framesBuf.size() - 8; ++bufIndex) {
            if( (bufIndex + 8) >= framesBuf.size() ) {
                LOG(INFO) << "Reach end, bufIndex: " << bufIndex;
                save4NextFrameIndex = bufIndex;
                break;
            }
            if((HEADER_0 == framesBuf[bufIndex]) && (HEADER_1 == framesBuf[bufIndex + 1]) && (HEADER_2_LONG_FRAME == framesBuf[bufIndex + 2])) {
                const unsigned char headerLength = framesBuf[bufIndex + 3];
                const unsigned char messageLength = framesBuf[bufIndex + 8];
                const size_t frameLength = headerLength + messageLength + CHECKSUM_LENGTH;
                LOG(INFO) << "bufIndex: " << bufIndex << "; frameLength: " << frameLength << "; headerLength: " << (int)headerLength << "; messageLength: " << (int)messageLength;
                if( (bufIndex + frameLength) > framesBuf.size() ) {
                    LOG(INFO) << "The last frame is not complete: " << bufIndex << ":" << frameLength;
                    save4NextFrameIndex = bufIndex;
                    break;
                }

                const std::string aFrame(framesBuf.substr(bufIndex, frameLength));
                ParseFrame(aFrame, headerLength);
                LOG(INFO) << "Avoid unnecessary loop: " << bufIndex << ":" << frameLength;
                bufIndex += frameLength;
                --bufIndex;
            } // else continue
        }
        framesBuf.erase(0, save4NextFrameIndex);
    }
}

void SerialReader::ParseFrame(const std::string &_frame, size_t _headerLength) {
    LOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
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
    LOG(INFO) << "messageID: " << messageID;
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
        ParseRawimu(_frame, _headerLength);
        break;
    default:
        LOG(ERROR) << "Unhandled messageID: " << messageID;
    }
}

void SerialReader::ParseRawimu(const std::string &inspvaxFrame, size_t _headerLength) {
    LOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;

    uchar2Double_t uchar2Double;

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 4 + i];
    }
    novatelMsg_.secondsIntoWeek = uchar2Double.doubleData;

    LOG(INFO) << std::fixed << novatelMsg_.secondsIntoWeek;
}



void SerialReader::ParseInspvax(const std::string &inspvaxFrame, size_t _headerLength) {
    LOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;

    uchar2Double_t uchar2Double;

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
    novatelMsg_.northVel = uchar2Double.doubleData;
    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 44 + i];
    }
    novatelMsg_.eastVel = uchar2Double.doubleData;
    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 52 + i];
    }
    novatelMsg_.upVel = uchar2Double.doubleData;
    novatelMsg_.absVel = sqrt(novatelMsg_.northVel * novatelMsg_.northVel + novatelMsg_.eastVel * novatelMsg_.eastVel + novatelMsg_.upVel * novatelMsg_.upVel);

    for(size_t i = 0; i < 8; ++i) {
        uchar2Double.uCharData[i] = inspvaxFrame[_headerLength + 76 + i];
    }
    novatelMsg_.azimuth = uchar2Double.doubleData;

    LOG(INFO) << novatelMsg_;
}


void SerialReader::ParseBestgnsspos(const std::string &bestgnssposFrame, size_t _headerLength) {
    LOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
    assert(bestgnssposFrame.size() > _headerLength + 64);
    unsigned char _svNum = bestgnssposFrame[_headerLength + 64];
    novatelMsg_.svNum = _svNum;
    LOG(INFO) << "novatelMsg_.svNum: " << (int)novatelMsg_.svNum;
}

void SerialReader::ParsePsrdop(const std::string &psrdopFrame, size_t _headerLength) {
    LOG(INFO) << __FUNCTION__ << " start, _headerLength: " << _headerLength;
    assert(psrdopFrame.size() > _headerLength + 11);
    uchar2Float_t uchar2Float;
    for(size_t i = 0; i < 8; ++i) {
        uchar2Float.uCharData[i] = psrdopFrame[_headerLength + 8 + i];
    }
    novatelMsg_.hdop = uchar2Float.floatData;
    LOG(INFO) << novatelMsg_.hdop;
}

bool SerialReader::IsCheckSumWrong(const std::string &__frame) {
    LOG(INFO) << __FUNCTION__ << " start.";
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

void SerialReader::Parse2SlamData(const std::string &_slamProtocol) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    std::vector<std::string> slamParsed;
    // 1539608480.750918,$GTIMU,0,2083.510,0.0300,-0.3019,0.1624,-0.2248,0.0010,0.9730,0,52.7*46,0.0000000,0.0000000,0.00
    boost::split(slamParsed, _slamProtocol, boost::is_any_of(",") );
    if(15 != slamParsed.size() ) {
        LOG(ERROR) << "Error parsing " << _slamProtocol << ", size: " << slamParsed.size();
        return;
    }
    slamData_.unixTime = public_tools::ToolsNoRos::string2double(slamParsed[0]);
    slamData_.gpsTime = public_tools::ToolsNoRos::string2double(slamParsed[3]);
    slamData_.gyroX = public_tools::ToolsNoRos::string2double(slamParsed[4]) / 180 * 3.141592654;
    slamData_.gyroY = public_tools::ToolsNoRos::string2double(slamParsed[5]) / 180 * 3.141592654;
    slamData_.gyroZ = public_tools::ToolsNoRos::string2double(slamParsed[6]) / 180 * 3.141592654;
    slamData_.accX = public_tools::ToolsNoRos::string2double(slamParsed[7]) * 9.81007;
    slamData_.accY = public_tools::ToolsNoRos::string2double(slamParsed[8]) * 9.81007;
    slamData_.accZ = public_tools::ToolsNoRos::string2double(slamParsed[9]) * 9.81007;
    slamData_.lat = public_tools::ToolsNoRos::string2double(slamParsed[12]);
    slamData_.lon = public_tools::ToolsNoRos::string2double(slamParsed[13]);
    slamData_.hei = public_tools::ToolsNoRos::string2double(slamParsed[14]);
    DLOG(INFO) << slamData_.lon << ":" << slamData_.lat;
    public_tools::ToolsNoRos::GeoToGauss(slamData_.lon, slamData_.lat, 3, 39, (slamData_.east), (slamData_.north));
    // slamData_.encoder_v = pCanParser_->decimalResult_;
    slamData_.yaw = 0;
    slamData_.pitch = 0;
    slamData_.roll = 0;

    slam10DatasMutex_.lock();
    slam10Datas_.emplace_back(slamData_);
    slam10Datas_.pop_front();
    slam10DatasMutex_.unlock();
}
