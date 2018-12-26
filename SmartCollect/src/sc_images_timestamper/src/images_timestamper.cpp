#include "images_timestamper.h"

ImagesTimestamper::ImagesTimestamper(const std::string &_serialName, const std::string &_imuPath) {
    LOG(INFO) << __FUNCTION__ << " start.";
    serialName_ = _serialName;

    std::string imuFileNamePrefix("");
    (void)public_tools::PublicTools::generateFileName(_imuPath, imuFileNamePrefix);
    imagesTimestampFile_ = _imuPath + imuFileNamePrefix + "_images_timestamp.txt";
}

ImagesTimestamper::~ImagesTimestamper() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

int ImagesTimestamper::Read() {
    LOG(INFO) << __FUNCTION__ << " start.";

    fd_ = open(serialName_.c_str(), O_RDONLY | O_NOCTTY);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open " << serialName_ << ", try change permission...";
        return -1;
    }

    if(public_tools::ToolsNoRos::SetSerialOption(fd_, 9600, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        return -1;
    }

    (void)ReadSerial();

    close(fd_);
    return 0;
}

void ImagesTimestamper::ReadSerial() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const char HEADER_0 = 0xaa;
    const char HEADER_1 = 0x55;
    const size_t FRAME_LENGTH = 20;
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
    while(ros::ok() ) {
        bzero(buf, BUFFER_SIZE);
        int nread = read(fd_, buf, BUFFER_SIZE);
        LOG_EVERY_N(INFO, 50) << "nread: " << nread;
        if(nread <= 0) {
            continue;
        }

        for(int i = 0; i < nread; ++i) {
            framesBuf += buf[i];
        }

        if(framesBuf.size() < 50) {
            continue;
        }
#ifndef NDEBUG
        for(size_t i = 0; i < framesBuf.size(); ++i) {
            LOG(INFO) << i << ":" << std::hex << (int)framesBuf[i];
        }
#endif
        LOG(INFO) << "framesBuf.size(): " << framesBuf.size();
        size_t save4NextFrameIndex = framesBuf.size();
        for(size_t bufIndex = 0; bufIndex < framesBuf.size(); ++bufIndex) {
            if((HEADER_0 == framesBuf[bufIndex]) && (HEADER_1 == framesBuf[bufIndex + 1])) {
                if( (bufIndex + FRAME_LENGTH) > framesBuf.size() ) {
                    LOG(INFO) << "The last frame is not complete: " << bufIndex << ":" << FRAME_LENGTH;
                    save4NextFrameIndex = bufIndex;
                    break;
                }

                const std::string aFrame(framesBuf.substr(bufIndex, FRAME_LENGTH));
                ParseFrame(aFrame);
                DLOG(INFO) << "Avoid unnecessary loop: " << bufIndex << ":" << FRAME_LENGTH;
                bufIndex += FRAME_LENGTH;
                --bufIndex;
            } // else continue
        }
        framesBuf.erase(0, save4NextFrameIndex);
    }
}

void ImagesTimestamper::ParseFrame(const std::string &_frame) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    const size_t _FRAME_LENGTH = 20;

#ifndef NDEBUG
    for(auto &c: _frame) {
        LOG(INFO) << std::hex << (int)c;
    }
#endif
    if(IsCheckSumWrong(_frame)) {
        LOG(ERROR) << "CheckSumWrong.";
        return;
    }

    assert(_FRAME_LENGTH == _frame.size());

    uchar2int64_t uchar2int64;
    for(size_t i = 0; i < 8; ++i) {
        // [7] --> [2]
        uchar2int64.uCharData[i] = _frame[9 - i];
    }
    const int64_t imagesId = uchar2int64.int64Data;

    uchar2Uint32_t uchar2Uint32;
    for(size_t i = 0; i < 4; ++i) {
        // [3] --> [14]
        uchar2Uint32.uCharData[i] = _frame[13 - i];
    }
    const int32_t imagesGpsTimeSec = uchar2Uint32.uint32Data;
    for(size_t i = 0; i < 4; ++i) {
        // [3] --> [10]
        uchar2Uint32.uCharData[i] = _frame[17 - i];
    }
    const uint32_t imagesGpsTimeNsec = uchar2Uint32.uint32Data;
    LOG(INFO) << "imagesGpsTimeNsec: " << imagesGpsTimeNsec;
    const double imagesGpsTime = imagesGpsTimeSec + imagesGpsTimeNsec / 100000000.;

    std::fstream file(imagesTimestampFile_, std::ios::out | std::ios::app);
    if(!file) {
        LOG(ERROR) << "Failed to open " << imagesTimestampFile_;
        return;
    }

    file << std::fixed
         << imagesId << ","
         << imagesGpsTime << "\n";

    file.close();
}

bool ImagesTimestamper::IsCheckSumWrong(const std::string &__frame) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    unsigned long ulCount = __frame.size();
    uint16_t sum = 0;
    for(size_t i = 2; i < ulCount - 2; ++i) {
        sum += static_cast<unsigned char>(__frame[i]);
    }

    uchar2Ushort_t uchar2Ushort;
    uchar2Ushort.uCharData[0] = __frame[ulCount - 1];
    uchar2Ushort.uCharData[1] = __frame[ulCount - 2];
    const uint16_t chkSum = uchar2Ushort.uShortData;
    sum += chkSum;
    DLOG(INFO) << "Sum: " << sum;
    return (sum != 0xFFFF);
}


