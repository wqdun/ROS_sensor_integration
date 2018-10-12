#include "serial_reader.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

SerialReader::SerialReader() {
    LOG(INFO) << __FUNCTION__ << " start.";
    isGonnaRun_ = true;
    fd_ = open("/dev/ttyUSB0", O_RDWR);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open device.";
        exit(1);
    }

    if(public_tools::ToolsNoRos::setSerialOption(fd_, 230400, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        exit(1);
    }
}

SerialReader::~SerialReader() {
    LOG(INFO) << __FUNCTION__ << " start.";
    close(fd_);
}

void SerialReader::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)WriteSerial();
    (void)ReadSerial();
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
    if(17 != gpfpdParsed.size() ) {
        LOG(ERROR) << "Error parsing " << gpfpd;
        return;
    }
    position = gpfpdParsed[6] + "," + gpfpdParsed[7] + "," + gpfpdParsed[8];
    return;
}

void SerialReader::ReadSerial() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const size_t BUFFER_SIZE = 1000;
    unsigned char buf[BUFFER_SIZE];
    std::string frameBuf("");
    std::string latLonHei("");
    std::string slamProtocol("");
    bool isFirstFrame = true;

    while(isGonnaRun_) {
        bzero(buf, BUFFER_SIZE);
        int nread = read(fd_, buf, BUFFER_SIZE);
        if(nread <= 0) {
            DLOG_EVERY_N(INFO, 10) << "nread: " << nread;
            continue;
        }

        for(size_t i = 0; i < nread; ++i) {
            bool isFrameCompleted = false;
            std::string frameCompleted("");
            switch(buf[i]) {
            case '$': {
                isFirstFrame = false;
                frameBuf = "$";
                break;
            }
            case '\r':
                break;
            case '\n':
                isFrameCompleted = true;
                frameCompleted = frameBuf;
                frameBuf.clear();
                break;
            default:
                frameBuf += buf[i];
            }

            if(isFirstFrame) {
                LOG_EVERY_N(INFO, 10) << "First frame might be incomplete, abandon it.";
                continue;
            }

            if(!isFrameCompleted) {
                continue;
            }

            if('F' == frameCompleted[3]) {
                DLOG(INFO) << "$GPFPD received: " << frameCompleted;
                (void)GetPositionFromGpfpd(frameCompleted, latLonHei);
            }
            else
            if('I' == frameCompleted[3]) {
                DLOG(INFO) << "$GTIMU received: " << frameCompleted;
                slamProtocol = frameCompleted + "," + latLonHei;
                DLOG(INFO) << slamProtocol;
                (void)Parse2SlamData(slamProtocol);
            }
            else {
                LOG(INFO) << "Receive " << frameCompleted;
            }
        }
    }
}

void SerialReader::Parse2SlamData(const std::string &_slamProtocol) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    std::vector<std::string> slamParsed;
    //$GTIMU,0,13651.300,-0.2424,-0.0248,0.0491,0.0054,0.0202,0.9980,0,53.9*75,0.0000000,0.0000000,0.00
    boost::split(slamParsed, _slamProtocol, boost::is_any_of(",") );
    if(14 != slamParsed.size() ) {
        LOG(ERROR) << "Error parsing " << _slamProtocol << ", size: " << slamParsed.size() ;
        return;
    }
    slamData_.gpsTime = public_tools::ToolsNoRos::string2double(slamParsed[2]);
    slamData_.gyroX = public_tools::ToolsNoRos::string2double(slamParsed[3]);
    slamData_.gyroY = public_tools::ToolsNoRos::string2double(slamParsed[4]);
    slamData_.gyroZ = public_tools::ToolsNoRos::string2double(slamParsed[5]);
    slamData_.accX = public_tools::ToolsNoRos::string2double(slamParsed[6]);
    slamData_.accY = public_tools::ToolsNoRos::string2double(slamParsed[7]);
    slamData_.accZ = public_tools::ToolsNoRos::string2double(slamParsed[8]);
    slamData_.lat = public_tools::ToolsNoRos::string2double(slamParsed[11]);
    slamData_.lon = public_tools::ToolsNoRos::string2double(slamParsed[12]);
    slamData_.hei = public_tools::ToolsNoRos::string2double(slamParsed[13]);
}