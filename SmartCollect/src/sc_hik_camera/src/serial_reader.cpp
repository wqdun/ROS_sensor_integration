#include "serial_reader.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

SerialReader::SerialReader() {
    LOG(INFO) << __FUNCTION__ << " start.";
    isGonnaRun_ = true;
    slam10Datas_.resize(10);

    pCanParser_.reset(new CanParser() );

    for(auto &slamData: slam10Datas_) {
        slamData = {0};
    }

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
    pCanParserThread_->join();
}

void SerialReader::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pCanParserThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CanParser::Run, pCanParser_) ) );

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
    if(18 != gpfpdParsed.size() ) {
        LOG(ERROR) << "Error parsing " << gpfpd << gpfpdParsed.size();
        return;
    }
    position = gpfpdParsed[7] + "," + gpfpdParsed[8] + "," + gpfpdParsed[9];
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

        for(int i = 0; i < nread; ++i) {
            bool isFrameCompleted = false;
            std::string frameCompleted("");
            switch(buf[i]) {
            case '$': {
                isFirstFrame = false;
                struct timeval now;
                gettimeofday(&now, NULL);
                frameBuf = std::to_string(now.tv_sec + now.tv_usec / 1000000.) + ",$";
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

            if(frameCompleted.find("GPFPD") < frameCompleted.size() ) {
                // 1539608480.746874,$GPFPD,0,2083.510,0.000,0.074,13.047,0.0000000,0.0000000,0.00,0.000,0.000,0.000,0.000,0,0,00*48
                LOG(INFO) << "$GPFPD received: " << frameCompleted;
                (void)GetPositionFromGpfpd(frameCompleted, latLonHei);
            }
            else
            if(frameCompleted.find("GTIMU") < frameCompleted.size() ) {
                // 1539608480.750918,$GTIMU,0,2083.510,0.0300,-0.3019,0.1624,-0.2248,0.0010,0.9730,0,52.7*46
                DLOG(INFO) << "$GTIMU received: " << frameCompleted;
                slamProtocol = frameCompleted + "," + latLonHei;
                // 1539608480.750918,$GTIMU,0,2083.510,0.0300,-0.3019,0.1624,-0.2248,0.0010,0.9730,0,52.7*46,0.0000000,0.0000000,0.00
                DLOG(INFO) << "slamProtocol: " << slamProtocol;
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

    public_tools::GeoToGauss(slamData_.lon, slamData_.lat, 20, 6, &(slamData_.east), &(slamData_.north));

    slamData_.encoder_v = 0;
    slamData_.yaw = 0;
    slamData_.pitch = 0;
    slamData_.roll = 0;

    slam10Datas_.emplace_back(slamData_);
    slam10Datas_.pop_front();
}
