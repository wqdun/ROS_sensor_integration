#include "serial_5651.h"

void Serial5651::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pubImu5651_ = nh_.advertise<sc_msgs::imu5651>("imu_string", 10);

    const int fd = open(serialDevice_.c_str(), O_RDWR | O_NOCTTY);
    if(fd < 0) {
        LOG(ERROR) << "Failed to open " << serialDevice_ << ", try change permission...";
        return;
    }

    if(public_tools::ToolsNoRos::SetSerialOption(fd, 115200, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd);
        close(fd);
        return;
    }

    // (void)WriteSerial(fd);
    (void)ReadSerial(fd);

    close(fd);
    return;
}

void Serial5651::WriteSerial(int _fd) {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = -1;
    const std::vector<std::string> cmds = {
        "$cmd,output,com0,null*ff",
        "$cmd,through,com0,null*ff",
        "$cmd,output,com0,gpfpd,0.1*ff",
        "$cmd,through,com0,gpgga,1*ff",
    };
    for(const auto &data2Send: cmds) {
        err = write(_fd, data2Send.c_str(), data2Send.size() );
        if(err < 0) {
            LOG(ERROR) << "Failed to send " << data2Send << ", err: " << err;
            close(_fd);
            exit(1);
        }
        usleep(10000);
        LOG(INFO) << "Send " << data2Send << " successfully, bytes: " << err;
    }
}

void Serial5651::PublishMsg() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    pubImu5651_.publish(imu232Msg_);
}

void Serial5651::ReadSerial(int _fd) {
    LOG(INFO) << __FUNCTION__ << " start.";

    const size_t BUFFER_SIZE = 1000;
    unsigned char buf[BUFFER_SIZE];
    bool isFirstFrame = true;
    std::string frameBuf("");
    double unixTime = -1.;

    int err = tcflush(_fd, TCIOFLUSH);
    LOG(INFO) << "tcflush: " << err;
    while(ros::ok()) {
        bzero(buf, BUFFER_SIZE);
        int nread = read(_fd, buf, BUFFER_SIZE);
        LOG_EVERY_N(INFO, 50) << "nread: " << nread;
        if(nread <= 0) {
            continue;
        }

        for(size_t i = 0; i < nread; ++i) {
            bool isFrameCompleted = false;
            switch(buf[i]) {
            case '$': {
                struct timeval now;
                gettimeofday(&now, NULL);
                unixTime = now.tv_sec + now.tv_usec / 1000000.;
                isFirstFrame = false;
                frameBuf = buf[i];
                break;
            }
            case '\r':
                break;
            case '\n':
                isFrameCompleted = true;
                break;
            default:
                frameBuf += buf[i];
            }

            if(isFirstFrame) {
                LOG_EVERY_N(INFO, 10) << "First frame might be incomplete, abandon: " << frameBuf;
                frameBuf.clear();
                continue;
            }

            if(!isFrameCompleted) {
                continue;
            }

            Parse5651Frame(frameBuf, unixTime);
            frameBuf.clear();

            PublishMsg();
        }
    }
    return;
}

void Serial5651::Parse5651Frame(const std::string &_frame, double _unixTime) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    assert(!_frame.empty() );

    if("$GPFPD" == _frame.substr(0, 6)) {
        if (Parse5651GpfpdFrame(_frame, _unixTime)) {
            WriteRtImuFile(_frame);
        }
        else {
            LOG(ERROR) << "Failed to parse " << _frame << ", no write it to " << rtImuFile_;
        }
    }
    else
    if("$GPGGA" == _frame.substr(0, 6)) {
        Parse5651GpggaFrame(_frame);
    }
    else {
        LOG(WARNING) << "Unhandled frame: " << _frame;
    }

    return;
}

void Serial5651::Parse5651GpggaFrame(const std::string &_gpggaFrame) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> gpggaFrameParsed;
    boost::split(gpggaFrameParsed, _gpggaFrame, boost::is_any_of(",*") );
    if(16 != gpggaFrameParsed.size()) {
        LOG(ERROR) << "Error parsing " << _gpggaFrame << "; gpggaFrameParsed.size(): " << gpggaFrameParsed.size();
        return;
    }

    // e.g. "279267.900"
    imu232Msg_.utc_time = gpggaFrameParsed[1];
    imu232Msg_.latitude_gpgga = gpggaFrameParsed[2] + gpggaFrameParsed[3];
    imu232Msg_.longitude_gpgga = gpggaFrameParsed[4] + gpggaFrameParsed[5];
    imu232Msg_.no_sv = gpggaFrameParsed[7];
    imu232Msg_.hdop = gpggaFrameParsed[8];

    return;
}

void Serial5651::WriteRtImuFile(const std::string &_gpfpdFrame) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::fstream file(rtImuFile_, std::ios::out | std::ios::app);
    if(!file) {
        LOG(ERROR) << "Failed to open " << rtImuFile_;
        return;
    }
    file << _gpfpdFrame << "\n";
    file.close();

    return;
}

bool Serial5651::Parse5651GpfpdFrame(const std::string &_gpfpdFrame, double __unixTime) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> gpfpdFrameParsed;
    boost::split(gpfpdFrameParsed, _gpfpdFrame, boost::is_any_of(",*") );
    if(17 != gpfpdFrameParsed.size()) {
        LOG(ERROR) << "Error parsing " << _gpfpdFrame << "; gpfpdFrameParsed.size(): " << gpfpdFrameParsed.size();
        return false;
    }

    // e.g. "279267.900"
    imu232Msg_.gps_week = gpfpdFrameParsed[1];
    imu232Msg_.gps_time = gpfpdFrameParsed[2];
    imu232Msg_.heading = gpfpdFrameParsed[3];
    imu232Msg_.pitch = gpfpdFrameParsed[4];
    imu232Msg_.roll = gpfpdFrameParsed[5];
    imu232Msg_.latitude = gpfpdFrameParsed[6];
    imu232Msg_.longitude = gpfpdFrameParsed[7];
    imu232Msg_.altitude = gpfpdFrameParsed[8];
    imu232Msg_.vel_east = gpfpdFrameParsed[9];
    imu232Msg_.vel_north = gpfpdFrameParsed[10];
    imu232Msg_.vel_up = gpfpdFrameParsed[11];
    imu232Msg_.baseline = gpfpdFrameParsed[12];
    imu232Msg_.nsv1_num = gpfpdFrameParsed[13];
    imu232Msg_.nsv2_num = gpfpdFrameParsed[14];
    imu232Msg_.status = gpfpdFrameParsed[15];

    const double GpsWeekTime = public_tools::ToolsNoRos::string2double(imu232Msg_.gps_time);
    unixTimeMinusGpsTimeQueue_.push_back(__unixTime - GpsWeekTime);
    if (unixTimeMinusGpsTimeQueue_.size() > 50) {
        unixTimeMinusGpsTimeQueue_.pop_front();
    }
    // else do nothing

    imu232Msg_.unix_time_minus_gps_time = FillerDeque(unixTimeMinusGpsTimeQueue_);
    return true;
}

