#include "comm_timer.h"

CommTimer::CommTimer(const std::string &_serialName, const std::string &_imuPath) {
    LOG(INFO) << __FUNCTION__ << " start.";
    serialName_ = _serialName;

    std::string imuFileNamePrefix("");
    (void)public_tools::PublicTools::generateFileName(_imuPath, imuFileNamePrefix);
    rtImuFile_ = _imuPath + imuFileNamePrefix + "_rt_track.txt";

    pubImu5651_ = nh_.advertise<sc_msgs::imu5651>("imu_string", 10);
}

CommTimer::~CommTimer() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

void CommTimer::PublishMsg() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pubImu5651_.publish(imu232Msg_);
}

int CommTimer::Read() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const int fd = open(serialName_.c_str(), O_RDWR | O_NOCTTY);
    if(fd < 0) {
        LOG(ERROR) << "Failed to open " << serialName_ << ", try change permission...";
        return -1;
    }

    if(public_tools::ToolsNoRos::SetSerialOption(fd, 115200, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd);
        return -1;
    }

    (void)ReadSerial(fd);

    close(fd);
    return 0;
}

void CommTimer::ReadSerial(int _fd) {
    LOG(INFO) << __FUNCTION__ << " start.";

    const size_t BUFFER_SIZE = 1000;
    unsigned char buf[BUFFER_SIZE];
    bool isFirstFrame = true;
    std::string frameBuf("");

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
            double unixTime = -1.;
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
                continue;
            }

            if(!isFrameCompleted) {
                continue;
            }

            Parse5651Frame(frameBuf, unixTime);
            frameBuf.clear();
        }
    }
    return;
}

void CommTimer::Parse5651Frame(const std::string &_frame, double _unixTime) {
    LOG(INFO) << __FUNCTION__ << " start.";
    assert(!_frame.empty() );

    if("$GPFPD" == _frame.substr(0, 6)) {
        WriteRtImuFile(_frame);
        Parse5651GpfpdFrame(_frame, _unixTime);
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

void CommTimer::Parse5651GpggaFrame(const std::string &_gpggaFrame) {
    LOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> gpggaFrameParsed;
    boost::split(gpggaFrameParsed, _gpggaFrame, boost::is_any_of(",*") );
    if(15 != gpggaFrameParsed.size()) {
        LOG(ERROR) << "Error parsing " << _gpggaFrame << "; gpggaFrameParsed.size(): " << gpggaFrameParsed.size();
        return;
    }

    // e.g. "279267.900"
    imu232Msg_.UtcTime = gpggaFrameParsed[1];
    imu232Msg_.LatitudeGpgga = gpggaFrameParsed[2] + gpggaFrameParsed[3];
    imu232Msg_.LongitudeGpgga = gpggaFrameParsed[4] + gpggaFrameParsed[5];
    imu232Msg_.NoSV = gpggaFrameParsed[7];
    imu232Msg_.Hdop = gpggaFrameParsed[8];

    return;
}

void CommTimer::WriteRtImuFile(const std::string &_gpfpdFrame) {
    LOG(INFO) << __FUNCTION__ << " start.";

    std::fstream file(rtImuFile_, std::ios::out | std::ios::app);
    if(!file)
    {
        LOG(ERROR) << "Failed to open " << rtImuFile_;
        return;
    }
    file << _gpfpdFrame << "\n";
    file.close();

    return;
}

void CommTimer::Parse5651GpfpdFrame(const std::string &_gpfpdFrame, double __unixTime) {
    LOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> gpfpdFrameParsed;
    boost::split(gpfpdFrameParsed, _gpfpdFrame, boost::is_any_of(",*") );
    if(15 != gpfpdFrameParsed.size()) {
        LOG(ERROR) << "Error parsing " << _gpfpdFrame << "; gpfpdFrameParsed.size(): " << gpfpdFrameParsed.size();
        return;
    }

    // e.g. "279267.900"
    imu232Msg_.GPSWeek = gpfpdFrameParsed[1];
    imu232Msg_.GPSTime = gpfpdFrameParsed[2];
    imu232Msg_.Heading = gpfpdFrameParsed[3];
    imu232Msg_.Pitch = gpfpdFrameParsed[4];
    imu232Msg_.Roll = gpfpdFrameParsed[5];
    imu232Msg_.Latitude = gpfpdFrameParsed[6];
    imu232Msg_.Longitude = gpfpdFrameParsed[7];
    imu232Msg_.Altitude = gpfpdFrameParsed[8];
    imu232Msg_.Vel_east = gpfpdFrameParsed[9];
    imu232Msg_.Vel_north = gpfpdFrameParsed[10];
    imu232Msg_.Vel_up = gpfpdFrameParsed[11];
    imu232Msg_.Baseline = gpfpdFrameParsed[12];
    imu232Msg_.NSV1_num = gpfpdFrameParsed[13];
    imu232Msg_.NSV2_num = gpfpdFrameParsed[14];
    imu232Msg_.Status = gpfpdFrameParsed[15];

    const double GpsWeekTime = public_tools::ToolsNoRos::string2double(imu232Msg_.GPSTime);
    unixTimeMinusGpsTime_ = __unixTime - GpsWeekTime;
    return;
}

double CommTimer::GetUnixTimeMinusGpsTime() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return unixTimeMinusGpsTime_;
}

