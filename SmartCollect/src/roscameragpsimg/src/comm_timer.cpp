#include "comm_timer.h"
#include "camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

CommTimer::CommTimer(const std::string &_rawdataPath) {
    LOG(INFO) << __FUNCTION__ << " start, rawdataPath_: " << _rawdataPath;
    ros::NodeHandle nh;
    pubImu5651_ = nh.advertise<sc_msgs::imu5651>("imu_string", 0);
    rawdataPath_ = _rawdataPath;
}

CommTimer::~CommTimer() {}

int CommTimer::getTime(Cameras *pCameras)
{
    char buf[1024];

    int fd1 = open("/dev/ttyS0", O_RDONLY);
    if(-1 == fd1)
    {
        LOG(ERROR) << "Failed to open /dev/ttyS0.";
        exit(6);
    }
    LOG(INFO) << "Open /dev/ttyS0 successfully.";

    int nset1 = setOpt(fd1, 115200, 8, 'N', 1);
    if(-1 == nset1)
    {
        LOG(ERROR) << "Failed to setup /dev/ttyS0 properties.";
        exit(1);
    }

    const std::string imuPath(rawdataPath_ + "/IMU/");
    std::string imuFileName("");
    public_tools::PublicTools::generateFileName(imuPath, imuFileName);
    imuFileName += "_rt_track.txt";
    LOG(INFO) << "imuFileName: " << imuFileName;
    std::string imuFile(imuPath + imuFileName);

    std::fstream file;
    std::string frameBuf("");

    bool isGpsWeekTimeUpdated = false;
    timespec time_sys_end, time_sys_start;
    double GPS_week_time = -1.;
    std::vector<std::string> parsed_data;
    bool isGpsTimeValidBeforeGpsWeekTimeCorrected = false;

    while(ros::ok() )
    {
        memset(buf, 0, 1024);
        int nread = read(fd1, buf, 1024);

        if(nread <= 0)
        {
            continue;
        }
        bool is_frame_completed = false;

        for(size_t i = 0; i < nread; ++i)
        {
            is_frame_completed = false;
            std::string frame_complete("");
            switch(buf[i])
            {
            case '$':
                frameBuf = buf[i];
                break;
            case '\r':
                break;
            case '\n':
                is_frame_completed = true;
                frame_complete = frameBuf;
                frameBuf.clear();
                break;
            default:
                frameBuf += buf[i];
            }

            if(!is_frame_completed)
            {
                continue;
            }

            boost::split(parsed_data, frame_complete, boost::is_any_of( ",*" ) );
            if(17 != parsed_data.size() )
            {
                LOG(ERROR) << "Error parsing " << frame_complete;
                continue;
            }

            // e.g. "279267.900"
            GPS_week_time = public_tools::PublicTools::string2num(parsed_data[2], (double)(0) );
            isGpsWeekTimeUpdated = true;

            imu232Msg_.GPSWeek = parsed_data[1];
            imu232Msg_.GPSTime = parsed_data[2];
            imu232Msg_.Heading = parsed_data[3];
            imu232Msg_.Pitch = parsed_data[4];
            imu232Msg_.Roll = parsed_data[5];
            imu232Msg_.Latitude = parsed_data[6];
            imu232Msg_.Longitude = parsed_data[7];
            imu232Msg_.Altitude = parsed_data[8];
            imu232Msg_.Vel_east = parsed_data[9];
            imu232Msg_.Vel_north = parsed_data[10];
            imu232Msg_.Vel_up = parsed_data[11];
            imu232Msg_.Baseline = parsed_data[12];
            imu232Msg_.NSV1_num = parsed_data[13];
            imu232Msg_.NSV2_num = parsed_data[14];
            imu232Msg_.Status = parsed_data[15];

            pubImu5651_.publish(imu232Msg_);

            if(imu232Msg_.Latitude.find("0.0000") < imu232Msg_.Latitude.size() ) {
                LOG_EVERY_N(INFO, 1000) << "imu232Msg_.Latitude is invalid: " << imu232Msg_.Latitude;
                continue;
            }
            const double sysWeekSec = ros::Time::now().toSec() - 24 * 3600 * 3;
            const int timeErr = (int)sysWeekSec % (24 * 7 * 3600) - GPS_week_time;
            LOG_FIRST_N(INFO, 1) << "Invalid GPS frame when time diff > 20 min.";
            LOG_EVERY_N(INFO, 1000) << "Unix time and GPS week time diff: " << timeErr << "s.";
            if(timeErr < -1200 || timeErr > 1200) {
                LOG_EVERY_N(WARNING, 100) << "Invalid GPS frame; time diff: " << timeErr << "s.";
                continue;
            }
            isGpsTimeValidBeforeGpsWeekTimeCorrected = true;

            file.open(imuFile, std::ios::out | std::ios::app);
            if(!file)
            {
                LOG(ERROR) << "Failed to open " << imuFile;
                exit(1);
            }
            file << frame_complete << "\n";
            file.close();
        }

        if(isGpsWeekTimeUpdated)
        {
            DLOG(INFO) << "Update GPS_week_time: " << std::fixed << GPS_week_time;
            pCameras->gpsWeekTimeCorrected_ = GPS_week_time;
            isGpsWeekTimeUpdated = false;
            clock_gettime(CLOCK_REALTIME, &time_sys_start);
        }
        else
        {
            clock_gettime(CLOCK_REALTIME, &time_sys_end);
            double time_diff = time_sys_end.tv_sec + time_sys_end.tv_nsec / 1000000000.0 - (time_sys_start.tv_sec + time_sys_start.tv_nsec / 1000000000.0);
            pCameras->gpsWeekTimeCorrected_ = GPS_week_time + time_diff;
            DLOG(INFO) << "pCameras->gpsWeekTimeCorrected_: " << std::fixed << pCameras->gpsWeekTimeCorrected_ << "; time_diff: " << time_diff;
        }
        pCameras->isGpsTimeValid_ = isGpsTimeValidBeforeGpsWeekTimeCorrected;
    }

    close(fd1);
    return 0;
}

int CommTimer::setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if(tcgetattr(fd, &oldtio) != 0)
    {
        LOG(INFO) << ("Setup Serial 1.");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~CSIZE;

    switch(nBits)
   {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch(nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch(nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if(1 == nStop)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else
    if(2 == nStop)
    {
        newtio.c_cflag |= CSTOPB;
    }
    else
    {
        LOG(ERROR) << "Setup nStop unavailable.";
        return -1;
    }

    tcflush(fd, TCIFLUSH);

    newtio.c_cc[VTIME] = 100;
    newtio.c_cc[VMIN] = 0;

    if(0 != tcsetattr(fd, TCSANOW, &newtio))
    {
        LOG(ERROR) << "Com setup error.";
        return -1;
    }

    LOG(INFO) << "Set done.";
    return 0;
}
