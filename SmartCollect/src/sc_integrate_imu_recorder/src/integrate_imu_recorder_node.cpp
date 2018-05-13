#include "integrate_imu_recorder.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    if(2 != argc) {
        LOG(ERROR) << "I need a record path, argc: " << argc;
        exit(1);
    }

    int fd = open("/dev/ttyUSB0", O_RDONLY);
    if(-1 == fd) {
        LOG(ERROR) << "Failed to open /dev/ttyUSB0.";
        exit(1);
    }
    LOG(INFO) << "Open " << ttyname(fd) << " successfully";

    // Setup port properties
    if(0 != set_opt(fd, 230400, 8, 'N', 1) ) {
        LOG(ERROR) << "Failed to set " << ttyname(fd);
        exit(1);
    }
    LOG(INFO) << "Set " << ttyname(fd) << " successfully";

    ros::init(argc, argv, "hdop_publisher");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    ros::Publisher pubHdop = nh.advertise<sc_msgs::scIntegrateImu>("imu422_hdop", 0);
    string rawInsFile("");
    // private_nh.param("raw_ins_path", rawInsFile, rawInsFile);
    rawInsFile = argv[1];
    string rtImuFileName("");
    (void)public_tools::PublicTools::generateFileName(rawInsFile, rtImuFileName);
    rawInsFile += (rtImuFileName + "_integrate_imu.dat");

    const string timeErrFile(rawInsFile + rtImuFileName + "_time_diff.txt");
    std::ofstream timeErrFileStream(timeErrFile.c_str());
    if(!timeErrFileStream) {
        LOG(ERROR) << "Failed to create: " << timeErrFile;
        exit(1);
    }
    timeErrFileStream << "gps_time,sys_time,time_error\n";
    timeErrFileStream.close();

    FILE *pOutFile;
    if(!(pOutFile = fopen(rawInsFile.c_str(), "wb") ) ) {
        LOG(ERROR) << "Failed to create: " << rawInsFile;
        exit(1);
    }
    LOG(INFO) << "Create "<< rawInsFile << " successfully.";

    const size_t BUFFER_SIZE = 1000;
    unsigned char buf[BUFFER_SIZE];
    int nread = 0;
    string completeGpgga("");
    bool isGpggaStart = false;
    sc_msgs::scIntegrateImu hdopMsg;

    while(ros::ok() ) {
        bzero(buf, BUFFER_SIZE);
        nread = read(fd, buf, BUFFER_SIZE);
        DLOG(INFO) << std::dec << "nread: " << nread;
        if(nread <= 0) {
            continue;
        }

#ifndef NDEBUG
        for(size_t i = 0; i < nread; ++i) {
            DLOG(INFO) << std::hex << (int)buf[i];
        }
#endif

        (void)fwrite(buf, nread, 1, pOutFile);

        if(!getGpgga(buf, nread, completeGpgga, isGpggaStart) ) {
            DLOG(INFO) << "Gpgga not complete or not updated.";
            continue;
        }
        if(!getGdopFromGpgga(completeGpgga, hdopMsg, timeErrFile) ) {
            LOG(ERROR) << "Failed to get hdop, completeGpgga: " << completeGpgga;
            continue;
        }

        LOG(INFO) << "Hdop: " << hdopMsg.Hdop;
        // TODO: calc diff to a file (sys - gps)
        pubHdop.publish(hdopMsg);
    }

    fclose(pOutFile);
    close(fd);
    return 0;
}

bool getGpgga(const unsigned char *inBuf, const size_t &bufSize, string &gpgga, bool &isGpggaStart) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    bool isGpggaComplete = false;
    for(size_t i = 0; i < bufSize; ++i) {
        switch(inBuf[i]) {
        case '$':
            isGpggaStart = true;
            gpgga = "$";
            break;
        case '\r':
            break;
        case '\n':
            if(!isGpggaStart) {
                DLOG(INFO) << "Met \\n before $: " << gpgga;
                break;
            }
            if(gpgga.size() < 6) {
                LOG(INFO) << "Met $**\\n: " << gpgga;
                gpgga.clear();
                isGpggaStart = false;
                break;
            }
            // when size > 6 && isGpggaStart: it must be $GPGGA..
            isGpggaComplete = true;
            isGpggaStart = false;
            // break loop
            i = bufSize;
            break;
        default:
            if(!isGpggaStart) {
                break;
            }
            gpgga += inBuf[i];
            if(6 == gpgga.size() ) {
                if("$GPGGA" != gpgga) {
                    LOG(INFO) << "Met $, but not $GPGGA: " << gpgga;
                    gpgga.clear();
                    isGpggaStart = false;
                }
            }
        }
    }
    DLOG(INFO) << "gpgga: " << gpgga;
    return isGpggaComplete;
}

bool getGdopFromGpgga(const string &inGpgga, sc_msgs::scIntegrateImu &out422Msg, const string &timeErrFile) {
    LOG(INFO) << __FUNCTION__ << " start.";
    vector<string> parsedGpgga;
    boost::split(parsedGpgga, inGpgga, boost::is_any_of(",") );
    if(15 != parsedGpgga.size() ) {
        // 1st frame might be wrong, 'cause no '$'
        LOG(ERROR) << "Error parsing: " << parsedGpgga.size();
        return false;
    }

    out422Msg.GpsWeekTime = parsedGpgga[1];
    out422Msg.Latitude = parsedGpgga[2];
    out422Msg.Longitude = parsedGpgga[4];
    out422Msg.NoSV = parsedGpgga[7];
    out422Msg.Hdop = parsedGpgga[8];

    const double gps422Time = public_tools::PublicTools::string2num(parsedGpgga[1], double(-1) );
    if(gps422Time < 0) {
        LOG(WARNING) << "422 GPS time is empty.";
        return true;
    }
    const double sysTime = ros::Time::now().toSec();
    double timeErr = gps422Time - sysTime;
    timeErr = fmod(timeErr, 3600.);
    if(timeErr > 1800) {
        timeErr -= 3600;
    }
    else
    if(timeErr < -1800) {
        timeErr += 3600;
    }
    // else {nothing}

    std::ofstream timeErrFileStream(timeErrFile.c_str());
    if(!timeErrFileStream) {
        LOG(ERROR) << "Failed to create: " << timeErrFile;
        exit(1);
    }
    timeErrFileStream << gps422Time << "," << sysTime << "," << timeErr << "\n";
    timeErrFileStream.close();

    return true;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
    struct termios newtio, oldtio;
    if(tcgetattr(fd, &oldtio) != 0) {
        perror("Setup Serial 1.\n");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~CSIZE;

    switch(nBits) {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    default:
        perror("Unsupported data size.\n");
        return -1;
    }

    switch(nEvent) {
    case 'o':
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'e':
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'n':
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        perror("Unsupported parity.\n");
        return -1;
    }

    switch(nSpeed) {
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
    case 230400:
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
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

    if(1 == nStop) {
        newtio.c_cflag &= ~CSTOPB;
    }
    else
    if(2 == nStop) {
        newtio.c_cflag |= CSTOPB;
    }
    else {
        perror("Setup nStop unavailable.\n");
        return -1;
    }

    tcflush(fd, TCIFLUSH);

    // time out 15s重要
    newtio.c_cc[VTIME] = 100;
    // Update the option and do it now 返回的最小值  重要
    newtio.c_cc[VMIN] = 0;

    if(0 != tcsetattr(fd, TCSANOW, &newtio) ) {
        perror("Com setup error.\n");
        return -1;
    }

    printf("Set done.\n");
    return 0;
}
