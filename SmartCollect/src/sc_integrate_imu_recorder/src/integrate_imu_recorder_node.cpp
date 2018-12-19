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
    if(0 != public_tools::ToolsNoRos::SetSerialOption(fd, 230400, 8, 'N', 1) ) {
        LOG(ERROR) << "Failed to set " << ttyname(fd);
        exit(1);
    }
    LOG(INFO) << "Set " << ttyname(fd) << " successfully";

    ros::init(argc, argv, "hdop_publisher");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    ros::Publisher pubHdop = nh.advertise<sc_msgs::scIntegrateImu>("imu422_hdop", 0);

    const std::string imuPath(argv[1]);

    std::string rtImuFileName("");
    (void)public_tools::PublicTools::generateFileName(imuPath, rtImuFileName);
    const std::string rawInsFile(imuPath + rtImuFileName + "_integrate_imu.dat");

    const string timeErrFile(imuPath + rtImuFileName + "_time_diff.txt");
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

    std::ofstream timeErrFileStream(timeErrFile.c_str(), std::ios::app);
    if(!timeErrFileStream) {
        LOG(ERROR) << "Failed to create: " << timeErrFile;
        exit(1);
    }
    timeErrFileStream << std::fixed << gps422Time << "," << sysTime << "," << timeErr << "\n";
    timeErrFileStream.close();

    return true;
}

