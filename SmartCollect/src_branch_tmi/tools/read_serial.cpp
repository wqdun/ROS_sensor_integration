#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <error.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

void GetPositionFromGpfpd(const std::string &gpfpd, std::string &position) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> gpfpdParsed;
    boost::split(gpfpdParsed, gpfpd, boost::is_any_of( ",*" ) );
    if(17 != gpfpdParsed.size() ) {
        LOG(ERROR) << "Error parsing " << gpfpd;
        return;
    }
    // _latitude = gpfpdParsed[6];
    // _longitude = gpfpdParsed[7];
    // _height = gpfpdParsed[8];
    position = gpfpdParsed[6] + "," + gpfpdParsed[7] + "," + gpfpdParsed[8];
    return;
}

int setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
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
    case 230400:
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        LOG(ERROR);
        exit(1);
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

int main() {
    char buf[1024];

    int fd1 = open("/dev/ttyUSB0", O_RDWR);
    if(-1 == fd1)
    {
        LOG(ERROR) << "Failed to open /dev/ttyS0.";
        exit(6);
    }


    LOG(INFO) << "Open /dev/ttyUSB0 successfully.";

    int nset1 = setOpt(fd1, 230400, 8, 'N', 1);
    if(-1 == nset1)
    {
        LOG(ERROR) << "Failed to setup /dev/ttyS0 properties.";
        exit(1);
    }

    int err = -1;
    // std::string data2Send("$cmd,set,com0,230400,none,8,1,rs232,log*ff");
    // err = write(fd1, data2Send.c_str(), data2Send.size() );
    // sleep(1);
    std::string data2Send = "$cmd,output,com0,null*ff";
    err = write(fd1, data2Send.c_str(), data2Send.size() );
    data2Send = "$cmd,output,com0,gpfpd,0.01*ff";
    err = write(fd1, data2Send.c_str(), data2Send.size() );
    data2Send = "$cmd,output,com0,gtimu,0.01*ff";
    err = write(fd1, data2Send.c_str(), data2Send.size() );
    if(err < 0) {
        LOG(ERROR) << "Failed to send " << data2Send << ", err: " << err;
        exit(1);
    }
    LOG(INFO) << "Send " << data2Send << " successfully, err: " << err;

    const std::string imuFile("serial.txt");
    LOG(INFO) << "imuFile: " << imuFile;

    std::fstream file;
    std::string frameBuf("");

    bool isGpsWeekTimeUpdated = false;
    timespec time_sys_end, time_sys_start;
    double GPS_week_time = -1.;
    std::vector<std::string> parsed_data;
    bool isGpsTimeValidBeforeGpsWeekTimeCorrected = false;

    bool isFirstFrame = true;
    LOG(INFO) << "First frame might be incomplete, abandon it.";

    std::string slamProtocol("");
    std::string _3dPosition("");

    while(1) {
        memset(buf, 0, 1024);
        int nread = read(fd1, buf, 1024);

        if(nread <= 0)
        {
            DLOG(INFO) << nread;
            continue;
        }
        for(size_t i = 0; i < nread; ++i) {
            DLOG(INFO) << buf[i];
        }

        for(size_t i = 0; i < nread; ++i)
        {
            bool is_frame_completed = false;
            std::string frame_complete("");
            switch(buf[i])
            {
            case '$': {
                isFirstFrame = false;

                // frameBuf = std::to_string(sysTime) + ",$";
                frameBuf = buf[i];
                break;
            }
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

            if(isFirstFrame) {
                LOG_EVERY_N(INFO, 10) << "First frame might be incomplete, abandon it.";
                continue;
            }

            if(!is_frame_completed)
            {
                continue;
            }

            // parse(frame_complete);
            if('F' == frame_complete[3]) {
                DLOG(INFO) << "$GPFPD received: " << frame_complete;
                (void)GetPositionFromGpfpd(frame_complete, _3dPosition);
            }
            else
            if('I' == frame_complete[3]) {
                DLOG(INFO) << "$GPIMU received: " << frame_complete;
                slamProtocol = frame_complete + "," + _3dPosition;
                LOG(INFO) << slamProtocol;
            }
            else {
                LOG(ERROR) << "Error parsing " << frame_complete;
                // exit(1);
            }

            file.open(imuFile, std::ios::out | std::ios::app);
            if(!file)
            {
                LOG(ERROR) << "Failed to open " << imuFile;
                exit(1);
            }
            file << frame_complete << "\n";
            file.close();
        }
    }

    close(fd1);
    return 0;
}


