#include "tools_no_ros.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

namespace public_tools
{
double ToolsNoRos::string2double(const std::string& str) {
    std::istringstream iss(str);
    double num;
    iss >> num;
    return num;
}

int ToolsNoRos::setSerialOption(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
    LOG(INFO) << __FUNCTION__ << " start.";
    struct termios newtio, oldtio;
    if(tcgetattr(fd, &oldtio) != 0) {
        LOG(ERROR) << ("Setup Serial 1.");
        exit(1);
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
        LOG(ERROR) << "Unsupported data size.";
        exit(1);
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
        LOG(ERROR) << "Unsupported parity.";
        exit(1);
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
        LOG(ERROR) << "nSpeed: " << nSpeed;
        exit(1);
    }

    if(1 == nStop) {
        newtio.c_cflag &= ~CSTOPB;
    }
    else
    if(2 == nStop) {
        newtio.c_cflag |= CSTOPB;
    }
    else {
        LOG(ERROR) << "Setup nStop unavailable.";
        exit(1);
    }

    tcflush(fd, TCIFLUSH);

    newtio.c_cc[VTIME] = 100;
    newtio.c_cc[VMIN] = 0;

    if(0 != tcsetattr(fd, TCSANOW, &newtio)) {
        LOG(ERROR) << "Com setup error.";
        exit(1);
    }

    LOG(INFO) << "Set done.";
    return 0;
}

}
// namespace public_tools
