#include "get_serial.h"

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
    struct termios newtio, oldtio;
    if(tcgetattr(fd, &oldtio) != 0) {
        perror("Setup Serial 1.");
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
    }

    switch(nEvent) {
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
        perror("Setup nStop unavailable.");
        return -1;
    }

    tcflush(fd, TCIFLUSH);

    newtio.c_cc[VTIME] = 100; // time out 15s重要
    newtio.c_cc[VMIN] = 0; // Update the option and do it now 返回的最小值  重要

    if(0 != (tcsetattr(fd, TCSANOW, &newtio))) {
        perror("Com setup error.");
        return -1;
    }

    return 0;
}

int open_serial(void) {
    int fd1 = open("/dev/ttyUSB0", O_RDONLY | O_NONBLOCK);
    if(-1 == fd1) {
        ROS_FATAL("Failed to open Com port, check the port name and permission.");
        exit(1);
    }

    // setup port properties
    int nset1 = set_opt(fd1, 115200, 8, 'N', 1);
    if(-1 == nset1) {
        ROS_FATAL("Failed to setup port properties.");
        exit(1);
    }

    return fd1;
}

int read_serial(int fd) {
    static std::string frameBuf;
    static char buf[1024];

    memset(buf, 0, 1024);
    int nread = read(fd, buf, 1024);
    if(nread < 0) {
        ROS_WARN("Failed to read Com port.");
        return nread;
    }
    ROS_DEBUG("Read %d char.", nread);

    for(size_t i = 0; i < nread; ++i) {
        switch(buf[i]) {

        // frame header
        case '$':
            frameBuf = buf[i];
            break;

        // frame footer
        case '\r':
            break;
        case '\n':
            // add # as new frame footer
            frameBuf += '#';
            frame2pub = frameBuf;
            frameBuf.clear();
            break;

        default:
            frameBuf += buf[i];
        }
    }

    return nread;
}
