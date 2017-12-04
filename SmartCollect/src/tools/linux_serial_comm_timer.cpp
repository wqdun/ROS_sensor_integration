#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
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
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using namespace std;

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

    if(0 != tcsetattr(fd, TCSANOW, &newtio)) {
        perror("Com setup error.");
        return -1;
    }

    // printf("set done!\n\r");
    return 0;
}

int main(int argc, char **argv) {
    // open serial port: ttyS0
    if(argc < 2) {
        cout << "Wrong arg, arg shouble be /dev/tty..." << endl;
        exit(1);
    }
    cout << argv[1] << endl;
    int fd = open(argv[1], O_RDONLY); //  | O_NONBLOCK); // 打开串口 // fd = open("/dev/ttyUSB0", O_RDWR);
    if(-1 == fd) {
        printf("Serial port error\n");
        exit(1);
    }
    cout << "Open " << ttyname(fd) << " successfully\n";

    // setup port properties
    int nset1 = set_opt(fd, 115200, 8, 'N', 1); // 设置串口属性
    if(-1 == nset1) {
        exit(1);
    }
    cout << "Set " << ttyname(fd) << " successfully\n";


    FILE *pOutFile;
    if(!(pOutFile = fopen("5651_422.dat", "wb") ) ) {
        cout << "Create file: failed.";
        exit(1);
    }
    cout << "Create file successfully: 5651_422.dat";
    unsigned char buf[50];
    fd_set rd;
    int nread = 0;
    string frameBuf("");

    while(1) {
        nread = read(fd, buf, sizeof(buf) );
        // string bufStr(buf);
        // fill_frame(bufStr, frameBuf);
        // now frameBuf should be human readable


        // parse
        // (void)fwrite(buf, nread, 1, pOutFile); // << C txt
        // if() set a member; then pub -> infor

        // cout << dec << "nread: " << nread << "\n";
        if(nread <= 0)
            // break;
            continue;

        for(int k = 0; k < nread; ++k) {
            printf("%02X ", buf[k]);
            // cout << hex << (int)buf[k]; // or nread
        }
        // cout << "\n";
        bzero(buf, sizeof(buf) );
    }

    fclose(pOutFile);

    close(fd);
    return 0;
}
