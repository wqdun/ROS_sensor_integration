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
    default:
        perror("Unsupported data size\n");
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
        perror("Unsupported parity\n");
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

     // time out 15s重要
    newtio.c_cc[VTIME] = 100;
    // Update the option and do it now 返回的最小值  重要
    newtio.c_cc[VMIN] = 0;

    if(0 != tcsetattr(fd, TCSANOW, &newtio) ) {
        perror("Com setup error.");
        return -1;
    }

    printf("set done!\n\r");
    return 0;
}

bool getGpgga(const char *inBuf, const size_t &bufSize, string &gpgga, bool &isGpggaStart) {
    cout << __FUNCTION__ << " start.\n";
    for(size_t i = 0; i < bufSize; ++i) {
        switch(inBuf[i]) {
        case '$':
            isGpggaStart = true;
            gpgga = "$";
            break;
        case '\r':
            break;
        case '\n':
            isGpggaStart = false;
            if(gpgga.size() > 6) {
                cout << "gpgga: " << gpgga << endl;
                return true;
            }
            // else: might be: "$ \n"
            break;
        default:
            if(!isGpggaStart) {
                break;
            }
            gpgga += inBuf[i];
            if(6 == gpgga.size() ) {
                if(gpgga != "$GPGGA") {
                    cout << "Though contains $, not $GPGGA.\n";
                    isGpggaStart = false;
                }
            }
        }
    }

    cout << "gpgga: " << gpgga << endl;
    return false;
}

bool getGdopFromGpgga(const string &inGpgga, string &outGdop) {
    cout << __FUNCTION__ << " start.\n";
    vector<string> parsedGpgga;
    boost::split(parsedGpgga, inGpgga, boost::is_any_of(",") );
    if(15 != parsedGpgga.size() ) {
        // 1st frame might be wrong, 'cause no '$'
        cout << "Error parsing: " << parsedGpgga.size() << endl;
        return false;
    }
    outGdop = parsedGpgga[8];
    return true;
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
    cout << "Create file successfully: 5651_422.dat\n";
    const size_t BUFFER_SIZE = 1000;

    char buf[BUFFER_SIZE];
    fd_set rd;
    int nread = 0;
    string completeGpgga("");
    bool isGpggaStart = false;

    while(1) {
        bzero(buf, BUFFER_SIZE);
        nread = read(fd, buf, BUFFER_SIZE);
        cout << dec << "nread: " << nread << "\n";
        if(nread <= 0) {
            continue;
        }
        for(int k = 0; k < nread; ++k) {
            printf("%02X ", (unsigned char)buf[k]);
        }
        cout << "\n";

        (void)fwrite(buf, nread, 1, pOutFile);

        if(!getGpgga(buf, nread, completeGpgga, isGpggaStart) ) {
            cout << "Gpgga not complete or is old.\n";
            continue;
        }
        string hdop("");
        if(!getGdopFromGpgga(completeGpgga, hdop) ) {
            cout << "Failed to get hdop, completeGpgga: " << completeGpgga << endl;
            continue;
        }
        cout << "hdop: " << hdop << endl;
    }

    fclose(pOutFile);
    close(fd);
    return 0;
}


