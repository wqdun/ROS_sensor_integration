#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

static double string2double(const string& str);

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
    char buf[1024];

    // open serial port: ttyS0
    if(argc < 2) {
        cout << "Wrong arg, arg shouble be /dev/tty..." << endl;
        exit(1);
    }
    cout << argv[1] << endl;
    int fd1 = open(argv[1], O_RDONLY | O_NONBLOCK); // 打开串口 // fd1 = open("/dev/ttyUSB0", O_RDWR);
    if(-1 == fd1) {
        printf("fd1 == -1\n");
        exit(1);
    }

    // setup port properties
    int nset1 = set_opt(fd1, 115200, 8, 'N', 1); // 设置串口属性
    if(-1 == nset1) {
        exit(1);
    }

    std::string frameBuf;
    long freq = 0;
    timespec t1, t2;
    long last_time_ns;
    long time_when_get_frame_s, time_when_get_frame_ns;
    double time_s;

    int last_minute = 0;

    while(1) {
        memset(buf, 0, 1024);
        int nread = read(fd1, buf, 1024); // 读串口
        if(nread <= 0) {
            continue;
        }

        for(size_t i = 0; i < nread; ++i) {
            bool is_frame_completed = false;
            string frame_complete = "";
            switch(buf[i]) {
            case '$':
                clock_gettime(CLOCK_REALTIME, &t2);
                // cout << "time_end  :" << t2.tv_nsec << endl;
                time_when_get_frame_s = t2.tv_sec;
                time_when_get_frame_ns = t2.tv_nsec;
                time_s = (double)time_when_get_frame_s + (double)time_when_get_frame_ns / 1000000000.0;
                // freq = 1000000000 / (t2.tv_nsec - last_time_ns);
                // cout << "The Freq is: " << freq << endl;
                frameBuf = buf[i];

                // clock_gettime(CLOCK_MONOTONIC, &t1);
                // cout << "time_start:" << t1.tv_nsec << endl;
                // last_time_ns = t1.tv_nsec;
                break;
            case '\r':
                break;
            case '\n':
                time_when_get_frame_s %= 3600;
                cout << "[" << time_when_get_frame_s / 60 << ":" << time_when_get_frame_s % 60 << "]:"
                     << "[" << std::fixed << time_s << "]"
                     << frameBuf << endl;
                is_frame_completed = true;
                frame_complete = frameBuf;
                frameBuf.clear();
                break;
            default:
                frameBuf += buf[i];
            }

            if(is_frame_completed) {
                vector<string> parsed_data;
                // parse the GPS time
                boost::split(parsed_data, frame_complete, boost::is_any_of( ",*" ), boost::token_compress_on);
                if(parsed_data.size() <= 16) {
                    // 1st frame might be wrong, 'cause no '$'
                    cout << "Error when parsing." << parsed_data.size() << endl;
                    continue;
                }

                // e.g. "279267.900"
                string GPS_week_time_str = parsed_data[2];
                double GPS_week_time = string2double(GPS_week_time_str);
                int hour = (int)GPS_week_time / (60 * 60) % 24;
                int minute = (int)GPS_week_time / 60 % 60;
                double second = fmod(GPS_week_time, 60);

                if(minute != last_minute) {
                    char command[100] = "";
                    sprintf(command, "date -s \"%d:%d:%f\"", hour, minute, second);
                    system(command);
                    cout << "Modify Unix time successfully." << endl;
                }
                last_minute = minute;
            }
        }
    }

    close(fd1);
    return 0;
}

static double string2double(const string& str) {
    std::istringstream iss(str);
    double num;
    iss >> num;
    return num;
}
