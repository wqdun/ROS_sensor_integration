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

void ToolsNoRos::GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP) {
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double jd_hd, wd_hd;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;     //  克拉索夫斯基椭球中子午弧长
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos, et3;   //  sinB,cosB
    double PI = 3.14159265358979;
    double b_e2 = 0.00669437999013;
    double b_c = 6378137;
    jd_hd = jd / 3600.0 * PI / 180.0;     // 将以秒为单位的经度转换成弧度
    wd_hd = wd / 3600.0 * PI / 180.0;    // 将以秒为单位的纬度转换成弧度
                                         // 如果不设中央经线（缺省参数: -1000），则计算中央经线，
                                         // 否则，使用传入的中央经线，不再使用带号和带宽参数
                                         //L = (DH - 0.5) * DH_width       // 计算中央经线的经度
    if (LP == -1000)
    {
        L = (DH - 0.5) * DH_width;
        // 计算中央经线的经度
    }
    else
    {
        L = LP;
    }
    l0 = jd / 3600.0 - L;       // 计算经差
    tsin = sin(wd_hd);        // 计算sinB
    tcos = cos(wd_hd);        // 计算cosB
                              // 计算克拉索夫斯基椭球中子午弧长X
                              //X = 111134.8611 / 3600.0 * wd - (32005.7799 * tsin + 133.9238 * pow(tsin, 3) + 0.6976 * pow(tsin, 5) + 0.0039 * pow(tsin, 7)) * tcos;
    X = 111132.9558 / 3600.0*wd - 16038.6496*sin(2 * wd_hd) + 16.8607*sin(4 * wd_hd) - 0.0220*sin(6 * wd_hd);
    et2 = b_e2 * pow(tcos, 2); //  et2 = (e' ** 2) * (cosB ** 2)
    et3 = b_e2 * pow(tsin, 2);
    N = b_c / sqrt(1 - et3);     //  N = C / sqrt(1 + et2)
    t = tan(wd_hd);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    *x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);

    *y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
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
