#include "tools_no_ros.h"
#include <glog/logging.h>

namespace public_tools
{
double ToolsNoRos::string2double(const std::string& str) {
    if(str.empty() ) {
        return -1;
    }
    std::istringstream iss(str);
    double num;
    iss >> num;
    return num;
}

int ToolsNoRos::string2int(const std::string& str) {
    if(str.empty() ) {
        return -1;
    }
    std::istringstream iss(str);
    int num;
    iss >> num;
    return num;
}

void ToolsNoRos::GeoToGauss(double longitude, double latitude, short beltWidth, int beltNumber, double &y, double &x) {
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double longitude2Rad, latitude2Rad;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos, et3;   //  sinB,cosB
    double PI = 3.14159265358979;
    double b_e2 = 0.00669437999013;
    double b_c = 6378137;
    longitude2Rad = longitude * PI / 180.0;
    latitude2Rad = latitude * PI / 180.0;

    if(3==beltWidth)
    {
        L=beltNumber*beltWidth;
    }
    if(6==beltWidth)
    {
        beltNumber=(longitude+6)/beltWidth;
        L=beltNumber*beltWidth-3;
    }

    l0 = longitude - L;       // 计算经差
    tsin = sin(latitude2Rad);        // 计算sinB
    tcos = cos(latitude2Rad);        // 计算cosB

    X = 111132.9558 *latitude - 16038.6496*sin(2 * latitude2Rad) + 16.8607*sin(4 * latitude2Rad) - 0.0220*sin(6 * latitude2Rad);
    et2 = b_e2 * pow(tcos, 2); //  et2 = (e' ** 2) * (cosB ** 2)
    et3 = b_e2 * pow(tsin, 2);
    N = b_c / sqrt(1 - et3);     //  N = C / sqrt(1 + et2)
    t = tan(latitude2Rad);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);

    y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
}

int ToolsNoRos::SetSerialOption(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
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

bool ToolsNoRos::IsFileExist(const std::string& fileName) {
    return (access(fileName.c_str(), 0) >= 0);
}

long ToolsNoRos::GetFileSizeInByte(const std::string& filename) {
    if(!IsFileExist(filename) ) {
        return 0;
    }

    struct stat statbuf;
    stat(filename.c_str(), &statbuf);
    return statbuf.st_size;
}

double ToolsNoRos::CalcUnixTimeByGpsWeek(int gpsWeek, double gpsWeekSecond) {
    return (gpsWeek * 7 * 24 * 3600 + gpsWeekSecond + 315964800);
}

bool ToolsNoRos::IsWhiteSpace(char c) {
    return std::isspace(c);
}

void ToolsNoRos::TrimWhiteSpaceInString(std::string &inputString) {
    inputString.erase(std::remove_if(inputString.begin(), inputString.end(), IsWhiteSpace), inputString.end());
}

}

// namespace public_tools
