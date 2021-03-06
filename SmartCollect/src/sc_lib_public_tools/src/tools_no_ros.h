#ifndef PUBLIC_TOOLS_NO_ROS_H
#define PUBLIC_TOOLS_NO_ROS_H

#include <termios.h>
#include <unistd.h>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <algorithm>

namespace public_tools
{

class ToolsNoRos {
public:
    static int SetSerialOption(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    static double string2double(const std::string& str);
    static int string2int(const std::string& str);
    static void GeoToGauss(double longitude, double latitude, short beltWidth, int beltNumber, double &y, double &x);
    static bool IsFileExist(const std::string& fileName);
    static long GetFileSizeInByte(const std::string& filename);
    static double CalcUnixTimeByGpsWeek(int gpsWeek, double gpsWeekSecond);
    static bool IsWhiteSpace(char c);
    static void TrimWhiteSpaceInString(std::string &inputString);

    static bool isOK_;


private:

};


}
// namespace public_tools

#endif

