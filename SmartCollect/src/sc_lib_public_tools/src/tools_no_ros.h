#ifndef PUBLIC_TOOLS_NO_ROS_H
#define PUBLIC_TOOLS_NO_ROS_H

#include <termios.h>
#include <unistd.h>
#include <string>
#include <math.h>

namespace public_tools
{

class ToolsNoRos {
public:
    static int SetSerialOption(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    static double string2double(const std::string& str);
    static void GeoToGauss(double longitude, double latitude, short beltWidth, int beltNumber, double &y, double &x);

    static bool isOK_;


private:

};


}
// namespace public_tools

#endif

