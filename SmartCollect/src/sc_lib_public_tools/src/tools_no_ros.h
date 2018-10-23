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
    static int setSerialOption(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    static double string2double(const std::string& str);
    static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP);


private:

};


}
// namespace public_tools

#endif

