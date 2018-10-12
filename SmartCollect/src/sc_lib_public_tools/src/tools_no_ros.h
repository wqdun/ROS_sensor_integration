#ifndef PUBLIC_TOOLS_NO_ROS_H
#define PUBLIC_TOOLS_NO_ROS_H

#include <termios.h>
#include <unistd.h>
#include <string>

namespace public_tools
{

class ToolsNoRos {
public:
    static int setSerialOption(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    static double string2double(const std::string& str);


private:

};


}
// namespace public_tools

#endif

