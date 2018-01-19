#ifndef __READ_422_H
#define __READ_422_H

#include <ros/ros.h>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <boost/algorithm/string/split.hpp>
#include <fcntl.h>
#include <boost/algorithm/string/classification.hpp>
#include <termios.h>
#include "hdop_teller/imu5651_422.h"
#include "../../public_tools/public_tools.h"

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
bool getGpgga(const unsigned char *inBuf, const size_t &bufSize, string &gpgga, bool &isGpggaStart);
bool getGdopFromGpgga(const string &inGpgga, hdop_teller::imu5651_422 &out422Msg);

#endif // __READ_422_H