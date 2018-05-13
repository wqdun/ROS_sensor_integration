#ifndef __INTEGRATE_IMU_RECORDER_H
#define __INTEGRATE_IMU_RECORDER_H

#include <ros/ros.h>
#include <fstream>
#include "sc_msgs/scIntegrateImu.h"
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <boost/algorithm/string/split.hpp>
#include <fcntl.h>
#include <boost/algorithm/string/classification.hpp>
#include <termios.h>
#include "../../sc_lib_public_tools/src/public_tools.h"

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
bool getGpgga(const unsigned char *inBuf, const size_t &bufSize, string &gpgga, bool &isGpggaStart);
bool getGdopFromGpgga(const string &inGpgga, sc_msgs::scIntegrateImu &out422Msg, const string &timeErrFile);

#endif // __INTEGRATE_IMU_RECORDER_H