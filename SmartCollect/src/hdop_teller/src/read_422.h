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
#include <std_msgs/String.h>
#include <termios.h>

// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
bool getGpgga(const unsigned char *inBuf, const size_t &bufSize, string &gpgga, bool &isGpggaStart);
bool getGdopFromGpgga(const string &inGpgga, string &outGdop);

#endif // __READ_422_H