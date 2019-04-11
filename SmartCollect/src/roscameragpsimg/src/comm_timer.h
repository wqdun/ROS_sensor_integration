#ifndef __COMM_TIME_H
#define __COMM_TIME_H

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
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
#include <string>
#include "sc_msgs/imu5651.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/coordtrans.h"
#include <fstream>
#include "lock.h"

class Cameras;

class CommTimer
{
public:
    CommTimer(const std::string &_rawdataPath);
    ~CommTimer();

    int getTime(Cameras *pCameras);

    sc_msgs::imu5651 imu232Msg_;


private:
    int setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

    std::string rawdataPath_;
    ros::Publisher pubImu5651_;
};

#endif
