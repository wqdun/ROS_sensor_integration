#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include <sstream>
#include "roscameragpsimg/imu5651.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/coordtrans.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;

//GPS时间

static double string2double(const string& str);

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

int get_time(void) ;//ros::NodeHandle* nh);
void coordtrans2wgsAndRecord(const std::string &_lon, const std::string &_lat);


//msg to show
//imu publisher
//roscameragpsimg::imu5651 msg_imu_string;
//ros::Publisher pub_imu_string;



