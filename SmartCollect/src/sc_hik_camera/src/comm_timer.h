#ifndef __COMM_TIME_H
#define __COMM_TIME_H

#include <glog/logging.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include "sc_msgs/imu5651.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

class CommTimer {
public:
    CommTimer(const std::string &_serialName, const std::string &_imuPath);
    ~CommTimer();
    int Run();
    double GetUnixTimeMinusGpsTime();
    void PublishMsg();


private:
    void ReadSerial(int _fd);
    void WriteSerial(int _fd);
    void Parse5651Frame(const std::string &_frame, double _unixTime);
    void Parse5651GpggaFrame(const std::string &_gpggaFrame);
    void WriteRtImuFile(const std::string &_gpfpdFrame);
    void Parse5651GpfpdFrame(const std::string &_gpfpdFrame, double __unixTime);

    ros::NodeHandle nh_;
    ros::Publisher pubImu5651_;
    sc_msgs::imu5651 imu232Msg_;
    std::string serialName_;
    std::string rtImuFile_;
    double unixTimeMinusGpsTime_;
};

#endif
