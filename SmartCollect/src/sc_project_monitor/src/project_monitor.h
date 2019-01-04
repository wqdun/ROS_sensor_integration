#ifndef __PROJECT_MONITOR_H
#define __PROJECT_MONITOR_H

#include <glog/logging.h>
#include <ros/ros.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "sc_msgs/DiskInfo.h"

class ProjectMonitor {
public:
    ProjectMonitor(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~ProjectMonitor();
    void run();
    void setRawdataPath(const std::string &_rawdataPath);


private:
    ros::Publisher pub2web_;
    std::string rawdataPath_;
    int lastImgNum_;

    long GetRawInsSizeInByte();
};

#endif // __PROJECT_MONITOR_H
