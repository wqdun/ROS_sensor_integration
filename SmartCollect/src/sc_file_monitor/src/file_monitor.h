#ifndef __FILE_MONITOR_H__
#define __FILE_MONITOR_H__

#include <ros/ros.h>
#include <sc_msgs/ProjectArr.h>
#include <dirent.h>

class FileMonitor {
public:
    FileMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~FileMonitor();
    void run();


private:
    void getProjects(const std::string &_projectPath, sc_msgs::ProjectArr::Ptr &_pProjectArr);

    ros::Publisher pubProjectArr_;
};

#endif

