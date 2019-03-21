#ifndef __FILE_MONITOR_H__
#define __FILE_MONITOR_H__

#include <dirent.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "sc_msgs/MonitorMsg.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

class DiskMonitor {
public:
    DiskMonitor();
    ~DiskMonitor();
    void run(const std::string &_projectPath, sc_msgs::MonitorMsg &_pMonitorMsg);


private:
    bool isProject(const std::string &_dirName);
    void getProjects(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg);
    void getDiskUsage(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg);
};

#endif

