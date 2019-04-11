#ifndef __FILE_MONITOR_H__
#define __FILE_MONITOR_H__

#include <dirent.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "sc_msgs/MonitorMsg.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"

class DiskMonitor {
public:
    DiskMonitor();
    ~DiskMonitor();
    void Run(const std::string &_projectPath, sc_msgs::MonitorMsg &_pMonitorMsg);


private:
    static bool IsNotProject(const std::string &_dirName);
    void GetProjects(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg);
    void GetDiskUsage(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg);
    void FilterProjects(std::vector<std::string> &dirs);
    void SortProjects(std::vector<std::string> &projects);
    bool IsProjectCollated(const std::string &_dirName);
};

#endif

