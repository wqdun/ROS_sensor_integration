#include "disk_monitor.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

DiskMonitor::DiskMonitor() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

DiskMonitor::~DiskMonitor() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye.";
}

void DiskMonitor::run(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg) {
    LOG(INFO) << __FUNCTION__ << " start.";

    (void)getDiskUsage(_projectPath, _monitorMsg);
    (void)getProjects(_projectPath, _monitorMsg);
    LOG(INFO) << __FUNCTION__ << " end.";
}

void DiskMonitor::getProjects(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, to monitor " << _projectPath;

    std::string lsCmd("ls -t " + _projectPath);
    if(public_tools::PublicTools::PopenWithReturn(lsCmd, _monitorMsg.projects) < 0) {
        LOG(ERROR) << "Failed to " << lsCmd;
        exit(1);
    }

    return;
}

void DiskMonitor::getDiskUsage(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg) {
    LOG(INFO) << __FUNCTION__ << " start.";
    std::vector<std::string> diskUsage;
    const std::string getDiskUsageCmd("df -BG /opt/smartc/record/ | tail -n1 | awk '{print $2\", \"$5}'");
    (void)public_tools::PublicTools::PopenWithReturn(getDiskUsageCmd, diskUsage);
    _monitorMsg.disk_usage = (1 == diskUsage.size())? diskUsage[0]: "error: I got !1 lines";
}

bool DiskMonitor::isProject(const std::string &_dirName) {
    LOG(INFO) << __FUNCTION__ << " start, param: " << _dirName;

    std::vector<std::string> splitDirName;
    (void)boost::split(splitDirName, _dirName, boost::is_any_of( "-" ));

    const size_t numOfDash = splitDirName.size() - 1;
    if(3 != numOfDash) {
        LOG(INFO) << _dirName << " has " << numOfDash << " -, not a project.";
        return false;
    }
    return true;
}
