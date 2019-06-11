#include "disk_monitor.h"
#include <glog/logging.h>

DiskMonitor::DiskMonitor() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

DiskMonitor::~DiskMonitor() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye.";
}

void DiskMonitor::Run(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg) {
    LOG(INFO) << __FUNCTION__ << " start.";

    (void)GetDiskUsage(_projectPath, _monitorMsg);
    (void)GetProjects(_projectPath, _monitorMsg);
    LOG(INFO) << __FUNCTION__ << " end.";
}

void DiskMonitor::GetProjects(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, to monitor " << _projectPath;

    std::string lsCmd("ls -t " + _projectPath);
    if(public_tools::PublicTools::PopenWithReturn(lsCmd, _monitorMsg.projects) < 0) {
        LOG(ERROR) << "Failed to " << lsCmd;
        _monitorMsg.is_disk_error = true;
    }

    for (auto &dir: _monitorMsg.projects) {
        public_tools::ToolsNoRos::TrimWhiteSpaceInString(dir);
    }

    FilterProjects(_monitorMsg.projects);
    SortProjects(_monitorMsg.projects);

    return;
}

void DiskMonitor::GetDiskUsage(const std::string &_projectPath, sc_msgs::MonitorMsg &_monitorMsg) {
    LOG(INFO) << __FUNCTION__ << " start.";
    std::vector<std::string> diskUsage;
    const std::string getDiskUsageCmd("df -BG /opt/smartc/record/ | tail -n1 | awk '{print $2\", \"$5}'");
    (void)public_tools::PublicTools::PopenWithReturn(getDiskUsageCmd, diskUsage);
    _monitorMsg.disk_usage = (1 == diskUsage.size())? diskUsage[0]: "error: I got !1 lines";
}

bool DiskMonitor::IsNotProject(const std::string &_dirName) {
    DLOG(INFO) << __FUNCTION__ << " start, param: " << _dirName;

    std::vector<std::string> splitDirName;
    (void)boost::split(splitDirName, _dirName, boost::is_any_of( "-" ));

    const size_t numOfDash = splitDirName.size() - 1;
    if(3 != numOfDash) {
        LOG(INFO) << _dirName << " has " << numOfDash << " -, not a project.";
        return true;
    }
    return false;
}

bool DiskMonitor::IsProjectCollated(const std::string &_dirName) {
    DLOG(INFO) << __FUNCTION__ << " start, param: " << _dirName;

    const std::string fullProjectPath("/opt/smartc/record/" + _dirName);
    return public_tools::PublicTools::isFileExist(fullProjectPath + "/Process/");
}

void DiskMonitor::FilterProjects(std::vector<std::string> &dirs) {
    LOG(INFO) << __FUNCTION__ << " start.";

    dirs.erase(std::remove_if(dirs.begin(), dirs.end(), IsNotProject), dirs.end());
    return;
}

void DiskMonitor::SortProjects(std::vector<std::string> &projects) {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> collatedProjects;
    collatedProjects.clear();
    std::vector<std::string> uncollatedProjects;
    uncollatedProjects.clear();

    for (const auto &project: projects) {
        if(IsProjectCollated(project)) {
            DLOG(INFO) << project << " has been collated.";
            collatedProjects.push_back(project + "(√)");
        }
        else {
            DLOG(INFO) << project << " has not been collated.";
            uncollatedProjects.push_back(project);
        }
    }

    projects.swap(uncollatedProjects);
    projects.insert(projects.end(), collatedProjects.begin(), collatedProjects.end());

    for (const auto &project: projects) {
        DLOG(INFO) << "project: " << project;
    }

}

