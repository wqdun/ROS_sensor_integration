#include "project_monitor.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "project_monitor_node");
    ProjectMonitor projectMonitor(ros::NodeHandle(), ros::NodeHandle("~") );
    const std::string rawdataPath(argv[1]);
    LOG(INFO) << "rawdataPath: " << rawdataPath;
    projectMonitor.setRawdataPath(rawdataPath);
    projectMonitor.run();
    return 0;
}