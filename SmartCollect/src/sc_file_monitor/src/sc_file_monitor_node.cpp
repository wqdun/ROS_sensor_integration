#include "file_monitor.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "sc_file_monitor_node");

    FileMonitor fileMonitor(ros::NodeHandle(), ros::NodeHandle("~") );
    fileMonitor.run();
    return 0;
}