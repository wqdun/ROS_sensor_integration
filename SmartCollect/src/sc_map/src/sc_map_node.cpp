#include "base_map.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    FLAGS_log_dir = "/opt/smartc/log";
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "sc_map_node");

    BaseMap baseMapper(ros::NodeHandle(), ros::NodeHandle("~") );
    baseMapper.run();
    return 0;
}