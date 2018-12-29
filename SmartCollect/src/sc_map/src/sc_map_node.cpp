#include "base_map.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if(2 != argc) {
        LOG(ERROR) << "usage: exec path_rawdata.";
        return -1;
    }
    ros::init(argc, argv, "sc_map_node");

    const std::string rawPath(argv[1]);
    BaseMap baseMapper(ros::NodeHandle(), ros::NodeHandle("~"), rawPath);
    baseMapper.run();
    return 0;
}