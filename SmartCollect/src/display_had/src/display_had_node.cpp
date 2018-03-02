#include "mif_read.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "displayer_node");

    MifReader mif_reader(ros::NodeHandle(), ros::NodeHandle("~"));
    mif_reader.run();
    LOG(INFO) << "HAD displayer.";
    return 0;
}