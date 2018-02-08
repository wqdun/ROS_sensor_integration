#include "center.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "center_node");

    // create process class, which subscribes to input messages
    InforProcess inforProcessor;

    // handle callbacks until shut down
    inforProcessor.run();

    return 0;
}