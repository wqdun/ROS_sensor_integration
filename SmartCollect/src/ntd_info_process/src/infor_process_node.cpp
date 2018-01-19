#include "infor_process.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    if(2 != argc) {
        LOG(ERROR) << "I need a event file path, argc: " << argc;
        exit(1);
    }
    string eventFilePath("");
    eventFilePath = argv[1];
    LOG(INFO) << "Save event to: "<< eventFilePath;

    ros::init(argc, argv, "ntd_info_process_node");

    // create process class, which subscribes to input messages
    InforProcess inforProcessor(eventFilePath);

    // handle callbacks until shut down
    inforProcessor.run();

    return 0;
}