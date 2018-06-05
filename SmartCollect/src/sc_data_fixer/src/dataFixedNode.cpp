#include "dataFixed.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    FLAGS_log_dir = "/opt/smartc/log/";
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if(2 != argc) {
        LOG(ERROR) << "usage: exec project_1,project_2...";
        return -1;
    }
    ros::init(argc, argv, "data_fixer_node");

    int imageCollectionHz = 10;
    dataFixed dataFixer(ros::NodeHandle(), ros::NodeHandle("~"), imageCollectionHz);

    const std::string projects(argv[1]);
    dataFixer.fixProjectsData(projects);
    return 0;
}


