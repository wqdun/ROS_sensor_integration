#include "infor_process.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "ntd_info_process_node");

    // create process class, which subscribes to input messages
    InforProcess inforProcessor;

    // handle callbacks until shut down
    ros::spin();

    return 0;
}