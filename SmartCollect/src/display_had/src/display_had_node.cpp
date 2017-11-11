#include "mif_read.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "displayer_node");

    MifReader mif_reader(ros::NodeHandle(), ros::NodeHandle("~"));
    mif_reader.run();
    DLOG(INFO) << "Had displayer.";
    return 0;
}