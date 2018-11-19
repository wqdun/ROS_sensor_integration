#include "server_daemon.h"

int main(int argc, char **argv) {
    // server node glog path:
    FLAGS_log_dir = "/opt/smartc/log";
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "server_daemon_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ServerDaemon serverDaemoner(node, private_nh);
    serverDaemoner.run();

    return 0;
}