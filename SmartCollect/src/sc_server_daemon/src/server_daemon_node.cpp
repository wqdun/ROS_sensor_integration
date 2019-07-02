#include "server_daemon.h"

int main(int argc, char **argv) {
    // server node glog path:
    FLAGS_log_dir = "/opt/smartc/log";
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";

    ros::init(argc, argv, "server_daemon_node");

    ServerDaemon serverDaemoner;
    if (2 == argc) {
        serverDaemoner.UpdateProjectInfo(argv[1]);
    }

    serverDaemoner.Run();
    return 0;
}
