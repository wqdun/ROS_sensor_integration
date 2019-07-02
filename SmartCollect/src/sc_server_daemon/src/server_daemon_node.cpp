#include <glog/logging.h>
#include <ros/ros.h>
#include "server_factory.h"
#include "base_server.h"

int main(int argc, char **argv) {
    // server node glog path:
    FLAGS_log_dir = "/opt/smartc/log";
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "server_daemon_node");

    ServerFactory serverFactory;
    serverFactory.SetSensorType("/opt/smartc/config/sensor_type.yaml");
    BaseServer *pServer = serverFactory.CreateServer();

    LOG(INFO) << "Got " << argc << " parameters.";
    if (2 == argc) {
        pServer->UpdateProjectInfo(argv[1]);
    }
    pServer->Run();

    delete pServer;
    pServer = NULL;
    return 0;
}