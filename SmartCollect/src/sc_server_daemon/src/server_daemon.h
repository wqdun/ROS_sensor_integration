#ifndef __SERVER_DAEMON_H
#define __SERVER_DAEMON_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include "SmartCollector/clientCmd.h"
#include "sc_server_daemon/serverMsg.h"

static bool isFileExist(const std::string& fileName);

class ServerDaemon {
public:
    ServerDaemon(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~ServerDaemon();
    void run();


private:
    ros::Subscriber subClient_;
    ros::Publisher pub2Client_;

    sc_server_daemon::serverMsg serverMsg_;
    std::string projectName_;
    bool isClientConnected_;

    void clientCB(const SmartCollector::clientCmd::ConstPtr& pClientMsg);
    void runSystemCmd(const std::string &systemCmd, const std::string &passwd);
};

#endif // __SERVER_DAEMON_H
