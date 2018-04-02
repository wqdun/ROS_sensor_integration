#ifndef _CENTER_SUBSCRIBER_H_
#define _CENTER_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sc_server_daemon/nodeParams.h>

class CenterSubscriber {
public:
    CenterSubscriber(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~CenterSubscriber() {}

    int64_t isSaveLidar_;


private:
    void clientCB(const sc_server_daemon::nodeParams::ConstPtr& pClientMsg);
    ros::Subscriber subClient_;
};

#endif
