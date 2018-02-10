#ifndef __CLIENT_H___
#define __CLIENT_H___

#include <ros/ros.h>
#include "myviz.h"
#include "sc_server_daemon/serverMsg.h"

class Client {
public:
    Client(ros::NodeHandle nh, MyViz *pViz);
    ~Client();
    void run();


private:
    MyViz *pViz_;
    ros::Subscriber subCenter_;
    ros::Subscriber subServer_;

    ros::Publisher pubIsRecord_;
    ros::Publisher pubClientCmd_;

    sc_center::centerMsg centerMsg_;
    sc_server_daemon::serverMsg serverMsg_;

    std_msgs::Int64 isRecord_;

    void centerCB(const sc_center::centerMsg::ConstPtr& pInfos);
    void serverCB(const sc_server_daemon::serverMsg::ConstPtr &pServerPulse);
};




#endif // __CLIENT_H

