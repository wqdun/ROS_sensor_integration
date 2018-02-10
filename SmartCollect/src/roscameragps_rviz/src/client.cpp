#include "client.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

Client::Client(ros::NodeHandle nh, MyViz *pViz) {
    pViz_ = pViz;
    subCenter_ = nh.subscribe("processed_infor_msg", 0, &Client::centerCB, this);
    subServer_ = nh.subscribe("sc_server2client", 0, &Client::serverCB, this);

    pubIsRecord_ = nh.advertise<std_msgs::Int64>("msg_save_control", 1);
    pubClientCmd_ = nh.advertise<SmartCollector::clientCmd>("sc_client_cmd", 10);

}

Client::~Client() {

}

void Client::run() {
    ros::Rate loop_rate(1);
    size_t freqDivider = 0;

    while(ros::ok() ) {
        ++freqDivider;
        freqDivider %= 256;
        ros::spinOnce();
        loop_rate.sleep();

        pubClientCmd_.publish(pViz_->clientCmdMsg_);

        pViz_->showCenterMsg(serverMsg_, centerMsg_);
        // 0.5 Hz set is_server_connected false
        if(0 == (freqDivider % 2) ) {
            // for server pulse is 1Hz
            serverMsg_.is_server_connected = false;
        }
    }
}

void Client::centerCB(const sc_center::centerMsg::ConstPtr& pInfos) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    centerMsg_ = *pInfos;
}

void Client::serverCB(const sc_server_daemon::serverMsg::ConstPtr &pServerMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, pServerMsg: " << (int64_t)(pServerMsg->is_server_connected);
    serverMsg_ = *pServerMsg;
}

