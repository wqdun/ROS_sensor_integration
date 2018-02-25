#ifndef _CENTER_SUBSCRIBER_H_
#define _CENTER_SUBSCRIBER_H_

#include <ros/ros.h>
#include <std_msgs/Int64.h>

class CenterSubscriber {
public:
    CenterSubscriber(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~CenterSubscriber() {}

    int64_t isSaveLidar_;


private:
    void centerCB(const std_msgs::Int64::ConstPtr& pCenterMsg);
    ros::Subscriber subCenter_;
};

#endif
