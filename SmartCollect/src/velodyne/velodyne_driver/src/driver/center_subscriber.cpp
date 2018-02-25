#include "center_subscriber.h"

CenterSubscriber::CenterSubscriber(ros::NodeHandle node, ros::NodeHandle private_nh) {
    isSaveLidar_ = false;
    subCenter_ = node.subscribe("center_msg_save_control", 0, &CenterSubscriber::centerCB, this);
}

void CenterSubscriber::centerCB(const std_msgs::Int64::ConstPtr& pCenterMsg) {
    isSaveLidar_ = pCenterMsg->data;
}
