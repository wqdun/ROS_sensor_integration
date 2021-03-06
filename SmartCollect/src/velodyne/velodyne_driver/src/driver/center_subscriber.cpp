#include "center_subscriber.h"
#include <glog/logging.h>

CenterSubscriber::CenterSubscriber(ros::NodeHandle node, ros::NodeHandle private_nh) {
    isSaveLidar_ = false;
    subClient_ = node.subscribe("sc_monitor", 10, &CenterSubscriber::clientCB, this);
}

void CenterSubscriber::clientCB(const sc_msgs::MonitorMsg::ConstPtr& pClientMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, is_record LIDAR: " << (int)(pClientMsg->is_record);
    isSaveLidar_ = pClientMsg->is_record;
}
