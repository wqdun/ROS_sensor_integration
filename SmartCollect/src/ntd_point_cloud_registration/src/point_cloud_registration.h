#ifndef _POINT_CLOUD_REGISTRATION_H
#define _POINT_CLOUD_REGISTRATION_H

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePcVec.h>

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
#include "velodyne_pointcloud/rawdata.h"

class pointCloudRegistration {
public:
    pointCloudRegistration();
    ~pointCloudRegistration();
    void run();


private:
    ros::NodeHandle nh_;
    ros::Subscriber subPc_;

    void pcCB(const velodyne_msgs::VelodynePcVec::Ptr &pPcMsg);

};







#endif