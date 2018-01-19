#ifndef _POINT_CLOUD_REGISTRATION_H
#define _POINT_CLOUD_REGISTRATION_H

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePcVec.h>
#include "ntd_info_process/imuPoints.h"

// #define NDEBUG
#undef NDEBUG
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
    ros::Subscriber subImu_;

    std::vector<ntd_info_process::imuPoint> imuPoints_;
    // lidarBegTime_ is for remove old IMU point from imuPoints_
    double lidarBegTime_;
    std::vector<sensor_msgs::PointCloud2> reservedPcMsg_;

    void pcCB(const velodyne_msgs::VelodynePcVec::Ptr &pPcMsg);
    void imuTime2LocalCB(const ntd_info_process::imuPoints::Ptr &pImuMsg);
};







#endif