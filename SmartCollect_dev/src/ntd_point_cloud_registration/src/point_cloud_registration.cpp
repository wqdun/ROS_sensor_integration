#include "point_cloud_registration.h"

pointCloudRegistration::pointCloudRegistration() {
    subPc_ = nh_.subscribe("velodyne_points_vector", 10, &pointCloudRegistration::pcCB, this);

}

pointCloudRegistration::~pointCloudRegistration() {

}

void pointCloudRegistration::run() {

}

// ~~0.1Hz
void pointCloudRegistration::pcCB(const velodyne_msgs::VelodynePcVec::Ptr &pPcMsg) {
    if(pPcMsg->pc_vec.empty() ) {
        // ERROR
        return;
    }
    velodyne_rawdata::VPointCloud pc;
    pcl::moveFromROSMsg(pPcMsg->pc_vec[0], pc);
    LOG(INFO) << pc.header.stamp;
    ros::Time beginTime(pcl_conversions::fromPCL(pc.header.stamp) );


}


