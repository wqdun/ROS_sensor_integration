#include "point_cloud_registration.h"

pointCloudRegistration::pointCloudRegistration() {
    lidarBegTime_ = -1.;
    subPc_ = nh_.subscribe("velodyne_points_vector", 10, &pointCloudRegistration::pcCB, this);
    subImu_ = nh_.subscribe("imu_time2local", 10, &pointCloudRegistration::imuTime2LocalCB, this);

}

pointCloudRegistration::~pointCloudRegistration() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye, Point Cloud Registration.";
}

void pointCloudRegistration::run() {
    ros::spin();
}

// ~~0.1Hz
void pointCloudRegistration::pcCB(const velodyne_msgs::VelodynePcVec::Ptr &pPcMsg) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    if(pPcMsg->pc_vec.empty() ) {
        LOG(ERROR) << "pPcMsg->pc_vec is empty.";
        exit(1);
    }

    DLOG(INFO) << "pPcMsg->pc_vec.size(): " << pPcMsg->pc_vec.size();
    // pPcMsg = reservedPcMsg_ + pPcMsg
    std::move(reservedPcMsg_.begin(), reservedPcMsg_.end(), std::inserter(pPcMsg->pc_vec, pPcMsg->pc_vec.begin() ) );
    reservedPcMsg_.clear();
    DLOG(INFO) << "pPcMsg->pc_vec.size(): " << pPcMsg->pc_vec.size();

    velodyne_rawdata::VPointCloud pc;
    pcl::moveFromROSMsg( (pPcMsg->pc_vec)[0], pc);
    DLOG(INFO) << "First pc.header.stamp: " << pc.header.stamp;
    lidarBegTime_ = (pcl_conversions::fromPCL(pc.header.stamp) ).toSec();
    pcl::moveFromROSMsg( (pPcMsg->pc_vec).back(), pc);
    DLOG(INFO) << "Last pc.header.stamp: " << pc.header.stamp;
    double lidarEndTime = (pcl_conversions::fromPCL(pc.header.stamp) ).toSec();
    LOG(INFO) << "LIDAR time [" << lidarBegTime_ << ", " << lidarEndTime << "].";

    const int hour = (int)lidarBegTime_ / 3600 % 24;
    const int minute = (int)lidarBegTime_ / 60 % 60;
    const double second = fmod(lidarBegTime_, 60);
    LOG(INFO) << "First pc.header.stamp: " << hour << ":" << minute << ":" << second;

    // parse LIDAR data among [lidarBegTime_, imuEndTime]; keep rest LIDAR data (imuEndTime, ) for next parsing
    // [lidarBegTime_(= -1), imuEndTime] means no LIDAR data to parse, all reserve
    const double imuBegTime = (imuPoints_.empty() )? -1.: (imuPoints_[0].day_second);
    const double imuEndTime = (imuPoints_.empty() )? -1.: (imuPoints_.back().day_second);
    LOG(INFO) << "IMU time [" << imuBegTime << ", " << imuEndTime << "].";
    std::vector<velodyne_rawdata::VPointCloud> pcVec;
    pcVec.clear();
    for(auto iterPc = (pPcMsg->pc_vec).begin(); iterPc != (pPcMsg->pc_vec).end(); ++iterPc) {
        pcl::moveFromROSMsg(*iterPc, pc);
        if( (pcl_conversions::fromPCL(pc.header.stamp) ).toSec() <= imuEndTime) {
            pcVec.push_back(pc);
        }
        else {
            reservedPcMsg_.push_back(*iterPc);
        }
    }

    // TODO: pcVec

    if(reservedPcMsg_.size() > 20000) {
        LOG(INFO) << "reservedPcMsg_.size(): " << reservedPcMsg_.size();
        LOG(INFO) << "Point Cloud size too big, IMU time: " << imuEndTime << " might be wrong.";
        reservedPcMsg_.erase(reservedPcMsg_.begin(), reservedPcMsg_.begin() + 6000);
    }
    LOG(INFO) << "reservedPcMsg_.size(): " << reservedPcMsg_.size();
    if(reservedPcMsg_.empty() ) {
        lidarBegTime_ = -1.;
    }
    else {
        pcl::moveFromROSMsg(reservedPcMsg_[0], pc);
        lidarBegTime_ = (pcl_conversions::fromPCL(pc.header.stamp) ).toSec();
    }
}

void pointCloudRegistration::imuTime2LocalCB(const ntd_info_process::imuPoints::Ptr &pImuMsg) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    // append IMU data to member var
    std::move(pImuMsg->imu_points.begin(), pImuMsg->imu_points.end(), std::back_inserter(imuPoints_) );
    LOG(INFO) << "imuPoints_ size: " << imuPoints_.size();
    DLOG(INFO) << "LIDAR begin time: " << lidarBegTime_;
    DLOG(INFO) << "IMU begin time: " << (imuPoints_.empty()? -1.: imuPoints_[0].day_second);
    // find LIDAR begin time <-> IMU day second
    for(auto iter = imuPoints_.begin(); iter != imuPoints_.end(); ++iter) {
        if(lidarBegTime_ < iter->day_second) {
            // e.g., lidarBegTime_ is 0, and imuPoints_ is 3, 4, 5...
            if(imuPoints_.begin() == iter) {
                LOG(INFO) << "LIDAR begin time: " << lidarBegTime_ << " < IMU begin time: " << imuPoints_[0].day_second;
                break;
            }
            --iter;
            // erase IMU data whose time < lidarBegTime_
            imuPoints_.erase(imuPoints_.begin(), iter);
            break;
        }
    }

    // if imuPoints_ too big
    if(imuPoints_.size() > 10000) {
        imuPoints_.erase(imuPoints_.begin(), imuPoints_.begin() + 6000);
    }
    LOG(INFO) << "imuPoints_ size: " << imuPoints_.size();
}

