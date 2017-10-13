/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>
#include <map>

#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>
// #include "imupac/imu5651.h"
#include "ntd_info_process/processed_infor_msg.h"

namespace velodyne_pointcloud
{
  using std::vector;
  using std::map;

  typedef struct {
    double pitch;
    double roll;
    double heading;
    geometry_msgs::Point currentWGS;
  } imuInfo_t;

  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}

    void Run();

  private:

    void callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level);
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);
    // void getHeadingCallback(const imupac::imu5651::ConstPtr& imuMsg);
    void getStatusCB(const ntd_info_process::processed_infor_msg::ConstPtr& imuMsg);
    imuInfo_t LookUpMap(const map<double, imuInfo_t> &map, const double x);


    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > srv_;

    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    ros::Subscriber velodyne_scan_;
    ros::Subscriber gps_sub_;

    ros::Publisher output_;
    ros::Publisher m_output4calc;

    /// configuration parameters
    typedef struct {
      int npackets;                    ///< number of packets to combine
    } Config;
    Config config_;

    imuInfo_t mImuInfo;
    map<double, imuInfo_t> mTime2ImuInfo;

    map<double, velodyne_rawdata::VPointCloud> mTime2Pc;
    // actually no time needed below
    // map<double, velodyne_rawdata::VPointCloud> mTime2PcWGS;
    velodyne_rawdata::VPointCloud mPcWGS;

    bool mIsSavedEnoughPackets;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
