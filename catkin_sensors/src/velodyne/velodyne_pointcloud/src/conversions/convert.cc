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

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

using std::string;

namespace velodyne_pointcloud
{
  using velodyne_rawdata::point_t;
  using velodyne_rawdata::deg2rad;
  using velodyne_rawdata::tf_rotate;

  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData()),
    mHeading(0)
  {
    data_->setup(private_nh);

    gps_sub_ = node.subscribe("processed_infor_msg", 1000, &Convert::getStatusCB, this);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    // min capacity is 100, to optimize efficiency
    mWGSCloudArr.reserve(100);
  }

  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr
      outMsg(new velodyne_rawdata::VPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], *outMsg, mHeading);
      }

    // publish the accumulated cloud message
    // ROS_INFO_STREAM("Publishing " << outMsg->height * outMsg->width
                     // << " Velodyne points, time: " << outMsg->header.stamp);
    // output_.publish(outMsg);

    // customized code: outMsg is array of VPoint; traverse it to WGS
    size_t pointCloudSize = outMsg->points.size();
    // about 20000
    ROS_INFO_STREAM("pointCloudSize: " << pointCloudSize);
    // ROS_INFO_STREAM("mCurrentWGS.x: " << mCurrentWGS.x << "; mCurrentWGS.y: " << mCurrentWGS.y << "; mCurrentWGS.z: " << mCurrentWGS.z);
    for(size_t i = 0; i < pointCloudSize; ++i) {
      point_t in_point = {
        outMsg->points[i].x,
        outMsg->points[i].y,
        outMsg->points[i].z
      };

      point_t angle_xyz = {
        deg2rad(mPitch),
        deg2rad(mRoll),
        deg2rad(mHeading)
      };

      point_t offset = {
        mCurrentWGS.x,
        mCurrentWGS.y,
        mCurrentWGS.z
      };

      tf_rotate(in_point, angle_xyz, offset, outMsg->points[i]);
    }

    // output_.publish(outMsg);

    // now outMsg is WGS, unit: m; and it contains history data
    mWGSCloudArr.push_back(*outMsg);

    // system crash when set 1000
    static const int MAX_SIZE = 60;
    // give WGS points to HuangBo when size reaches 60: about 100'000 points
    if(mWGSCloudArr.size() < MAX_SIZE) {
      // mWGSCloudArr.erase(mWGSCloudArr.begin(), mWGSCloudArr.begin() + MAX_SIZE / 2);
      return;
    }

    // BELOW ONLY for display DEBUG
    // modify history WGS coordinate: keep car(or IMU) position Origin
    velodyne_rawdata::VPointCloud::Ptr pWGSCoordCloud(new velodyne_rawdata::VPointCloud());

    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    // time of last frame
    pWGSCoordCloud->header.stamp = outMsg->header.stamp;
    pWGSCoordCloud->header.frame_id = outMsg->header.frame_id;
    pWGSCoordCloud->height = 1;

    velodyne_rawdata::VPoint imuCoord;
    for(size_t i = 0; i < mWGSCloudArr.size(); ++i) {
      for(size_t j = 0; j < mWGSCloudArr[i].points.size(); ++j) {
        imuCoord.x = mWGSCloudArr[i].points[j].x - mCurrentWGS.x;
        imuCoord.y = mWGSCloudArr[i].points[j].y - mCurrentWGS.y;
        imuCoord.z = mWGSCloudArr[i].points[j].z - mCurrentWGS.z;

        imuCoord.ring = mWGSCloudArr[i].points[j].ring;
        imuCoord.intensity = mWGSCloudArr[i].points[j].intensity;
        pWGSCoordCloud->points.push_back(imuCoord);
        ++(pWGSCoordCloud->width);
      }
    }

    ROS_INFO_STREAM("Publishing " << pWGSCoordCloud->height * pWGSCoordCloud->width
                     << " Velodyne points, time: " << pWGSCoordCloud->header.stamp);
    output_.publish(pWGSCoordCloud);
    mWGSCloudArr.clear();
  }

  void Convert::getStatusCB(const ntd_info_process::processed_infor_msg::ConstPtr& imuMsg) {
    mPitch = imuMsg->current_pitch;
    mRoll = imuMsg->current_roll;
    mHeading = imuMsg->current_heading;
    mCurrentWGS = imuMsg->current_wgs;
    mGPStime = imuMsg->GPStime;

    ROS_INFO_STREAM("imuMsg->Heading:" << mHeading << " at " << mGPStime << ":" << mPitch << ":" << mRoll << ":" << mCurrentWGS.x);
  }

  // static tm GpsWeekTime2HumanTime(const double ) {

  // }



} // namespace velodyne_pointcloud
