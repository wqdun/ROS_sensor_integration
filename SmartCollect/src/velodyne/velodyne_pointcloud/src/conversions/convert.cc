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
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData()),
    pPcVecMsg_(new velodyne_msgs::VelodynePcVec)
  {
    data_->setup(private_nh);
    pPcVecMsg_->pc_vec.clear();

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    outputPcVec_ = node.advertise<velodyne_msgs::VelodynePcVec>("velodyne_points_vector", 10);
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
  }

  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  LOG(INFO) << "Reconfigure Request";
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    // if (output_.getNumSubscribers() == 0)         // no one listening?
    //   return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr
      outMsg(new velodyne_rawdata::VPointCloud());
    velodyne_rawdata::VPointCloud::Ptr pPcImuCoord(new velodyne_rawdata::VPointCloud());

    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg->header.frame_id = pPcImuCoord->header.frame_id = scanMsg->header.frame_id;
    outMsg->height = pPcImuCoord->height = 1;

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack_vlp16(scanMsg->packets[i], *outMsg, *pPcImuCoord, pPcVecMsg_->pc_vec);
      }

    LOG(INFO) << "pPcVecMsg_->pc_vec.size(): " << pPcVecMsg_->pc_vec.size();
    // ~~760/s
    if(pPcVecMsg_->pc_vec.size() >= 10000) {
      // process and clear
      outputPcVec_.publish(pPcVecMsg_);
      pPcVecMsg_->pc_vec.clear();
    }

    // publish the accumulated cloud message
    DLOG(INFO) << "Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp;
    output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud
