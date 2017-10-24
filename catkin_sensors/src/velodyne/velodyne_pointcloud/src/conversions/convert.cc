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

namespace velodyne_pointcloud
{
  using std::string;
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


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
    static size_t subCnt = 0;
    ROS_INFO_STREAM("Sub count:" << ++subCnt);
    // if (output_.getNumSubscribers() == 0)         // no one listening?
    //   return;                                     // avoid much work

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
        data_->mPktStr.clear();
        data_->unpack(scanMsg->packets[i], *outMsg);
        if(data_->mPktStr.empty()) {
          continue;
        }
        saveFile(data_->mPktStr);
      }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }

static int saveFile(const string &str2write) {
    static const size_t MAX_PKT_CNT = 100000;
    static size_t pktCnt = 0;
    static char fileName[50];
    static FILE *pOutFile;
    // get unix time stamp as file name
    if(0 == pktCnt) {
        time_t tt = time(NULL);
        tm *t= localtime(&tt);
        (void)sprintf(fileName, "%02d_%02d_%02d.lidar", t->tm_hour, t->tm_min, t->tm_sec);

        pOutFile = fopen(fileName, "wb");
        if(!pOutFile) {
            ROS_WARN_STREAM("Create file:" << fileName << " failed, errno:" << errno);
        }
        ROS_DEBUG_STREAM("Create file:" << fileName << " successfully.");
    }

    const char *cStr2write = str2write.c_str();
    const size_t len = strlen(cStr2write);
    const size_t tmp = fwrite(cStr2write, 1, len, pOutFile);
    if(len != tmp) {
      ROS_WARN_STREAM("Write error: should " << len << ", while: " << tmp);
    }

    ++pktCnt;
    if(MAX_PKT_CNT == pktCnt) {
      pktCnt = 0;
      fclose(pOutFile);
    }

    return 0;
}

} // namespace velodyne_pointcloud
