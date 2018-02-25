/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw Velodyne LIDAR packets to PointCloud2.

*/

#include <ros/ros.h>
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

#include "convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  velodyne_pointcloud::Convert conv(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
