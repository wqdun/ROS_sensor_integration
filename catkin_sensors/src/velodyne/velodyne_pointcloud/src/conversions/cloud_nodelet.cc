/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts raw Velodyne 3D LIDAR packets to a
    PointCloud2.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "convert.h"

namespace velodyne_pointcloud
{
  class CloudNodelet: public nodelet::Nodelet
  {
  public:

    CloudNodelet() {}
    ~CloudNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Convert> conv_;
  };

  /** @brief Nodelet initialization. */
  void CloudNodelet::onInit()
  {

    conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle()));
    conv_->Run();
    // Convert *pCon = new Convert(getNodeHandle(), getPrivateNodeHandle());
    // pCon->Run();
    // conv_.reset(pCon);
    // Convert conv(getNodeHandle(), getPrivateNodeHandle());
    // conv.Run();

    // ROS_INFO_STREAM(__FUNCTION__);

  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pointcloud, CloudNodelet,
                        velodyne_pointcloud::CloudNodelet, nodelet::Nodelet);
