/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>
#include "velodyne_msgs/Velodyne2Center.h"

namespace velodyne_driver
{

static double getDaySecond(const double rosTime, const double pktTime);

class VelodyneDriver
{
public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(int64_t isSaveLidar);

private:

  ///Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::
              VelodyneNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    double time_offset;              ///< time in seconds added to each velodyne time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  ros::Publisher pub2Center_;

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  std::string mRecordFile;
};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
