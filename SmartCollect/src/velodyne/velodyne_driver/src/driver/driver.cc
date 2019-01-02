/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  lastMinute_ = -1;
  lidarPkgName_ = "";
  lastIsSaveLidar_ = false;
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));

  // e.g., mRecordFile: /home/navi/catkin_ws/record/1005-1-077-180110/rawdata/Lidar/
  private_nh.param("record_path", mRecordFile, mRecordFile);
  std::string lidarFileName("");
  // e.g., lidarFileName: 10051077180110150921
  (void)public_tools::PublicTools::generateFileName(mRecordFile, lidarFileName, false);
  // now we get /home/navi/catkin_ws/record/1005-1-077-180110/rawdata/Lidar/10051077180110
  mRecordFile += lidarFileName;
  ROS_INFO_STREAM("mRecordFile: " << mRecordFile);

  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") ||
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "VLP16")
    {
      packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
    }
  else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  // private_nh.param("rpm", config_.rpm, 600.0);
  config_.rpm = 1200;
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 50);
  pub2Center_ = node.advertise<velodyne_msgs::Velodyne2Center>("velodyne_pps_status", 0);
}

static bool parsePositionPkt(const char *pkt, velodyne_msgs::Velodyne2Center &parsedRes) {
    LOG_EVERY_N(INFO, 20) << __FUNCTION__ << " start.";
    // 00 .. 03
    parsedRes.pps_status_index = pkt[202];

    if('$' != pkt[206]) {
      LOG(INFO) << "No position packet received: " << std::hex << (int)pkt[206];
      // PPS_STATUS[0] is "No PPS", refer infor_process.h
      // A validity - A-ok, V-invalid, refer VLP-16 manual
      parsedRes.is_gprmc_valid = "N";
      return false;
    }

    // $GPRMC,,V,,,,,,,,,,N*53
    std::stringstream isGprmcValid;
    size_t dotCnt = 0;
    for(size_t i = 210; i < 230; ++i) {
      if(',' == pkt[i]) {
        ++dotCnt;
      }
      if(2 == dotCnt) {
        isGprmcValid << pkt[i + 1];
        break;
      }
    }

    // "3,A"
    parsedRes.is_gprmc_valid = isGprmcValid.str();
    return true;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(int64_t isSaveLidar)
{
  DLOG(INFO) << __FUNCTION__ << " start, isSaveLidar: " << isSaveLidar;
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  bool isPositionPkt = false;
  char positionPkt[POSITION_PACKET_SIZE];
  bzero(positionPkt, POSITION_PACKET_SIZE);
  for (int i = 0; i < config_.npackets; )// ++i)
    {
      while (true)
        {
          // keep reading until full packet received
          int rc = input_->getPacket(&scan->packets[i], config_.time_offset, isPositionPkt, positionPkt);
          if (rc == 0) {
            if(!isPositionPkt) {
              ++i;
            }
            break;       // got a full packet?
          }
          if (rc < 0) return false; // end of file reached?
        }
    }

  // publish message using time of last packet read
  DLOG(INFO) << "Publishing a full Velodyne scan.";
  scan->header.stamp = scan->packets[config_.npackets - 1].stamp;
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);

  velodyne_msgs::Velodyne2Center pps_status;
  // only last position pkt be published, ignore the rest
  (void)parsePositionPkt(positionPkt, pps_status);

  pps_status.velodyne_rpm = CalcLidarRpm(scan->packets.front(), scan->packets[(config_.npackets) / 4]);
  pub2Center_.publish(pps_status);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  bool isGonnaCreateNewLidarFile = false;

  if(lastIsSaveLidar_ != isSaveLidar) {
    lastIsSaveLidar_ = isSaveLidar;
    if(!isSaveLidar) {
      // last save, but now not save
      return true;
    }
    else {
      isGonnaCreateNewLidarFile = true;
    }
  }
  // last save and now save; or now not save and last not save
  else {
    if(!isSaveLidar) {
      return true;
    }
    // else: last save and now save; do nothing
  }

  // write LIDAR file
  time_t now = time(NULL);
  tm tmNow = { 0 };
  localtime_r(&now, &tmNow);
  const int nowMinute = tmNow.tm_hour * 100 + tmNow.tm_min;
  if(nowMinute != lastMinute_) {
    lastMinute_ = nowMinute;
    isGonnaCreateNewLidarFile = true;
  }

  if(isGonnaCreateNewLidarFile) {
    char nowSecond[50];
    (void)sprintf(nowSecond, "%02d%02d%02d", tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);
    // 10051077180110
    lidarPkgName_ = mRecordFile + nowSecond + "_lidar.dat";
    LOG(INFO) << "Create new file: " << lidarPkgName_;
  }

  FILE *pOutFile;
  if(!(pOutFile = fopen(lidarPkgName_.c_str(), "ab") ) ) {
    LOG(ERROR) << "Create file:" << lidarPkgName_ << " failed, errno:" << errno;
    exit(1);
  }
  DLOG(INFO) << "Create file:" << lidarPkgName_ << " successfully.";

  for(size_t i = 0; i < scan->packets.size(); ++i) {
    const double pktDaySecond = scan->packets[i].stamp.toSec();
    (void)fwrite(&pktDaySecond, sizeof(pktDaySecond), 1, pOutFile);
    (void)fwrite(&(scan->packets[i].data[0]), packet_size, 1, pOutFile);
  }
  fclose(pOutFile);


  return true;
}

double VelodyneDriver::CalcLidarRpm(const velodyne_msgs::VelodynePacket &startPkt, const velodyne_msgs::VelodynePacket &midPkt) {
  DLOG(INFO) << __FUNCTION__ << " start.";

  const uint16_t startAzimuthTimes100 = (startPkt.data[3] << 8) + startPkt.data[2];
  const uint16_t endAzimuthTimes100 = (midPkt.data[3] << 8) + midPkt.data[2];

  const double startTime = startPkt.stamp.toSec();
  const double endTime = midPkt.stamp.toSec();

  DLOG(INFO) << "endTime - startTime: " << endTime - startTime;
  DLOG(INFO) << "endAzimuthTimes100 - startAzimuthTimes100: " << endAzimuthTimes100 - startAzimuthTimes100;
  const double rounds = (endAzimuthTimes100 - startAzimuthTimes100 + 36000) % 36000 / 36000.;
  const double rpm = 60. * rounds / (endTime - startTime);

  DLOG(INFO) << "rpm: " << rpm;
  return rpm;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}

} // namespace velodyne_driver
