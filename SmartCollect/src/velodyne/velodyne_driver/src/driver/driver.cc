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
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));

  private_nh.param("record_path", mRecordFile, mRecordFile);
  // get unix time stamp as file name
  time_t tt = time(NULL);
  tm *t= localtime(&tt);
  char fileName[50];
  (void)sprintf(fileName, "%02d_%02d_%02d.lidar", t->tm_hour, t->tm_min, t->tm_sec);
  mRecordFile += fileName;
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

  private_nh.param("rpm", config_.rpm, 600.0);
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
  mPubPpsStatus = node.advertise<std_msgs::String>("velodyne_pps_status", 0);
}

static void parsePositionPkt(const char *pkt, std::string &parsedRes) {
    ROS_DEBUG_STREAM(__FUNCTION__ << " start.");
    if('$' != pkt[206]) {
      ROS_INFO_STREAM("No $GPRMC received: " << pkt[206]);
      // 4: "No position packet"; V-invalid, refer VLP-16 manual
      parsedRes = "4,V";
      return;
    }

    // 00 .. 03
    parsedRes = std::to_string(pkt[202]);

    // $GPRMC,,V,,,,,,,,,,N*53
    // A validity - A-ok, V-invalid, refer VLP-16 manual
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
    parsedRes += ("," + isGprmcValid.str() );
    return;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  std_msgs::String ppsStatus;
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
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = scan->packets[config_.npackets - 1].stamp;
  scan->header.frame_id = config_.frame_id;

  FILE *pOutFile;
  if(!(pOutFile = fopen(mRecordFile.c_str(), "ab") ) ) {
    ROS_ERROR_STREAM("Create file:" << mRecordFile << " failed, errno:" << errno);
    exit(1);
  }
  ROS_DEBUG_STREAM("Create file:" << mRecordFile << " successfully.");

  for(size_t i = 0; i < scan->packets.size(); ++i) {
    const double pktDaySecond = getDaySecond(ros::Time::now().toSec(), scan->packets[i].stamp.toSec());

    (void)fwrite(&pktDaySecond, sizeof(pktDaySecond), 1, pOutFile);
    (void)fwrite(&(scan->packets[i].data[0]), packet_size, 1, pOutFile);
  }

  fclose(pOutFile);

  // only last position pkt be published, ignore the rest
  (void)parsePositionPkt(positionPkt, ppsStatus.data);
  mPubPpsStatus.publish(ppsStatus);

  output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}


static double getDaySecond(const double rosTime, const double pktTime) {
  // Ros time is UTC time(+0), not local time(Beijing: +8)
  int rosHour = (int)rosTime / 3600 % 24;
  const int rosMinute = (int)rosTime / 60 % 60;
  const int pktMinute = (int)pktTime / 60;
  const int errMinute = rosMinute - pktMinute;
  if(errMinute > 20) {
    ++rosHour;
  }
  else
  if(errMinute < -20) {
    --rosHour;
  }
  // else {
  //   // do nothing when errMinute in [-10, 10]
  // }

  // in case: -1 || 24
  rosHour = (rosHour + 24) % 24;

  return pktTime + 3600 * rosHour;
}

} // namespace velodyne_driver
