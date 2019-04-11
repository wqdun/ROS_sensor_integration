 /* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include "rfans_driver/RfansCommand.h"
#include "bufferDecode.h"

#include <pcl_conversions/pcl_conversions.h>

// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

static const int RFANS_POINT_CLOUD_NUM = 1024 ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static sensor_msgs::PointCloud2 outCloud ;

static ros::Publisher  s_output;
static ros::Subscriber s_sub ;

pcl::PointCloud<pcl::PointXYZI> cloud;


static void RFansPacketReceived(rfans_driver::RfansPacket pkt) {
  int rtn = 0 ;
  rtn =  SSBufferDec::Depacket(pkt, outCloud,s_output) ;
  return ;
}

int main ( int argc , char ** argv )
{
  // Initialize the ROS system

  ros::init ( argc , argv , "calculation_node") ;
  ros::NodeHandle nh ;
  SSBufferDec::InitPointcloud2(outCloud) ;

  //node name
  std::string node_name = ros::this_node::getName();

  std::string lidarDataSavePath("/tmp/");
  ros::param::get(node_name + "/data_save_path", lidarDataSavePath);
  LOG(INFO) << "lidarDataSavePath: " << lidarDataSavePath;
  SSBufferDec::SetLidaDataSavePath(lidarDataSavePath);

  SSBufferDec::InitPointcloud2(outCloud) ;

  std::string advertise_name = "rfans_points";
  std::string advertise_path = node_name + "/advertise_name";
  ros::param::get(advertise_path,advertise_name);
  advertise_path = "rfans_driver/" + advertise_name;
  //ROS_INFO("%s : advertise name %s : %s",node_name.c_str(), advertise_name.c_str(), advertise_path.c_str() );

  //subscribe name
  std::string subscribe_name = "rfans_packets";
  std::string subscribe_path = node_name + "/subscribe_name";
  ros::param::get(subscribe_path, subscribe_name);
  subscribe_path = "rfans_driver/" + subscribe_name;
  //ROS_INFO("%s : subscribe name %s : %s",node_name.c_str(), subscribe_name.c_str(), subscribe_path.c_str() );

  //angle durantion
  float angle_duration = 60;
  std::string angle_duration_path = node_name + "/angle_duration";
  ros::param::get(angle_duration_path, angle_duration);
  SSBufferDec::SetAngleDuration(angle_duration);
  ROS_INFO("%s : angle_duration : %f",node_name.c_str(), angle_duration);

  s_sub= nh.subscribe (subscribe_path , RFANS_POINT_CLOUD_NUM, &RFansPacketReceived ) ;
  s_output = nh.advertise<sensor_msgs::PointCloud2>(advertise_path, RFANS_POINT_CLOUD_NUM);
  ros::spin () ;

  return  0;
}
