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
  // using velodyne_rawdata::VPoint;
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData()),
    mIsSavedEnoughPackets(false)
  {
    data_->setup(private_nh);

    gps_sub_ = node.subscribe("processed_infor_msg", 1000, &Convert::getStatusCB, this);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    m_output4calc = node.advertise<sensor_msgs::PointCloud2>("velodyne_points_for_calc", 10);


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
    if(mIsSavedEnoughPackets) {
      // TODO: put it to a buffer?
      return;
    }

    if (output_.getNumSubscribers() == 0 && 0 == m_output4calc.getNumSubscribers())         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_rawdata::VPointCloud::Ptr
      pTempPktPc(new velodyne_rawdata::VPointCloud());
    // pTempPktPc's header is a pcl::PCLHeader, convert it before stamp assignment
    // actually each packet has a stamp, here use last packet stamp, refer VelodyneDriver::poll
    pTempPktPc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    pTempPktPc->header.frame_id = scanMsg->header.frame_id;
    pTempPktPc->height = 1;

    // process each packet provided by the driver; generally 75 packets per scan
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        pTempPktPc->points.clear();
        data_->unpack(scanMsg->packets[i], *pTempPktPc);
        // now packet point cloud is in lidar coord, unit: m
        // get lidar day second by ros time
        const double rosTimeInS = ros::Time::now().toSec();
        const double pktTimeInS = scanMsg->packets[i].stamp.toSec();
        const double pktDaySecond = getDaySecond(rosTimeInS, pktTimeInS);

        // TODO: consider using pointer map to raise efficiency
        // mTime2Pc[getDaySecond(rosTimeInS, pktTimeInS)] = *pTempPktPc;
        mTime2Pc.insert(std::make_pair(getDaySecond(rosTimeInS, pktTimeInS), *pTempPktPc));

        // ROS_INFO_STREAM("Receive scanMsg->packets[" << i << "] with stamp: " << std::fixed << scanMsg->packets[i].stamp.toSec() << ":daySecond:" << pktDaySecond);
      }

    // output_.publish(pTempPktPc);

    if(60 <= mTime2Pc.size()) {
      // when mTime2Pc contains 1000 packets
      mIsSavedEnoughPackets = true;
    }
    // TODO: below for debug, which can be commented.
    size_t pointCntInTime2Pc = 0;
    for(const auto& time2Pc: mTime2Pc) {
      pointCntInTime2Pc += time2Pc.second.points.size();
    }
    ROS_INFO_STREAM("This scan contains packets: " << scanMsg->packets.size() << ", points: " << pTempPktPc->height * pTempPktPc->width);
    ROS_INFO_STREAM("Now mTime2Pc contains packets: " << mTime2Pc.size() << ", points: " << pointCntInTime2Pc);
  }

  void Convert::getStatusCB(const ntd_info_process::processed_infor_msg::ConstPtr& imuMsg) {
    imuInfo_t tempImuInfo;

    tempImuInfo.pitch = imuMsg->current_pitch;
    tempImuInfo.roll = imuMsg->current_roll;
    tempImuInfo.heading = imuMsg->current_heading;
    tempImuInfo.currentWGS = imuMsg->current_wgs;

    const double daySecond = fmod(imuMsg->GPStime, 86400); // GPStime % (3600 * 24)

    mTime2ImuInfo[daySecond] = tempImuInfo;
    // TODO: Not too big, gotta erase some

    ROS_INFO_STREAM("imuMsg->Heading:" << tempImuInfo.heading << " at " << daySecond << ":" << tempImuInfo.pitch << ":" << tempImuInfo.roll << ":" << tempImuInfo.currentWGS.x);
  }

  imuInfo_t Convert::LookUpMap(const map<double, imuInfo_t> &map, const double x) {
    if(map.empty()) {
      // ROS_WARN_STREAM("I got no GPS info.");
      imuInfo_t zeroImuInfo;
      zeroImuInfo.pitch = zeroImuInfo.roll = zeroImuInfo.heading = 0;
      zeroImuInfo.currentWGS.x = zeroImuInfo.currentWGS.y = zeroImuInfo.currentWGS.z = 0;
      return zeroImuInfo;
    }
    const auto little_iter = map.cbegin();
    const auto big_iter = map.crbegin();

    if(x <= little_iter->first) {
      ROS_INFO_STREAM(x << " is too small.");
      return little_iter->second;
  }
    if(x >= big_iter->first) {
      ROS_INFO_STREAM(x << " is too big.");
      return big_iter->second;
    }

    const auto next = map.lower_bound(x);
    // to float type, 2.0000000000000000000001 might be equal to 2
    // in this case, --temp may fail
    if(next == little_iter) {
      ROS_INFO_STREAM(x << " is the small endpoint.");
      return little_iter->second;
    }

    auto temp = next;
    const auto previous = (--temp);

    // calculate correspond imuInfo
    imuInfo_t tempImuInfo;
    tempImuInfo.pitch = (next->second.pitch - previous->second.pitch) / (next->first - previous->first) * (x - previous->first) + previous->second.pitch;
    tempImuInfo.roll = (next->second.roll - previous->second.roll) / (next->first - previous->first) * (x - previous->first) + previous->second.roll;
    tempImuInfo.heading = (next->second.heading - previous->second.heading) / (next->first - previous->first) * (x - previous->first) + previous->second.heading;

    tempImuInfo.currentWGS.x = (next->second.currentWGS.x - previous->second.currentWGS.x) / (next->first - previous->first) * (x - previous->first) + previous->second.currentWGS.x;
    tempImuInfo.currentWGS.y = (next->second.currentWGS.y - previous->second.currentWGS.y) / (next->first - previous->first) * (x - previous->first) + previous->second.currentWGS.y;
    tempImuInfo.currentWGS.z = (next->second.currentWGS.z - previous->second.currentWGS.z) / (next->first - previous->first) * (x - previous->first) + previous->second.currentWGS.z;

    return tempImuInfo;
  }

  void Convert::Run() {
    // Since I've got gps map and lidar map, interpolate gps position and convert
    // lidar data to WGS using interpolated gps.
    ros::Rate loop_rate(200);
    while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();

      TfCoord();
    }
  }

void Convert::TfCoord() {
  // ROS_INFO_STREAM("Convert::TfCoord start.");
  if(!mIsSavedEnoughPackets) {
    return;
  }

  velodyne_rawdata::VPointCloud::Ptr pWgsCloud(new velodyne_rawdata::VPointCloud());
  velodyne_rawdata::VPointCloud::Ptr pWgsCloud4debug(new velodyne_rawdata::VPointCloud());
  pWgsCloud->header.stamp = mTime2Pc.cbegin()->second.header.stamp;

  velodyne_msgs::VelodyneScanPtr pScan(new velodyne_msgs::VelodyneScan);
  pScan->header.stamp = ros::Time::now();
  pWgsCloud4debug->header.stamp = pcl_conversions::toPCL(pScan->header).stamp;

  pWgsCloud->header.frame_id = pWgsCloud4debug->header.frame_id = mTime2Pc.cbegin()->second.header. frame_id;
  pWgsCloud->height = pWgsCloud4debug->height = 1;

  const double _1stTimePc = mTime2Pc.cbegin()->first;
  ROS_INFO_STREAM("PC time 1:" << _1stTimePc << "; time end:" << mTime2Pc.crbegin()->first);

  if(!mTime2ImuInfo.empty()) {
    ROS_INFO_STREAM("IMU time 1:" << mTime2ImuInfo.cbegin()->first << "; time end:" << mTime2ImuInfo.crbegin()->first);
    // remove some outdated data in mTime2ImuInfo
    removeOutdatedImuInfo(_1stTimePc);
  }
  else {
    ROS_WARN_STREAM("mTime2ImuInfo is empty.");
  }

  // for each packet
  for(const auto& time2Pc: mTime2Pc) {
    const imuInfo_t imuInfo = LookUpMap(mTime2ImuInfo, time2Pc.first);
    // TODO: see if could find PC time in IMU

    // now time2Pc.first is related to imuInfo, so we can convert point cloud
    // packet to WGS using imuInfo
    const point_t angleXyzImu2Wgs = {
      deg2rad(imuInfo.pitch),
      deg2rad(imuInfo.roll),
      deg2rad(imuInfo.heading)
    };
    const point_t offsetXyzImu2Wgs = {
      imuInfo.currentWGS.x,
      imuInfo.currentWGS.y,
      imuInfo.currentWGS.z
    };
    // for each point in packet
    for(const auto& point: time2Pc.second.points) {
      // 1st convert: lidar to imu coord
      static velodyne_rawdata::VPoint imuPoint;
      static const point_t angleXyzLidar2Imu = {deg2rad(20), 0, 0};
      static const point_t offsetXyzLidar2Imu = {0, 0, 0.3187};
      tf_rotate(point, angleXyzLidar2Imu, offsetXyzLidar2Imu, imuPoint);

      // 2nd convert: imu to WGS coord
      static velodyne_rawdata::VPoint wgsPoint;
      if(!mTime2ImuInfo.empty()) {
        tf_rotate(imuPoint, angleXyzImu2Wgs, offsetXyzImu2Wgs, wgsPoint);
      }
      else {
        wgsPoint = imuPoint;
      }
      wgsPoint.ring = point.ring;
      wgsPoint.intensity = point.intensity;

      pWgsCloud->points.push_back(wgsPoint);

      // TODO: below for debug
      wgsPoint.x -= offsetXyzImu2Wgs.x;
      wgsPoint.y -= offsetXyzImu2Wgs.y;
      wgsPoint.z -= offsetXyzImu2Wgs.z;
      // wgsPoint.x /= 10;
      // wgsPoint.y /= 10;
      // wgsPoint.z /= 10;

      pWgsCloud4debug->points.push_back(wgsPoint);
    }
  }

  // all packets converted, empty pointCloud
  mTime2Pc.clear();
  mIsSavedEnoughPackets = false;
  // pub converted pcloud
  m_output4calc.publish(pWgsCloud);
  ROS_INFO_STREAM(pWgsCloud->points.size());

  ROS_INFO_STREAM(pWgsCloud4debug->size() << "pWgsCloud4debug->header.frame_id:" << pWgsCloud4debug->header.frame_id);
  ROS_INFO_STREAM("pWgsCloud4debug->points:" << pWgsCloud4debug->points[2000].x << ":" << pWgsCloud4debug->points[2000].y << ":" << pWgsCloud4debug->points[2000].z);

  output_.publish(pWgsCloud4debug);
}

void Convert::removeOutdatedImuInfo(const double beginTime) {
  auto beginIter = mTime2ImuInfo.lower_bound(beginTime);
  if(beginIter == mTime2ImuInfo.cbegin()) {
    ROS_INFO_STREAM(beginTime << " <= 1st imu time:" << mTime2ImuInfo.cbegin()->first);
    return;
  }
  --beginIter;
  ROS_INFO_STREAM("Origin mTime2ImuInfo size is: " << mTime2ImuInfo.size());
  mTime2ImuInfo.erase(mTime2ImuInfo.cbegin(), beginIter);
  ROS_INFO_STREAM("Now mTime2ImuInfo size is: " << mTime2ImuInfo.size());

  return;
}

static void multiply_matrix(const velodyne_rawdata::VPoint &in_xyz, const double tf_matrix[][3], velodyne_rawdata::VPoint &out_xyz) {
    out_xyz.x = tf_matrix[0][0] * in_xyz.x + tf_matrix[0][1] * in_xyz.y + tf_matrix[0][2] * in_xyz.z;
    out_xyz.y = tf_matrix[1][0] * in_xyz.x + tf_matrix[1][1] * in_xyz.y + tf_matrix[1][2] * in_xyz.z;
    out_xyz.z = tf_matrix[2][0] * in_xyz.x + tf_matrix[2][1] * in_xyz.y + tf_matrix[2][2] * in_xyz.z;
  }
static void tf_rotate(const velodyne_rawdata::VPoint &in_xyz, const point_t &angle_xyz, const point_t &offset, velodyne_rawdata::VPoint &out_xyz) {
    double Ax[3][3], Ay[3][3], Az[3][3];
    Ax[0][0] = 1; Ax[0][1] = 0; Ax[0][2] = 0;
    Ax[1][0] = 0; Ax[1][1] = cos(angle_xyz.x); Ax[1][2] = sin(angle_xyz.x);
    Ax[2][0] = 0; Ax[2][1] = -sin(angle_xyz.x); Ax[2][2] = cos(angle_xyz.x);
    Ay[0][0] = cos(angle_xyz.y); Ay[0][1] = 0; Ay[0][2] = -sin(angle_xyz.y);
    Ay[1][0] = 0; Ay[1][1] = 1; Ay[1][2] = 0;
    Ay[2][0] = sin(angle_xyz.y); Ay[2][1] = 0; Ay[2][2] = cos(angle_xyz.y);
    Az[0][0] = cos(angle_xyz.z); Az[0][1] = sin(angle_xyz.z); Az[0][2] = 0;
    Az[1][0] = -sin(angle_xyz.z); Az[1][1] = cos(angle_xyz.z); Az[1][2] = 0;
    Az[2][0] = 0; Az[2][1] = 0; Az[2][2] = 1;
    velodyne_rawdata::VPoint out1_xyz, out2_xyz, out3_xyz;
    multiply_matrix(in_xyz, Ax, out1_xyz);
    multiply_matrix(out1_xyz, Ay, out2_xyz);
    multiply_matrix(out2_xyz, Az, out3_xyz);
    out_xyz.x = offset.x + out3_xyz.x;
    out_xyz.y = offset.y + out3_xyz.y;
    out_xyz.z = offset.z + out3_xyz.z;
}

// degree to radian
static double deg2rad(const double deg) {
  return deg * M_PI / 180;
}

static double getDaySecond(const double rosTime, const double pktTime) {
  int rosHour = (int)rosTime / 3600 % 24;
  const int rosMinute = (int)rosTime / 60 % 60;
  const int pktMinute = (int)pktTime / 60;
  const int errMinute = rosMinute - pktMinute;
  if(errMinute > 10) {
    ++rosHour;
  }
  else
  if(errMinute < -10) {
    --rosHour;
  }
  // else {
  //   // do nothing when errMinute in [-10, 10]
  // }

  // in case: -1 || 24
  rosHour = (rosHour + 24) % 24;

  return pktTime + 3600 * rosHour;
}

} // namespace velodyne_pointcloud
