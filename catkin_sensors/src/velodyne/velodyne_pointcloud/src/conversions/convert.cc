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
        outMsg->points.clear();
        data_->unpack(scanMsg->packets[i], *outMsg);

        mTime2Pc[scanMsg->packets[i].stamp.toSec()] = *outMsg;
        ROS_INFO_STREAM("scanMsg->packets[" << i << "].stamp.toSec():" << std::fixed << scanMsg->packets[i].stamp.toSec());
        // TODO: notice its size
      }

    ROS_INFO_STREAM(mTime2Pc.size());

    if(1000 <= mTime2Pc.size()) {
      mIsSavedEnoughPackets = true;
    }

    // publish the accumulated cloud message
    // ROS_INFO_STREAM("Publishing " << outMsg->height * outMsg->width
                     // << " Velodyne points, time: " << outMsg->header.stamp);
    // output_.publish(outMsg);
  }

  void Convert::getStatusCB(const ntd_info_process::processed_infor_msg::ConstPtr& imuMsg) {
    mImuInfo.pitch = imuMsg->current_pitch;
    mImuInfo.roll = imuMsg->current_roll;
    mImuInfo.heading = imuMsg->current_heading;
    mImuInfo.currentWGS = imuMsg->current_wgs;

    double daySecond = fmod(imuMsg->GPStime, 86400); // GPStime % (3600 * 24)

    mTime2ImuInfo[daySecond] = mImuInfo;

    // TODO: Not too big, gotta erase some

    ROS_INFO_STREAM("imuMsg->Heading:" << mImuInfo.heading << " at " << daySecond << ":" << mImuInfo.pitch << ":" << mImuInfo.roll << ":" << mImuInfo.currentWGS.x);
  }

  imuInfo_t Convert::LookUpMap(const map<double, imuInfo_t> &map, const double x) {
    if(map.empty()) {
      ROS_WARN_STREAM("I got no GPS info.");
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
      ROS_INFO_STREAM("Convert::Run start." << int(mIsSavedEnoughPackets));
      if(!mIsSavedEnoughPackets) {
        continue;
      }
      mPcWGS.clear();

      velodyne_rawdata::VPointCloud::Ptr pWgsCloud4debug(new velodyne_rawdata::VPointCloud());
      mPcWGS.header.stamp = pWgsCloud4debug->header.stamp = mTime2Pc.cbegin()->second.header.stamp;
      mPcWGS.header.frame_id = pWgsCloud4debug->header.frame_id = mTime2Pc.cbegin()->second.header. frame_id;
      mPcWGS.height = pWgsCloud4debug->height = 1;
      // specify a lidar time, and find nearest gps time
      auto iter = mTime2Pc.begin();
      // for each packet
      for(; iter != mTime2Pc.end(); ++iter) {
        velodyne_rawdata::VPoint tempVPoint;
        // ROS_INFO_STREAM("mTime2ImuInfo.size():" << mTime2ImuInfo.size() << ":x:" << iter->first);
        imuInfo_t imuInfo = LookUpMap(mTime2ImuInfo, iter->first);
        // now iter->first is related to imuInfo, so we can convert point cloud
        // packet to WGS using imuInfo
        point_t angle_xyz = {
          deg2rad(imuInfo.pitch),
          deg2rad(imuInfo.roll),
          deg2rad(imuInfo.heading)
        };
        point_t offset = {
          imuInfo.currentWGS.x,
          imuInfo.currentWGS.y,
          imuInfo.currentWGS.z
        };
        // for each point in packet
        for(const auto point: iter->second.points) {
          point_t in_point = {
            point.x,
            point.y,
            point.z
          };

          tempVPoint = point;
          // TODO: converted pcloud saved to new pcloud(how to do converted data, we can
          // pick it using Time stamp)
          tf_rotate(in_point, angle_xyz, offset, tempVPoint);
          mPcWGS.points.push_back(tempVPoint);
          tempVPoint.x -= offset.x;
          tempVPoint.y -= offset.y;
          tempVPoint.z -= offset.z;
          pWgsCloud4debug->points.push_back(tempVPoint);
        }

      }
      // all packets converted, empty pointCloud
      mTime2Pc.clear();
      mIsSavedEnoughPackets = false;
      // pub converted pcloud
      m_output4calc.publish(mPcWGS);
      output_.publish(pWgsCloud4debug);
      // display for debug
      // modify history WGS coordinate: keep car(or IMU) position Origin
      // velodyne_rawdata::VPointCloud::Ptr pWGSCoordCloud(new velodyne_rawdata::VPointCloud());

      // // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      // // time of last frame
      // pWGSCoordCloud->header.stamp = outMsg->header.stamp;
      // pWGSCoordCloud->header.frame_id = outMsg->header.frame_id;
      // pWGSCoordCloud->height = 1;

      // velodyne_rawdata::VPoint imuCoord;
      // for(size_t i = 0; i < mWGSCloudArr.size(); ++i) {
      //   for(size_t j = 0; j < mWGSCloudArr[i].points.size(); ++j) {
      //     imuCoord.x = mWGSCloudArr[i].points[j].x - mCurrentWGS.x;
      //     imuCoord.y = mWGSCloudArr[i].points[j].y - mCurrentWGS.y;
      //     imuCoord.z = mWGSCloudArr[i].points[j].z - mCurrentWGS.z;

      //     imuCoord.ring = mWGSCloudArr[i].points[j].ring;
      //     imuCoord.intensity = mWGSCloudArr[i].points[j].intensity;
      //     pWGSCoordCloud->points.push_back(imuCoord);
      //     ++(pWGSCoordCloud->width);
      //   }
      // }

      // ROS_INFO_STREAM("Publishing " << pWGSCoordCloud->height * pWGSCoordCloud->width
      //                  << " Velodyne points, time: " << pWGSCoordCloud->header.stamp);
      // output_.publish(pWGSCoordCloud);
      // mWGSCloudArr.clear();
    }
  }

} // namespace velodyne_pointcloud
