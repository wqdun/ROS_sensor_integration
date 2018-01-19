#include "display_track.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

TrackDisplayer::TrackDisplayer() {
    mEncryptedGausses.clear();
    // advertise of gps_lonlathei
    mPubTrack = nh.advertise<visualization_msgs::Marker>("gps_lonlathei", 0);

    // min capacity is 1000, to optimize efficiency
    mLineStrip.points.reserve(1000);
    // set mLineStrip.header mArrow.header
    mLineStrip.header.frame_id = mArrow.header.frame_id = "/velodyne";
    // set mLineStrip.header.stamp mArrow.header.stamp
    mLineStrip.header.stamp = mArrow.header.stamp = ros::Time::now();
    // set mLineStrip.ns mArrow.ns
    mLineStrip.ns = mArrow.ns = "points_and_lines";
    // set mLineStrip-mArrow action
    mLineStrip.action = mArrow.action = visualization_msgs::Marker::ADD;
    // set mLineStrip.pose
    mLineStrip.pose.orientation.w = 1.0;
    // mArrow.pose.orientation.w = 1.0;
    // set points-mLineStrip-mArrow id
    mLineStrip.id = 0;
    mArrow.id = 1;
    //set point-mLineStrip-mArrow type
    mLineStrip.type = visualization_msgs::Marker::LINE_STRIP;
    mArrow.type = visualization_msgs::Marker::ARROW;

    // POINTS markers use x and y scale for width/height respectively
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for line width
    mLineStrip.scale.x = 1.0;
    // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length
    mArrow.scale.x = 1.0;
    mArrow.scale.y = 1.6;
    mArrow.scale.z = 3.2;

    // Line strip is blue; set .a = 0 to hide display
    mLineStrip.color.r = 1.0f;
    mLineStrip.color.a = 1.0;
    // mArrow is red
    mArrow.color.g = 1.0f;
    mArrow.color.a = 1.0;
}

void TrackDisplayer::displayTrack(const geometry_msgs::Point &encryptedGauss) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    mEncryptedGausses.push_back(encryptedGauss);

    const size_t point_cnt = mEncryptedGausses.size();
    LOG_EVERY_N(INFO, 50) << "Got way points: " << point_cnt;
    if(point_cnt >= 1000) {
        mEncryptedGausses.erase(mEncryptedGausses.begin(), mEncryptedGausses.begin() + 500);
    }

    public_tools::PublicTools::transform_coordinate(mEncryptedGausses, mEncryptedGausses.back(), mLineStrip.points);
    if(point_cnt >= 2) {
        mArrow.points.clear();
        mArrow.points.push_back(mLineStrip.points[point_cnt - 2]);
        // below mLineStrip.points.x/y/z is 0
        mArrow.points.push_back(mLineStrip.points[point_cnt - 1]);
        // pub mArrow
        mPubTrack.publish(mArrow);
    }

    // remove latest points, for there is arrow already
    mLineStrip.points.pop_back();
    // pub mLineStrip
    mPubTrack.publish(mLineStrip);
}
