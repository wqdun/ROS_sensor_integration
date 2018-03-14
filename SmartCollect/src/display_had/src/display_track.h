#ifndef DISPLAY_TRACK_H
#define DISPLAY_TRACK_H

#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "sc_center/centerMsg.h"

using std::string;
using std::vector;


class MyViz;
class TrackDisplayer {

public:
    TrackDisplayer();
    void displayTrack(const geometry_msgs::Point &encryptedGauss, const MyViz * const pMyViz);


private:
    ros::NodeHandle nh;
    ros::Publisher mPubTrack;
    ros::Publisher mPubRecordLines;

    visualization_msgs::Marker mLineStrip;
    visualization_msgs::Marker mArrow;
    visualization_msgs::MarkerArray mRecordedLines;

    public_tools::geoLines_t mRecordedAbsLines;

    vector<geometry_msgs::Point> mEncryptedGausses;

    double trackScaleRatio_;
    int isRecordLast_;


    void initMarker(visualization_msgs::Marker &marker, const size_t id);
};

#endif
