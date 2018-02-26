#ifndef DISPLAY_TRACK_H
#define DISPLAY_TRACK_H

#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "sc_center/centerMsg.h"

using std::string;
using std::vector;

class TrackDisplayer {

public:
    TrackDisplayer();
    void displayTrack(const geometry_msgs::Point &encryptedGauss);


private:
    ros::NodeHandle nh;
    ros::Publisher mPubTrack;

    visualization_msgs::Marker mLineStrip;
    visualization_msgs::Marker mArrow;

    vector<geometry_msgs::Point> mEncryptedGausses;
    double trackScaleRatio_;
};

#endif
