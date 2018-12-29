#ifndef DISPLAY_TRACK_H
#define DISPLAY_TRACK_H

#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <visualization_msgs/MarkerArray.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "sc_center/centerMsg.h"

using std::string;
using std::vector;


class geoLineWithIsRecord_t {

public:
    geoLineWithIsRecord_t(bool _isRecord, vector<geometry_msgs::Point> _geoLine):
        isRecord(_isRecord),
        geoLine(_geoLine) {}

    ~geoLineWithIsRecord_t() {}

    bool isRecord;
    vector<geometry_msgs::Point> geoLine;
};

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

    vector<geoLineWithIsRecord_t> geoLinesWithIsRecord;

    vector<geometry_msgs::Point> mEncryptedGausses;

    double trackScaleRatio_;
    bool isRecordLast_;


    void initMarker(visualization_msgs::Marker &marker, const size_t id, bool _isRecord);
};

#endif
