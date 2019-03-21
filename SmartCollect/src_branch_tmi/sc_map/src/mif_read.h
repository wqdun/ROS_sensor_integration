#ifndef MIF_READ_H
#define MIF_READ_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sqlite3.h>
#include <string>
using std::string;
using std::stoi;
#include <vector>
using std::vector;
#include <dirent.h>
#include <bitset>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/coordtrans.h"
#include "sc_server_daemon/monitorMsg.h"
#include "MortonCodecTransJava.h"
#include "display_track.h"
#include "display_plan_layer.h"
#include <pwd.h>

static public_tools::geoLines_t gAbsLines;


class MyViz;
class MifReader {

public:
    MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~MifReader();
    void run(const MyViz * const pMyViz);


private:
    void readFile(const string &file);
    void gpsCallback(const sc_server_daemon::monitorMsg::ConstPtr& pGpsMsg);
    void initMarker(visualization_msgs::Marker &marker, const size_t id);

    ros::Subscriber mSubGps;
    ros::Publisher mPubArray;
    geometry_msgs::Point mCurrentWGS;
    string mMifPath;
    vector<int> mTileList;
    visualization_msgs::MarkerArray mLineArray;
    bool mIsInSameMesh;
    TrackDisplayer *mpTrackDisplayer;
    PlanLayerDisplayer *pPlanLayerDisplayer_;
    double mifScaleRatio_;
};

#endif