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

#include "../../public_tools/public_tools.h"
#include "../../public_tools/coordtrans.h"
#include "ntd_info_process/processed_infor_msg.h"
#include "MortonCodecTransJava.h"
#include "display_track.h"
#include "display_plan_layer.h"

static public_tools::geoLines_t gAbsLines;

class MifReader {

public:
    MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh, const string& _imuRecordPath);
    ~MifReader();
    void run();


private:
    void readFile(const string &file);
    void gpsCallback(const ntd_info_process::processed_infor_msg::ConstPtr& pGpsMsg);
    void initMarker(visualization_msgs::Marker &marker, const size_t id);

    ros::Subscriber mSubGps;
    ros::Publisher mPubArray;
    geometry_msgs::Point mCurrentWGS;
    string mMifPath;
    vector<int> mTileList;
    visualization_msgs::MarkerArray mLineArray;
    bool mIsInSameMesh;
    std::ofstream mTrackMarsFile;
    TrackDisplayer *mpTrackDisplayer;
    PlanLayerDisplayer *pPlanLayerDisplayer_;
};

#endif