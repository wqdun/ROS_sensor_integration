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
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
#include <dirent.h>
#include <bitset>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "../../public_tools/public_tools.h"
#include "../../public_tools/coordtrans.h"
#include "ntd_info_process/processed_infor_msg.h"
#include "MortonCodecTransJava.h"

typedef vector<geometry_msgs::Point> MifLine_t;
typedef vector<MifLine_t> MifLines_t;
static MifLines_t gAbsLines;

class MifReader {

public:
    MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~MifReader();
    void run();


private:
    void getFiles(const string& path, vector<string>& files);
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
};

static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, const geometry_msgs::Point &current_gauss, vector<geometry_msgs::Point> &points_transformed);

#endif