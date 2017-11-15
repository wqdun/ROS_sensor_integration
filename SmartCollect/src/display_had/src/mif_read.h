#ifndef MIF_READ_H
#define MIF_READ_H

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sqlite3.h>
#include <string>
using std::string;
#include <vector>
using std::vector;
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>
#include <dirent.h>
#include "../../public_tools/public_tools.h"
#include "ntd_info_process/processed_infor_msg.h"

class MifReader {

public:
    MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~MifReader() {}
    void run();

private:
    void getFiles(const string& path, vector<string>& files);
    void readFile(const string &file);
    void getLonLat(const string &line, geometry_msgs::Point &point);
    void gpsCallback(const ntd_info_process::processed_infor_msg::ConstPtr& pGpsMsg);

    ros::Subscriber mSubGps;
    ros::Publisher mif_pub;
    geometry_msgs::Point mNowLocation;
    string mMifPath;
    visualization_msgs::Marker mLineStrip;
};






#endif