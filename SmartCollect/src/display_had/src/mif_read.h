#ifndef MIF_READ_H
#define MIF_READ_H

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
using std::string;
#include <vector>
using std::vector;
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>
#include <dirent.h>
#include "../../public_tools/public_tools.h"

class MifReader {

public:
    MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~MifReader() {}
    void run();

private:
    void getFiles(const string& path, vector<string>& files);
    void readFile(const string &file);
    void getLonLat(const string &line, geometry_msgs::Point &point);

    ros::Publisher mif_pub;
    string mMifPath;
    visualization_msgs::Marker mLineStrip;
};






#endif