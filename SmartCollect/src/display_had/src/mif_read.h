#ifndef MIF_READ_H
#define MIF_READ_H

#include <ros/ros.h>
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

    string mMifPath;
};






#endif