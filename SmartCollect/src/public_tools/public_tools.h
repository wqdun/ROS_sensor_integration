#ifndef PUBLIC_TOOLS_H
#define PUBLIC_TOOLS_H

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <visualization_msgs/Marker.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <dirent.h>

namespace public_tools
{

typedef struct {
    double x;
    double y;
    double z;
} pointXYZ_t;

typedef geometry_msgs::Point geoPoint_t;
typedef vector<geometry_msgs::Point> geoPoints_t;
typedef vector<geoPoints_t> geoLines_t;


class PublicTools {
public:

    static int string2int(const string& str);
    static double string2double(const string& str);
    static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP);
    static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, const geometry_msgs::Point &current_gauss, vector<geometry_msgs::Point> &points_transformed);
    static void generateFileName(const std::string &path, std::string &fileName);
    static void getFilesInDir(const std::string &baseDir, const std::string &keyWord, std::vector<std::string> &files);
};


}
// namespace public_tools

#endif