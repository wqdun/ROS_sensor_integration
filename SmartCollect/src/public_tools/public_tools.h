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
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

namespace public_tools
{

typedef struct {
    double x;
    double y;
    double z;
} pointXYZ_t;

class PublicTools {
public:

    static int string2int(const string& str);
    static double string2double(const string& str);
    static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP);
    static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, const geometry_msgs::Point &current_gauss, vector<geometry_msgs::Point> &points_transformed);
};


}
// namespace public_tools

#endif