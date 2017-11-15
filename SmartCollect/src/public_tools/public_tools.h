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

};


}
// namespace public_tools

#endif