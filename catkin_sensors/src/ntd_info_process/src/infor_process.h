#ifndef __INFOR_PROCESS_H
#define __INFOR_PROCESS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
// #include <vector>

#include "imupac/imu5651.h"
#include "ntd_info_process/processed_infor_msg.h"

using std::vector;
using std::string;

static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP);
static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, vector<geometry_msgs::Point> &points_transformed);
static double string2num(const string& str);

class InforProcess {
public:
    InforProcess();
    ~InforProcess();


private:
    void gpsCB(const imupac::imu5651::ConstPtr& pGPSmsg);

    ros::NodeHandle nh;
    ros::Subscriber mSub;
    ros::Publisher mPub;
};


#endif // __INFOR_PROCESS_H