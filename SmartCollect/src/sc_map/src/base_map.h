#ifndef __BASE_MAP_H__
#define __BASE_MAP_H__

#include <ros/ros.h>
#include <sc_msgs/Lines2D.h>
// GDAL library
#include <gdal.h>
#include <gdal_alg.h>
#include <cpl_conv.h>
#include <cpl_port.h>
#include <cpl_multiproc.h>
#include <ogr_srs_api.h>
#include <ogrsf_frmts.h>
// GDAL library END
#include "../../sc_lib_public_tools/src/public_tools.h"

class BaseMap {
public:
    BaseMap(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~BaseMap();
    void run();


private:
    void getLines(const std::string &_shpFile);

    sc_msgs::Lines2D baseMapLines_;
    ros::Publisher pubBaseMap_;
};

#endif

