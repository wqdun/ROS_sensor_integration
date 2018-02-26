#ifndef __DISPLAY_PLAN_LAYER_H
#define __DISPLAY_PLAN_LAYER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "../../sc_lib_public_tools/src/public_tools.h"

// GDAL library
#include <gdal.h>
#include <gdal_alg.h>
#include <cpl_conv.h>
#include <cpl_port.h>
#include <cpl_multiproc.h>
#include <ogr_srs_api.h>
#include <ogrsf_frmts.h>

class PlanLayerDisplayer {
public:
    PlanLayerDisplayer(const std::string &_planLayerPath);
    void displayPlanLayer(const public_tools::geoPoint_t &encryptedGauss);


private:
    ros::NodeHandle nh;
    ros::Publisher pubPlanLayer_;

    public_tools::geoLines_t planLayerLines_;
    visualization_msgs::MarkerArray planLayerLines2Show_;

    void getPlanLines(const std::string &planLayerFile);
    void initMarker(visualization_msgs::Marker &marker, const size_t id);
    double planLayerScaleRatio_;
};

#endif
