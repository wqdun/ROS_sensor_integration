#ifndef __BASE_MAP_H__
#define __BASE_MAP_H__

#include <ros/ros.h>
#include "sc_msgs/Lines2D.h"
#include "sc_msgs/MonitorMsg.h"
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
#include "../../sc_lib_public_tools/src/coordtrans.h"
#include "track.h"

class BaseMap {
public:
    BaseMap(ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string &rawdataDir);
    ~BaseMap();
    void run();


private:
    void getLines(const std::string &_shpFile, sc_msgs::Lines2D &_lines);
    void monitorCB(const sc_msgs::MonitorMsg::ConstPtr& pMonitorMsg);

    ros::Subscriber subMonitor_;
    ros::Publisher pubBaseMap_;
    ros::Publisher pubPlanLayer_;
    ros::Publisher pubRecordedLayer_;
    ros::Publisher pubUnrecordedTrack_;
    ros::Publisher pubRecordedTrack_;

    sc_msgs::Lines2D baseMapLines_;
    sc_msgs::Lines2D planLayerLines_;
    sc_msgs::Lines2D recordedLayerLines_;

    boost::shared_ptr<Track> pTracker_;
    bool isRecord_;
    sc_msgs::Point2D gpsLonLat_;
};

#endif

