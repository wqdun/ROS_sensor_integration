#ifndef __TO_MARS_H__
#define __TO_MARS_H__

// GDAL library
#include <gdal.h>
#include <gdal_alg.h>
#include <cpl_conv.h>
#include <cpl_port.h>
#include <cpl_multiproc.h>
#include <ogr_srs_api.h>
#include <ogrsf_frmts.h>
// GDAL library END

class ToMars {
public:
    ToMars(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~ToMars();
    void convert();


private:
    void getLines(const std::string &_shpFile, sc_msgs::Lines2D &_lines);
    void monitorCB(const sc_msgs::MonitorMsg::ConstPtr& pMonitorMsg);

    std::vector<std::vector<Point2D>> lines_;
};

#endif

