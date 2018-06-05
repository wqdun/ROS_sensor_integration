#ifndef __MAP_LAYERS_H__
#define __MAP_LAYERS_H__

#include <ros/ros.h>
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "sc_msgs/Lines2D.h"

class MapLayers {
public:
    MapLayers(ros::NodeHandle node, ros::NodeHandle private_nh, const std::string &_projects);
    ~MapLayers();
    void run();


private:
    ros::Publisher pubLayer_;
    sc_msgs::Lines2D recordedLines_;

    void getLinesFromLayers(const std::vector<std::string> &_projectArr);
    void addPoint2Line(const std::string &_line, sc_msgs::Line2D &_line2d);
    void addLine2Lines(const sc_msgs::Line2D &_line2d, sc_msgs::Lines2D &recordedLines);
    void getLines(const std::string &layerFile, sc_msgs::Lines2D &_lines);
};

#endif
