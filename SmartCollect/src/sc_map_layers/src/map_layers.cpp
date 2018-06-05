#include "map_layers.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

MapLayers::MapLayers(ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string &_projects) {
    LOG(INFO) << __FUNCTION__ << " start.";

    pubLayer_ = nh.advertise<sc_msgs::Lines2D>("sc_recorded_layers", 0);

    std::vector<std::string> projectArr;
    (void)boost::split(projectArr, _projects, boost::is_any_of(",") );
    LOG(INFO) << "I got " << projectArr.size() << " projects layers to show.";

    (void)getLinesFromLayers(projectArr);
}

MapLayers::~MapLayers() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye, MapLayers.";
}

void MapLayers::getLinesFromLayers(const std::vector<std::string> &_projectArr) {
    LOG(INFO) << __FUNCTION__ << " start.";

    recordedLines_.lines2D.clear();
    for(auto &project: _projectArr) {
        const std::string projectDir("/opt/smartc/record/" + project + "/Rawdata/IMU/");

        std::vector<std::string> recordedLayerFiles;
        (void)public_tools::PublicTools::getFilesInDir(projectDir, "_recorded_layer.txt", recordedLayerFiles);
        if(1 != recordedLayerFiles.size() ) {
            LOG(WARNING) << "Got " << recordedLayerFiles.size() << " recordedLayerFiles in " << projectDir << ", should be 1.";
        }
        else {
            (void)getLines(recordedLayerFiles[0], recordedLines_);
        }
    }
}

void MapLayers::addPoint2Line(const std::string &_line, sc_msgs::Line2D &_line2d) {
    LOG(INFO) << __FUNCTION__ << " start.";

    if(_line.empty() ) {
        LOG(WARNING) << "Empty line.";
        return;
    }

    std::vector<std::string> lngLat;
    boost::split(lngLat, _line, boost::is_any_of(",") );
    if(2 != lngLat.size() ) {
        LOG(ERROR) << "Got a wrong line: " << _line;
        exit(1);
    }

    sc_msgs::Point2D lngLat2d;
    lngLat2d.x = public_tools::PublicTools::string2num(lngLat[0], double(-1) );
    lngLat2d.y = public_tools::PublicTools::string2num(lngLat[1], double(-1) );
    _line2d.line2D.push_back(lngLat2d);
    return;
}

void MapLayers::addLine2Lines(const sc_msgs::Line2D &_line2d, sc_msgs::Lines2D &recordedLines) {
    LOG(INFO) << __FUNCTION__ << " start.";

    if(_line2d.line2D.empty() ) {
        LOG(WARNING) << "Empty recordedLine.";
        return;
    }

    recordedLines.lines2D.push_back(_line2d);
    return;
}

void MapLayers::getLines(const std::string &layerFile, sc_msgs::Lines2D &_lines) {
    LOG(INFO) << __FUNCTION__ << " start, reading " << layerFile;
    int lineNumLast = recordedLines_.lines2D.size();

    std::fstream fin(layerFile);
    std::string lineNoCR("");
    sc_msgs::Line2D recordedLine;
    recordedLine.line2D.clear();
    while(getline(fin, lineNoCR) ) {
        if(lineNoCR.empty() ) {
            (void)addLine2Lines(recordedLine, recordedLines_);
            recordedLine.line2D.clear();
            continue;
        }

        (void)addPoint2Line(lineNoCR, recordedLine);
    }
    (void)addLine2Lines(recordedLine, recordedLines_);

    int lineNumNow = recordedLines_.lines2D.size();

    LOG(INFO) << "Got " << lineNumNow << " - " << lineNumLast << " = " << lineNumNow - lineNumLast << " lines in " << layerFile;
}

void MapLayers::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    ros::Rate rate(0.5);
    while(ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        pubLayer_.publish(recordedLines_);
    }
}

