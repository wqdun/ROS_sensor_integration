#ifndef __TRACK_H__
#define __TRACK_H__

#include <fstream>
#include "sc_msgs/Lines2D.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

class Track {
public:
    Track(const std::string &_rawdataDir);
    ~Track();
    void run(bool _isRecord, const sc_msgs::Point2D &_gpsPoint);

    sc_msgs::Lines2D recordedLines_;
    sc_msgs::Lines2D unrecordedLines_;


private:
    void addPoint(bool _isRecord, const sc_msgs::Point2D &_point);
    void addLine(bool _isRecord, const sc_msgs::Point2D &_point);
    sc_msgs::Lines2D sparse(const sc_msgs::Lines2D &lines2sparse);
    size_t debugPointNum(const sc_msgs::Lines2D &lines);

    std::string layerFile_;;
    bool isRecordLast_;
};

#endif

