#ifndef __TRACK_H__
#define __TRACK_H__
#include "sc_msgs/Lines2D.h"

class Track {
public:
    Track();
    ~Track();
    void run(bool _isRecord, const sc_msgs::Point2D &_gpsPoint);

    sc_msgs::Lines2D recordedLines_;
    sc_msgs::Lines2D unrecordedLines_;


private:
    void addPoint(bool _isRecord, const sc_msgs::Point2D &_point);
    void addLine(bool _isRecord, const sc_msgs::Point2D &_point);
    sc_msgs::Lines2D sparse(const sc_msgs::Lines2D &lines2sparse);
    size_t debugPointNum(const sc_msgs::Lines2D &lines);

    bool isRecordLast_;
};

#endif

