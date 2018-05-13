#include "track.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

Track::Track() {
    LOG(INFO) << __FUNCTION__ << " start.";
    isRecordLast_ = false;
}

Track::~Track() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye.";
}

void Track::run(bool _isRecord, const sc_msgs::Point2D &_gpsPoint) {
    LOG(INFO) << __FUNCTION__ << " start, _isRecord: " << _isRecord;
    if(_gpsPoint.x <= 0.0001) {
        LOG(WARNING) << "Got a wrong lat: " << _gpsPoint.x;
        return;
    }

    size_t recordedPointNum = debugPointNum(recordedLines_);
    size_t unrecordedPointNum = debugPointNum(unrecordedLines_);

    (void)addPoint(_isRecord, _gpsPoint);

    const size_t _POINT_MAX = 200; // 10000
    if(recordedPointNum >= _POINT_MAX) {
        recordedLines_ = sparse(recordedLines_);
    }
    // else {nothing}
    if(unrecordedPointNum >= _POINT_MAX) {
        unrecordedLines_ = sparse(unrecordedLines_);
    }
    // else {nothing}

    DLOG(INFO) << "debugPointNum(unrecordedLines_): " << recordedPointNum << "; line size(): " << unrecordedLines_.lines2D.size();
    DLOG(INFO) << "debugPointNum(recordedLines_): " << unrecordedPointNum << "; line size(): " << recordedLines_.lines2D.size();

    LOG(INFO) << __FUNCTION__ << " end.";
}

size_t Track::debugPointNum(const sc_msgs::Lines2D &lines) {
    size_t pointNum = 0;
    for(auto &_line: lines.lines2D) {
        pointNum += _line.line2D.size();
    }
    return pointNum;
}

sc_msgs::Lines2D Track::sparse(const sc_msgs::Lines2D &lines2sparse) {
    LOG(INFO) << __FUNCTION__ << " start.";

    sc_msgs::Lines2D sparsedLines;
    for(auto &_line: lines2sparse.lines2D) {
        const size_t _pointNum = _line.line2D.size();
        sc_msgs::Line2D sparsedLine;
        LOG(INFO) << "Never sparse start and end point.";
        if(0 == (_pointNum % 2) ) {
            sparsedLine.line2D.push_back(_line.line2D.back() );
        }
        // delete [1] [3] [5]...
        for(size_t i = 0; i < _pointNum; ++i) {
            if(0 == (i % 2) ) {
                sparsedLine.line2D.push_back(_line.line2D[i]);
            }
        }
        sparsedLines.lines2D.push_back(sparsedLine);
    }

    LOG(INFO) << __FUNCTION__ << " end.";
    return sparsedLines;
}

void Track::addPoint(bool _isRecord, const sc_msgs::Point2D &_point) {
    LOG(INFO) << __FUNCTION__ << " start, _isRecord: " << _isRecord;

    if(_isRecord == isRecordLast_) {
        if(_isRecord) {
            if(unrecordedLines_.lines2D.empty() ) {
                (void)addLine(true, _point);
            }
            else {
                recordedLines_.lines2D.back().line2D.push_back(_point);
            }
        }
        else {
            if(unrecordedLines_.lines2D.empty() ) {
                (void)addLine(false, _point);
            }
            else {
                unrecordedLines_.lines2D.back().line2D.push_back(_point);
            }
        }
    }
    else {
        if(isRecordLast_) {
            recordedLines_.lines2D.back().line2D.push_back(_point);
        }
        else {
            unrecordedLines_.lines2D.back().line2D.push_back(_point);
        }
        (void)addLine(_isRecord, _point);
    }

    isRecordLast_ = _isRecord;
    LOG_EVERY_N(INFO, 50) << "unrecordedLines_ counts: " << unrecordedLines_.lines2D.size();
    LOG_EVERY_N(INFO, 50) << "recordedLines_ counts: " << recordedLines_.lines2D.size();
    LOG(INFO) << __FUNCTION__ << " end.";
}

void Track::addLine(bool _isRecord, const sc_msgs::Point2D &_point) {
    LOG(INFO) << __FUNCTION__ << " start, _isRecord: " << _isRecord;

    sc_msgs::Line2D line2d;
    line2d.line2D.push_back(_point);
    if(_isRecord) {
        recordedLines_.lines2D.push_back(line2d);
    }
    else {
        unrecordedLines_.lines2D.push_back(line2d);
    }
}


