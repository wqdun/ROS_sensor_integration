#include "display_track.h"
#include "../../roscameragps_rviz/src/myviz.h"

// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

TrackDisplayer::TrackDisplayer() {
    trackScaleRatio_ = 0.01;
    mEncryptedGausses.clear();
    mRecordedAbsLines.clear();
    // advertise of gps_lonlathei
    mPubTrack = nh.advertise<visualization_msgs::Marker>("gps_lonlathei", 0);
    mPubRecordLines = nh.advertise<visualization_msgs::MarkerArray>("recorded_lines", 0);

    // min capacity is 1000, to optimize efficiency
    mLineStrip.points.reserve(1000);
    // set mLineStrip.header mArrow.header
    mLineStrip.header.frame_id = mArrow.header.frame_id = "/velodyne";
    // set mLineStrip.ns mArrow.ns
    mLineStrip.ns = mArrow.ns = "points_and_lines";
    // set mLineStrip-mArrow action
    mLineStrip.action = mArrow.action = visualization_msgs::Marker::ADD;
    // set mLineStrip.pose
    mLineStrip.pose.orientation.w = 1.0;
    // mArrow.pose.orientation.w = 1.0;
    // set points-mLineStrip-mArrow id
    mLineStrip.id = 0;
    mArrow.id = 1;

    //set point-mLineStrip-mArrow type
    mLineStrip.type = visualization_msgs::Marker::LINE_STRIP;
    mArrow.type = visualization_msgs::Marker::ARROW;

    // POINTS markers use x and y scale for width/height respectively
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for line width
    mLineStrip.scale.x = 1.0 * trackScaleRatio_;
    // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length
    mArrow.scale.x = 1.0 * trackScaleRatio_;
    mArrow.scale.y = 1.6 * trackScaleRatio_;
    mArrow.scale.z = 3.2 * trackScaleRatio_;

    mLineStrip.color.r = 1.0f;
    mLineStrip.color.g = 1.0f;
    mLineStrip.color.b = 0.0f;

    mArrow.color.r = 0.0f;
    mArrow.color.g = 1.0f;
    mArrow.color.b = 0.0f;

    // set .a = 0 to hide display
    mLineStrip.color.a = 1.0f;
    mArrow.color.a = 1.0f;

    mLineStrip.lifetime = mArrow.lifetime = ros::Duration(1.0);
    isRecordLast_ = false;
    geoLinesWithIsRecord.clear();
}

void TrackDisplayer::initMarker(visualization_msgs::Marker &marker, const size_t id, bool _isRecord) {
    marker.points.clear();
    marker.header.frame_id = "/velodyne";
    marker.ns = "recorded_track";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width; POINTS markers use x and y scale for width/height respectively
    if(_isRecord) {
        marker.scale.x = 1.0 * trackScaleRatio_;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    }
    else {
        marker.scale.x = 1.0 * trackScaleRatio_ * 0.6;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }

    // set .a = 0 to hide display
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(1.0);
}

void TrackDisplayer::displayTrack(const geometry_msgs::Point &encryptedGauss, const MyViz * const pMyViz) {
    const bool _isRecordNow = (bool)(pMyViz->clientCmdMsg_.is_record);
    DLOG(INFO) << __FUNCTION__ << " start, is_record: " << _isRecordNow;

    if(_isRecordNow == isRecordLast_) {
        // record -> record or no record -> no record
        if(geoLinesWithIsRecord.empty() ) {
            geoLinesWithIsRecord.emplace_back(_isRecordNow, vector<geometry_msgs::Point>({encryptedGauss}) );
        }
        else {
            geoLinesWithIsRecord.back().geoLine.push_back(encryptedGauss);
        }
    }
    else {
        // record -> no record or no record -> record
        geoLinesWithIsRecord.emplace_back(_isRecordNow, vector<geometry_msgs::Point>({encryptedGauss}) );
    }
    isRecordLast_ = _isRecordNow;

    size_t recordedPointCnt = 0;
    size_t unrecordedPointCnt = 0;
    // count points number in both recorded and unrecorded lines
    for(auto &_line: geoLinesWithIsRecord) {
        if(_line.isRecord) {
            recordedPointCnt += _line.geoLine.size();
        }
        else {
            unrecordedPointCnt += _line.geoLine.size();
        }
    }
    LOG_EVERY_N(INFO, 50) << "Recorded points count: " << recordedPointCnt;
    LOG_EVERY_N(INFO, 50) << "Unrecorded points count: " << unrecordedPointCnt;
    LOG_EVERY_N(INFO, 50) << "Line counts: " << geoLinesWithIsRecord.size();

    // when points > 10000, do rarefaction: delete recordedLine[1], [3]...
    if(recordedPointCnt >= 20000) {
        LOG(INFO) << "Recorded points rarefaction, recorded points count: " << recordedPointCnt;
        for(auto &_line: geoLinesWithIsRecord) {
            if(!(_line.isRecord) ) {
                continue;
            }
            for(auto iter = _line.geoLine.begin(); iter != _line.geoLine.end();) {
                ++iter;
                if(_line.geoLine.end() == iter) {
                    break;
                }
                iter = _line.geoLine.erase(iter);
            }
        }

        recordedPointCnt = 0;
        for(auto &_line: geoLinesWithIsRecord) {
            if(_line.isRecord) {
                recordedPointCnt += _line.geoLine.size();
            }
        }
        LOG(INFO) << "Rarefaction end, recorded points count: " << recordedPointCnt;
    }

    const int UNRECORDED_POINT_MAX = 2000;
    if(unrecordedPointCnt >= UNRECORDED_POINT_MAX) {
        LOG(INFO) << "Unrecorded points rarefaction, unrecorded points count: " << unrecordedPointCnt;

        unrecordedPointCnt = 0;
        for(auto iter = geoLinesWithIsRecord.begin(); iter != geoLinesWithIsRecord.end();) {
            if(iter->isRecord) {
                ++iter;
                continue;
            }
            unrecordedPointCnt += iter->geoLine.size();
            if(unrecordedPointCnt <= (UNRECORDED_POINT_MAX / 2) ) {
                iter = geoLinesWithIsRecord.erase(iter);
                if(geoLinesWithIsRecord.end() == iter) {
                    break;
                }
            }
            else {
                iter->geoLine.erase(iter->geoLine.begin(), iter->geoLine.begin() + (iter->geoLine.size() + (UNRECORDED_POINT_MAX / 2) - unrecordedPointCnt) );
                break;
            }
        }

        unrecordedPointCnt = 0;
        for(auto &_line: geoLinesWithIsRecord) {
            if(!(_line.isRecord) ) {
                unrecordedPointCnt += _line.geoLine.size();
            }
        }
        LOG(INFO) << "Rarefaction end, unrecorded points count: " << unrecordedPointCnt;
    }

    // display tracks
    mRecordedLines.markers.clear();
    for(size_t markerId = 0; markerId < geoLinesWithIsRecord.size(); ++markerId) {
        visualization_msgs::Marker offsetLine;
        initMarker(offsetLine, markerId, geoLinesWithIsRecord[markerId].isRecord);
        public_tools::PublicTools::transform_coordinate(geoLinesWithIsRecord[markerId].geoLine, encryptedGauss, offsetLine.points, trackScaleRatio_);
        mRecordedLines.markers.push_back(offsetLine);
    }
    mPubRecordLines.publish(mRecordedLines);

    const size_t lastLineSize = mRecordedLines.markers.back().points.size();
    if(lastLineSize >= 2) {
        mArrow.points.clear();
        mArrow.points.push_back(mRecordedLines.markers.back().points[lastLineSize - 2]);
        mArrow.points.push_back(mRecordedLines.markers.back().points.back() );
        // pub mArrow
        mPubTrack.publish(mArrow);
    }
}
