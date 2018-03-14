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
    isRecordLast_ = 0;
}

void TrackDisplayer::initMarker(visualization_msgs::Marker &marker, const size_t id) {
    marker.points.clear();
    marker.header.frame_id = "/velodyne";
    marker.ns = "recorded_track";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width; POINTS markers use x and y scale for width/height respectively
    marker.scale.x = 1.0 * trackScaleRatio_;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    // set .a = 0 to hide display
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(1.0);
}

void TrackDisplayer::displayTrack(const geometry_msgs::Point &encryptedGauss, const MyViz * const pMyViz) {
    const int _isRecordNow = (int)(pMyViz->clientCmdMsg_.is_record);
    DLOG(INFO) << __FUNCTION__ << " start, is_record: " << _isRecordNow;

    if(_isRecordNow) {
        // record -> record: add a point to last line
        if(isRecordLast_) {
            mRecordedAbsLines.back().push_back(encryptedGauss);
        }
        // not record -> record: add a line
        else {
            mRecordedAbsLines.push_back(vector<geometry_msgs::Point>( {encryptedGauss} ) );
        }

        // when points > 5000, do rarefaction: delete recordedLine[1], [3]...
        size_t recordedPointCnt = 0;
        for(auto &recordedLine: mRecordedAbsLines) {
            recordedPointCnt += recordedLine.size();
        }
        LOG_EVERY_N(INFO, 50) << "Record way points: " << recordedPointCnt;
        DLOG(INFO) << "Record line counts: " << mRecordedAbsLines.size();

        if(recordedPointCnt >= 10000) {
            LOG(INFO) << "do rarefaction: " << recordedPointCnt;
            for(auto &recordedLine: mRecordedAbsLines) {
                for(auto iter = recordedLine.begin(); iter != recordedLine.end();) {
                    ++iter;
                    if(recordedLine.end() == iter) {
                        break;
                    }
                    iter = recordedLine.erase(iter);
                }
            }
            LOG(INFO) << "do rarefaction end: " << recordedPointCnt;
        }
    }
    // whatever -> not record: do nothing
    // else {}
    isRecordLast_ = _isRecordNow;

    // display recorded tracks
    mRecordedLines.markers.clear();
    for(size_t markerId = 0; markerId < mRecordedAbsLines.size(); ++markerId) {
        visualization_msgs::Marker offsetLine;
        initMarker(offsetLine, markerId);
        public_tools::PublicTools::transform_coordinate(mRecordedAbsLines[markerId], encryptedGauss, offsetLine.points, trackScaleRatio_);
        mRecordedLines.markers.push_back(offsetLine);
    }
    mPubRecordLines.publish(mRecordedLines);

    // display recorded and unrecorded tracks
    mEncryptedGausses.push_back(encryptedGauss);

    const size_t point_cnt = mEncryptedGausses.size();
    LOG_EVERY_N(INFO, 50) << "Got way points: " << point_cnt;
    if(point_cnt >= 1000) {
        mEncryptedGausses.erase(mEncryptedGausses.begin(), mEncryptedGausses.begin() + 500);
    }

    public_tools::PublicTools::transform_coordinate(mEncryptedGausses, mEncryptedGausses.back(), mLineStrip.points, trackScaleRatio_);
    if(point_cnt >= 2) {
        mArrow.points.clear();
        mArrow.points.push_back(mLineStrip.points[point_cnt - 2]);
        // below mLineStrip.points.x/y/z is 0
        mArrow.points.push_back(mLineStrip.points[point_cnt - 1]);
        // pub mArrow
        mPubTrack.publish(mArrow);
    }

    // remove latest points, for there is arrow already
    mLineStrip.points.pop_back();
    // pub mLineStrip
    mPubTrack.publish(mLineStrip);
}
