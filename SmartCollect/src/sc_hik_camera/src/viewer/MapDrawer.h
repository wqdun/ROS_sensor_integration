//
// Created by root on 18-5-20.
//

#ifndef VINS_PC_MAPDRAWER_H
#define VINS_PC_MAPDRAWER_H


#include <pangolin/display/opengl_render_state.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <mutex>
#include <loop/keyframe_database.h>
#include <vector>
#include "global_param.hpp"

class MapDrawer {
public:
    MapDrawer(const std::string &strSettingPath);



    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawGrids();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawMarkerPoints();
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void SetAllFrames(std::vector<DRAWFRAME_DATA> frames_to_draw);
    void SetAllPoints(std::vector<Vector3d> points_3d);
    void SetMarkerPoints(VINS* pEstimator);

    float mScaleic;

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    vector<cv::Mat> cvTwcs;
    vector<cv::Mat> cvpoints_3d;
    vector<MarkerPoints3D> eimarkers_3d;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
    std::mutex mMutexPoints;
    std::mutex mMutexMarkers;
};


#endif //VINS_PC_MAPDRAWER_H
