//
//  feature_tracker.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_tracker_hpp
#define feature_tracker_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
//#include "global_param.hpp"
#include <string>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#define F_THRESHOLD 1.0
#define EQUALIZE 1
extern "C"
{
using namespace cv;
using namespace std;
using namespace Eigen;
/*
    image frame
    --------> x:480
    |
    |
    |
    |
    |
    | y:640
 */
struct max_min_pts{
    Point2f min;
    Point2f max;
};

struct greaterThanPtr :
        public std::binary_function<const float *, const float *, bool>
{
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

class FeatureTracker
{
public:
    FeatureTracker(camodocal::CameraPtr m_camera,int col,int row,int max_cnt,int min_dis, double _fx, double _fy, double _cx, double _cy);
    void FindFeatures(const cv::Mat &_img, cv::Mat &result);
    void readImage(const cv::Mat &_img, cv::Mat &result, double _cur_time, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len,vector<int>& _track_cnt, cv::Mat& rot12);
    void setMask();
    bool rejectWithF(cv::Mat& rot12);
    void addPoints();
    bool updateID(unsigned int i);
    vector<cv::Point2f> undistortedPoints();
    void undistortedPoints2();
    void goodFeaturesToTrack2( double _maxVal, cv::Mat _image, vector<cv::Point2f> &_corners,
                               int maxCorners, double qualityLevel, double minDistance,
                               cv::Mat _mask, int blockSize=3, int gradientSize=3,
                               bool useHarrisDetector=false, double harrisK=0.04 );

    /*
     varialbles
    */
    int frame_cnt;
    cv::Mat mask;
    cv::Mat cur_img, pre_img, forw_img;

    vector<cv::Point2f> n_pts,cur_pts,pre_pts,forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;

    vector<int> ids,track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    vector<max_min_pts> parallax_cnt;
    static int n_id;
    int img_cnt;
    int col,row;
    int max_cnt;  // max count of points for LK
    int min_dis;  // minDistance Minimum possible Euclidean distance between the returned corners.

    double m_fx;
    double m_fy;
    double m_cx;
    double m_cy;


    /*
     interface
    */
    map<int, Eigen::Matrix<double, 7, 1>> image_msg;
    bool update_finished;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    bool inBorder(const cv::Point2f &pt);


};
}
#endif /* feature_tracker_hpp */
