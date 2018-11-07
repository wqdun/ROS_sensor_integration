//
//  initial_aligment.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef initial_aligment_hpp
#define initial_aligment_hpp

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>
#include "imu_factor.h"
#include "utility.hpp"
#include <map>
#include "feature_manager.hpp"

using namespace Eigen;
using namespace std;

class ImageFrame
{
public:
    ImageFrame(){};
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t, cv::Mat _rot12):points{_points},t{_t},is_key_frame{false}
    {
        rot12 = _rot12.clone();
        //cout << "in class rot12" << rot12 << endl;

        if(!rot12.empty())
        {
            //cout << rot12.at<double>(0, 0) << "\t" << rot12.at<double>(0, 1) << endl;
            //cout << rot12.at<float>(0, 0) << "\t" << rot12.at<float>(0, 1) << endl;
            eigenrot12(0, 0) = rot12.at<double>(0, 0);
            eigenrot12(0, 1) = rot12.at<double>(0, 1);
            eigenrot12(0, 2) = rot12.at<double>(0, 2);
            eigenrot12(1, 0) = rot12.at<double>(1, 0);
            eigenrot12(1, 1) = rot12.at<double>(1, 1);
            eigenrot12(1, 2) = rot12.at<double>(1, 2);
            eigenrot12(2, 0) = rot12.at<double>(2, 0);
            eigenrot12(2, 1) = rot12.at<double>(2, 1);
            eigenrot12(2, 2) = rot12.at<double>(2, 2);
            //cout << "header:" << _t << "\t" << "in class rot12" << eigenrot12 << endl;
            is_rot12_valid = true;
        } else{
            is_rot12_valid = false;
        }

    };
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
    double t;
    Matrix3d R;  //q_v^i
    Vector3d T;
    cv::Mat rot12;
    Matrix3d eigenrot12;
    IntegrationBase *pre_integration;
    bool is_key_frame;
    bool is_rot12_valid;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
bool VisualIMUAlignmentOdometry(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x, Vector3d &tio, Matrix3d& ric);

#endif /* initial_aligment_hpp */
