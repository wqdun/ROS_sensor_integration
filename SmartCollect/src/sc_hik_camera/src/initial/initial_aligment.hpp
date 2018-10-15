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
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):points{_points},t{_t},is_key_frame{false}
    {
    };
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
    double t;
    Matrix3d R;  //q_v^i
    Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);

#endif /* initial_aligment_hpp */
