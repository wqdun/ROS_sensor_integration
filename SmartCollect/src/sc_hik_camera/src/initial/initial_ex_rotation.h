//
// Created by diner on 17-6-24.
//

#ifndef VINS_INITIAL_EX_ROTATION_H
#define VINS_INITIAL_EX_ROTATION_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
#include "global_param.hpp"

using namespace Eigen;
using namespace std;

class InitialEXRotation
{
public:
    InitialEXRotation();
    bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
private:
    Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    int frame_count;

    vector< Matrix3d > Rc;
    vector< Matrix3d > Rimu;
    vector< Matrix3d > Rc_g;
    Matrix3d ric;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //VINS_INITIAL_EX_ROTATION_H
