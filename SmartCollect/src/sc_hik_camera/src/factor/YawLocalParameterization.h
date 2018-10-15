//
// Created by root on 18-7-18.
//

#ifndef VINS_PC_YAWLOCALPARAMETERIZATION_H
#define VINS_PC_YAWLOCALPARAMETERIZATION_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "utility.hpp"

class YawLocalParameterization : public ceres::LocalParameterization {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 1; };
    virtual int LocalSize() const { return 1; };
};


#endif //VINS_PC_YAWLOCALPARAMETERIZATION_H
