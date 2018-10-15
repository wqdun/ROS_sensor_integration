//
// Created by root on 18-6-20.
//

#ifndef VINS_PC_MAINLOOP_ENCODER_DISPLACEMENT_FACTOR_H
#define VINS_PC_MAINLOOP_ENCODER_DISPLACEMENT_FACTOR_H

#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility.hpp"
#include "global_param.hpp"
#include "integration_base.h"
class EncoderDisplacementFactor : public ceres::SizedCostFunction<3, 7, 7, 9>{
public:
    EncoderDisplacementFactor(const Matrix3d &_Rio, const Vector3d &_pio, IntegrationBase* _pre_integration);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    Matrix3d Rio;
    Vector3d pio;
    //Vector3d delta_p_encoder;
    Eigen::Matrix3d sqrt_info;
    IntegrationBase* pre_integration;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif //VINS_PC_MAINLOOP_ENCODER_DISPLACEMENT_FACTOR_H
