//
// Created by root on 12/13/17.
//

#ifndef AI2_SPEEDPRIOR_H
#define AI2_SPEEDPRIOR_H

#endif //AI2_SPEEDPRIOR_H

#include <iostream>
#include <Eigen/Dense>
#include "utility.hpp"
#include "integration_base.h"
#include "global_param.hpp"
#include <ceres/ceres.h>

using namespace Eigen;
class SpeedPriorFactor : public ceres::SizedCostFunction<1, 9>
{
public:
    SpeedPriorFactor()=delete;
    SpeedPriorFactor(double _average_speed):average_speed(_average_speed){b = 4;scale = 100;}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Vector3d Vs(parameters[0][0], parameters[0][1], parameters[0][2]);
        double speed = sqrt(Vs.x()*Vs.x()+Vs.y()*Vs.y()+Vs.z()*Vs.z());
        residuals[0] = scale * (sqrt(b * b + (average_speed - speed)*(average_speed - speed)) - b);
        if(jacobians)
        {
            if(jacobians[0])
            {
                double dy_dx = (speed-average_speed)/sqrt(b * b + (average_speed - speed)*(average_speed - speed));
                double dx_dv1 = Vs.x()/speed;
                double dx_dv2 = Vs.y()/speed;
                double dx_dv3 = Vs.z()/speed;
                double dy_dv1 = dy_dx * dx_dv1;
                double dy_dv2 = dy_dx * dx_dv2;
                double dy_dv3 = dy_dx * dx_dv3;
                Eigen::Matrix<double, 1, 3> dy_dv(1, 3);
                dy_dv << dy_dv1,dy_dv2,dy_dv3;
                Eigen::Map<Eigen::Matrix<double, 1, 9>> jacobian_speed(jacobians[0]);
                jacobian_speed.setZero();
                jacobian_speed.block<1, 3>(0, 0) = scale * dy_dv;
            }
        }
    }
    double average_speed;
    double b;
    double scale;
};