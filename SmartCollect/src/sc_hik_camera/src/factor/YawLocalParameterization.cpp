//
// Created by root on 18-7-18.
//

#include <global_param.hpp>
#include "YawLocalParameterization.h"

bool YawLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    /*Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();*/
    x_plus_delta[0] = x[0] + delta[0];
    if(x_plus_delta[0] > C_PI)
        x_plus_delta[0] -= (2 * C_PI);
    else if (x_plus_delta[0] < -C_PI)
        x_plus_delta[0] += (2 * C_PI);

    return true;
}
bool YawLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 1, 1>> j(jacobian);
    j.setIdentity();

    return true;
}