//
// Created by root on 18-6-20.
//

#include "encoder_displacement_factor.h"
//Eigen::Matrix3d EncoderDisplacementFactor::sqrt_info;

EncoderDisplacementFactor::EncoderDisplacementFactor(const Matrix3d &_Rio, const Vector3d &_pio, IntegrationBase* _pre_integration)
{
    Rio = _Rio;
    pio = _pio;
    pre_integration = _pre_integration;
    //delta_p_encoder = _delta_p_encoder;
    Eigen::Matrix3d cov_encoder_p = pre_integration->covariance2.block<3, 3>(O_ENV,O_ENV);
    sqrt_info = Eigen::LLT<Eigen::Matrix<double, 3, 3>>(cov_encoder_p.inverse()).matrixL().transpose();
}

bool EncoderDisplacementFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    //Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[2][3], parameters[2][4], parameters[2][5]);
    Eigen::Vector3d Bgi(parameters[2][6], parameters[2][7], parameters[2][8]);

    Eigen::Map<Eigen::Vector3d> residual(residuals);

    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();

    Eigen::Vector3d delta_p_encoder = pre_integration->CorrectEncoderP(Bai, Bgi);


    residual = Ri.inverse() * (Pj - Pi) - pio + Ri.inverse() * Rj * pio - delta_p_encoder;
    residual = sqrt_info * residual;

    if(jacobians)
    {
        if(jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.block<3, 3>(0, 0) = -Ri.inverse();
            Eigen::Vector3d displacement_imu = Ri.inverse() * (Pj - Pi);
            Eigen::Vector3d transformed_pio = Ri.inverse() * Rj * pio;
            jaco_i.block<3, 3>(0, 3) = Utility::skewSymmetric(displacement_imu) + Utility::skewSymmetric(transformed_pio);
            jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if(jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.block<3, 3>(0, 0) = Ri.inverse();
            jaco_j.block<3, 3>(0, 3) = Ri.inverse() * Rj * (-Utility::skewSymmetric(pio));
            jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }

        if(jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> jacobian_pose_vi(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_vi;
            Eigen::Matrix3d denp_dba = pre_integration->jacobian2.template block<3, 3>(O_ENV, O_BA);
            Eigen::Matrix3d denp_dbg = pre_integration->jacobian2.template block<3, 3>(O_ENV, O_BG);
            jaco_vi.block<3, 3>(0, 0) = -denp_dba;
            jaco_vi.block<3, 3>(0, 3) = -denp_dbg;
            jacobian_pose_vi.rightCols<6>() = sqrt_info * jaco_vi;
            jacobian_pose_vi.leftCols<3>() = Eigen::Matrix3d::Zero();
        }
    }

    return true;
}