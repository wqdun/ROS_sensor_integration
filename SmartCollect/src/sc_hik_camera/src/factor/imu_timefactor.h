//
//  imu_factor.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/25.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef imu_timefactor_h
#define imu_timefactor_h

/*#include <iostream>
#include <Eigen/Dense>
#include "utility.hpp"
#include "integration_base.h"
#include "global_param.hpp"
#include <ceres/ceres.h>

using namespace Eigen;
class IMUTimeFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9, 1>
{
public:
    IMUTimeFactor() = delete;
    IMUTimeFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

        double syndt = parameters[4][0] - pre_integration->vins_dt;
        Eigen::Vector3d gyr_begin = pre_integration->linearized_gyr - pre_integration->linearized_bg;
        //double wx1 = gyr_begin.x(); double wy1 = gyr_begin.y(); double wz1 = gyr_begin.z();
        Eigen::Vector3d gyr_end = pre_integration->gyr_buf.back() - pre_integration->linearized_bg;
        //double wx2 = gyr_end.x(); double wy2 = gyr_end.y(); double wz2 = gyr_end.z();
        Eigen::Quaterniond rb_bk = Utility::deltaQ(gyr_begin * syndt);
        Eigen::Quaterniond re_bk1 = Utility::deltaQ(gyr_end * syndt);
        Eigen::Vector3d accb = pre_integration->linearized_acc - pre_integration->linearized_ba;
        Eigen::Vector3d acce = pre_integration->acc_buf.back() - pre_integration->linearized_ba;

        //Eigen::Matrix<double, 15, 15> Fd;
        //Eigen::Matrix<double, 15, 12> Gd;

        //Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
        //Eigen::Quaterniond pQj = Qi * delta_q;
        //Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
        //Eigen::Vector3d pBaj = Bai;
        //Eigen::Vector3d pBgj = Bgi;

        //Vi + Qi * delta_v - g * sum_dt = Vj;
        //Qi * delta_q = Qj;

        //delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
        //delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
        //delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration->evaluate_witht(Pi, Qi, Vi, Bai, Bgi,
                                             Pj, Qj, Vj, Baj, Bgj,
                                             rb_bk, re_bk1, accb, acce, syndt);
        Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
        residual.block<3, 1>(O_R, 0) = 2 * (re_bk1 * corrected_delta_q.inverse() * rb_bk.inverse() * (Qi.inverse() * Qj)).vec();

        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = p* sqrt_info * residual;

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);



            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

            if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
            {
                std::cout << pre_integration->jacobian << std::endl;
                ///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                Vector3d G{0,0,GRAVITY};
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(rb_bk * corrected_delta_q * re_bk1.inverse())).bottomRightCorner<3, 3>();
#endif
                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    std::cout << sqrt_info << std::endl;
                    assert(false);
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -rb_bk.toRotationMatrix() * dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -rb_bk.toRotationMatrix() * dp_dbg;

#if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -(Utility::Qleft(Qj.inverse() * Qi * rb_bk *corrected_delta_q) * Utility::Qright(re_bk1.inverse())).bottomRightCorner<3, 3>() * dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -rb_bk.toRotationMatrix() * dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -rb_bk.toRotationMatrix() * dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

                assert(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                assert(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(re_bk1 * corrected_delta_q.inverse() * rb_bk.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                assert(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                assert(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                assert(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                assert(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
            if (jacobians[4])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 1>> jacobian_t(jacobians[4]);
                jacobian_t.setZero();
                //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                //Eigen::Matrix3d dq_dw1 = (Utility::Qleft(re_bk1.inverse() * corrected_delta_q.inverse() * rb_bk) * Utility::Qright(Qi.inverse() * Qj)).bottomRightCorner<3, 3>();
                //Eigen::Matrix3d dq_dw2 = -(Utility::Qleft(Qj.inverse() * Qi * rb_bk.inverse() * corrected_delta_q * re_bk1)).bottomRightCorner<3, 3>();
                Eigen::Matrix3d dq_dw1 = -(Utility::Qleft(Qj.inverse() * Qi * rb_bk) * Utility::Qright(corrected_delta_q * re_bk1.inverse())).bottomRightCorner<3, 3>();
                Eigen::Matrix3d dq_dw2 = (Utility::Qleft(re_bk1) * Utility::Qright(corrected_delta_q.inverse() * rb_bk.inverse() * Qi.inverse() * Qj)).bottomRightCorner<3, 3>();
                Eigen::Vector3d dq_dt1 = dq_dw1 * gyr_begin; //Eigen::Vector3d dq_dt1 = 0.5 * dq_dw1 * gyr_begin;
                Eigen::Vector3d dq_dt2 = dq_dw2 * gyr_end; //Eigen::Vector3d dq_dt2 = 0.5 * dq_dw2 * gyr_end;
                Eigen::Vector3d dq_dt = dq_dt1 + dq_dt2;
                jacobian_t.block<3, 1>(O_R, 0) = dq_dt;

                Eigen::Vector3d innerv = pre_integration->mcorrected_delta_v + syndt * accb - syndt * corrected_delta_q.toRotationMatrix() * acce;
                Eigen::Matrix3d dv_dw1 = rb_bk.toRotationMatrix() * Utility::skewSymmetric(innerv) * (Eigen::Matrix3d::Identity() - 0.5 * Utility::skewSymmetric(gyr_begin * syndt));
                Eigen::Vector3d dv_dt1 = dv_dw1 * gyr_begin; //Eigen::Vector3d dv_dt1 = 0.5 * dv_dw1 * gyr_begin;
                Eigen::Vector3d dv_dt2 = rb_bk.toRotationMatrix() * (corrected_delta_q.toRotationMatrix() * acce - accb);
                Eigen::Vector3d dv_dt = dv_dt1 + dv_dt2;
                jacobian_t.block<3, 1>(O_V, 0) = dv_dt;

                Eigen::Vector3d innerp = pre_integration->mcorrected_delta_p + pre_integration->sum_dt * syndt * accb - syndt * pre_integration->mcorrected_delta_v;
                Eigen::Matrix3d dp_dw1 = rb_bk.toRotationMatrix() * Utility::skewSymmetric(innerp) * (Eigen::Matrix3d::Identity() - 0.5 * Utility::skewSymmetric(gyr_begin * syndt));
                Eigen::Vector3d dp_dt1 = dp_dw1 * gyr_begin; //Eigen::Vector3d dp_dt1 = 0.5 * dp_dw1 * gyr_begin;
                Eigen::Vector3d dp_dt2 = rb_bk.toRotationMatrix() * (pre_integration->mcorrected_delta_v - accb * pre_integration->sum_dt);
                Eigen::Vector3d dp_dt = dp_dt1 + dp_dt2;
                jacobian_t.block<3, 1>(O_P, 0) = dp_dt;

                jacobian_t = sqrt_info * jacobian_t;
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    IntegrationBase* pre_integration;
    static double p;

};*/


#endif /* imu_timefactor_h */
