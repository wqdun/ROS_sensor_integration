//
//  imu_factor.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/25.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef imu_factor_h
#define imu_factor_h

#include <iostream>
#include <Eigen/Dense>
#include "utility.hpp"
#include "integration_base.h"
#include "global_param.hpp"
#include <ceres/ceres.h>

/*using namespace Eigen;
class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
    IMUFactor() = delete;
    IMUFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
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
        residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
                                             Pj, Qj, Vj, Baj, Bgj);
        
        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = p* sqrt_info * residual;
        
        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);
            
            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
            
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
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
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
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;
                
#if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif
                
                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;
                
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
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
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

/*using namespace Eigen;
class IMUFactor : public ceres::SizedCostFunction<18, 7, 9, 7, 9>
{
public:
    IMUFactor() = delete;
    IMUFactor(const Matrix3d &_Rio, const Vector3d &_pio, IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
        Rio = _Rio;
        pio = _pio;
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

        Eigen::Map<Eigen::Matrix<double, 18, 1>> residual(residuals);
        residual.head<15>() = pre_integration->evaluate2(Pi, Qi, Vi, Bai, Bgi,
                                             Pj, Qj, Vj, Baj, Bgj);

        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Vector3d delta_p_encoder = pre_integration->CorrectEncoderP(Bai, Bgi);
        residual.tail<3>() = Ri.inverse() * (Pj - Pi) - pio + Ri.inverse() * Rj * pio - delta_p_encoder;

        Eigen::Matrix<double, 18, 18> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 18, 18>>(pre_integration->covariance2.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = p* sqrt_info * residual;

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian2.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian2.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian2.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian2.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian2.template block<3, 3>(O_V, O_BG);

            Eigen::Matrix3d denp_dba = pre_integration->jacobian2.template block<3, 3>(O_ENV, O_BA);
            Eigen::Matrix3d denp_dbg = pre_integration->jacobian2.template block<3, 3>(O_ENV, O_BG);

            if (pre_integration->jacobian2.maxCoeff() > 1e8 || pre_integration->jacobian2.minCoeff() < -1e8)
            {
                std::cout << pre_integration->jacobian2 << std::endl;
                ///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                Vector3d G{0,0,GRAVITY};
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif
                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i.block<3, 3>(O_ENV, O_P) = -Ri.inverse();
                Eigen::Vector3d displacement_imu = Ri.inverse() * (Pj - Pi);
                Eigen::Vector3d transformed_pio = Ri.inverse() * Rj * pio;
                jacobian_pose_i.block<3, 3>(O_ENV, O_R) = Utility::skewSymmetric(displacement_imu) + Utility::skewSymmetric(transformed_pio);

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    std::cout << sqrt_info << std::endl;
                    assert(false);
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_ENV, O_BA - O_V) = -denp_dba;
                jacobian_speedbias_i.block<3, 3>(O_ENV, O_BG - O_V) = -denp_dbg;

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

                assert(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                assert(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_j.block<3, 3>(O_ENV, O_P) = Ri.inverse();
                jacobian_pose_j.block<3, 3>(O_ENV, O_R) = Ri.inverse() * Rj * (-Utility::skewSymmetric(pio));

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                assert(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                assert(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                assert(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                assert(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    Matrix3d Rio;
    Vector3d pio;
    IntegrationBase* pre_integration;

    static double p;

};*/

using namespace Eigen;
class IMUFactor : public ceres::SizedCostFunction<18, 7, 9, 7, 9, 1>
{
public:
    IMUFactor() = delete;
    IMUFactor(const Matrix3d &_Rioyx, const Vector3d &_pio, IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
        Rioyx = _Rioyx;
        pio = _pio;
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

        double yawio = parameters[4][0];

        Eigen::Matrix3d Ryawio;
        Ryawio << cos(yawio), -sin(yawio), 0,
                sin(yawio), cos(yawio), 0,
                0, 0, 1;

        Eigen::Matrix3d Rio = Ryawio * Rioyx;

        //pre_integration->repropagate(Bai, Bgi, Rio);

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

        Eigen::Map<Eigen::Matrix<double, 18, 1>> residual(residuals);
        residual.head<15>() = pre_integration->evaluate2(Pi, Qi, Vi, Bai, Bgi,
                                                         Pj, Qj, Vj, Baj, Bgj);

        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Vector3d delta_p_encoder = pre_integration->CorrectEncoderP(Bai, Bgi);
        residual.tail<3>() = Ri.inverse() * (Pj - Pi) - pio + Ri.inverse() * Rj * pio - delta_p_encoder;

        Eigen::Matrix<double, 18, 18> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 18, 18>>(pre_integration->covariance2.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = p* sqrt_info * residual;

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian2.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian2.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian2.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian2.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian2.template block<3, 3>(O_V, O_BG);

            Eigen::Matrix3d denp_dba = pre_integration->jacobian2.template block<3, 3>(O_ENV, O_BA);
            Eigen::Matrix3d denp_dbg = pre_integration->jacobian2.template block<3, 3>(O_ENV, O_BG);

            if (pre_integration->jacobian2.maxCoeff() > 1e8 || pre_integration->jacobian2.minCoeff() < -1e8)
            {
                std::cout << pre_integration->jacobian2 << std::endl;
                ///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                Vector3d G{0,0,GRAVITY};
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif
                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i.block<3, 3>(O_ENV, O_P) = -Ri.inverse();
                Eigen::Vector3d displacement_imu = Ri.inverse() * (Pj - Pi);
                Eigen::Vector3d transformed_pio = Ri.inverse() * Rj * pio;
                jacobian_pose_i.block<3, 3>(O_ENV, O_R) = Utility::skewSymmetric(displacement_imu) + Utility::skewSymmetric(transformed_pio);

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                    std::cout << sqrt_info << std::endl;
                    assert(false);
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_ENV, O_BA - O_V) = -denp_dba;
                jacobian_speedbias_i.block<3, 3>(O_ENV, O_BG - O_V) = -denp_dbg;

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

                assert(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                assert(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_j.block<3, 3>(O_ENV, O_P) = Ri.inverse();
                jacobian_pose_j.block<3, 3>(O_ENV, O_R) = Ri.inverse() * Rj * (-Utility::skewSymmetric(pio));

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                assert(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                assert(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                assert(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                assert(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
            if(jacobians[4])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 1>> jacobian_rio_yaw(jacobians[4]);
                jacobian_rio_yaw.setZero();
                Eigen::Vector3d denv_dyaw = pre_integration->ComputeJacobianyaw(yawio);
                jacobian_rio_yaw.block<3, 1>(O_ENV, 0) = -denv_dyaw;
                jacobian_rio_yaw = sqrt_info * jacobian_rio_yaw;
                assert(fabs(jacobian_rio_yaw.maxCoeff()) < 1e8);
                assert(fabs(jacobian_rio_yaw.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    Matrix3d Rioyx;
    //Matrix3d Riox;
    Vector3d pio;
    IntegrationBase* pre_integration;

    static double p;

};


#endif /* imu_factor_h */
