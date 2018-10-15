//
//  integration_base.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/25.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef integration_base_h
#define integration_base_h

#include "utility.hpp"
#include <ceres/ceres.h>
#include "global_param.hpp"

using namespace Eigen;
class IntegrationBase
{
public:

    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,const double &_encoder_v0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg, const Eigen::Matrix3d &_Rio)
    : acc_0{_acc_0}, gyr_0{_gyr_0}, encoder_v0{_encoder_v0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0}, linearized_encoder_v{_encoder_v0},
    linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg}, Rio{_Rio},
    m_jacobian{Eigen::Matrix<double, 16, 16>::Identity()}, m_covariance{Eigen::Matrix<double, 16, 16>::Zero()},
    jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
    sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}, delta_encoder_p{Eigen::Vector3d::Zero()},
    jacobian2{Eigen::Matrix<double, 18, 18>::Identity()}, covariance2{Eigen::Matrix<double, 18, 18>::Zero()}
    {
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (a_n * a_n) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (g_n * g_n) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (a_n * a_n) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (g_n * g_n) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (a_w * a_w) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (g_w * g_w) * Eigen::Matrix3d::Identity();

        noise2 = Eigen::Matrix<double, 24, 24>::Zero();
        noise2.block<3, 3>(0, 0) =  (a_n * a_n) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(3, 3) =  (g_n * g_n) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(6, 6) =  (a_n * a_n) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(9, 9) =  (g_n * g_n) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(12, 12) =  (a_w * a_w) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(15, 15) =  (g_w * g_w) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(18, 18) =  (v_n * v_n) * Eigen::Matrix3d::Identity();
        noise2.block<3, 3>(21, 21) =  (v_n * v_n) * Eigen::Matrix3d::Identity();
    }

    static double a_n;
    static double a_w;
    static double g_n;
    static double g_w;
    static double v_n;

    static void setIMUNoise(double _a_n ,double _a_w,double _g_n,double _g_w,double _v_n){
        a_n = _a_n;
        a_w = _a_w;
        g_n = _g_n;
        g_w = _g_w;
        v_n = _v_n;
    };

    void push_back(double dt, double encoder_v, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        encoder_v_buf.push_back(encoder_v);
        propagate(dt, encoder_v, acc, gyr);
    }
    
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        encoder_v0 = linearized_encoder_v;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        delta_encoder_p.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        covariance.setZero();
        jacobian2.setIdentity();
        covariance2.setZero();
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], encoder_v_buf[i], acc_buf[i], gyr_buf[i]);
    }

    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg, const Eigen::Matrix3d &_Rio)
    {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        encoder_v0 = linearized_encoder_v;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        delta_encoder_p.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        Rio = _Rio;
        jacobian.setIdentity();
        covariance.setZero();
        jacobian2.setIdentity();
        covariance2.setZero();
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], encoder_v_buf[i], acc_buf[i], gyr_buf[i]);
    }

    void clear()
    {
        dt_buf.clear();
        acc_buf.clear();
        gyr_buf.clear();
        encoder_v_buf.clear();
    }

    void repropagate()
    {
        sum_dt = 0.0;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        delta_encoder_p.setZero();
        jacobian.setIdentity();
        covariance.setZero();
        jacobian2.setIdentity();
        covariance2.setZero();
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], encoder_v_buf[i], acc_buf[i], gyr_buf[i]);
    }
    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const double &_encoder_v0, const double &_encoder_v1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v, const Eigen::Vector3d &delta_encoder_p,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v, Eigen::Vector3d &result_delta_encoder_p,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        //ROS_INFO("midpoint integration");
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        Eigen::Quaterniond qio{Rio};
        Vector3d encoderv_vec0(0, _encoder_v0, 0);
        Vector3d encoderv_vec1(0, _encoder_v1, 0);
        Vector3d un_encoderv_vec0 = (delta_q * qio) * encoderv_vec0;
        Vector3d un_encoderv_vec1 = (result_delta_q * qio) * encoderv_vec1;
        Vector3d un_encoderv_vec = 0.5 * (un_encoderv_vec0 + un_encoderv_vec1);
        result_delta_encoder_p = delta_encoder_p + un_encoderv_vec * _dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;
        
        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;
            
            R_w_x<< 0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
            
            R_a_0_x<< 0, -a_0_x(2), a_0_x(1),
                      a_0_x(2), 0, -a_0_x(0),
                      -a_0_x(1), a_0_x(0), 0;
            
            R_a_1_x<< 0, -a_1_x(2), a_1_x(1),
                      a_1_x(2), 0, -a_1_x(0),
                      -a_1_x(1), a_1_x(0), 0;

            Vector3d v_en_0_x = qio * encoderv_vec0;
            Vector3d v_en_1_x = qio * encoderv_vec1;
            Matrix3d R_v_en_0_x, R_v_en_1_x;
            R_v_en_0_x << 0, -v_en_0_x(2), v_en_0_x(1),
                    v_en_0_x(2), 0, -v_en_0_x(0),
                    -v_en_0_x(1), v_en_0_x(0), 0;
            R_v_en_1_x << 0, -v_en_1_x(2), v_en_1_x(1),
                    v_en_1_x(2), 0, -v_en_1_x(0),
                    -v_en_1_x(1), v_en_1_x(0), 0;
            
            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
            -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
            -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;
            
            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;
            
            //step_jacobian = F;
            //step_V = V;
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();


            MatrixXd F2 = MatrixXd::Zero(18, 18);
            F2.block<3, 3>(0, 0) = Matrix3d::Identity();
            F2.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F2.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F2.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F2.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F2.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F2.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F2.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F2.block<3, 3>(6, 6) = Matrix3d::Identity();
            F2.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F2.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F2.block<3, 3>(9, 9) = Matrix3d::Identity();
            F2.block<3, 3>(12, 12) = Matrix3d::Identity();
            F2.block<3, 3>(15, 3) = -0.5 * delta_q.toRotationMatrix() * R_v_en_0_x * _dt + (-0.5) * result_delta_q.toRotationMatrix() * R_v_en_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F2.block<3, 3>(15, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_v_en_1_x * _dt * -_dt;
            F2.block<3, 3>(15, 15) = Matrix3d::Identity();

            MatrixXd V2 = MatrixXd::Zero(18, 24);
            V2.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V2.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V2.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V2.block<3, 3>(0, 9) =  V2.block<3, 3>(0, 3);
            V2.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V2.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V2.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V2.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V2.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V2.block<3, 3>(6, 9) =  V2.block<3, 3>(6, 3);
            V2.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V2.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;
            V2.block<3, 3>(15, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_v_en_1_x  * _dt * 0.5 * _dt;
            V2.block<3, 3>(15, 9) = V2.block<3, 3>(15, 3);
            V2.block<3, 3>(15, 18) = 0.5 * delta_q.toRotationMatrix() * _dt;
            V2.block<3, 3>(15, 21) = 0.5 * result_delta_q.toRotationMatrix() * _dt;

            jacobian2 = F2 * jacobian2;
            covariance2 = F2 * covariance2 * F2.transpose() + V2 * noise2 * V2.transpose();
        }
        
    }

    void propagate(double _dt, double _encoder_v1, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        encoder_v1 = _encoder_v1;
        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_delta_encoder_p;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;

        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, encoder_v0, encoder_v1, delta_p, delta_q, delta_v, delta_encoder_p,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v, result_delta_encoder_p,
                            result_linearized_ba, result_linearized_bg, 1);

        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        delta_encoder_p = result_delta_encoder_p;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;
        encoder_v0 = encoder_v1;

    }
    
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;
        
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);
        
        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);
        
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        
        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
        
        Vector3d G{0,0,GRAVITY};
        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    Eigen::Matrix<double, 15, 1> evaluate2(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian2.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian2.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian2.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian2.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian2.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        Vector3d G{0,0,GRAVITY};
        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    Eigen::Vector3d CorrectEncoderP(const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi)
    {
        Eigen::Matrix3d denp_dba = jacobian2.block<3, 3>(O_ENV, O_BA);
        Eigen::Matrix3d denp_dbg = jacobian2.block<3, 3>(O_ENV, O_BG);
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        Eigen::Vector3d corrected_delta_encoder_p = delta_encoder_p + denp_dba * dba + denp_dbg * dbg;
        return corrected_delta_encoder_p;
    }

    Eigen::Vector3d ComputeJacobianyaw(double yawio)
    {
        Vector3d Jacobian_yaw(0, 0, 0);
        Matrix3d Ryawio_jacobian;
        Ryawio_jacobian << -sin(yawio), -cos(yawio), 0,
                cos(yawio), -sin(yawio), 0,
                0, 0, 0;

        Eigen::Vector3d tmp_gyr_0 = gyr_0;
        Eigen::Quaterniond tmp_delta_q = Eigen::Quaterniond::Identity();
        double tmp_encoder_v0 = encoder_v0;
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        {
            Eigen::Vector3d tmp_gyr_1 = gyr_buf[i];
            double tmp_encoder_v1 = encoder_v_buf[i];
            double tmp_dt = dt_buf[i];
            Eigen::Vector3d un_tmp_gyr = 0.5 * (tmp_gyr_0 + tmp_gyr_1) - linearized_bg;
            Eigen::Vector3d tmp_encoderv_vec0(0, tmp_encoder_v0, 0);
            Eigen::Vector3d tmp_encoderv_vec1(0, tmp_encoder_v1, 0);
            Eigen::Quaterniond result_tmp_delta_q = tmp_delta_q * Eigen::Quaterniond(1, un_tmp_gyr(0) * tmp_dt / 2, un_tmp_gyr(1) * tmp_dt / 2, un_tmp_gyr(2) * tmp_dt / 2);
            Eigen::Vector3d delta_Jacobian_yaw = 0.5 * tmp_delta_q.toRotationMatrix() * Ryawio_jacobian * tmp_encoderv_vec0 * tmp_dt + 0.5 * result_tmp_delta_q.toRotationMatrix() * Ryawio_jacobian * tmp_encoderv_vec1 * tmp_dt;
            Jacobian_yaw = Jacobian_yaw + delta_Jacobian_yaw;
            tmp_gyr_0 = tmp_gyr_1;
            tmp_encoder_v0 = tmp_encoder_v1;
            tmp_delta_q = result_tmp_delta_q;
        }

        return Jacobian_yaw;
    }

    /*Eigen::Matrix<double, 15, 1> evaluate_witht(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                              const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj,
                                              const Eigen::Quaterniond &rb_bk, const Eigen::Quaterniond &re_bk1, const Eigen::Vector3d &accb, const Eigen::Vector3d &acce, const double &syndt)
    {
            Eigen::Matrix<double, 15, 1> residuals;

            Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

            Eigen::Vector3d dba = Bai - linearized_ba;
            Eigen::Vector3d dbg = Bgi - linearized_bg;

            Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
            Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
            Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

            mcorrected_delta_v = corrected_delta_v;
            mcorrected_delta_p = corrected_delta_p;

            Vector3d G{0,0,GRAVITY};
            residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - (rb_bk.toRotationMatrix() * corrected_delta_p + sum_dt * syndt * rb_bk.toRotationMatrix() * accb - syndt * rb_bk.toRotationMatrix() * corrected_delta_v);
            residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
            residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - (rb_bk.toRotationMatrix() * corrected_delta_v + syndt * rb_bk.toRotationMatrix() * accb - syndt * rb_bk.toRotationMatrix() * corrected_delta_q.toRotationMatrix() * acce);
            residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
            residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
            return residuals;
    }*/
    
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    double encoder_v0;
    double encoder_v1;
    double linearized_encoder_v;
    
    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;
    
    Eigen::Matrix<double, 16, 16> m_jacobian, m_covariance;
    
    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;
    Eigen::Matrix<double, 18, 18> jacobian2, covariance2;
    Eigen::Matrix<double, 24, 24> noise2;
    
    double sum_dt;
    double vins_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    Eigen::Matrix3d Rio;

    Eigen::Vector3d delta_encoder_p;
    
    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
    std::vector<double> encoder_v_buf;

    Eigen::Vector3d mcorrected_delta_v;
    Eigen::Vector3d mcorrected_delta_p;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif /* integration_base_h */
