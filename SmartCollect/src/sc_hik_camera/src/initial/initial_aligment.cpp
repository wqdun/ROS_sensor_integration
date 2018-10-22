//
//  initial_aligment.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "initial_aligment.hpp"
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;

    MatrixXd jacoA(all_image_frame.size() * 3, 3);
    VectorXd jacob(all_image_frame.size() * 3);
    jacoA.setZero();
    jacob.setZero();
    int tmp_count = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        jacoA.block<3, 3>(tmp_count * 3, 0) = tmp_A;
        jacob.segment<3>(tmp_count * 3) = tmp_b;
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;

        tmp_count++;
    }

    cout << "b norm before \t" << jacob.norm() << endl;
    delta_bg = A.ldlt().solve(b);
    cout << "b norm after \t" << (jacoA * delta_bg - jacob).norm() << endl;
    cout << " delta_bg ! " << delta_bg.transpose() << endl;
    
    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;
    
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

bool solveGyroscopeBiasOdometry(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Matrix3d& ric)
{

    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int countvalid = 0;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        //cout << "enter solve gyroscope bias: " << endl;
        //cout << frame_j->second.rot12.at<double>(0, 0) << endl;
        frame_j = next(frame_i);
        if(frame_j->second.is_rot12_valid == true)
        {
            countvalid++;

        }
    }
    if(countvalid < 3)
        return false;

    //MatrixXd jacoA(all_image_frame.size() * 3, 3);
    //VectorXd jacob(all_image_frame.size() * 3);
    //jacoA.setZero();
    //jacob.setZero();
    //int tmp_count = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        if(frame_j->second.is_rot12_valid == false)
            continue;
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();

        Eigen::Quaterniond q_ij(ric * frame_j->second.eigenrot12 * ric.transpose());
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        //jacoA.block<3, 3>(tmp_count * 3, 0) = tmp_A;
        //jacob.segment<3>(tmp_count * 3) = tmp_b;
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
        //tmp_count++;
    }
    //cout << "b norm before \t" << jacob.norm() << endl;
    delta_bg = A.ldlt().solve(b);
    //delta_bg = jacoA.colPivHouseholderQr().solve(jacob);
    //cout << "b norm after \t" << (jacoA * delta_bg - jacob).norm() << endl;
    cout << " delta_bg ! " << delta_bg.transpose() << endl;

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
        //Vector3d test_vec = 0.5 * frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG) * delta_bg;
        //frame_j->second.pre_integration->delta_q.x() = frame_j->second.pre_integration->delta_q.x() + test_vec.x();
        //frame_j->second.pre_integration->delta_q.y() = frame_j->second.pre_integration->delta_q.y() + test_vec.y();
        //frame_j->second.pre_integration->delta_q.z() = frame_j->second.pre_integration->delta_q.z() + test_vec.z();
    }

    return true;
}

/*bool solveGyroscopeBiasOdometry(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Matrix3d& ric)
{

    Matrix<double, 6, 6> A;
    Matrix<double, 6, 1> b;
    Matrix<double, 6, 1> delta_bg_x;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int countvalid = 0;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        //cout << "enter solve gyroscope bias: " << endl;
        //cout << frame_j->second.rot12.at<double>(0, 0) << endl;
        frame_j = next(frame_i);
        if(frame_j->second.is_rot12_valid == true)
        {
            countvalid++;

        }
    }
    if(countvalid < 3)
        return false;

    MatrixXd jacoA(all_image_frame.size() * 3, 6);
    VectorXd jacob(all_image_frame.size() * 3);
    jacoA.setZero();
    jacob.setZero();
    int tmp_count = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        if(frame_j->second.is_rot12_valid == false)
            continue;
        MatrixXd tmp_A(3, 6);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();

        Eigen::Quaterniond q_ij(ric * frame_j->second.eigenrot12 * ric.transpose());
        tmp_A.block<3, 3>(0, 0) = -0.5 * Utility::Qleft(q_ij.inverse() * frame_j->second.pre_integration->delta_q).bottomRightCorner<3, 3>() * frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        Quaterniond qic;
        qic = ric;
        Quaterniond qrot12;
        qrot12 = frame_j->second.eigenrot12;
        Matrix4d jaco_tmp_A1 = (Utility::Qleft(qic) * Utility::Qright(qrot12.inverse() * qic.inverse() * frame_j->second.pre_integration->delta_q));
        Matrix4d jaco_tmp_A2 = (Utility::Qleft(qic * qrot12.inverse()) * Utility::Qright(qic.inverse() * frame_j->second.pre_integration->delta_q));
        tmp_A.block<3, 3>(0, 3) =  jaco_tmp_A1.bottomRightCorner<3, 3>() - jaco_tmp_A2.bottomRightCorner<3, 3>();
        tmp_b = -(q_ij.inverse() * frame_j->second.pre_integration->delta_q).vec();
        jacoA.block<3, 6>(tmp_count * 3, 0) = tmp_A;
        jacob.segment<3>(tmp_count * 3) = tmp_b;
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
        tmp_count++;
    }
    cout << "b norm before \t" << jacob.norm() << endl;
    delta_bg_x = A.ldlt().solve(b);
    //delta_bg = jacoA.colPivHouseholderQr().solve(jacob);
    cout << "b norm after \t" << (jacoA * delta_bg_x - jacob).norm() << endl;
    Vector3d delta_bg;
    delta_bg = delta_bg_x.head<3>();
    cout << " delta_bg ! " << delta_bg.transpose() << endl;

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
        //Vector3d test_vec = 0.5 * frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG) * delta_bg;
        //frame_j->second.pre_integration->delta_q.x() = frame_j->second.pre_integration->delta_q.x() + test_vec.x();
        //frame_j->second.pre_integration->delta_q.y() = frame_j->second.pre_integration->delta_q.y() + test_vec.y();
        //frame_j->second.pre_integration->delta_q.z() = frame_j->second.pre_integration->delta_q.z() + test_vec.z();
    }

    return true;
}*/


MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

void RefineGravityOdometry(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x, Vector3d& tio)
{
    Vector3d g0 = g.normalized() * G_NORM;

    cout << g0 << endl;
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
        A.setZero();
        b.setZero();
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 8);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;

            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;

            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p - frame_j->second.pre_integration->delta_encoder_p - tio + frame_j->second.pre_integration->delta_q.normalized().toRotationMatrix() * tio - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_j->second.pre_integration->delta_q.toRotationMatrix();
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;
            //Vector3d TIC;
            //TIC<<TIC_X,TIC_Y,TIC_Z;
            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(3 * i, 3 * i) += r_A.topLeftCorner<6, 6>();
            A.block<6, 2>(3 * i, n_state - 2) += r_A.topRightCorner<6, 2>();
            A.block<2, 6>(n_state - 2, 3 * i) += r_A.bottomLeftCorner<2, 6>();
            A.bottomRightCorner<2, 2>() += r_A.bottomRightCorner<2, 2>();

            b.segment<6>(3 * i) += r_b.head<6>();
            b.tail<2>() += r_b.tail<2>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        VectorXd dg = x.segment<2>(n_state - 2);
        g0 = (g0 + lxly * dg).normalized() * G_NORM;
        cout << "dg" << "\t" << dg << endl;
        cout << "g0" << "\t" << g0 << endl;
    }
    g = g0;
}

void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    Vector3d g0 = g.normalized() * G_NORM;
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;
    
    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);
            
            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();
            
            double dt = frame_j->second.pre_integration->sum_dt;
            
            
            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
            Vector3d TIC;
            TIC<<TIC_X,TIC_Y,TIC_Z;
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC - frame_i->second.R.transpose() * dt * dt / 2 * g0;
            //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
            
            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;
            
            
            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();
            
            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
            
            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();
            
            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();
            
            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        VectorXd dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * G_NORM;
        //double s = x(n_state - 1);
    }
    g = g0;
}

bool SolveScale(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    printf("SolveScale\n");
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 3 + 1;
    
    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);
        
        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();
        
        double dt = frame_j->second.pre_integration->sum_dt;
        
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;
        Vector3d TIC;
        TIC<<TIC_X,TIC_Y,TIC_Z;
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC;
        //tmp_b.block<3, 1>(0, 0) = imu_factors[i + 1]->pre_integration.delta_p;
        //cout << "delta_p   " << frame_j->second.imu_factor->pre_integration.delta_p.transpose() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration.delta_v.transpose() << endl;
        
        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();
        
        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;
        
        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();
        
        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();
        
        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0;
    printf("estimated scale: %f\n", s);
    g = x.segment<3>(n_state - 4);
    cout << " result g     " << g.norm() << " " << g.transpose() << endl;
    if(fabs(g.norm() - G_NORM) > G_THRESHOLD || s < 0)
    {
        return false;
    }
    
    RefineGravity(all_image_frame, g, x);
    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    printf("refine estimated scale: %f", s);
    cout << " refine     " << g.norm() << " " << g.transpose() << endl;
    if(s > 0.0 )
    {
        printf("initial succ!\n");
        
    }
    else
    {
        printf("initial fail\n");
        return false;
    }
    return true;
}

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    solveGyroscopeBias(all_image_frame, Bgs);
    
    if(SolveScale(all_image_frame, g, x))
        return true;
    else 
        return false;
}

bool VisualIMUAlignmentOdometry(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x, Vector3d &tio, Matrix3d& ric)
{
    if(!solveGyroscopeBiasOdometry(all_image_frame, Bgs, ric))
    {
        return false;
    }
    else
    {
        RefineGravityOdometry(all_image_frame, g, x, tio);
        return true;
    }
}