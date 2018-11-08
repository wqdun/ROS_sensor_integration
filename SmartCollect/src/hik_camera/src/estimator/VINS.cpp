//
//  VINS.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include <factor/encoder_displacement_factor.h>
#include "VINS.hpp"
double FOCUS_LENGTH_X;
double FOCUS_LENGTH_Y;
double PX;
double PY;
//bool LOOP_CLOSURE = false;
double  IMUFactor::p = 0;
//double  IMUTimeFactor::p = 0;

VINS::VINS(string strSettingPath, Matrix3d _Rio, Vector3d _tio)
:fail_times{0},need_recover(false),
failure_hand{false},is_failure{false}
     //drawresult{0.0, 0.0, 0.0, 0.0, 0.0, 7.0}
{

    fSettings = cv::FileStorage(strSettingPath, cv::FileStorage::READ);
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(strSettingPath);

    cv::Mat cv_R, cv_T;
    fSettings["extrinsicRotation"] >> cv_R;
    fSettings["extrinsicTranslation"] >> cv_T;
    fSettings["datafolder"] >> msdatafolder;
    //mswriteodometrypath = "odometry.txt";
cout << "before opening txt" << endl;
    //FILE* fpodometry = fopen(mswriteodometrypath.c_str(),"w");
    //fclose(fpodometry);
cout << "after closing txt" << endl;
    cv::cv2eigen(cv_R, config_ric);
    cv::cv2eigen(cv_T, config_tic);

    config_rio = _Rio;
    config_tio = _tio;

    f_manager = new FeatureManager(Rs,config_ric);

    printf("init VINS begins\n");
    t_drift.setZero();
    r_drift.setIdentity();
    clearState();

    last_P.setZero();
    last_R.setIdentity();
    last_P_old.setZero();
    last_R_old.setIdentity();
    
    nmarkerID = 0;
//    cout << ProjectionFactor::sqrt_info <<endl;
//
    this->setIMUModel();
}

void VINS::setIMUModel()
{

    ProjectionFactor::sqrt_info = FOCUS_LENGTH_X / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCUS_LENGTH_X / 1.5 * Matrix2d::Identity();
    IMUFactor::p = 1;
    //IMUTimeFactor::p = 1;
}

void VINS::clearState()
{
    frame_count = 0;
    first_imu = false;
    solver_flag = INITIAL;
    last_marginalization_info = nullptr;
    initial_timestamp = 0;
    csolve_visiononly = false;
    vsolve_visiononly = false;
    all_image_frame.clear();
    while(!imu_slide_quaternions.empty())
    imu_slide_quaternions.pop();
    while(!imu_t.empty())
    imu_t.pop();
    while(!imu_angular_velocities.empty())
    imu_angular_velocities.pop();
    cumulate_quaternion.w() = 1;
    cumulate_quaternion.x() = 0;
    cumulate_quaternion.y() = 0;
    cumulate_quaternion.z() = 0;
    sum_dt = 0;
    sum_dx = 0;
    average_speed = 0;
    big_turning = false;
    imu_written = false;
    synchronized_flag = false;
    dt2 = 0;
    //myawio = -0.017;
    //myawio = -0.02;
    rec_header = 0;
    myawio = -0.012;
    optimize_num = 0;
    mean_diff_accs = 0;
    count_keyframe = 0;
    pre_cumu_R.setIdentity();
    pre_cumu_P.setZero();
    rec_enh.setZero();
    Delta_t.clear();
    Min_thetas.clear();

    printf("clear state\n");
    for (int i = 0; i < 10 * (WINDOW_SIZE + 1); i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        cPs[i].setZero();
        cQs[i].w() = 1;
        cQs[i].x() = 0;
        cQs[i].y() = 0;
        cQs[i].z() = 0;
        Qualities[i] = true;
        ImuLinks[i] = 1;
        nFeatures[i] = 0;
        IMU_linear[i].setZero();
        IMU_angular[i].setIdentity();
        pre_integrations[i] = nullptr;
        cumulate_preintegrations[i] = nullptr;
        dt_buf[i].clear();
        encoder_v_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
    }
    preintegrations_lasttwo = nullptr;

    //    [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
//            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
//            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
//            0.0, 0.0, 0.0, 1.0]    T^i_c

//    ric << 0.0148655429818, -0.999880929698, 0.00414029679422,
//            0.999557249008, 0.0149672133247, 0.025715529948,
//            -0.0257744366974, 0.00375618835797, 0.999660727178;


//    ric << 0,-1,0,
//            -1,0,0,
//            0,0,-1;
//
//    tic <<  -0.021640145497,
//            -0.064676986768,
//            0.00981073058949;

    ric << config_ric;
    tic << config_tic;

//    tic << TIC_X,
//           TIC_Y,
//           TIC_Z;
//    ric = Utility::ypr2R(Vector3d(RIC_y,RIC_p,RIC_r));
    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (output_pre_integration != nullptr)
        delete output_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    output_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();
    f_manager->clearState();

}

void VINS::old2new()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic.x();
        para_Ex_Pose[i][1] = tic.y();
        para_Ex_Pose[i][2] = tic.z();
        Quaterniond q{ric};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
    para_dt[0] = dt2;
    para_yaw[0] = myawio;
     //triangulate
    VectorXd dep = f_manager->getDepthVector();
    for (int i = 0; i < f_manager->getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
}

void VINS::solve_marker()
{
    Vector3d Pcs[frame_count - 1];
    Matrix3d Rcs[frame_count - 1];
    vector<cv::Mat> vcvPs;
    for(int i = 0; i < frame_count - 1; i++)
    {
        Pcs[i] = Ps[i] + Rs[i] * tic;
        Rcs[i] = Rs[i] * ric;
	Matrix3d Rws = Rcs[i].inverse();
        Vector3d tws = -Rws * Pcs[i];
        cv::Mat mP(3, 4, CV_64F);
	for(int row = 0; row < 3; row++)
	{
	    for(int col = 0; col < 3; col++)
	    {
		mP.at<double>(row, col) = Rws(row, col);
            }
	    mP.at<double>(row, 3) = tws(row, 0);
	}
	vcvPs.push_back(mP.clone());
    }

    //FILE* fp_mpts2 = fopen("/home/nvidia/markerpoints2.txt", "a");
    //for(int i = 0; i < vmarkerspID.size(); i++)
    //{
	//for(int j = 0; j < vmarkerspID[i].vmarkerspf.size(); j++)
	//{
	//	fprintf(fp_mpts2, "%16.6lf\t", Headers[vmarkerspID[i].vmarkerspf[j].frame_idx]);
	//	fprintf(fp_mpts2, "%lf\t,%lf\n", vmarkerspID[i].vmarkerspf[j].pt[0], vmarkerspID[i].vmarkerspf[j].pt[1]);	
	//}
    //}
    //fprintf(fp_mpts2, "\n\n");
    //fclose(fp_mpts2);

    Eigen::Matrix3d mK = Eigen::Matrix3d::Identity();
    mK(0, 0) = 1377;
    mK(1, 1) = 1377;
    mK(0, 2) = 471.5;
    mK(1, 2) = 314.9;
    Eigen::Matrix3d mK_inv = mK.inverse();
    
    cv::Mat DistCoef(4, 1, CV_64F);
    DistCoef.at<double>(0, 0) = -0.2424;
    DistCoef.at<double>(1, 0) = 0.4378;
    DistCoef.at<double>(2, 0) = -0.000276;
    DistCoef.at<double>(3, 0) = -0.00187;

    cv::Mat cvK = cv::Mat::eye(3, 3, CV_64F);
    cvK.at<double>(0, 0) = mK(0, 0);
    cvK.at<double>(0, 2) = mK(0, 2);
    cvK.at<double>(1, 1) = mK(1, 1);
    cvK.at<double>(1, 2) = mK(1, 2);

    vector<MarkerPoints3D> vgood3DPointshehe;
    for(int i = 0; i < vmarkerspID.size(); i++)
    {
        int count_markers = 0;
        for(int j = 0; j < vmarkerspID[i].vmarkerspf.size(); j++)
	{
	    if(vmarkerspID[i].vmarkerspf[j].frame_idx >= 0 && vmarkerspID[i].vmarkerspf[j].frame_idx < (frame_count - 1))
            {
		count_markers++;
            }
	}
	cout << "count_markers(keyframe count):" << count_markers<<endl;

        if(count_markers < 3)
            continue;

	vector<cv::Mat> vgoodPoses;
        vector<cv::Mat> vgoodobs;
	for(int j = 0; j < vmarkerspID[i].vmarkerspf.size(); j++)
        {
	     if(vmarkerspID[i].vmarkerspf[j].frame_idx >= 0 && vmarkerspID[i].vmarkerspf[j].frame_idx < (frame_count - 1))
             {
                 double mx = vmarkerspID[i].vmarkerspf[j].pt[0];
                 double my = vmarkerspID[i].vmarkerspf[j].pt[1];
                 //Eigen::Vector3d vm(mx, my, 1);
                 //Eigen::Vector3d vdp;
                 //vdp = mK_inv * vm;
                 cv::Mat cvpd(1, 2, CV_64F);
                 cvpd.at<double>(0, 0) = mx;
                 cvpd.at<double>(0, 1) = my;
		 cout << cvpd << endl;
                 cvpd = cvpd.reshape(2);
                 cv::undistortPoints(cvpd, cvpd,cvK, DistCoef,cv::Mat(),cv::Mat::eye(3, 3, CV_64F));		 
                 cvpd = cvpd.reshape(1);
		 cout << cvpd << endl; 
                 cv::Mat tmp_obs(2, 1, CV_64F); 
                 tmp_obs.at<double>(0, 0) = cvpd.at<double>(0, 0);
                 tmp_obs.at<double>(1, 0) = cvpd.at<double>(0, 1);
		 vgoodobs.push_back(tmp_obs.clone());
		 vgoodPoses.push_back(vcvPs[vmarkerspID[i].vmarkerspf[j].frame_idx].clone());
             }
        }

	

	 cv::Mat P1 = vgoodPoses.front().clone();
         cv::Mat P2 = vgoodPoses.back().clone();
         cv::Mat ptu1 = vgoodobs.front().clone();
         cv::Mat ptu2 = vgoodobs.back().clone();
         cv::Mat ah3Dpt(4, 1, CV_64F);
         cv::triangulatePoints(P1, P2, ptu1, ptu2, ah3Dpt);



         cv::Mat a3Dpt(3, 1, CV_64F);
         a3Dpt.at<double>(0, 0) = ah3Dpt.at<double>(0, 0) / ah3Dpt.at<double>(3, 0);
         a3Dpt.at<double>(1, 0) = ah3Dpt.at<double>(1, 0) / ah3Dpt.at<double>(3, 0);
         a3Dpt.at<double>(2, 0) = ah3Dpt.at<double>(2, 0) / ah3Dpt.at<double>(3, 0);

			cv::Mat mP_last1 = P1.clone();
			cv::Mat mR_last1 = mP_last1.colRange(0, 3).rowRange(0, 3);
			cv::Mat mt_last1 = mP_last1.rowRange(0, 3).col(3);
			cv::Mat pincamera1(3, 1, CV_64F);
			pincamera1 = mR_last1 * a3Dpt + mt_last1;
			cout << "last z:"<<" " << pincamera1.at<double>(2, 0) << endl;
	
			cv::Mat mP_last2 = P2.clone();
			cv::Mat mR_last2 = mP_last2.colRange(0, 3).rowRange(0, 3);
			cv::Mat mt_last2 = mP_last2.rowRange(0, 3).col(3);
			cv::Mat pincamera2(3, 1, CV_64F);
			pincamera2 = mR_last2 * a3Dpt + mt_last2;
			cout << "last z:"<<" " << pincamera2.at<double>(2, 0) << endl;
	 cv::Point3f pa3Dpt;
         pa3Dpt.x = a3Dpt.at<double>(0, 0);
	 pa3Dpt.y = a3Dpt.at<double>(1, 0);
         pa3Dpt.z = a3Dpt.at<double>(2, 0);
         vector<cv::Point3f> vpa3Dpt;
         vpa3Dpt.push_back(pa3Dpt);
	vector<cv::Mat> vrealgoodPoses;
        vector<cv::Mat> vrealgoodobs;
	int count_real_good = 0;
	 for(int j = 0; j < vgoodobs.size(); j++)
	{
		//cv::Mat rvec(3, 1, CV_64F);
         	//cv::Rodrigues(vgoodPoses[j].rowRange(0, 3).colRange(0, 3), rvec);
		//cv::Mat tvec(3, 1, CV_64F);
                //tvec = vgoodPoses[j].rowRange(0, 3).col(3);
	        //vector<cv::Point2f>  vpimgpt;
	        //cv::projectPoints(vpa3Dpt, rvec, tvec, cv::Mat::eye(3, 3, CV_64F), DistCoef, vpimgpt);
		cv::Mat Rn(3, 3, CV_64F);
		cv::Mat tn(3, 1, CV_64F);
		Rn = vgoodPoses[j].rowRange(0, 3).colRange(0, 3);
		tn = vgoodPoses[j].rowRange(0, 3).col(3);
		cv::Mat imnh(3, 1, CV_64F);
		cv::Mat imn(2, 1, CV_64F);
                imnh = Rn * a3Dpt + tn;
                            imn.at<double>(0, 0) = imnh.at<double>(0, 0) / imnh.at<double>(2, 0);
                            imn.at<double>(1, 0) = imnh.at<double>(1, 0) / imnh.at<double>(2, 0);
		//if(sqrt((vpimgpt[0].x - vgoodobs[j].at<double>(0, 0)) * (vpimgpt[0].x - vgoodobs[j].at<double>(0, 0)) + (vpimgpt[1].x - vgoodobs[j].at<double>(1, 0)) * (vpimgpt[1].x - vgoodobs[j].at<double>(1, 0))) * 1100 < 50 )
		if(sqrt((imn.at<double>(0, 0) - vgoodobs[j].at<double>(0, 0)) * (imn.at<double>(0, 0) - vgoodobs[j].at<double>(0, 0)) + (imn.at<double>(1, 0) - vgoodobs[j].at<double>(1, 0)) * (imn.at<double>(1, 0) - vgoodobs[j].at<double>(1, 0))) * 1100 < 50 )
		{
			count_real_good++;
			vrealgoodPoses.push_back(vgoodPoses[j].clone());
			vrealgoodobs.push_back(vgoodobs[j].clone());
		}

cout<<"Pose of keyframe:"<<endl;
cout << vgoodPoses[j] << endl;
cout << vgoodobs[j] << endl;		

cout <<"reprojection error:"<< sqrt((imn.at<double>(0, 0) - vgoodobs[j].at<double>(0, 0)) * (imn.at<double>(0, 0) - vgoodobs[j].at<double>(0, 0)) + (imn.at<double>(1, 0) - vgoodobs[j].at<double>(1, 0)) * (imn.at<double>(1, 0) - vgoodobs[j].at<double>(1, 0))) * 1100 << endl;
	}
	cout << "count_real_good:"<< count_real_good << endl;
         if(count_real_good < 3)
	{
		continue;
	}
	else
	{
		bool flag_refined;
         	float norm1;
         	float norm2;
         	cv::Mat worldpos_refined = Triangulation(vrealgoodPoses, vrealgoodobs, a3Dpt.clone(), cv::Mat::eye(3, 3, CV_64F), 10, flag_refined, norm1, norm2);
		
		cout << "marker	ID:" <<  vmarkerspID[i].ID << " " <<"worldpos_refined:" << worldpos_refined <<endl;
		bool minusflag = false;
		for(int j = 0; j < vrealgoodobs.size(); j++)
		{
			cv::Mat mP_last = vrealgoodPoses[j].clone();
			cv::Mat mR_last = mP_last.colRange(0, 3).rowRange(0, 3);
			cv::Mat mt_last = mP_last.rowRange(0, 3).col(3);
			cv::Mat pincamera(3, 1, CV_64F);
			pincamera = mR_last * worldpos_refined + mt_last;
			cout << "last z:"<<"j:"<<j<<" " << pincamera.at<double>(2, 0) << endl;
			if(pincamera.at<double>(2, 0) < 0)
			{
				minusflag = true;
				cout << "break" << endl;
				break; 
			}
		}
		if(minusflag == true)
		{
			continue;
		}

		if(flag_refined == true)
		{
			cout << "refinement is successful" << "\t" << norm1 << "\t" << norm2 << endl;
		}
		else
		{
			cout << "refinement is unsuccessful" << "\t" << norm1 << "\t" << norm2  << endl;
		}
		MarkerPoints3D rec_pt3D;
		rec_pt3D.ID = vmarkerspID[i].ID;
		rec_pt3D.maxobsnum = count_real_good;
		rec_pt3D.pos[0] = worldpos_refined.at<double>(0, 0);
		rec_pt3D.pos[1] = worldpos_refined.at<double>(1, 0);
		rec_pt3D.pos[2] = worldpos_refined.at<double>(2, 0);  	
		vgood3DPointshehe.push_back(rec_pt3D);
	}

	
    }
    mvcurrentmarkers = vgood3DPointshehe;
}

cv::Mat VINS::Triangulation(const std::vector<cv::Mat> vPoses, const std::vector<cv::Mat> vobs, cv::Mat P3w, cv::Mat K, int niterate, bool& which,float &norm1,float &norm2)
{
    if(vPoses.size() < 2)
    {
        which = false;
        norm1 = 0;
        norm2 = 0;
        return P3w.clone();
    }
    cv::Mat matA(2 * vPoses.size(),3,CV_64F);
    cv::Mat matb(2 * vPoses.size(),1,CV_64F);
    std::vector<cv::Mat> vJacobian2;
    std::vector<cv::Mat> vKt;
    cv::Mat P3wIni = P3w.clone();
    for(int i = 0; i < vPoses.size(); i++)
    {
        cv::Mat Pose = vPoses[i];
        cv::Mat R = Pose.rowRange(0,3).colRange(0,3);
        cv::Mat t = Pose.rowRange(0,3).col(3);
        cv::Mat Jacobian2 = K * R;
        cv::Mat Kt = K * t;
        vJacobian2.push_back(Jacobian2.clone());
        vKt.push_back(Kt.clone());
    }
    int countiterate = 0;
    cv::Mat resini;
    while(countiterate < niterate + 1)
    {
        countiterate++;
        for(int i = 0; i < vJacobian2.size(); i++)
        {
            cv::Mat Jacobian2 = vJacobian2[i].clone();
            cv::Mat Kt = vKt[i].clone();
            cv::Mat P3c = Jacobian2 * P3w + Kt;
            cv::Mat Jacobian1 = cv::Mat::zeros(2,3,CV_64F);
            double Xbar = P3c.at<double>(0,0);
            double Ybar = P3c.at<double>(1,0);
            double Zbar = P3c.at<double>(2,0);
            Jacobian1.at<double>(0,0) = 1.0/Zbar;
            Jacobian1.at<double>(0,2) = -Xbar/(Zbar * Zbar);
            Jacobian1.at<double>(1,1) = 1.0/Zbar;
            Jacobian1.at<double>(1,2) = -Ybar/(Zbar * Zbar);
            cv::Mat Jacobian = Jacobian1 * Jacobian2;
            Jacobian.copyTo(matA.rowRange(2 * i, 2 * i + 2).colRange(0,3));
            double ubar = Xbar/Zbar;
            double vbar = Ybar/Zbar;
            cv::Mat obsbar(2,1,CV_64F);
            obsbar.at<double>(0,0) = ubar;
            obsbar.at<double>(1,0) = vbar;
            cv::Mat res = vobs[i] - obsbar;
            res.copyTo(matb.rowRange(2 * i, 2 * i + 2));
        }
        cv::Mat mDeltaX(3,1,CV_64F);
        cv::solve(matA,matb,mDeltaX,cv::DECOMP_SVD);
        if(countiterate < niterate + 1)
        {
            P3w = P3w + mDeltaX;
            //cout << matA << endl;
            //cout << matb << endl;
            //cout << mDeltaX << endl;
            //cout << P3w <<endl;
        }

        if(countiterate == 1)
            resini = matb.clone();

    }
    norm1 = cv::norm(resini);
    norm2 = cv::norm(matb);
    if(cv::norm(matb) < cv::norm(resini))
    {
        which = true;
        return P3w.clone();
    } else{
        which = false;
        return P3wIni.clone();
    }


}

void VINS::new2old()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);

    Vector3d origin_P0 = Ps[0];

    if (is_failure)
    {
            //printf("failure recover %lf %lf %lf\n", last_P.x(), last_P.y(), last_P.z());
            origin_R0 = Utility::R2ypr(last_R_old);
            origin_P0 = last_P_old;
            is_failure=false;
    }

    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());

    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        //if ((!LOOP_CLOSURE) || (!loop_enable))
        if(true)
        {
            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);
            if(i==0){
            /*FILE* fp=fopen("/storage/emulated/0/Android/data/com.example.root.mainloop/3Dtrack.txt","a");
            fprintf(fp,"%lf\t%lf\t%lf\n",Ps[0].x(),Ps[0].y(),Ps[0].z());
            fclose(fp);*/
            }
        }
        else
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
            Vs[i] = Vector3d(para_SpeedBias[i][0],
                             para_SpeedBias[i][1],
                             para_SpeedBias[i][2]);
        }

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }
    Vector3d cur_P0 = Ps[0];

    if(LOOP_CLOSURE && loop_enable)
    {
        loop_enable = false;
        for(int i = 0; i< WINDOW_SIZE; i++)
        {
            if(front_pose.header == Headers[i])
            {
                Matrix3d Rs_loop = Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                Vector3d Ps_loop = Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);

                Rs_loop = rot_diff * Rs_loop;
                Ps_loop = rot_diff * (Ps_loop - Vector3d(para_Pose[0][0], para_Pose[0][1], para_Pose[0][2])) + origin_P0;

                double drift_yaw = Utility::R2ypr(front_pose.Q_old.toRotationMatrix()).x() - Utility::R2ypr(Rs_loop).x();
                r_drift = Utility::ypr2R(Vector3d(drift_yaw, 0, 0));
                //r_drift = front_pose.Q_old.toRotationMatrix() * Rs_loop.transpose();
                t_drift = front_pose.P_old - r_drift * Ps_loop;
            }
        }
    }


    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }
    dt2 = para_dt[0];
    myawio = para_yaw[0];
    cout << myawio << endl;
    VectorXd dep = f_manager->getDepthVector();
    for (int i = 0; i < f_manager->getFeatureCount(); i++)
    {
        dep(i) = para_Feature[i][0];
    }
    f_manager->setDepth(dep);
}

bool VINS::failureDetection()
{
   /* if (f_manager->last_track_num < 2)
    {
        return true;
    }*/
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        return true;
    }
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        return true;
    }
    return false;
}

void VINS::failureRecover()
{
    int his_index = 0;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(Headers_his[i] == Headers[0])
        {
            his_index = i;
            break;
        }
        if(i == WINDOW_SIZE -1)
            his_index = i;
    }
    Vector3d his_R0 = Utility::R2ypr(Rs_his[his_index]);

    Vector3d his_P0 = Ps_his[his_index];

    Vector3d cur_R0 = Utility::R2ypr(Rs[0]);
    Vector3d cur_P0 = Ps[0];

    double y_diff = his_R0.x() - cur_R0.x();

    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Rs[i] = rot_diff * Rs[i];
        Ps[i] = rot_diff * (Ps[i] - cur_P0) + his_P0;
        Vs[i] = rot_diff * Vs[i];
    }
}

void VINS::update_loop_correction()
{
    //update loop correct pointcloud
    correct_point_cloud.clear();
    for (auto &it_per_id : f_manager->feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 4 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (/*it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||*/ it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d tmp = r_drift * Rs[imu_i] * (ric * pts_i + tic) + r_drift * Ps[imu_i] + t_drift;
        correct_point_cloud.push_back(tmp.cast<float>());
    }
    //update correct pose
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        Vector3d correct_p = r_drift * Ps[i] + t_drift;
        correct_Ps[i] = correct_p.cast<float>();
        Matrix3d correct_r = r_drift * Rs[i];
        correct_Rs[i] = correct_r.cast<float>();
    }
}

void VINS::processIMU(double t, double dt, double encoder_v , const Vector3d &linear_acceleration, const Vector3d &angular_velocity, const Vector3d &gps_position, const Vector3d &gps_altitude)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        encoder_v0 = encoder_v;
        enh_0 = gps_position;
        ypr_0 = gps_altitude;
        t0 = t;
    }

    if (!pre_integrations[frame_count])
    {
        Eigen::Matrix3d tmp_Ryawio;
        tmp_Ryawio << cos(myawio), -sin(myawio), 0,
                sin(myawio), cos(myawio), 0,
                0, 0, 1;
        Eigen::Matrix3d recent_rio = tmp_Ryawio * config_rio;
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Bas[frame_count], Bgs[frame_count],recent_rio};
        if(frame_count == WINDOW_SIZE - 1 && !preintegrations_lasttwo)
        {
            preintegrations_lasttwo = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Bas[frame_count], Bgs[frame_count],recent_rio};
        }
    }

    if (frame_count != 0)
    {
        //covariance propagate
        if(pre_integrations[frame_count]->dt_buf.size() == 0)
        {
            pre_integrations[frame_count]->vins_dt = dt2;
        }
        pre_integrations[frame_count]->push_back(dt, encoder_v, linear_acceleration, angular_velocity);
        if(frame_count >= WINDOW_SIZE - 1)
        {
            preintegrations_lasttwo->push_back(dt, encoder_v, linear_acceleration, angular_velocity);
        }

        if(solver_flag != NON_LINEAR) //comments because of recovering
        tmp_pre_integration->push_back(dt, encoder_v, linear_acceleration, angular_velocity);
        output_pre_integration->push_back(dt, encoder_v, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        encoder_v_buf[frame_count].push_back(encoder_v);

        //midpoint integration
        {
            Vector3d g{0,0,GRAVITY};
            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
            //cout << "dt:" << dt << " a:" <<un_acc.transpose() << " v:" << Vs[j].transpose() << " un_gyr:" << un_gyr.transpose() << endl;
            w_norm = un_gyr.norm();
//            ProjectionFactor::sqrt_info =(un_gyr.norm() > 0.4?0.0:1)*FOCUS_LENGTH_X / 1.5 * Matrix2d::Identity();
//            IMUFactor::p =(un_gyr.norm() > 0.4?0.0:1);
        }


    }


    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    encoder_v0 = encoder_v;
    enh_0 = gps_position;
    ypr_0 = gps_altitude;
    //cout << setprecision(13) << t << "\t" << encoder_v << endl;

    if(dt_buf[frame_count].size() == 2)
    {
        Vector3d* linearized_accs = new Vector3d [frame_count + 1];
        for(int i = 1 ; i <= frame_count; i++)
        {
            linearized_accs[i] = pre_integrations[i]->linearized_acc;
        }
        double sum_diff_accs = 0;
        for(int i = 1 ; i < frame_count; i++)
        {
            Vector3d diff_accs =  linearized_accs[i + 1] - linearized_accs[i];
            sum_diff_accs += diff_accs.norm();
        }
        mean_diff_accs = sum_diff_accs / (double)(frame_count - 1);
    }
    ///////////////////////quaternions///////////////
    cumulate_quaternion = cumulate_quaternion * Utility::deltaQ(angular_velocity * dt);
    imu_slide_quaternions.push(Utility::deltaQ(angular_velocity * dt));
    imu_t.push(t);
    imu_angular_velocities.push(angular_velocity);

    if(((2*abs(atan2(cumulate_quaternion.vec().norm(),cumulate_quaternion.w()))*180.0/3.14 > 10 && 2*abs(atan2(cumulate_quaternion.vec().norm(),cumulate_quaternion.w()))*180.0/3.14 < 50 && mean_diff_accs > 0.7)||(mean_diff_accs > 1.4)) && big_turning == false && imu_written == false && count_keyframe > (1.5 * WINDOW_SIZE)/*(t - t0)>12*/)
    {
        int sum_features = 0;
        for(int i = 0; i < WINDOW_SIZE; i++)
        {
            sum_features += nFeatures[i];
        }
        if(sum_features > 25 * WINDOW_SIZE)
        {

            if(mean_diff_accs > 1.4)
            {
                mean_diff_accs = 0.8;
            }
            big_turning = true;
            vsolve_visiononly = true;
            imu_endheader = Headers[WINDOW_SIZE];
        }
    }
    if(big_turning == true)
    {
        if(t > imu_endheader + 0.05)
        {

            std::queue<double> tmp_imu_t = imu_t;
            std::queue<Vector3d> tmp_imu_angular_velocities = imu_angular_velocities;
            cimu_angular_velocities.clear();
            cimu_t.clear();
            while(!tmp_imu_angular_velocities.empty())
            {
                cimu_angular_velocities.push_back(tmp_imu_angular_velocities.front());
                cimu_t.push_back(tmp_imu_t.front());
                tmp_imu_angular_velocities.pop();
                tmp_imu_t.pop();
            }
            big_turning = false;
            imu_written = true;
            /*FILE* fp = fopen("/storage/emulated/0/Android/data/com.example.root.mainloop/files/imu1.csv","w");
            std::queue<double> tmp_imu_t = imu_t;
            std::queue<Vector3d> tmp_imu_angular_velocities = imu_angular_velocities;
            while(!tmp_imu_t.empty())
            {
                double write_t = tmp_imu_t.front();
                Vector3d write_angular_velocity = tmp_imu_angular_velocities.front();
                tmp_imu_t.pop();
                tmp_imu_angular_velocities.pop();
                fprintf(fp,"%lf,",write_t);
                fprintf(fp,"%lf,%lf,%lf\n",write_angular_velocity.x(),write_angular_velocity.y(),write_angular_velocity.z());

            }
            fclose(fp);
            big_turning = false;
            imu_written = true;*/
        }
    }


}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")
int VINS::decideImuLink()
{
    if(frame_count == WINDOW_SIZE)
    {
             double time_tonow = 0;
             int index_frame = 1;
             for(int i = frame_count; i > 0; i--)
             {
                 time_tonow += pre_integrations[i]->sum_dt;
                 if(time_tonow > 1.5)
                 {
                     index_frame = i + 1;
                     break;
                 }
             }
             if(index_frame > frame_count)
             {
                 index_frame = frame_count;
             }
             Matrix3d Rs0 = Rs[index_frame - 1];
             Vector3d Ps0 = Ps[index_frame - 1];
             Vector3d Vs0 = Vs[index_frame - 1];
             Vector3d acc_0 = pre_integrations[index_frame]->linearized_acc;
             Vector3d gyr_0 = pre_integrations[index_frame]->linearized_gyr;
             for(int i = index_frame; i <= frame_count; i++)
             {
                  for(int k = 0; k < pre_integrations[i]->acc_buf.size(); k++)
                  {
                       Vector3d linear_acceleration = pre_integrations[i]->acc_buf[k];
                       Vector3d angular_velocity = pre_integrations[i]->gyr_buf[k];
                       double dt = pre_integrations[i]->dt_buf[k];
                       Vector3d g{0,0,GRAVITY};
                       Vector3d un_acc_0 = Rs0 * (acc_0 - Bas[index_frame - 1]) - g;
                       Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[index_frame - 1];
                       Rs0 *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
                       Vector3d un_acc_1 = Rs0 * (linear_acceleration - Bas[index_frame - 1]) - g;
                       Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
                       Ps0 += dt * Vs0 + 0.5 * dt * dt * un_acc;
                       Vs0 += dt * un_acc;
                       acc_0 = linear_acceleration;
                       gyr_0 = angular_velocity;
                  }
             }

             if(Vs0.norm()>2.5)
             {
                return 1;
             }
             Vector3d tmp_P = Ps[frame_count];
             if((Ps0 - tmp_P).norm() > 1)
             {
                  Rs[frame_count] = Rs0;
                  Ps[frame_count] = Ps0;
                  Vs[frame_count] = Vs0;
                  return frame_count - index_frame + 1;
             }
             if(abs(Ps0.z() - tmp_P.z()) > 0.5)
             {
                  Rs[frame_count] = Rs0;
                  Ps[frame_count] = Ps0;
                  Vs[frame_count] = Vs0;
                  return frame_count - index_frame + 1;
             }

             Matrix3d tmp_R = Rs[frame_count];
             Matrix3d delta_R = tmp_R.transpose() * Rs0;
             Quaterniond delta_Q(delta_R);
             double delta_angle;
             delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
             if (delta_angle > 20)
             {
                  Rs[frame_count] = Rs0;
                  Ps[frame_count] = Ps0;
                  Vs[frame_count] = Vs0;
                  return frame_count - index_frame + 1;
             }
             return 2;
        }
        else
        {
             return 1;
        }
}

void VINS::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double header, int buf_num, cv::Mat rot12, Eigen::Vector3d xyz, Eigen::Vector3d ypr, vector<MarkerPerFrame> vmarkerspf)
{
    //printf("adding feature points %lu\n", image_msg.size());
    int track_num;
    int solved_pts = 0;
    int feature_num = 0;
    if (f_manager->addFeatureCheckParallax(frame_count, image, dt2, track_num,(solver_flag == NON_LINEAR),solved_pts,feature_num))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

	cout<<"vmarkerspf size:"<<vmarkerspf.size()<<endl;
    for(int i = 0; i < vmarkerspf.size(); i++)
    {
		cout<<"vmarker:"<<vmarkerspf[i].pt[0] << "  " << vmarkerspf[i].pt[1] << endl;
		bool findflag = false;
		for(int j = 0; j < vmarkerspID.size(); j++)
		{
			if((sqrt((vmarkerspID[j].pt_last.pt[0] - vmarkerspf[i].pt[0]) * (vmarkerspID[j].pt_last.pt[0] - vmarkerspf[i].pt[0]) + (vmarkerspID[j].pt_last.pt[1] - vmarkerspf[i].pt[1]) * (vmarkerspID[j].pt_last.pt[1] - vmarkerspf[i].pt[1])) < 100) && (vmarkerspID[j].pt_last.frame_idx < frame_count))
			{
				vmarkerspf[i].frame_idx = frame_count;
				vmarkerspID[j].vmarkerspf.push_back(vmarkerspf[i]);
				vmarkerspID[j].pt_last = vmarkerspf[i];
				findflag = true;
				break;
			}
			//else{
				
			//}
		}
		if(findflag == false)
		{
			cout << "minimum distance more than 100" << endl;
			MarkerPerID markerpID;
			markerpID.ID = nmarkerID;
			vmarkerspf[i].frame_idx = frame_count;
			markerpID.vmarkerspf.push_back(vmarkerspf[i]);
			markerpID.pt_last = vmarkerspf[i];
			vmarkerspID.push_back(markerpID);
			nmarkerID++;
		}
	}


//    printf("marginalization_flag %d\n", int(marginalization_flag));
//    printf("this frame is-------------------------------%s\n", marginalization_flag ? "reject" : "accept");
//    printf("Solving %d\n", frame_count);
   // printf("number of feature: %d\n", feature_num = f_manager->getFeatureCount());


//    printf("calibrating extrinsic param, rotation movement is needed\n");
//    if (frame_count != 0)
//    {
//        vector<pair<Vector3d, Vector3d>> corres = f_manager->getCorresponding(frame_count - 1, frame_count);
//        Matrix3d calib_ric;
//        if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
//        {
//            printf("initial extrinsic rotation calib success\n");
////            cout<<"initial extrinsic rotation: " << endl << calib_ric;
//            ric = calib_ric;
//            //RIC[0] = calib_ric;
//            cout << "ric:"<< ric << endl;
//        }
//    }

    nFeatures[frame_count] = feature_num;
    Headers[frame_count] = header;

    if(solved_pts >= 5)
        Qualities[frame_count] = true;
    else
        Qualities[frame_count] = false;
    ImuLinks[frame_count] = decideImuLink();
    ImageFrame imageframe(image, header, rot12.clone());
    //cout << setprecision(16) << "header:" << "\t" << header << endl;
    //cout << rot12 << endl;
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));

    {
        Eigen::Matrix3d tmp_Ryawio;
        tmp_Ryawio << cos(myawio), -sin(myawio), 0,
                sin(myawio), cos(myawio), 0,
                0, 0, 1;
        Eigen::Matrix3d recent_rio = tmp_Ryawio * config_rio;
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Vector3d(0,0,0), Vector3d(0,0,0),recent_rio};
        if(output_pre_integration != nullptr)
        {
            //FILE* fpodometry = fopen(mswriteodometrypath.c_str(),"a");
            //double normodometry = sqrt(output_pre_integration->delta_encoder_p[0]*output_pre_integration->delta_encoder_p[0]+output_pre_integration->delta_encoder_p[1]*output_pre_integration->delta_encoder_p[1]+output_pre_integration->delta_encoder_p[2]*output_pre_integration->delta_encoder_p[2]);
            //fprintf(fpodometry,"%lf\t%lf\n",header,normodometry);
            //fclose(fpodometry);
            delete output_pre_integration;
            output_pre_integration = nullptr;
        }


        output_pre_integration = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Vector3d(0,0,0), Vector3d(0,0,0),recent_rio};
    }


    if(solver_flag == INITIAL)
    {


        if (frame_count == WINDOW_SIZE)
        {
            /*if(track_num < 20)
            {
                clearState();
                return;
            }*/
            bool result = false;
            if(header - initial_timestamp > 0.1)
            {
                //result = solveInitial();
                //result = solveInitialWithOdometry();
                result = solveInitialWithOdometry2();
                initial_timestamp = header;
            }
            if(result)
            {
                solve_ceres(buf_num);
                /*if(final_cost>200)
                {
                    delete last_marginalization_info;
                    last_marginalization_info = nullptr;
                    solver_flag = INITIAL;
                    init_status = FAIL_CHECK;
                    fail_times++;
                    slideWindow();
                }
                else*/
                {
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    f_manager->removeFailures();
                    last_R = Rs[WINDOW_SIZE];
                    last_P = Ps[WINDOW_SIZE];
                    last_R_old = Rs[0];
                    last_P_old = Ps[0];
                    init_status = SUCC;
                    fail_times = 0;
                    update_loop_correction();
                }

            }
            else
            {
                Matrix3d pre_cumu_Rnew;
                pre_cumu_Rnew = pre_cumu_R * imageframe.pre_integration->delta_q.toRotationMatrix();
                pre_cumu_P = pre_cumu_P + pre_cumu_R * ( imageframe.pre_integration->delta_encoder_p + config_tio) - pre_cumu_Rnew * config_tio;
                pre_cumu_R = pre_cumu_Rnew;
                rec_header = header;
                pair<double,pair<Matrix3d, Vector3d>> pos_before_initialize;
                pos_before_initialize.first = rec_header;
                pos_before_initialize.second.first = pre_cumu_R;
                pos_before_initialize.second.second = pre_cumu_P;
                vpos_before_initialize.push_back(pos_before_initialize);
                slideWindow();
            }
        }
        else
        {
            if(imageframe.pre_integration != NULL)
            {
                Matrix3d pre_cumu_Rnew;
                pre_cumu_Rnew = pre_cumu_R * imageframe.pre_integration->delta_q.toRotationMatrix();
                pre_cumu_P = pre_cumu_P + pre_cumu_R * ( imageframe.pre_integration->delta_encoder_p + config_tio) - pre_cumu_Rnew * config_tio;
                pre_cumu_R = pre_cumu_Rnew;
            }
            rec_header = header;
            pair<double,pair<Matrix3d, Vector3d>> pos_before_initialize;
            pos_before_initialize.first = rec_header;
            pos_before_initialize.second.first = pre_cumu_R;
            pos_before_initialize.second.second = pre_cumu_P;
            vpos_before_initialize.push_back(pos_before_initialize);
            frame_count++;
        }

    }
    else
    {
        bool is_nonlinear = true;
        f_manager->triangulate(Ps, tic, ric, is_nonlinear);
        //if(Qualities[frame_count] == false)
        //    solve_pnp();
        solve_ceres(buf_num);

        if(failureDetection())
        {
            is_failure=true;
            clearState();
            return;
        }
        slideWindow();
        f_manager->removeFailures();
        update_loop_correction();

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R_old = Rs[0];
        last_P_old = Ps[0];
    }
}

void VINS::solve_pnp()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        if(i < WINDOW_SIZE)
        {
            problem.SetParameterBlockConstant(para_Pose[i]);
            problem.SetParameterBlockConstant(para_SpeedBias[i]);
        }
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    for (int i = 0; i < NUM_OF_F; i++)
    {
        problem.AddParameterBlock(para_Feature[i], SIZE_FEATURE);
        problem.SetParameterBlockConstant(para_Feature[i]);
    }
    old2new();
    //IMU factor
    IMUFactor* imu_factor = new IMUFactor(config_rio, config_tio, preintegrations_lasttwo);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[WINDOW_SIZE - 2], para_SpeedBias[WINDOW_SIZE - 2], para_Pose[WINDOW_SIZE], para_SpeedBias[WINDOW_SIZE]);

    //SpeedPrior factor
    if(Qualities[WINDOW_SIZE] == false)
    {
        SpeedPriorFactor* speed_factor = new SpeedPriorFactor(average_speed);
        problem.AddResidualBlock(speed_factor,NULL,para_SpeedBias[WINDOW_SIZE]);
    }

    //projection factor
    int feature_index = -1;
    for (auto &it_per_id : f_manager->feature)
    {
        if(it_per_id.solve_flag != 1)
            continue;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            if (imu_i == imu_j || imu_j != WINDOW_SIZE)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;

            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
        }
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 4;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    //options.max_num_iterations = 6;
    //options.max_solver_time_in_seconds = 0.08;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    final_cost = summary.final_cost;
    cout<< "cost:" << final_cost << " "<< "time:"<< summary.total_time_in_seconds <<endl;

    new2old();

    vector<ceres::ResidualBlockId> residual_set;
    problem.GetResidualBlocks(&residual_set);
    for (auto it : residual_set)
        problem.RemoveResidualBlock(it);
}


void VINS::solve_ceres(int buf_num)
{
    //optimize_num++;
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        problem.SetParameterBlockConstant(para_Ex_Pose[i]);

    }

    {
        problem.AddParameterBlock(para_dt, 1);
        problem.SetParameterBlockConstant(para_dt);

        problem.AddParameterBlock(para_yaw, 1);
        //if(optimize_num < 50)
        problem.SetParameterBlockConstant(para_yaw);
    }
    for (int i = 0; i < NUM_OF_F; i++)
    {
        problem.AddParameterBlock(para_Feature[i], SIZE_FEATURE);
    }


    old2new();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(config_rio, config_tio, pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j],para_yaw);
    }

    /*for (int i = 0; i < WINDOW_SIZE; i++) {
        int j = i + 1;
        EncoderDisplacementFactor* encoder_factor = new EncoderDisplacementFactor(config_rio,config_tio,pre_integrations[j]);
        problem.AddResidualBlock(encoder_factor, NULL, para_Pose[i],para_Pose[j],para_SpeedBias[i]);
    }*/
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager->feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;

            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_dt);
                    /*
                    double **para = new double *[5];
                    para[0] = para_Pose[imu_i];
                    para[1] = para_Pose[imu_j];
                    para[2] = para_Ex_Pose[0];
                    para[3] = para_Feature[feature_index];
                    para[4] = para_Td[0];
                    f_td->check(para);
                    */

            }

            f_m_cnt++;
        }
    }

    if(LOOP_CLOSURE)
        {
            //loop close factor
            //front_pose.measurements.clear();
            if(front_pose.header != retrive_pose_data.header)
            {
                front_pose = retrive_pose_data;  //need lock
                printf("use loop\n");
            }
            if(!front_pose.measurements.empty())
            {
                //the retrive pose is in the current window
                if(front_pose.header >= Headers[0])
                {
                    //tmp_retrive_pose_buf.push(front_pose);
                    printf("loop front pose  in window\n");
                    for(int i = 0; i < WINDOW_SIZE; i++)
                    {
                        if(front_pose.header == Headers[i])
                        {
                            //for (int k = 0; k < 7; k++)
                              //  front_pose.loop_pose[k] = para_Pose[i][k];
				vector<Point2d> loop_all_2dpts;
				vector<Point3d> loop_all_3dpts;
			    int retrive_feature_index = 0;
			    for (auto &it_per_id : f_manager->feature)
                            {
                                it_per_id.used_num = it_per_id.feature_per_frame.size();
                                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                    continue;
				int start = it_per_id.start_frame;
                                //feature has been obeserved in ith frame
                                int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
				if(start <= i && end >=0)
                                {
					while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                    {
                                        retrive_feature_index++;
                                    }

                                    if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                    {
                                        Point2d pts_j;
					pts_j.x = front_pose.measurements[retrive_feature_index].x;
					pts_j.y = front_pose.measurements[retrive_feature_index].y;	
					//Vector3d pts_j = Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
					Vector3d vpts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
					int loop_imu_i = it_per_id.start_frame;
					Quaterniond loop_Qi(para_Pose[loop_imu_i][6], para_Pose[loop_imu_i][3], para_Pose[loop_imu_i][4], para_Pose[loop_imu_i][5]);
					Vector3d loop_Pi(para_Pose[loop_imu_i][0], para_Pose[loop_imu_i][1], para_Pose[loop_imu_i][2]);	
					Matrix3d loop_Ri = loop_Qi.toRotationMatrix();				
					Vector3d loop_3Dpt = loop_Ri * (ric * vpts_i + tic) + loop_Pi;
					Point3d pts_3d;
					pts_3d.x = loop_3Dpt[0];  pts_3d.y = loop_3Dpt[1]; pts_3d.z = loop_3Dpt[2]; 
					loop_all_2dpts.push_back(pts_j);
					loop_all_3dpts.push_back(pts_3d);
				    }
				}
			    }
			    cv::Mat loop_rvec(3, 1, CV_64F);
			    cv::Mat loop_tvec(3, 1, CV_64F);		
			    cv::solvePnPRansac(loop_all_3dpts, loop_all_2dpts, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), loop_rvec, loop_tvec, false, 100, 0.007, 0.95, cv::noArray() );
			    cv::Mat loop_solvedRcw(3, 3, CV_64F);
			    cv::Rodrigues(loop_rvec, loop_solvedRcw);
			    cv::Mat solvedRwc(3, 3, CV_64F);
			    solvedRwc =	loop_solvedRcw.t();
			    cv::Mat solvedtwc(3, 1, CV_64F);
			    solvedtwc = -loop_solvedRcw.t() * loop_tvec;
			    cv::Mat cvric; cvric = Utility::toCvMatd(ric);	
			    cv::Mat cvtic; cvtic = Utility::toCvMatd(tic);
			    cv::Mat solvedRwi(3, 3, CV_64F);
			    solvedRwi = solvedRwc * cvric.t();
			    cv::Mat solvedtwi(3, 1, CV_64F);
			    solvedtwi = solvedtwc - solvedRwi * cvtic;
			    Matrix3d solvedRwi_eigen;
			    Vector3d solvedtwi_eigen;
			    for(int row = 0; row < 3; row++)
			    {
				for(int col = 0; col < 3; col++)
				{
					solvedRwi_eigen(row, col) = solvedRwi.at<double>(row, col);
				}
				solvedtwi_eigen(row, 0) = solvedtwi.at<double>(row, 0);
			    }
			    Quaterniond solvedQ{solvedRwi_eigen};
			    front_pose.loop_pose[0] = solvedtwi_eigen[0];		
			    front_pose.loop_pose[1] = solvedtwi_eigen[1];
			    front_pose.loop_pose[2] = solvedtwi_eigen[2];
			    front_pose.loop_pose[3] = solvedQ.x();
			    front_pose.loop_pose[4] = solvedQ.y();
			    front_pose.loop_pose[5] = solvedQ.z(); 
			    front_pose.loop_pose[6] = solvedQ.w(); 		 	 	
		 					
                            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                            problem.AddParameterBlock(front_pose.loop_pose, SIZE_POSE, local_parameterization);

                            Ps_retrive = front_pose.P_old;
                            Qs_retrive = front_pose.Q_old.toRotationMatrix();
                            //int retrive_feature_index = 0;
			    retrive_feature_index = 0;
                            int feature_index = -1;
                            int loop_factor_cnt = 0;
                            for (auto &it_per_id : f_manager->feature)
                            {
                                it_per_id.used_num = it_per_id.feature_per_frame.size();
                                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                                    continue;
                                ++feature_index;
                                int start = it_per_id.start_frame;
                                //feature has been obeserved in ith frame
                                int end = (int)(start + it_per_id.feature_per_frame.size() - i - 1);
                                if(start <= i && end >=0)
                                {
                                    while(front_pose.features_ids[retrive_feature_index] < it_per_id.feature_id)
                                    {
                                        retrive_feature_index++;
                                    }

                                    if(front_pose.features_ids[retrive_feature_index] == it_per_id.feature_id)
                                    {
                                        Vector3d pts_j = Vector3d(front_pose.measurements[retrive_feature_index].x, front_pose.measurements[retrive_feature_index].y, 1.0);
                                        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                        //double ratio = 1.0;
                                        //LoopClosureFactor *f = new LoopClosureFactor(pts_i, pts_j, Ps_retrive, Qs_retrive, ratio);
                                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                        problem.AddResidualBlock(f, loss_function, para_Pose[start], front_pose.loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);

                                        //printf("loop add factor %d %d %lf %lf %d\n",retrive_feature_index,feature_index,
                                        //                                         pts_j.x(), pts_i.x(),front_pose.features_ids.size());
                                        retrive_feature_index++;
                                        loop_factor_cnt++;
                                        loop_enable = true;
                                    }

                                }
                            }
                            printf("add %d loop factor\n", loop_factor_cnt);
                        }
                    }
                }
            }
        }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 8;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 6;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    //options.use_nonmonotonic_steps = true;
    if(buf_num<4)
           options.max_solver_time_in_seconds = 0.085;
	else
           options.max_solver_time_in_seconds = 0.07;
       //else if(buf_num<12)
       //    options.max_solver_time_in_seconds = 0.11;
       //else
       //    options.max_solver_time_in_seconds = 0.09;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << "final cost:" << summary.final_cost << endl;

    if(LOOP_CLOSURE)
        {
            for(int i = 0; i< WINDOW_SIZE; i++)
            {
                if(front_pose.header == Headers[i])
                {
                    Matrix3d Rs_i = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                    Vector3d Ps_i = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
                    Matrix3d Rs_loop = Quaterniond(front_pose.loop_pose[6],  front_pose.loop_pose[3],  front_pose.loop_pose[4],  front_pose.loop_pose[5]).normalized().toRotationMatrix();
                    Vector3d Ps_loop = Vector3d( front_pose.loop_pose[0],  front_pose.loop_pose[1],  front_pose.loop_pose[2]);

                    front_pose.relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                    front_pose.relative_q = Rs_loop.transpose() * Rs_i;
                    front_pose.relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                }
            }
        }
    new2old();

    solve_marker();

    vector<ceres::ResidualBlockId> residual_set;
    problem.GetResidualBlocks(&residual_set);
    for (auto it : residual_set)
        problem.RemoveResidualBlock(it);
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        old2new();

        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(config_rio, config_tio, pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1],para_yaw},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        /*{
            EncoderDisplacementFactor* encoder_factor = new EncoderDisplacementFactor(config_rio,config_tio,pre_integrations[1]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(encoder_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_Pose[1], para_SpeedBias[0]},
                                                                           vector<int>{0, 2});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }*/

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager->feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_dt},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        marginalization_info->preMarginalize();
        marginalization_info->marginalize();

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        addr_shift[reinterpret_cast<long>(para_dt)] = para_dt;
        addr_shift[reinterpret_cast<long>(para_yaw)] = para_yaw;

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            old2new();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    //ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }


            marginalization_info->preMarginalize();


            marginalization_info->marginalize();

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_dt)] = para_dt;
            addr_shift[reinterpret_cast<long>(para_yaw)] = para_yaw;

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }

}

bool VINS::solveInitialWithOdometry2() {
    printf("solve initial------------------------------------------\n");
    double sum_dis = 0;
double min_dis = 10000;
    for(int i = 0; i < 10; i++)
{
	sum_dis += pre_integrations[i]->delta_encoder_p.norm();
if( pre_integrations[i]->delta_encoder_p.norm() < min_dis)
{
	min_dis = pre_integrations[i]->delta_encoder_p.norm();
}
}
    

    //if (!relativePosewithOdometry(0))
    //if(false)
    if(sum_dis < 3.5 || min_dis < 0.15)
    {
        printf("init solve 5pts between first frame and last frame failed\n");
        return false;
    }
cout << "sum_dis:" << endl;
    Quaterniond *Q = new Quaterniond[all_image_frame.size()];
    Vector3d *T = new Vector3d[all_image_frame.size()];

    Quaterniond *Q2 = new Quaterniond[frame_count + 1];
    Vector3d* T2 = new Vector3d[frame_count + 1];

    GetInitialPosesFromOdometry2(Q, T, Q2, T2);

    map<double, ImageFrame>::iterator frame_it;
    frame_it = all_image_frame.begin();
    int idxhehe = 0;
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        if((frame_it->first) == Headers[i])
        {
            //cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[idxhehe].toRotationMatrix();
            frame_it->second.T = T[idxhehe] + Q[idxhehe].toRotationMatrix() * tic;
            i++;
            idxhehe++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            //frame_it->second.is_key_frame = false;
            i++;
        }
        frame_it->second.is_key_frame = false;
        frame_it->second.R = Q[idxhehe].toRotationMatrix();
        frame_it->second.T = T[idxhehe] + Q[idxhehe].toRotationMatrix() * tic;
        idxhehe++;
    }

    Vector3d g_odo(0, 0, G_NORM);
    g = config_rio * g_odo;

    delete[] Q;
    delete[] T;
    delete[] Q2;
    delete[] T2;


    {
        cout << "before calibrate Bgs" << endl;
        map<double, ImageFrame>::iterator frame_it2;
        frame_it2 = all_image_frame.begin();
        frame_it2++;
        double sum_error = 0;
        for(; frame_it2 != all_image_frame.end(); frame_it2++)
        {
            Eigen::Quaterniond q_ij(ric * frame_it2->second.eigenrot12 * ric.transpose());
            sum_error += ((frame_it2->second.pre_integration->delta_q.inverse() * q_ij).vec().norm()) * ((frame_it2->second.pre_integration->delta_q.inverse() * q_ij).vec().norm());
        }
        cout << sqrt(sum_error) << endl;
    }

    if (visualInitialAlignOdometry())
    {
        {
            cout << "after calibrate Bgs" << endl;
            map<double, ImageFrame>::iterator frame_it2;
            frame_it2 = all_image_frame.begin();
            frame_it2++;
            double sum_error = 0;
            for(; frame_it2 != all_image_frame.end(); frame_it2++)
            {
                Eigen::Quaterniond q_ij(ric * frame_it2->second.eigenrot12 * ric.transpose());
                sum_error += ((frame_it2->second.pre_integration->delta_q.inverse() * q_ij).vec().norm()) * ((frame_it2->second.pre_integration->delta_q.inverse() * q_ij).vec().norm());
            }
            cout << sqrt(sum_error) << endl;
        }

        return true;
    }
    else
    {
        init_status = FAIL_ALIGN;
        fail_times++;
        return false;
    }
}

//#pragma GCC pop_options
bool VINS::solveInitialWithOdometry()
{
    printf("solve initial------------------------------------------\n");
    printf("PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g = {0,0,0};
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            if(dt == 0)
            {
                printf("dt == 0!\n");
                init_status = FAIL_IMU;
                fail_times++;
                return false;
            }
            //cout << "delta_v"<< frame_it->second.pre_integration->delta_v << endl;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            //cout << "tmp_g" << tmp_g<<endl;
            sum_g += tmp_g;
            //cout << "sum_g" << sum_g<<endl;

        }

        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        cout << "aver_g " << aver_g.transpose() << endl;
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //  cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        printf("IMU variation %f!\n", var);
        if(var < 0.20)
        {
            printf("init IMU variation not enough!\n");
            init_status = FAIL_IMU;
            fail_times++;
            return false;
        }
    }
    // global sfm
    Quaterniond *Q = new Quaterniond[frame_count + 1];
    Vector3d *T = new Vector3d[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    {
        for (auto &it_per_id : f_manager->feature)
        {
            int imu_j = it_per_id.start_frame - 1;

            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Vector3d pts_j = it_per_frame.point;
                tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f.push_back(tmp_feature);
        }
        /////////////////////add code here using odemotry to initialize////////////////////////////
        if (!relativePosewithOdometry(0))
        {
            printf("init solve 5pts between first frame and last frame failed\n");
            return false;
        }
        GetInitialPosesFromOdometry(Q, T);
        GlobalSFM sfm;
        if(!sfm.constructodometry(frame_count + 1, Q, T,
                          sfm_f, sfm_tracked_points))
        {
            printf("global SFM failed!");
            init_status = FAIL_SFM;
            marginalization_flag = MARGIN_OLD;
            fail_times++;
            return false;
        }
        ///////////////////////////////////////////////////////////////////////////////////////////
    }

    cout << "initial camera poses:" << endl;
    for(int i = 0; i < (frame_count + 1); i++)
    {
        cout << Q[i].w() << "\t" << Q[i].x() << "\t" << Q[i].y() << "\t" << Q[i].z() << "\t";
        cout << T[i][0] << "\t" << T[i][1] << "\t" << T[i][2] << endl;
    }
    //solve pnp for all frame
    cout << "all initial camera poses:" << endl;
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * ric.transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
            it = sfm_tracked_points.find(feature_id);
            if(it != sfm_tracked_points.end())
            {
                Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);

                Vector2d img_pts = id_pts.second[0].second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                0, 1, 0,
                0, 0, 1);

        if(pts_3_vector.size() < 6 )
        {
            printf("init Not enough points for solve pnp !\n");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            printf("init solve pnp fail!\n");
            init_status = FAIL_PNP;
            fail_times++;
            return false;
        }
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);

        T_pnp = R_pnp * (-T_pnp);
        Matrix3d R_hehe;
        R_hehe << R_pnp(0, 0), R_pnp(0, 1), R_pnp(0, 2),
                R_pnp(1, 0), R_pnp(1, 1), R_pnp(1, 2),
                R_pnp(2, 0), R_pnp(2, 1), R_pnp(2, 2);
        Quaterniond tmp_Q_pnp{R_hehe};
        cout << tmp_Q_pnp.w() << "\t" << tmp_Q_pnp.x() << "\t" << tmp_Q_pnp.y() << "\t" << tmp_Q_pnp.z() << "\t";
        cout << T_pnp(0, 0) << "\t" << T_pnp(1, 0) << "\t" << T_pnp(2, 0) << endl;
        frame_it->second.R = R_pnp * ric.transpose();  //q^v_c * q^c_i = q^v_i
        frame_it->second.T = T_pnp;
    }
    delete[] Q;
    delete[] T;

    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());

    if (visualInitialAlign())
    {
        return true;
    }
    else
    {
        init_status = FAIL_ALIGN;
        fail_times++;
        return false;
    }
}

void VINS::GetInitialPosesFromOdometry(Quaterniond* q, Vector3d* T)
{
    q[0].w() = 1; q[0].x() = 0; q[0].y() = 0; q[0].z() = 0;
    T[0].setZero();
    Quaterniond q_total;
    q_total.setIdentity();
    Vector3d* dT = new Vector3d[frame_count + 1];
    dT[0].setZero();
    for(int i = 1; i < (frame_count + 1); i++)
    {
        q_total = pre_integrations[i]->delta_q.inverse() * q_total;
        q[i] = ric.transpose() * q_total * ric;
        dT[i] = pre_integrations[i]->delta_encoder_p + config_tio - (ric * q[i - 1] * q[i].inverse() * ric.transpose()) * config_tio;
        dT[i] = ric.transpose() * dT[i] - ric.transpose() * tic + (q[i - 1] * q[i].inverse() * ric.transpose()) * tic;
        T[i] = T[i - 1] + q[i - 1].inverse() * dT[i];
        cout << "q:" << q[i].w() << "\t" << q[i].x() << "\t" << q[i].y() << "\t" << q[i].z() << endl;
        cout << "T:" << T[i].x() << "\t" << T[i].y() << "\t" << T[i].z() << endl;
        //T[i] = -(q[i] * T[i]);
    }
    for(int i = 0; i < (frame_count + 1); i++)
    {
        T[i] = -(q[i] * T[i]);
    }
}

void VINS::GetInitialPosesFromOdometry2(Quaterniond* q, Vector3d* T, Quaterniond* q2, Vector3d* T2) {
    {
        q[0].w() = 1; q[0].x() = 0; q[0].y() = 0; q[0].z() = 0;
        T[0].setZero();
        Quaterniond q_total;
        q_total.setIdentity();
        Vector3d *dT = new Vector3d[all_image_frame.size()];
        dT[0].setZero();
        map<double, ImageFrame>::iterator frame_it;
        frame_it = all_image_frame.begin( );
        frame_it++;
        for (int i = 1; i < all_image_frame.size(); i++) {
            q_total = frame_it->second.pre_integration->delta_q.inverse() * q_total;
            q[i] = q_total.inverse();
            dT[i] = frame_it->second.pre_integration->delta_encoder_p + config_tio - q[i - 1] * q[i].inverse() * config_tio;
            T[i] = T[i - 1] + q[i - 1] * dT[i];
            frame_it++;
        }
    }

    {
        q2[0].w() = 1; q2[0].x() = 0; q2[0].y() = 0; q2[0].z() = 0;
        T2[0].setZero();
        Quaterniond q_total2;
        q_total2.setIdentity();
        Vector3d* dT2 = new Vector3d[frame_count + 1];
        dT2[0].setZero();
        for(int i = 1; i < (frame_count + 1); i++)
        {
            q_total2 = pre_integrations[i]->delta_q.inverse() * q_total2;
            q2[i] = q_total2.inverse();
            dT2[i] = pre_integrations[i]->delta_encoder_p + config_tio - q2[i - 1] * q2[i].inverse() * config_tio;
            T2[i] = T2[i - 1] + q2[i - 1] * dT2[i];
        }
    }


}

bool VINS::solveInitial()
{
    printf("solve initial------------------------------------------\n");
    printf("PS %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g = {0,0,0};
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            if(dt == 0)
            {
                printf("dt == 0!\n");
                init_status = FAIL_IMU;
                fail_times++;
                return false;
            }
            //cout << "delta_v"<< frame_it->second.pre_integration->delta_v << endl;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            //cout << "tmp_g" << tmp_g<<endl;
            sum_g += tmp_g;
            //cout << "sum_g" << sum_g<<endl;

        }

        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        cout << "aver_g " << aver_g.transpose() << endl;
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
          //  cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        printf("IMU variation %f!\n", var);
        if(var < 0.20)
        {
            printf("init IMU variation not enough!\n");
            init_status = FAIL_IMU;
            fail_times++;
            return false;
        }
    }
    // global sfm
    Quaterniond *Q = new Quaterniond[frame_count + 1];
    Vector3d *T = new Vector3d[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    {
        for (auto &it_per_id : f_manager->feature)
        {
            int imu_j = it_per_id.start_frame - 1;

            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Vector3d pts_j = it_per_frame.point;
                tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            }
            sfm_f.push_back(tmp_feature);
        }
        Matrix3d relative_R;
        Vector3d relative_T;
        int l;
        if (!relativePose(0, relative_R, relative_T, l))
        {
            printf("init solve 5pts between first frame and last frame failed\n");
            return false;
        }
        GlobalSFM sfm;
        if(!sfm.construct(frame_count + 1, Q, T, l,
                          relative_R, relative_T,
                          sfm_f, sfm_tracked_points))
        {
            printf("global SFM failed!");
            init_status = FAIL_SFM;
            marginalization_flag = MARGIN_OLD;
            fail_times++;
            return false;
        }
    }

    cout << "initial camera poses:" << endl;
    for(int i = 0; i < (frame_count + 1); i++)
    {
        cout << Q[i].w() << "\t" << Q[i].x() << "\t" << Q[i].y() << "\t" << Q[i].z() << "\t";
        cout << T[i][0] << "\t" << T[i][1] << "\t" << T[i][2] << endl;
    }
    //solve pnp for all frame
    cout << "all initial camera poses:" << endl;
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            cout << "key frame " << i << endl;
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * ric.transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
            it = sfm_tracked_points.find(feature_id);
            if(it != sfm_tracked_points.end())
            {
                Vector3d world_pts = it->second;
                cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                pts_3_vector.push_back(pts_3);

                Vector2d img_pts = id_pts.second[0].second.head<2>();
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

        if(pts_3_vector.size() < 6 )
        {
            printf("init Not enough points for solve pnp !\n");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            printf("init solve pnp fail!\n");
            init_status = FAIL_PNP;
            fail_times++;
            return false;
        }
        cv::Rodrigues(rvec, r);
        //cout << "r " << endl << r << endl;
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //cout << "R_pnp " << endl << R_pnp << endl;
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);

        T_pnp = R_pnp * (-T_pnp);
        Matrix3d R_hehe;
        R_hehe << R_pnp(0, 0), R_pnp(0, 1), R_pnp(0, 2),
                R_pnp(1, 0), R_pnp(1, 1), R_pnp(1, 2),
                R_pnp(2, 0), R_pnp(2, 1), R_pnp(2, 2);
        Quaterniond tmp_Q_pnp{R_hehe};
        cout << tmp_Q_pnp.w() << "\t" << tmp_Q_pnp.x() << "\t" << tmp_Q_pnp.y() << "\t" << tmp_Q_pnp.z() << "\t";
        cout << T_pnp(0, 0) << "\t" << T_pnp(1, 0) << "\t" << T_pnp(2, 0) << endl;
        frame_it->second.R = R_pnp * ric.transpose();  //q^v_c * q^c_i = q^v_i
        frame_it->second.T = T_pnp;
    }
    delete[] Q;
    delete[] T;

    printf("init PS after pnp %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());

    if (visualInitialAlign())
    {
        return true;
    }
    else
    {
        init_status = FAIL_ALIGN;
        fail_times++;
        return false;
    }

}

bool VINS::visualInitialAlign()
{
    TS(solve_g);
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        printf("solve g failed!");
        printf("init PS alignment failed %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
        return false;
    }
    TE(solve_g);
    printf("init PS algnment succ:Ps %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    printf("init PS algnment succ:Vs %lf %lf %lf\n", Vs[0].x(),Vs[0].y(), Vs[0].z());

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    VectorXd dep = f_manager->getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager->clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP;
    TIC_TMP.setZero();
    f_manager->triangulate(Ps, TIC_TMP, ric, true);

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * tic - (s * Ps[0] - Rs[0] * tic);

    printf("PS after scale %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    printf("VS after scale %lf %lf %lf\n", Vs[0].x(),Vs[0].y(), Vs[0].z());

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    printf("init finish--------------------\n");

    for (auto &it_per_id : f_manager->feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw0 = Utility::R2ypr(R0* Rs[0]).x();
    //    double yaw0 = Utility::R2ypr(R0).x();

    Matrix3d yaw_refine = Utility::ypr2R(Vector3d{-yaw0,0,0});
    R0 = yaw_refine * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        init_poses.push_back(Ps[i]);
    }

    return true;
}

bool VINS::visualInitialAlignOdometry()
{
    TS(solve_g);
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignmentOdometry(all_image_frame, Bgs, g, x, config_tio, config_ric);

    if(!result)
    {
        printf("solve g failed!");
        printf("init PS alignment failed %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
        return false;
    }
    TE(solve_g);
    printf("init PS algnment succ:Ps %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    printf("init PS algnment succ:Vs %lf %lf %lf\n", Vs[0].x(),Vs[0].y(), Vs[0].z());

    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;//camera position
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    VectorXd dep = f_manager->getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager->clearDepth(dep);

    Vector3d TIC_TMP;
    TIC_TMP.setZero();
    f_manager->triangulate(Ps, TIC_TMP, ric, true);

    //double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = Ps[i] - Rs[i] * tic - (Ps[0] - Rs[0] * tic);

    printf("PS after scale %lf %lf %lf\n", Ps[0].x(),Ps[0].y(), Ps[0].z());
    printf("VS after scale %lf %lf %lf\n", Vs[0].x(),Vs[0].y(), Vs[0].z());

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    printf("init finish--------------------\n");

    for (auto &it_per_id : f_manager->feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        //if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        //    continue;
        //it_per_id.estimated_depth *= s;
    }

    cout << "g:" << "\t" << g << endl;

    Matrix3d R0 = Utility::g2R(g);
    double yaw0 = Utility::R2ypr(R0* Rs[0]).x();
    //    double yaw0 = Utility::R2ypr(R0).x();

    Matrix3d yaw_refine = Utility::ypr2R(Vector3d{-yaw0,0,0});
    R0 = yaw_refine * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        init_poses.push_back(Ps[i]);
    }

    Vector3d disdiff_local;
    disdiff_local = Ps[frame_count - 1] - Ps[0];
    Vector3d disdiff_gps;
    disdiff_gps = pre_integrations[frame_count]->enh_0 - pre_integrations[1]->enh_0;
    disdiff_local[2] = 0;
    disdiff_gps[2] = 0;
    double norm_local = sqrt( disdiff_local[0] * disdiff_local[0] + disdiff_local[1] * disdiff_local[1]);
    double norm_gps = sqrt( disdiff_gps[0]* disdiff_gps[0] + disdiff_gps[1] * disdiff_gps[1]);
    if(norm_gps > 0.00001)
{
    disdiff_local[0] /= norm_local;
    disdiff_local[1] /= norm_local;
    disdiff_gps[0] /= norm_gps;
    disdiff_gps[1] /= norm_gps;

    Matrix3d Rgl = Eigen::Quaterniond::FromTwoVectors(disdiff_local, disdiff_gps).toRotationMatrix();
    cout << "transformed local:" << Rgl * (Ps[frame_count -1] - Ps[0]) << endl;
    cout << "gps diff: "<< pre_integrations[frame_count]->enh_0 - pre_integrations[1]->enh_0 << endl;
    for(int i = 0; i <= frame_count; i++)
    {
        Ps[i] = Rgl * Ps[i];
        Rs[i] = Rgl * Rs[i];
        Vs[i] = Rgl * Vs[i];
    }
}

 for(int i = 0; i <= frame_count; i++)
    {
         cout << "ps:" << Ps[i] << endl;
         cout << "rs:" << Rs[i] << endl;
         cout << "vs:" << Vs[i] << endl;
    }
    rec_enh[0] = pre_integrations[1]->enh_0[0];
    rec_enh[1] = pre_integrations[1]->enh_0[1];
    rec_enh[2] = pre_integrations[1]->enh_0[2];

    //FILE* fp1 = fopen("/home/nvidia/initialgps.txt", "w");
//for(int i = 1; i <= frame_count; i++)
//{
//    fprintf(fp1, "%f\t%f\t%f\t%f\n", Headers[i - 1], pre_integrations[i]->enh_0[0], pre_integrations[i]->enh_0[1], pre_integrations[i]->enh_0[2]);

//}
//fclose(fp1);
    /*for(int j = 0; j < vpos_before_initialize.size(); j++)
    {
        if(vpos_before_initialize[j].first == Headers[0])
        {
            Matrix3d tmp_ROri = vpos_before_initialize[j].second.first;
            Vector3d tmp_POri = vpos_before_initialize[j].second.second;
            double tmp_yaw_diff = Utility::R2ypr(tmp_ROri).x() - Utility::R2ypr(Rs[0]).x();
            Matrix3d tmp_Ryaw_diff = Utility::ypr2R(Vector3d{tmp_yaw_diff, 0, 0});
            for(int i = 0; i <= frame_count; i++)
            {
                Ps[i] = tmp_Ryaw_diff * Ps[i];
                Rs[i] = tmp_Ryaw_diff * Rs[i];
                Vs[i] = tmp_Ryaw_diff * Vs[i];
            }
            for(int i = 0; i <= frame_count; i++)
            {
                Ps[i] = Ps[i] + tmp_POri;
            }
            break;
        }
    }*/
    return true;
}

bool VINS::relativePose(int camera_id, Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager->getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            parallax_num_view = average_parallax * 520;
            if(average_parallax * 520 < 30)
            {
                init_status = FAIL_PARALLAX;
                fail_times++;
                return false;
            }
            if(m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                printf("average_parallax %f choose l %d and newest frame to triangulate the whole structure\n", average_parallax * 520, l);
                return true;
            }
            else
            {
                init_status = FAIL_RELATIVE;
                fail_times++;
                return false;
            }
        }
    }
    return false;
}

bool VINS::relativePosewithOdometry(int camera_id) {
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager->getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            parallax_num_view = average_parallax * 520;
            if(average_parallax * 520 < 30)
            {
                init_status = FAIL_PARALLAX;
                fail_times++;
                return false;
            }
            if(m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                printf("average_parallax %f choose l %d and newest frame to triangulate the whole structure\n", average_parallax * 520, l);
                return true;
            }
            else
            {
                init_status = FAIL_RELATIVE;
                fail_times++;
                return false;
            }
        }
    }
    return false;
}
bool VINS::relativePose2(int camera_id, Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager->getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            parallax_num_view = average_parallax * 520;
            if(average_parallax * 520 < 30)
            {
                return false;
            }
            if(m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}
/*
 marginalize the state from the sliding window and change feature start frame
 */
void VINS::slideWindow()
{
    //marginalize old keyframe
    if (marginalization_flag == MARGIN_OLD)
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            sum_dt += pre_integrations[1]->sum_dt;
            sum_dx += sqrt((Ps[1].x()-Ps[0].x())*(Ps[1].x()-Ps[0].x())+(Ps[1].y()-Ps[0].y())*(Ps[1].y()-Ps[0].y())+(Ps[1].z()-Ps[0].z())*(Ps[1].z()-Ps[0].z()));
            if(sum_dt > 0)
            {
                average_speed = sum_dx/sum_dt;
            }
            if(preintegrations_lasttwo != NULL)
            delete preintegrations_lasttwo;
            Eigen::Matrix3d tmp_Ryawio;
            tmp_Ryawio << cos(myawio), -sin(myawio), 0,
                    sin(myawio), cos(myawio), 0,
                    0, 0, 1;
            Eigen::Matrix3d recent_rio = tmp_Ryawio * config_rio;
            tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Vector3d(0,0,0), Vector3d(0,0,0),recent_rio};
            preintegrations_lasttwo = new IntegrationBase{pre_integrations[frame_count]->linearized_acc, pre_integrations[frame_count]->linearized_gyr, pre_integrations[frame_count]->linearized_encoder_v, enh_0, ypr_0, Bas[frame_count - 1], Bgs[frame_count - 1], recent_rio};
            for(int i = 0; i < pre_integrations[frame_count]->acc_buf.size(); i++ )
            {
                Vector3d acc_now = pre_integrations[frame_count]->acc_buf[i];
                Vector3d gyr_now = pre_integrations[frame_count]->gyr_buf[i];
                double dt_now = pre_integrations[frame_count]->dt_buf[i];
                double encoder_v_now = pre_integrations[frame_count]->encoder_v_buf[i];
                preintegrations_lasttwo->push_back(dt_now, encoder_v_now, acc_now, gyr_now);
            }
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);
                std::swap(pre_integrations[i], pre_integrations[i + 1]);
                dt_buf[i].swap(dt_buf[i + 1]);
                encoder_v_buf[i].swap(encoder_v_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                Headers[i] = Headers[i + 1];
                Qualities[i] = Qualities[i + 1];
                ImuLinks[i] = ImuLinks[i + 1];
                nFeatures[i] = nFeatures[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Qualities[WINDOW_SIZE] = Qualities[WINDOW_SIZE - 1];
            ImuLinks[WINDOW_SIZE] = ImuLinks[WINDOW_SIZE - 1];
            nFeatures[WINDOW_SIZE] = nFeatures[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            
            vector<MarkerPerID>::iterator Iter;
            for(Iter = vmarkerspID.begin(); Iter != vmarkerspID.end(); )
            {
				(*Iter).pt_last.frame_idx -= 1;
				for(int j = 0; j < (*Iter).vmarkerspf.size(); j++)
				{
					(*Iter).vmarkerspf[j].frame_idx -= 1;
				}
				if((*Iter).vmarkerspf.front().frame_idx < 0)
				{
					(*Iter).vmarkerspf.erase((*Iter).vmarkerspf.begin());
				}
				if((*Iter).vmarkerspf.size() == 0)
				{
					Iter = vmarkerspID.erase(Iter);
				}
				else
				{
					Iter++;
				}
			}

            if(pre_integrations[WINDOW_SIZE] != NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE],recent_rio};

            dt_buf[WINDOW_SIZE].clear();
            encoder_v_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (solver_flag == INITIAL)
            {
                double t_0 = Headers[0];
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }

            while(imu_t.front() < (Headers[0] - 0.05) && big_turning == false)
            {
                  Eigen::Quaterniond front_imu = imu_slide_quaternions.front();
                  imu_slide_quaternions.pop();
                  imu_t.pop();
                  imu_angular_velocities.pop();
                  cumulate_quaternion = front_imu.inverse() * cumulate_quaternion;
            }
            slideWindowOld();
            if(solver_flag == NON_LINEAR)
            {
                count_keyframe++;
            }
        }
    }
    else  //non keyframe
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                double tmp_encoder_v = encoder_v_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_encoder_v, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                encoder_v_buf[frame_count - 1].push_back(tmp_encoder_v);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Qualities[frame_count - 1] = Qualities[frame_count];
            ImuLinks[frame_count - 1] = ImuLinks[frame_count];
            nFeatures[frame_count - 1] = nFeatures[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            if(pre_integrations[WINDOW_SIZE]!=NULL)
            {
                delete pre_integrations[WINDOW_SIZE];
            }
            
            vector<MarkerPerID>::iterator Iter;
            for(Iter = vmarkerspID.begin(); Iter != vmarkerspID.end(); )
            {
				for(int j = 0; j < (*Iter).vmarkerspf.size(); j++)
				{
					if((*Iter).vmarkerspf[j].frame_idx == (frame_count - 1))
					{
						(*Iter).vmarkerspf.erase((*Iter).vmarkerspf.begin() + j);
					}
					if((*Iter).vmarkerspf[j].frame_idx == frame_count)
					{
						(*Iter).vmarkerspf[j].frame_idx -= 1;
					}
				}
				if((*Iter).vmarkerspf.size() == 0)
				{
					Iter = vmarkerspID.erase(Iter);
					continue;
				}
				if((*Iter).pt_last.frame_idx == (frame_count - 1))
				{
					(*Iter).pt_last = (*Iter).vmarkerspf.back();
				}
				if((*Iter).pt_last.frame_idx == frame_count)
				{
					(*Iter).pt_last.frame_idx -= 1;
				}
				Iter++;
			}
            
            Eigen::Matrix3d tmp_Ryawio;
            tmp_Ryawio << cos(myawio), -sin(myawio), 0,
                    sin(myawio), cos(myawio), 0,
                    0, 0, 1;
            Eigen::Matrix3d recent_rio = tmp_Ryawio * config_rio;
            tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Vector3d(0,0,0), Vector3d(0,0,0),recent_rio};
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, encoder_v0, enh_0, ypr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE], recent_rio};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            encoder_v_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

void VINS::slideWindowOld()
{
    //printf("marginalize back\n");
    point_cloud.clear();
    for (auto &it_per_id : f_manager->feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2
            &&it_per_id.solve_flag == 1 )
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d tmp = Rs[imu_i] * (ric * pts_i + tic) + Ps[imu_i];
            point_cloud.push_back(tmp.cast<float>());
        }
    }
    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric;
        R1 = Rs[0] * ric;
        P0 = back_P0 + back_R0 * tic;
        P1 = Ps[0] + Rs[0] * tic;
        f_manager->removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager->removeBack();

}
void VINS::slideWindowNew()
{
    //printf("marginalize front\n");
    f_manager->removeFront(frame_count);
}
/*void VINS::PrepareForVisionOnlyBA()
{
    csfm_f.clear();
    {
        for (auto &it_per_id : f_manager->feature)
        {
             int imu_j = it_per_id.start_frame - 1;

             SFMFeature tmp_feature;
             tmp_feature.state = false;
             tmp_feature.id = it_per_id.feature_id;
             for (auto &it_per_frame : it_per_id.feature_per_frame)
             {
                  imu_j++;
                  Vector3d pts_j = it_per_frame.point;
                  tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
             }
             csfm_f.push_back(tmp_feature);
        }
    }

    if (!relativePose2(0, crelative_R, crelative_T, cl))
    {
         imu_written = false;
         return;
    }
    else
    {
        csolve_visiononly = true;
    }
}*/

void VINS::PrepareForVisionOnlyBA()
{
    csfm_f.clear();
    {
        for (auto &it_per_id : f_manager->feature)
        {
             int imu_j = it_per_id.start_frame - 1;

             SFMFeature tmp_feature;
             tmp_feature.state = false;
             tmp_feature.id = it_per_id.feature_id;
             for (auto &it_per_frame : it_per_id.feature_per_frame)
             {
                  imu_j++;
                  Vector3d pts_j = it_per_frame.point;
                  tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
             }
             csfm_f.push_back(tmp_feature);
        }
    }
    for(int i = 0; i <= WINDOW_SIZE; i++)
    {
        cPs[i] = cPs[i] + cQs[i].toRotationMatrix() * tic;
        Quaterniond qic{ric};
        Quaterniond cQ_camera = cQs[i] * qic;
        cQs[i] = cQ_camera.inverse();
        cPs[i] = -cQs[i].toRotationMatrix() * cPs[i];
    }
    csolve_visiononly = true;
}

vector<MarkerPerFrame> VINS::getptsonsecondlastframe()
{
	vector<MarkerPerFrame> vtmpmarkerspf;
	vector<MarkerPerID>::iterator Iter;
	for(Iter = vmarkerspID.begin(); Iter != vmarkerspID.end(); )
        {
		for(int j = 0; j < (*Iter).vmarkerspf.size(); j++)
		{
			if((*Iter).vmarkerspf[j].frame_idx == (WINDOW_SIZE - 1))
			{
				vtmpmarkerspf.push_back((*Iter).vmarkerspf[j]);	
			}
		}
	}
	return vtmpmarkerspf;
}