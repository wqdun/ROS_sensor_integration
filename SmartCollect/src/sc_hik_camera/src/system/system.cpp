//
// Created by diner on 17-7-19.
//

#include <pangolin/display/display.h>
#include <pangolin/var/var.h>
#include <pangolin/pangolin.h>
#include <global_param.hpp>
#include "system.h"


//静态变量必须在类外初始化，否则编译器报错，找不到引用

double IntegrationBase::a_n = 0;
double IntegrationBase::a_w = 0;
double IntegrationBase::g_n = 0;
double IntegrationBase::g_w = 0;
double IntegrationBase::v_n = 0;
bool vinssystem::stopflag = false;

vinssystem::vinssystem()
{

}


vinssystem::~vinssystem() {

    measurement_process->join();
    //synchronization_thread->join();
    loop_detection->join();
    global_loop_detection->join();
    delete mpEstimator;
    delete measurement_process;
    //delete synchronization_thread;
    delete mpFeaturetracker;

//    if (loop_detection != NULL) delete loop_detection;
}

void vinssystem::setstopflag()
{
   stopflag = true;
}

void vinssystem::create(string voc_file, string pattern_file,string setting_file)
{
    start_global_optimization=false;
   //yaw_now = 0;
   //pitch_now = 0;
   //roll_now = 0;
   //dt_initial = 0;
   //synchronizing_flag = false;
   //synchronized_flag = false;
    dx = dy = dz = rx = ry = rz = 0;
    px = py = pz = gps_yaw = gps_pitch = gps_roll = 0;
    rencoder_v = 0;

    mVocFile = voc_file;

    mPatternFile = pattern_file;

    mSettingFile = setting_file;
    g_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(mSettingFile);

    fSettings = cv::FileStorage(mSettingFile, cv::FileStorage::READ);
    cv::FileNode projectionSettings;

    projectionSettings = fSettings["projection_parameters"];
    //string sdatafolder;
    fSettings["datafolder"] >> msdatafolder;

    projectionSettings["fx"] >> FOCUS_LENGTH_X;
    projectionSettings["fy"] >> FOCUS_LENGTH_Y;
    projectionSettings["cx"] >> PX;
    projectionSettings["cy"] >> PY;

    Vector3d g_imu;
    Vector3d toi;
    cv::Mat cv_g_imu, cv_toi;
    fSettings["GravityinIMU"] >> cv_g_imu;
    fSettings["IMUvehicleTranslation"] >> cv_toi;
    cv::cv2eigen(cv_g_imu, g_imu);
    cv::cv2eigen(cv_toi, toi);

    Vector3d ng_car(0, 0, 1);
    Vector3d ng_imu = g_imu.normalized();
    Matrix3d Rio_tmp = Quaterniond::FromTwoVectors(ng_car, ng_imu).toRotationMatrix();
    double yaw_tmp = Utility::R2ypr(Rio_tmp).x();
    Rio = Utility::ypr2R(Eigen::Vector3d{-yaw_tmp, 0, 0}) * Rio_tmp;
    tio = (-1) * Rio * toi;

    cout << "Rio:" << Rio << endl;
    cout << "tio:" << tio << endl;

    mpEstimator = new VINS(mSettingFile,Rio,tio);
    //mpViewer = new Viewer2(this);


    measurement_process = new thread(&vinssystem::process,this);
    //synchronization_thread = new thread(&vinssystem::solve_visiononly,this);
    th_display = new thread(&vinssystem::Run,this);

    //mpViewer = new Viewer(mSettingFile);

    //mptViewer = new thread(&Viewer::Run, mpViewer);

    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(mSettingFile);

    fSettings["image_width"] >> imgWidth;

    fSettings["image_height"] >> imgHeight;

    fSettings["max_cnt"] >> max_cnt;

    fSettings["min_dist"] >> min_dis;



    mpFeaturetracker = new FeatureTracker(m_camera,imgWidth,imgHeight,max_cnt,min_dis,FOCUS_LENGTH_X,FOCUS_LENGTH_Y,PX,PY);

    fSettings["acc_n"] >> a_n;
    fSettings["acc_w"] >> a_w;
    fSettings["gyr_n"] >> g_n;
    fSettings["gyr_w"] >> g_w;
    fSettings["env_n"] >> v_n;

    mpDrawer = new MapDrawer(setting_file);


    IntegrationBase::setIMUNoise(a_n,a_w,g_n,g_w,v_n);

    if (LOOP_CLOSURE)
    {
        printf("LOOP_CLOSURE true");
        loop_detection =  new thread(&vinssystem::process_loop_detection,this);
        global_loop_detection = new thread(&vinssystem::process_global_loop_detection,this);
    }

    cv::Mat Twc = Mat::eye(4, 4, CV_32F);

    float fps = 20;
    mT = 1e3/fps;
}

void vinssystem::updateView() {

    s.str("");s1.str("");s2.str("");s3.str("");

    if (mpEstimator->solver_flag == VINS::INITIAL)
    {
        {
            Vector3d P_real;
            P_real = mpEstimator->r_drift * (mpEstimator->pre_cumu_P + mpEstimator->pre_cumu_R * mpEstimator->tic) + mpEstimator->t_drift;
            /*Matrix3d*/ //ric_double = ric.cast<double>();
            Matrix3d R_real;
            R_real = (mpEstimator->r_drift * mpEstimator->pre_cumu_R)*mpEstimator->ric;

            Quaterniond Qs{mpEstimator->pre_cumu_R * mpEstimator->ric};
            DRAWFRAME_DATA temp_frame;
            temp_frame.P_origin = mpEstimator->pre_cumu_P + mpEstimator->pre_cumu_R * mpEstimator->tic;
            temp_frame.P_draw = P_real;
            temp_frame.header = mpEstimator->rec_header;
            temp_frame.iskeyframeflag = false;
            temp_frame.Q_origin = Qs;
            temp_frame.dt = mpEstimator->dt2;
            temp_frame.R_draw = R_real;
            temp_frame.ric = mpEstimator->ric;
            temp_frame.tic = mpEstimator->tic;
            keyframe_database.m_draw.lock();
            keyframe_database.frames_to_draw.push_back(temp_frame);
            for(int i = keyframe_database.frames_to_draw.size() - 1; i >= 0; i--)
            {
                if((mpEstimator->frame_count == WINDOW_SIZE) && (mpEstimator->Headers[WINDOW_SIZE - 2] == keyframe_database.frames_to_draw[i].header))
                {
                    keyframe_database.frames_to_draw[i].iskeyframeflag = true;
                    break;
                }
            }
            mpDrawer->SetAllFrames(keyframe_database.frames_to_draw);
            keyframe_database.m_draw.unlock();

            m_points3d.lock();
            points_3d.clear();
            mpDrawer->SetAllPoints(points_3d);
            m_points3d.unlock();
            s << "ypr: wait for initialization" ;  //需要使用mutex
            s1 << "v: wait for initialization" ;  //需要使用mutex
            s2 << "P:" << setprecision(3)
               << P_real.x() << " "
               << P_real.y() << " "
               << P_real.z() << " ";  //需要使用mutex
            s3 << "cost: wait for initialization";
        }

        return;
    }

    Matrix3f ric;
    Vector3f tic;

    cv::Mat cv_R, cv_T;
    fSettings["extrinsicRotation"] >> cv_R;
    fSettings["extrinsicTranslation"] >> cv_T;
    cv::cv2eigen(cv_R, ric);
    cv::cv2eigen(cv_T, tic);

    cv::Mat Rwc,twc,Twc;

    Matrix3f Rwc_eigen = (mpEstimator->correct_Rs[WINDOW_SIZE-1] * ric); //q^w_c = q^w_i * q^i_c

    Vector3f twc_eigen = (mpEstimator->correct_Ps[WINDOW_SIZE-1] + mpEstimator->correct_Rs[WINDOW_SIZE-1] * tic); //p^w_{wc}  =  p^w_{wi} +q^w_ip^i_{ci}

    Matrix3d R;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            R(i, j) = Rwc_eigen(i, j);
    }

    Vector3d ypr =  Utility::R2ypr(R);

    cv::eigen2cv(Rwc_eigen, Rwc);
    cv::eigen2cv(twc_eigen,twc);
    Twc = Mat::eye(4, 4, CV_32F);
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));

    Vector3d P_real;
    P_real = mpEstimator->r_drift * (mpEstimator->Ps[WINDOW_SIZE-1] + mpEstimator->Rs[WINDOW_SIZE - 1] * mpEstimator->tic) + mpEstimator->t_drift;
    /*Matrix3d*/ ric_double = ric.cast<double>();
    Matrix3d R_real;
    R_real = (mpEstimator->r_drift * mpEstimator->Rs[WINDOW_SIZE-1])*mpEstimator->ric;

    Quaterniond Qs{mpEstimator->Rs[WINDOW_SIZE-1] * mpEstimator->ric};
    DRAWFRAME_DATA temp_frame;
    temp_frame.P_origin = mpEstimator->Ps[WINDOW_SIZE-1] + mpEstimator->Rs[WINDOW_SIZE - 1] * mpEstimator->tic;
    temp_frame.P_draw = P_real;
    temp_frame.header = mpEstimator->Headers[WINDOW_SIZE-1] - 0.00;
    temp_frame.iskeyframeflag = false;
    temp_frame.Q_origin = Qs;
    temp_frame.dt = mpEstimator->dt2;
    temp_frame.R_draw = R_real;
    temp_frame.ric = mpEstimator->ric;
    temp_frame.tic = mpEstimator->tic;
    keyframe_database.m_draw.lock();
    keyframe_database.frames_to_draw.push_back(temp_frame);
    for(int i = keyframe_database.frames_to_draw.size() - 1; i >= 0; i--)
    {
        if(mpEstimator->Headers[WINDOW_SIZE - 2] == keyframe_database.frames_to_draw[i].header)
        {
            keyframe_database.frames_to_draw[i].iskeyframeflag = true;
            break;
        }
    }
    mpDrawer->SetAllFrames(keyframe_database.frames_to_draw);
    keyframe_database.m_draw.unlock();


    Vector3d eular_angles = Utility::R2ypr((mpEstimator->r_drift * mpEstimator->Rs[WINDOW_SIZE-1])*ric_double);
    Vector3f eular_angles_float;
    eular_angles_float = eular_angles.cast<float>();
    //yaw_now = eular_angles_float.x();
    //pitch_now = eular_angles_float.y();
    //roll_now = eular_angles_float.z();

    //----------------------------3D points-------------------
    Vector3d tic_double = tic.cast<double>();
    for(int i = 0; i < WINDOW_SIZE ; i++)
    {
        corrected_Rs[i] = mpEstimator->r_drift * mpEstimator->Rs[i];
        corrected_Ps[i] = mpEstimator->r_drift * mpEstimator->Ps[i] + mpEstimator->t_drift;
    }
    m_points3d.lock();
    points_3d.clear();
    for (auto &it_per_id : mpEstimator->f_manager->feature)
    {
       it_per_id.used_num = it_per_id.feature_per_frame.size();
       if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
           continue;
       if (it_per_id.start_frame + it_per_id.used_num - 1 < WINDOW_SIZE - 1)
           continue;
       if (it_per_id.solve_flag == 1 )
       {
           int imu_i = it_per_id.start_frame;
           Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
           Vector3d tmp = corrected_Rs[imu_i] * (mpEstimator->ric * pts_i + tic_double) + corrected_Ps[imu_i];
           points_3d.push_back(tmp);
       }
    }
    mpDrawer->SetAllPoints(points_3d);
    m_points3d.unlock();
    //--------------------------------------------------------

    s << "ypr: " << setprecision(3)
      << ypr.x()<< " "
      << ypr.y() << " "
      << ypr.z() << " "
      << "Ba:"
      << mpEstimator->Bas[WINDOW_SIZE-1].x() << " "
      << mpEstimator->Bas[WINDOW_SIZE-1].y() << " "
      << mpEstimator->Bas[WINDOW_SIZE-1].z() << " ";  //需要使用mutex
    s1 << "v: " << setprecision(6)
       << mpEstimator->Vs[WINDOW_SIZE-1].x() << " "
       << mpEstimator->Vs[WINDOW_SIZE-1].y() << " "
       << mpEstimator->Vs[WINDOW_SIZE-1].z() << " "
       << "Bg:"<< setprecision(3)
       << mpEstimator->Bgs[WINDOW_SIZE-1].x() << " "
       << mpEstimator->Bgs[WINDOW_SIZE-1].y() << " "
       << mpEstimator->Bgs[WINDOW_SIZE-1].z() << " ";  //需要使用mutex
    s2 << "P:" << setprecision(12)
       << P_real.x() + mpEstimator->rec_enh[0] << " "
       << P_real.y() + mpEstimator->rec_enh[1] << " "
       << P_real.z() + mpEstimator->rec_enh[2]<< " ";  //需要使用mutex

    s3 << "cost: " << mpEstimator->final_cost;

 //   if (mpEstimator->solver_flag == VINS::NON_LINEAR)
//    mpViewer->addKeyFramePos(Twc);
}

/*void vinssystem::update() {
    //latest_time = lateast_imu_time;
    //tmp_P = mpEstimator->Ps[WINDOW_SIZE];
    //tmp_Q = Eigen::Quaterniond{mpEstimator->Rs[WINDOW_SIZE]};
    //tmp_V = mpEstimator->Vs[WINDOW_SIZE];
    //tmp_Ba = mpEstimator->Bas[WINDOW_SIZE];
    //tmp_Bg = mpEstimator->Bgs[WINDOW_SIZE];
    //acc_0 = mpEstimator->acc_0;
    //gyr_0 = mpEstimator->gyr_0;

    updateView();
}*/


void vinssystem::process() {
    while (true)
    {
//        printf("process...\n");
        std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
        {
//            printf("condition wait...\n");
            return (measurements = getMeasurements()).size() != 0;
        });
        lk.unlock();
//        printf("get measuments already!!\n");
        waiting_lists = measurements.size();
        /*if(synchronizing_flag == true && dt_initial < 0)
        {
            synchronizing_flag = false;
            double last_header = mpEstimator->Headers[WINDOW_SIZE - 1] + dt_initial;
            double current_imu = current_time;
            std::vector<ImuConstPtr> tmpimu_msg_buf;
            for(int i = mpEstimator->pre_integrations[WINDOW_SIZE - 1]->acc_buf.size() - 1; i >= 0; i--)
            {
                if(current_imu < last_header)
                {
                    break;
                }
                double dt = mpEstimator->pre_integrations[WINDOW_SIZE - 1]->dt_buf[i];
                Vector3d acc = mpEstimator->pre_integrations[WINDOW_SIZE - 1]->acc_buf[i];
                Vector3d gyr = mpEstimator->pre_integrations[WINDOW_SIZE - 1]->gyr_buf[i];
                ImuConstPtr imu_msg = new IMU_MSG();
                imu_msg->header = current_imu;
                imu_msg->acc = acc;
                imu_msg->gyr = gyr;
                tmpimu_msg_buf.push_back(imu_msg);
                current_imu -= dt;
            }
            while(!tmpimu_msg_buf.empty())
            {
                ImuConstPtr imu_msg = tmpimu_msg_buf.back();
                send_imu(imu_msg);
                tmpimu_msg_buf.pop_back();
            }
        }
        if(synchronizing_flag == true && dt_initial > 0)
        {
            synchronizing_flag = false;
            std::pair<std::vector<ImuConstPtr>, ImgConstPtr> measurement = measurements[0];
            double last_header = mpEstimator->Headers[WINDOW_SIZE - 1] + dt_initial;
            std::vector<ImuConstPtr> imu_msgs = measurement.first;
            std::vector<ImuConstPtr> new_imu_msgs;
            for(int i = 0; i < imu_msgs.size(); i++)
            {
                ImuConstPtr imu_msg = imu_msgs[i];
                if(imu_msg->header > last_header)
                {
                    new_imu_msgs.push_back(imu_msg);
                }
            }
            measurement.first = new_imu_msgs;
            measurements[0] = measurement;
        }*/
        for (auto &measurement : measurements)
        {
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            auto img_msg = measurement.second;
            for (auto &imu_msg : measurement.first)
                send_imu(imu_msg,img_msg->header);



            //map<int, Vector3d> image = img_msg->point_clouds;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            // printf("IMG:  rocessing vision data with stamp %f \n", img_msg->header);

            double header = img_msg->header;
            for (auto pt_cloud : img_msg->point_clouds)
            {
                int feature_id = pt_cloud.first;
                xyz_uv_velocity = pt_cloud.second;
                image[feature_id].emplace_back(0, xyz_uv_velocity);
            }
            //TS(process_image);
            mpEstimator->processImage(image,header,waiting_lists, img_msg->rot12.clone(), Vector3d(px, py, pz), Vector3d(gps_yaw, gps_pitch, gps_roll));
            //TE(process_image);


            if(mpEstimator->solver_flag == mpEstimator->NON_LINEAR)
            {
                //Vector3d rotation_vins = Utility::R2ypr(vins.Rs[WINDOW_SIZE]);
                //printf("attitude compare\n");
                //printf("attitude vins pitch: %lf, roll: %lf\n", rotation_vins.y(), rotation_vins.z());
                //printf("attitude imu  pitch: %lf, roll: %lf\n", rotation_imu.y(), rotation_imu.z());
            }

            /**
            *** start build keyframe database for loop closure
            **/
            if(LOOP_CLOSURE)
            {
                static bool first_frame = true;
                if(mpEstimator->solver_flag != mpEstimator->NON_LINEAR)
                    first_frame = true;
                if(mpEstimator->marginalization_flag == mpEstimator->MARGIN_OLD && mpEstimator->solver_flag == mpEstimator->NON_LINEAR && !image_buf_loop.empty())
                {
                    first_frame = false;
                    if (!first_frame && keyframe_freq % LOOP_FREQ == 0)
                    {
                        keyframe_freq = 0;
                        /**
                        ** save the newest keyframe to the keyframe database
                        ** only need to save the pose to the keyframe database
                        **/
                        Vector3d T_w_i = mpEstimator->Ps[WINDOW_SIZE - 2];
                        Matrix3d R_w_i = mpEstimator->Rs[WINDOW_SIZE - 2];
                        i_buf.lock();
                        while(!image_buf_loop.empty() && image_buf_loop.front().second < mpEstimator->Headers[WINDOW_SIZE - 2])
                        {
                            image_buf_loop.pop();
                        }
                        i_buf.unlock();
                        //assert(mpEstimator->Headers[WINDOW_SIZE - 1].stamp.toSec() == image_buf.front().second);
                        // relative_T   i-1_T_i relative_R  i-1_R_i


                        if(mpEstimator->Headers[WINDOW_SIZE - 2] == image_buf_loop.front().second)
                        {
                            const char *pattern_file = mPatternFile.c_str();
                            KeyFrame* keyframe = new KeyFrame(mpEstimator->Headers[WINDOW_SIZE - 2], global_frame_cnt, T_w_i, R_w_i, image_buf_loop.front().first, pattern_file, keyframe_database.cur_seg_index);
                            keyframe->setExtrinsic(mpEstimator->tic, mpEstimator->ric);
                            /*
                             ** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
                             ** and save the pointcloud to the keyframe for reprojection search correspondance
                             */
                            keyframe->buildKeyFrameFeatures(*mpEstimator);
                            keyframe_database.add(keyframe);
                            /*erase_index.clear();
                            m_keyframedatabase_resample.lock();

                            keyframe_database.resample(erase_index);
                            m_keyframedatabase_resample.unlock();*/
                            global_frame_cnt++;
                        }

                    }
                    else
                    {
                        first_frame = false;
                    }


                    // update loop info
                    // m_keyframedatabase_resample.lock();
                    for (int i = 0; i < WINDOW_SIZE; i++)
                    {
                        if(mpEstimator->Headers[i]== mpEstimator->front_pose.header)
                        {
                            KeyFrame* cur_kf = keyframe_database.getKeyframe(mpEstimator->front_pose.cur_index);

                            cur_kf->updateLoopConnection( mpEstimator->front_pose.relative_t,
                                                          mpEstimator->front_pose.relative_q,
                                                          mpEstimator->front_pose.relative_yaw);
                            break;
                        }
                    }
                    /*
                    ** update the keyframe pose when this frame slides out the window and optimize loop graph
                    */
                    int search_cnt = 0;

                    for(int i = 0; i < keyframe_database.size(); i++)
                    {
                        search_cnt++;
                        KeyFrame* kf = keyframe_database.getLastKeyframe(i);
                        if(kf->header == mpEstimator->Headers[0])
                        {
                            kf->updateOriginPose(mpEstimator->Ps[0], mpEstimator->Rs[0]);
                            //update edge
                            // if loop happens in this frame, update pose graph;
                            if (kf->has_loop)
                            {
                                kf_global_index = kf->global_index;
                                start_global_optimization = true;
                            }
                            break;
                        }
                        else
                        {
                            if(search_cnt > 2 * WINDOW_SIZE)
                                break;
                        }
                    }
                    keyframe_freq++;
                    // m_keyframedatabase_resample.unlock();
                }
            }
            updateView();
            waiting_lists--;

        }
    }

}

std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> vinssystem::getMeasurements() {

//    printf("get measuments\n");

    //      ------------------------->  t
    //           |``````````|
    //          img  imus  img
    //
    //
    //           (``````````|)   get one image and those imu datas between it and last image
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> measurements;
    while (true)
    {
//        printf("loop for get measuments\n");
        if (imu_msg_buf.empty() || img_msg_buf.empty())
            return measurements;

        //     two queue
        //     imu    front-------------------------------->back
        //     img    front-------------------------------->back


        //     imu    front--------------------->back
        //     img                                      front-------------------->back

        if (!(imu_msg_buf.back()->header > img_msg_buf.front()->header + mpEstimator->dt2))
        {
            printf("wait for imu, only should happen at the beginning\n");
            return measurements;
        }


        //     imu         front------------------------------>back
        //     img    front----------------------------------->back
        //                        ||
        //                        ||
        //                       \  /
        //                        \/
        //     imu    front---------------------------------->back
        //     img     front--------------------------------->back

        if (!(imu_msg_buf.front()->header < img_msg_buf.front()->header + mpEstimator->dt2))
        {
            printf("throw img, only should happen at the beginning\n");
            img_msg_buf.pop();

            continue;
        }

        ImgConstPtr img_msg = img_msg_buf.front();
        img_msg_buf.pop();

        std::vector<ImuConstPtr> IMUs;
        while (imu_msg_buf.front()->header <= img_msg->header + mpEstimator->dt2)
        {
            IMUs.emplace_back(imu_msg_buf.front());
            imu_msg_buf.pop();
        }
        IMUs.emplace_back(imu_msg_buf.front());
        //     imu    front---------------------------------->back
        //     img           front--------------------------->back
        //   extract   ``````````|
//        printf("IMU_buf = %d\n",IMUs.size());
        measurements.emplace_back(IMUs, img_msg);
//
//    std::vector<ImuConstPtr> IMUs;
//    ImgConstPtr img_msg;
//    measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;

}

void vinssystem::send_imu(const ImuConstPtr &imu_msg,double header) {

    double t = imu_msg->header;
    double img_t = header + mpEstimator->dt2;
    if(t <= img_t)
    {
        if (current_time < 0)
            current_time = t;
        double dt = (t - current_time);
        current_time = t;

        dx = imu_msg->acc.x() ;
        dy = imu_msg->acc.y() ;
        dz = imu_msg->acc.z() ;

        rx = imu_msg->gyr.x() ;
        ry = imu_msg->gyr.y() ;
        rz = imu_msg->gyr.z() ;

        px = imu_msg->enh.x();
        py = imu_msg->enh.y();
        pz = imu_msg->enh.z();

        gps_yaw = imu_msg->ypr.x();
        gps_pitch = imu_msg->ypr.y();
        gps_roll = imu_msg->ypr.z();


        rencoder_v = imu_msg->encoder_v;
        mpEstimator->processIMU(t, dt, rencoder_v, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Vector3d(px, py, pz), Vector3d(gps_yaw, gps_pitch, gps_roll));
    }
    else
    {
        double dt_1 = img_t - current_time;
        double dt_2 = t - img_t;
        current_time = img_t;
        double w1 = dt_2 / (dt_1 + dt_2);
        double w2 = dt_1 / (dt_1 + dt_2);
        dx = w1 * dx + w2 * imu_msg->acc.x();
        dy = w1 * dy + w2 * imu_msg->acc.y();
        dz = w1 * dz + w2 * imu_msg->acc.z();
        rx = w1 * rx + w2 * imu_msg->gyr.x();
        ry = w1 * ry + w2 * imu_msg->gyr.y();
        rz = w1 * rz + w2 * imu_msg->gyr.z();
        rencoder_v = w1 * rencoder_v + w2 * imu_msg->encoder_v;

        px = w1 * px + w2 * imu_msg->enh.x();
        py = w1 * py + w2 * imu_msg->enh.y();
        pz = w1 * pz + w2 * imu_msg->enh.z();

        gps_yaw = w1 * gps_yaw + w2 * imu_msg->ypr.x();
        gps_pitch = w1 * gps_pitch + w2 * imu_msg->ypr.y();
        gps_roll = w1 * gps_roll + w2 * imu_msg->ypr.z();

        mpEstimator->processIMU(t, dt_1, rencoder_v, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Vector3d(px, py, pz), Vector3d(gps_yaw, gps_pitch, gps_roll));
    }
    //  printf("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f\n", t, dt, dx, dy, dz, rx, ry, rz);


}

void vinssystem::process_loop_detection() {

    if(loop_closure == NULL)
    {
        const char *voc_file = mVocFile.c_str();
        TS(load_voc);
        printf("loop start loop");
        cout << "voc file: " << voc_file << endl;
        loop_closure = new LoopClosure(voc_file, imgWidth, imgHeight);
        TE(load_voc);
        printf("loop load vocbulary");
        loop_closure->initCameraModel(mSettingFile);
        //voc_init_ok = true;
    }
    double previous_header=-1;
    while(LOOP_CLOSURE)
    {

        bool loop_succ = false;
        if (loop_check_cnt < global_frame_cnt)
        {
            KeyFrame* cur_kf = keyframe_database.getLastKeyframe();
            if(cur_kf->header==previous_header)
            {
                sleep(0.05);
                continue;
            }
            else
            {
                previous_header=cur_kf->header;
            }
            //assert(loop_check_cnt == cur_kf->global_index);
            loop_check_cnt++;
            cur_kf->check_loop = 1;

            cv::Mat current_image;
            current_image = cur_kf->image;

            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;



            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            //if(current_image.rows&&current_image.cols)
            cur_kf->extractBrief(current_image);
            printf("loop extract %d feature\n", cur_kf->keypoints.size());
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            if(loop_succ)
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                    printf("NO such frame in keyframe_database\n");
                    assert(false);
                }
                printf("loop succ %d with %drd image\n", process_keyframe_cnt-1, old_index);
                assert(old_index!=-1);

                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;

                old_kf->getPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;

                if(measurements_old_norm.size()>MIN_LOOP_NUM)
                {

                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    mpEstimator->retrive_pose_data = (retrive_data);
                    printf("loop push\n");

                    //cout << "old pose " << T_w_i_old.transpose() << endl;
                    //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                    // add loop edge in current frame
                    cur_kf->detectLoop(old_index);
                    keyframe_database.addLoop(old_index);
                    old_kf->is_looped = 1;
                    //loop_old_index = old_index;
                }
            }
            cur_kf->image.release();
        }
        if(loop_succ){
            sleep(2);
        }
        sleep(0.05);  //0.1s
    }
}

/*void vinssystem::solve_visiononly()
{
    while(true)
    {
        if(mpEstimator->csolve_visiononly == true)
        {
            Quaterniond *Q = new Quaterniond[WINDOW_SIZE + 1];
            Vector3d *T = new Vector3d[WINDOW_SIZE + 1];
            map<int, Vector3d> sfm_tracked_points;
            GlobalSFM sfm;
            sfm.setHeaders(mpEstimator->cHeaders);
            //if(sfm.construct2(WINDOW_SIZE + 1, Q, T, mpEstimator->cl,mpEstimator->crelative_R, mpEstimator->crelative_T,mpEstimator->csfm_f, sfm_tracked_points))
            if(sfm.construct3(WINDOW_SIZE + 1, Q, T, mpEstimator->cQs, mpEstimator->cPs, mpEstimator->csfm_f, sfm_tracked_points))
            {

                vector<double> deltathetas;
                vector<double> dts;
                for(double t = -0.06; t < 0.04; t += 0.003)
                {
                    double dtheta = deltaThetaGivendt(t,Q);
                    deltathetas.push_back(dtheta);
                    dts.push_back(t);
                }
                double mindtheta = 90;
                double recordmint = 0;
                for( int i = 0; i < dts.size(); i++ )
                {
                    if(deltathetas[i] < mindtheta)
                    {
                        mindtheta = deltathetas[i];
                        recordmint = dts[i];
                    }
                }
                if(mindtheta < (WINDOW_SIZE - 1) * 1.5)
                {
                    FILE* fp = fopen("/storage/emulated/0/Android/data/com.example.root.mainloop/files/temporal_calibrate.csv","a");
                    fprintf(fp,"%f,%f\n",recordmint,mindtheta);
                    fclose(fp);
                    mpEstimator->Delta_t.push_back(recordmint);
                    mpEstimator->Min_thetas.push_back(mindtheta);
                    if(mpEstimator->Delta_t.size() < MEDIAN_WINDOW)
                    {
                        mpEstimator->imu_written = false;
                    }
                    else if(mpEstimator->Delta_t.size() == MEDIAN_WINDOW)
                    {

                    }

                }
                else
                {
                    mpEstimator->imu_written = false;
                }


            }
            else
            {
                mpEstimator->imu_written = false;
            }
            delete [] Q;
            delete [] T;
            mpEstimator->csolve_visiononly = false;
            sleep(1.17);
        }
        else
        {
            sleep(0.03);
        }
    }
}*/

/*void vinssystem::bubbleSort(double* arr, int n)
{
    int i, j;
    for (i = 0; i < n-1; i++){

        // Last i elements are already in place
        for (j = 0; j < n-i-1; j++){
            if (arr[j] > arr[j+1]){
                double tmp = arr[j + 1];
                arr[j + 1] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}*/

/*double vinssystem::deltaThetaGivendt(double dt, Quaterniond* Q)
{
    std::vector<double> tmp_imu_t;
    std::vector<Vector3d> tmp_imu_angular_velocities;
    for( int i = 0; i < mpEstimator->cimu_angular_velocities.size(); i++ )
    {
        double th = mpEstimator->cimu_t[i] - dt;
        tmp_imu_t.push_back(th);
    }
    tmp_imu_angular_velocities = mpEstimator->cimu_angular_velocities;
    Quaterniond *dQs_cam = new Quaterniond[WINDOW_SIZE - 1];
    for( int i = 0; i < WINDOW_SIZE - 1; i++ )
    {
        Quaterniond q1 = Q[i];
        Quaterniond q2 = Q[i + 1];
        Quaterniond dq = q1.inverse()*q2;
        dQs_cam[i] = dq;
    }
    int j = 0;
    Quaterniond imu_q;
    imu_q.w() = 1; imu_q.x() = 0; imu_q.y() = 0; imu_q.z() = 0;
    Quaterniond qic{mpEstimator->ric};
    Quaterniond *dQs_imu = new Quaterniond[WINDOW_SIZE - 1];
    for( int i = 0; i < tmp_imu_angular_velocities.size(); i++ )
    {
        if(tmp_imu_t[i] < mpEstimator->cHeaders[j])
        {
            continue;
        }
        else
        {
            if(i == 0)
            {
                return 90;
            }
            Vector3d gyr_0 = tmp_imu_angular_velocities[i - 1];
            Vector3d gyr_1 = tmp_imu_angular_velocities[i];
            Vector3d un_gyr = 0.5 * (gyr_0 + gyr_1);
            double dt;
            if(tmp_imu_t[i - 1] < mpEstimator->cHeaders[j])
            {
                dt = tmp_imu_t[i] - mpEstimator->cHeaders[j];
            }
            else if(tmp_imu_t[i] < mpEstimator->cHeaders[j + 1])
            {
                dt = tmp_imu_t[i] - tmp_imu_t[i - 1];
            }
            else
            {
                dt = mpEstimator->cHeaders[j + 1] - tmp_imu_t[i - 1];
            }
            imu_q = imu_q * Utility::deltaQ(un_gyr * dt);
            if(tmp_imu_t[i] >= mpEstimator->cHeaders[j + 1])
            {
                i--;
                dQs_imu[j] = qic.inverse() * imu_q * qic;
                imu_q.w() = 1; imu_q.x() = 0; imu_q.y() = 0; imu_q.z() = 0;
                j++;
            }
            if(j == WINDOW_SIZE - 1)
            {
                break;
            }
        }
    }
    double sum_theta = 0;
    for( int i = 0; i < WINDOW_SIZE - 1; i++ )
    {
        Quaterniond q1 = dQs_cam[i];
        Quaterniond q2 = dQs_imu[i];
        Quaterniond dq = q1.inverse()*q2;
        double dtheta = 2*abs(atan2(dq.vec().norm(),dq.w()))*180.0/3.14;
        sum_theta += dtheta;
    }
    return sum_theta;
}*/

void vinssystem::process_global_loop_detection() {
    while(true)
    {
        if(start_global_optimization)
        {
            start_global_optimization = false;
            keyframe_database.optimize4DoFLoopPoseGraph(kf_global_index,loop_correct_t,loop_correct_r);
            mpEstimator->t_drift = loop_correct_t;
            mpEstimator->r_drift = loop_correct_r;
            //sleep(0.27);
            sleep(1.17);
        }
        sleep(0.03);
    }
}

void vinssystem::inputIMU(ImuConstPtr imu_msg) {
    //img_msg callback
    m_buf.lock();
    imu_msg_buf.push(imu_msg);
//    printf("IMU_buf timestamp %lf, acc_x = %lf\n",imu_msg_buf.front()->header,imu_msg_buf.front()->acc.x());
    m_buf.unlock();
    con.notify_one();
}



cv::Mat vinssystem::inputImage(cv::Mat& image,double t,pair<double,vector<Box>> onebox) {

//    printf("image processing\n");


    IMG_MSG *img_msg = new IMG_MSG();
    //cout << (videoCamera->grayscaleMode) << endl;
    img_msg->header = t;

    cv::Mat gray;

    if (image.channels() == 3)
        cv::cvtColor(image, gray, CV_RGB2GRAY);  //euroc provide gray image already
    else if(image.channels()==4)
        cv::cvtColor(image, gray, CV_RGBA2GRAY);
    else
        gray = image;

    cv::Mat img_with_feature;
    cv::Mat img_equa;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3);
    clahe->apply(gray, img_equa);
    //img_equa = gray;
    // TS(time_feature);

    vector<Point2f> good_pts;
    vector<double> track_len;
    vector<int> track_cnt;
    cv::Mat rot12;
    mpFeaturetracker->readImage(img_equa, img_with_feature, img_msg->header, frame_cnt, good_pts, track_len,track_cnt, rot12);

    //  TE(time_feature);

    //image msg buf
    int is_calculate = false;
    if (mpFeaturetracker->img_cnt == 0) {

        img_msg->point_clouds = mpFeaturetracker->image_msg;
        img_msg->rot12 = rot12.clone();
        //img_msg callback
        m_buf.lock();
        //img_msg->header += dt_initial;
        img_msg_buf.push(img_msg);
        is_calculate = true;
//        printf("Img timestamp %lf\n",img_msg_buf.front()->header);
        m_buf.unlock();
        con.notify_one();

        if (LOOP_CLOSURE) {
            i_buf.lock();
            cv::Mat loop_image = gray.clone();
            image_buf_loop.push(make_pair(loop_image, img_msg->header));
            if (image_buf_loop.size() > WINDOW_SIZE)
                image_buf_loop.pop();
            i_buf.unlock();
        }
    } else {
        is_calculate = false;
    }


    mpFeaturetracker->img_cnt = (mpFeaturetracker->img_cnt + 1) % FREQ;

    if (image.channels() == 1)
    cvtColor(image,image,CV_GRAY2BGRA);  //从GRAY转为BGRA

    for (int i = 0; i < good_pts.size(); i++) {
        double len=std::min(1.0, 1.0 * track_cnt[i]/WINDOW_SIZE);
        cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - len), 0, 255 * len,255), 7); //BGR
    }

    for(int i = 0; i < onebox.second.size(); i++)
    {
        cv::Point2f pt1,pt2;
        pt1.x = onebox.second[i].x0;
        pt1.y = onebox.second[i].y0;
        pt2.x = onebox.second[i].x1;
        pt2.y = onebox.second[i].y1;
        cv::rectangle(image,pt1,pt2,cv::Scalar(0,0,255));
    }

    cv::Mat imText;
    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
    cv::Scalar initcolor=cv::Scalar(0,0,0,255);
    imText = cv::Mat(image.rows+(textSize.height+10)*4,image.cols,image.type(),initcolor);
    image.copyTo(imText.rowRange(0,image.rows).colRange(0,image.cols));
    //imText.rowRange(image.rows,imText.rows) = cv::Mat::zeros((textSize.height+10)*4,image.cols,image.type(),cv::Scalar(0,0,0,255));
//    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5-(textSize.height+10)*3),cv::FONT_HERSHEY_PLAIN,5,cv::Scalar(255,0,255,255),5,8);
//    cv::putText(imText,s1.str(),cv::Point(5,imText.rows-5-(textSize.height+10)*2),cv::FONT_HERSHEY_PLAIN,5,cv::Scalar(255,0,255,255),5,8);
    cv::putText(imText,s2.str(),cv::Point(5,imText.rows-5-(textSize.height+10)),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,255,255),1,8);
  //  cv::putText(imText,s3.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,255,255),1,8);
    imText.copyTo(mimText);
    return imText;
    //cv::imshow("VINS: Current Frame",imText);
    //cv::waitKey(100);
}

void vinssystem::Run()
{

//         //创建一个窗口
//     pangolin::CreateWindowAndBind("Main",640,480);
//     //启动深度测试
//     glEnable(GL_DEPTH_TEST);

//     // Define Projection and initial ModelView matrix
//     pangolin::OpenGlRenderState s_cam(
//             pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
//             //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
//             pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
//     );

//     // Create Interactive View in window
//     pangolin::Handler3D handler(s_cam);
//     //setBounds 跟opengl的viewport 有关
//     //看SimpleDisplay中边界的设置就知道
//     pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
//                             .SetHandler(&handler);

//     while(!pangolin::ShouldQuit())
//     {
//         // Clear screen and activate view to render into
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);

//         // Render OpenGL Cube
// //        pangolin::glDrawColouredCube();\
//         //坐标轴的创建
//         pangolin::glDrawAxis(3);

//         //点的创建
//         glPointSize(10.0f);
//         glBegin(GL_POINTS);
//         glColor3f(1.0,1.0,1.0);
//         glVertex3f(0.0f,0.0f,0.0f);
//         glVertex3f(1,0,0);
//         glVertex3f(0,2,0);
//         glEnd();

//         //把下面的点都做一次旋转变换
//         glPushMatrix();
//         //col major
//         std::vector<GLfloat > Twc = {1,0,0,0, 0,1,0,0 , 0,0,1,0 ,0,0,5,1};
//         glMultMatrixf(Twc.data());

//         //直线的创建
//         const float w = 2;
//         const float h = w*0.75;
//         const float z = w*0.6;
//         glLineWidth(2);
//         glColor3f(1.0,0,0);
//         glBegin(GL_LINES);

//         glVertex3f(0,0,0);
//         glVertex3f(w,h,z);
//         glVertex3f(0,0,0);
//         glVertex3f(w,-h,z);
//         glVertex3f(0,0,0);
//         glVertex3f(-w,-h,z);
//         glVertex3f(0,0,0);
//         glVertex3f(-w,h,z);
//         glVertex3f(w,h,z);
//         glVertex3f(-w,h,z);
//         glVertex3f(-w,h,z);
//         glVertex3f(-w,-h,z);
//         glVertex3f(-w,-h,z);
//         glVertex3f(w,-h,z);
//         glVertex3f(w,-h,z);
//         glVertex3f(w,h,z);
//         glEnd();

//         glPopMatrix();

//         // Swap frames and Process Events
//         pangolin::FinishFrame();
//     }



    pangolin::CreateWindowAndBind("VINS-PC: Map Viewer",1024,768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    // pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    // pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    // pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    // pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    // pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    // pangolin::Var<bool> menuReset("menu.Reset",false,false);
    // pangolin::Var<bool> menuWriteFrames("menu.WriteFrames",false,false);

    float mViewpointF = 500;
    float mViewpointX = -1.0;
    float mViewpointY = -3.8;
    float mViewpointZ = 7.0;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, -1.0,-3.8,0,0.0,1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'a', vinssystem::setstopflag);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    cv::namedWindow("VINS-PC2: Current Frame", CV_WINDOW_NORMAL);
    while(true)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mpDrawer->GetCurrentOpenGLCameraMatrix(Twc);
        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpDrawer->DrawGrids();
        mpDrawer->DrawMapPoints();
        mpDrawer->DrawKeyFrames(true,true);
        pangolin::FinishFrame();
        if(!mimText.empty())
        {
            cv::imshow("VINS-PC2: Current Frame",mimText);
            cv::waitKey(mT);
        }
        usleep(100000);
        // if(menuWriteFrames)
        // {
        //     string strwriteframepath = msdatafolder + "frames.txt";
        //     keyframe_database.WriteAllFrames(strwriteframepath.c_str(),ric_double.inverse());
        //     menuWriteFrames = false;
        // }
//          glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);

//         // Render OpenGL Cube
// //        pangolin::glDrawColouredCube();\
//         //坐标轴的创建
//         pangolin::glDrawAxis(3);

//         //点的创建
//         glPointSize(10.0f);
//         glBegin(GL_POINTS);
//         glColor3f(1.0,1.0,1.0);
//         glVertex3f(0.0f,0.0f,0.0f);
//         glVertex3f(1,0,0);
//         glVertex3f(0,2,0);
//         glEnd();

//         //把下面的点都做一次旋转变换
//         glPushMatrix();
//         //col major
//         std::vector<GLfloat > Twc = {1,0,0,0, 0,1,0,0 , 0,0,1,0 ,0,0,5,1};
//         glMultMatrixf(Twc.data());

//         //直线的创建
//         const float w = 2;
//         const float h = w*0.75;
//         const float z = w*0.6;
//         glLineWidth(2);
//         glColor3f(1.0,0,0);
//         glBegin(GL_LINES);

//         glVertex3f(0,0,0);
//         glVertex3f(w,h,z);
//         glVertex3f(0,0,0);
//         glVertex3f(w,-h,z);
//         glVertex3f(0,0,0);
//         glVertex3f(-w,-h,z);
//         glVertex3f(0,0,0);
//         glVertex3f(-w,h,z);
//         glVertex3f(w,h,z);
//         glVertex3f(-w,h,z);
//         glVertex3f(-w,h,z);
//         glVertex3f(-w,-h,z);
//         glVertex3f(-w,-h,z);
//         glVertex3f(w,-h,z);
//         glVertex3f(w,-h,z);
//         glVertex3f(w,h,z);
//         glEnd();

//         glPopMatrix();

//         // Swap frames and Process Events
//         pangolin::FinishFrame();
    }
}
