//
//  VINS.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef VINS_hpp
#define VINS_hpp

#include <stdio.h>
#include "feature_manager.hpp"
#include "utility.hpp"
#include "projection_factor.hpp"
#include "pose_local_parameterization.hpp"
#include "projection_td_factor.h"
#include "global_param.hpp"
#include <ceres/ceres.h>
#include "marginalization_factor.hpp"
#include "imu_factor.h"
//#include "draw_result.hpp"
#include <opencv2/core/eigen.hpp>
#include <initial/initial_ex_rotation.h>
#include "initial_sfm.hpp"
#include "initial_aligment.hpp"
#include "motion_estimator.hpp"
#include "loop_closure_factor.hpp"
#include <opencv2/opencv.hpp>
#include "speedprior.h"
#include "imu_timefactor.h"

extern double FOCUS_LENGTH_X;
extern double FOCUS_LENGTH_Y;
extern double PX;
extern double PY;
//extern bool LOOP_CLOSURE;
struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Vector3d P_old;
    Quaterniond Q_old;
    Vector3d P_cur;
    Quaterniond Q_cur;
    vector<cv::Point2f> measurements;
    vector<int> features_ids;
    bool use;
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
};

class VINS
{
public:

    typedef IMUFactor IMUFactor_t;

    VINS(string strSettingPath, Matrix3d _Rio, Vector3d _tio);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    FeatureManager *f_manager;
    MotionEstimator m_estimator;
    int frame_count;

    Matrix3d ric;
    Vector3d tic;
    Matrix3d config_ric;
    Vector3d config_tic;
    Matrix3d config_rio;
    Vector3d config_tio;
    MarginalizationFlag  marginalization_flag;
    bool Rs_init;
    bool synchronized_flag;
    Vector3d Ps[10 * (WINDOW_SIZE + 1)];  // p^w_i
    Vector3d Vs[10 * (WINDOW_SIZE + 1)];  // v^w_i
    Matrix3d Rs[10 * (WINDOW_SIZE + 1)];  // q^w_i
    Vector3d Bas[10 * (WINDOW_SIZE + 1)];
    Vector3d Bgs[10 * (WINDOW_SIZE + 1)];
    Vector3d cPs[10 * (WINDOW_SIZE + 1)];
    Quaterniond cQs[10 * (WINDOW_SIZE + 1)];
    int count_keyframe;

    double cHeaders[WINDOW_SIZE + 1];
    Vector3d Ps_retrive;
    Quaterniond Qs_retrive;

    std::vector<double> cimu_t;
    std::vector<Vector3d> cimu_angular_velocities;
    std::vector<double> Delta_t;
    std::vector<double> Min_thetas;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_dt[1];
    double para_yaw[1];

    //for loop closure
    RetriveData retrive_pose_data, front_pose;
    bool loop_enable;
    vector<Vector3f> correct_point_cloud;
    Vector3f correct_Ps[WINDOW_SIZE];
    Matrix3f correct_Rs[WINDOW_SIZE];
    Vector3d t_drift;
    Matrix3d r_drift;
    double rec_header;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
    vector<Vector3f> point_cloud;

    int feature_num;
    bool propagate_init_ok;
    double dt2;
    double myawio;
    int optimize_num;
    string msdatafolder;
    //string mswriteodometrypath;

    IntegrationBase *pre_integrations[10 * (WINDOW_SIZE + 1)];
    IntegrationBase *cumulate_preintegrations[10 * (WINDOW_SIZE + 1)];
    IntegrationBase *preintegrations_lasttwo;
    bool first_imu;
    Vector3d acc_0, gyr_0;
    Vector3d enh_0, ypr_0;
    double encoder_v0;
    vector<double> dt_buf[10 * (WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[10 * (WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[10 * (WINDOW_SIZE + 1)];
    vector<double> encoder_v_buf[10 * (WINDOW_SIZE + 1)];
    Matrix<double, 7, 1> IMU_linear[10 * (WINDOW_SIZE + 1)];
    Matrix3d IMU_angular[10 * (WINDOW_SIZE + 1)];
    double Headers[10 * (WINDOW_SIZE + 1)];
    bool Qualities[10 * (WINDOW_SIZE + 1)];
    int ImuLinks[10 * (WINDOW_SIZE + 1)];
    int nFeatures[10 * (WINDOW_SIZE + 1)];
    Vector3d g;

    vector<Vector3d> init_point_cloud;
    vector<Vector3d> margin_map;
    vector<Vector3d> key_poses;
    vector<Vector3d> init_poses;
    vector<Vector3d> init_velocity;
    vector<Matrix3d> init_rotation;
    vector<MarkerPerID> vmarkerspID;
    vector<Eigen::Quaterniond> init_quaternion;
    vector<MarkerPoints3D> mvcurrentmarkers;
    double initial_timestamp;
    vector<Vector3d> gt_init_poses;
    Vector3d init_P;
    Vector3d init_V;
    Matrix3d init_R;

    Matrix3d pre_cumu_R;
    Vector3d pre_cumu_P;

    SolverFlag solver_flag;
    Matrix3d Rc[10 * (WINDOW_SIZE + 1)];
    double mean_diff_accs;

    //for initialization
    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;
    IntegrationBase *output_pre_integration;
    Matrix3d back_R0;
    Vector3d back_P0;
    //for falure detection
    double spatial_dis;
    bool failure_hand;
    bool is_failure;
    Vector3d ypr_imu;
    Vector3d Ps_his[WINDOW_SIZE];
    Matrix3d Rs_his[WINDOW_SIZE];
    double Headers_his[WINDOW_SIZE];
    bool need_recover;

    Matrix3d last_R,last_R_old;
    Vector3d last_P,last_P_old;

    Vector3d rec_enh;

    //for visulization
    //DrawResult drawresult;
    bool updating_image_show;
    bool updating_imageAI;
    cv::Mat image_show;
    cv::Mat imageAI;
    Matrix3f RcForView;
    enum InitStatus
    {
        FAIL_IMU,
        FAIL_PARALLAX,
        FAIL_RELATIVE,
        FAIL_SFM,
        FAIL_PNP,
        FAIL_ALIGN,
        FAIL_CHECK,
        SUCC
    };
    InitStatus init_status;
    int parallax_num_view;
    int fail_times;
    double final_cost;
    double visual_cost;
    int visual_factor_num;
    InitialEXRotation initial_ex_rotation;
    cv::FileStorage fSettings;
    double w_norm;
    camodocal::CameraPtr m_camera;
    std::queue<Quaterniond> imu_slide_quaternions;
    std::queue<double> imu_t;
    std::queue<Vector3d> imu_angular_velocities;
    Quaterniond cumulate_quaternion;

    double sum_dt;
    double sum_dx;
    double average_speed;
    double imu_endheader;
    bool big_turning;
    bool imu_written;
    double t0;
    int nmarkerID;

    vector<SFMFeature> csfm_f;
    Matrix3d crelative_R;
    Vector3d crelative_T;
    int cl;
    bool csolve_visiononly;
    bool vsolve_visiononly;
    vector<pair<double, pair<Matrix3d, Vector3d>>> vpos_before_initialize;

    void solve_marker();
    void solve_ceres(int buf_num);
    void solve_pnp();
    void solveCalibration();
    void old2new();
    void new2old();
    void clearState();
    void setIMUModel();
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    int decideImuLink();
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double header, int buf_num, cv::Mat rot12, Eigen::Vector3d xyz, Eigen::Vector3d ypr, vector<MarkerPerFrame> vmarkerspf);
    void processIMU(double t, double dt, double encoder_v, const Vector3d &linear_acceleration, const Vector3d &angular_velocity, const Vector3d &gps_position, const Vector3d &gps_altitude);
    void changeState();
    bool solveInitial();
    bool solveInitialWithOdometry2();
    bool solveInitialWithOdometry();
    bool relativePose(int camera_id, Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool relativePosewithOdometry(int camera_id);
    bool relativePose2(int camera_id, Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    bool visualInitialAlignOdometry();
    bool failureDetection();
    void failureRecover();
    void update_loop_correction();
    void PrepareForVisionOnlyBA();
    void GetInitialPosesFromOdometry(Quaterniond* q, Vector3d* T);
    void GetInitialPosesFromOdometry2(Quaterniond* q, Vector3d* T, Quaterniond* q2, Vector3d* T2);
    cv::Mat Triangulation(const std::vector<cv::Mat> vPoses, const std::vector<cv::Mat> vobs, cv::Mat P3w, cv::Mat K, int niterate, bool& which,float &norm1,float &norm2);


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif /* VINS_hpp */
