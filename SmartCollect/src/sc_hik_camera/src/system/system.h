//
// Created by diner on 17-7-19.
//

#ifndef VINS_SYSTEM_H
#define VINS_SYSTEM_H
#include <stdio.h>
#include <mutex>
#include <thread>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <queue>
#include <map>
#include <loop_closure.h>
#include <keyframe_database.h>
//#include "viewer.h"
#include <VINS.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <global_param.hpp>
#include <utility.hpp>
#include <viewer/Viewer.h>
#include <viewer/MapDrawer.h>

#include <sstream>
#include "data_input.h"
#include "data_output.h"
#include "classification.h"
#include "detection_cuda.h"
#include <boost/algorithm/string.hpp>
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;
using namespace Eigen;

//class Viewer2;

class vinssystem {

public:
    vinssystem();
    void create(string voc_file, string pattern_file,string setting_file);
    void inputIMU(ImuConstPtr imu_msg);
    cv::Mat inputImage(cv::Mat& image,double t,pair<double,vector<Box>> onebox);
    void Run();
    KeyFrameDatabase keyframe_database;
    VINS *mpEstimator;
    //float yaw_now;
    //float pitch_now;
    //float roll_now;
    vector<Vector3d> points_3d;
    std::mutex m_points3d;
    //double dt_initial;
    //bool synchronized_flag;
    cv::Mat mimText;
    //Viewer2* mpViewer;
    double mT;
    MapDrawer* mpDrawer;
    string msdatafolder;
    Matrix3d ric_double;
    Matrix3d Rio;
    Vector3d tio;
    double rencoder_v;
    double v_n;
    static void setstopflag();
    static bool stopflag;
    ~vinssystem();
    Detector* pdetector_SSD;



    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    std::condition_variable con;
    queue<ImuConstPtr> imu_msg_buf;
    queue<ImgConstPtr> img_msg_buf;
    //std::mutex m_posegraph_buf;
    //queue<int> optimize_posegraph_buf;
    //bool synchronizing_flag;

    //int sum_of_wait = 0;

    std::mutex m_buf;
    //std::mutex m_state;
    std::mutex i_buf;
    //std::mutex m_loop_drift;
    //std::mutex m_keyframedatabase_resample;
    //std::mutex m_update_visualization;

    //double latest_time;

    //Eigen::Vector3d tmp_P;
    //Eigen::Quaterniond tmp_Q;
    //Eigen::Vector3d tmp_V;
    //Eigen::Vector3d tmp_Ba;
    //Eigen::Vector3d tmp_Bg;
    //Eigen::Vector3d acc_0;
    //Eigen::Vector3d gyr_0;
//    Viewer *mpViewer;
//    std::thread* mptViewer;
    std::thread* measurement_process;
    std::thread* loop_detection;
    std::thread* global_loop_detection;
    //std::thread* synchronization_thread;
    std::thread* th_display;
//private:
    stringstream s;
    stringstream s1;
    stringstream s2;
    stringstream s3;


    //queue<pair<cv::Mat, double>> image_buf;
    queue<pair<cv::Mat, double>> image_buf_loop;
    LoopClosure *loop_closure;

//private:
    int process_keyframe_cnt = 0;
    //int miss_keyframe_num = 0;
    int keyframe_freq = 0;
    int global_frame_cnt = 0;
    int loop_check_cnt = 0;
    int waiting_lists = 0;  //number of measurements waiting to be processed
//camera param
    //vector<int> erase_index;
//private:
    Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();
    double current_time = -1;
    //double lateast_imu_time = -1;
    //bool voc_init_ok = false;
    int old_index = -1;
    FeatureTracker *mpFeaturetracker;
    int frame_cnt = 0;
    int count_inputImage;

    void updateView();
    //void update();
    void process();
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();
    void send_imu(const ImuConstPtr &imu_msg,double header);
    void process_loop_detection();
    void process_global_loop_detection();
    //void solve_visiononly();
    //double deltaThetaGivendt(double dt,Quaterniond* Q);
    //void bubbleSort(double* arr, int n);
    camodocal::CameraPtr m_camera;
    cv::FileStorage fSettings;

    string mVocFile;
    string mPatternFile;
    string mSettingFile;

    int kf_global_index;
    bool start_global_optimization ;

    int imgWidth;    // cols
    int imgHeight;   // rows
    int max_cnt;
    int min_dis;

    Vector3d corrected_Ps[WINDOW_SIZE];
    Matrix3d corrected_Rs[WINDOW_SIZE];
    double dx , dy, dz, rx, ry, rz;
    double px, py, pz, gps_yaw, gps_pitch, gps_roll;

    double a_n,a_w,g_n,g_w;



};


#endif //VINS_SYSTEM_H
