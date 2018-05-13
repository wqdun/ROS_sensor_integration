#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <FlyCapture2.h>


#include "comm_timer.h"
#include "dlfcn.h"
#include <execinfo.h>

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>

#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include "PGCamera.h"
#include <sc_msgs/MonitorMsg.h>

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

//namespace
using namespace std;
using namespace cv;
using namespace FlyCapture2;

//parameters of control
const int pix8  = 8;
const int pix24 = 24;

//time control
extern vector<string> parsed_data;
extern string global_gps ;
extern string imu_string;
string imu_path;

//save control
int is_save_cam = 1;
float cam_gain  = 1;
float cam_gain_record = 0.0;
int count_record = 0;

//format control
string format_str;
int format_int;

//path control
string pathSave_str = "";

//cam_fps
extern double cam_fps;


//#define IMAGE_ 100

//thread 1 get time
void *thread(void *ptr)
{
    LOG(INFO) << __FUNCTION__ << " start; thread create state is :  OK !";
    get_time();
}

//save controll callback
void sub_save_cam_callback(const sc_msgs::MonitorMsg::ConstPtr& pClientMsg)
{
    DLOG(INFO) << __FUNCTION__ << " start, is_record Camera: " << (int)(pClientMsg->is_record);
    is_save_cam = pClientMsg->is_record;
    cam_gain = pClientMsg->cam_gain;
    if(count_record == 0)
    {
        cam_gain_record = cam_gain;
        count_record = 1;
    }
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "image_publisher");

    ros::NodeHandle nh;
    ros::Subscriber sub_save_cam = nh.subscribe("sc_monitor", 10, sub_save_cam_callback);

    LOG(INFO) << "argc: " << argc;

    if(argc != 4)
    {
        LOG(INFO) << "usage: exec image_format path_save_image path_save_imu, param num: " << argc;
        return -1;
    }

    format_str = argv[1];
    pathSave_str = argv[2];
    imu_path = argv[3];
    LOG(INFO) << "format_str: " << format_str << "; pathSave_str: " << pathSave_str << "; imu_path: " << imu_path;

    if(format_str == "jpg")
    {
        format_int = 0;
    }
    else if(format_str == "raw")
    {
        format_int = 1;
    }
    else if(format_str == "jpgcount" )
    {
        format_int = 2;
    }
    else if(format_str == "png" )
    {
        format_int = 3;
    }
    else
    {
        format_int = -1;
    }


    //[1]初始化相机对象expected primary-expression before
    int ImageWidth = 1920;
    int ImageHeight= 1200;

    int m_camera_num = 1;

    vector<CPGCamera *> camera_pg;
    for(int i=0;i<m_camera_num;i++)
    {
       LOG(INFO) << "Before init camera!";
       CPGCamera *camera_pg_temp = new CPGCamera(ImageWidth, ImageHeight, nh, pathSave_str + "/camera"+ to_string(i)+"/");
       if(NULL == camera_pg_temp)
       {
        LOG(ERROR) << "Failed to create CPGCamera.";
        exit(-1);
       }
       camera_pg.push_back(camera_pg_temp);
    }

    for(int i=0;i<camera_pg.size();++i)
    {
       bool caminit = camera_pg[i]->InitCamera(i);

       LOG(INFO) << "Open state: " << std::boolalpha << caminit;
       if( !caminit )
       {
          LOG(ERROR) << "Failed to InitCamera.";
          exit(-1);
       }
       camera_pg[i]->m_CameraID = i;
       camera_pg[i]->StartCapture();
    }

    //[2]初始化相机
    /*---------------------------------------
	@ GetNumOfCameras
	@ GetCameraFromIndex
	@ Connect
	@ RestoreFromMemoryChannel
	@ GetTriggerMode
	@ SetTriggerMode
    ---------------------------------------*/

    sleep(2);
    //[3]GPS时间获取-多线程
    LOG(INFO) << "Start time get!";

    pthread_t id;
    int ret = pthread_create(&id, NULL, thread, &nh);
    if(ret)
    {
        LOG(INFO)<< "create thread state is: error! ";
        return 1;
    }

    //publish rate
    const int rate = 1;
    ros::Rate loop_rate(rate);

    //cam speed

    while(nh.ok())
    {
       ros::spinOnce();
       loop_rate.sleep();
       if(count_record == 1 && cam_gain != cam_gain_record)
       {
           camera_pg[0]->StopCapture();
           bool tellgain = camera_pg[0]->SetCameragain();
           LOG(INFO) << "cam_gain_record: " << cam_gain_record << " changed to cam_gain: " << cam_gain;
           if(tellgain)
           {
               LOG(INFO) << "cam_gain_record changed OK!";
           }
           else
           {
               LOG(INFO) << "cam_gain_record changed failed!";
           }
           LOG(INFO) << "camera_pg->StartCapture()";
           camera_pg[0]->StartCapture();
           cam_gain_record = cam_gain;
       }
    }

    return 0;
}

