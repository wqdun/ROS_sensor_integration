#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<FlyCapture2.h>


#include"./comm_timer.h"
#include "dlfcn.h"
#include <execinfo.h>

#include<stdio.h>
#include<iostream>
#include <pthread.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>

#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include"./PGCamera.h"
#include "SmartCollector/clientCmd.h"

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

//format control
string format_str;
int    format_int;

//path control
string pathSave_str = "";

//cam_fps
extern double cam_fps;


//#define IMAGE_ 100

//thread 1 get time
void *thread(void *ptr)
{
   cout<<"thread create state is :  OK !"<<endl;
   get_time();
}

//save controll callback
void sub_save_cam_callback(const SmartCollector::clientCmd::ConstPtr& pClientMsg)
{
    DLOG(INFO) << __FUNCTION__ << " start, is_record Camera: " << pClientMsg->is_record;
    is_save_cam = pClientMsg->is_record;
}
//time delay
void delay(int time);

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc,argv,"image_publisher");

    ros::NodeHandle nh;
#ifdef IMAGE_
    image_transport::ImageTransport it(nh);
    image_transport::Publisher  pub   = it.advertise("camera/image",1);
#endif

    ros::Subscriber sub_save_cam = nh.subscribe("sc_client_cmd", 10, sub_save_cam_callback);

    //get parameters
    DLOG(INFO)<<"argc"<< argc;


    if(argc==4)
    {
        format_str    = argv[1];
        pathSave_str  = argv[2];
        imu_path    = argv[3];
    }
    else
    {
        DLOG(INFO)<<"usage: exec image_format path_save_image path_save_imu:";
        return 0;
    }

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


    //[1]初始化相机对象expected primary-expression before ‘
    int ImageWidth = 1920;
    int ImageHeight= 1200;
    CPGCamera *camera_pg = NULL;
    DLOG(INFO) << "before init camera !";
    camera_pg = new CPGCamera(ImageWidth, ImageHeight, nh);
    camera_pg->m_nBufferWidth = ImageWidth;
    camera_pg->m_nBufferHeight= ImageHeight;

    //[2]初始化相机
    /*---------------------------------------
	@ GetNumOfCameras
	@ GetCameraFromIndex
	@ Connect
	@ RestoreFromMemoryChannel
	@ GetTriggerMode
	@ SetTriggerMode
    ---------------------------------------*/
    DLOG(INFO) << "before open camera !";
    int m_CameraID = 0;
    bool caminit = false;
    for(int i = 0;  ; i++)
    {
        caminit = camera_pg->InitCamera(m_CameraID);
        DLOG(INFO) << i << ", open state: " << caminit;
        if(caminit==true)
        {
            break;
        }
        else
        {
            usleep(500000);
            camera_pg->ReleaseCamera();
        }
    }

    camera_pg->m_CameraID = m_CameraID;
    camera_pg->StartCapture();

    if(caminit)
    {
        DLOG(INFO)<<"camera open state is : ok ";
        cout<<"camera open state is : ok "<<endl;
    }
    else
    {
        DLOG(INFO)<<"camera open state is : failed ";
        cout<<"camera open state is : failed "<<endl;
        return 0;
    }

    //publish rate
    static int rate = 1;
    ros::Rate loop_rate(rate);
    FlyCapture2::Error error;

    //[0]GPS时间获取-多线程
    DLOG(INFO) <<"start time get!";

    pthread_t id;
    int ret = pthread_create(&id, NULL, thread, &nh);
    if(ret)
    {
        DLOG(INFO)<< "create thread  state  is:  error! ";
        return 1;
    }

    //cam speed
    while(nh.ok())
    {
       ros::spinOnce();
       loop_rate.sleep();

    }
    delete camera_pg;
    return 0;
}

void delay(int time)
{
    clock_t time_old = clock();
    while( clock() - time_old < time){};
}

