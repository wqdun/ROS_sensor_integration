#include "camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

bool CPGCamera::sIsSaveImg_ = true; //false;
std::string CPGCamera::sRawdataPath_("");

Cameras::Cameras(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    LOG(INFO) << __FUNCTION__ << " start.";

    camGainLast_ = -1;
    subServer_ = nh.subscribe("sc_monitor", 10, &Cameras::serverCB, this);

    FlyCapture2::Error error;
    BusManager busMgr;
    unsigned int cameraNum = 0;
    error = busMgr.GetNumOfCameras(&cameraNum);
    if(error != PGRERROR_OK) {
        logErrorTrace(error);
        exit(1);
    }
    LOG(INFO) << "Number of cameras detected: " << cameraNum;
    if(cameraNum < 1) {
        LOG(INFO) << "Insufficient number of cameras: " << cameraNum;
        exit(1);
    }

    pCpgCameras_.clear();
    (void)getCameras(cameraNum);
    (void)startAll();


}

void Cameras::getCameras(unsigned int _cameraNum) {
    LOG(INFO) << __FUNCTION__ << " start.";

    ros::NodeHandle nh;
    for(int i = 0; i < _cameraNum; ++i) {
        CPGCamera *pCPGCamera = new CPGCamera(nh, i);
        if(NULL == pCPGCamera) {
            LOG(ERROR) << "Failed to create CPGCamera.";
            exit(1);
        }

        pCpgCameras_.push_back(pCPGCamera);
    }
}

void Cameras::startAll() {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(auto &pCpgCamera: pCpgCameras_) {
        pCpgCamera->StartCapture();
    }
}

Cameras::~Cameras() {}

void Cameras::logErrorTrace(FlyCapture2::Error error) {
    LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
}

void Cameras::serverCB(const sc_msgs::MonitorMsg::ConstPtr& pClientMsg) {
    LOG(INFO) << __FUNCTION__ << " start, is_record Camera: " << (int)(pClientMsg->is_record);

    (void)CPGCamera::SetIsSaveImg(pClientMsg->is_record);

    int8_t camGain = pClientMsg->cam_gain;
    if(camGain != camGainLast_)
    {
        pCpgCameras_[0]->StopCapture();
        bool isGainSet = pCpgCameras_[0]->SetCameragain(camGain);
        LOG(INFO) << "camGainLast_: " << camGainLast_ << " --> camGain: " << camGain;
        if(isGainSet)
        {
            LOG(INFO) << "camGain changed successfully.";
        }
        else
        {
            LOG(WARNING) << "Failed to set camGain.";
        }
        pCpgCameras_[0]->StartCapture();
    }

    camGainLast_ = camGain;
}


void Cameras::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    pCommTimer_.reset(new CommTimer(rawdataPath_) );
    boost::thread thrd(boost::bind(&CommTimer::getTime, pCommTimer_) );
    LOG(INFO) << "I am a CommTimer thread.";

    (void)CPGCamera::SetRawdataPath(rawdataPath_);

    ros::Rate rate(1);

    while(ros::ok() ) {
        ros::spinOnce();
        rate.sleep();

    }
}





    // ros::Publisher pub_5651  = nh_time.advertise<roscameragpsimg::imu5651>("imu_string", 1000);

    // roscameragpsimg::imu5651  msg;

