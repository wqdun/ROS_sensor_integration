#include "ptgrey_camera_manager.h"
#include "ptgrey_save_image_task.h"
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>

PtgreyCameraManager::PtgreyCameraManager(const std::string &_rawPath):
    threadPool_(4, 30)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    isSaveImg_ = false;
    rawDataPath_ = _rawPath;

    LogBuildInfo();
    FlyCapture2::BusManager busMgr;
    unsigned int cameraNum = 0;
    FlyCapture2::Error error = busMgr.GetNumOfCameras(&cameraNum);
    if(error != FlyCapture2::PGRERROR_OK) {
        LogErrorTrace(error);
        exit(1);
    }

    int shmid = shmget((key_t)1234, sizeof(struct SharedMem), 0666|IPC_CREAT);
    if(shmid < 0) {
        LOG(ERROR) << "shget failed.";
        exit(1);
    }
    void *shm = shmat(shmid, 0, 0);
    if(shm == (void *)-1) {
        LOG(ERROR) << "shmat failed.";
        exit(1);
    }
    LOG(INFO) << "Memory attached at " << shm;
    sharedMem_ = (struct SharedMem*)shm;
    sharedMem_->cameraNum = cameraNum;

    LOG(INFO) << "Number of cameras detected: " << cameraNum;
    if(cameraNum < 1) {
        LOG(INFO) << "Insufficient number of cameras: " << cameraNum;
        exit(2);
    }

    pSinglePtgreyCameras_.clear();
    (void)GetCameras(cameraNum, _rawPath);
}

void PtgreyCameraManager::GetCameras(unsigned int _cameraNum, const std::string &__rawdataDir) {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(int i = 0; i < _cameraNum; ++i) {
        SinglePtgreyCamera *pSinglePtgreyCamera = new SinglePtgreyCamera(this);
        if(!pSinglePtgreyCamera) {
            LOG(ERROR) << "Failed to create SinglePtgreyCamera.";
            exit(1);
        }

        pSinglePtgreyCamera->InitCamera(i, __rawdataDir);
        pSinglePtgreyCameras_.push_back(pSinglePtgreyCamera);
    }
}

PtgreyCameraManager::~PtgreyCameraManager() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.stop();
    LOG(INFO) << __FUNCTION__ << " end.";
}

void PtgreyCameraManager::LogBuildInfo() {
    FlyCapture2::FC2Version fc2Version;
    FlyCapture2::Utilities::GetLibraryVersion(&fc2Version);

    std::ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    LOG(INFO) << version.str();

    std::ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    LOG(INFO) << timeStamp.str();
}

void PtgreyCameraManager::LogErrorTrace(FlyCapture2::Error error) {
    LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
}

void PtgreyCameraManager::RunAllCameras() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(auto &pSinglePtgreyCamera: pSinglePtgreyCameras_) {
        pSinglePtgreyCamera->Run();
    }
}

void PtgreyCameraManager::RegisterCB() {
    subMonitor_ = nh_.subscribe("sc_monitor", 10, &PtgreyCameraManager::MonitorCB, this);
}

void PtgreyCameraManager::MonitorCB(const sc_msgs::MonitorMsg::ConstPtr& pMonitorMsg) {
    LOG_EVERY_N(INFO, 20) << __FUNCTION__ << " start, is_record Camera: " << (int)(pMonitorMsg->is_record);
    isSaveImg_ = (bool)pMonitorMsg->is_record;
    unixTimeMinusGpsTime_ = pMonitorMsg->unix_time_minus_gps_time;
}

void PtgreyCameraManager::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.start();
    (void)RunAllCameras();
    (void)RegisterCB();

    const int CAM_NUM = pSinglePtgreyCameras_.size();
    int i = -1;
    ros::Rate rate(2);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        ++i;
        i %= CAM_NUM;
        pSinglePtgreyCameras_[i]->PublishImage();
        pSinglePtgreyCameras_[i]->PublishImageFreq();
    }

    (void)PressEnterToExit();
}

void PtgreyCameraManager::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

