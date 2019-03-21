#include "ptgrey_camera_manager.h"
#include "ptgrey_save_image_task.h"

PtgreyCameraManager::PtgreyCameraManager(const std::string &_rawPath):
    threadPool_(4, 30)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    LogBuildInfo();

    FlyCapture2::BusManager busMgr;
    unsigned int cameraNum = 0;
    FlyCapture2::Error error = busMgr.GetNumOfCameras(&cameraNum);
    if(error != FlyCapture2::PGRERROR_OK) {
        LogErrorTrace(error);
        exit(1);
    }
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

void PtgreyCameraManager::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.start();
    (void)RunAllCameras();

    const int CAM_NUM = pSinglePtgreyCameras_.size();
    int i = -1;
    ros::NodeHandle nh;
    ros::Rate rate(2);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        ++i;
        i %= CAM_NUM;
        pSinglePtgreyCameras_[i]->PublishImage();
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
