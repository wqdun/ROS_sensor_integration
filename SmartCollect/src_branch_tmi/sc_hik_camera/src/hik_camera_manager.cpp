#include "hik_camera_manager.h"

HikCameraManager::HikCameraManager(const std::string &_rawPath):
    threadPool_(10, 30)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    rawDataPath_ = _rawPath;
    pSingleCameras_.clear();
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    isSaveImg_ = false;
    unixTimeMinusGpsTime_ = -1;

    const std::string rawdataImuPath(_rawPath + "/IMU/");

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
}

HikCameraManager::~HikCameraManager() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.stop();
    LOG(INFO) << __FUNCTION__ << " end.";
}

void HikCameraManager::MonitorCB(const sc_msgs::MonitorMsg::ConstPtr& pMonitorMsg) {
    LOG_EVERY_N(INFO, 20) << __FUNCTION__ << " start, is_record Camera: " << (int)(pMonitorMsg->is_record);
    isSaveImg_ = (bool)pMonitorMsg->is_record;
    unixTimeMinusGpsTime_ = pMonitorMsg->unix_time_minus_gps_time;
}

void HikCameraManager::RegisterCB() {
    subMonitor_ = nh_.subscribe("sc_monitor", 10, &HikCameraManager::MonitorCB, this);
}

void HikCameraManager::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)RegisterCB();
    int err = MV_OK;
    threadPool_.start();
    err = MV_CC_EnumDevices(MV_GIGE_DEVICE, &deviceList_);
    assert(MV_OK == err);
    const size_t camNum = deviceList_.nDeviceNum;
    LOG(INFO) << "Find " << camNum << " Devices.";
    sharedMem_->cameraNum = camNum;

    if(public_tools::PublicTools::isFileExist("/dev/ttyS0")) {
        sharedMem_->imuSerialPortStatus = 0;
    }
    else {
        sharedMem_->imuSerialPortStatus = -1;
    }

    for(size_t i = 0; i < camNum; ++i) {
        boost::shared_ptr<SingleCamera> pSingleCamera(new SingleCamera(this));
        pSingleCamera->SetCamera(deviceList_, i);
        pSingleCameras_.push_back(pSingleCamera);
    }

    int i = -1;
    ros::Rate rate(2);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        if(0 != camNum) {
            ++i;
            i %= camNum;
            pSingleCameras_[i]->PublishImage();
        }
    }
}

void HikCameraManager::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

