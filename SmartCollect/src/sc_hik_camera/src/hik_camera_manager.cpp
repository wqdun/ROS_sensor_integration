#include "hik_camera_manager.h"

HikCameraManager::HikCameraManager(const std::string &_rawPath):
    threadPool_(10, 30)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    rawDataPath_ = _rawPath;
    pSingleCameras_.clear();
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    pSerialReader_.reset(new SerialReader("/dev/ttyUSB1") );
    pSerialReaderThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SerialReader::Read, pSerialReader_) ) );
}

HikCameraManager::~HikCameraManager() {
    LOG(INFO) << __FUNCTION__ << " start.";
    pSerialReaderThread_->join();
    threadPool_.stop();
    LOG(INFO) << __FUNCTION__ << " end.";
}

void HikCameraManager::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    int err = MV_OK;
    threadPool_.start();
    err = MV_CC_EnumDevices(MV_GIGE_DEVICE, &deviceList_);
    assert(MV_OK == err);
    const size_t camNum = deviceList_.nDeviceNum;
    LOG(INFO) << "Find " << camNum << " Devices.";

    for(size_t i = 0; i < camNum; ++i) {
        boost::shared_ptr<SingleCamera> pSingleCamera(new SingleCamera(this));
        pSingleCamera->SetCamera(deviceList_, i);
        pSingleCameras_.push_back(pSingleCamera);
    }

    int i = -1;
    ros::Rate rate(4);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        pSerialReader_->PublishMsg();
        if(0 != camNum) {
            ++i;
            i %= camNum;
            pSingleCameras_[i]->PublishImageAndFreq();
        }
    }
}

double HikCameraManager::GetGpsTimeFromSerial() {
    LOG(INFO) << __FUNCTION__ << " start.";
    return pSerialReader_->GetGpsTime();
}


void HikCameraManager::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

