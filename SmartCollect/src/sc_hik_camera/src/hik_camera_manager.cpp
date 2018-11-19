#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

#include "hik_camera_manager.h"

HikCameraManager::HikCameraManager(ros::NodeHandle nh, ros::NodeHandle private_nh):
    threadPool_(30, 30)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    nh_ = nh;
    pubImages_.clear();
    pSingleCameras_.clear();
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

HikCameraManager::~HikCameraManager() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)DoClean();

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

        const std::string _cameraIP(pSingleCamera->GetCameraIP());
        LOG(INFO) << i << ":" << _cameraIP;
        (void)SetAdvertiseTopic(_cameraIP);
    }

    int i = -1;
    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        ++i;
        i %= camNum;

        PublishImage(i);
    }
}

void HikCameraManager::SetAdvertiseTopic(const std::string &advertiseName) {
    LOG(INFO) << __FUNCTION__ << " start; topic name: " << advertiseName;
    image_transport::Publisher pub;
    image_transport::ImageTransport it(nh_);
    pub = it.advertise("camera/image" + advertiseName, 0);

    pubImages_.push_back(pub);
    return;
}

void HikCameraManager::PublishImage(size_t index) {
    cv::Mat imageResized;
    // pSingleCamera->mat2PubMutex_.lock();
    cv::resize(pSingleCameras_[index]->mat2Pub_, imageResized, cv::Size(pSingleCameras_[index]->mat2Pub_.cols / 10, pSingleCameras_[index]->mat2Pub_.rows / 10));
    // pSingleCamera->mat2PubMutex_.unlock();

    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageResized).toImageMsg();
    pubImages_[index].publish(imgMsg);
}

void HikCameraManager::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

void HikCameraManager::DoClean() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.stop();
}
