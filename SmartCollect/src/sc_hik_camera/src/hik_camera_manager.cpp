#include "hik_camera_manager.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

HikCameraManager::HikCameraManager(ros::NodeHandle nh, ros::NodeHandle private_nh):
    threadPool_(20, 20)
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
    LOG(INFO) << "Find " << deviceList_.nDeviceNum << " Devices.";

    for(size_t i = 0; i < deviceList_.nDeviceNum; ++i) {
        boost::shared_ptr<SingleCamera> pSingleCamera(new SingleCamera);
        pSingleCamera->SetCamera(deviceList_, i);
        pSingleCameras_.push_back(pSingleCamera);

        const std::string _cameraIP(pSingleCamera->GetCameraIP());
        LOG(INFO) << i << ":" << _cameraIP;
        (void)SetAdvertiseTopic(_cameraIP);
    }

    // ros::Rate rate(20);
    while(ros::ok()) {
        // ros::spinOnce();
        // rate.sleep();
        SingleCamera::s_matImageMutex_.lock();
        if(SingleCamera::s_time2Mat_.empty()) {
            LOG_EVERY_N(INFO, 500000) << "SingleCamera::s_time2Mat_.empty()";
            SingleCamera::s_matImageMutex_.unlock();
            continue;
        }
        time2Mat_t _time2Mat = SingleCamera::s_time2Mat_.front();
        SingleCamera::s_time2Mat_.pop_front();
        SingleCamera::s_matImageMutex_.unlock();

        SaveImageTask *pSaveImageTask = new SaveImageTask(_time2Mat.header, _time2Mat.cameraIP, _time2Mat.frameNum, _time2Mat.matImage);
        threadPool_.append_task(pSaveImageTask);

        PublishImage(_time2Mat.cameraIndex, _time2Mat.matImage);
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

void HikCameraManager::PublishImage(size_t index, const cv::Mat &image2Pub) {
    cv::Mat imageResized;
    cv::resize(image2Pub, imageResized, cv::Size(image2Pub.cols / 10, image2Pub.rows / 10));
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageResized).toImageMsg();

    DLOG(INFO) << "pubImages_[index].getTopic(), e.g., /camera/image6666; " << pubImages_[index].getTopic();
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
