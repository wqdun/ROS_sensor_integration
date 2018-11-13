#include "hik_camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

HikCamera::HikCamera(ros::NodeHandle nh, ros::NodeHandle private_nh):
    threadPool_(10, 20)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    nh_ = nh;
    pubImages_.clear();
    pSingleCameras_.clear();
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

HikCamera::~HikCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
    (void)DoClean();

    LOG(INFO) << __FUNCTION__ << " end.";
}

void HikCamera::Run() {
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

    while(ros::ok()) {
        SingleCamera::s_matImageMutex_.lock();
        LOG(INFO) << "SingleCamera::s_time2Mat_.size(): " << SingleCamera::s_time2Mat_.size();
        if(SingleCamera::s_time2Mat_.empty()) {
            LOG_EVERY_N(INFO, 20) << "SingleCamera::s_time2Mat_.empty()";
            SingleCamera::s_matImageMutex_.unlock();
            usleep(200000);
            continue;
        }
        time2Mat_t _time2Mat = SingleCamera::s_time2Mat_.front();
        SingleCamera::s_time2Mat_.pop_front();
        SingleCamera::s_matImageMutex_.unlock();

        SaveImageTask *pSaveImageTask = new SaveImageTask(_time2Mat.header, _time2Mat.cameraIP, _time2Mat.matImage);
        threadPool_.append_task(pSaveImageTask);

        PublishImage(_time2Mat.cameraIndex, _time2Mat.matImage);
    }
}

void HikCamera::SetAdvertiseTopic(const std::string &advertiseName) {
    LOG(INFO) << __FUNCTION__ << " start.";
    image_transport::Publisher pub;
    image_transport::ImageTransport it(nh_);
    pub = it.advertise("camera/image" + advertiseName, 0);

    pubImages_.push_back(pub);
    return;
}

void HikCamera::PublishImage(size_t index, const cv::Mat &image2Pub) {
    cv::Mat imageResized;
    cv::resize(image2Pub, imageResized, cv::Size(image2Pub.cols / 10, image2Pub.rows / 10));
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imageResized).toImageMsg();

    DLOG(INFO) << "pubImages_[index].getTopic(), e.g., /camera/image6666; " << pubImages_[index].getTopic();
    pubImages_[index].publish(imgMsg);
}

void HikCamera::PressEnterToExit() {
    LOG(INFO) << "Wait enter to stop grabbing.";
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while(getchar() != '\n');
}

void HikCamera::DoClean() {
    LOG(INFO) << __FUNCTION__ << " start.";
    threadPool_.stop();
}
