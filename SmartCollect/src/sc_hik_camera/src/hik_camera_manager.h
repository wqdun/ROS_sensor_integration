#ifndef __HIK_CAMERA_MANAGER_H__
#define __HIK_CAMERA_MANAGER_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <deque>
#include <assert.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "include/MvCameraControl.h"
#include "../../sc_lib_public_tools/src/thread_pool.h"
#include "save_image_task.h"
#include "single_camera.h"

class HikCameraManager {
public:
    HikCameraManager(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~HikCameraManager();
    void Run();

    threadpool<SaveImageTask> threadPool_;



private:
    ros::NodeHandle nh_;
    std::vector<image_transport::Publisher> pubImages_;
    std::vector<boost::shared_ptr<SingleCamera>> pSingleCameras_;
    MV_CC_DEVICE_INFO_LIST deviceList_;

    void SetAdvertiseTopic(const std::string &advertiseName);
    void DoClean();
    void PressEnterToExit();
    void PublishImage(size_t index, const cv::Mat &image2Pub);
};

#endif

