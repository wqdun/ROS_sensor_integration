#ifndef __CAMERA_H
#define __CAMERA_H

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <FlyCapture2.h>
#include <dlfcn.h>
#include <execinfo.h>

#include <pthread.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <sc_msgs/MonitorMsg.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "comm_timer.h"
#include "PGCamera.h"

class Cameras {
public:
    Cameras(ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string &_imgFormat, const std::string &_rawdataDir);
    ~Cameras();
    void run();

    double gpsWeekTimeCorrected_;
    bool isGpsTimeValid_;


private:
    void serverCB(const sc_msgs::MonitorMsg::ConstPtr& pClientMsg);
    void logErrorTrace(FlyCapture2::Error error);
    void getCameras(unsigned int _cameraNum, const std::string &__rawdataDir);
    void startAllCapture();
    void pubTopic(int _i);
    bool flyImage2msg(const FlyCapture2::Image &inFlyImage);
    bool convertImage(cv::Mat* matImage, const Image* image);
    int getMasterIndex();
    void setCameraFrameRate(double frameRate);
    void setCameraGigEPacketSize(GigECamera *_pGigECamera, unsigned int _packetSize);
    void setAllCamerasGigEPacketSize(unsigned int _packetSize);


    ros::Subscriber subServer_;
    ros::Publisher pubCamSpeed_;

    std::string imgFormat_;
    std::vector<CPGCamera *> pCpgCameras_;
    boost::shared_ptr<CommTimer> pCommTimer_;
    boost::shared_ptr<boost::thread> timeThread_;
    int8_t camGainLast_;
    cv::Mat cvMatImg_;
    sensor_msgs::ImagePtr msgImg_;
    int masterIndex_;
};

#endif
