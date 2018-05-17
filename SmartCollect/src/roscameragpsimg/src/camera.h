#ifndef __CAMERA_H
#define __CAMERA_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <FlyCapture2.h>
#include "comm_timer.h"
#include "dlfcn.h"
#include <execinfo.h>

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <ctime>

#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include "PGCamera.h"
#include <sc_msgs/MonitorMsg.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

class Cameras {
public:
    Cameras(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Cameras();
    void run();

    std::string imgFormat_;
    std::string rawdataPath_;


private:
    void serverCB(const sc_msgs::MonitorMsg::ConstPtr& pClientMsg);
    void logErrorTrace(FlyCapture2::Error error);
    void getCameras(unsigned int _cameraNum);
    void startAll();



    ros::Subscriber subServer_;

    std::vector<CPGCamera *> pCpgCameras_;
    boost::shared_ptr<CommTimer> pCommTimer_;
    int8_t camGainLast_;



};

#endif
