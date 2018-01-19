#ifndef __INFOR_PROCESS_H
#define __INFOR_PROCESS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include "roscameragpsimg/imu5651.h"
#include "hdop_teller/imu5651_422.h"
#include "ntd_info_process/processed_infor_msg.h"
#include "velodyne_msgs/Velodyne2Center.h"
#include "../../public_tools/public_tools.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>

using std::vector;
using std::string;
using std::istringstream;

class InforProcess {
public:
    InforProcess(const string &_eventFilePath);
    ~InforProcess();
    void run();


private:
    void gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg);
    void velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg);
    void rawImuCB(const hdop_teller::imu5651_422::ConstPtr& pRawImuMsg);
    void cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void myVizCB(const std_msgs::Int64::ConstPtr& pMyVizMsg);
    double mGpsTime[2];
    ros::NodeHandle nh;
    ros::Subscriber mSub;
    ros::Subscriber mSubVelodyne;
    ros::Subscriber mSub422;
    ros::Subscriber mSubCameraImg;
    ros::Subscriber mSubMyViz;

    ros::Publisher mPub;
    ros::Publisher mPubIsSaveFile;

    ntd_info_process::processed_infor_msg mOutMsg;
    const string PPS_STATUS[4] {
      "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };
    bool mIsVelodyneUpdated;
    bool mIsRawImuUpdated;
    bool mIsGpsUpdated;
    string eventFilePath_;
};


#endif // __INFOR_PROCESS_H