#ifndef __INFOR_PROCESS_H
#define __INFOR_PROCESS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include "roscameragpsimg/imu5651.h"
#include "ntd_info_process/processed_infor_msg.h"
#include "../../public_tools/public_tools.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

using std::vector;
using std::string;
using std::istringstream;

class InforProcess {
public:
    InforProcess();
    ~InforProcess();
    void run();


private:
    void gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg);
    void velodyneCB(const std_msgs::String::ConstPtr& pVelodyneMsg);
    void rawImuCB(const std_msgs::String::ConstPtr& pRawImuMsg);
    double mGpsTime[2];
    ros::NodeHandle nh;
    ros::Subscriber mSub;
    ros::Subscriber mSubVelodyne;
    ros::Subscriber mSub422;
    ros::Publisher mPub;
    ntd_info_process::processed_infor_msg mOutMsg;
    const string PPS_STATUS[4] {
      "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };
    bool mIsVelodyneUpdated;
    bool mIsRawImuUpdated;
};


#endif // __INFOR_PROCESS_H