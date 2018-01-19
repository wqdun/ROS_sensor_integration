#ifndef __INFOR_PROCESS_H
#define __INFOR_PROCESS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <vector>
using std::vector;
#include "roscameragpsimg/imu5651.h"
#include "hdop_teller/imu5651_422.h"
#include "ntd_info_process/processed_infor_msg.h"
#include "ntd_info_process/imuPoints.h"
#include "../../public_tools/public_tools.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using std::string;
using std::istringstream;

typedef struct {
    double daytime;
    public_tools::pointXYZ_t point;
} time2point_t;

class InforProcess {
public:
    InforProcess();
    ~InforProcess();
    void run();


private:
    void gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg);
    void velodyneCB(const std_msgs::String::ConstPtr& pVelodyneMsg);
    void rawImuCB(const hdop_teller::imu5651_422::ConstPtr& pRawImuMsg);
    void cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    double mGpsTime[2];
    ros::NodeHandle nh;
    ros::Subscriber mSub;
    ros::Subscriber mSubVelodyne;
    ros::Subscriber mSub422;
    ros::Subscriber mSubCameraImg;
    ros::Publisher mPub;
    ros::Publisher pubTime2Local_;
    ntd_info_process::processed_infor_msg mOutMsg;
    const string PPS_STATUS[4] {
      "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };
    bool mIsVelodyneUpdated;
    bool mIsRawImuUpdated;
    bool mIsGpsUpdated;

    ntd_info_process::imuPoints time2LocalMsg_;
};


#endif // __INFOR_PROCESS_H