#ifndef __INFOR_PROCESS_H
#define __INFOR_PROCESS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <visualization_msgs/Marker.h>
#include <vector>
using std::vector;
#include "roscameragpsimg/imu5651.h"
#include "sc_integrate_imu_recorder/scIntegrateImu.h"
#include "sc_center/centerMsg.h"
#include "sc_center/imuPoints.h"
#include "velodyne_msgs/Velodyne2Center.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>

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
    void velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg);
    void rawImuCB(const sc_integrate_imu_recorder::scIntegrateImu::ConstPtr& pRawImuMsg);
    void cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void myVizCB(const std_msgs::Int64::ConstPtr& pMyVizMsg);

    double mGpsTime[2];
    ros::NodeHandle nh;
    ros::Subscriber mSub232;
    ros::Subscriber mSubVelodyne;
    ros::Subscriber mSub422;
    ros::Subscriber mSubCameraImg;
    ros::Subscriber mSubMyViz;

    ros::Publisher mPub;
    ros::Publisher pubTime2Local_;
    ros::Publisher mPubIsSaveFile;

    sc_center::centerMsg mOutMsg;
    const string PPS_STATUS[4] {
      "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };
    bool mIsVelodyneUpdated;
    bool mIsRawImuUpdated;
    bool mIsGpsUpdated;
    bool mIsServerConnected;
    sc_center::imuPoints time2LocalMsg_;
};


#endif // __INFOR_PROCESS_H