#ifndef __INFOR_PROCESS_H
#define __INFOR_PROCESS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include "roscameragpsimg/imu5651.h"
#include "ntd_info_process/processed_infor_msg.h"
#include "../../public_tools/public_tools.h"

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

    double mGpsTime[2];
    ros::NodeHandle nh;
    ros::Subscriber mSub;
    ros::Publisher mPub;
    ntd_info_process::processed_infor_msg mOutMsg;
};


#endif // __INFOR_PROCESS_H