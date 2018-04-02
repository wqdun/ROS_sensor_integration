#ifndef __SERVER_DAEMON_H
#define __SERVER_DAEMON_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "sc_server_daemon/clientCmd.h"
#include "sc_server_daemon/serverMsg.h"
#include "sc_server_daemon/monitorMsg.h"
#include "sc_server_daemon/projectInfoMsg.h"
#include "roscameragpsimg/imu5651.h"
#include "velodyne_msgs/Velodyne2Center.h"
#include "sc_integrate_imu_recorder/scIntegrateImu.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

class ServerDaemon {
public:
    ServerDaemon(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~ServerDaemon();
    void run();


private:
    ros::Subscriber subClient_;
    ros::Publisher pub2scNodes_;

    ros::Subscriber sub232_;
    ros::Subscriber subVelodyne_;
    ros::Subscriber sub422_;
    ros::Subscriber subCameraImg_;
    ros::Subscriber subServer_;
    ros::Publisher pub2client_;

    ros::Publisher pub2broswer_;

    sc_server_daemon::nodeParams nodeParams_;
    sc_server_daemon::monitorMsg monitorMsg_;
    sc_server_daemon::projectInfoMsg projectInfoMsg_;

    double mGpsTime[2];
    const std::string PPS_STATUS[4] {
        "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };


    void clientCB(const sc_server_daemon::clientCmd::ConstPtr& pClientMsg);

    bool isGpsUpdated_, isVelodyneUpdated_, isRawImuUpdated_, isCameraUpdated_;
    void gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg);
    void velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg);
    void rawImuCB(const sc_integrate_imu_recorder::scIntegrateImu::ConstPtr& pRawImuMsg);
    void cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void updateprojectInfoMsg(const std::string &projectInfo);
};

#endif // __SERVER_DAEMON_H
