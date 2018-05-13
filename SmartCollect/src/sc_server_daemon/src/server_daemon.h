#ifndef __SERVER_DAEMON_H
#define __SERVER_DAEMON_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "sc_msgs/ClientCmd.h"
#include "sc_msgs/ServerMsg.h"
#include "sc_msgs/MonitorMsg.h"
#include "sc_msgs/ProjectInfoMsg.h"
#include "sc_msgs/DiskInfo.h"
#include "roscameragpsimg/imu5651.h"
#include "velodyne_msgs/Velodyne2Center.h"
#include "sc_msgs/scIntegrateImu.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "dataFixed.h"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "disk_monitor.h"

class ServerDaemon {
public:
    ServerDaemon(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~ServerDaemon();
    void run();


private:
    ros::Subscriber subClient_;

    ros::Subscriber sub232_;
    ros::Subscriber subVelodyne_;
    ros::Subscriber sub422_;
    ros::Subscriber subCameraImg_;
    ros::Subscriber subProjectMonitor_;
    ros::Subscriber subServer_;
    ros::Publisher pub2client_;

    sc_msgs::MonitorMsg monitorMsg_;
    sc_msgs::ProjectInfoMsg projectInfoMsg_;

    double mGpsTime[2];
    const std::string PPS_STATUS[4] {
        "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };

    boost::shared_ptr<dataFixed> pDataFixer_;
    boost::shared_ptr<DiskMonitor> pDiskMonitor_;


    void clientCB(const sc_msgs::ClientCmd::ConstPtr& pClientMsg);

    bool isGpsUpdated_, isVelodyneUpdated_, isRawImuUpdated_, isCameraUpdated_, isDiskInfoUpdated_;
    void gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg);
    void velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg);
    void rawImuCB(const sc_msgs::scIntegrateImu::ConstPtr& pRawImuMsg);
    void cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void projectMonitorCB(const sc_msgs::DiskInfo::ConstPtr& pDiskInfoMsg);
    void updateProjectInfo(const std::string &projectInfo);
};

#endif // __SERVER_DAEMON_H
