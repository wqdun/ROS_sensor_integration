#ifndef __SERVER_DAEMON_H
#define __SERVER_DAEMON_H

#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <netinet/in.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/shm.h>
#include "sc_msgs/ClientCmd.h"
#include "sc_msgs/ServerMsg.h"
#include "sc_msgs/MonitorMsg.h"
#include "sc_msgs/ProjectInfoMsg.h"
#include "sc_msgs/DiskInfo.h"
#include "sc_msgs/imu5651.h"
#include "velodyne_msgs/Velodyne2Center.h"
#include "sc_msgs/scIntegrateImu.h"
#include "sc_msgs/DataFixerProgress.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "../../sc_lib_public_tools/src/shm_data.h"
#include "disk_monitor.h"

class ServerDaemon {
public:
    ServerDaemon(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~ServerDaemon();
    void run();


private:
    ros::Subscriber subClient_;

    ros::Subscriber subSerial_;
    ros::Subscriber subVelodyne_;
    ros::Subscriber sub422_;
    ros::Subscriber subCameraImg_;
    ros::Subscriber subProjectMonitor_;
    ros::Subscriber subDataFixer_;
    ros::Publisher pub2client_;

    sc_msgs::MonitorMsg monitorMsg_;
    sc_msgs::ProjectInfoMsg projectInfoMsg_;

    std::vector<std::string> gpsTime_;
    const std::string PPS_STATUS[4] {
        "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };
    struct SharedMem *sharedMem_;

    boost::shared_ptr<DiskMonitor> pDiskMonitor_;

    void clientCB(const sc_msgs::ClientCmd::ConstPtr& pClientMsg);

    bool isGpsUpdated_, isVelodyneUpdated_, isRawImuUpdated_, isCameraUpdated_, isDiskInfoUpdated_;
    void SerialCB(const sc_msgs::imu5651::ConstPtr& pImu5651Msg);
    void velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg);
    void cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void projectMonitorCB(const sc_msgs::DiskInfo::ConstPtr& pDiskInfoMsg);
    void dataFixerCB(const sc_msgs::DataFixerProgress::ConstPtr& pDataFixerProgressMsg);
    void CheckHardware();
    void CheckLidar();
    void CheckCamera();
    void CheckImu();
    void ParsePositionPkt(const char *pkt);
    void CheckDiskCapacity();

    void updateProjectInfo(const std::string &projectInfo);
};

#endif // __SERVER_DAEMON_H
