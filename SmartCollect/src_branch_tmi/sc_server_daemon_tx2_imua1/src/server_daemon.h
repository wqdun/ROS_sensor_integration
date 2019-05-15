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
#include "sc_msgs/Novatel.h"
#include "velodyne_msgs/Velodyne2Center.h"
#include "sc_msgs/scIntegrateImu.h"
#include "sc_msgs/DataFixerProgress.h"
#include "../../sc_lib_public_tools/src/public_tools.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "../../sc_lib_public_tools/src/shm_data.h"
#include "disk_monitor.h"

class ServerDaemon {
public:
    ServerDaemon();
    ~ServerDaemon();
    void Run();
    void UpdateProjectInfo(const std::string &projectInfo);


private:
    ros::NodeHandle nh_;
    ros::Subscriber subClient_;
    ros::Subscriber subSerial_;
    ros::Subscriber subVelodyne_;
    ros::Subscriber sub422_;
    ros::Subscriber subCamera0Fps_, subCamera1Fps_, subCamera2Fps_;
    ros::Subscriber subProjectMonitor_;
    ros::Subscriber subDataFixer_;
    ros::Publisher pub2client_;

    sc_msgs::MonitorMsg monitorMsg_;
    sc_msgs::ProjectInfoMsg projectInfoMsg_;

    std::vector<double> gpsTime_;
    const std::string PPS_STATUS[4] {
        "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    };
    struct SharedMem *sharedMem_;

    boost::shared_ptr<DiskMonitor> pDiskMonitor_;
    std::string projectInfo_;

    double camera0Fps_, camera1Fps_, camera2Fps_;

    void clientCB(const sc_msgs::ClientCmd::ConstPtr& pClientMsg);

    bool isGpsUpdated_, isVelodyneUpdated_, isRawImuUpdated_, isCamera0FpsUpdated_, isCamera1FpsUpdated_, isCamera2FpsUpdated_, isDiskInfoUpdated_;
    void ImuA1CB(const sc_msgs::Novatel::ConstPtr& pNovatelMsg);
    void velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg);
    void Camera0FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void Camera1FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void Camera2FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg);
    void projectMonitorCB(const sc_msgs::DiskInfo::ConstPtr& pDiskInfoMsg);
    void dataFixerCB(const sc_msgs::DataFixerProgress::ConstPtr& pDataFixerProgressMsg);
    void CheckHardware();
    void CheckLidar();
    void CheckCamera();
    void CheckImu();
    void ParsePositionPkt(const char *pkt);
    void CheckDiskCapacity();
    int SetScTimeByGpsTime();
    bool IsGpsTimeGood();
    bool IsScTimeBad();
    void RestartSelf();
    void RegisterCBs();
    bool IsFpsEqual(double a, double b);
    bool IsFpsGood(double a, double b, double c);
};

#endif // __SERVER_DAEMON_H
