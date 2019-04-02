#ifndef __HIK_CAMERA_MANAGER_H__
#define __HIK_CAMERA_MANAGER_H__

#include <glog/logging.h>
#include <vector>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <sys/types.h>
#include <sys/shm.h>
#include "include/MvCameraControl.h"
#include "../../sc_lib_public_tools/src/thread_pool.h"
#include "../../sc_lib_public_tools/src/shm_data.h"
#include "save_image_task.h"
#include "single_camera.h"
#include "comm_timer.h"
#include "sc_msgs/MonitorMsg.h"

class HikCameraManager {
public:
    HikCameraManager(const std::string &_rawPath);
    ~HikCameraManager();
    void Run();
    double GetUnixTimeMinusGpsTimeFromSerial();

    threadpool<SaveImageTask> threadPool_;
    std::string rawDataPath_;
    bool isSaveImg_;


private:
    std::vector<boost::shared_ptr<SingleCamera>> pSingleCameras_;
    MV_CC_DEVICE_INFO_LIST deviceList_;
    boost::shared_ptr<CommTimer> pSerialReader_;
    boost::shared_ptr<boost::thread> pSerialReaderThread_;
    ros::NodeHandle nh_;
    ros::Subscriber subMonitor_;
    struct SharedMem *sharedMem_;
    double unixTimeMinusGpsTime_;

    void DoClean();
    void PressEnterToExit();
    void MonitorCB(const sc_msgs::MonitorMsg::ConstPtr& pMonitorMsg);
    void RegisterCB();
};

#endif

