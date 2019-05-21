#ifndef __SERIAL_5651_H__
#define __SERIAL_5651_H__

#include <glog/logging.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include "base_serial.h"
#include "sc_msgs/imu5651.h"
#include "../../sc_lib_public_tools/src/tools_no_ros.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

class Serial5651: public BaseSerial {
public:
    virtual ~Serial5651() {
        LOG(INFO) << "Goodbye Serial5651..";
    }
    void Run();


private:
    void ReadSerial(int _fd);
    void WriteSerial(int _fd);
    void PublishMsg();

    void Parse5651Frame(const std::string &_frame, double _unixTime);
    void Parse5651GpggaFrame(const std::string &_gpggaFrame);
    void WriteRtImuFile(const std::string &_gpfpdFrame);
    bool Parse5651GpfpdFrame(const std::string &_gpfpdFrame, double __unixTime);

    ros::NodeHandle nh_;
    ros::Publisher pubImu5651_;
    sc_msgs::imu5651 imu232Msg_;
    std::string serialName_;
    std::string rtImuFile_;
};

#endif
