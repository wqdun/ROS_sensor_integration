#include "project_monitor.h"

ProjectMonitor::ProjectMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    LOG(INFO) << __FUNCTION__ << " start.";
    pub2web_ = nh.advertise<sc_msgs::DiskInfo>("sc_disk_info", 0);
    lastImgNum_ = 0;
}

ProjectMonitor::~ProjectMonitor() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye, projectMonitor.";
}

void ProjectMonitor::setRawdataPath(const std::string &_rawdataPath) {
    LOG(INFO) << __FUNCTION__ << " start, param: " << _rawdataPath;
    rawdataPath_ = _rawdataPath;
}

void ProjectMonitor::run() {
    sc_msgs::DiskInfo diskInfo;
    const double _rate = 0.5;
    ros::Rate rate(_rate);

    while(ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        std::vector<std::string> files_jpg;
        (void)public_tools::PublicTools::getFilesInDir(rawdataPath_, "jpg", files_jpg);
        DLOG(INFO) << "The number of jpg is " << files_jpg.size();
        diskInfo.img_num = files_jpg.size();
        diskInfo.img_save_fps = (diskInfo.img_num - lastImgNum_) * _rate;
        lastImgNum_ = diskInfo.img_num;

        std::vector<std::string> lidarSize;
        const std::string getLidarSizeCmd("du -sm " + rawdataPath_ + "/Lidar/ | awk '{print $1}'");
        (void)public_tools::PublicTools::PopenWithReturn(getLidarSizeCmd, lidarSize);
        diskInfo.lidar_size = (1 == lidarSize.size())? (public_tools::PublicTools::string2num(lidarSize[0], int32_t(-2))): -1;

        pub2web_.publish(diskInfo);
    }
}

