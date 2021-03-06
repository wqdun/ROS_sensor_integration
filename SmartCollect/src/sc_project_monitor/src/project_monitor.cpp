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

        int jpgFilesCount = 0;
        (void)public_tools::PublicTools::GetFilesCountInDir(rawdataPath_ + "/Image/", "jpg", jpgFilesCount);
        DLOG(INFO) << "The number of jpg is " << jpgFilesCount;
        diskInfo.img_num = jpgFilesCount;
        diskInfo.img_save_fps = (diskInfo.img_num - lastImgNum_) * _rate;
        lastImgNum_ = diskInfo.img_num;

        std::vector<std::string> lidarSize;
        const std::string getLidarSizeCmd("du -sm " + rawdataPath_ + "/Lidar/ | awk '{print $1}'");
        (void)public_tools::PublicTools::PopenWithReturn(getLidarSizeCmd, lidarSize);
        diskInfo.lidar_size = (1 == lidarSize.size())? (public_tools::PublicTools::string2num(lidarSize[0], int32_t(-2))): -1;

        diskInfo.raw_ins_size = GetRawInsSizeInByte();
        diskInfo.timestamp_size = GetTimestampFileSizeInByte();

        pub2web_.publish(diskInfo);
    }
}

long ProjectMonitor::GetRawInsSizeInByte() {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> files_dat;
    (void)public_tools::PublicTools::getFilesInDir(rawdataPath_ + "/IMU/", "dat", files_dat);
    if(files_dat.empty() ) {
        LOG(WARNING) << "Failed to find dat file in " << rawdataPath_;
        return 0;
    }

    return public_tools::ToolsNoRos::GetFileSizeInByte(files_dat[0]);
}

long ProjectMonitor::GetTimestampFileSizeInByte() {
    DLOG(INFO) << __FUNCTION__ << " start.";

    std::vector<std::string> timestampFile;
    (void)public_tools::PublicTools::getFilesInDir(rawdataPath_ + "/IMU/", "timestamp", timestampFile);
    if(timestampFile.empty() ) {
        LOG(WARNING) << "Failed to find timestamp file in " << rawdataPath_;
        return 0;
    }

    return public_tools::ToolsNoRos::GetFileSizeInByte(timestampFile[0]);
}


