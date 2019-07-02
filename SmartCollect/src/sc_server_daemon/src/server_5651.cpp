#include "server_5651.h"

Server5651::Server5651() {
    gpsTime_.resize(2);
    gpsTime_[0].clear();
    gpsTime_[1].clear();

    // below for SC control
    _monitorMsg_.is_record = 0;
    _monitorMsg_.cam_gain = 20;
    _monitorMsg_.pps_HWcheck = -1;
    _monitorMsg_.gprmc_HWcheck = -1;
    _monitorMsg_.imu_HWcheck = -1;
    _monitorMsg_.is_disk_error = false;

    isGpsUpdated_ = isVelodyneUpdated_ = isRawImuUpdated_
        = isCamera0FpsUpdated_ = isCamera1FpsUpdated_ = isCamera2FpsUpdated_
        = isDiskInfoUpdated_ = false;

    projectInfo_.clear();
    pDiskMonitor_.reset(new DiskMonitor() );

    int shmid = shmget((key_t)1234, sizeof(struct SharedMem), 0666 | IPC_CREAT);
    if(shmid < 0) {
        LOG(ERROR) << "shget failed.";
        exit(1);
    }
    void *shm = shmat(shmid, 0, 0);
    if(shm == (void *)-1) {
        LOG(ERROR) << "shmat failed.";
        exit(1);
    }
    LOG(INFO) << "Memory attached at " << shm;
    sharedMem_ = (struct SharedMem*)shm;
}

Server5651::~Server5651() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

void Server5651::RegisterCBs() {
    subClient_ = nh_.subscribe("sc_client_cmd", 1, &BaseServer::clientCB, this);
    subSerial_ = nh_.subscribe("imu_string", 10, &BaseServer::SerialCB, this);
    subVelodyne_ = nh_.subscribe("velodyne_pps_status", 10, &BaseServer::velodyneCB, this);
    subCamera0Fps_ = nh_.subscribe("cam_speed5555", 10, &BaseServer::Camera0FpsCB, this);
    subCamera1Fps_ = nh_.subscribe("cam_speed6666", 10, &BaseServer::Camera1FpsCB, this);
    subCamera2Fps_ = nh_.subscribe("cam_speed7777", 10, &BaseServer::Camera2FpsCB, this);
    subProjectMonitor_ = nh_.subscribe("sc_disk_info", 10, &BaseServer::projectMonitorCB, this);
    subDataFixer_ = nh_.subscribe("sc_data_fixer_progress", 10, &BaseServer::dataFixerCB, this);

    pub2client_ = nh_.advertise<sc_msgs::MonitorMsg>("sc_monitor", 10);
}

void Server5651::Run() {
    const size_t HOSTNAME_SIZE = 60;
    char hostname[HOSTNAME_SIZE];
    bzero(hostname, HOSTNAME_SIZE);
    (void)gethostname(hostname, HOSTNAME_SIZE);
    _monitorMsg_.host_name = hostname;
    RegisterCBs();

    ros::Rate rate(2);
    size_t freqDivider = 0;
    bool isScTimeCalibrated = false;
    while(ros::ok() ) {
        ++freqDivider;
        freqDivider %= 256;
        ros::spinOnce();
        rate.sleep();

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            if(!isGpsUpdated_) {
                LOG(INFO) << "RS232 node not running.";
                _monitorMsg_.GPStime = _monitorMsg_.lat_lon_hei.x = _monitorMsg_.lat_lon_hei.y = _monitorMsg_.lat_lon_hei.z = _monitorMsg_.pitch_roll_heading.x = _monitorMsg_.pitch_roll_heading.y = _monitorMsg_.pitch_roll_heading.z = _monitorMsg_.speed = -2.;
                _monitorMsg_.hdop = -2;
            }
            isGpsUpdated_ = false;
        }

        // 1Hz
        if(0 == (freqDivider % 2) ) {
            if(!isVelodyneUpdated_) {
                DLOG(INFO) << "velodyne node not running.";
                _monitorMsg_.pps_status = _monitorMsg_.is_gprmc_valid = "";
            }
            isVelodyneUpdated_ = false;
        }

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            _monitorMsg_.is_cameras_good = (
                isCamera0FpsUpdated_
                && isCamera1FpsUpdated_
                && isCamera2FpsUpdated_
                && IsFpsGood(camera0Fps_, camera1Fps_, camera2Fps_)
            );

            if(!isCamera0FpsUpdated_) {
                DLOG(INFO) << "Failed to get camera 0 fps.";
                camera0Fps_ = 0;
            }
            isCamera0FpsUpdated_ = false;

            if(!isCamera1FpsUpdated_) {
                DLOG(INFO) << "Failed to get camera 1 fps.";
                camera1Fps_ = 0;
            }
            isCamera1FpsUpdated_ = false;

            if(!isCamera2FpsUpdated_) {
                DLOG(INFO) << "Failed to get camera 2 fps.";
                camera2Fps_ = 0;
            }
            isCamera2FpsUpdated_ = false;

            _monitorMsg_.camera_fps = (camera0Fps_ + camera1Fps_ + camera2Fps_) / 3.;
            _monitorMsg_.camera0_fps = camera0Fps_;
            _monitorMsg_.camera1_fps = camera1Fps_;
            _monitorMsg_.camera2_fps = camera2Fps_;
        }

        // 0.25Hz
        if(0 == (freqDivider % 8) ) {
            if(!isDiskInfoUpdated_) {
                DLOG(INFO) << "project monitor node not running.";
                _monitorMsg_.img_num = _monitorMsg_.lidar_size = -2;
            }
            isDiskInfoUpdated_ = false;
        }

        _monitorMsg_.projects.clear();
        _monitorMsg_.disk_usage.clear();
        (void)pDiskMonitor_->Run("/opt/smartc/record/", _monitorMsg_);

        if(public_tools::PublicTools::isFileExist("/tmp/data_fixer_progress_100%") ) {
            _monitorMsg_.process_num = _monitorMsg_.total_file_num;
        }

        struct timeval now;
        gettimeofday(&now, NULL);
        _monitorMsg_.unix_time = now.tv_sec + now.tv_usec / 1000000.;

        if(!isScTimeCalibrated) {
            if(IsGpsTimeGood()) {
                if(IsScTimeBad() ) {
                    SetScTimeByGpsTime();
                    LOG(INFO) << "SC time is calibrated.";
                }
                else {
                    LOG(INFO) << "SC time is good.";
                }
                isScTimeCalibrated = true;
            }
            else {
                LOG(INFO) << "Wait GPS time till good.";
            }
        }

        _monitorMsg_.sc_check_camera_num = sharedMem_->cameraNum;
        _monitorMsg_.sc_check_imu_serial_port = sharedMem_->imuSerialPortStatus;

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            LogSystemStatus();
        }

        pub2client_.publish(_monitorMsg_);
    }
}

void Server5651::LogSystemStatus() {
    std::vector<std::string> memCpuStatus;
    memCpuStatus.clear();
    const std::string getMemCpuStatusCmd("/usr/bin/vmstat -w");
    (void)public_tools::PublicTools::PopenWithReturn(getMemCpuStatusCmd, memCpuStatus);
    for (const auto &memCpu: memCpuStatus) {
        LOG(INFO) << "memCpuStatus: " << memCpu;
    }

    std::vector<std::string> netStatus;
    netStatus.clear();
    const std::string getNetStatusCmd("/sbin/ifconfig");
    (void)public_tools::PublicTools::PopenWithReturn(getNetStatusCmd, netStatus);
    for (const auto &net: netStatus) {
        LOG(INFO) << "netStatus: " << net;
    }
}

bool Server5651::IsFpsGood(double a, double b, double c) {
    return (
        (IsFpsEqual(a, b))
        && (IsFpsEqual(a, c))
        && (IsFpsEqual(b, c))
    );
}

bool Server5651::IsFpsEqual(double a, double b) {
    const double FACTOR = 0.4;
    return (((a - b) > -FACTOR) && ((a - b) < FACTOR));
}

int Server5651::SetScTimeByGpsTime() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const double gpsUnixTime = public_tools::ToolsNoRos::CalcUnixTimeByGpsWeek(_monitorMsg_.GPSweek, _monitorMsg_.GPStime);

    timeval tv;
    tv.tv_sec = static_cast<time_t>(gpsUnixTime);
    tv.tv_usec = 0;

    settimeofday(&tv, NULL);
    (void)public_tools::PublicTools::PopenWithoutReturn("/sbin/hwclock -w");
    RestartSelf();
    LOG(INFO) << "gpsUnixTime: " << std::fixed << gpsUnixTime << " set; errno: " << errno;
    return 0;
}

bool Server5651::IsGpsTimeGood() {
    return ("A" == _monitorMsg_.is_gprmc_valid) && (_monitorMsg_.GPStime > 1.);
}

void Server5651::RestartSelf() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
    (void)public_tools::PublicTools::PopenWithoutReturn("bash " + launchScript + " restart_server " + projectInfo_ + " &");

    LOG(INFO) << __FUNCTION__ << " end.";
}

bool Server5651::IsScTimeBad() {
    double errSecond = fabs(_monitorMsg_.unix_time - _monitorMsg_.GPStime);
    errSecond = fmod(errSecond, 24 * 3600);
    return (errSecond > 1000 && errSecond < 85400);
}

