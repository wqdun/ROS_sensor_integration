#include "server_daemon.h"

ServerDaemon::ServerDaemon() {
    gpsTime_.resize(2);
    gpsTime_[0].clear();
    gpsTime_[1].clear();

    // below for SC control
    monitorMsg_.is_record = 0;
    monitorMsg_.cam_gain = 20;
    monitorMsg_.pps_HWcheck = -1;
    monitorMsg_.gprmc_HWcheck = -1;
    monitorMsg_.imu_HWcheck = -1;
    monitorMsg_.is_disk_error = false;

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

ServerDaemon::~ServerDaemon() {}

void ServerDaemon::RegisterCBs() {
    subClient_ = nh_.subscribe("sc_client_cmd", 1, &ServerDaemon::clientCB, this);
    subSerial_ = nh_.subscribe("imu_string", 10, &ServerDaemon::SerialCB, this);
    subVelodyne_ = nh_.subscribe("velodyne_pps_status", 10, &ServerDaemon::velodyneCB, this);
    subCamera0Fps_ = nh_.subscribe("cam_speed5555", 10, &ServerDaemon::Camera0FpsCB, this);
    subCamera1Fps_ = nh_.subscribe("cam_speed6666", 10, &ServerDaemon::Camera1FpsCB, this);
    subCamera2Fps_ = nh_.subscribe("cam_speed7777", 10, &ServerDaemon::Camera2FpsCB, this);
    subProjectMonitor_ = nh_.subscribe("sc_disk_info", 10, &ServerDaemon::projectMonitorCB, this);
    subDataFixer_ = nh_.subscribe("sc_data_fixer_progress", 10, &ServerDaemon::dataFixerCB, this);

    pub2client_ = nh_.advertise<sc_msgs::MonitorMsg>("sc_monitor", 10);
}

void ServerDaemon::Run() {
    const size_t HOSTNAME_SIZE = 60;
    char hostname[HOSTNAME_SIZE];
    bzero(hostname, HOSTNAME_SIZE);
    (void)gethostname(hostname, HOSTNAME_SIZE);
    monitorMsg_.host_name = hostname;
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
                monitorMsg_.GPStime = monitorMsg_.lat_lon_hei.x = monitorMsg_.lat_lon_hei.y = monitorMsg_.lat_lon_hei.z = monitorMsg_.pitch_roll_heading.x = monitorMsg_.pitch_roll_heading.y = monitorMsg_.pitch_roll_heading.z = monitorMsg_.speed = -2.;
                monitorMsg_.hdop = -2;
            }
            isGpsUpdated_ = false;
        }

        // 1Hz
        if(0 == (freqDivider % 2) ) {
            if(!isVelodyneUpdated_) {
                DLOG(INFO) << "velodyne node not running.";
                monitorMsg_.pps_status = monitorMsg_.is_gprmc_valid = "";
            }
            isVelodyneUpdated_ = false;
        }

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            monitorMsg_.is_cameras_good = (
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

            monitorMsg_.camera_fps = (camera0Fps_ + camera1Fps_ + camera2Fps_) / 3.;
            monitorMsg_.camera0_fps = camera0Fps_;
            monitorMsg_.camera1_fps = camera1Fps_;
            monitorMsg_.camera2_fps = camera2Fps_;
        }

        // 0.25Hz
        if(0 == (freqDivider % 8) ) {
            if(!isDiskInfoUpdated_) {
                DLOG(INFO) << "project monitor node not running.";
                monitorMsg_.img_num = monitorMsg_.lidar_size = -2;
            }
            isDiskInfoUpdated_ = false;
        }

        monitorMsg_.projects.clear();
        monitorMsg_.disk_usage.clear();
        (void)pDiskMonitor_->Run("/opt/smartc/record/", monitorMsg_);

        if(public_tools::PublicTools::isFileExist("/tmp/data_fixer_progress_100%") ) {
            monitorMsg_.process_num = monitorMsg_.total_file_num;
        }

        struct timeval now;
        gettimeofday(&now, NULL);
        monitorMsg_.unix_time = now.tv_sec + now.tv_usec / 1000000.;

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

        monitorMsg_.sc_check_camera_num = sharedMem_->cameraNum;
        monitorMsg_.sc_check_imu_serial_port = sharedMem_->imuSerialPortStatus;

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            LogSystemStatus();
        }

        pub2client_.publish(monitorMsg_);
    }
}

void ServerDaemon::LogSystemStatus() {
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

bool ServerDaemon::IsFpsGood(double a, double b, double c) {
    return (
        (IsFpsEqual(a, b))
        && (IsFpsEqual(a, c))
        && (IsFpsEqual(b, c))
    );
}

bool ServerDaemon::IsFpsEqual(double a, double b) {
    const double FACTOR = 0.4;
    return (((a - b) > -FACTOR) && ((a - b) < FACTOR));
}

int ServerDaemon::SetScTimeByGpsTime() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const double gpsUnixTime = public_tools::ToolsNoRos::CalcUnixTimeByGpsWeek(monitorMsg_.GPSweek, monitorMsg_.GPStime);

    timeval tv;
    tv.tv_sec = static_cast<time_t>(gpsUnixTime);
    tv.tv_usec = 0;

    settimeofday(&tv, NULL);
    (void)public_tools::PublicTools::PopenWithoutReturn("/sbin/hwclock -w");
    RestartSelf();
    LOG(INFO) << "gpsUnixTime: " << std::fixed << gpsUnixTime << " set; errno: " << errno;
    return 0;
}

bool ServerDaemon::IsGpsTimeGood() {
    return ("A" == monitorMsg_.is_gprmc_valid) && (monitorMsg_.GPStime > 1.);
}

void ServerDaemon::RestartSelf() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
    (void)public_tools::PublicTools::PopenWithoutReturn("bash " + launchScript + " restart_server " + projectInfo_ + " &");

    LOG(INFO) << __FUNCTION__ << " end.";
}

bool ServerDaemon::IsScTimeBad() {
    double errSecond = fabs(monitorMsg_.unix_time - monitorMsg_.GPStime);
    errSecond = fmod(errSecond, 24 * 3600);
    return (errSecond > 1000 && errSecond < 85400);
}

void ServerDaemon::dataFixerCB(const sc_msgs::DataFixerProgress::ConstPtr& pDataFixerProgressMsg) {
    DLOG(INFO) << __FUNCTION__ << " start: " << pDataFixerProgressMsg->processNum << "/" << pDataFixerProgressMsg->totalFileNum;

    monitorMsg_.total_file_num = pDataFixerProgressMsg->totalFileNum;
    monitorMsg_.process_num = pDataFixerProgressMsg->processNum;
}

void ServerDaemon::Camera0FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCamera0FpsUpdated_ = true;
    camera0Fps_ = pCameraImgMsg->data;
}

void ServerDaemon::Camera1FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCamera1FpsUpdated_ = true;
    camera1Fps_ = pCameraImgMsg->data;
}

void ServerDaemon::Camera2FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCamera2FpsUpdated_ = true;
    camera2Fps_ = pCameraImgMsg->data;
}

void ServerDaemon::velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg) {
    // 10Hz
    DLOG(INFO) << __FUNCTION__ << " start.";
    isVelodyneUpdated_ = true;

    if( (pVelodyneMsg->pps_status_index) > 3) {
        LOG(ERROR) << "Invalid PPS status: " << pVelodyneMsg->pps_status_index;
        exit(1);
    }

    monitorMsg_.pps_status = PPS_STATUS[pVelodyneMsg->pps_status_index];
    // A validity - A-ok, V-invalid, refer VLP-16 manual
    monitorMsg_.is_gprmc_valid = pVelodyneMsg->is_gprmc_valid;
    monitorMsg_.velodyne_rpm = pVelodyneMsg->velodyne_rpm;
}

// void ServerDaemon::ImuA1CB(const sc_msgs::Novatel::ConstPtr& pNovatelMsg) {
//     // 100Hz
//     gpsTime_[0] = gpsTime_[1];
//     gpsTime_[1] = pNovatelMsg->GPS_week_sec;
//     // do nothing if receive same frame
//     if(gpsTime_[0] == gpsTime_[1]) {
//         LOG_EVERY_N(INFO, 10) << "Same frame received, GPStime: " << pNovatelMsg->GPS_week_sec;
//         return;
//     }
//     isGpsUpdated_ = true;

//     monitorMsg_.GPStime = gpsTime_[1];
//     monitorMsg_.hdop = pNovatelMsg->hdop;

//     sc_msgs::Point3D p;
//     // lat: 1 degree is about 100000 m
//     p.x = pNovatelMsg->latitude;
//     // lon: 1 degree is about 100000 m
//     p.y = pNovatelMsg->longitude;
//     p.z = pNovatelMsg->height;
//     monitorMsg_.lat_lon_hei = p;

//     p.x = pNovatelMsg->pitch;
//     p.y = pNovatelMsg->roll;
//     p.z = pNovatelMsg->azimuth;
//     monitorMsg_.pitch_roll_heading = p;

//     // const double vEast = pNovatelMsg->east_vel;
//     // const double vNorth = pNovatelMsg->north_vel;
//     // const double vUp = pNovatelMsg->up_vel;
//     // monitorMsg_.speed = sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp) * 3.6;
//     monitorMsg_.speed = pNovatelMsg->abs_vel * 3.6;

//     monitorMsg_.no_sv = pNovatelMsg->sv_num;

//     // std::stringstream iss(pNovatelMsg->status);
//     // int _status = 0;
//     // iss >> std::hex >> _status;
//     monitorMsg_.status = pNovatelMsg->ins_status;

//     monitorMsg_.unix_time_minus_gps_time = pNovatelMsg->unix_time_minus_gps_time;
// }

void ServerDaemon::SerialCB(const sc_msgs::imu5651::ConstPtr& pImu5651Msg) {
    DLOG(INFO) << __FUNCTION__ << " start: " << pImu5651Msg->gps_time;
    gpsTime_[0] = gpsTime_[1];
    gpsTime_[1] = pImu5651Msg->gps_time;
    if(gpsTime_[0] == gpsTime_[1]) {
        DLOG(INFO) << "Same frame received, GPStime: " << pImu5651Msg->gps_time;
        return;
    }
    isGpsUpdated_ = true;

    monitorMsg_.GPSweek = public_tools::ToolsNoRos::string2int(pImu5651Msg->gps_week);
    monitorMsg_.GPStime = public_tools::ToolsNoRos::string2double(gpsTime_[1]);

    sc_msgs::Point3D p;
    // lat: 1 degree is about 100000 m
    p.x = public_tools::ToolsNoRos::string2double(pImu5651Msg->latitude);
    // lon: 1 degree is about 100000 m
    p.y = public_tools::ToolsNoRos::string2double(pImu5651Msg->longitude);
    p.z = public_tools::ToolsNoRos::string2double(pImu5651Msg->altitude);
    monitorMsg_.lat_lon_hei = p;

    p.x = public_tools::ToolsNoRos::string2double(pImu5651Msg->pitch);
    p.y = public_tools::ToolsNoRos::string2double(pImu5651Msg->roll);
    p.z = public_tools::ToolsNoRos::string2double(pImu5651Msg->heading);
    monitorMsg_.pitch_roll_heading = p;

    const double vEast = public_tools::ToolsNoRos::string2double(pImu5651Msg->vel_east);
    const double vNorth = public_tools::ToolsNoRos::string2double(pImu5651Msg->vel_north);
    const double vUp = public_tools::ToolsNoRos::string2double(pImu5651Msg->vel_up);
    monitorMsg_.speed = pImu5651Msg->vel_east.empty()? -1: (sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp) * 3.6);

    std::stringstream iss(pImu5651Msg->status);
    int _status = 0;
    iss >> std::hex >> _status;
    monitorMsg_.status = _status;

    monitorMsg_.no_sv = public_tools::ToolsNoRos::string2int(pImu5651Msg->no_sv);
    monitorMsg_.hdop = public_tools::ToolsNoRos::string2double(pImu5651Msg->hdop);

    monitorMsg_.unix_time_minus_gps_time = pImu5651Msg->unix_time_minus_gps_time;
}

void ServerDaemon::clientCB(const sc_msgs::ClientCmd::ConstPtr& pClientMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, client command: " << (int)(pClientMsg->system_cmd);

    switch(pClientMsg->system_cmd) {
        case 0: {
            LOG(INFO) << "I am gonna do nothing.";
            break;
        }
        case 1: {
            LOG(INFO) << "I am gonna shutdown.";
            (void)public_tools::PublicTools::PopenWithoutReturn("halt -p");
            break;
        }
        case 2: {
            LOG(INFO) << "I am gonna reboot.";
            (void)public_tools::PublicTools::PopenWithoutReturn("reboot");
            break;
        }
        case 3: {
            LOG(INFO) << "I am gonna cleanup server processes.";
            (void)UpdateProjectInfo("");
            const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
            if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
                LOG(ERROR) << launchScript << " does not exist.";
                exit(1);
            }
            LOG(INFO) << "launchScript: " << launchScript;
            (void)public_tools::PublicTools::PopenWithoutReturn("bash " + launchScript + " cleanup");
            break;
        }
        case 4: {
            LOG(INFO) << "I am gonna fix projects data: " << pClientMsg->cmd_arguments;
            LOG(INFO) << "Is cmd_arguments empty?: " << std::boolalpha << pClientMsg->cmd_arguments.empty();
            if(pClientMsg->cmd_arguments.empty() ) {
                break;
            }
            const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
            if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
                LOG(ERROR) << launchScript << " does not exist.";
                exit(1);
            }
            const std::string fixDataCmd("bash " + launchScript + " fixdata " + pClientMsg->cmd_arguments);
            LOG(INFO) << "fixDataCmd: " << fixDataCmd;
            (void)public_tools::PublicTools::PopenWithoutReturn(fixDataCmd);
            break;
        }
        case 5: {
            LOG(INFO) << "I am gonna remove projects: " << pClientMsg->cmd_arguments;
            LOG(INFO) << "Is cmd_arguments empty?: " << std::boolalpha << pClientMsg->cmd_arguments.empty();
            std::vector<std::string> projectArr;
            if(!pClientMsg->cmd_arguments.empty() ) {
                (void)boost::split(projectArr, pClientMsg->cmd_arguments, boost::is_any_of(",") );
            }

            LOG(INFO) << "I got " << projectArr.size() << " projects to process.";
            for(auto &project: projectArr) {
                const std::string cmd("rm -rf \'/opt/smartc/record/" + project + "\'; true");
                LOG(INFO) << "I am gonna: " + cmd;
                (void)public_tools::PublicTools::PopenWithoutReturn(cmd);
            }
            break;
        }
        case 6: {
            LOG(INFO) << "I am gonna launch a new project: " << pClientMsg->cmd_arguments;
            (void)UpdateProjectInfo(pClientMsg->cmd_arguments);
            projectInfo_ = pClientMsg->cmd_arguments;
            const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
            if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
                LOG(ERROR) << launchScript << " does not exist.";
                exit(1);
            }
            LOG(INFO) << "launchScript: " << launchScript;
            (void)public_tools::PublicTools::PopenWithoutReturn("bash " + launchScript + " server " + pClientMsg->cmd_arguments);
            break;
        }
        case 7: {
            LOG(INFO) << "I am gonna update is_record and cam_gain.";
            LOG(INFO) << "Is cmd_arguments empty?: " << std::boolalpha << pClientMsg->cmd_arguments.empty();
            std::vector<std::string> isRecord_camGain;
            if(!pClientMsg->cmd_arguments.empty() ) {
                (void)boost::split(isRecord_camGain, pClientMsg->cmd_arguments, boost::is_any_of(",") );
            }
            monitorMsg_.is_record = public_tools::PublicTools::string2num(isRecord_camGain[0], 0);
            monitorMsg_.cam_gain = public_tools::PublicTools::string2num(isRecord_camGain[1], 20);
            break;
        }
        case 8: {
            LOG(INFO) << "I am gonna do SLAM: " << pClientMsg->cmd_arguments;
            LOG(INFO) << "Is cmd_arguments empty?: " << std::boolalpha << pClientMsg->cmd_arguments.empty();
            if(pClientMsg->cmd_arguments.empty() ) {
                break;
            }
            const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
            if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
                LOG(ERROR) << launchScript << " does not exist.";
                exit(1);
            }
            const std::string doSlamCmd("bash " + launchScript + " doSlam " + pClientMsg->cmd_arguments);
            LOG(INFO) << "doSlamCmd: " << doSlamCmd;
            (void)public_tools::PublicTools::PopenWithoutReturn(doSlamCmd);
            break;
        }

        default: {
            LOG(ERROR) << "Client command: " << pClientMsg->system_cmd;
        }
    }

    return;
}

// void ServerDaemon::CheckHardware() {
//     LOG(INFO) << __FUNCTION__ << " start.";
//     CheckLidar();
//     CheckCamera();
//     CheckImu();
//     CheckDiskCapacity();
// }

// void ServerDaemon::CheckCamera() {
//     LOG(INFO) << __FUNCTION__ << " start.";
//     if( (0 != public_tools::PublicTools::PopenWithoutReturn("ifconfig | grep '5.5.5.5'")) || (0 != public_tools::PublicTools::PopenWithoutReturn("ifconfig | grep '6.6.6.6'")) || (0 != public_tools::PublicTools::PopenWithoutReturn("ifconfig | grep '7.7.7.7'")) ) {
//         monitorMsg_.camera_HWcheck = -2;
//     }
//     else {
//         monitorMsg_.camera_HWcheck = 3;
//     }
// }

// void ServerDaemon::CheckImu() {
//     LOG(INFO) << __FUNCTION__ << " start.";

//     if(public_tools::PublicTools::isFileExist("/dev/ttyUSB0") && public_tools::PublicTools::isFileExist("/dev/ttyUSB1") && public_tools::PublicTools::isFileExist("/dev/ttyUSB2")) {
//         monitorMsg_.imu_HWcheck = 3;
//     }
//     else {
//         monitorMsg_.imu_HWcheck = -2;
//     }
// }

// void ServerDaemon::CheckDiskCapacity() {
//     LOG(INFO) << __FUNCTION__ << " start.";
//     std::vector<std::string> diskFreeSpaceInGB;
//     const std::string getDiskFreeSpaceInGBCmd("df -BG /opt/smartc/record/ | tail -n1 | awk '{print $4}'");
//     (void)public_tools::PublicTools::PopenWithReturn(getDiskFreeSpaceInGBCmd, diskFreeSpaceInGB);
//     LOG(INFO) << "diskFreeSpaceInGB: " << diskFreeSpaceInGB[0];
//     assert(1 == diskFreeSpaceInGB.size());
//     monitorMsg_.disk_free_space_GB_HWcheck = public_tools::ToolsNoRos::string2int(diskFreeSpaceInGB[0]);
// }

// void ServerDaemon::CheckLidar() {
//     LOG(INFO) << __FUNCTION__ << " start.";
//     const size_t MAXLINE = 1024;
//     const size_t POSITION_PACKET_SIZE = 512;
//     const uint16_t POSITION_PORT_NUMBER = 8308; // default position port

//     sockaddr_in sockaddr;
//     bzero(&sockaddr, sizeof(sockaddr));
//     sockaddr.sin_family = AF_INET;
//     sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
//     sockaddr.sin_port = htons(POSITION_PORT_NUMBER);

//     int listenfd = -1;
//     if((listenfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
//         LOG(ERROR) << "Create socket error: " << strerror(errno);
//         monitorMsg_.pps_HWcheck = -1;
//         monitorMsg_.gprmc_HWcheck = -1;
//         return;
//     }
//     if(bind(listenfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
//         LOG(ERROR) << "Bind socket error: " << strerror(errno);
//         monitorMsg_.pps_HWcheck = -1;
//         monitorMsg_.gprmc_HWcheck = -1;
//         close(listenfd);
//         return;
//     }
//     if (fcntl(listenfd, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
//         LOG(ERROR) << "SockfdPosition non-block.";
//         monitorMsg_.pps_HWcheck = -1;
//         monitorMsg_.gprmc_HWcheck = -1;
//         close(listenfd);
//         return;
//     }

//     LOG(INFO) << "Please wait for the client information...";
//     struct pollfd fds;
//     fds.fd = listenfd;
//     fds.events = POLLIN;
//     const int POLL_TIMEOUT = 1000; // one second (in msec)
//     do {
//         int retval = poll(&fds, 1, POLL_TIMEOUT);
//         if(retval < 0) {
//             if(errno != EINTR) {
//                 LOG(ERROR) << "poll() error: " << strerror(errno);
//                 monitorMsg_.pps_HWcheck = -1;
//                 monitorMsg_.gprmc_HWcheck = -1;
//                 close(listenfd);
//                 return;
//             }
//         }
//         if(retval == 0) {
//             LOG(ERROR) << "Velodyne poll() timeout, go check IP configuration.";
//             monitorMsg_.pps_HWcheck = -2;
//             monitorMsg_.gprmc_HWcheck = -2;
//             close(listenfd);
//             return;
//         }
//         if((fds.revents & POLLERR) || (fds.revents & POLLHUP) || (fds.revents & POLLNVAL)) {
//             LOG(ERROR) << "poll() reports Velodyne error.";
//             monitorMsg_.pps_HWcheck = -1;
//             monitorMsg_.gprmc_HWcheck = -1;
//             close(listenfd);
//             return;
//         }
//     }
//     while((fds.revents & POLLIN) == 0);

//     char buff[MAXLINE];
//     const size_t nbytes = recvfrom(listenfd, buff, POSITION_PACKET_SIZE, 0, NULL, NULL);


//     LOG(INFO) << nbytes;
// #ifndef NDEBUG
//     for(size_t i = 0; i < nbytes; ++i) {
//         LOG(INFO) << i << ": " << std::hex << (int)buff[i];
//     }
// #endif
//     LOG(INFO) << buff[206] << " should be $";

//     ParsePositionPkt(buff);

//     close(listenfd);
//     return;
// }

// void ServerDaemon::ParsePositionPkt(const char *pkt) {
//     LOG(INFO) << __FUNCTION__ << " start.";
//     if('$' != pkt[206]) {
//         LOG(ERROR) << "No position packet received: " << std::hex << (int)pkt[206];
//         monitorMsg_.pps_HWcheck = -1;
//         // A validity - A-ok, V-invalid, refer VLP-16 manual
//         monitorMsg_.gprmc_HWcheck = -1;
//         return;
//     }

//     // 00 .. 03
//     // "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
//     monitorMsg_.pps_HWcheck = pkt[202];
//     // $GPRMC,,V,,,,,,,,,,N*53
//     size_t dotCnt = 0;
//     for(size_t i = 210; i < 230; ++i) {
//         if(',' == pkt[i]) {
//             ++dotCnt;
//         }
//         if(2 == dotCnt) {
//             monitorMsg_.gprmc_HWcheck = pkt[i + 1];
//             return;
//         }
//     }

//     monitorMsg_.gprmc_HWcheck = -1;
//     return;
// }

void ServerDaemon::projectMonitorCB(const sc_msgs::DiskInfo::ConstPtr& pDiskInfoMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, disk image number: " << pDiskInfoMsg->img_num;
    isDiskInfoUpdated_ = true;
    monitorMsg_.img_num = pDiskInfoMsg->img_num;
    monitorMsg_.lidar_size = pDiskInfoMsg->lidar_size;
    monitorMsg_.img_save_fps = pDiskInfoMsg->img_save_fps;
    monitorMsg_.raw_ins_size = pDiskInfoMsg->raw_ins_size;
    monitorMsg_.timestamp_size = pDiskInfoMsg->timestamp_size;
}

void ServerDaemon::UpdateProjectInfo(const std::string &projectInfo) {
    LOG(INFO) << __FUNCTION__ << " start, projectInfo: " << projectInfo;
    if(projectInfo.empty() ) {
        LOG(INFO) << "Clear projectInfo message.";
        monitorMsg_.project_info.city_code = monitorMsg_.project_info.daynight_code = 0;
        monitorMsg_.project_info.task_id.clear();
        monitorMsg_.project_info.device_id.clear();
        monitorMsg_.project_info.date.clear();

        monitorMsg_.is_record = 0;
        monitorMsg_.cam_gain = 20;
        return;
    }

    std::vector<std::string> parsedProjectInfo;
    (void)boost::split(parsedProjectInfo, projectInfo, boost::is_any_of("-") );
    if(4 != parsedProjectInfo.size() ) {
        LOG(ERROR) << "Failed to parse " << projectInfo << ", parsedProjectInfo.size(): " << parsedProjectInfo.size();
        exit(1);
    }

    monitorMsg_.project_info.city_code = public_tools::PublicTools::string2num(parsedProjectInfo[0], 0);
    monitorMsg_.project_info.daynight_code = public_tools::PublicTools::string2num(parsedProjectInfo[1], 0);
    monitorMsg_.project_info.task_id = parsedProjectInfo[2].substr(0, parsedProjectInfo[2].size() - 4);
    monitorMsg_.project_info.device_id = parsedProjectInfo[2].substr(parsedProjectInfo[2].size() - 4);
    monitorMsg_.project_info.date = parsedProjectInfo[3];
    return;
}

