#include "server_daemon.h"

ServerDaemon::ServerDaemon(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    // below for SC control
    subClient_ = nh.subscribe("sc_client_cmd", 10, &ServerDaemon::clientCB, this);
    monitorMsg_.is_record = 0;
    monitorMsg_.cam_gain = 20;
    monitorMsg_.pps_HWcheck = -1;
    monitorMsg_.gprmc_HWcheck = -1;

    // below for composing monitor data
    isGpsUpdated_ = isVelodyneUpdated_ = isRawImuUpdated_ = isCameraUpdated_ = isDiskInfoUpdated_ = false;
    subSerial_ = nh.subscribe("sc_novatel", 10, &ServerDaemon::SerialCB, this);
    subVelodyne_ = nh.subscribe("velodyne_pps_status", 0, &ServerDaemon::velodyneCB, this);
    subCameraImg_ = nh.subscribe("cam_speed5555", 0, &ServerDaemon::cameraImgCB, this);
    subProjectMonitor_ = nh.subscribe("sc_disk_info", 0, &ServerDaemon::projectMonitorCB, this);
    subDataFixer_ = nh.subscribe("sc_data_fixer_progress", 0, &ServerDaemon::dataFixerCB, this);

    pub2client_ = nh.advertise<sc_msgs::MonitorMsg>("sc_monitor", 0);
    gpsTime_[0] = gpsTime_[1] = -1;

    pDiskMonitor_.reset(new DiskMonitor() );
}

ServerDaemon::~ServerDaemon() {}

void ServerDaemon::run() {
    ros::Rate rate(2);
    size_t freqDivider = 0;

    while(ros::ok() ) {
        ++freqDivider;
        freqDivider %= 256;
        ros::spinOnce();
        rate.sleep();

        // 1Hz
        if(0 == (freqDivider % 2) ) {
            if(!isGpsUpdated_) {
                DLOG(INFO) << "RS232 node not running.";
                monitorMsg_.GPStime = monitorMsg_.lat_lon_hei.x = monitorMsg_.lat_lon_hei.y = monitorMsg_.lat_lon_hei.z = monitorMsg_.pitch_roll_heading.x = monitorMsg_.pitch_roll_heading.y = monitorMsg_.pitch_roll_heading.z = monitorMsg_.speed = -2.;
                monitorMsg_.nsv1_num = monitorMsg_.nsv2_num = -2;
            }
            isGpsUpdated_ = false;
        }

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            if(!isRawImuUpdated_) {
                DLOG(INFO) << "sc_integrate_imu_recorder node not running.";
                monitorMsg_.hdop = monitorMsg_.latitude = monitorMsg_.longitude = monitorMsg_.noSV_422 = "sc_integrate_imu_recorder node not running";
            }
            isRawImuUpdated_ = false;
        }

        // 1Hz
        if(0 == (freqDivider % 2) ) {
            if(!isVelodyneUpdated_) {
                DLOG(INFO) << "velodyne node not running.";
                monitorMsg_.pps_status = monitorMsg_.is_gprmc_valid = "velodyne node not running";
            }
            isVelodyneUpdated_ = false;
        }

        // 0.5Hz
        if(0 == (freqDivider % 4) ) {
            if(!isCameraUpdated_) {
                DLOG(INFO) << "camera node not running.";
                monitorMsg_.camera_fps = -2.;
            }
            isCameraUpdated_ = false;
        }

        // 0.25Hz
        if(0 == (freqDivider % 8) ) {
            if(!isDiskInfoUpdated_) {
                DLOG(INFO) << "project monitor node not running.";
                monitorMsg_.img_num = -2;
            }
            isDiskInfoUpdated_ = false;
        }

        monitorMsg_.projects.clear();
        monitorMsg_.disk_usage.clear();
        (void)pDiskMonitor_->run("/opt/smartc/record/", monitorMsg_);

        if(public_tools::PublicTools::isFileExist("/tmp/data_fixer_progress_100%") ) {
            monitorMsg_.process_num = monitorMsg_.total_file_num;
        }

        pub2client_.publish(monitorMsg_);
    }
}

void ServerDaemon::dataFixerCB(const sc_msgs::DataFixerProgress::ConstPtr& pDataFixerProgressMsg) {
    LOG(INFO) << __FUNCTION__ << " start: " << pDataFixerProgressMsg->processNum << "/" << pDataFixerProgressMsg->totalFileNum;

    monitorMsg_.total_file_num = pDataFixerProgressMsg->totalFileNum;
    monitorMsg_.process_num = pDataFixerProgressMsg->processNum;
}

void ServerDaemon::cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCameraUpdated_ = true;

    monitorMsg_.camera_fps = pCameraImgMsg->data;
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
}

void ServerDaemon::SerialCB(const sc_msgs::Novatel::ConstPtr& pNovatelMsg) {
    // 100Hz
    gpsTime_[0] = gpsTime_[1];
    gpsTime_[1] = pNovatelMsg->seconds_into_week;
    // do nothing if receive same frame
    if(gpsTime_[0] == gpsTime_[1]) {
        LOG_EVERY_N(INFO, 10) << "Same frame received, GPStime: " << pNovatelMsg->seconds_into_week;
        return;
    }
    isGpsUpdated_ = true;

    monitorMsg_.GPStime = gpsTime_[1];
    monitorMsg_.hdop_novatel = pNovatelMsg->hdop;

    sc_msgs::Point3D p;
    // lat: 1 degree is about 100000 m
    p.x = pNovatelMsg->latitude;
    // lon: 1 degree is about 100000 m
    p.y = pNovatelMsg->longitude;
    p.z = pNovatelMsg->height;
    monitorMsg_.lat_lon_hei = p;

    p.x = pNovatelMsg->pitch;
    p.y = pNovatelMsg->roll;
    p.z = pNovatelMsg->azimuth;
    monitorMsg_.pitch_roll_heading = p;

    const double vEast = pNovatelMsg->east_vel;
    const double vNorth = pNovatelMsg->north_vel;
    const double vUp = pNovatelMsg->up_vel;
    monitorMsg_.speed = sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp) * 3.6;

    monitorMsg_.nsv2_num = monitorMsg_.nsv1_num = pNovatelMsg->sv_num;
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
            (void)public_tools::PublicTools::runShellCmd("halt -p");
            break;
        }
        case 2: {
            LOG(INFO) << "I am gonna reboot.";
            (void)public_tools::PublicTools::runShellCmd("reboot");
            break;
        }
        case 3: {
            LOG(INFO) << "I am gonna cleanup server processes.";
            (void)updateProjectInfo("");
            const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
            if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
                LOG(ERROR) << launchScript << " does not exist.";
                exit(1);
            }
            LOG(INFO) << "launchScript: " << launchScript;
            (void)public_tools::PublicTools::runShellCmd("bash " + launchScript + " cleanup");
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
            (void)public_tools::PublicTools::runShellCmd(fixDataCmd);
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
                // const std::string cmd("mv \'/opt/smartc/record/" + project + "\' /tmp/; true");
                const std::string cmd("rm -rf \'/opt/smartc/record/" + project + "\'; true");
                LOG(INFO) << "I am gonna: " + cmd;
                (void)public_tools::PublicTools::runShellCmd(cmd);
            }
            break;
        }
        case 6: {
            LOG(INFO) << "I am gonna launch a new project: " << pClientMsg->cmd_arguments;
            (void)updateProjectInfo(pClientMsg->cmd_arguments);
            const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
            if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
                LOG(ERROR) << launchScript << " does not exist.";
                exit(1);
            }
            LOG(INFO) << "launchScript: " << launchScript;
            (void)public_tools::PublicTools::runShellCmd("bash " + launchScript + " server " + pClientMsg->cmd_arguments);
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
            LOG(INFO) << "I am gonna check hardware.";
            (void)CheckHardware();
            break;
        }

        default: {
            LOG(ERROR) << "Client command: " << pClientMsg->system_cmd;
        }
    }

    return;
}

void ServerDaemon::CheckHardware() {
    LOG(INFO) << __FUNCTION__ << " start.";
    CheckLidar();
    CheckCamera();
    CheckDiskCapacity();
}


void ServerDaemon::CheckCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";


}

void ServerDaemon::CheckDiskCapacity() {
    LOG(INFO) << __FUNCTION__ << " start.";
    std::vector<std::string> diskFreeSpaceInGB;
    const std::string getDiskFreeSpaceInGBCmd("df -BG /opt/smartc/record/ | tail -n1 | awk '{print $2\", \"$5}'");
    (void)public_tools::PublicTools::popenWithReturn(getDiskFreeSpaceInGBCmd, diskFreeSpaceInGB);
    LOG(INFO) << diskFreeSpaceInGB;
    // monitorMsg_.disk_usage = (1 == diskUsage.size())? diskUsage[0]: "error: I got !1 lines";

}




void ServerDaemon::CheckLidar() {
    LOG(INFO) << __FUNCTION__ << " start.";
    const size_t MAXLINE = 1024;
    const size_t POSITION_PACKET_SIZE = 512;
    const uint16_t POSITION_PORT_NUMBER = 8308; // default position port

    sockaddr_in sockaddr;
    bzero(&sockaddr, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr.sin_port = htons(POSITION_PORT_NUMBER);

    int listenfd = -1;
    if((listenfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        LOG(ERROR) << "Create socket error: " << strerror(errno);
        monitorMsg_.pps_HWcheck = -1;
        monitorMsg_.gprmc_HWcheck = -1;
        return;
    }
    if(bind(listenfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        LOG(ERROR) << "Bind socket error: " << strerror(errno);
        monitorMsg_.pps_HWcheck = -1;
        monitorMsg_.gprmc_HWcheck = -1;
        close(listenfd);
        return;
    }
    if (fcntl(listenfd, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        LOG(ERROR) << "SockfdPosition non-block.";
        monitorMsg_.pps_HWcheck = -1;
        monitorMsg_.gprmc_HWcheck = -1;
        close(listenfd);
        return;
    }

    LOG(INFO) << "Please wait for the client information...";
    struct pollfd fds;
    fds.fd = listenfd;
    fds.events = POLLIN;
    const int POLL_TIMEOUT = 1000; // one second (in msec)
    do {
        int retval = poll(&fds, 1, POLL_TIMEOUT);
        if(retval < 0) {
            if(errno != EINTR) {
                LOG(ERROR) << "poll() error: " << strerror(errno);
                monitorMsg_.pps_HWcheck = -1;
                monitorMsg_.gprmc_HWcheck = -1;
                close(listenfd);
                return;
            }
        }
        if(retval == 0) {
            LOG(ERROR) << "Velodyne poll() timeout, go check IP configuration.";
            monitorMsg_.pps_HWcheck = -2;
            monitorMsg_.gprmc_HWcheck = -2;
            close(listenfd);
            return;
        }
        if((fds.revents & POLLERR) || (fds.revents & POLLHUP) || (fds.revents & POLLNVAL)) {
            LOG(ERROR) << "poll() reports Velodyne error.";
            monitorMsg_.pps_HWcheck = -1;
            monitorMsg_.gprmc_HWcheck = -1;
            close(listenfd);
            return;
        }
    }
    while((fds.revents & POLLIN) == 0);

    char buff[MAXLINE];
    const size_t nbytes = recvfrom(listenfd, buff, POSITION_PACKET_SIZE, 0, NULL, NULL);


    LOG(INFO) << nbytes;
#ifndef NDEBUG
    for(size_t i = 0; i < nbytes; ++i) {
        LOG(INFO) << i << ": " << std::hex << (int)buff[i];
    }
#endif
    LOG(INFO) << buff[206] << " should be $";

    ParsePositionPkt(buff);

    close(listenfd);
    return;
}

void ServerDaemon::ParsePositionPkt(const char *pkt) {
    LOG(INFO) << __FUNCTION__ << " start.";
    if('$' != pkt[206]) {
        LOG(ERROR) << "No position packet received: " << std::hex << (int)pkt[206];
        monitorMsg_.pps_HWcheck = -1;
        // A validity - A-ok, V-invalid, refer VLP-16 manual
        monitorMsg_.gprmc_HWcheck = -1;
        return;
    }

    // 00 .. 03
    // "No PPS", "Synchronizing PPS", "PPS locked", "PPS Error"
    monitorMsg_.pps_HWcheck = pkt[202];
    // $GPRMC,,V,,,,,,,,,,N*53
    size_t dotCnt = 0;
    for(size_t i = 210; i < 230; ++i) {
        if(',' == pkt[i]) {
            ++dotCnt;
        }
        if(2 == dotCnt) {
            monitorMsg_.gprmc_HWcheck = pkt[i + 1];
            return;
        }
    }

    monitorMsg_.gprmc_HWcheck = -1;
    return;
}

void ServerDaemon::projectMonitorCB(const sc_msgs::DiskInfo::ConstPtr& pDiskInfoMsg) {
    LOG(INFO) << __FUNCTION__ << " start, disk image number: " << pDiskInfoMsg->img_num;
    isDiskInfoUpdated_ = true;
    monitorMsg_.img_num = pDiskInfoMsg->img_num;
    monitorMsg_.lidar_size = pDiskInfoMsg->lidar_size;
}

void ServerDaemon::updateProjectInfo(const std::string &projectInfo) {
    LOG(INFO) << __FUNCTION__ << " start, projectInfo: " << projectInfo;
    if(projectInfo.empty() ) {
        LOG(INFO) << "Clear projectInfo message.";
        monitorMsg_.project_info.city_code = monitorMsg_.project_info.daynight_code = 0;
        monitorMsg_.project_info.task_id.clear();
        monitorMsg_.project_info.device_id.clear();

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
    return;
}

