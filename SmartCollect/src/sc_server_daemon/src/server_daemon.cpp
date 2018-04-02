#include "server_daemon.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

ServerDaemon::ServerDaemon(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    // below for SC control
    subClient_ = nh.subscribe("sc_client_cmd", 10, &ServerDaemon::clientCB, this);
    pub2scNodes_ = nh.advertise<sc_server_daemon::nodeParams>("sc_node_params", 10);
    nodeParams_.is_record = 0;
    nodeParams_.cam_gain = 20;
    pub2broswer_ = nh.advertise<sc_server_daemon::projectInfoMsg>("sc_project_info", 10);

    // below for composing monitor data
    isGpsUpdated_ = isVelodyneUpdated_ = isRawImuUpdated_ = isCameraUpdated_ = false;
    sub232_ = nh.subscribe("imu_string", 0, &ServerDaemon::gpsCB, this);
    subVelodyne_ = nh.subscribe("velodyne_pps_status", 0, &ServerDaemon::velodyneCB, this);
    sub422_ = nh.subscribe("imu422_hdop", 0, &ServerDaemon::rawImuCB, this);
    subCameraImg_ = nh.subscribe("cam_speed", 0, &ServerDaemon::cameraImgCB, this);
    pub2client_ = nh.advertise<sc_server_daemon::monitorMsg>("sc_monitor", 0);
    mGpsTime[0] = mGpsTime[1] = -1;
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

        DLOG(INFO) << "Publish a server command: is_record: " << (int)(nodeParams_.is_record) << ", cam_gain: " << nodeParams_.cam_gain;
        pub2scNodes_.publish(nodeParams_);
        pub2broswer_.publish(projectInfoMsg_);

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

        pub2client_.publish(monitorMsg_);
    }
}

void ServerDaemon::rawImuCB(const sc_integrate_imu_recorder::scIntegrateImu::ConstPtr& pRawImuMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in 1Hz.";
    isRawImuUpdated_ = true;

    // format: $GPGGA,,,,,,0,,,,,,,,*66
    monitorMsg_.latitude = pRawImuMsg->Latitude;
    monitorMsg_.longitude = pRawImuMsg->Longitude;
    monitorMsg_.hdop = pRawImuMsg->Hdop;
    monitorMsg_.noSV_422 = pRawImuMsg->NoSV;
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

void ServerDaemon::gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg) {
    // 100Hz
    mGpsTime[0] = mGpsTime[1];
    mGpsTime[1] = public_tools::PublicTools::string2num(pGPSmsg->GPSTime, -1.0);
    // do nothing if receive same frame
    if(mGpsTime[0] == mGpsTime[1]) {
        LOG(INFO) << "Same frame received, GPStime: " << pGPSmsg->GPSTime;
        return;
    }
    isGpsUpdated_ = true;

    monitorMsg_.GPStime = mGpsTime[1];
    sc_server_daemon::point3D p;
    // lat: 1 degree is about 100000 m
    p.x = public_tools::PublicTools::string2num(pGPSmsg->Latitude, -1.0);
    // lon: 1 degree is about 100000 m
    p.y = public_tools::PublicTools::string2num(pGPSmsg->Longitude, -1.0);
    p.z = public_tools::PublicTools::string2num(pGPSmsg->Altitude, -1.0);
    monitorMsg_.lat_lon_hei = p;

    p.x = public_tools::PublicTools::string2num(pGPSmsg->Pitch, -1.0);
    p.y = public_tools::PublicTools::string2num(pGPSmsg->Roll, -1.0);
    p.z = public_tools::PublicTools::string2num(pGPSmsg->Heading, -1.0);
    monitorMsg_.pitch_roll_heading = p;

    if(pGPSmsg->Vel_east.empty() || pGPSmsg->Vel_north.empty() || pGPSmsg->Vel_up.empty() ) {
        LOG(WARNING) << "pGPSmsg->V is empty: " << pGPSmsg->Vel_east;
        monitorMsg_.speed = -1;
    }
    else {
        double vEast = public_tools::PublicTools::string2num(pGPSmsg->Vel_east, 0.0);
        double vNorth = public_tools::PublicTools::string2num(pGPSmsg->Vel_north, 0.0);
        double vUp = public_tools::PublicTools::string2num(pGPSmsg->Vel_up, 0.0);
        monitorMsg_.speed = sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp) * 3.6;
    }

    monitorMsg_.nsv1_num = public_tools::PublicTools::string2num(pGPSmsg->NSV1_num, -1);
    monitorMsg_.nsv2_num = public_tools::PublicTools::string2num(pGPSmsg->NSV2_num, -1);
}

void ServerDaemon::clientCB(const sc_server_daemon::clientCmd::ConstPtr& pClientMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, client command: " << pClientMsg->project_name;

    // shutdown
    if(1 == pClientMsg->system_cmd) {
        (void)public_tools::PublicTools::runShellCmd("halt -p");
        return;
    }
    // reboot
    if(2 == pClientMsg->system_cmd) {
        (void)public_tools::PublicTools::runShellCmd("reboot");
        return;
    }
    // cleanup server processes
    if(3 == pClientMsg->system_cmd) {
        (void)updateprojectInfoMsg("");
        // add a "true" to avoid exit
        (void)public_tools::PublicTools::runShellCmd("pkill sc_integrate_; pkill roscameragps; killall nodelet; true");
        return;
    }

    // system_cmd is 0
    if(pClientMsg->project_name.empty() ) {
        LOG(INFO) << "Empty project name: update is_record and cam_gain.";
        nodeParams_.is_record = pClientMsg->node_params.is_record;
        nodeParams_.cam_gain = pClientMsg->node_params.cam_gain;
        return;
    }
    // system_cmd is 0 && new project name received
    LOG(INFO) << "Project name: " << pClientMsg->project_name;
    (void)updateprojectInfoMsg(pClientMsg->project_name);

    const std::string launchScript("/opt/smartc/src/tools/launch_project.sh");
    if(!(public_tools::PublicTools::isFileExist(launchScript) ) ) {
        LOG(ERROR) << launchScript << " does not exist.";
        exit(1);
    }
    LOG(INFO) << "launchScript: " << launchScript;

    (void)public_tools::PublicTools::runShellCmd("bash " + launchScript + " server " + pClientMsg->project_name);
    return;
}

void ServerDaemon::updateprojectInfoMsg(const std::string &projectInfo) {
    LOG(INFO) << __FUNCTION__ << " start, projectInfo: " << projectInfo;
    if(projectInfo.empty() ) {
        LOG(INFO) << "Clear projectInfo message.";
        projectInfoMsg_.city_code = projectInfoMsg_.daynight_code = 0;
        projectInfoMsg_.task_id.clear();
        projectInfoMsg_.device_id.clear();

        nodeParams_.is_record = 0;
        nodeParams_.cam_gain = 20;
        return;
    }

    std::vector<std::string> parsedProjectInfo;
    (void)boost::split(parsedProjectInfo, projectInfo, boost::is_any_of("-") );
    if(4 != parsedProjectInfo.size() ) {
        LOG(ERROR) << "Failed to parse " << projectInfo << ", parsedProjectInfo.size(): " << parsedProjectInfo.size();
        exit(1);
    }

    projectInfoMsg_.city_code = public_tools::PublicTools::string2num(parsedProjectInfo[0], 0);
    projectInfoMsg_.daynight_code = public_tools::PublicTools::string2num(parsedProjectInfo[1], 0);
    projectInfoMsg_.task_id = parsedProjectInfo[2].substr(0, parsedProjectInfo[2].size() - 4);
    projectInfoMsg_.device_id = parsedProjectInfo[2].substr(parsedProjectInfo[2].size() - 4);
    return;
}

