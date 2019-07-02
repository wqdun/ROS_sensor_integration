#include <glog/logging.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "base_server.h"
#include "../../sc_lib_public_tools/src/public_tools.h"

BaseServer::~BaseServer() {
    LOG(INFO) << "Goodbye BaseServer..";
}

void BaseServer::UpdateProjectInfo(const std::string &projectInfo) {
    LOG(INFO) << __FUNCTION__ << " start, projectInfo: " << projectInfo;
    if(projectInfo.empty() ) {
        LOG(INFO) << "Clear projectInfo message.";
        _monitorMsg_.project_info.city_code = _monitorMsg_.project_info.daynight_code = 0;
        _monitorMsg_.project_info.task_id.clear();
        _monitorMsg_.project_info.device_id.clear();
        _monitorMsg_.project_info.date.clear();

        _monitorMsg_.is_record = 0;
        _monitorMsg_.cam_gain = 20;
        return;
    }

    std::vector<std::string> parsedProjectInfo;
    (void)boost::split(parsedProjectInfo, projectInfo, boost::is_any_of("-") );
    if(4 != parsedProjectInfo.size() ) {
        LOG(ERROR) << "Failed to parse " << projectInfo << ", parsedProjectInfo.size(): " << parsedProjectInfo.size();
        exit(1);
    }

    _monitorMsg_.project_info.city_code = public_tools::PublicTools::string2num(parsedProjectInfo[0], 0);
    _monitorMsg_.project_info.daynight_code = public_tools::PublicTools::string2num(parsedProjectInfo[1], 0);
    _monitorMsg_.project_info.task_id = parsedProjectInfo[2].substr(0, parsedProjectInfo[2].size() - 4);
    _monitorMsg_.project_info.device_id = parsedProjectInfo[2].substr(parsedProjectInfo[2].size() - 4);
    _monitorMsg_.project_info.date = parsedProjectInfo[3];
    return;
}


void BaseServer::dataFixerCB(const sc_msgs::DataFixerProgress::ConstPtr& pDataFixerProgressMsg) {
    DLOG(INFO) << __FUNCTION__ << " start: " << pDataFixerProgressMsg->processNum << "/" << pDataFixerProgressMsg->totalFileNum;

    _monitorMsg_.total_file_num = pDataFixerProgressMsg->totalFileNum;
    _monitorMsg_.process_num = pDataFixerProgressMsg->processNum;
}

void BaseServer::Camera0FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCamera0FpsUpdated_ = true;
    camera0Fps_ = pCameraImgMsg->data;
}

void BaseServer::Camera1FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCamera1FpsUpdated_ = true;
    camera1Fps_ = pCameraImgMsg->data;
}

void BaseServer::Camera2FpsCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    DLOG(INFO) << __FUNCTION__ << " start in ?Hz, camera_fps: " << pCameraImgMsg->data;
    isCamera2FpsUpdated_ = true;
    camera2Fps_ = pCameraImgMsg->data;
}

void BaseServer::velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg) {
    // 10Hz
    DLOG(INFO) << __FUNCTION__ << " start.";
    isVelodyneUpdated_ = true;

    if( (pVelodyneMsg->pps_status_index) > 3) {
        LOG(ERROR) << "Invalid PPS status: " << pVelodyneMsg->pps_status_index;
        exit(1);
    }

    _monitorMsg_.pps_status = PPS_STATUS[pVelodyneMsg->pps_status_index];
    // A validity - A-ok, V-invalid, refer VLP-16 manual
    _monitorMsg_.is_gprmc_valid = pVelodyneMsg->is_gprmc_valid;
    _monitorMsg_.velodyne_rpm = pVelodyneMsg->velodyne_rpm;
}

void BaseServer::SerialCB(const sc_msgs::imu5651::ConstPtr& pImu5651Msg) {
    DLOG(INFO) << __FUNCTION__ << " start: " << pImu5651Msg->gps_time;
    gpsTime_[0] = gpsTime_[1];
    gpsTime_[1] = pImu5651Msg->gps_time;
    if(gpsTime_[0] == gpsTime_[1]) {
        DLOG(INFO) << "Same frame received, GPStime: " << pImu5651Msg->gps_time;
        return;
    }
    isGpsUpdated_ = true;

    _monitorMsg_.GPSweek = public_tools::ToolsNoRos::string2int(pImu5651Msg->gps_week);
    _monitorMsg_.GPStime = public_tools::ToolsNoRos::string2double(gpsTime_[1]);

    sc_msgs::Point3D p;
    // lat: 1 degree is about 100000 m
    p.x = public_tools::ToolsNoRos::string2double(pImu5651Msg->latitude);
    // lon: 1 degree is about 100000 m
    p.y = public_tools::ToolsNoRos::string2double(pImu5651Msg->longitude);
    p.z = public_tools::ToolsNoRos::string2double(pImu5651Msg->altitude);
    _monitorMsg_.lat_lon_hei = p;

    p.x = public_tools::ToolsNoRos::string2double(pImu5651Msg->pitch);
    p.y = public_tools::ToolsNoRos::string2double(pImu5651Msg->roll);
    p.z = public_tools::ToolsNoRos::string2double(pImu5651Msg->heading);
    _monitorMsg_.pitch_roll_heading = p;

    const double vEast = public_tools::ToolsNoRos::string2double(pImu5651Msg->vel_east);
    const double vNorth = public_tools::ToolsNoRos::string2double(pImu5651Msg->vel_north);
    const double vUp = public_tools::ToolsNoRos::string2double(pImu5651Msg->vel_up);
    _monitorMsg_.speed = pImu5651Msg->vel_east.empty()? -1: (sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp) * 3.6);

    std::stringstream iss(pImu5651Msg->status);
    int _status = 0;
    iss >> std::hex >> _status;
    _monitorMsg_.status = _status;

    _monitorMsg_.no_sv = public_tools::ToolsNoRos::string2int(pImu5651Msg->no_sv);
    _monitorMsg_.hdop = public_tools::ToolsNoRos::string2double(pImu5651Msg->hdop);

    _monitorMsg_.unix_time_minus_gps_time = pImu5651Msg->unix_time_minus_gps_time;
}

void BaseServer::clientCB(const sc_msgs::ClientCmd::ConstPtr& pClientMsg) {
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
            _monitorMsg_.is_record = public_tools::PublicTools::string2num(isRecord_camGain[0], 0);
            _monitorMsg_.cam_gain = public_tools::PublicTools::string2num(isRecord_camGain[1], 20);
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

void BaseServer::projectMonitorCB(const sc_msgs::DiskInfo::ConstPtr& pDiskInfoMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, disk image number: " << pDiskInfoMsg->img_num;
    isDiskInfoUpdated_ = true;
    _monitorMsg_.img_num = pDiskInfoMsg->img_num;
    _monitorMsg_.lidar_size = pDiskInfoMsg->lidar_size;
    _monitorMsg_.img_save_fps = pDiskInfoMsg->img_save_fps;
    _monitorMsg_.raw_ins_size = pDiskInfoMsg->raw_ins_size;
    _monitorMsg_.timestamp_size = pDiskInfoMsg->timestamp_size;
}



