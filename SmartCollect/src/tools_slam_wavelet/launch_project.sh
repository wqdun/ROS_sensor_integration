#!/bin/bash
clear
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
absolute_catkin_path=$(cd ${absolute_script_path}/../.. && pwd)

mkdir -p /opt/smartc/log/
result_log="/opt/smartc/log/${script_name}.log"

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
}

get_sudo_permission() {
    echo 123 | sudo -S su >/dev/null 2>&1
}

source_ROS_Env() {
    log_with_time "$FUNCNAME start."
    local ROS_setup=${absolute_catkin_path}"/devel/setup.bash"
    if [ ! -f "${ROS_setup}" ]; then
        log_with_time "[ERROR] ${ROS_setup} does not exist."
        exit 1
    fi
    source ${ROS_setup} >/dev/null 2>&1
}

create_record_path() {
    log_with_time "$FUNCNAME start, param: $*"
    local _absolute_record_path=$1
    mkdir -p "${_absolute_record_path}/Image/"
    mkdir -p "${_absolute_record_path}/IMU/"
    mkdir -p "${_absolute_record_path}/Lidar/"
}

add_path_to_veledyne_launch() {
    log_with_time "$FUNCNAME start, param: $*"
    local _absolute_record_path=$1
    local absolute_velodyne_launch="${absolute_catkin_path}/src/velodyne/velodyne_driver/launch/nodelet_manager.launch"
    local parse_xml_script="${absolute_script_path}/parse_xml.py"
    chmod +x "${parse_xml_script}"
    "${parse_xml_script}" "${absolute_velodyne_launch}" "\$(arg manager)_driver" "record_path" "${_absolute_record_path}/Lidar/" "${absolute_velodyne_launch%.*}.xml"

    log_with_time "Add record path ${_absolute_record_path} to launch file: ${absolute_velodyne_launch}."
}

redirect_glog_path() {
    log_with_time "$FUNCNAME start, param: $*"
    local _absolute_record_path=$1
    mkdir -p "${_absolute_record_path}/Log/"
    export GLOG_log_dir="${_absolute_record_path}/Log/"
}

start_smart_collector_server() {
    log_with_time "$FUNCNAME start, param: $*"
    log_with_time "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"
    local _absolute_record_path=$1

    get_sudo_permission
    sudo chmod +r /dev/ttyUSB0
    pkill sc_integrate_
    rosrun sc_integrate_imu_recorder sc_integrate_imu_recorder_node "${_absolute_record_path}/IMU/" &
    sleep 0.2

    get_sudo_permission
    sudo chmod +r /dev/ttyS0

    /sbin/ifconfig eth0 | /bin/grep -w inet >>$result_log 2>&1
    if [ $? -ne 0 ]; then
        log_with_time "sc_camera_ip_forcer_node has not been executed, gonna run it."
        bash /opt/smartc/src/tools/force_ip_per_minute.sh
    else
        log_with_time "sc_camera_ip_forcer_node already been executed."
    fi

    for i in $(seq 30)
    do
        pidof sc_camera_ip_forcer_node
        if [ $? -eq 0 ]; then
            log_with_time "sc_camera_ip_forcer_node is running, waiting..."
            sleep 1
            continue
        fi
        pkill sc_camera_ip_
        pkill sc_camera
        /opt/smartc/devel/lib/sc_camera/sc_camera jpg "${_absolute_record_path}/" &
        break
    done

    killall nodelet
    roslaunch velodyne_pointcloud VLP16_points.launch &
    sleep 0.2

    pkill sc_project_mon
    /opt/smartc/devel/lib/sc_project_monitor/sc_project_monitor_node "${_absolute_record_path}/" &
    sleep 0.2

    pkill sc_map_node
    /opt/smartc/devel/lib/sc_map/sc_map_node "${_absolute_record_path}/" &
    sleep 0.2
}

do_kill() {
    log_with_time "$FUNCNAME start."

    pkill sc_integrate_
    pkill sc_camera
    killall nodelet
    pkill sc_project_mon
    pkill sc_map_node
    return 0
}

do_fixdata() {
    log_with_time "$FUNCNAME start, param: $*"

    local _projects=$1
    /opt/smartc/devel/lib/sc_data_fixer/sc_data_fixer_node "${_projects}" &
    return 0
}

do_slam() {
    log_with_time "$FUNCNAME start, param: $*"
    local _projects="/opt/smartc/record/"$1

    (
        killall multiScanRegistration
        cd /opt/LidarSlam/velodyneSlam/multiScanRegistrationPackage \
            && chmod +x multiScanRegistration \
            && bash multiScanRegistration.sh &
    )

    (
        killall laserOdometry
        cd /opt/LidarSlam/velodyneSlam/laserOdometryPackage \
            && chmod +x laserOdometry \
            && bash laserOdometry.sh "${_projects}" >>$result_log 2>&1 &
    )

    (
        killall laserMapping
        cd /opt/LidarSlam/velodyneSlam/laserMappingPackage \
            && chmod +x laserMapping \
            && bash laserMapping.sh "${_projects}" >>$result_log 2>&1 &
    )

    (
        killall transformMaintenance
        cd /opt/LidarSlam/velodyneSlam/transforMaintenancePackage \
            && chmod +x transformMaintenance \
            && bash transformMaintenance.sh >>$result_log 2>&1 &
    )

    (
        killall advertisePointcloud
        cd /opt/LidarSlam/velodyneSlam/advertisePointCloudPackage \
            && chmod +x advertisePointcloud \
            && bash advertisePointcloud.sh "${_projects}" >>$result_log 2>&1 &
    )

    return 0
}

main() {
    if [ "AA$1" = "AAserver" ]; then
        task_name=$2
        local absolute_record_path=${absolute_catkin_path}"/record/"${task_name}"/Rawdata/"
        mkdir -p "${absolute_record_path}"
        local absolute_glog_path=${absolute_catkin_path}"/record/"${task_name}"/Rawdata/Log"
        mkdir -p "${absolute_glog_path}"

        source_ROS_Env
        create_record_path "${absolute_record_path}"
        add_path_to_veledyne_launch "${absolute_record_path}"
        redirect_glog_path "${absolute_record_path}"
        start_smart_collector_server "${absolute_record_path}"
        return
    fi

    if [ "AA$1" = "AAcleanup" ]; then
        do_kill
        return
    fi

    if [ "AA$1" = "AAfixdata" ]; then
        local projects=$2
        do_fixdata "${projects}"
        return
    fi

    if [ "AA$1" = "AAdoSlam" ]; then
        local projects=$2
        do_slam "${projects}"
        return
    fi
}

main $@
exit $?

