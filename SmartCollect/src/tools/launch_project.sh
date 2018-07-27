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
    "${parse_xml_script}" "${_absolute_record_path}" "${absolute_velodyne_launch}"

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
    pkill sc_camera

    local net_card_num=$(/usr/bin/lspci | grep Ethernet | wc -l)
    if [ $net_card_num -gt 4 ]; then
        log_with_time "I have $net_card_num network cards, I am gonna force IP."
        /opt/smartc/devel/lib/sc_camera_ip_forcer/sc_camera_ip_forcer_node
        sleep 5
    else
        log_with_time "I have $net_card_num network cards, I need not force IP."
    fi
    /opt/smartc/devel/lib/sc_camera/sc_camera jpg "${_absolute_record_path}/" &
    sleep 0.2

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
}

main $@
exit $?

