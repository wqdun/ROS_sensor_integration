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

_date=$(date +%Y%m%d%H%M%S)
_date=${_date:2}
_absolute_record_path="/opt/smartc/record/1001-1-RFANS32-${_date}/Rawdata/"
mkdir -p "${_absolute_record_path}"
mkdir -p "${_absolute_record_path}/Lidar/"
mkdir -p "${_absolute_record_path}/IMU/"

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

add_path_to_launch() {
    log_with_time "$FUNCNAME start, param: $*"
    local _absolute_record_path=$1
    local absolute_launch="/opt/smartc/src/ROSDriver/launch/node_manager.launch"
    local parse_xml_script="/opt/smartc/src/tools/parse_xml.py"
    chmod +x "${parse_xml_script}"
    "${parse_xml_script}" "${absolute_launch}" "calculation_node" "data_save_path" "${_absolute_record_path}/Lidar/" "${absolute_launch%.*}_with_data_save_path.launch"

    log_with_time "Add record path ${_absolute_record_path} to launch file: ${absolute_launch}."
}

source_ROS_Env

get_sudo_permission
sudo chown -R navi:navi /opt/smartc/record/

get_sudo_permission
sudo chmod +r /dev/ttyUSB0
pkill sc_integrate_
rosrun sc_integrate_imu_recorder sc_integrate_imu_recorder_node "${_absolute_record_path}/IMU/" &
sleep 0.2

get_sudo_permission
sudo chmod +r /dev/ttyS0
pkill sc_camera
/opt/smartc/devel/lib/sc_camera/sc_camera jpg "${_absolute_record_path}/" &
sleep 0.2

pkill calculation_
pkill driver_node
add_path_to_launch "${_absolute_record_path}"
roslaunch rfans_driver node_manager_with_data_save_path.launch

# to monitor: _date=$(date +%Y%m%d); _date=${_date:2}; watch -n1 -d "ls -l /opt/smartc/record/lidar.dat; ls -l /opt/smartc/record/1001-1-RFANS32-${_date}/Rawdata/IMU; ls -l /opt/smartc/record/1001-1-RFANS32-${_date}/Rawdata/Image | wc -l;"
