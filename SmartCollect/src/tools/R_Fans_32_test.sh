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

_absolute_record_path="/opt/smartc/record/1001-1-RFANS32-180726/Rawdata/"
mv "${_absolute_record_path}" "/opt/smartc/record/1001-1-RFANS32-180726/Rawdata_$(date +%Y%m%d_%H_%M_%S)"
if [ "${_absolute_record_path}AA" == "AA" ]; then
    echo "[ERROR] Need a path of Rawdata."
    exit 1
fi

mkdir -p ${_absolute_record_path}/IMU/

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
mv /opt/smartc/record/lidar.dat /opt/smartc/record/lidar_$(date +%Y%m%d_%H_%M_%S).dat
roslaunch rfans_driver node_manager_with_data_save_path.launch


# to monitor: watch -n1 -d "ls -l /opt/smartc/record/lidar.dat; ls -l /opt/smartc/record/1001-1-RFANS32-180726/Rawdata/IMU; ls -l /opt/smartc/record/1001-1-RFANS32-180726/Rawdata/Image | wc -l;"
