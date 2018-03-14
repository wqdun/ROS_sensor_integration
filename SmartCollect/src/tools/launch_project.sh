#!/bin/bash
clear
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
absolute_catkin_path=$(cd ${absolute_script_path}/../.. && pwd)

task_name=$2
absolute_record_path=${absolute_catkin_path}"/record/"${task_name}"/Rawdata/"
mkdir -p "${absolute_record_path}"
absolute_glog_path=${absolute_catkin_path}"/record/"${task_name}"/Rawdata/Log"
mkdir -p "${absolute_glog_path}"
result_log="${absolute_glog_path}/${script_name}.log"
cp /dev/null $result_log

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
    log_with_time "$FUNCNAME start."
    mkdir -p "${absolute_record_path}/Image/"
    mkdir -p "${absolute_record_path}/IMU/"
    mkdir -p "${absolute_record_path}/Lidar/"
}

add_path_to_veledyne_launch() {
    log_with_time "$FUNCNAME start."
    local absolute_velodyne_launch="${absolute_catkin_path}/src/velodyne/velodyne_driver/launch/nodelet_manager.launch"
    local parse_xml_script="${absolute_script_path}/parse_xml.py"
    chmod +x "${parse_xml_script}"
    "${parse_xml_script}" "${absolute_record_path}" "${absolute_velodyne_launch}"

    log_with_time "Add record path ${absolute_record_path} to launch file: ${absolute_velodyne_launch}."
}

redirect_glog_path() {
    log_with_time "$FUNCNAME start."
    mkdir -p "${absolute_record_path}/Log/"
    export GLOG_log_dir="${absolute_record_path}/Log/"
}

start_smart_collector_server() {
    log_with_time "$FUNCNAME start."
    log_with_time "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"

    get_sudo_permission
    sudo chmod +r /dev/ttyUSB0
    pkill sc_integrate_
    rosrun sc_integrate_imu_recorder sc_integrate_imu_recorder_node "${absolute_record_path}/IMU/" &
    sleep 0.2

    get_sudo_permission
    sudo chmod +r /dev/ttyS0
    sudo ifconfig eth0 mtu 9000
    pkill roscameragps
    rosrun roscameragpsimg roscameragpsimg jpg "${absolute_record_path}/Image/" "${absolute_record_path}/IMU/" &
    sleep 0.2

    killall nodelet
    roslaunch velodyne_pointcloud VLP16_points.launch &
    sleep 0.2
}

start_smart_collector_client() {
    log_with_time "$FUNCNAME start."
    log_with_time "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"

    pkill sc_center_
    rosrun sc_center sc_center_node &
    sleep 0.2
}

do_rviz() {
    log_with_time "$FUNCNAME start."
    log_with_time "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"

    pkill display_had_
    # rosrun display_had display_had_node &
    sleep 0.2

    killall rviz
    /opt/ros/indigo/bin/rviz >>$result_log 2>&1 &
    sleep 0.2
}

main() {
    if [ "AA$1" = "AAserver" ]; then
        source_ROS_Env
        create_record_path
        add_path_to_veledyne_launch
        redirect_glog_path
        start_smart_collector_server
        return
    fi

    if [ "AA$1" = "AAclient" ]; then
        source_ROS_Env
        redirect_glog_path
        start_smart_collector_client
        return
    fi

    if [ "AA$1" = "AArviz" ]; then
        source_ROS_Env
        redirect_glog_path
        do_rviz
        return
    fi
}

main $@
exit $?

