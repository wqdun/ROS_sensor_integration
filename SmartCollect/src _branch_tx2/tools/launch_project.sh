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
    mkdir -p "${_absolute_record_path}/Image/panoramas/"
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

make_tty_softlink() {
    log_with_time "$FUNCNAME start, param: $*"

    local ls_res=$(ls /dev/ttyUSB*)
    local usb_num=$(echo ${ls_res} | awk '{print NF}')
    if [ "${usb_num}" -lt 3 ]; then
        log_with_time "usb_num should >= 3: ${usb_num}"
        exit ${usb_num}
    fi

    rm /dev/novatel_usb*
    local novatel_usb3=$(echo ${ls_res} | awk '{print $NF}')
    local novatel_usb2=$(echo ${ls_res} | awk '{print $(NF-1)}')
    local novatel_usb1=$(echo ${ls_res} | awk '{print $(NF-2)}')

    ln -s ${novatel_usb3} /dev/novatel_usb3
    ln -s ${novatel_usb2} /dev/novatel_usb2
    ln -s ${novatel_usb1} /dev/novatel_usb1
}

make_serial_softlink_by_path() {
    log_with_time "$FUNCNAME start, param: $*; HOSTNAME: ${HOSTNAME}"

    # sc0010 differ from sc0009
    if [ "AA${HOSTNAME}" = "AAsc0010" ]; then
        rm /dev/imuRawIns
        local imuRawIns=$(ls /dev/serial/by-path | grep "usb-0:4:1.0" | head -n1)
        if [ "AA${imuRawIns}" = "AA" ]; then
            log_with_time "Failed to find usb-0:4:1.0 in /dev/serial/by-path."
            exit 1
        fi
        ln -s "/dev/serial/by-path/"${imuRawIns} /dev/imuRawIns

        rm /dev/timeStamper
        local timeStamper=$(ls /dev/serial/by-path | grep "usb-0:3:1.0" | head -n1)
        if [ "AA${timeStamper}" = "AA" ]; then
            log_with_time "Failed to find usb-0:3:1.0 in /dev/serial/by-path."
            exit 1
        fi
        ln -s "/dev/serial/by-path/"${timeStamper} /dev/timeStamper
    elif [ "AA${HOSTNAME}" = "AAsc0009" ]; then
        rm /dev/imuRawIns
        local imuRawIns=$(ls /dev/serial/by-path | grep "usb-0:4:1.0" | head -n1)
        if [ "AA${imuRawIns}" = "AA" ]; then
            log_with_time "Failed to find usb-0:4:1.0 in /dev/serial/by-path."
            exit 1
        fi
        ln -s "/dev/serial/by-path/"${imuRawIns} /dev/imuRawIns

        rm /dev/timeStamper
        local timeStamper=$(ls /dev/serial/by-path | grep "usb-0:2:1.0" | head -n1)
        if [ "AA${timeStamper}" = "AA" ]; then
            log_with_time "Failed to find usb-0:2:1.0 in /dev/serial/by-path."
            exit 1
        fi
        ln -s "/dev/serial/by-path/"${timeStamper} /dev/timeStamper
    else
        log_with_time "Unknown HOSTNAME: ${HOSTNAME}"
    fi
}

start_smart_collector_server() {
    log_with_time "$FUNCNAME start, param: $*"
    log_with_time "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH}"
    local _absolute_record_path=$1

    log_with_time "sysctl -a start."
    sysctl -a >>$result_log 2>&1
    log_with_time "ulimit start."
    ulimit -c >>$result_log 2>&1

    get_sudo_permission
    # make_tty_softlink
    make_serial_softlink_by_path

    get_sudo_permission
    sudo chmod +r /dev/imuRawIns
    local task_keyword="sc_rawimu_rec"
    pkill "${task_keyword}"
    echo "pkill -INT ${task_keyword}; pkill ${task_keyword}" >"/tmp/kill_smartc.sh"
    /opt/smartc/devel/lib/sc_rawimu_recorder/sc_rawimu_recorder_node "/dev/imuRawIns" "${_absolute_record_path}/IMU/" &
    sleep 0.2

    local task_keyword="sc_images_time"
    pkill "${task_keyword}"
    echo "pkill -INT ${task_keyword}; pkill ${task_keyword}" >>"/tmp/kill_smartc.sh"
    /opt/smartc/devel/lib/sc_images_timestamper/sc_images_timestamper_node "/dev/timeStamper" "${_absolute_record_path}/IMU/" &
    sleep 0.2

    local task_keyword="sc_hik_camer"
    pkill "${task_keyword}"
    echo "pkill -INT ${task_keyword}; pkill ${task_keyword}" >>"/tmp/kill_smartc.sh"
    /opt/smartc/devel/lib/sc_hik_camera/sc_hik_camera_node "${_absolute_record_path}/" &

    killall nodelet
    echo "killall nodelet" >>"/tmp/kill_smartc.sh"
    roslaunch velodyne_pointcloud VLP16_points.launch &
    sleep 0.2

    local task_keyword="sc_project_mon"
    pkill "${task_keyword}"
    echo "pkill -INT ${task_keyword}; pkill ${task_keyword}" >>"/tmp/kill_smartc.sh"
    /opt/smartc/devel/lib/sc_project_monitor/sc_project_monitor_node "${_absolute_record_path}/" &
    sleep 0.2

    local task_keyword="sc_map_node"
    pkill "${task_keyword}"
    echo "pkill -INT ${task_keyword}; pkill ${task_keyword}" >>"/tmp/kill_smartc.sh"
    /opt/smartc/devel/lib/sc_map/sc_map_node "${_absolute_record_path}/" &
    sleep 0.2
}

do_kill() {
    log_with_time "$FUNCNAME start."

    bash "/tmp/kill_smartc.sh"
    return 0
}

do_fixdata() {
    log_with_time "$FUNCNAME start, param: $*"

    local _projects=$1
    /opt/smartc/devel/lib/sc_data_fixer/sc_data_fixer_node "${_projects}" &
    return 0
}

do_restart_server() {
    log_with_time "$FUNCNAME start, param: $*"

    pkill -INT sc_server_d; pkill sc_server_d
    /opt/smartc/devel/lib/sc_server_daemon/sc_server_daemon_node &
    return 0
}

main() {
    if [ "AA$1" = "AAserver" ]; then
        task_name=$2
        echo "${task_name}" | grep "9999" >/dev/null 2>&1
        if [ $? -eq 0 ]; then
            local absolute_record_path="/tmp/${task_name}/Rawdata/"
        else
            local absolute_record_path="${absolute_catkin_path}/record/${task_name}/Rawdata/"
        fi

        sysctl -w kernel.core_pattern=/var/crash/core.%u.%e.%p.%t
        sysctl -w fs.suid_dumpable=1
        ulimit -c unlimited >>result_log 2>&1

        local absolute_glog_path="${absolute_record_path}/Log"
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

    if [ "AA$1" = "AArestart_server" ]; then
        do_restart_server
        return
    fi
}

main $@
exit $?

