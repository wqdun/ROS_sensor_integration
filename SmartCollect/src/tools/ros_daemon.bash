#!/bin/bash

### BEGIN INIT INFO
# Provides:          ros_daemon.bash
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

script_name=$(basename $0)
mkdir -p /opt/smartc/log/
result_log=/opt/smartc/log/${script_name}".log"

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
}

mount_data_disk() {
    log_with_time "$FUNCNAME start."

    local disk_num=$(grep -v [0-9]$ /proc/partitions | grep -v ^$ | grep -cv name)
    if [ $disk_num -ne 2 ]; then
        log_with_time "I have $disk_num disk, not 2."
        return
    fi

    log_with_time "I have 2 disks."
    local disk1=$(grep -v [0-9]$ /proc/partitions | grep -v ^$ | grep -v name | head -n1 | awk '{print $4}')
    local disk2=$(grep -v [0-9]$ /proc/partitions | grep -v ^$ | grep -v name | tail -n1 | awk '{print $4}')
    local root_mounter=$(df | grep "/$" | awk '{print $1}' | awk -F/ '{print $3}')
    local is_root_mount_disk1=$(echo "$root_mounter" | grep "$disk1")

    local data_disk="${disk2}"
    if [ "AA${is_root_mount_disk1}" = "AA" ]; then
        data_disk="${disk1}"
        log_with_time "$disk1 is data disk, $disk2 is / mounter."
    fi

    local data_disk_partition_num=$(ls /dev/* | grep "$data_disk" | grep -vc "$data_disk$")
    if [ $data_disk_partition_num -ne 1 ]; then
        log_with_time "I have ${data_disk_partition_num} disk partitions in ${data_disk}, not 1."
        return
    fi
    local data_disk_partition=$(ls /dev/* | grep "$data_disk" | grep -v "$data_disk$")
    log_with_time "I have ${data_disk_partition_num} disk partition: ${data_disk_partition} in ${data_disk}."

    mkdir -p /opt/smartc/record/
    mount "$data_disk_partition" /opt/smartc/record/
    log_with_time "$FUNCNAME success."
    return
}

run_sc_server_daemon_node() {
    log_with_time "$FUNCNAME start."
    log_with_time "ROS_MASTER_URI: $ROS_MASTER_URI"

    if [ -f "/opt/smartc/devel/lib/sc_server_daemon/sc_server_daemon_node" ]; then
        /opt/smartc/devel/lib/sc_server_daemon/sc_server_daemon_node &
    else
        log_with_time "[ERROR] Failed to find sc_server_daemon_node."
    fi

    log_with_time "$FUNCNAME return $?."
}

export_java_env() {
    log_with_time "$FUNCNAME start."
    log_with_time "JRE_HOME: $JRE_HOME"

    export JAVA_HOME=/opt/jvm/java/
    export JRE_HOME=${JAVA_HOME}/jre
    export CLASSPATH=.:${JAVA_HOME}/lib:${JRE_HOME}/lib
    export PATH=${JAVA_HOME}/bin:$PATH
    log_with_time "JRE_HOME: $JRE_HOME"
}

run_tomcat() {
    log_with_time "$FUNCNAME start."

    export_java_env
    /opt/apache-tomcat-*/bin/startup.sh >>$result_log 2>&1
    log_with_time "$FUNCNAME return $?."
}

run_rosbridge() {
    log_with_time "$FUNCNAME start."

    if [ ! -f "/opt/smartc/devel/setup.bash" ]; then
        log_with_time "[ERROR] Failed to find /opt/smartc/devel/setup.bash."
        return
    fi
    if [ ! -f "/opt/ros/indigo/share/rosbridge_server/launch/rosbridge_websocket.launch" ]; then
        log_with_time "[ERROR] Failed to find rosbridge_websocket.launch."
        return
    fi
    if [ ! -f "/opt/ros/indigo/lib/web_video_server/web_video_server" ]; then
        log_with_time "[ERROR] Failed to find web_video_server"
        return
    fi

    . /opt/smartc/devel/setup.bash
    log_with_time "roslaunch start."
    roslaunch /opt/ros/indigo/share/rosbridge_server/launch/rosbridge_websocket.launch >>$result_log 2>&1 &
    sleep 1
    log_with_time "roslaunch end."

    . /opt/ros/indigo/setup.bash
    /opt/ros/indigo/lib/web_video_server/web_video_server >>$result_log 2>&1 &
    sleep 1
    log_with_time "$FUNCNAME return $?."
}

run_minemap_service() {
    log_with_time "$FUNCNAME start."

    service redis start >>$result_log 2>&1
    service postgresql start >>$result_log 2>&1
    service nginx start >>$result_log 2>&1
    (
        log_with_time "Run authorization script."
        cd /data/minemap/program/minemap-business/authorization/ && ./start.sh || log_with_time "Failed to run authorization script."
        sleep 1

        log_with_time "Run minemap-data script."
        cd /data/minemap/program/minemap-data/ && ./start.sh || log_with_time "Failed to run minemap-data script."
        sleep 1

        log_with_time "PATH: ${PATH}."
        log_with_time "Run minemap-business script."
        export PATH=/opt/ros/indigo/bin:/data/minemap/program/postgres/bin:/opt/java/jdk1.8.0_144/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games
        cd /data/minemap/program/minemap-business/minemap/ && ./start.sh || log_with_time "Failed to run minemap-business script."
        log_with_time "Now PATH is ${PATH}."
    )
    log_with_time "$FUNCNAME return $?."
}

kill_rosbridge() {
    log_with_time "$FUNCNAME start."

    pkill web_video_se
    killall roslaunch >>$result_log 2>&1
    log_with_time "$FUNCNAME return $?."
}

kill_tomcat() {
    log_with_time "$FUNCNAME start."
    log_with_time "JRE_HOME: $JRE_HOME"

    export_java_env
    /opt/apache-tomcat-*/bin/shutdown.sh >>$result_log 2>&1
    log_with_time "$FUNCNAME return $?."
}

set_network() {
    log_with_time "$FUNCNAME start."

    local net_card_num=$(/usr/bin/lspci | grep Ethernet | wc -l)
    if [ $net_card_num -gt 4 ]; then
        log_with_time "I have $net_card_num network cards, I am a Panorama SmartC Device."
        set_panorama_network
    else
        log_with_time "I have $net_card_num network cards, I am not a Panorama SmartC Device."
        set_singlecam_network
    fi

}

set_panorama_network() {
    log_with_time "$FUNCNAME start."
    # set_panorama_camera_network
    # set_panorama_lidar_network
}

set_singlecam_network() {
    log_with_time "$FUNCNAME start."
    set_singlecam_camera_network
    set_singlecam_lidar_network
}

set_singlecam_camera_network() {
    log_with_time "$FUNCNAME start."

    local mtu_eth0=$(netstat -i | grep eth0 | awk '{print $2}')
    log_with_time "I have $mtu_eth0 mtu at eth0."

    ifconfig eth0 mtu 9000 >>$result_log 2>&1
    ifconfig eth0 169.254.0.7 netmask 255.255.0.0 >>$result_log 2>&1

    local mtu_eth0_changed=$(netstat -i | grep eth0 | awk '{print $2}')
    log_with_time "Now I have $mtu_eth0_changed mtu at eth0."
}

set_singlecam_lidar_network() {
    log_with_time "$FUNCNAME start."

    ifconfig eth1 192.168.0.7 netmask 255.255.255.0 >>$result_log 2>&1
}

set_camera_mtu() {
    log_with_time "$FUNCNAME start."

    local mtu=$(netstat -i | grep eth0 | awk '{print $2}')
    log_with_time "I have $mtu mtu at eth0."

    ifconfig eth0 mtu 9000 >>$result_log 2>&1
    ifconfig eth1 mtu 9000 >>$result_log 2>&1
    ifconfig eth2 mtu 9000 >>$result_log 2>&1
    ifconfig eth3 mtu 9000 >>$result_log 2>&1
    ifconfig eth4 mtu 9000 >>$result_log 2>&1
    ifconfig eth5 mtu 9000 >>$result_log 2>&1

    local mtu=$(netstat -i | grep eth0 | awk '{print $2}')
    log_with_time "Now I have $mtu mtu at eth0."

    log_with_time "$FUNCNAME return $?."
}

set_camera_network() {
    log_with_time "$FUNCNAME start."

    ifconfig eth0 169.254.63.173 netmask 255.255.255.0 >>$result_log 2>&1
    ifconfig eth1 169.254.64.1 netmask 255.255.255.0 >>$result_log 2>&1
    ifconfig eth2 169.254.65.119 netmask 255.255.255.0 >>$result_log 2>&1
    ifconfig eth3 169.254.97.233 netmask 255.255.255.0 >>$result_log 2>&1
    ifconfig eth4 169.254.66.122 netmask 255.255.255.0 >>$result_log 2>&1
    ifconfig eth5 169.254.70.50 netmask 255.255.255.0 >>$result_log 2>&1
    ifconfig eth6 192.102.0.7 netmask 255.255.255.0 >>$result_log 2>&1
}

do_start() {
    log_with_time "$FUNCNAME start."

    set_network
    ifconfig >>$result_log
    sleep 1

    mount_data_disk
    sleep 1

    . /opt/ros/indigo/setup.bash
    /opt/ros/indigo/bin/roscore >>$result_log 2>&1 &
    sleep 1
    run_sc_server_daemon_node
    sleep 1
    run_tomcat
    sleep 1
    run_rosbridge
    sleep 1

    local net_card_num=$(/usr/bin/lspci | grep Ethernet | wc -l)
    if [ $net_card_num -gt 4 ]; then
        log_with_time "I have $net_card_num network cards, I am gonna force IP."
        sleep 5
        /opt/smartc/devel/lib/sc_camera_ip_forcer/sc_camera_ip_forcer_node
    else
        log_with_time "I have $net_card_num network cards, I need not force IP."
    fi

    run_minemap_service

    log_with_time "$FUNCNAME return $?."
}

do_stop() {
    log_with_time "$FUNCNAME start."

    kill_rosbridge
    kill_tomcat

    log_with_time "kill sc_server_daemon_node by key word."
    pkill sc_server_d
    kill $(pgrep roscore) >>$result_log 2>&1

    log_with_time "umount /opt/smartc/record/ start."
    umount /opt/smartc/record/ >>$result_log 2>&1

    log_with_time "$FUNCNAME return $?."
}

log_with_time "[BEGIN] $0 $*."

case "$1" in
start)
    do_start
    ;;
stop)
    do_stop
    ;;
restart)
    do_stop
    do_start
    ;;
*)
    log_with_time "Wrong parameter: $*."
    ;;
esac

log_with_time "[END] $0 $*."
exit 0
