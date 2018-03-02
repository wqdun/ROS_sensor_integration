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
result_log=/opt/smartc/log/${script_name}".log"

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
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

do_start() {
    source /opt/ros/indigo/setup.bash >>$result_log 2>&1
    /opt/ros/indigo/bin/roscore >>$result_log 2>&1 &
    sleep 0.5
    run_sc_server_daemon_node
}

do_stop() {
    log_with_time "$FUNCNAME start."
    kill $(pgrep roscore) >>$result_log 2>&1
    # kill sc_server_daemon_node by key word
    pkill sc_server_d >>$result_log 2>&1

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
