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
result_log=/var/log/${script_name}".log"

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
}

do_start() {
    source /opt/ros/indigo/setup.bash >>$result_log 2>&1
    /opt/ros/indigo/bin/roscore >>$result_log 2>&1 &
}

do_stop() {
    kill $(pgrep roscore) >>$result_log 2>&1
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
