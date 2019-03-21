#!/bin/bash

### BEGIN INIT INFO
# Provides:          slam_daemon.bash
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

script_name=$(basename $0)
absolute_script_path=$(cd $(dirname $0) && pwd)
mkdir -p /opt/smartc/log/
result_log=/opt/smartc/log/${script_name}".log"

. "${absolute_script_path}/lib.sh"

do_start() {
    log_with_time "$FUNCNAME start."

    su root -c '/usr/bin/tightvncserver :6'

    log_with_time "$FUNCNAME return $?."
}

do_stop() {
    log_with_time "$FUNCNAME start."

    pkill Xtightvnc

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









