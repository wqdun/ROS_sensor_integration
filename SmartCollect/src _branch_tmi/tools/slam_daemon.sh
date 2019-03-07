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
    # export DISPLAY=:0.0
    # ls /var/run/lightdm/root/:0 >>$result_log 2>&1
    # /usr/bin/x11vnc -display :0 -auth /var/run/lightdm/root/:0 -forever -bg -o /var/log/x11vnc.log -rfbauth /home/nvidia/.vnc/passwd -rfbport 5900 >>$result_log 2>&1
    # /usr/bin/x11vnc -auth guess -forever -loop -noxdamage -repeat -rfbauth /home/nvidia/.vnc/passwd -rfbport 5900 -shared >>$result_log 2>&1 &
    chmod -R 777 /dev/bus/ >>$result_log 2>&1
    ifconfig eth0 6.6.6.66 >>$result_log 2>&1
    ifconfig eth0 mtu 9000 >>$result_log 2>&1

    # xrandr --fb 1024x768 >>$result_log 2>&1
    log_with_time "$FUNCNAME return $?."
}

do_stop() {
    log_with_time "$FUNCNAME start."
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









