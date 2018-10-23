#!/bin/bash
clear
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
absolute_catkin_path=$(cd ${absolute_script_path}/../.. && pwd)

get_sudo_permission() {
    echo nvidia | sudo -S su >/dev/null 2>&1
}

get_sudo_permission
sudo mkdir -p /opt/smartc/log/
result_log="/opt/smartc/log/${script_name}.log"
get_sudo_permission
sudo touch ${result_log}
sudo chmod a+w ${result_log}

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
}

main() {
    log_with_time "$FUNCNAME start, param: $*"
    get_sudo_permission
    sudo chmod -R 777 /dev/bus/ >>$result_log 2>&1
    /opt/smartc4/src/sc_hik_camera/build/system_test

    return
}

main $@
exit $?

