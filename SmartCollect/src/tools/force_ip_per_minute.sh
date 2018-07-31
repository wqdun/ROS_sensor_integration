#!/bin/bash
clear
SHELL=/bin/bash
PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
absolute_catkin_path=$(cd ${absolute_script_path}/../.. && pwd)

mkdir -p /opt/smartc/log/
result_log="/opt/smartc/log/${script_name}.log"

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

force_ip() {
    log_with_time "$FUNCNAME start."

    local net_card_num=$(/usr/bin/lspci | grep Ethernet | wc -l)
    if [ $net_card_num -gt 4 ]; then
        log_with_time "I have $net_card_num network cards, I am gonna force IP."
        sleep 5
        /opt/smartc/devel/lib/sc_camera_ip_forcer/sc_camera_ip_forcer_node
    else
        log_with_time "I have $net_card_num network cards, I need not force IP."
    fi
}

main() {
    log_with_time "$FUNCNAME start."
}

main $@
exit $?

# * * * * * root [ $(/sbin/ifconfig eth0 | /bin/grep "RUNNING" | /usr/bin/wc -l) -gt 0 ] && /opt/smartc/devel/lib/sc_camera_ip_forcer/sc_camera_ip_forcer_node || /bin/echo $(/bin/date +%Y/%m/%d-%H:%M:%S): "Ethernet not power on, no need to force camera-IP." >>/opt/smartc/log/force_ip_per_minute.log
