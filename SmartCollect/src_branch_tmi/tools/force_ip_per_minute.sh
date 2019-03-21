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

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
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

force_ip() {
    log_with_time "$FUNCNAME start."

    local net_card_num=$(/usr/bin/lspci | grep Ethernet | wc -l)
    if [ $net_card_num -gt 4 ]; then
        log_with_time "I have $net_card_num network cards, I am gonna force IP."
        /opt/smartc/devel/lib/sc_camera_ip_forcer/sc_camera_ip_forcer_node
        log_with_time "sc_camera_ip_forcer_node returns $?, view details in /opt/smartc/log/sc_camera_ip_forcer_node.INFO"
    else
        log_with_time "I have $net_card_num network cards, I need not force IP."
    fi
}

set_camera_mtu() {
    log_with_time "$FUNCNAME start."

    local mtu=$(netstat -i | grep eth0 | awk '{print $2}')
    log_with_time "I have $mtu mtu at eth0."
    if [ $mtu -eq 9000 ]; then
        log_with_time "MTU already set to 9000."
        return
    fi

    ifconfig eth0 mtu 9000 >>$result_log 2>&1
    ifconfig eth1 mtu 9000 >>$result_log 2>&1
    ifconfig eth2 mtu 9000 >>$result_log 2>&1
    ifconfig eth3 mtu 9000 >>$result_log 2>&1
    ifconfig eth4 mtu 9000 >>$result_log 2>&1
    ifconfig eth5 mtu 9000 >>$result_log 2>&1

    local mtu=$(netstat -i | grep eth0 | awk '{print $2}')
    log_with_time "Now I have $mtu mtu at eth0, wait 10s for MTU taking effect."
    sleep 10

    log_with_time "$FUNCNAME return $?."
}

is_smartc_running() {
    log_with_time "$FUNCNAME start."

    pidof sc_camera >>$result_log 2>&1
    return $?
}

is_ethernet_power_on() {
    log_with_time "$FUNCNAME start."

    ifconfig eth0 | grep RUNNING >>$result_log 2>&1
    return $?
}


main() {
    log_with_time "$FUNCNAME start."

    is_smartc_running
    if [ $? -eq 0 ]; then
        log_with_time "SmartC is running."
        return
    fi
    log_with_time "SmartC is not running."

    is_ethernet_power_on
    if [ $? -ne 0 ]; then
        log_with_time "Ethernet is power off."
        return
    fi
    log_with_time "Ethernet is power on."

    log_with_time "Gonna set IPs."
    set_camera_network
    set_camera_mtu
    force_ip
    return
}

main $@
exit $?

# * * * * * root [ $(/sbin/ifconfig eth0 | /bin/grep "RUNNING" | /usr/bin/wc -l) -gt 0 ] && /opt/smartc/devel/lib/sc_camera_ip_forcer/sc_camera_ip_forcer_node || /bin/echo $(/bin/date +%Y/%m/%d-%H:%M:%S): "Ethernet not power on, no need to force camera-IP." >>/opt/smartc/log/force_ip_per_minute.log
