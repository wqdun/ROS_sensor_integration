#!/bin/bash

result_log=/tmp/tmp.log
check_WiFi_network() {
    echo "$FUNCNAME start."

    for (( i = 1; i < 5; ++i )); do
        /sbin/ip -s link | grep "^[0-9]" | grep -E "wl" | head -n1 | grep "UP" >>$result_log 2>&1
        if [ $? -eq 0 ]; then
            echo "Failed to setup WiFi network, gonna restart NetworkManager ${i} time."
            /usr/sbin/service NetworkManager restart
            sleep 20
        else
            echo "Setup WiFi network successfully ${i} time."
            break
        fi
    done

    echo "$FUNCNAME end."
}

check_WiFi_network