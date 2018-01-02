# Copyright 2014 Intel Corporation, All Rights Reserved.
# log function library
# website: http://blog.csdn.net/gong_xucheng/article/details/50952672

log_info() {
    DATE_N=`date "+%Y-%m-%d %H:%M:%S"`
    USER_N=`whoami`
    echo "${DATE_N} [INFO] $@" >>${log_file}
}

log_error() {
    DATE_N=`date "+%Y-%m-%d %H:%M:%S"`
    USER_N=`whoami`
    echo -e "${DATE_N} [ERROR] $@" >>${log_file}
}

fn_log() {
    if [ $? -eq 0 ]; then
        log_info "$@"
    else
        log_error "$@"
        exit 1
    fi
}
