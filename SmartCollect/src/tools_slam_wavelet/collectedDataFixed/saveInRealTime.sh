#!/bin/bash
#ps aux | grep "saveInRealTime.sh" | grep -v "grep" | grep -v "$$"
#if [ "$?" -eq 0 ];then
#    exit
#fi
dir_old=$(pwd)
user=$(whoami)
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
log="${absolute_script_path}/${script_name}.log"

get_sudo_permission()
{
    echo 123 | sudo -S su >/dev/null 2>&1
}
log_with_time()
{
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$log
}

jugeFileExist()
{
    local totalResouceDir=$1
    local totalTargetDir=$2
    local resouceFloder=$3
    local flag=0
    for targetFloder in $(ls ${totalTargetDir})
    do
        if [ ${resouceFloder} == ${targetFloder} ];then
            diff ${totalResouceDir}/${resouceFloder} ${totalTargetDir}/${targetFloder} >/dev/null 2>&1
            if [ $? -eq 0 ];then
                log_with_time "${totalResouceDir}/${resouceFloder} does not copy"
                flag=1
            fi
        fi
    done
    if [ ${flag} == 0 ]; then
        cp -rf ${totalResouceDir}/${resouceFloder} ${totalTargetDir}/
        log_with_time "copy :${totalResouceDir}/${resouceFloder} To ${totalTargetDir}/"
    fi
}

getTargetRootPath()
{
    log_with_time "$FUNCNAME start."

    local diskNum=$(ls "/media/${user}/" | wc -l)
    if [ ${diskNum} -ne 1 ]; then
        log_with_time "[ERROR] diskNum is ${diskNum}, should be 1."
        exit
    fi

    local diskName=$(ls -d /media/${user}/*)
    echo "${diskName}"
}

copyFile()
{
    local resouceDir=$1
    local targetDir=$2
    ##调用文强代码
    # local negativeProject=$(getNegativeProjectPath ${resouceDir})

    for oneResourceDir in $(ls -t ${resouceDir})
    do
        isActiveProjectPath "${resouceDir}/${oneResourceDir}"
        if [ $? -ne 0 ]; then
            log_with_time "${resouceDir}/${oneResourceDir} is an active project."
            continue
        fi

        jugeFileExist ${resouceDir} ${targetDir} ${oneResourceDir}
    done
}

isActiveProjectPath() {
    log_with_time "$FUNCNAME start, check $1."

    local projectPath=$1

    local logPath="${projectPath}/Rawdata/Log/"
    local latestLog=$(ls -t "${logPath}" | head -n1)
    local absLatestLog="${logPath}/${latestLog}"
    local modifyTimeOfLatestLog=$(stat -t "${absLatestLog}" | awk '{print $14}')
    local timeNow=$(date +%s)
    local timeErr=$((${timeNow}-${modifyTimeOfLatestLog}))
    if [ "${timeErr}" -le "10" ]; then
        log_with_time "${latestLog} is active."
        return 1
    fi

    log_with_time "${latestLog} is not active."
    return 0
}

main()
{
    local resoucePath="/opt/smartc/record/"
    local targetPath="$(getTargetRootPath)/"
    echo $targetPath
    log_with_time "resoucePath: ${resoucePath}; targetPath: ${targetPath}"
    copyFile "${resoucePath}" "${targetPath}"
}

main
exit $?

