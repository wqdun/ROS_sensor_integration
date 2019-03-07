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

copyFile()
{
    resouceFileOneDir=$1
    targetFileOneDir=$2
    resouceFile=$3
    fileExistMark=0
    dirMark=0
    if [ -d ${resouceFileOneDir}/${resouceFile} ];then
        dirMark=1
    else
        oneSouceFileSizv4e=$(du --max-depth=0 "${resouceFileOneDir}/${resouceFile}" | awk {'print $1'})
    fi

    for targetOneFile in $(ls ${targetFileOneDir})
    do
        if [ ${resouceFile} == ${targetOneFile} ];then
            if [ "${dirMark}" == 1 ];then
                fileExistMark=1
                break
            else
                oneTargetFileSize=$(du --max-depth=0 "${targetFileOneDir}/${targetOneFile}" | awk {'print $1'})
                if [ AA$oneTargetFileSize == AA$oneSouceFileSize ];then
                    fileExistMark=1
                    break
                fi
            fi
        fi
    done
    if [ ${fileExistMark} == 0 ];then
        if [ "${dirMark}" == 1 ];then
            mkdir -p ${targetFileOneDir}/${resouceFile} >> "${log}" 2>&1
            log_with_time "make the dir:from ${resouceFileOneDir}/${resouceFile} to ${targetFileOneDir}/${resouceFile}"
        else
            cp ${resouceFileOneDir}/${resouceFile} ${targetFileOneDir} >> "${log}" 2>&1
            log_with_time "copy the file:from ${resouceFileOneDir}/${resouceFile} to ${targetFileOneDir}"
        fi
    fi
}


makeDir()
{
    local resocuceFileDir=$1
    local targetFileDir=$2

    for resouceFile in $(ls ${resocuceFileDir})
    do
        if [ -d ${resocuceFileDir}/${resouceFile} ];then
            if [ ${resouceFile} == "Temp" ];then
                continue
            fi
            copyFile "${resocuceFileDir}" "${targetFileDir}" "${resouceFile}"
            resouceFileSize=$(du --max-depth=0 "${resocuceFileDir}/${resouceFile}" | awk {'print $1'})
            targetFileSize=$(du --max-depth=0 "${targetFileDir}/${resouceFile}" | awk {'print $1'})
            log_with_time "the resouce dir: ${resocuceFileDir}/${resouceFile} size: ${resouceFileSize}"
            log_with_time "the target dir: ${targetFileDir}/${resouceFile} size: ${targetFileSize}"
            if [ AA${resouceFileSize} == AA${targetFileSize} ];then
                continue
                log_with_time "dir size is the same"
            fi
            makeDir "${resocuceFileDir}/${resouceFile}" "${targetFileDir}/${resouceFile}"
        else
            copyFile "${resocuceFileDir}" "${targetFileDir}" "${resouceFile}"
        fi
    done
}

getTargetRootPath()
{
    log_with_time "$FUNCNAME start."

    local diskNum=$(ls "/media/${user}/" | wc -l)
    if [ ${diskNum} -ne 1 ]; then
        log_with_time "[ERROR] diskNum is ${diskNum}, should be 1."
        # exit
    fi

    local diskName=$(ls -d /media/${user}/*1)
    echo "${diskName}"
}




getNegativeProjectPath() {
    echo "$FUNCNAME start."

    local recordPath=$1
    local latestProject=$(ls -t "${recordPath}" | head -n1)
    if [ "AA${latestProject}" = "AA" ]; then
        log_with_time "No files found in ${recordPath}."
        echo ""
        return
    fi

    local latestLogPath="${recordPath}/${latestProject}/Rawdata/Log/"
    local latestLog=$(ls -t "${latestLogPath}" | head -n1)
    local modifyTimeOfLatestLog=$(stat -t "${latestLog}" | awk '{print $14}')
    local timeNow=$(date +%s)
    local timeErr=$((${timeNow}-${modifyTimeOfLatestLog}))
    if [ "${timeErr}" -le "10" ]; then
        log_with_time "${latestLog} is active."
        echo ""
        return
    fi

    log_with_time "${latestLog} is not active."
    echo "${latestProject}"
    return
}

main()
{
    local resoucePath="/opt/smartc/recordTest/"
    local targetPath="$(getTargetRootPath)/"
    echo $targetPath

    log_with_time "resoucePath: ${resoucePath}; targetPath: ${targetPath}"
    makeDir "${resoucePath}" "${targetPath}"

}

main
exit $?

