#!/bin/bash
absolute_script_path=$(cd $(dirname $0) && pwd)
echo "${absolute_script_path}"
cd ../../record
script_name=$(basename $0)
result_log="${absolute_script_path}/${script_name}.log"

log_with_time() {
    local now_time=$(date +%Y/%m/%d-%H:%M:%S)
    echo "$now_time: $*" >>$result_log
}

projectAllPath=$(pwd)
for projectSeq in $(ls)
do
    if [ ! -d "${projectAllPath}/${projectSeq}/Process" ];then
        log_with_time "Process File does not exist"
        continue
    fi
    ls "${projectSeq}/Process" *.md5 >> "${result_log}" 2>&1
    if [ $? -eq 0 ]; then
        log_with_time "*.md5 already exist."
        continue
    fi
    log_with_time "start to create md5."
    md5FileCreatTime=$(date +%H%M%S)
    projectName=$(echo $projectSeq | tr -d "-")
    md5FileName="${projectAllPath}/${projectSeq}/Process/${projectName}${md5FileCreatTime}.md5"
    for oneFile in $(ls ${projectSeq}/Process/ );do
        if [ ${oneFile} == *.md5 ];then
            continue
        fi
        md5sum "${projectSeq}/Process/${oneFile}" >> ${md5FileName}
    done

    if [ ! -d "${projectAllPath}/${projectSeq}/Rawdata" ];then
        log_with_time "Rawdata File does not exist"
        continue
    fi
    if [ ! -d "${projectAllPath}/${projectSeq}/Rawdata/Image" ];then
        log_with_time "Image File does not exist"
        continue
    fi

    for img_file in $(ls ${projectSeq}/Rawdata/Image ); do
        md5sum "${projectSeq}/Rawdata/Image/${img_file}" >> $md5FileName
    done
    if [ ! -d "${projectAllPath}/${projectSeq}/Rawdata/Lidar" ];then
        log_with_time "Lidar File does not exist"
        continue
    fi
    for lidarFile in $(ls ${projectSeq}/Rawdata/Lidar);do
        md5sum "${projectSeq}/Rawdata/Lidar/${lidarFile}" >> $md5FileName
    done
    if [ ! -d "${projectAllPath}/${projectSeq}/Rawdata/IMU" ];then
        log_with_time "IMU File does not exist"
        continue
    fi
    for IMUFile in $(ls ${projectSeq}/Rawdata/IMU);do
        md5sum "${projectSeq}/Rawdata/IMU/${IMUFile}" >> $md5FileName
    done
     if [ ! -d "${projectAllPath}/${projectSeq}/Rawdata/Log" ];then
        log_with_time "Log File does not exist"
        continue
    fi
    for LogFile in $(ls ${projectSeq}/Rawdata/Log);do
        md5sum "${projectSeq}/Rawdata/Log/${LogFile}" >> $md5FileName
    done


done
