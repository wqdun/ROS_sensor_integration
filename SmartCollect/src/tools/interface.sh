dir_old=$(pwd)
# generally absolute_script_path is ~/catkin_ws/src/tools
absolute_script_path=$(cd $(dirname $0) && pwd)
script_name=$(basename $0)
absolute_catkin_path=$(cd ${absolute_script_path}/../.. && pwd)

# use log_info/log_error/fn_log
source ${absolute_script_path}/liblog.sh

# log to Desktop before creating project
export log_file=${HOME}/Desktop/${script_name}".log"
cp /dev/null ${log_file}

copy_planlayer_dir_to_task() {
    log_info "$FUNCNAME start."
    local planlayer_dir=$HOME"/Desktop/Planlayer"
    mkdir -p $planlayer_dir \
        && cp -r $planlayer_dir $absolute_task_path

    local tab_file_num=$(ls $planlayer_dir/*.TAB 2>/dev/null | wc -l)
    if [ $tab_file_num -ne 1 ]; then
        log_warn "Number of TAB file in $planlayer_dir is: ${tab_file_num}, should be 1."
    else
        log_info "Number of TAB file in $planlayer_dir is: ${tab_file_num}."
    fi

    return 0
}

# source ROS environment
ROS_setup=${absolute_catkin_path}"/devel/setup.bash"
if [ ! -f "${ROS_setup}" ]; then
    log_error "${ROS_setup} does not exist."
    exit -1
fi
source ${ROS_setup} >/dev/null 2>&1

# auto input password
echo 123 | sudo -S su >/dev/null 2>&1
sudo chmod +r /dev/ttyS0
sudo chmod +r /dev/ttyUSB0
sudo ifconfig eth0 mtu 9000
fn_log "ifconfig eth0."

# new build a project directory: e.g., record/1001-1-071-171227/Rawdata
date_YMD=$(date +%Y%m%d)
chmod +x ${absolute_script_path}/task_config.py
task_name=$(${absolute_script_path}/task_config.py ${HOME}/Desktop/task_config.json)"-"${date_YMD:2}
absolute_record_path=${absolute_catkin_path}"/record/"${task_name}"/Rawdata"
if [ -d ${absolute_record_path} ]; then
    log_error "${absolute_record_path} already exist, go modify task_config.json."
    exit -1
fi
mkdir -p ${absolute_record_path}

# Log path
absolute_log_path="${absolute_record_path}/Log"
mkdir -p ${absolute_log_path}

# Image path
absolute_record_pimage=${absolute_record_path}"/Image"
mkdir -p ${absolute_record_pimage}

# IMU path
absolute_record_pimu=${absolute_record_path}"/IMU"
mkdir -p ${absolute_record_pimu}

# Lidar path
absolute_record_plidar=${absolute_record_path}"/Lidar"
mkdir -p ${absolute_record_plidar}

# Plan layer path
absolute_task_path=${absolute_record_path}"/Task"
mkdir -p ${absolute_task_path}
copy_planlayer_dir_to_task

# add record path to launch file
absolute_velodyne_launch="${absolute_catkin_path}/src/velodyne/velodyne_driver/launch/nodelet_manager.launch"
chmod +x ${absolute_script_path}"/parse_xml.py"
${absolute_script_path}"/parse_xml.py" ${absolute_record_path} ${absolute_velodyne_launch}
log_info "Add record path ${absolute_record_path} to launch file: ${absolute_velodyne_launch}."

# redirect glog path to ./Log, default glog path is /tmp
# use export, if not: roslaunch cannot get GLOG_log_dir, only rosrun can
export GLOG_log_dir="${absolute_log_path}"

# start Smart Collector
cd ${absolute_catkin_path}

pgrep roscore >/dev/null 2>&1
if [ $? -ne 0 ]; then
    roscore &
    sleep 3
fi

rosrun SmartCollector SmartCollector &
sleep 0.5
rosrun ntd_info_process ntd_info_process_node ${absolute_record_pimu}"/event.txt" &
sleep 0.5
rosrun hdop_teller hdop_teller_node ${absolute_record_pimu}/ &
sleep 0.5
rosrun roscameragpsimg roscameragpsimg jpg ${absolute_record_pimage}/ ${absolute_record_pimu}/ &
sleep 0.5
rosrun display_had display_had_node ${absolute_record_pimu}/ &
sleep 0.5
rviz &
sleep 0.5
roslaunch velodyne_pointcloud VLP16_points.launch
