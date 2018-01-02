cd
cd catkin_ws/
source devel/setup.bash
dir_old=$(pwd)
absolute_script_path=$(cd $(dirname $0) && pwd)

if [ ! -f "devel/setup.bash" ]; then
    echo "[ERROR] ${dir_old} is not a valid project path."
    exit -1
fi
source devel/setup.bash >/dev/null 2>&1

# auto input passwd
echo 123 | sudo -S su >/dev/null 2>&1
sudo chmod +r /dev/ttyS0
sudo chmod +r /dev/ttyUSB0
sudo ifconfig eth0 mtu 9000

# new build a project directory: e.g., record/1001-1-071-171227/rawdata
date_YMD=$(date +%Y%m%d)
record_path=$(./src/tools/task_config.py)"-"${date_YMD:2}
absolute_record_path=${absolute_script_path}/record/${record_path}"/rawdata"
mkdir -p ${absolute_record_path}

# Log path
absolute_log_path="${absolute_record_path}/Log"
mkdir -p ${absolute_log_path}
export log_file=${absolute_log_path}/$0".log"
# then we can use log_info and log_error
. src/tools/lib.sh

# Image path
absolute_record_pimage=${absolute_record_path}"/Image"
mkdir -p ${absolute_record_pimage}

# IMU path
absolute_record_pimu=${absolute_record_path}"/IMU"
mkdir -p ${absolute_record_pimu}

# Lidar path
absolute_record_plidar=${absolute_record_path}"/Lidar"
mkdir -p ${absolute_record_plidar}

# add record path to launch file
absolute_record_sensor="${absolute_script_path}/src/launch/ntd_sensors.launch"
absolute_record_ptcloud="${absolute_script_path}/src/velodyne/velodyne_driver/launch/nodelet_manager.launch"
chmod +x src/tools/parseXml.py
src/tools/parseXml.py ${absolute_record_path} ${absolute_record_sensor} ${absolute_record_ptcloud}
log_info "Add record path to launch file: "${absolute_record_ptcloud}.

# redirect glog path to ./log, default glog path is /tmp
# use export, if not: roslaunch cannot get GLOG_log_dir, only rosrun can
export GLOG_log_dir="${absolute_log_path}"
roscore &
rosrun ntd_info_process ntd_info_process_node &
rosrun hdop_teller hdop_teller_node ${absolute_record_pimu} &
rosrun roscameragpsimg roscameragpsimg jpg ${absolute_record_pimage}/ ${absolute_record_pimu}/ &
rosrun roscameragps_rviz roscameragps_rviz &
# roslaunch ./src/tools/start.xml
