# set -x
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
sudo ifconfig eth0 mtu 9000

# new build a record directory
record_path=project_$(date +%Y_%m_%d_%H_%M_%S)
absolute_record_path="${absolute_script_path}/record/${record_path}"

mkdir -p ${absolute_record_path}

# image path
absolute_record_pimage="${absolute_script_path}/record/${record_path}/Image"
mkdir -p ${absolute_record_pimage}

# imu  path
absolute_record_pimu="${absolute_script_path}/record/${record_path}/IMU"
mkdir -p ${absolute_record_pimu}

# lidar path
absolute_record_plidar="${absolute_script_path}/record/${record_path}/Lidar"
mkdir -p ${absolute_record_plidar}
# output path for parsed Lidar data: xiaobo
absolute_record_plidar_parsed="${absolute_script_path}/record/${record_path}/Lidar_Parsed"
mkdir -p ${absolute_record_plidar_parsed}

# add record path to launch file
absolute_record_ptcloud="${absolute_script_path}/src/velodyne/velodyne_driver/launch/nodelet_manager.launch"
absolute_record_sensor="${absolute_script_path}/src/launch/ntd_sensors.launch"

echo $absolute_record_ptcloud
echo $absolute_record_sensor
# exit

chmod +x src/tools/parseXml.py
src/tools/parseXml.py ${absolute_record_path} ${absolute_record_sensor} ${absolute_record_ptcloud}

# redirect glog path to ./log, default glog path is /tmp
absolute_log_path="${absolute_record_path}/log"
mkdir -p ${absolute_log_path}
# use export, if not: roslaunch cannot get GLOG_log_dir, only rosrun can
export GLOG_log_dir="${absolute_log_path}"

roslaunch ./src/tools/out.xml

