# set -x
dir_old=$(pwd)
absolute_script_path=$(cd $(dirname $0) && pwd)

if [ ! -f "devel/setup.bash" ]; then
    echo "[ERROR] ${dir_old} is not a valid project path."
    exit -1
fi
. devel/setup.bash

# new build a record directory
record_path=$(date +%H_%M_%S)
absolute_record_path="${absolute_script_path}/record/${record_path}"
mkdir -p ${absolute_record_path}

# add record path to launch file
tools/parseXml.py ${absolute_record_path}

roslaunch out.xml

