#!/bin/bash

WORK_DIR="ROSDriver"
CUR_DIR=`echo $PWD | awk -F '\/' '{print $NF}'`

if [ $WORK_DIR == $CUR_DIR ] ;then
	cd ../
	catkin_make --source ROSDriver
fi

if [ $? != 0 ]; then 
	echo "Error occured when compiling !"	
	exit 1
fi

echo ""
echo "Setup Env ..."
echo "source ./devel/setup.bash"
source ./devel/setup.bash
if [ $? != 0 ]; then 
	echo "Setup Env Failed !"	
	exit 1
fi

echo ""
echo "Launch rfans driver ..."
echo "roslaunch rfans_driver node_manager.launch"
echo ""
roslaunch rfans_driver node_manager.launch


#rosrun rviz rviz 
#file-> open config -> RFans_Rviz_cfg.rviz (存放于代码的顶级目录中)

