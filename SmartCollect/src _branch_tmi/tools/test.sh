#!/bin/bash

add_path_to_veledyne_launch() {
    echo "$FUNCNAME start, param: $*"
    local _absolute_record_path="/opt/"
    local absolute_velodyne_launch="${absolute_catkin_path}/src/velodyne/velodyne_driver/launch/nodelet_manager.launch"
    local parse_xml_script="${absolute_script_path}/parse_xml.py"
    chmod +x "${parse_xml_script}"
    "${parse_xml_script}" "/opt/smartc/src/velodyne/velodyne_driver/launch/nodelet_manager.launch" "$(arg manager)_driver" "record_path" "/opt/" "/opt/smartc/src/velodyne/velodyne_driver/launch/nodelet_manager.xml"

    echo "Add record path ${_absolute_record_path} to launch file: ${absolute_velodyne_launch}."
}


add_path_to_veledyne_launch

