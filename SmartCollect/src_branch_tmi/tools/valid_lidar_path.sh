#!/bin/sh

for lidar_file in ../../record/project_2017_11_24_1*
do
    valid_file=$lidar_file/Lidar/*.lidar
    echo "Check $valid_file"
    ./a.out $valid_file
done