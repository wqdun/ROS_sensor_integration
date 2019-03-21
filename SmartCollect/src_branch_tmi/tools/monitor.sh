newest_rawdata=$(ls -t /opt/smartc/record/ | head -n1)
absolute_newest="/opt/smartc/record/${newest_rawdata}/Rawdata/"
echo "Watching ${absolute_newest}..."
watch -n1 -d "ls -l ${absolute_newest}/Lidar/lidar.dat; ls -l ${absolute_newest}/IMU; ls -l ${absolute_newest}/Image | wc -l;"
