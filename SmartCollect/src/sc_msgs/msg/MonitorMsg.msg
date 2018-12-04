# below from RS232
float64 GPStime
Point3D lat_lon_hei
Point3D pitch_roll_heading
# speed in km/h
float64 speed
int32 nsv1_num
int32 nsv2_num
float32 hdop_novatel

# below from RS422
string latitude
string longitude
string hdop
string noSV_422

# below from velodyne
string pps_status
string is_gprmc_valid

# below from camera
float64 camera_fps

# below from data fixer
int32 total_file_num
int32 process_num

# below from web controller
ProjectInfoMsg project_info

# below from server daemon self
# e.g., 80% (80M / 100M)
string disk_usage
string[] projects

# below from project monitor
int32 img_num
# unit: MB
int32 lidar_size

# below node control parameters, received from web
int8 is_record
int8 cam_gain

# below for hardware check
# -1: unchecked
# -2: IP not set or cable not connected
# LIDAR
int8 pps_HWcheck
#  0: "No PPS"
#  1: "Synchronizing PPS"
#  2: "PPS locked"
#  3: "PPS Error"
int8 gprmc_HWcheck
#  'A'(65): OK
#  'V'(86): Unavailable
# IMU
int8 imu_HWcheck
# CAMERA
int8 camera_HWcheck

# DISK USAGE CHECK
int32 disk_free_space_GB_HWcheck


