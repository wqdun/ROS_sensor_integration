string host_name
float64 unix_time

# below from 5651 GPFPD
int32 GPSweek
float64 GPStime
Point3D lat_lon_hei
Point3D pitch_roll_heading
# speed in km/h
float64 speed
int32 status

# below from 5651 GPGGA
int32 no_sv
float64 hdop

float64 unix_time_minus_gps_time

# below from velodyne
string pps_status
string is_gprmc_valid
float64 velodyne_rpm

# below from camera
float64 camera_fps
float64 camera0_fps
float64 camera1_fps
float64 camera2_fps

int8 is_cameras_good

# below from data fixer
int32 total_file_num
int32 process_num

# below from web controller
ProjectInfoMsg project_info

# below from server daemon self
# e.g., 80% (80M / 100M)
string disk_usage
string[] projects

int8 is_disk_error

# below from project monitor
int32 img_num
# unit: MB
int32 lidar_size
float32 img_save_fps
uint64 raw_ins_size
uint64 timestamp_size

# below node control parameters, received from web
int8 is_record
int8 cam_gain

# below for hardware check
int32 sc_check_camera_num
int32 sc_check_imu_serial_port

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




