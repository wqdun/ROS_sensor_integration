# below from RS232
float64 GPStime
Point3D lat_lon_hei
Point3D pitch_roll_heading
float64 speed
int32 nsv1_num
int32 nsv2_num

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
