# below from RS232

float64 GPStime
point3D latlonhei
float64 current_pitch
float64 current_roll
float64 current_heading
float64 current_speed
int32 nsv1_num
int32 nsv2_num


# below from RS422

float64 latitude
float64 longitude
float64 hdop
int32 noSV_422


# below from velodyne

string pps_status
string is_gprmc_valid


# below from camera

float64 camera_fps


# below from server

int8 is_server_connected
int8 is_project_already_exist
