#include "infor_process.h"
using std::vector;
using std::istringstream;

InforProcess::InforProcess() {
    mSub = nh.subscribe("gps_msg", 1000, &InforProcess::gpsCB, this);
    // mPub = nh.advertise<geometry_msgs::Point>("current_wgs84_msg", 1000);
    mPub = nh.advertise<ntd_info_process::processed_infor_msg>("processed_infor_msg", 1000);
}

InforProcess::~InforProcess() {
}

void InforProcess::gpsCB(const imupac::imu5651::ConstPtr& pGPSmsg) {
    // lat: 1 degree is about 100000 m
    double lat = string2num(pGPSmsg->Lattitude);
    // lon: 1 degree is about 100000 m
    double lon = string2num(pGPSmsg->Longitude);
    double hei = string2num(pGPSmsg->Altitude);

    double pitch = string2num(pGPSmsg->Pitch);
    double roll = string2num(pGPSmsg->Roll);
    double heading = string2num(pGPSmsg->Heading);
    double gpsTime = string2num(pGPSmsg->GPSTime);

    // loss GPS signal, abandon it
    if(lat < 0.1) {
        ROS_INFO_STREAM("Loss of GPS signal.");
        return;
    }

    double gauss_x = 0;
    double gauss_y = 0;
    GeoToGauss(lon * 3600, lat * 3600, 39, 3, &gauss_y, &gauss_x, 117);

    ntd_info_process::processed_infor_msg out_msg;
    // ntd_info_process::point_wgs p;
    geometry_msgs::Point p;
    // gauss_x: North; gauss_y: East
    p.x = gauss_y;
    p.y = gauss_x;
    p.z = hei;
    out_msg.current_wgs = p;

    out_msg.current_pitch = pitch;
    out_msg.current_roll = roll;
    out_msg.current_heading = heading;
    out_msg.GPStime = gpsTime;

    mPub.publish(out_msg);
}


static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP) {
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double jd_hd, wd_hd;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;     //  克拉索夫斯基椭球中子午弧长
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos, et3;   //  sinB,cosB
    double PI = 3.14159265358979;
    double b_e2 = 0.00669437999013;
    double b_c = 6378137;
    jd_hd = jd / 3600.0 * PI / 180.0;     // 将以秒为单位的经度转换成弧度
    wd_hd = wd / 3600.0 * PI / 180.0;    // 将以秒为单位的纬度转换成弧度
                                         // 如果不设中央经线（缺省参数: -1000），则计算中央经线，
                                         // 否则，使用传入的中央经线，不再使用带号和带宽参数
                                         //L = (DH - 0.5) * DH_width       // 计算中央经线的经度
    if (LP == -1000)
    {
        L = (DH - 0.5) * DH_width;
        // 计算中央经线的经度
    }
    else
    {
        L = LP;
    }
    l0 = jd / 3600.0 - L;       // 计算经差
    tsin = sin(wd_hd);        // 计算sinB
    tcos = cos(wd_hd);        // 计算cosB
                              // 计算克拉索夫斯基椭球中子午弧长X
                              //X = 111134.8611 / 3600.0 * wd - (32005.7799 * tsin + 133.9238 * pow(tsin, 3) + 0.6976 * pow(tsin, 5) + 0.0039 * pow(tsin, 7)) * tcos;
    X = 111132.9558 / 3600.0*wd - 16038.6496*sin(2 * wd_hd) + 16.8607*sin(4 * wd_hd) - 0.0220*sin(6 * wd_hd);
    et2 = b_e2 * pow(tcos, 2); //  et2 = (e' ** 2) * (cosB ** 2)
    et3 = b_e2 * pow(tsin, 2);
    N = b_c / sqrt(1 - et3);     //  N = C / sqrt(1 + et2)
    t = tan(wd_hd);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    *x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);

    *y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
}

static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, vector<geometry_msgs::Point> &points_transformed) {
    ROS_DEBUG("transform_coordinate start.");
    points_transformed.clear();
    geometry_msgs::Point temp_point;
    geometry_msgs::Point last_point_gauss = points_gauss.back();
    for(auto point: points_gauss) {
        temp_point.x = point.x - last_point_gauss.x;
        temp_point.y = point.y - last_point_gauss.y;
        temp_point.z = point.z - last_point_gauss.z;
        points_transformed.push_back(temp_point);
    }
}


// template <class T>
// T string2num(const string& str) {
//     istringstream iss(str);
//     T num;
//     iss >> num;
//     return num;
// }

static double string2num(const string& str) {
    istringstream iss(str);
    double num;
    iss >> num;
    return num;
}