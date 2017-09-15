#include "ros/ros.h"
#include <sstream>
// #include "std_msgs/String.h"

#include <visualization_msgs/Marker.h>
#include "imupac/imu5651.h"

using std::string;
using std::istringstream;
using std::cout;
using std::endl;

static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP);

class gps_display {
public:
    gps_display() {
        gps_sub = nh.subscribe("message", 1000, &gps_display::gpsCallback, this);
        gps_pub = nh.advertise<visualization_msgs::Marker>("gps_lonlathei", 10);

        points.header.frame_id = "/my_frame";
        line_strip.header.frame_id = "/my_frame";

        points.header.stamp = ros::Time::now();
        line_strip.header.stamp = ros::Time::now();

        points.ns = "points_and_lines";
        line_strip.ns = "points_and_lines";

        points.action = visualization_msgs::Marker::ADD;
        line_strip.action = visualization_msgs::Marker::ADD;

        points.pose.orientation.w = 1.0;
        line_strip.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        // set .a = 0 to hide display
        points.color.a = 1.0;
        // Line strip is blue
        line_strip.color.b = 1.0f;
        line_strip.color.a = 1.0;

        gauss_x_origin = 0;
        gauss_y_origin = 0;
        is_origin_set = false;
    }

    void gpsCallback(const imupac::imu5651::ConstPtr& msg) {
        // lat: 1 degree is about 100000 m
        double lat = string2num(msg->Lattitude);
        // lon: 1 degree is about 100000 m
        double lon = string2num(msg->Longitude);
        double hei = string2num(msg->Altitude);

        // loss GPS signal
        if(lat < 0.1) {
            cout << "Loss GPS signal." << endl;
            return;
        }

        geometry_msgs::Point p;
        double gauss_x = 0;
        double gauss_y = 0;
        GeoToGauss(lon * 3600, lat * 3600, 39, 3, &gauss_y, &gauss_x, 117);
        // cout << std::fixed << gauss_x << endl;
        // cout << std::fixed << gauss_y << endl;

        // unit: m; when GPS fixed: set origin point; Beijing: 116.251917,40.078302
        if(!is_origin_set) {
            gauss_x_origin = gauss_x;
            gauss_y_origin = gauss_y;
            is_origin_set = true;
        }

        // 1 grid is 10 m
        p.x = (gauss_x - gauss_x_origin) / 10;
        p.y = (gauss_y - gauss_y_origin) / 10;
        cout << std::fixed << p.x << endl;
        cout << std::fixed << p.y << endl;
        // p.x = 2;
        // p.y = 1;

        points.points.push_back(p);
        line_strip.points.push_back(p);

        if(line_strip.points.size() > 1000) {
            points.points.clear();
            line_strip.points.clear();
        }
        cout << points.points.size() << endl;

        gps_pub.publish(points);
        gps_pub.publish(line_strip);
    }


private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Publisher gps_pub;

    visualization_msgs::Marker points;
    visualization_msgs::Marker line_strip;

    double gauss_x_origin;
    double gauss_y_origin;
    bool is_origin_set;

    // template <class T>
    // T string2num(const string& str) {
    //     istringstream iss(str);
    //     T num;
    //     iss >> num;
    //     return num;
    // }

    double string2num(const string& str) {
        istringstream iss(str);
        double num;
        iss >> num;
        return num;
    }
};


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

int main(int argc, char** argv) {
    ros::init(argc, argv, "display_gps_point");
    gps_display gps_displayer;
    ros::spin();

    return 0;
}





