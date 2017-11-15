#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include "../../public_tools/public_tools.h"
#include "ntd_info_process/processed_infor_msg.h"

using std::string;
using std::istringstream;
using std::vector;
using std::cout;
using std::endl;
using std::ios;
using std::ofstream;

//transform_coordinate
static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, vector<geometry_msgs::Point> &points_transformed);

class gps_display {
public:

    gps_display() {
        //subscribe of gps_msg
        gps_sub = nh.subscribe("processed_infor_msg", 1000, &gps_display::gpsCallback, this);
        //advertise of gps_lonlathei
        gps_pub = nh.advertise<visualization_msgs::Marker>("gps_lonlathei", 10);

        // min capacity is 1000, to optimize efficiency
        points.points.reserve(1000);
        //set points.header
        points.header.frame_id = "/velodyne";
        //set line_strip.header
        line_strip.header.frame_id = "/velodyne";
        //set arrow.header
        arrow.header.frame_id = "/velodyne";

        //set points.header.stamp
        points.header.stamp = ros::Time::now();
        //set line_strip.header.stamp
        line_strip.header.stamp = ros::Time::now();
        //set arrow.header.stamp
        arrow.header.stamp = ros::Time::now();

        //set points.ns
        points.ns = "points_and_lines";
        //set line_strip.ns
        line_strip.ns = "points_and_lines";
        //set arrow.ns
        arrow.ns = "points_and_lines";

        //set points-line_strip-arrow action
        points.action = visualization_msgs::Marker::ADD;
        line_strip.action = visualization_msgs::Marker::ADD;
        arrow.action = visualization_msgs::Marker::ADD;

        //set points-line_strip.pose
        points.pose.orientation.w = 1.0;
        line_strip.pose.orientation.w = 1.0;
        // arrow.pose.orientation.w = 1.0;

        //set points-line_strip-arrow id
        points.id = 0;
        line_strip.id = 1;
        arrow.id = 2;

        //set point-line_strip-arrow type
        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        arrow.type = visualization_msgs::Marker::ARROW;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.012;
        points.scale.y = 0.012;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.01;
        // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length.
        arrow.scale.x = 0.01;
        arrow.scale.y = 0.015;
        arrow.scale.z = 0.03;

        // Points are green
        points.color.g = 1.0f;
        // set .a = 0 to hide display
        points.color.a = 0.0;
        // Line strip is blue
        line_strip.color.r = 1.0f;
        line_strip.color.a = 1.0;
        // arrow is red
        arrow.color.g = 1.0f;
        arrow.color.a = 1.0;

        gauss_x_origin = 0;
        gauss_y_origin = 0;
        height_origin = 0;
        is_origin_set = false;
    }

    void gpsCallback(const ntd_info_process::processed_infor_msg::ConstPtr& msg) {
        // lat: 1 degree is about 100000 m
        double lat = msg->latlonhei.lat;
        // lon: 1 degree is about 100000 m
        double lon = msg->latlonhei.lon;
        double hei = msg->latlonhei.hei;

        geometry_msgs::Point p;
        double gauss_x = 0;
        double gauss_y = 0;
        public_tools::PublicTools::GeoToGauss(lon * 3600, lat * 3600, 39, 3, &gauss_y, &gauss_x, 117);

        // 1 grid is 100 m; gauss_x: North; gauss_y: East
        p.y = gauss_x / 10;// 100;
        p.x = gauss_y / 10;// 100;
        p.z = hei / 10;// 100;

        static vector<geometry_msgs::Point> gauss_coordinations;
        gauss_coordinations.push_back(p);
        int point_cnt = gauss_coordinations.size();
        if(point_cnt >= 1000) {
            gauss_coordinations.erase(gauss_coordinations.begin(), gauss_coordinations.begin() + point_cnt / 2);
        }
        ROS_INFO_STREAM_THROTTLE(10, "Now I have got " << point_cnt << " way points.");

        transform_coordinate(gauss_coordinations, points.points);
        if(point_cnt >= 2) {
            arrow.points.clear();
            arrow.points.push_back(points.points[point_cnt - 2]);
            arrow.points.push_back(points.points[point_cnt - 1]); // 0
            // arrow.points[0].z = 0;
            // pub arrow
            gps_pub.publish(arrow);
            // cout << arrow.points.size() << arrow.points[0].x << arrow.points[0].y << arrow.points[0].z << arrow.points[1].x << arrow.points[1].y << arrow.points[1].z << endl;
        }

        points.points.pop_back();
        line_strip.points = points.points;
        //pub points && line_strip
        gps_pub.publish(points);
        gps_pub.publish(line_strip);
    }


private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Publisher gps_pub;

    visualization_msgs::Marker points;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker arrow;

    double gauss_x_origin;
    double gauss_y_origin;
    double height_origin;
    bool is_origin_set;
};


static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, vector<geometry_msgs::Point> &points_transformed) {
    ROS_DEBUG("transform_coordinate start.");
    points_transformed.clear();
    geometry_msgs::Point temp_point;
    geometry_msgs::Point last_point_gauss = points_gauss.back();
    for(auto &point: points_gauss) {
        temp_point.x = point.x - last_point_gauss.x;
        temp_point.y = point.y - last_point_gauss.y;
        temp_point.z = point.z - last_point_gauss.z;
        points_transformed.push_back(temp_point);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "display_gps_point");
    gps_display gps_displayer;
    ros::spin();

    return 0;
}
