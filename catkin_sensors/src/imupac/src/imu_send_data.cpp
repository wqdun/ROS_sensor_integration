#include "ros/ros.h"
#include "std_msgs/String.h"
#include "get_serial.h"
#include <string>
#include <iostream>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

// ROS customized msg
#include "imupac/imu5651.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;

std::string frame2pub;
int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_send");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<imupac::imu5651>("message", 1000);
    ros::Rate loop_rate(100);
    int fd = open_serial();
    // std_msgs::String msg;

    imupac::imu5651 msg;
    vector<string> parsed_data;

    while(ros::ok()) {
        if(0 != read_serial(fd)) {
            continue;
        }
        // what if 2 messages received
        if('#' != frame2pub[frame2pub.size() - 1]) {
            cout << "Error frame." << endl;
            continue;
        }
        boost::split(parsed_data, frame2pub, boost::is_any_of( ",*" ), boost::token_compress_on);
        if(parsed_data.size() <= 16) {
            cout << "Error when parsing." << endl;
            continue;
        }
        msg.GPSWeek = parsed_data[1];
        msg.GPSTime = parsed_data[2];
        msg.Heading = parsed_data[3];
        msg.Pitch = parsed_data[4];
        msg.Roll = parsed_data[5];
        msg.Lattitude = parsed_data[6];
        msg.Longitude = parsed_data[7];
        msg.Altitude = parsed_data[8];
        msg.Vel_east = parsed_data[9];
        msg.Vel_north = parsed_data[10];
        msg.Vel_up = parsed_data[11];
        msg.Baseline = parsed_data[12];
        msg.NSV1_num = parsed_data[13];
        msg.NSV2_num = parsed_data[14];
        msg.Status = parsed_data[15];

        //TODO: BELOW test
        // 116.251917,40.078302
        // msg.Lattitude = "40.078302";
        // msg.Longitude = "116.251917";
        // msg.Altitude = "0";
        // ABOVE test

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(fd);
    return 0;
}


