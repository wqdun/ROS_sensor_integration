#include "ros/ros.h"
#include "std_msgs/String.h"
#include "get_serial.h"
#include <string>
#include <iostream>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>

// ROS customized msg
#include "imupac/imu5651.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::ofstream;
using std::ios;

int saveFile(const string &str2write);

std::string frame2pub;

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_send");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<imupac::imu5651>("gps_msg", 1000);
    ros::Rate loop_rate(100);
    int fd = open_serial();
    // std_msgs::String msg;

    imupac::imu5651 msg;
    vector<string> parsed_data;

    while(ros::ok()) {
        if(read_serial(fd) < 0) {
            continue;
        }
        // what if 2 messages received
        if(frame2pub.empty()) {
            ROS_INFO("No complete frame received.");
            continue;
        }
        boost::split(parsed_data, frame2pub, boost::is_any_of( ",*" ), boost::token_compress_on);
        if(17 != parsed_data.size()) {
            ROS_INFO("Error frame, size is: %d.", (int)parsed_data.size());
            continue;
        }

        // save raw data to file
        if(0 != saveFile(frame2pub)) {
            ROS_WARN("Error!");
            exit;
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

int saveFile(const string &str2write) {
    static const int MAXLINE = 200000;
    static int lineCnt = 0;
    static char fileName[50];
    static ofstream outFile;
    // get unix time stamp as file name
    if(0 == lineCnt) {
        time_t tt = time(NULL);
        tm *t= localtime(&tt);
        (void)sprintf(fileName, "%02d_%02d_%02d.imu", t->tm_hour, t->tm_min, t->tm_sec);

        outFile.open(fileName, ios::app);
        if(!outFile) {
            ROS_WARN_STREAM("Create file:" << fileName << " failed.");
            return -1;
        }
        ROS_INFO_STREAM("Create file:" << fileName << " successfully.");
    }

    outFile << str2write << endl;
    ++lineCnt;
    lineCnt %= MAXLINE;

    if(0 == lineCnt) {
        outFile.close();
    }
    return 0;
}

