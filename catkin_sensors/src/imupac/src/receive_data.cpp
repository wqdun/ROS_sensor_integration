#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "imupac/imu5651.h"

using std::cout;
using std::endl;

void messageCallback(const imupac::imu5651::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    cout << "GPSWeek:" << msg->GPSWeek << endl;
    cout << "GPSTime:" << msg->GPSTime << endl;
    cout << "Heading:" << msg->Heading << endl;
    cout << "Pitch  :" << msg->Pitch << endl;
    cout << "Roll   :" << msg->Roll << endl;
    cout << "Lattitu:" << msg->Lattitude << endl;
    cout << "Longitu:" << msg->Longitude << endl;
    cout << "Altitud:" << msg->Altitude << endl;
    cout << "Vel_eas:" << msg->Vel_east << endl;
    cout << "Vel_nor:" << msg->Vel_north << endl;
    cout << "Vel_up :" << msg->Vel_up << endl;
    cout << "Baselin:" << msg->Baseline << endl;
    cout << "NSV1_nu:" << msg->NSV1_num << endl;
    cout << "NSV2_nu:" << msg->NSV2_num << endl;
    cout << "Status :" << msg->Status << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "receive");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("message", 1000, messageCallback);
    ros::spin();
    return 0;
}

