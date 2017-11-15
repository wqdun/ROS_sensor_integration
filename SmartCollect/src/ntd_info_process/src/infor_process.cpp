#include "infor_process.h"

InforProcess::InforProcess() {
    mSub = nh.subscribe("imu_string", 1000, &InforProcess::gpsCB, this);
    // mPub = nh.advertise<geometry_msgs::Point>("current_wgs84_msg", 1000);
    mPub = nh.advertise<ntd_info_process::processed_infor_msg>("processed_infor_msg", 1000);
}

InforProcess::~InforProcess() {
}

void InforProcess::run() {
    ros::Rate rate(5);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        mPub.publish(mOutMsg);
    }
}

void InforProcess::gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg) {
    mGpsTime[0] = mGpsTime[1];
    mGpsTime[1] = public_tools::PublicTools::string2double(pGPSmsg->GPSTime);
    // do nothing if receive same frame
    if(mGpsTime[0] == mGpsTime[1]) {
        return;
    }

    // lat: 1 degree is about 100000 m
    double lat = public_tools::PublicTools::string2double(pGPSmsg->Lattitude);
    // loss GPS signal, abandon it
    if(lat < 0.1) {
        ROS_INFO_STREAM_THROTTLE(10, "Loss of GPS signal.");
        return;
    }

    // lon: 1 degree is about 100000 m
    double lon = public_tools::PublicTools::string2double(pGPSmsg->Longitude);
    double hei = public_tools::PublicTools::string2double(pGPSmsg->Altitude);
    double pitch = public_tools::PublicTools::string2double(pGPSmsg->Pitch);
    double roll = public_tools::PublicTools::string2double(pGPSmsg->Roll);
    double heading = public_tools::PublicTools::string2double(pGPSmsg->Heading);
    double gpsTime = public_tools::PublicTools::string2double(pGPSmsg->GPSTime);

    ntd_info_process::point_wgs p;
    // geometry_msgs::Point p;
    // gauss_x: North; gauss_y: East
    p.lat = lat;
    p.lon = lon;
    p.hei = hei;
    mOutMsg.latlonhei = p;

    mOutMsg.current_pitch = pitch;
    mOutMsg.current_roll = roll;
    mOutMsg.current_heading = heading;
    mOutMsg.GPStime = gpsTime;
}

