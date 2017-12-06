#include "infor_process.h"
// simulate car moving
// #define SIMULATION

InforProcess::InforProcess() {
    mSub = nh.subscribe("imu_string", 0, &InforProcess::gpsCB, this);
    mSubVelodyne = nh.subscribe("velodyne_pps_status", 0, &InforProcess::velodyneCB, this);
    mSub422 = nh.subscribe("imu422_hdop", 0, &InforProcess::rawImuCB, this);
    mPub = nh.advertise<ntd_info_process::processed_infor_msg>("processed_infor_msg", 0);
    mIsVelodyneUpdated = mIsRawImuUpdated = false;

#ifdef SIMULATION
    mOutMsg.latlonhei.lat = 40.071975;
    mOutMsg.latlonhei.lon = 116.239563;
#endif
}

InforProcess::~InforProcess() {
}

void InforProcess::run() {
    ros::Rate rate(8);
    size_t freqDivider = 0;

    while(ros::ok()) {
        ++freqDivider;
        freqDivider %= 256;
        ros::spinOnce();
        rate.sleep();

#ifdef SIMULATION
        mOutMsg.latlonhei.lat += 0.00002;
        mOutMsg.latlonhei.lon -= 0.00002;
#endif

        // for rawImuCB is 1Hz, my freq should <= 1Hz: 0.5Hz
        if(0 == (freqDivider % 16) ) {
            if(!mIsRawImuUpdated) {
                // -1: hdop not updated
                mOutMsg.hdop = -1;
            }
            mIsRawImuUpdated = false;
        }

        if(!mIsVelodyneUpdated) {
            mOutMsg.pps_status = "Signal not Received";
        }
        mIsVelodyneUpdated = false;

        mPub.publish(mOutMsg);

    }
}

void InforProcess::velodyneCB(const std_msgs::String::ConstPtr& pVelodyneMsg) {
    // 10Hz
    mIsVelodyneUpdated = true;
    const uint8_t status_index = public_tools::PublicTools::string2uchar(pVelodyneMsg->data);
    mOutMsg.pps_status = (status_index <= 3)? PPS_STATUS[status_index]: ("Out range: " + pVelodyneMsg->data);
    if(status_index > 3) {
        LOG(ERROR) << "Invalid PPS status: " << status_index;
    }

}

void InforProcess::rawImuCB(const std_msgs::String::ConstPtr& pRawImuMsg) {
    // $GPGGA sentence is set 1Hz
    mIsRawImuUpdated = true;
    if("" == pRawImuMsg->data) {
        // in case: $GPGGA,,,,,,0,,,,,,,,*66
        mOutMsg.hdop = 0;
        return;
    }
    mOutMsg.hdop = public_tools::PublicTools::string2double(pRawImuMsg->data);
}

void InforProcess::gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg) {
#ifndef SIMULATION
    mGpsTime[0] = mGpsTime[1];
    mGpsTime[1] = public_tools::PublicTools::string2double(pGPSmsg->GPSTime);
    // do nothing if receive same frame
    if(mGpsTime[0] == mGpsTime[1]) {
        return;
    }

    double gpsTime = mGpsTime[1];
    // lat: 1 degree is about 100000 m
    double lat = public_tools::PublicTools::string2double(pGPSmsg->Latitude);
    // lon: 1 degree is about 100000 m
    double lon = public_tools::PublicTools::string2double(pGPSmsg->Longitude);
    double hei = public_tools::PublicTools::string2double(pGPSmsg->Altitude);

    double pitch = public_tools::PublicTools::string2double(pGPSmsg->Pitch);
    double roll = public_tools::PublicTools::string2double(pGPSmsg->Roll);
    double heading = public_tools::PublicTools::string2double(pGPSmsg->Heading);

    double vEast = public_tools::PublicTools::string2double(pGPSmsg->Vel_east);
    double vNorth = public_tools::PublicTools::string2double(pGPSmsg->Vel_north);
    double vUp = public_tools::PublicTools::string2double(pGPSmsg->Vel_up);
    double vAbs = sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp);

    uint8_t nsv1_num = public_tools::PublicTools::string2uchar(pGPSmsg->NSV1_num);
    uint8_t nsv2_num = public_tools::PublicTools::string2uchar(pGPSmsg->NSV2_num);

    mOutMsg.GPStime = gpsTime;

    ntd_info_process::point_wgs p;
    p.lat = lat;
    p.lon = lon;
    p.hei = hei;
    mOutMsg.latlonhei = p;
    DLOG(INFO) << "lat: " << mOutMsg.latlonhei.lat;

    mOutMsg.current_pitch = pitch;
    mOutMsg.current_roll = roll;
    mOutMsg.current_heading = heading;
    mOutMsg.current_speed = vAbs;
    mOutMsg.nsv1_num = nsv1_num;
    mOutMsg.nsv2_num = nsv2_num;
#endif
}

