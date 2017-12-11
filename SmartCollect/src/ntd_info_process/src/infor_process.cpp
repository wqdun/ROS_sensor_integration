#include "infor_process.h"
// simulate car moving
// #define SIMULATION

InforProcess::InforProcess() {
    mSub = nh.subscribe("imu_string", 0, &InforProcess::gpsCB, this);
    mSubVelodyne = nh.subscribe("velodyne_pps_status", 0, &InforProcess::velodyneCB, this);
    mSub422 = nh.subscribe("imu422_hdop", 0, &InforProcess::rawImuCB, this);
    mSubCameraImg = nh.subscribe("cam_speed", 0, &InforProcess::cameraImgCB, this);

    mPub = nh.advertise<ntd_info_process::processed_infor_msg>("processed_infor_msg", 0);
    mIsVelodyneUpdated = mIsRawImuUpdated = mIsGpsUpdated = false;

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

        if(!mIsGpsUpdated) {
            mOutMsg.GPStime = mOutMsg.latlonhei.lat = mOutMsg.latlonhei.lon = mOutMsg.latlonhei.hei = mOutMsg.current_pitch = mOutMsg.current_roll = mOutMsg.current_heading = mOutMsg.current_speed = -2.;
            mOutMsg.nsv1_num = mOutMsg.nsv2_num = -2;
        }
        mIsGpsUpdated = false;

        // for rawImuCB is 1Hz, my freq should <= 1Hz: 0.5Hz
        if(0 == (freqDivider % 16) ) {
            if(!mIsRawImuUpdated) {
                // -1: hdop not updated
                mOutMsg.hdop = mOutMsg.latitude = mOutMsg.longitude = mOutMsg.noSV_422 = -2;
            }
            mIsRawImuUpdated = false;
        }

        // 8Hz
        if(!mIsVelodyneUpdated) {
            mOutMsg.pps_status = mOutMsg.is_gprmc_valid = "No GPRMC received";
        }
        mIsVelodyneUpdated = false;

        mPub.publish(mOutMsg);
    }
}

void InforProcess::cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    mOutMsg.camera_fps = pCameraImgMsg->data;
}

void InforProcess::velodyneCB(const std_msgs::String::ConstPtr& pVelodyneMsg) {
    // 10Hz
    mIsVelodyneUpdated = true;
    if(pVelodyneMsg->data.empty() ) {
        LOG(ERROR) << "Got no GPRMC from LIDAR.";
        exit(1);
    }

    vector<string> pps_gprmc;
    boost::split(pps_gprmc, pVelodyneMsg->data, boost::is_any_of(",") );
    if(pps_gprmc.size() < 2) {
        LOG(ERROR) << "Wrong pps_gprmc: " << pVelodyneMsg->data;
        exit(1);
    }

    const int status_index = public_tools::PublicTools::string2int(pps_gprmc[0]);
    if(status_index > 3) {
        LOG(ERROR) << "Invalid PPS status: " << pps_gprmc[0];
        exit(1);
    }

    mOutMsg.pps_status = PPS_STATUS[status_index];
    // A validity - A-ok, V-invalid, refer VLP-16 manual
    mOutMsg.is_gprmc_valid = pps_gprmc[1];
}

void InforProcess::rawImuCB(const hdop_teller::imu5651_422::ConstPtr& pRawImuMsg) {
    // $GPGGA sentence is set 1Hz
    mIsRawImuUpdated = true;

    // in case: $GPGGA,,,,,,0,,,,,,,,*66
    mOutMsg.latitude = (pRawImuMsg->Latitude.empty() )? -1: (public_tools::PublicTools::string2double(pRawImuMsg->Latitude) );
    mOutMsg.longitude = (pRawImuMsg->Longitude.empty() )? -1: (public_tools::PublicTools::string2double(pRawImuMsg->Longitude) );
    mOutMsg.hdop = (pRawImuMsg->Hdop.empty() )? -1: (public_tools::PublicTools::string2double(pRawImuMsg->Hdop) );
    mOutMsg.noSV_422 = (pRawImuMsg->NoSV.empty() )? -1: (public_tools::PublicTools::string2int(pRawImuMsg->NoSV) );
}

void InforProcess::gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg) {
#ifndef SIMULATION
    mIsGpsUpdated = true;
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

    int nsv1_num = public_tools::PublicTools::string2int(pGPSmsg->NSV1_num);
    int nsv2_num = public_tools::PublicTools::string2int(pGPSmsg->NSV2_num);

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

