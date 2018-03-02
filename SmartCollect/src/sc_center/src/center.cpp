#include "center.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>
// simulate car moving
// #define SIMULATION

InforProcess::InforProcess() {
    mSub232 = nh.subscribe("imu_string", 0, &InforProcess::gpsCB, this);
    mSubVelodyne = nh.subscribe("velodyne_pps_status", 0, &InforProcess::velodyneCB, this);
    mSub422 = nh.subscribe("imu422_hdop", 0, &InforProcess::rawImuCB, this);
    mSubCameraImg = nh.subscribe("cam_speed", 0, &InforProcess::cameraImgCB, this);
    mSubMyViz = nh.subscribe("msg_save_control", 0, &InforProcess::myVizCB, this);
    subServer_ = nh.subscribe("sc_server2center", 0, &InforProcess::serverCB, this);

    mPub = nh.advertise<sc_center::centerMsg>("processed_infor_msg", 0);
    pubTime2Local_ = nh.advertise<sc_center::imuPoints>("imu_time2local", 0);
    mPubIsSaveFile = nh.advertise<std_msgs::Int64>("center_msg_save_control", 0);

    mGpsTime[0] = mGpsTime[1] = -1;
    mIsVelodyneUpdated = mIsRawImuUpdated = mIsGpsUpdated = false;
    time2LocalMsg_.imu_points.clear();

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
        mOutMsg.latlonhei.x += 0.00002;
        mOutMsg.latlonhei.y -= 0.00002;
#else
        if(!mIsGpsUpdated) {
            mOutMsg.GPStime = mOutMsg.latlonhei.x = mOutMsg.latlonhei.y = mOutMsg.latlonhei.z = mOutMsg.current_pitch = mOutMsg.current_roll = mOutMsg.current_heading = mOutMsg.current_speed = -2.;
            mOutMsg.nsv1_num = mOutMsg.nsv2_num = -2;
        }
        mIsGpsUpdated = false;
#endif
        // 0.5Hz
        if(0 == (freqDivider % 16) ) {
            // for rawImuCB is 1Hz
            if(!mIsRawImuUpdated) {
                // -1: hdop not updated
                mOutMsg.hdop = mOutMsg.latitude = mOutMsg.longitude = mOutMsg.noSV_422 = -2;
            }
            mIsRawImuUpdated = false;
        }

        // 8Hz
        if(!mIsVelodyneUpdated) {
            mOutMsg.pps_status = mOutMsg.is_gprmc_valid = "Signal Lost";
        }
        mIsVelodyneUpdated = false;

        mPub.publish(mOutMsg);
    }
}

void InforProcess::serverCB(const sc_server_daemon::serverMsg::ConstPtr &pServerMsg) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    mOutMsg.is_project_already_exist = pServerMsg->is_project_already_exist;
}

void InforProcess::cameraImgCB(const std_msgs::Float64::ConstPtr& pCameraImgMsg) {
    mOutMsg.camera_fps = pCameraImgMsg->data;
}

void InforProcess::myVizCB(const std_msgs::Int64::ConstPtr& pMyVizMsg) {
    // only publish isSaveFile
    mPubIsSaveFile.publish(pMyVizMsg);
}

void InforProcess::velodyneCB(const velodyne_msgs::Velodyne2Center::ConstPtr& pVelodyneMsg) {
    // 10Hz
    mIsVelodyneUpdated = true;

    if( (pVelodyneMsg->pps_status_index) > 3) {
        LOG(ERROR) << "Invalid PPS status: " << pVelodyneMsg->pps_status_index;
        exit(1);
    }

    mOutMsg.pps_status = PPS_STATUS[pVelodyneMsg->pps_status_index];
    // A validity - A-ok, V-invalid, refer VLP-16 manual
    mOutMsg.is_gprmc_valid = pVelodyneMsg->is_gprmc_valid;
}

void InforProcess::rawImuCB(const sc_integrate_imu_recorder::scIntegrateImu::ConstPtr& pRawImuMsg) {
    // $GPGGA sentence is set 1Hz
    mIsRawImuUpdated = true;

    // in case: $GPGGA,,,,,,0,,,,,,,,*66
    mOutMsg.latitude = public_tools::PublicTools::string2num(pRawImuMsg->Latitude, -1.0);
    mOutMsg.longitude = public_tools::PublicTools::string2num(pRawImuMsg->Longitude, -1.0);
    mOutMsg.hdop = public_tools::PublicTools::string2num(pRawImuMsg->Hdop, -1.0);
    mOutMsg.noSV_422 = public_tools::PublicTools::string2num(pRawImuMsg->NoSV, -1);
}

void InforProcess::gpsCB(const roscameragpsimg::imu5651::ConstPtr& pGPSmsg) {
#ifndef SIMULATION
    mGpsTime[0] = mGpsTime[1];
    mGpsTime[1] = public_tools::PublicTools::string2num(pGPSmsg->GPSTime, -1.0);
    // do nothing if receive same frame
    if(mGpsTime[0] == mGpsTime[1]) {
        return;
    }
    mIsGpsUpdated = true;

    double gpsTime = mGpsTime[1];
    // lat: 1 degree is about 100000 m
    double lat = public_tools::PublicTools::string2num(pGPSmsg->Latitude, -1.0);
    // lon: 1 degree is about 100000 m
    double lon = public_tools::PublicTools::string2num(pGPSmsg->Longitude, -1.0);
    double hei = public_tools::PublicTools::string2num(pGPSmsg->Altitude, -1.0);

    double pitch = public_tools::PublicTools::string2num(pGPSmsg->Pitch, -1.0);
    double roll = public_tools::PublicTools::string2num(pGPSmsg->Roll, -1.0);
    double heading = public_tools::PublicTools::string2num(pGPSmsg->Heading, -1.0);

    double vAbs = -1.;
    if(pGPSmsg->Vel_east.empty() || pGPSmsg->Vel_north.empty() || pGPSmsg->Vel_up.empty() ) {
        // vAbs = -1: invalid
    }
    else {
        double vEast = public_tools::PublicTools::string2num(pGPSmsg->Vel_east, 0.0);
        double vNorth = public_tools::PublicTools::string2num(pGPSmsg->Vel_north, 0.0);
        double vUp = public_tools::PublicTools::string2num(pGPSmsg->Vel_up, 0.0);
        vAbs = sqrt(vEast * vEast + vNorth * vNorth + vUp * vUp);
    }

    int nsv1_num = public_tools::PublicTools::string2num(pGPSmsg->NSV1_num, -1);
    int nsv2_num = public_tools::PublicTools::string2num(pGPSmsg->NSV2_num, -1);

    mOutMsg.GPStime = gpsTime;

    sc_center::point3D p;
    p.x = lat;
    p.y = lon;
    p.z = hei;
    mOutMsg.latlonhei = p;
    DLOG(INFO) << "lat: " << lat;

    mOutMsg.current_pitch = pitch;
    mOutMsg.current_roll = roll;
    mOutMsg.current_heading = heading;
    mOutMsg.current_speed = vAbs;
    mOutMsg.nsv1_num = nsv1_num;
    mOutMsg.nsv2_num = nsv2_num;

#endif
}
