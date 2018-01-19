#include "mif_read.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

MifReader::MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh, const string& _imuRecordPath) {
    mMifPath = _imuRecordPath + "/../../../../data/";
    LOG(INFO) << "Mif data is: " << mMifPath;
    const string planLayerPath(_imuRecordPath + "/../Task/Planlayer/");

    mLineArray.markers.reserve(2000);
    mSubGps = nh.subscribe("processed_infor_msg", 0, &MifReader::gpsCallback, this);
    mPubArray = nh.advertise<visualization_msgs::MarkerArray>("mif_array", 0);

    mIsInSameMesh = false;
    mCurrentWGS.x = -0.5;

    // e.g., _imuRecordPath: /home/navi/catkin_ws/record/1005-1-077-180110/rawdata/IMU/
    string encrypTrackFileName("");
    (void)public_tools::PublicTools::generateFileName(_imuRecordPath, encrypTrackFileName);
    const string trackfileInImuPath(_imuRecordPath + encrypTrackFileName + "_encryp_track.txt");
    mTrackMarsFile.open(trackfileInImuPath);
    if(!mTrackMarsFile.is_open() ) {
        LOG(ERROR) << "Failed to open " << trackfileInImuPath;
        exit(1);
    }
    LOG(INFO) << "Logging MARS track in " << trackfileInImuPath;
    mTrackMarsFile << std::fixed;

    // Below for display GPS track
    mpTrackDisplayer = new TrackDisplayer();
    // for display plan layer
    pPlanLayerDisplayer_ = new PlanLayerDisplayer(planLayerPath);
}

MifReader::~MifReader() {
    mTrackMarsFile.close();
    delete mpTrackDisplayer;
    delete pPlanLayerDisplayer_;
    LOG(INFO) << "Goodbye.";
}

void MifReader::run() {
    vector<string> roadDBs;
    (void)public_tools::PublicTools::getFilesInDir(mMifPath, "road_15.db", roadDBs);
    if(1 != roadDBs.size() ) {
        LOG(ERROR) << "Got " << roadDBs.size() << " roadDBs, should be 1.";
        exit(1);
    }
    string roadDB(roadDBs[0]);

    ros::Rate rate(2);
    while(ros::ok() ) {
        ros::spinOnce();
        rate.sleep();

        if(mCurrentWGS.x < 0) {
            LOG(INFO) << "GPS not fixed, mCurrentWGS.x: " << mCurrentWGS.x;
            continue;
        }
        mpTrackDisplayer->displayTrack(mCurrentWGS);
        pPlanLayerDisplayer_->displayPlanLayer(mCurrentWGS);

        if(mTileList.empty()) {
            LOG(INFO) << "mTileList is empty.";
            continue;
        }

        // fill gAbsGaussList using road.db and current WGS
        (void)readFile(roadDB);

        DLOG(INFO) << "gAbsLines contains " << gAbsLines.size() << " lines.";
        if(gAbsLines.empty()) {
            LOG(INFO) << "Found no lines in sqlite.";
            continue;
        }

        mLineArray.markers.clear();
        // mLineStrip coordination transfer to current position
        for(size_t markerId = 0; markerId < gAbsLines.size(); ++markerId) {
            visualization_msgs::Marker offsetLine;
            initMarker(offsetLine, markerId);
            public_tools::PublicTools::transform_coordinate(gAbsLines[markerId], mCurrentWGS, offsetLine.points);
            mLineArray.markers.push_back(offsetLine);
        }
        DLOG(INFO) << "I got " << mLineArray.markers.size() << " lines to show.";
        mPubArray.publish(mLineArray);
    }
}

void MifReader::initMarker(visualization_msgs::Marker &marker, const size_t id) {
    marker.points.clear();
    marker.header.frame_id = "/velodyne";
    marker.ns = "sd_map";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width; POINTS markers use x and y scale for width/height respectively
    marker.scale.x = .5;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    // set .a = 0 to hide display
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1.0);
}

void MifReader::gpsCallback(const ntd_info_process::processed_infor_msg::ConstPtr& pGpsMsg) {
    // lat: 1 degree is about 100000 m
    const double lat = pGpsMsg->latlonhei.lat;
    // lon: 1 degree is about 100000 m
    const double lng = pGpsMsg->latlonhei.lon;
    DLOG(INFO) << __FUNCTION__ << " start, lat: " << lat;
    if(lng < 115) {
        LOG(INFO) << "Wrong coordination (" << lng << ", " << lat << ").";
        return;
    }

    double newlng = 0;
    double newlat = 0;
    if(0 != coordtrans("wgs84", "gcj02", lng, lat, newlng, newlat) ) {
        LOG(ERROR) << "Failed translate coordination (" << lng << ", " << lat << ") to Mars.";
        exit(1);
    }
    DLOG(INFO) << "Origin coord (" << lng << ", " << lat << ");";
    DLOG(INFO) << "Mars coord (" << newlng << ", " << newlat << ").";

    // TODO: Refactor using sqlite
    mTrackMarsFile << newlng << ", " << newlat << "\n";

    double currentGaussX;
    double currentGaussY;
    public_tools::PublicTools::GeoToGauss(newlng * 3600, newlat * 3600, 39, 3, &currentGaussY, &currentGaussX, 117);
    mCurrentWGS.x = currentGaussY;
    mCurrentWGS.y = currentGaussX;
    mCurrentWGS.z = pGpsMsg->latlonhei.hei;

    const int MESH_LEVEL = 15;
    static long mesh_last = -1;
    const long mesh = MortonCodec::getMortonFromLonLat(newlng, newlat, MESH_LEVEL);

    DLOG(INFO) << "Current morton: " << mesh;
    if(mesh == mesh_last) {
        DLOG(INFO) << "Same morton, no need get tile list again.";
        return;
    }
    mIsInSameMesh = false;
    mesh_last = mesh;

    mTileList = MortonCodec::getTileNumList(mesh, MESH_LEVEL);

    for(auto tile: mTileList) {
        DLOG(INFO) << "tile: " << tile;
    }
    DLOG(INFO) << "mTileList Size: " << mTileList.size();
}

static int querySqlCB(void *itemCnt, int argc, char **argv, char **colNames) {
    DLOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";
    ++(*((size_t *)itemCnt));

    if(argc < 1) {
        LOG(ERROR) << "Failed to query SQL.";
        exit(1);
    }

    // parse GEOM text string, e.g., LINESTRING (116.33345 39.97807, 116.33343 39.97744)
    const string geomStr(argv[0]);
    vector<string> latLonlist;
    boost::split(latLonlist, geomStr, boost::is_any_of( ",)(" ), boost::token_compress_on);
    // if contain only 1 point
    if(latLonlist.size() - 2 < 2) {
        LOG(WARNING) << geomStr << " contains no lines.";
        return 0;
    }

    public_tools::geoPoints_t mifLine;
    mifLine.clear();
    for(int i = 1; i < latLonlist.size() - 1; ++i) {
        std::istringstream iss(latLonlist[i]);
        double lat;
        double lon;
        iss >> lon >> lat;
        DLOG_EVERY_N(INFO, 100) << lon << " : " << lat;

        double gauss_x = 0;
        double gauss_y = 0;
        public_tools::PublicTools::GeoToGauss(lon * 3600, lat * 3600, 39, 3, &gauss_y, &gauss_x, 117);
        geometry_msgs::Point gaussXY;
        gaussXY.y = gauss_x;
        gaussXY.x = gauss_y;
        mifLine.push_back(gaussXY);
    }
    gAbsLines.push_back(mifLine);

    return 0;
}

void MifReader::readFile(const string &file) {
    DLOG(INFO) << __FUNCTION__ << " start, param: " << file;
    if(mIsInSameMesh) {
        LOG(INFO) << "In same mesh area, no need update gAbsGaussList.";
        return;
    }
    mIsInSameMesh = true;

    gAbsLines.clear();

    sqlite3 *db;
    if(sqlite3_open(file.c_str(), &db)) {
        LOG(ERROR) << "Failed to open " << file;
        exit(1);
    }

    string sql("");
    int errNo;
    char *errMsg = 0;

    for(auto tile: mTileList) {
        sql += " or MESH = " + std::to_string(tile);
    }
    // substring to ignore " or " ahead
    sql = "SELECT GEOM FROM ROAD WHERE " + sql.substr(4);
    LOG(INFO) << "Query SQL: " << sql;
    size_t queryCnt = 0;
    errNo = sqlite3_exec(db, sql.c_str(), &querySqlCB, (void *)&queryCnt, &errMsg);
    if(SQLITE_OK != errNo) {
        LOG(ERROR) << "SQL error: " << errMsg;
        sqlite3_free(errMsg);
        exit(1);
    }
    LOG(INFO) << "Got " << queryCnt << " items.";

    sqlite3_close(db);
}
