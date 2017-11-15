#include "mif_read.h"

MifReader::MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    private_nh.param("mif_path", mMifPath, mMifPath);
    DLOG(INFO) << "Mif data is:" << mMifPath;

    mSubGps = nh.subscribe("processed_infor_msg", 1000, &MifReader::gpsCallback, this);
    mif_pub = nh.advertise<visualization_msgs::Marker>("mif_lonlathei", 10);
    mLineStrip.points.reserve(1000);
    mLineStrip.header.frame_id = "/velodyne";

    mLineStrip.header.stamp = ros::Time::now();
    mLineStrip.ns = "points_and_lines";
    mLineStrip.action = visualization_msgs::Marker::ADD;
    mLineStrip.pose.orientation.w = 1.0;
    mLineStrip.id = 0;
    mLineStrip.type = visualization_msgs::Marker::LINE_STRIP;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    mLineStrip.scale.x = 0.01;
    // Line strip is blue
    // set .a = 0 to hide display
    mLineStrip.color.r = 1.0f;
    mLineStrip.color.a = 1.0;

}

void MifReader::run() {
    vector<string> roadDBs;
    (void)getFiles(mMifPath, roadDBs);
    DLOG(INFO) << "Got " << roadDBs.size() << " road DB files.";
    for(auto &roadDB: roadDBs) {
        readFile(roadDB);
    }

    LOG(INFO) << "I've got " << mLineStrip.points.size() << " points.";

    ros::Rate rate(1);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        // BELOW 4 test
        mLineStrip.points.clear();
        geometry_msgs::Point tempPoint;
        tempPoint.x = tempPoint.y = tempPoint.z = 0;
        mLineStrip.points.push_back(tempPoint);
        tempPoint.x = tempPoint.y = tempPoint.z = 1;
        mLineStrip.points.push_back(tempPoint);
        mif_pub.publish(mLineStrip);
        // END 4 test
    }
}

void MifReader::gpsCallback(const ntd_info_process::processed_infor_msg::ConstPtr& pGpsMsg) {
    mNowLocation.x = pGpsMsg->latlonhei.lat;
    mNowLocation.y = pGpsMsg->latlonhei.lon;
    mNowLocation.z = pGpsMsg->latlonhei.hei;
}

void MifReader::getFiles(const string &basePath, vector<string> &files) {
    DIR *dir;
    dirent *ptr;

    // if dir == NULL
    if( !(dir = opendir(basePath.c_str())) ) {
        LOG(ERROR) << "Failed to open " << basePath;
        exit(1);
    }

    // while ptr != NULL
    while(ptr = readdir(dir)) {
        // ignore . and ..
        if( !(strcmp(ptr->d_name, ".")) ||
            !(strcmp(ptr->d_name, "..")) ) {
            continue;
        }

        // regular file
        if(8 == ptr->d_type) {
            // only deal ROAD_LANE_MARKING_GEO mif & mid
            const string fileName(ptr->d_name);
            if(string::npos != fileName.find("road.db")) {
                files.push_back(basePath + "/" + fileName);
            }
            // else do nothing
            // else {}
        }
        // directory
        else
        if(4 == ptr->d_type) {
            const string subDir(basePath + "/" + ptr->d_name);
            getFiles(subDir, files);
        }
        // else {
        //     // ignore links(10) & others
        // }
    }

    closedir(dir);
    return;
}

static int queryCB(void *in_param, int argc, char **argv, char **colNames){
    LOG(INFO) << __FUNCTION__ << " start, param: " << (const char *)in_param;
    for(int i = 0; i < argc; ++i) {
        DLOG(INFO) << colNames[i] << ":" << argv[i]? argv[i] : "NULL";
    }
    return 0;
}

void MifReader::readFile(const string &file) {
    DLOG(INFO) << __FUNCTION__ << " start, param: " << file;
    sqlite3 *db;

    if(sqlite3_open(file.c_str(), &db)) {
        LOG(ERROR) << "Failed to open " << file;
        exit(1);
    }

    string sql;
    int errNo;
    char *errMsg = 0;

    // sql = "CREATE TABLE COMPANY(" \
    //             "ID INT PRIMARY KEY     NOT NULL," \
    //             "NAME           TEXT    NOT NULL," \
    //             "AGE            INT     NOT NULL," \
    //             "ADDRESS        CHAR(50)," \
    //             "SALARY         REAL );";
    // errNo = sqlite3_exec(db, sql.c_str(), queryCB, 0, &errMsg);
    // if(SQLITE_OK != errNo) {
    //     LOG(ERROR) << "SQL error: " << errMsg;
    //     sqlite3_free(errMsg);
    //     exit(1);
    // }

    const string inParam = "Callback function called.";
    sql = "SELECT * from ROAD WHERE ID = 1000189";
    errNo = sqlite3_exec(db, sql.c_str(), queryCB, (void *)inParam.c_str(), &errMsg);
    if(SQLITE_OK != errNo) {
        LOG(ERROR) << "SQL error: " << errMsg;
        sqlite3_free(errMsg);
        exit(1);
    }

    sqlite3_close(db);


    // std::ifstream inFile(file);
    // if(!inFile) {
    //     LOG(ERROR) << "Failed to open " << file;
    //     exit(1);
    // }

    // const size_t BUFFER_SIZE = 100;
    // char *pLine = new char[BUFFER_SIZE];
    // string line("");
    // std::stringstream ss;
    // size_t plineCnt;
    // while(inFile.getline(pLine, BUFFER_SIZE)) {
    //     line.assign(pLine);
    //     if(0 != line.find("Pline")) {
    //         DLOG(INFO) << "Not begin with Pline.";
    //         continue;
    //     }

    //     ss.str(line);
    //     string tmpStr;
    //     ss >> tmpStr >> tmpStr;
    //     plineCnt = public_tools::PublicTools::string2int(tmpStr);
    //     DLOG(INFO) << "This Pline contains " << plineCnt << " points.";
    //     for(size_t i = 0; i < plineCnt; ++i) {
    //         inFile.getline(pLine, BUFFER_SIZE);
    //         line.assign(pLine);
    //         geometry_msgs::Point point3d;
    //         (void)getLonLat(line, point3d);
    //         mLineStrip.points.push_back(point3d);
    //     }

    // }
    // delete[] pLine;

    // // auto closed when exit scope
    // // inFile.close();
}


void MifReader::getLonLat(const string &line, geometry_msgs::Point &point) {
    DLOG(INFO) << __FUNCTION__ << " start, param: " << line;
    double lon = 0;
    double lat = 0;
    std::stringstream ss(line);
    ss >> lon >> lat;
    DLOG(INFO) << std::setprecision(12) << "lon:" << lon << "; lat:" << lat;

    double gauss_x = 0;
    double gauss_y = 0;
    public_tools::PublicTools::GeoToGauss(lon * 3600, lat * 3600, 39, 3, &gauss_y, &gauss_x, 117);
    // gauss_x: North; gauss_y: East
    point.y = gauss_x;
    point.x = gauss_y;

    return;
}

