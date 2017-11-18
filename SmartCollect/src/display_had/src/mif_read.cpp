#include "mif_read.h"

MifReader::MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    private_nh.param("mif_path", mMifPath, mMifPath);
    DLOG(INFO) << "Mif data is:" << mMifPath;

    // mLineArray.reserve(5000);

    mSubGps = nh.subscribe("processed_infor_msg", 1, &MifReader::gpsCallback, this);
    mif_pub = nh.advertise<visualization_msgs::Marker>("mif_lonlathei", 10);
    mPubArray = nh.advertise<visualization_msgs::MarkerArray>("mif_array", 10);

    mIsInSameMesh = false;
}

void MifReader::run() {
    vector<string> roadDBs;
    (void)getFiles(mMifPath, roadDBs);
    DLOG(INFO) << "Got " << roadDBs.size() << " road DB files.";

    ros::Rate rate(1);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if(mTileList.empty()) {
            LOG(INFO) << "No receive GPS signal.";
            continue;
        }

        for(auto &roadDB: roadDBs) {
            // fill gAbsGaussList using road.db and current WGS
            readFile(roadDB);
        }

        DLOG(INFO) << "gAbsLines contains " << gAbsLines.size() << " lines.";
        if(gAbsLines.empty()) {
            continue;
        }

        // mLineStrip coordination transfer to current position
        for(size_t markerId = 0; markerId < gAbsLines.size(); ++markerId) {
            visualization_msgs::Marker offsetLine;
            initMarker(offsetLine, markerId);
            transform_coordinate(gAbsLines[markerId], mCurrentWGS, offsetLine.points);
            mLineArray.markers.push_back(offsetLine);
        }
        mPubArray.publish(mLineArray);
    }
}


void MifReader::initMarker(visualization_msgs::Marker &marker, const size_t id) {
    marker.points.clear();
    // marker.points.reserve(50);
    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sd_map";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.type = visualization_msgs::Marker::POINTS;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    // POINTS markers use x and y scale for width/height respectively
    marker.scale.x = 0.01;
    // marker.scale.y = 0.01;
    // Line strip is blue
    // set .a = 0 to hide display
    marker.color.r = 1.0f;
    marker.color.a = 1.0;
}


void MifReader::gpsCallback(const ntd_info_process::processed_infor_msg::ConstPtr& pGpsMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, lat: " << pGpsMsg->latlonhei.lat;
    double currentGaussX;
    double currentGaussY;

    public_tools::PublicTools::GeoToGauss(pGpsMsg->latlonhei.lon * 3600, pGpsMsg->latlonhei.lat * 3600, 39, 3, &currentGaussY, &currentGaussX, 117);
    mCurrentWGS.x = currentGaussY;
    mCurrentWGS.y = currentGaussX;
    mCurrentWGS.z = pGpsMsg->latlonhei.hei;

    static long mesh_last = -1;
    const long mesh = getMortonFromLonLat(pGpsMsg->latlonhei.lon, pGpsMsg->latlonhei.lat, 13);

    DLOG(INFO) << "Current morton: " << mesh;
    if(mesh == mesh_last) {
        DLOG(INFO) << "Same morton, no need get tile list again.";
        // mIsInSameMesh = true;
        return;
    }
    mIsInSameMesh = false;
    mesh_last = mesh;

    mTileList = getTileNumList(mesh, 13);

    for(auto tile: mTileList) {
        DLOG(INFO) << "tile: " << tile;
    }
    DLOG(INFO) << "mTileList Size: " << mTileList.size();
}

long MifReader::getMortonFromLonLat(const double dLon, const double dLat, const int level) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    const int LEVEL_MIN = 0;
    const int LEVEL_MAX = 26;
    if((level < LEVEL_MIN) || (level > LEVEL_MAX)) {
        LOG(ERROR) << "Wrong level: " << level;
        exit(1);
    }

    long morton = 0;
    // how many columns in current level
    int col = pow(2, level);
    // how many rows in current level
    int row = col;
    DLOG(INFO) << "row: " << row;
    // mesh size in current level
    double width = 180.0 / col;
    int rowNum = getDimensionCode(dLat, width, col);
    int colNum = getDimensionCode(dLon, width, row);
    morton = interleave(rowNum, colNum);
    DLOG(INFO) << "morton: " << morton;
    // 0 level highest bit
    // 0级最高位
    if(dLon < 0) {
        morton |= 1 << (2 * level);
    }
    if(dLat < 0) {
        morton |= 1 << (2 * level - 1);
    }
    return morton;
}

/******************************************************************************
// Author     : WanGuangyong & LiYandong
// Date       : *
// Description: calculate row/column by longitude and latitude
// Input      : x:
                    longitude or latitude
                width:
                    mesh size(unit: degree)
                nBlockNum:
                    number of meshes in current longitude/latitude
// Output     :
// Return     : row/column
// Others     : NA
******************************************************************************/
int MifReader::getDimensionCode(double x, double width, int nBlockNum) {
    DLOG(INFO) << __FUNCTION__ << " params: " << x << "; " << width << "; " << nBlockNum;
    return (x < 0)? (nBlockNum - 1) + (int)(x / width): (int)(x / width);
}

/******************************************************************************
// Author     : WanGuangyong & LiYandong
// Date       : *
// Description: interweave the code according to column values
                根据行列值交织编码
// Input      : dLon
                dLat
                level
// Output     : NA
// Return     : Morton code
// Others     : NA
******************************************************************************/
long MifReader::interleave(int rowNum, int colNum) {
    DLOG(INFO) << __FUNCTION__ << " start, rowNum: " << rowNum << "; colNum: " << colNum;
    long morton = 0;
    int s = getBinaryLength(rowNum);
    int e = getBinaryLength(colNum);
    int size = s > e ? s : e;
    for (int i = 0; i < size; ++i) {
        morton |= (colNum & (1 << i)) << i | (rowNum & (1 << i)) << (i + 1);
    }
    return morton;
}

int MifReader::getBinaryLength(int in) {
    for(int i = 0; i < 64; ++i) {
        if(!(in >> i)) {
            return i;
        }
    }

    LOG(ERROR) << "Number too big: " << in;
    exit(1);
}


/******************************************************************************
// Author     : WanGuangyong & LiYandong
// Date       : *
// Description: get grid ID by row/column number
                通过行列号获得格网ID
// Input      : row/column number
// Output     : NA
// Return     : grid ID
// Others     : NA
******************************************************************************/
int MifReader::getTileNum(const vector<int> &xy) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    string binaryStrX = toBinaryString(xy[0]);
    string binaryStrY = toBinaryString(xy[1]);
    string binaryStr("");
    for (int i = 0; i < binaryStrX.length() || i < binaryStrY.length(); ++i) {
        if (i >= binaryStrX.length()) {
            binaryStr.append("0");
        }
        else {
            binaryStr.append(binaryStrX.substr(binaryStrX.length() - i - 1, 1));
        }
        if (i >= binaryStrY.length()) {
            binaryStr.append("0");
        } else {
            binaryStr.append(binaryStrY.substr(binaryStrY.length() - i - 1, 1));
        }
    }
    (void)reverse(binaryStr.begin(), binaryStr.end());
    return stoi(binaryStr, nullptr, 2);
}

// get tile numbers(including itself)
vector<int> MifReader::getTileNumList(int id, int level) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    // get max row number
    // 获得最大行列号
    int maxY = pow(2, level);
    int maxX = pow(2, level + 1);

    vector<int> listTileNum;
    vector<int> res = getXY(id);
    for(int i = -1; i < 2; ++i) {
        for(int j = -1; j < 2; ++j) {
            if(res[0] + i >= 0 && res[1] + j >= 0 &&
               res[0] + i < maxX && res[1] + j < maxY) {
                const vector<int> rowColumn { res[0] + i, res[1] + j };
                listTileNum.push_back(getTileNum(rowColumn));
            }
        }
    }
    DLOG(INFO) << __FUNCTION__ << " end.";
    return listTileNum;
}


string MifReader::toBinaryString(int inNum) {
    const int BIT = (sizeof(inNum) << 3);
    std::bitset<BIT> bst(inNum);

    std::ostringstream oss;
    oss << bst;
    string binaryStringWith0Ahead(oss.str());
    return binaryStringWith0Ahead.substr(binaryStringWith0Ahead.find("1"));
}

/******************************************************************************
// Author     : WanGuangyong & LiYandong
// Date       : *
// Description: get row number by mesh ID
// Input      : mesh ID
// Output     : NA
// Return     :
// Others     : NA
******************************************************************************/
vector<int> MifReader::getXY(int id) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    string binaryStr = toBinaryString(id);
    int length = binaryStr.length();
    if(length % 2 != 0) {
        binaryStr = "0" + binaryStr;
    }
    // column
    string binaryXStr("");
    // row
    string binaryYStr("");
    DLOG(INFO) << "binaryStr: " << binaryStr;
    for(int i = 0; i < binaryStr.length(); i += 2) {
        binaryYStr.append(binaryStr.substr(i, 1));
        if(binaryStr.length() > i + 1) {
            binaryXStr.append(binaryStr.substr(i + 1, 1));
        }
    }
    DLOG(INFO) << binaryXStr << " : " << binaryYStr;

    const vector<int> xy {
        stoi(binaryXStr, nullptr, 2),
        stoi(binaryYStr, nullptr, 2)
    };
    DLOG(INFO) << __FUNCTION__ << " end.";
    return xy;
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

static int querySqlCB(void *itemCnt, int argc, char **argv, char **colNames) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    ++(*((size_t *)itemCnt));
    for(int i = 0; i < argc; ++i) {
        DLOG(INFO) << colNames[i] << ":" << argv[i]? argv[i] : "NULL";
    }

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

    MifLine_t mifLine;
    mifLine.clear();
    for(int i = 1; i < latLonlist.size() - 1; ++i) {
        std::istringstream iss(latLonlist[i]);
        double lat;
        double lon;
        iss >> lon >> lat;
        DLOG(INFO) << lon << " : " << lat;

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

static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, const geometry_msgs::Point &current_gauss, vector<geometry_msgs::Point> &points_transformed) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    points_transformed.clear();
    geometry_msgs::Point temp_point;
    for(auto &point: points_gauss) {
        // 100 m/grid
        temp_point.x = (point.x - current_gauss.x) * 0.01;
        temp_point.y = (point.y - current_gauss.y) * 0.01;
        // z always 0
        // temp_point.z = point.z - current_gauss.z;
        points_transformed.push_back(temp_point);
    }
}