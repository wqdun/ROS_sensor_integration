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
    // mesh size in current level
    double width = 180.0 / col;
    int rowNum = getDimensionCode(dLat, width, col);
    int colNum = getDimensionCode(dLon, width, row);
    morton = interleave(rowNum, colNum);
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
// Author     : Wuhan
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
    return (x < 0)? (nBlockNum - 1) + (int)(x / width): (int)(x / width);
}

/******************************************************************************
// Author     : Wuhan
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
// Author     : Wuhan
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
// Author     : Wuhan
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


static int querySqlCB(void *in_param, int argc, char **argv, char **colNames) {
    DLOG(INFO) << __FUNCTION__ << " start, param: " << (const char *)in_param;
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

    const string inParam = "Callback function called.";
    sql = "SELECT * from ROAD WHERE ID = 1000189";
    errNo = sqlite3_exec(db, sql.c_str(), &querySqlCB, (void *)inParam.c_str(), &errMsg);
    if(SQLITE_OK != errNo) {
        LOG(ERROR) << "SQL error: " << errMsg;
        sqlite3_free(errMsg);
        exit(1);
    }

    sqlite3_close(db);


    long mesh = getMortonFromLonLat(121, 43, 13);
    LOG(INFO) << mesh << "::Morton.";
    vector<int> anses = getTileNumList(mesh, 13);
    for(auto ans: anses) {
        LOG(INFO) << "ans:" << ans;
    }
    LOG(INFO) << "ans Size:" << anses.size();
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

