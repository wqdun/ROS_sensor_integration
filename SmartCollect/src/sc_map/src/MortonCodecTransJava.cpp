/******************************************************************************
// This class is migrated from Wuhan Team's MortonCodec.java
// Author     : 05586
// Date       : 2017/11/21
// Description: calculate mesh by longitude and latitude
******************************************************************************/
#include "MortonCodecTransJava.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

long MortonCodec::getMortonFromLonLat(const double dLon, const double dLat, const int level) {
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
int MortonCodec::getDimensionCode(double x, double width, int nBlockNum) {
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
long MortonCodec::interleave(int rowNum, int colNum) {
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

int MortonCodec::getBinaryLength(int in) {
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
int MortonCodec::getTileNum(const vector<int> &xy) {
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
vector<int> MortonCodec::getTileNumList(int id, int level) {
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


string MortonCodec::toBinaryString(int inNum) {
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
vector<int> MortonCodec::getXY(int id) {
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
