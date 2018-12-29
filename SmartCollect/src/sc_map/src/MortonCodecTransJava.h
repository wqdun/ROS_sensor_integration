#ifndef __MORTON_CODEC_TRANS_JAVA_H
#define __MORTON_CODEC_TRANS_JAVA_H

#include <dirent.h>
#include <bitset>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <vector>
using std::vector;
#include <string>
using std::string;


class MortonCodec {

public:
    static long getMortonFromLonLat(const double dLon, const double dLat, const int level);
    static vector<int> getTileNumList(int id, int level);


private:
    static int getDimensionCode(double x, double width, int nBlockNum);
    static long interleave(int rowNum, int colNum);
    static int getBinaryLength(int in);
    static int getTileNum(const vector<int> &xy);
    static string toBinaryString(int inNum);
    static vector<int> getXY(int id);

};




#endif // __MORTON_CODEC_TRANS_JAVA_H