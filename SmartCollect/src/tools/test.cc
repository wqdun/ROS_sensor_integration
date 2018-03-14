#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <cstdio>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <pwd.h>
// #include <boost/thread.hpp>
// #include <boost/thread/mutex.hpp>
// #include <boost/lexical_cast.hpp>
// #include <boost/bind.hpp>
#include "../sc_lib_public_tools/include/rapidjson/document.h"
#include "../sc_lib_public_tools/include/rapidjson/prettywriter.h"
#include "../sc_lib_public_tools/include/rapidjson/ostreamwrapper.h"
#include "../sc_lib_public_tools/include/rapidjson/istreamwrapper.h"
#include "../sc_lib_public_tools/include/rapidjson/filereadstream.h"   // FileReadStream
#include "../sc_lib_public_tools/include/rapidjson/encodedstream.h"    // AutoUTFInputStream
#include "../sc_lib_public_tools/include/rapidjson/error/en.h"
#include <fstream>

// g++ -std=c++11 test.cc -lboost_system -lboost_thread
#define LOG(INFO) cout
// #define LOG(ERROR) cerr

#define BOOST_DATE_TIME_SOURCE
#define BOOST_THREAD_NO_LIB

using namespace std;
// using namespace boost;

int main() {
    typedef vector<string> geoPoints_t;
    typedef vector<geoPoints_t> geoLines_t;

    geoPoints_t v3({"1", "2", "3"});
    geoLines_t mRecordedAbsLines(4, v3);

    size_t recordedPointCnt = 0;
    for(auto recordedLine: mRecordedAbsLines) {
        recordedPointCnt += recordedLine.size();
    }
    cout << "Record way points: " << recordedPointCnt << endl;

    LOG(INFO) << "do rarefaction: " << recordedPointCnt;
    for(auto &recordedLine: mRecordedAbsLines) {
        // for(auto iter = recordedLine.begin(); iter != recordedLine.end();) {
        //     iter = recordedLine.erase(iter);
        //     ++iter;
        // }
        // for() {

        // }
    }
    LOG(INFO) << "do rarefaction end: " << recordedPointCnt;

     vector<int> i3({1, 2, 3, 4});
     for(auto iter = i3.begin(); iter != i3.end();) {
        ++iter;
        if(i3.end() == iter) {
            break;
        }
        iter = i3.erase(iter);
    }



    // for(auto &recordedLine: mRecordedAbsLines) {
    //     for(auto iter = recordedLine.begin(); iter != recordedLine.end(); ) {
    //         cout << "1: " << *iter << endl;
    //         ++iter;
    //         // erase: Index Out of Bounds: Segmentation fault (core dumped)
    //         iter = recordedLine.erase(iter);
    //         // *iter: access index Out of Bounds: didn't core, but dangerous(do not do that)
    //         cout << "2: " << *iter << endl;
    //     }
    // }

    // recordedPointCnt = 0;
    // for(auto &recordedLine: mRecordedAbsLines) {
    //     recordedPointCnt += recordedLine.size();
    // }
    // cout << "Record way points: " << recordedPointCnt << endl;

    // mRecordedAbsLines.push_back(vector<string>( {"Hi"} ) );

    recordedPointCnt = 0;
    for(auto &recordedLine: mRecordedAbsLines) {
        recordedPointCnt += recordedLine.size();
    }
    cout << "Record way points: " << recordedPointCnt << endl;

    mRecordedAbsLines.clear();
    cout << mRecordedAbsLines.back().back() << endl;


}
