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
    vector<vector<int> > ivv = {
        {1},
        {5, 6, 7, 8},
        {8, 9},
        {10}
    };
    // ivv.clear();

    for(auto &iv: ivv) {
        if(1 == iv.size() ) {
            continue;
        }
        for(auto i: iv) {
            cout << i << " ";
        }
        cout << endl;
    }
    cout << endl;


    int cnt = 0;
    for(auto iter = ivv.begin(); iter != ivv.end();) {
        cnt += iter->size();
        if(cnt <= 5) {
            iter = ivv.erase(iter);
            if(iter == ivv.end() ) {
                break;
            }
        }
        // cnt > 5
        else {
            iter->erase(iter->begin(), iter->begin() + (iter->size() + 5 - cnt) );
            break;
        }
    }

    for(auto &iv: ivv) {
        for(auto i: iv) {
            cout << i << " ";
        }
        cout << endl;
    }

    cout << ivv.size() << endl;
}
