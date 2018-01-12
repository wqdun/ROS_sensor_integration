#include <iostream>
#include <sstream>
#include <cmath>
#include <bitset>
#include <string>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <pthread.h>
#include <unistd.h>
// GDAL library
#include <gdal.h>
#include "gdal_alg.h"
#include "cpl_conv.h"
#include "cpl_port.h"
#include "cpl_multiproc.h"
#include "ogr_srs_api.h"
// #include "proj_api.h"

using namespace std;

pthread_t ntid;
int g_i = 0;

void *thread1(void *arg) {
    while(1) {
        cout << "thread1" << endl;
        ++g_i;
        usleep(500000);

    }
    // ++g_i;
}

const short month[2][13] {
    365, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    366, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
};

bool isLeapYear(int year) {
    return (
        (0 == (year % 4) )
        && ( (0 != (year % 100) ) || (0 == (year % 4) ) )
    );
}

static void generateFileName(const string &path, string &fileName) {
  // get project name from path
  const string prj_start("/record/");
  string::size_type prj_start_index = path.find(prj_start) + prj_start.size();
  string::size_type prj_end_index = path.find("/rawdata/");
  if( (path.npos == prj_start_index) || (path.npos == prj_end_index) ) {
    cout << "Wrong path, path " << path << endl;
    exit(1);
  }
  const string prj_name(path.substr(prj_start_index, prj_end_index - prj_start_index) );

  // remove "-" from project name to get file name
  vector<string> parsed_prj_name;
  boost::split(parsed_prj_name, prj_name, boost::is_any_of( "-" ) );
  if(4 != parsed_prj_name.size() ) {
    cout << "Wrong project name, parsed_prj_name.size(): " << parsed_prj_name.size() << endl;
    exit(1);
  }
  fileName = parsed_prj_name[0] + parsed_prj_name[1] + parsed_prj_name[2] + parsed_prj_name[3];

  // append UNIX local time (Beijing time) stamp to file name
  time_t tt = time(NULL);
  tm *t= localtime(&tt);
  char nowTime[50];
  (void)sprintf(nowTime, "%02d%02d%02d", t->tm_hour, t->tm_min, t->tm_sec);
  fileName += nowTime;
}



int main(int argc, char **argv) {
  string mRecordFile = "/home/navi/catkin_ws/record/1005-1-077-180110/rawdata/Lidar/";
  string lidarFileName("");
  (void)generateFileName(mRecordFile, lidarFileName);
  mRecordFile += (lidarFileName + "_lidar.dat");
  cout << mRecordFile << endl;
}


