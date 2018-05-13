#include "public_tools.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

namespace public_tools
{
void PublicTools::GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP) {
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double jd_hd, wd_hd;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;     //  克拉索夫斯基椭球中子午弧长
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos, et3;   //  sinB,cosB
    double PI = 3.14159265358979;
    double b_e2 = 0.00669437999013;
    double b_c = 6378137;
    jd_hd = jd / 3600.0 * PI / 180.0;     // 将以秒为单位的经度转换成弧度
    wd_hd = wd / 3600.0 * PI / 180.0;    // 将以秒为单位的纬度转换成弧度
                                         // 如果不设中央经线（缺省参数: -1000），则计算中央经线，
                                         // 否则，使用传入的中央经线，不再使用带号和带宽参数
                                         //L = (DH - 0.5) * DH_width       // 计算中央经线的经度
    if (LP == -1000)
    {
        L = (DH - 0.5) * DH_width;
        // 计算中央经线的经度
    }
    else
    {
        L = LP;
    }
    l0 = jd / 3600.0 - L;       // 计算经差
    tsin = sin(wd_hd);        // 计算sinB
    tcos = cos(wd_hd);        // 计算cosB
                              // 计算克拉索夫斯基椭球中子午弧长X
                              //X = 111134.8611 / 3600.0 * wd - (32005.7799 * tsin + 133.9238 * pow(tsin, 3) + 0.6976 * pow(tsin, 5) + 0.0039 * pow(tsin, 7)) * tcos;
    X = 111132.9558 / 3600.0*wd - 16038.6496*sin(2 * wd_hd) + 16.8607*sin(4 * wd_hd) - 0.0220*sin(6 * wd_hd);
    et2 = b_e2 * pow(tcos, 2); //  et2 = (e' ** 2) * (cosB ** 2)
    et3 = b_e2 * pow(tsin, 2);
    N = b_c / sqrt(1 - et3);     //  N = C / sqrt(1 + et2)
    t = tan(wd_hd);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    *x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);

    *y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
}

void PublicTools::transform_coordinate(const geoPoints_t &points_gauss, const geoPoint_t &current_gauss, geoPoints_t &points_transformed, double scaleRatio) {
    DLOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";
    points_transformed.clear();
    geoPoint_t temp_point;
    for(auto &point: points_gauss) {
        // 100 m/grid
        temp_point.x = (point.x - current_gauss.x) * scaleRatio;
        temp_point.y = (point.y - current_gauss.y) * scaleRatio;
        // temp_point.z = (point.z - current_gauss.z);
        // ignore height, always 0
        temp_point.z = 0 * scaleRatio;
        points_transformed.push_back(temp_point);
    }
}

void PublicTools::generateFileName(const std::string &path, std::string &fileName, bool isAppendTime) {
    // get project name from path
    const std::string prj_start("/record/");
    const std::string::size_type prj_start_index = path.find(prj_start) + prj_start.size();
    const std::string::size_type prj_end_index = path.find("/Rawdata/");
    if( (path.npos == prj_start_index) || (path.npos == prj_end_index) ) {
        ROS_ERROR_STREAM("Wrong path, path " << path);
        exit(1);
    }
    const std::string prj_name(path.substr(prj_start_index, prj_end_index - prj_start_index) );

    // remove "-" from project name to get file name
    std::vector<std::string> parsed_prj_name;
    boost::split(parsed_prj_name, prj_name, boost::is_any_of( "-" ) );
    if(4 != parsed_prj_name.size() ) {
        ROS_ERROR_STREAM("Wrong project name, parsed_prj_name.size(): " << parsed_prj_name.size() );
        exit(1);
    }
    fileName = parsed_prj_name[0] + parsed_prj_name[1] + parsed_prj_name[2] + parsed_prj_name[3];

    if(!isAppendTime) {
        return;
    }

    // append UNIX local time (Beijing time) stamp to file name
    time_t tt = time(NULL);
    tm *t= localtime(&tt);
    char nowTime[50];
    (void)sprintf(nowTime, "%02d%02d%02d", t->tm_hour, t->tm_min, t->tm_sec);
    fileName += nowTime;
    return;
}

void PublicTools::getFilesInDir(const std::string &baseDir, const std::string &keyWord, std::vector<std::string> &files) {
    DIR *dir;
    dirent *ptr;

    // if dir == NULL
    if( !(dir = opendir(baseDir.c_str())) ) {
        LOG(WARNING) << "Failed to open " << baseDir;
        return;
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
            const std::string fileName(ptr->d_name);
            if(std::string::npos != fileName.find(keyWord) ) {
                files.push_back(baseDir + "/" + fileName);
            }
            // else do nothing
            // else {}
        }
        // directory
        else
        if(4 == ptr->d_type) {
            const std::string subDir(baseDir + "/" + ptr->d_name);
            getFilesInDir(subDir, keyWord, files);
        }
        // else {
        //     // ignore links(10) & others
        // }
    }

    closedir(dir);
    return;
}

int PublicTools::popenWithReturn(const std::string &cmd, std::vector<std::string> &cmdReturn) {
    const size_t maxByte = 1000;
    char result[maxByte];
    FILE *fpin;

    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to open " << cmd;
        return -1;
    }

    while(fgets(result, maxByte, fpin) ) {
        cmdReturn.push_back(result);
    }

    if(0 != pclose(fpin) ) {
        LOG(WARNING) << "Failed to close " << cmd;
        return -2;
    }

    return 0;
}

// degree to radian
double PublicTools::deg2rad(const double deg) {
  return deg * M_PI / 180;
}

double PublicTools::getDaySecond(const double rosTime, const double pktTime) {
  // Ros time is UTC time(+0), not local time(Beijing: +8)
  int rosHour = (int)rosTime / 3600 % 24;
  const int rosMinute = (int)rosTime / 60 % 60;
  const int pktMinute = (int)pktTime / 60;
  const int errMinute = rosMinute - pktMinute;
  if(errMinute > 20) {
    ++rosHour;
  }
  else
  if(errMinute < -20) {
    --rosHour;
  }
  // else {
  //   // do nothing when errMinute in [-10, 10]
  // }

  // in case: -1 || 24
  rosHour = (rosHour + 24) % 24;

  ROS_INFO_STREAM_ONCE("UTC time +18 s is GPS time.");
  return fmod(pktTime + 3600 * rosHour + 18., 86400.);
}

// Copyright (c) 2013 Uli Köhler
// License: Apache2.0
// A buffer-overflow-safe readlink() wrapper for C++.
// return A string containing the readlink()ed filename, or
// an empty string with errno being set to the appropriate error.
// See the readlink() man(2) for errno details.
// URL: https://techoverflow.net/2013/12/31/buffer-overflow-safe-readlink-in-c/
std::string PublicTools::safeReadlink(const std::string& filename) {
    size_t bufferSize = 255;

    //Increase buffer size until the buffer is large enough
    while (1) {
        char* buffer = new char[bufferSize];
        size_t rc = readlink (filename.c_str(), buffer, bufferSize);
        if (rc == -1) {
            delete[] buffer;
            if(errno == EINVAL) {
                //We know that bufsize is positive, so
                // the file is not a symlink.
                errno = 0;
                return filename;
            } else if(errno == ENAMETOOLONG) {
                bufferSize += 255;
            } else {
                //errno still contains the error code
                return "";
            }
        } else {
            //Success! rc == number of valid chars in buffer
            errno = 0;
            return string(buffer, rc);
        }
    }
}

bool PublicTools::isFileExist(const std::string& fileName) {
    std::fstream _file;
    _file.open(fileName.c_str(), std::ios::in);
    return (bool)(_file);
}

void PublicTools::runShellCmd(const std::string &cmd) {
    LOG(INFO) << __FUNCTION__ << " start, run " << cmd;

    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << cmd;
        exit(1);
    }
    int err = 0;
    if(0 != (err = pclose(fpin) ) ) {
        LOG(ERROR) << "Failed to run " << cmd << ", returns: " << err;
        exit(1);
    }
    LOG(INFO) << "Run: " << cmd << " end.";
    return;
}

bool PublicTools::isInChina(double lat, double lon) {
   LOG(INFO) << __FUNCTION__ << " start.";
   if (lat >= 20) {
    return true;
   }
   return false;
}


}
// namespace public_tools
