#include "public_tools.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

namespace public_tools
{
int PublicTools::string2int(const string& str) {
    std::istringstream iss(str);
    int num;
    iss >> num;
    return num;
}

double PublicTools::string2double(const string& str) {
    std::istringstream iss(str);
    double num;
    iss >> num;
    return num;
}

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

void PublicTools::transform_coordinate(const geoPoints_t &points_gauss, const geoPoint_t &current_gauss, geoPoints_t &points_transformed) {
    DLOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";
    points_transformed.clear();
    geoPoint_t temp_point;
    for(auto &point: points_gauss) {
        // 100 m/grid
        temp_point.x = (point.x - current_gauss.x); // * 0.01;
        temp_point.y = (point.y - current_gauss.y); // * 0.01;
        // temp_point.z = (point.z - current_gauss.z);
        // ignore height, always 0
        temp_point.z = 0;
        points_transformed.push_back(temp_point);
    }
}

void PublicTools::generateFileName(const std::string &path, std::string &fileName) {
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
        LOG(ERROR) << "Failed to open " << baseDir;
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

}
// namespace public_tools
