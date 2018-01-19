#ifndef PUBLIC_TOOLS_H
#define PUBLIC_TOOLS_H

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <visualization_msgs/Marker.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <dirent.h>

namespace public_tools
{

typedef struct {
    double x;
    double y;
    double z;
} pointXYZ_t;

typedef geometry_msgs::Point geoPoint_t;
typedef vector<geometry_msgs::Point> geoPoints_t;
typedef vector<geoPoints_t> geoLines_t;


class PublicTools {
public:

    template <typename T>
    static T string2num(const string& str, const T retIfEmpty);

    static void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y, double *x, double LP);
    static void transform_coordinate(const vector<geometry_msgs::Point> &points_gauss, const geometry_msgs::Point &current_gauss, vector<geometry_msgs::Point> &points_transformed);
    static double deg2rad(const double deg);
    static double getDaySecond(const double rosTime, const double pktTime);
    template <typename T>
    static void tf_rotate(const T &in_xyz, const pointXYZ_t &angle_xyz, const pointXYZ_t &offset, T &out_xyz);
    static void generateFileName(const std::string &path, std::string &fileName);
    static void getFilesInDir(const std::string &baseDir, const std::string &keyWord, std::vector<std::string> &files);

private:
    // only used by tf_rotate
    template <typename T>
    static void multiply_matrix(const T &in_xyz, const double tf_matrix[][3], T &out_xyz);
};

// template function should placed in header file, else report undefined reference error

// ! retIfEmpty determine the return type:
// ! If you want return a double, and already check input str not empty: you can simply set retIfEmpty (double)0
template <typename T>
T PublicTools::string2num(const string& str, const T retIfEmpty) {
    if(str.empty() ) {
        LOG(INFO) << __FUNCTION__ << " param is null.";
        return retIfEmpty;
    }
    std::istringstream iss(str);
    T num;
    iss >> num;
    return num;
}

template <typename T>
void PublicTools::multiply_matrix(const T &in_xyz, const double tf_matrix[][3], T &out_xyz) {
    out_xyz.x = tf_matrix[0][0] * in_xyz.x + tf_matrix[0][1] * in_xyz.y + tf_matrix[0][2] * in_xyz.z;
    out_xyz.y = tf_matrix[1][0] * in_xyz.x + tf_matrix[1][1] * in_xyz.y + tf_matrix[1][2] * in_xyz.z;
    out_xyz.z = tf_matrix[2][0] * in_xyz.x + tf_matrix[2][1] * in_xyz.y + tf_matrix[2][2] * in_xyz.z;
  }

template <typename T>
void PublicTools::tf_rotate(const T &in_xyz, const pointXYZ_t &angle_xyz, const pointXYZ_t &offset, T &out_xyz) {
    double Ax[3][3], Ay[3][3], Az[3][3];
    Ax[0][0] = 1; Ax[0][1] = 0; Ax[0][2] = 0;
    Ax[1][0] = 0; Ax[1][1] = cos(angle_xyz.x); Ax[1][2] = sin(angle_xyz.x);
    Ax[2][0] = 0; Ax[2][1] = -sin(angle_xyz.x); Ax[2][2] = cos(angle_xyz.x);
    Ay[0][0] = cos(angle_xyz.y); Ay[0][1] = 0; Ay[0][2] = -sin(angle_xyz.y);
    Ay[1][0] = 0; Ay[1][1] = 1; Ay[1][2] = 0;
    Ay[2][0] = sin(angle_xyz.y); Ay[2][1] = 0; Ay[2][2] = cos(angle_xyz.y);
    Az[0][0] = cos(angle_xyz.z); Az[0][1] = sin(angle_xyz.z); Az[0][2] = 0;
    Az[1][0] = -sin(angle_xyz.z); Az[1][1] = cos(angle_xyz.z); Az[1][2] = 0;
    Az[2][0] = 0; Az[2][1] = 0; Az[2][2] = 1;
    T out1_xyz, out2_xyz, out3_xyz;
    multiply_matrix(in_xyz, Ax, out1_xyz);
    multiply_matrix(out1_xyz, Ay, out2_xyz);
    multiply_matrix(out2_xyz, Az, out3_xyz);
    out_xyz.x = offset.x + out3_xyz.x;
    out_xyz.y = offset.y + out3_xyz.y;
    out_xyz.z = offset.z + out3_xyz.z;
}

}
// namespace public_tools

#endif