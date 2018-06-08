#ifndef DATAFIXED_H
#define DATAFIXED_H
#pragma GCC diagnostic error "-std=c++11"
#include <ros/ros.h>
#include "sc_msgs/DataFixerProgress.h"
#include <string>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <ogrsf_frmts.h>
#pragma pack(1)
struct ImageStruct
{
    std::string timestamp;
    int CammeraIndex;
};

struct ontimeDataFormat{
    int GPSWeek;
    double GPSWeekTime;
    double Heading;
    double Pitch;
    double Roll;
    double Latitude;
    double Longitude;
    double Height;
    double Ve;
    double Vn;
    double Vu;
    double Baseline;
    int NSV1;
    int NSV2;
};

const size_t BIT_IN_PACKET_DATA = 1206;

struct imageTraceDataFormat{
    std::string Pano_name;
    std::string Date;
    double GpsTime;
    std::string BeijingTime;
    double Easting;
    double Northing;
    double H_ell;
    double Latitude;
    double Longitude;
    double Roll;
    double Pitch;
    double Heading;
};

class dataFixed{
public:
    typedef struct {
        double timeStamp;
        unsigned char data[1206];
    }pktDataFormat;
    int readOntimeTraceData(std::string &projectPath,std::vector<ontimeDataFormat> &traceFileData);
    void GeoToGauss(double longitude, double latitude, short beltWidth, double *y, double *x);
    int reNameImageAndMkTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData);
    int mkLidarTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData);
    int mkImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,std::string &reNewPicture,imageTraceDataFormat &oneLostImageTraceData);
    int mkLostImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,imageTraceDataFormat &oneLostImageTraceData);
    void saveImageTraceData(std::string &savePath,std::string &projectName,std::vector<imageTraceDataFormat> &imageTraceData,int saveMark);
    dataFixed(ros::NodeHandle node, ros::NodeHandle private_nh, int cinValue);
    void fixProjectsData(const std::string &_projects);
    bool panoramasSort(std::string &projectPath);

    unsigned long totalFileNum;
    unsigned long processNum;


private:
    int imageCollectionHz;
    double beganGPSTime;
    double endGPSTime;
    double minGPSTime;
    double maxGPSTime;
    unsigned long img2LidarProsVeloRatio = 5000;

    unsigned long minGPSTimeMark;
    std::string belongtoLidarProjectName;
    ros::Publisher pubProgress_;
    sc_msgs::DataFixerProgress progressMsg_;

    void initMemberVar();
    bool markPointGeo2Gauss(std::string &projectPath);
    void pubProgress(unsigned long _processNum, unsigned long _totalFileNum);
    void removeFile(const std::string &file);
    void touchFile(const std::string &file);
};

#pragma pack()
#endif
