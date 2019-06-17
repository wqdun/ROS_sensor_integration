//#!/bin/sh

//#for file in $(ls /home/dun/1005-1-008566-180131/Rawdata/Image)
//#do
 // #  echo $file
//  #cppExe $file
//  #./a.out
//  #mv Image/$file Image/hello_$file
//#done
#include "dataFixed.h"
int main(){
    std::vector<ontimeDataFormat> imuData;
    int imageCollectionHz=0;
    std::cout << "Please input frequency of collecting Image:" << std::endl;
    std::cin >> imageCollectionHz;
    std::string projectPath;
    //="/home/dun/1005-1-008566-180131/";
    std::cout << "Please input Project Path Way:" << std::endl;
    std::cin >> projectPath;
    dataFixed rawData(imageCollectionHz);
    std::cout << "Loading IMU ontime Data" << std::endl;
    rawData.readOntimeTraceData(projectPath,imuData);
    if(0==imuData.size())
    {
        std::cout << "Load IMU ontime Data failed!" << std::endl;
        return 1;
    }
    rawData.reNameImageAndMkTraceFile(projectPath,imuData);
    std::cout << "Generating Lidar Data ontime Trace Data" << std::endl;
    rawData.mkLidarTraceFile(projectPath,imuData);
    std::cout << "work done" << std::endl;
 return 0;
}