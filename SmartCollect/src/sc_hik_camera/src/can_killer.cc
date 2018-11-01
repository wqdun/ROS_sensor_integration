#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <bitset>
#include "controlcan.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

void StopDevice() {
    usleep(100000);//延时100ms
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms
    VCI_CloseDevice(VCI_USBCAN2, 0);
    usleep(100000);//延时100ms
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << __FUNCTION__ << " start.";
    if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) != 1) {
        LOG(INFO) << "Failed to open device.";
        StopDevice();
        return 1;
    }
    LOG(INFO) << "Open device successfully.";
    StopDevice();
    return 0;
}

// g++ can_test.cpp can_parser.cpp -lglog -Wl,-rpath,/home/nvidia/Music -L. -lcontrolcan

