#include "can_parser.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

CanParser::CanParser() {
    LOG(INFO) << __FUNCTION__ << " start.";

    decimalResult_ = -1.;
    isCanParserRunning_ = true;
}

void CanParser::StopDevice() {
    isCanParserRunning_ = false;
    usleep(100000);//延时100ms
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms
    VCI_CloseDevice(VCI_USBCAN2, 0);
    usleep(100000);//延时100ms
}

CanParser::~CanParser() {
    LOG(INFO) << __FUNCTION__ << " start.";

    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms。
    VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    usleep(100000);//延时100ms。
    // 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
    LOG(INFO) << __FUNCTION__ << " end.";
    exit(1);
}

int CanParser::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    if(1 != VCI_OpenDevice(VCI_USBCAN2, 0, 0)) {
        LOG(ERROR) << "Open device error.";
        StopDevice();
        return -1;
    }
    if(1 != VCI_ReadBoardInfo(VCI_USBCAN2, 0, &vciInfo_) ) {
        LOG(ERROR) << "Get VCI_ReadBoardInfo error.";
        return -1;
    }

    VCI_INIT_CONFIG config;
    // 只接受A1的ID
    config.AccCode = 0x14200000;
    config.AccMask = 0x01E00000;
    // 接收所有帧
    config.Filter = 1;
    // 波特率500 Kbps  0x03  0x1C
    config.Timing0 = 0x00;
    config.Timing1 = 0x1C;
    // 正常模式
    config.Mode = 0;

    if(1 != VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) ) {
        LOG(ERROR) << "Initial CAN1 error.";
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if(1 != VCI_StartCAN(VCI_USBCAN2, 0, 0) ) {
        LOG(ERROR) << "Start CAN1 error.";
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if(1 != VCI_InitCAN(VCI_USBCAN2, 0, 1, &config)) {
        LOG(ERROR) << "Initial CAN2 error.";
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if(1 != VCI_StartCAN(VCI_USBCAN2, 0, 1) ) {
        LOG(ERROR) << "Start CAN2 error.";
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    Receiver();
    return 0;
}

void CanParser::Receiver() {
    int reclen = 0;
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳

    const double CIRCUMFERENCE = 2.1595677;

    int ind = 0;
    double velocity = 0;
    int level = 0;
    LOG(INFO) << "isCanParserRunning_: " << isCanParserRunning_;
    while(isCanParserRunning_) {
        // 调用接收函数，如果有数据，进行数据处理显示
        if( (reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 0) ) > 0) {
            for(int j = 0; j < reclen; j++) {
                // velocity
                if(rec[j].ID == 0xA1) {
                    unsigned int convert_byte7 = rec[j].Data[6] & 0x3f;
                    unsigned int merge = convert_byte7<<8|rec[j].Data[5];
                    LOG(INFO) << "merge: " << merge;
                    double rpm = int(merge) * 0.5;
                    velocity = rpm / 60. * CIRCUMFERENCE;
                }
                // 1: reverse gear
                // 0: neutral
                // 2: stop gear
                // 4: driving gear
                if(rec[j].ID == 0xAE) {
                    unsigned int convert_level_byte7 = rec[j].Data[6] & 0x07;
                    level = convert_level_byte7;
                    LOG(INFO) << "level: " << level;
                }

                if(1 == level) {
                    decimalResult_ = velocity * -1.;
                }
                else {
                    decimalResult_ = velocity;
                }
            }
        }
        // 变换通道号，以便下次读取另一通道，交替读取
        ind = !ind;
    }

    LOG(INFO) << "decimalResult_: " << decimalResult_;
    LOG(INFO) << "Run thread exit.";
}


