/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _FRAME_LIB_H_
#define _FRAME_LIB_H_
#include <iostream>
#include <string>
#include "rfans_driver/RfansPacket.h"

using namespace std;

const static int REG_DEVICE_CTRL = (0x40);
const static int REG_DATA_TRANS = (0x70);

const static int CMD_SCAN_SPEED_5HZ = 0;
const static int CMD_SCAN_SPEED_10HZ = 0x50;
const static int CMD_SCAN_SPEED_20HZ = 0xF0;
const static int CMD_SCAN_ENABLE = 0x1;
const static int CMD_LASER_ENABLE = 0x2;
const static int CMD_CALC_DATA = 0x2;
const static int CMD_DEBUG_DATA = 0x5;
const static int CMD_RCV_CLOSE = 0x0;

const static int ANGLE_SPEED_5HZ = 5;
const static int ANGLE_SPEED_10HZ = 10;
const static int ANGLE_SPEED_20HZ = 20;

static unsigned short DEVICE_PORT_NUMBER = 2014;
static unsigned short PC_PORT_NUMBER = 2014;
static std::string DEVICE_IP_STRING = "192.168.0.3";

static unsigned short UDP_FRAME_MIN =(18);

#define DEB_FRAME_WRITE (0xA5)  //write head sync
#define DEB_FRAME_READ  (0x5a)  //read head sync
#define DEB_FRAME_ERROR  (0xE7) //err  haad sync

#define FRAME_MSG_LENGTH (1024)

const int UDPREG_MAX_COUNT = 256;
const int ROMREG_MAX_COUNT = 0x7FF;

const unsigned char RFANS_PRODUCT_MODEL_V6G_X16_0X32 = 0X32;
const unsigned char RFANS_PRODUCT_MODEL_V6G_X32_0X33 = 0X33;

const unsigned char RFANS_PRODUCT_MODEL_V6_X32_0X40 = 0X40;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16A_0X41 = 0X41;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16B_0X42 = 0X42;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16Even_0X43 = 0X43;
const unsigned char RFANS_PRODUCT_MODEL_V6_X16Odd_0X44 = 0X44;

const unsigned char RFANS_PRODUCT_MODEL_V6P_X32_0X45 = 0X45;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16A_0X46 = 0X46;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16B_0X47 = 0X47;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16Even_0X48 = 0X48;
const unsigned char RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49 = 0X49;

const unsigned char RFANS_PRODUCT_MODEL_V6A_X32_0X4A = 0X4A;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16A_0X4B = 0X4B;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16B_0X4C = 0X4C;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D = 0X4D;
const unsigned char RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E = 0X4E;

const unsigned char RFANS_PRODUCT_MODEL_V6A_X16M_0X4F = 0X4F;
const unsigned char RFANS_PRODUCT_MODEL_V6B_X32_0X50=0X50 ;

// const unsigned char RFANS_PRODUCT_MODEL_CFANS_X32_0X3780=0X3780 ;
const unsigned char RFANS_PRODUCT_MODEL_V6A_E1_0X55 = 0X55;
const unsigned char RFANS_PRODUCT_MODEL_V6A_E2_0X56 = 0X56;

const int RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[] = {
  5, 7, 9, 11, 13, 15, 16, 18, 17, 19, 20, 21, 23, 25, 27, 29

};

const int RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[] = {
  6,8,10,12,14,16,18,17,19,20,22,21,24,26,28,30
};

const int RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[] = {
  1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31
};

const int RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[] = {
  0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30
};

////ŒÔ¿Ì±‡∫≈Ω«∂»À≥–Ú
const double HANGLE_V6B_X32_0x40[] = {
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
    6.01, -4.068, 3.377, -6.713,
};


const double HANGLE_V5_X16[] = {
  -2.5, 2.5,
  -2.5, 2.5,
  -2.5, 2.5,
  -2.5, 2.5,
  -2.5, 2.5,
  -2.5, 2.5,
  -2.5, 2.5,
  -2.5, 2.5 };

const double VANGLE_V5_X16[] = {
  -15.0, -13.0,
  -11.0, -9.0,
  -7.0, -5.0,
  -4.0, -3.0,
  -2.0, -1.0,
     0,  1.0,
   3.0,  5.0,
   7.0,  9.0,  };

//ŒÔ¿Ì±‡∫≈Ω«∂»À≥–Ú
const double HANGLE_V6_X32_0x40[] = {
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,
  6.35, -3.85, 3.85, -6.35,

};

const double HANGLE_V6A_E1_0x55[] = {
    -4.068, -6.713, -4.068, -6.713,
    -4.068, -6.713, -4.068, -6.713,
    -4.068, -6.713, -4.068, -6.713,
    -4.068, -6.713, -4.068, -6.713,
};

const double VANGLE_V6_X32_0X40[] = {
  -20.5, -19.5, -18.5, -17.5,
  -16.5, -15.5, -14.5, -13.5,
  -12.5, -11.5, -10.5, -9.5,
  -8.5,   -7.5,  -6.5, -5.5,
  -4.5,   -3.5,  -2.5, -1.5,
  -0.5,    0.5,   1.5,  2.5,
  3.5,     4.5,   5.5,  6.5,
  7.5,     8.5,   9.5, 10.5,
};

const double VANGLE_V6A_X32[] = {
  -20, -19, -18, -17,
  -16, -15, -14, -13,
  -12, -11, -10,  -9,
   -8,  -7,  -6,  -5,
   -4,  -3,  -2,  -1,
    0,   1,   2,   3,
    4,   5,   6,   7,
    8,   9,  10,  11,
};
static double VAngle_16E1[16] = {
    -19.5, -17.5, -15.5, -13.5,
    -11.5, -9.5, -7.5, -5.5,
    -3.5, -1.5, 0.5, 2.5,
    4.5, 6.5, 8.5, 10.5
};
static double VAngle_16E2[16] = {
    -19, -17, -15, -13,
    -11, -9, -7, -5,
    -3,    -1, 1, 3,
    5, 7, 9, 11
};

const double HANGLE_V6G_X32_0X33[] = {
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
  5.52, -3.48, 3.48, -5.52,
};

const double VANGLE_V6G_X32_0X33[] = {
  -25,    -22,   -19, -16,
  -13,    -11,    -9,  -7,
 -5.5,   -4.5,  -3.5, -2.9,
-2.45,   -2.1, -1.75, -1.4,
-1.05,   -0.7, -0.35,    0,
 0.35,    0.7,  1.05,  1.4,
  2.5,    3.5,   4.5,    6,
    8,     10,    12,   15,
 };


typedef enum{
  eCmdWrite,  //write command
  eCmdRead,   //read command
  eCmdQuery,  //Query command
} SCD_FRAME_TYPE_E ;

typedef struct _frams_buffer{
  char msgStream[FRAME_MSG_LENGTH];
  int writeIdx;
  int readIdx;
  int length;
} FRAMS_BUFFER_S;

#pragma pack(1)
typedef struct {                 //! rfans udp command package
  unsigned char msgHead;         //!< head sync
  unsigned char msgCheckSum;     //!< check sum
  unsigned short regAddress;     //!< register add
  unsigned int regData;          //!< register data
} DEB_FRAME_S;

typedef enum {
  eDevCmdIdle = 0,
  eDevCmdWork,
  eDevCmdSimu,
  eDevCmdBreak,
  eDevCmdReset,
  eDevCmdAsk,
} DEB_CMD_E;

typedef enum {
  eFormatCalcData = 0x2,
  eFormatDebugData = 0x5,
}DEB_DFORMAT_E;


static int DEVICE_MOTOR_HZ = 5;
typedef struct {
  DEB_CMD_E cmdstat;
  DEB_DFORMAT_E dataFormat;
  int scnSpeed;
  int lsrFreq;
  float rangeMin, rangeMax;
}DEB_PROGRM_S;

static const size_t upd_packet_size = 0x8000;  //32KB
const unsigned char ID_RFANSBLOCKV2_SYNC = 0x96;
const unsigned char ID_RFANSBLOCKV32_0_15_SYNC = 0x97;
const unsigned char ID_RFANSBLOCKV32_16_31_SYNC = 0x98;

const unsigned char ID_RFANSBLOCKV6G_0_15_SYNC = 0x99;
const unsigned char ID_RFANSBLOCKV6G_16_31_SYNC = 0x9A;

const unsigned char ID_RFANSBLOCKV32_GM_0_15_SYNC = 0x87;
const unsigned char ID_RFANSBLOCKV32_GM_16_31_SYNC = 0x88;

const unsigned char ID_RFANSBLOCK_GM_SYNC = 0xee;


const int UDP_PACKET_SIZE_V5A = 1380;
const int UDP_PACKET_SIZE_V6G = 1206;

typedef struct {
  unsigned short angle  : 16 ;          //scan angle, 0.01°„
  unsigned short rangeOne  : 16 ;       //echo 1,  cm
  unsigned short rangeTwo  : 16 ;       //echo 2,  cm
  unsigned char  intentOne : 8 ;        //0~255
  unsigned char  intentTwo : 8 ;        //0~255
}SCDRFANS_POINT_S;

const int RFANS_LASER_COUNT = 16 ;
typedef struct {                                        //138 byte
  unsigned char           dataID : 8;                   //Sync Number
  unsigned char           chksum : 8;                   //chksum  Bytes[3,138]
  unsigned int            t0stampH  : 32;               //T0 Bloceek
  unsigned int            t0stampL  : 32;               //T0 tag
  SCDRFANS_POINT_S        laserData[RFANS_LASER_COUNT] ;//
}SCDRFANS_BLOCK_S;

typedef struct {
  float x,y,z ;
  float intent;
  unsigned char laserid;
  double timeflag;
  //float angle, range ;
}RFANS_XYZ_S;

static const int DECODE_BUFFER_SIZE = 0x40000; // 256KB

static const int UDP_PACKET_SIZE = 2*1024; // 256KB
static const int UDP_PACKET_BUFFER_SIZE = 16*1024; // 256KB
typedef struct  {
  int packetsize ;
  unsigned char buffer[UDP_PACKET_SIZE];
}UDP_PACKET_S;

typedef struct {
  int wrHead, rdTail, bufSize;
  UDP_PACKET_S buffer[UDP_PACKET_BUFFER_SIZE];
} UDP_PACKET_BUFFER_S;


typedef struct {
  int wrHead, rdTail, bufSize;
  unsigned char buffer[DECODE_BUFFER_SIZE];
} UDP_DECBUFFER_S;

typedef struct
{
  unsigned short range;
  unsigned char intensity;
}RFans_Laser32Block_S;

const unsigned short RFANS_UDPFRAMV6G_FLAT = 0xFFEE;
const unsigned short LASER32BLOCK_COUNT = 32;
typedef struct
{
  unsigned short flag; //oxFFEE
  unsigned short azimuthAngle;
  RFans_Laser32Block_S laserBlock[32];
}RFans_DataBlock_S;
const unsigned short UDP32FRAMEV6G_COUNT = 12;
const unsigned short RFANS_GM_16_FLAG = 0x3732;
//const unsigned short RFANS_GM_32_FLAG = 0x373C;
const unsigned short RFANS_V6_GM_33_FLAG = 0x3733;


typedef struct
{
  SCDRFANS_BLOCK_S blockdata[10];
}RFans_UDPFRAMEV5_S;

typedef struct
{
    //42 bype header + 12*100byte group + 4byte GPS Timestamp + 2byte Reserved
  RFans_DataBlock_S dataBlock[UDP32FRAMEV6G_COUNT];//each 100byte & total 12 group
  unsigned int gpsTimestamp;
  unsigned char gmReservedA;
  unsigned char gmReservedB;
}RFans_UDP32FRAMEV6G_S;

#pragma pack()

#ifdef __cplusplus
extern "C"
{
#endif
int swapchar( unsigned char * _data, int size_ ) ;
int checkSum(unsigned char * _dataBuf, int count_ ) ;

DEB_FRAME_S packDEBV3Frame(SCD_FRAME_TYPE_E flag, int regAddress_, int regData_);

void writeFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size);

void readDEBFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, DEB_FRAME_S *mtRegMap);

int processFrameV5( RFans_UDPFRAMEV5_S *mtFrame,std::vector<SCDRFANS_BLOCK_S> &outBlocks);
int processFrameV6G(RFans_UDP32FRAMEV6G_S *mtFrame, std::vector<SCDRFANS_BLOCK_S> &outBlocks);
//int  searchUDPPacket(UDP_DECBUFFER_S *mtUdpBuffer, BLOCK_VECTOR_S *outBlocks);

int searchBlock(unsigned char *data, int size,int &flag,
								SCDRFANS_BLOCK_S *outBlock) ;

#ifdef __cplusplus
}
#endif


#endif
