#include <string.h>
#include <ros/ros.h>
#include "ioapi.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
#include "bufferDecode.h"

static const int LINE_POINT_MINI_COUNT = 500; //一条扫描线最少点的个数
static const float ANGLE_CIRCLE_CONDITION = 270; //角度抖动处理值
static const float UINTCONVERT = 0.01;

static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const size_t BLOCK_COUNT_MAX = sizeof(rfans_driver::RfansPacket().data)/138;

static float s_angle_duration = 359 ;
const double TIME_FLAG_SCALE = 0.0000001;
static int LINE_POINT_COUNT = 128*1024;

static std::vector<RFANS_XYZ_S> s_lineData;
static int s_lineCount = 0 ;
static float s_lastAngle = 0 ;

static std::vector<float> s_vangles;
static std::vector<float> s_hangles;

FILE *logFile = NULL;

inline unsigned int strToList(std::vector<float> &list, std::string str)
{
  unsigned int count = 0;
  string::size_type prev = 0;
  string::size_type pos = 0;
  string tmpValue ;
  list.clear();
  while((pos = str.find_first_of(',', pos))!= string::npos)
  {
    tmpValue =  str.substr(prev, pos - prev);
    list.push_back( atof(tmpValue.c_str()));
    pos++;
    prev = pos;
    count++;
  }
  return count;
}

inline int calcXyz(unsigned char flag,float &mtRange, float &mtAngle, RFANS_XYZ_S &outXyz) {
  int rtn = 1;
  double tmptheta=0, ot = 0 ;

  switch (flag)
  {
  case RFANS_PRODUCT_MODEL_V6G_X32_0X33:
    mtAngle+= HANGLE_V6G_X32_0X33[outXyz.laserid];
    tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
    break;
 case RFANS_PRODUCT_MODEL_V6P_X32_0X45:
  case RFANS_PRODUCT_MODEL_V6_X32_0X40:
    mtAngle+= HANGLE_V6_X32_0x40[outXyz.laserid];
    tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
    break;
 case RFANS_PRODUCT_MODEL_V6P_X16A_0X46:
  case RFANS_PRODUCT_MODEL_V6_X16A_0X41:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    break;

  case RFANS_PRODUCT_MODEL_V6P_X16B_0X47:
  case RFANS_PRODUCT_MODEL_V6_X16B_0X42:

    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    break;
 case RFANS_PRODUCT_MODEL_V6P_X16Even_0X48:
  case RFANS_PRODUCT_MODEL_V6_X16Even_0X43:

    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    break;

  case RFANS_PRODUCT_MODEL_V6P_X16Odd_0X49:
  case RFANS_PRODUCT_MODEL_V6G_X16_0X32:
  case RFANS_PRODUCT_MODEL_V6_X16Odd_0X44:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    tmptheta = VANGLE_V6_X32_0X40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]];
    break;

 case RFANS_PRODUCT_MODEL_V6A_X32_0X4A:
    mtAngle+= HANGLE_V6_X32_0x40[outXyz.laserid];
    tmptheta = VANGLE_V6A_X32[outXyz.laserid] * M_PI / 180.0;
    break;
  case RFANS_PRODUCT_MODEL_V6A_X16A_0X4B:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X41_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6A_X16B_0X4C:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;

    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X42_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6A_X16Even_0X4D:

    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]];
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X43_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6A_X16Odd_0X4E:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    tmptheta = VANGLE_V6A_X32[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]] * M_PI / 180.0;
    mtAngle+= HANGLE_V6_X32_0x40[RFANS_PRODUCT_MODEL_V6_X16A_0X44_LASER_ID[outXyz.laserid]];
     break;
  case RFANS_PRODUCT_MODEL_V6A_E1_0X55:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      tmptheta = VAngle_16E1[outXyz.laserid] * M_PI / 180.0;
      mtAngle+= HANGLE_V6A_E1_0x55[outXyz.laserid];
       break;
  case RFANS_PRODUCT_MODEL_V6A_E2_0X56:
      outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
            outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
      tmptheta = VAngle_16E2[outXyz.laserid] * M_PI / 180.0;
      mtAngle+= HANGLE_V6A_E1_0x55[outXyz.laserid];
       break;
  case ID_RFANSBLOCKV32_16_31_SYNC:
    outXyz.laserid += RFANS_LASER_COUNT;
    tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV6G_16_31_SYNC:
    outXyz.laserid += RFANS_LASER_COUNT;
    tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV32_0_15_SYNC:
    tmptheta = VANGLE_V6_X32_0X40[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV6G_0_15_SYNC:
    tmptheta = VANGLE_V6G_X32_0X33[outXyz.laserid] * M_PI / 180.0;
    break;
  case ID_RFANSBLOCKV2_SYNC:
    tmptheta = VANGLE_V5_X16[outXyz.laserid] * M_PI / 180.0;
    break;
  case RFANS_PRODUCT_MODEL_V6A_X16M_0X4F:
    outXyz.laserid = outXyz.laserid >= RFANS_LASER_COUNT ?
          outXyz.laserid - RFANS_LASER_COUNT : outXyz.laserid;
    mtAngle+= HANGLE_V5_X16[outXyz.laserid];
    tmptheta = VANGLE_V5_X16[outXyz.laserid] * M_PI / 180.0;
     break;
  case RFANS_PRODUCT_MODEL_V6B_X32_0X50:
      mtAngle += HANGLE_V6B_X32_0x40[outXyz.laserid];
      tmptheta = VANGLE_V6A_X32[outXyz.laserid] * M_PI / 180.0;
     break;
  }

  if(mtAngle>360) mtAngle -= 360;
  if(mtAngle<0) mtAngle +=360 ;

  ot = mtAngle*M_PI / 180.0;

  outXyz.x = mtRange*cos(tmptheta) *cos(-ot );
  outXyz.y = mtRange*cos(tmptheta) *sin(-ot );
  outXyz.z = mtRange*sin(tmptheta) ;
  return rtn ;
}

inline int checkFrame_sum(float mtAngle,int mtlaserId,sensor_msgs::PointCloud2 &outCloud) {
  int rtn = 0 ;
  static float s_angleSum = 0;
  float angleDif = 0 ;
  if(mtlaserId!=0) return rtn ;
  if(mtAngle > s_lastAngle) {
    angleDif = mtAngle-s_lastAngle;
    s_angleSum += angleDif;
    s_lastAngle = mtAngle;
  } else {
    //1: 360-> 0
    if(s_lastAngle -mtAngle> 300)  {
      angleDif = 360 - s_lastAngle+mtAngle;
      s_angleSum += angleDif;
      s_lastAngle = mtAngle;
    } else {
      //nothing
    }

  }

  if(s_angleSum >= s_angle_duration) {
    outCloud.width = s_lineCount;
    outCloud.data.resize( outCloud.point_step*outCloud.width  );
    outCloud.row_step = outCloud.data.size();
    memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
    rtn = 1;
    s_lineCount = 0 ;
    s_angleSum = 0 ;
  }
  return rtn ;
}

inline int checkFrame(float mtAngle,int mtlaserId,sensor_msgs::PointCloud2 &outCloud) {
  int rtn = 0 ;
  float angleDif = 0 ;
  if( mtlaserId !=0 ) return rtn ;

  if(mtAngle > s_lastAngle) {
    angleDif = mtAngle-s_lastAngle;
  } else {
    angleDif = 360 - s_lastAngle+mtAngle;
  }

  if(angleDif < 0 ) angleDif= 360+angleDif;

  if(angleDif >= s_angle_duration) {
    outCloud.width = s_lineCount;
    outCloud.data.resize( outCloud.point_step*outCloud.width  );
    outCloud.row_step = outCloud.data.size();
    memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );
    rtn = 1;
    s_lastAngle = mtAngle;
    s_lineCount = 0 ;

  }
  return rtn ;
}

inline int processFrameV6G(RFans_UDP32FRAMEV6G_S *mtFrame, sensor_msgs::PointCloud2 &outCloud)
{
  int rtn = 0;
  bool tmp_isFull = false;
  float s_angleStep = 0;
  unsigned char mtSync;
  RFANS_XYZ_S tmpXyz ;
  int tmpDif = mtFrame->dataBlock[1].azimuthAngle - mtFrame->dataBlock[0].azimuthAngle;
  if ( tmpDif< -35000 ){
    tmpDif = tmpDif + 36000;
  }
  s_angleStep = (tmpDif) / 32.0;

  const float CONVERT_4mm_2_m =0.004f;

  unsigned int tmpDiftime = 0;
  float tmpAngle,tmpRange;

  float timeOffset = 0.0000015625; //us   640KHz

  for (int j = 0; j < UDP32FRAMEV6G_COUNT; j++)//12Group
  {
    RFans_DataBlock_S *mtBlock = &mtFrame->dataBlock[j];
    mtSync = mtBlock->flag;
    for (int i = 0; i < 32; i++) {


      tmpXyz.timeflag = mtFrame->gpsTimestamp*0.000001 + timeOffset*i*j; // us->s//640Hz

      tmpXyz.intent  = mtBlock->laserBlock[i].intensity;

      tmpRange = mtBlock->laserBlock[i].range *CONVERT_4mm_2_m;
      tmpAngle = (mtBlock->azimuthAngle + (i*s_angleStep))*UINTCONVERT;

      tmpXyz.laserid = i;

      calcXyz(mtFrame->gmReservedA,tmpRange, tmpAngle, tmpXyz);
      if ( checkFrame_sum(tmpAngle,tmpXyz.laserid ,outCloud) ) rtn =1 ;

      s_lineData[s_lineCount] = tmpXyz;
      ++s_lineCount;
      if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;
    }

  }
  //m_lastBloc
  return rtn;
}


inline int processFrameV5( RFans_UDPFRAMEV5_S *mtFrame, sensor_msgs::PointCloud2 &outCloud)
{
  int rtn = 0;
  RFANS_XYZ_S tmpXyz ;
  float angleDif = 0 ,T0_STEP_VALUE = 0;
  float tmpAngle,tmpRange;
  for (int i = 0; i < 10; i++)
  {
    SCDRFANS_BLOCK_S *mtBlock = &mtFrame->blockdata[i];
    unsigned char tmpCheck = checkSum(((unsigned char*)mtBlock) + 2, sizeof(SCDRFANS_BLOCK_S)-2);
    if (tmpCheck != mtBlock->chksum) {  //check sum
      continue;
    }

    for(int j = 0 ; j <RFANS_LASER_COUNT;j++ ) {
      tmpXyz.laserid = j;

      switch (mtBlock->dataID ) {
      case ID_RFANSBLOCKV32_16_31_SYNC:
      case ID_RFANSBLOCKV6G_16_31_SYNC:
      case ID_RFANSBLOCKV32_0_15_SYNC:
      case ID_RFANSBLOCKV6G_0_15_SYNC:
        T0_STEP_VALUE = 15.625;
        break;
      case ID_RFANSBLOCKV2_SYNC:
        T0_STEP_VALUE = 31.25;
        break;
      }

      tmpXyz.timeflag = mtBlock->t0stampH + TIME_FLAG_SCALE*(mtBlock->t0stampL + T0_STEP_VALUE*j);
      tmpAngle = mtBlock->laserData[j].angle *UINTCONVERT;
      tmpRange = mtBlock->laserData[j].rangeOne*UINTCONVERT;
      tmpXyz.intent  = mtBlock->laserData[j].intentTwo;
      calcXyz(mtBlock->dataID,tmpRange, tmpAngle, tmpXyz);

      if ( checkFrame(tmpAngle,tmpXyz.laserid,outCloud) ) rtn =1 ;

      s_lineData[s_lineCount] = tmpXyz;
      ++s_lineCount;
      if(s_lineCount>=LINE_POINT_COUNT) s_lineCount = LINE_POINT_COUNT-1;

    }
  }
  return rtn;
}

std::string SSBufferDec::s_lidarDataSavePath_;
SSBufferDec::SSBufferDec()
{
  reset();
}

SSBufferDec::~SSBufferDec()
{

}

int SSBufferDec::moveWriteIndex(int setpIndex)
{
  m_decBuf.bufSize += setpIndex;
  m_udpSize = setpIndex;
  m_udpCount++;
  m_decBuf.wrHead = (m_decBuf.wrHead+setpIndex)%DECODE_BUFFER_SIZE;
  return m_decBuf.bufSize ;
}

int SSBufferDec::moveReadIndex(int setpIndex)
{
  m_decBuf.bufSize -= setpIndex;
  m_decBuf.rdTail = (m_decBuf.rdTail+setpIndex)%DECODE_BUFFER_SIZE;
  return m_decBuf.bufSize ;
}

unsigned char *SSBufferDec::getWriteIndex()
{
  return m_decBuf.buffer+m_decBuf.wrHead ;
}

unsigned char *SSBufferDec::getReadIndex()
{
  return m_decBuf.buffer+m_decBuf.rdTail ;
}


int SSBufferDec::writeBuffer(unsigned char *data, int size)
{
  if(m_decBuf.bufSize+size >= DECODE_BUFFER_SIZE) return 0 ;

  memcpy(m_decBuf.buffer+m_decBuf.wrHead,data,size);
  m_decBuf.bufSize += size;
  m_decBuf.wrHead += size ;

  //  ROS_INFO_STREAM( "writeBuffer"
  //                  << " bufSize " << m_decBuf.bufSize
  //                  << " wrHead "<< m_decBuf.wrHead
  //                  << " rdTail " <<m_decBuf.rdTail);
  return size ;
}

int SSBufferDec::readPacket(rfans_driver::RfansPacket &pkt)
{
  int rtn = 0;
  if(m_decBuf.bufSize > 0 ) {
    pkt.data.resize(m_decBuf.bufSize);
    memcpy(&pkt.data[0], m_decBuf.buffer,  m_decBuf.bufSize);
    pkt.udpCount = m_udpCount;
    pkt.udpSize = m_udpSize;
    reset();
    rtn = 1;
  }
  return rtn ;
}

int SSBufferDec::size()
{
  return m_decBuf.bufSize;
}

int SSBufferDec::freeSize()
{
  return DECODE_BUFFER_SIZE-m_decBuf.bufSize;
}


static char s_tmpBuffer[DECODE_BUFFER_SIZE];

void SSBufferDec::bufferReverse()
{
  if (m_decBuf.bufSize > 0) {
    memcpy(s_tmpBuffer, m_decBuf.buffer + m_decBuf.rdTail, m_decBuf.bufSize);
    memcpy(m_decBuf.buffer, s_tmpBuffer, m_decBuf.bufSize);
    m_decBuf.rdTail = 0;
    m_decBuf.wrHead = m_decBuf.bufSize;
  } else {
    m_decBuf.wrHead = m_decBuf.bufSize = m_decBuf.rdTail = 0;
  }
  return;
}

void SSBufferDec::reset()
{
  //memset(&m_decBuf,0,sizeof(m_decBuf)) ;
  m_decBuf.bufSize = m_decBuf.wrHead = m_decBuf.rdTail = 0 ;
  memset(&m_packet,0,sizeof(m_packet)) ;
  m_status = eReady;
  s_preAngle =0 ;
  m_packetSize = 0;
  m_blockCout = 0 ;
  m_udpCount = 0 ;
}

int SSBufferDec::Depacket(rfans_driver::RfansPacket &inPack, sensor_msgs::PointCloud2 &outCloud , ros::Publisher &rosOut)
{
  int rtn =0, updateflag = 0;

  RFans_UDP32FRAMEV6G_S *tmpFrameV6;
  RFans_UDPFRAMEV5_S * tmpFrameV5;

  LOG_EVERY_N(INFO, 100) << "inPack.udpSize: " << inPack.udpSize << "; udpCount: " << inPack.udpCount;

  FILE *pOutFile;
  std::string lidarPkgName(s_lidarDataSavePath_ + "/lidar.dat");
  if(!(pOutFile = fopen(lidarPkgName.c_str(), "ab") ) ) {
    LOG(ERROR) << "Create file:" << lidarPkgName << " failed, errno:" << errno;
    exit(1);
  }
  DLOG(INFO) << "Create file:" << lidarPkgName << " successfully.";

  if( UDP_PACKET_SIZE_V5A == inPack.udpSize) {
    for( int i = 0 ; i < inPack.udpCount;i++) {
      tmpFrameV5 = (RFans_UDPFRAMEV5_S*)(&inPack.data[0] + i*inPack.udpSize);
      if( processFrameV5(tmpFrameV5,outCloud) ){
        rosOut.publish(outCloud);
        SSBufferDec::ResetPointCloud2(outCloud);
      }
    }
  }
  else
  if(UDP_PACKET_SIZE_V6G == inPack.udpSize) {
    for( int i = 0 ; i < inPack.udpCount;i++) {
      tmpFrameV6 = (RFans_UDP32FRAMEV6G_S*)(&inPack.data[0] + i*inPack.udpSize);
      DLOG(INFO) << std::fixed << "GPS Time stamp[" << i << "]: " << tmpFrameV6->gpsTimestamp / 1000000 << " s";

      const double pktDaySecond = public_tools::PublicTools::getDaySecond(ros::Time::now().toSec(), tmpFrameV6->gpsTimestamp / 1000000.);
      (void)fwrite(&pktDaySecond, sizeof(pktDaySecond), 1, pOutFile);
      (void)fwrite(tmpFrameV6, UDP_PACKET_SIZE_V6G, 1, pOutFile);

      if( processFrameV6G(tmpFrameV6,outCloud) ) {
        rosOut.publish(outCloud);
        SSBufferDec::ResetPointCloud2(outCloud);
      }
    }
  }
  else {
    LOG(WARNING) << "inPack.udpSize " <<inPack.udpSize;
  }

  fclose(pOutFile);
  return rtn ;
}

void SSBufferDec::SetLidaDataSavePath(const std::string &_lidarSavePath) {
  LOG(INFO) << __FUNCTION__ << " start.";
  s_lidarDataSavePath_ = _lidarSavePath;
}

void SSBufferDec::InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) {
  static const size_t DataSize = sizeof(rfans_driver::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S ) * sizeof(RFANS_XYZ_S) *RFANS_LASER_COUNT;
  initCloud.data.clear();
  initCloud.data.resize( DataSize); //point data

  initCloud.is_bigendian = false ;//false;      //stream foramt
  initCloud.fields.resize(6);          //line format
  initCloud.is_dense = false;

  int tmpOffset = 0 ;
  for(int i=0; i < initCloud.fields.size() ;i++) {
    switch(i) { //value type
    case 0:
      initCloud.fields[i].name = "x" ;
      initCloud.fields[i].datatype = 7u;
      break;
    case 1:
      initCloud.fields[i].name = "y" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 2:
      initCloud.fields[i].name = "z" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 3:
      initCloud.fields[i].name = "intensity" ;
      initCloud.fields[i].datatype = 7u;//2u;
      tmpOffset += 4;
      break;
    case 4:
      initCloud.fields[i].name = "laserid" ;
      initCloud.fields[i].datatype = 2u;
      tmpOffset += 1;
    case 5:
      initCloud.fields[i].name = "timeflag" ;
      initCloud.fields[i].datatype = 8u;
      tmpOffset += 8;
      break;
    }
    initCloud.fields[i].offset = tmpOffset ;      //value offset
    initCloud.fields[i].count = 1 ;
  }
  initCloud.height = 1;
  initCloud.point_step = sizeof(RFANS_XYZ_S);
  initCloud.row_step = DataSize ;
  initCloud.width = 0 ;


  //node name
  std::string node_name = ros::this_node::getName();

  std::string frame_id_str = "/world";
  std::string frame_id_path = node_name + "/frame_id";
  ros::param::get(frame_id_path,frame_id_str);

  initCloud.header.frame_id = frame_id_str;

  s_lastAngle = 0 ;
  s_lineData.resize(LINE_POINT_COUNT);
  s_lineCount = 0 ;

  s_hangles.clear();
  s_vangles.clear();
  //logFile = fopen("/home/liyp/test.log","w+");
//  //device  ip name
//  std::string vangles_str = "\
//      -25,   -22,   -19,   -16,\
//      -13,   -11,    -9,    -7,\
//      -5.5,  -4.5,  -3.5,  -2.9,\
//      -2.45,  -2.1, -1.75,  -1.4,\
//      -1.05,  -0.7, -0.35,     0,\
//      0.35,   0.7,  1.05,   1.4,\
//      2.5,   3.5,   4.5,     6,\
//      8,    10,    12,    15, ";

//      std::string vangle_path = node_name + "/laser_vangle";
//  ros::param::get(vangle_path,vangles_str);
//  strToList(s_vangles,vangles_str);

//  std::string hangles_str = "\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0,\
//      0,  0,  0,  0, ";

//  std::string hangle_path = node_name + "/laser_hangle";
//  ros::param::get(hangle_path, hangles_str);
//  ROS_INFO("hable %s\n",hangles_str.c_str() );
//  strToList(s_hangles, hangles_str);
}

void SSBufferDec::ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) {
  initCloud.width = 0;
}

void SSBufferDec::SetAngleDuration(float value)
{
  if(value <10  || value > 360)
    return ;
  s_angle_duration = value;
}


