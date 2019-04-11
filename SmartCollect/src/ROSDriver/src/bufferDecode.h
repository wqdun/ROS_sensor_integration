#ifndef _BUFFER_DECODER_H_
#define _BUFFER_DECODER_H_
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "rfans_driver/RfansPacket.h"
#include "ssFrameLib.h"
#include <ros/ros.h>
#include "../../sc_lib_public_tools/src/public_tools.h"

typedef enum {
  eReady,
  eSearch,
  eReadPackData,
  eReadFinish
}DEC_STSTUS_E ;

class SSBufferDec {
public:
  SSBufferDec();
  ~SSBufferDec();
  int moveWriteIndex(int setpIndex);
  int moveReadIndex(int setpIndex);
  unsigned char * getWriteIndex();
  unsigned char * getReadIndex();

  int writeBuffer(unsigned char *data, int size);

  int readPacket(rfans_driver::RfansPacket &pkt);
  //int readPacketStream(rfans_driver::RfansPacket &pkt);
  int size();
  int freeSize();
  void reset();

  int getUdpCount() { return m_udpCount;}
  int getUdpSize() {return m_udpSize;}

  static int Depacket(rfans_driver::RfansPacket &inPack, sensor_msgs::PointCloud2 &outCloud , ros::Publisher &rosOut);
  static void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) ;
  static void SetLidaDataSavePath(const std::string &_lidarSavePath);
  static void ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) ;

  static void SetAngleDuration(float value);
private:  //int readPacket(std::vector<SCDRFANS_BLOCK_S> &outBlocks);
  int readBuffer() ;
  int readBufferSteam() ;
  int ouputPacket(rfans_driver::RfansPacket &pkt);
  void bufferReverse();
private:
  UDP_DECBUFFER_S m_decBuf;
  DEC_STSTUS_E m_status ;
  rfans_driver::RfansPacket m_packet ;
  int m_packetSize ;
  int m_blockCout ;
  float s_preAngle ;
  int m_udpSize;
  int m_udpCount;
  static std::string s_lidarDataSavePath_;
};


#endif
