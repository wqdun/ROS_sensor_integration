/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef __RFANS_IOAPI_H
#define __RFANS_IOAPI_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "ssFrameLib.h"

class SSBufferDec;
namespace rfans_driver
{
/** @brief Rfans IOAPI base class */
class IOAPI
{
public:
  IOAPI() ;
  ~IOAPI();

  /** @brief Read one Rfans packet.
                 *
                 * @param packet from Rfans
                 *
                 * @returns 0 if successful,
                 *          -1 if end of file
                 *          > 0 if incomplete packet (is this possible?)
                 */
  /** @brief Write . */
  virtual int write(unsigned char *data, int size) = 0;

  /** @brief read . */
  virtual int read(unsigned char *data, int size) = 0;

  /** @brief reset . */
  virtual int reset() = 0;

  virtual int HW_WRREG(int flag, int regAddress, unsigned int regData) ;

  virtual int revPacket(rfans_driver::RfansPacket &pkt) ;
  virtual int getPacket(rfans_driver::RfansPacket &pkt) ;
protected:
  SSBufferDec *m_bufferPro ;
};

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////
/** @brief Rfans IOAPI from socket. */
class IOSocketAPI : public IOAPI
{
public:
  IOSocketAPI( std::string ipstr = DEVICE_IP_STRING,
               uint16_t devport   = DEVICE_PORT_NUMBER,
               int16_t pcport     = PC_PORT_NUMBER);
  virtual ~IOSocketAPI();
  virtual int write(unsigned char *data, int size) ;
  virtual int read(unsigned char *data, int size) ;
  virtual int reset() ;
private:
  int m_sockfd;
  in_addr devip_;
  sockaddr_in m_devaddr;

  ros::NodeHandle private_nh_;
  uint16_t m_devport_;
  uint16_t m_pcport_;
  std::string devip_str_;
};

////////////////////////////////////////////////////////////////////////
// SSFileAP class implementation



static const size_t DATA_FILE_SIZE = 0x8000000; //128M Bytes
static const char s_dataPath[] = "./";

////////////////////////////////////////////////////////////////////////
// SSFileAP class implementation
////////////////////////////////////////////////////////////////////////
class SSFileAPI: public IOAPI{


public:
  SSFileAPI(const char *fileName=0);
  ~SSFileAPI();

  int create_file(int flag=0);


  virtual int write(unsigned char *data, int size);

  virtual int read(unsigned char *data, int size);

  virtual int reset();

  int printf(char *msgStr, int size) ;

  int outputFile(std::vector<SCDRFANS_BLOCK_S> &pointCloud, std::vector<RFANS_XYZ_S> &outXyzBlocks, int flag=1);
  int outputFile(sensor_msgs::PointCloud2 &initCloud, int flag=1);
private:
  FILE *s_rawFile ;
  SCDRFANS_BLOCK_S *m_blocks;
};

} // rfans_driver namespace

#endif //__RFANS_IOAPI_H
