/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _RFANS_DRIVER_H_
#define _RFANS_DRIVER_H_
#include <ros/ros.h>
#include "ioapi.h"


namespace rfans_driver
{
class Rfans_Driver
{
public:
  Rfans_Driver(ros::NodeHandle mtNode, int type=0);
  ~Rfans_Driver();

  int spinOnce();
  int prog_Set(DEB_PROGRM_S &program);
private:
  rfans_driver::IOAPI *m_devapi;
  ros::Publisher m_output;
  ros::ServiceServer m_svr ;
  rfans_driver::RfansPacket tmpPacket;
  std::string m_LidarDataSavePath_;
};

}

#endif //_RFANS_DRIVER_H_
