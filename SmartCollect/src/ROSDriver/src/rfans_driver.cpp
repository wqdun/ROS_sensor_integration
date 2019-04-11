/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "rfans_driver.h"
#include "rfans_driver/RfansCommand.h"

// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

namespace rfans_driver {

static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const int RFANS_PACKET_NUM = 1024 ;

static Rfans_Driver *s_this = NULL;

static char s_simuFileName[] = "test.imp";

/** @brief Rfans Command Handle */
bool CommandHandle(rfans_driver::RfansCommand::Request  &req,
                   rfans_driver::RfansCommand::Response &res)
{
  res.status = 1;

  ROS_INFO("request: cmd= %d , speed = %d Hz", (int)req.cmd, (int)req.speed);
  ROS_INFO("sending back response: [%d]", (int)res.status);

  DEB_PROGRM_S tmpProg ;
  tmpProg.cmdstat = (DEB_CMD_E)req.cmd;
  tmpProg.dataFormat = eFormatCalcData;
  tmpProg.scnSpeed = req.speed;
  if(s_this) {
    s_this->prog_Set(tmpProg);
  }
  return true;
}

Rfans_Driver::Rfans_Driver(ros::NodeHandle mtNode, int type)
{
  bool ok = true;

  std::string filename = s_simuFileName;
  //node name
  std::string node_name = ros::this_node::getName();

  //command name
  std::string command_name = "rfans_control";
  std::string command_path = node_name + "/control_name";
  ros::param::get(command_path,command_name);
  command_path = "rfans_driver/" + command_name;
  m_svr = mtNode.advertiseService(command_path, CommandHandle);

  m_LidarDataSavePath_ = "/tmp/";
  ros::param::get(node_name + "/data_save_path", m_LidarDataSavePath_);
  LOG(INFO) << "m_LidarDataSavePath_: " << m_LidarDataSavePath_;

  //advertise name
  std::string advertise_name = "rfans_packets";
  std::string advertise_path = node_name + "/advertise_name";
  ros::param::get(advertise_path,advertise_name);
  advertise_path = "rfans_driver/" + advertise_name;
  //ROS_INFO("%s : advertise name %s : %s",node_name.c_str(), advertise_name.c_str(), advertise_path.c_str() );


  //device  ip name
  std::string device_ip = DEVICE_IP_STRING;
  std::string ip_path = node_name + "/device_ip";
  ros::param::get(ip_path,device_ip);
  //ROS_INFO("%s : device_ip name %s : %s",node_name.c_str(), device_ip.c_str(), ip_path.c_str() );

  //device port name
  int dataport = DEVICE_PORT_NUMBER;
  std::string port_path = node_name + "/device_port";
  ros::param::get(port_path,dataport);
  //ROS_INFO("%s : device_port  %d : %s",node_name.c_str(), dataport, port_path.c_str() );

  //driver init
  m_output = mtNode.advertise<rfans_driver::RfansPacket>(advertise_path, RFANS_PACKET_NUM);
  if(type) {

    ros::param::get("/rfans_driver/playfile",filename);
    //ROS_INFO("Rfans_Driver:  palyback %s",filename.c_str());
    m_devapi = new rfans_driver::SSFileAPI(filename.c_str());

  } else {
    m_devapi = new rfans_driver::IOSocketAPI(device_ip,dataport,dataport);
    //ROS_INFO("Rfans_Driver:  worker ip %s  port %d ", device_ip.c_str(), dataport);
  }

  //device port name
  int scnSpeed = 10;
  std::string rps_path = node_name + "/rps";
  ok = ros::param::get(rps_path,scnSpeed);

  // device rps setup
  if(ok) {
    DEB_PROGRM_S tmpProg ;
    tmpProg.cmdstat = eDevCmdWork;
    tmpProg.dataFormat = eFormatCalcData;
    tmpProg.scnSpeed =  scnSpeed;
    prog_Set(tmpProg);
  }

  s_this = this ;
}

Rfans_Driver::~Rfans_Driver()
{
  if(m_devapi) delete m_devapi;
}

/** @brief Rfnas Driver Core */
int Rfans_Driver::spinOnce()
{
  int rtn = 0 ;

  m_devapi->revPacket(tmpPacket);

  rtn = m_devapi->getPacket(tmpPacket);
  if(rtn > 0) {
    m_output.publish(tmpPacket) ;
  }
  return rtn ;
}

/** @brief control the device
         *  @param .parameters
         */
int Rfans_Driver::prog_Set(DEB_PROGRM_S &program)
{
  unsigned int tmpData = 0;

  switch (program.dataFormat) {
  case eFormatCalcData:
    tmpData |= CMD_CALC_DATA;
    break;
  case eFormatDebugData:
    tmpData |= CMD_DEBUG_DATA;
    break;
  }
  m_devapi->HW_WRREG(0, REG_DATA_TRANS, tmpData);
  //===============================================================
  tmpData = 0;
  switch (program.scnSpeed) {
  case ANGLE_SPEED_10HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_10HZ;
    break;
  case ANGLE_SPEED_20HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_20HZ;
    break;
  case ANGLE_SPEED_5HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_5HZ;
    break;
  default:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_5HZ;
    break;
  }

  tmpData |= CMD_LASER_ENABLE;
  switch (program.cmdstat) {
  case eDevCmdWork:
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case eDevCmdIdle:
    tmpData = CMD_RCV_CLOSE;
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case eDevCmdAsk:
    break;
  default:
    break;
  }

  return 0;

}

//end prog_Set

} //rfans_driver namespace
