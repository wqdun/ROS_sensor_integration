/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include "rfans_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rfans_driver");
  ros::NodeHandle node;
  rfans_driver::Rfans_Driver * p_rfansDriver;
  if(argc >1 ) {
    p_rfansDriver = new rfans_driver::Rfans_Driver(node,1);
  } else {
    p_rfansDriver = new rfans_driver::Rfans_Driver(node,0);
  }


  while( ros::ok() ) {  //loop until shut down
    p_rfansDriver->spinOnce();
    ros::spinOnce();
  }
  return 0;
}
