#ifndef __SUB_PYTHON_H
#define __SUB_PYTHON_H

#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sc_msgs/Point2D.h>

class PythonSubscriber {
public:
    PythonSubscriber();
    ~PythonSubscriber();
    void run();


private:
    void ChatterCB(const std_msgs::String::ConstPtr& pChatter);
    void PointCB(const sc_msgs::Point2D::ConstPtr& pPoint2D);

};

#endif // __SUB_PYTHON_H
