#include "sub_python.h"

PythonSubscriber::PythonSubscriber() {

}

PythonSubscriber::~PythonSubscriber() {}

void PythonSubscriber::run() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber subChatter = nh.subscribe("chatter", 10, &PythonSubscriber::ChatterCB, this);
    ros::Subscriber subPoint = nh.subscribe("point", 10, &PythonSubscriber::PointCB, this);

    ros::Rate rate(2);
    while(ros::ok() ) {

        ros::spinOnce();
        rate.sleep();


    }
}

void PythonSubscriber::ChatterCB(const std_msgs::String::ConstPtr& pChatter) {
    LOG(INFO) << __FUNCTION__ << " start: " << pChatter->data;

    return;
}


void PythonSubscriber::PointCB(const sc_msgs::Point2D::ConstPtr& pPoint2D) {
    LOG(INFO) << __FUNCTION__ << " start: " << pPoint2D->x;

    return;
}
