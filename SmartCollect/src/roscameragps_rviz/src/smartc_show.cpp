#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <FlyCapture2.h>
#include "lock.h"
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <vector>

//--rviz-test
#include "myviz.h"
#include <QApplication>
#include <QWidget>
#include <QDialog>
#include "Dialog.h"

//--rviz-msg
#include<std_msgs/Int64.h>
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>


namespace rviz {
class Display;
class RenderPanel;
class VisualizationManager;
}

using namespace std;
using namespace cv;
using namespace FlyCapture2;

// global control
bool close_flag   = false;
int  save_control = 0;


int main(int argc, char** argv)
{
    // google::InitGoogleLogging(argv[0]);
    QApplication app(argc, argv);

    MyViz* myviz = new MyViz(argc, argv);
    myviz->show();
    // myviz->setDisabled(true);

    // TODO: should set IP before ROS thread
    // sleep(10);

    app.exec();
    return 0;
}
