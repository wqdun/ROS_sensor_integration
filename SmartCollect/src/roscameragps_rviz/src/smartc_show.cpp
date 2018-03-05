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

#include <std_msgs/Int64.h>
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

int main(int argc, char** argv) {
    FLAGS_log_dir = "/opt/smartc/log";
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "argc: " << argc << "; argv[0]: " << argv[0];

    ros::init(argc, argv, "myviz");

    MyViz myviz(argc, argv);
    myviz.show();

    QApplication app(argc, argv);
    app.exec();
    return 0;
}
