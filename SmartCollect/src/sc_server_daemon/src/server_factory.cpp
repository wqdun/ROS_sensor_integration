#include "server_factory.h"
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "server_5651.h"
#include "server_a1.h"

void ServerFactory::SetSensorType(const std::string &sensorTypeYaml) {
    LOG(INFO) << __FUNCTION__ << " start.";

    cv::FileStorage fs(sensorTypeYaml, cv::FileStorage::READ);
    fs["IMU"] >> __imuType_;
    LOG(INFO) << "Get IMU type: " << __imuType_;

    fs.release();
}

BaseServer *ServerFactory::CreateServer() {
    // if ("a1" == __imuType_) {
    //     return new ServerA1();
    // }

    if ("5651" == __imuType_) {
        return new Server5651();
    }

    LOG(ERROR) << "Unsupported type: " << __imuType_;
    exit(1);
}
