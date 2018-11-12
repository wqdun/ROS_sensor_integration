#include "save_image_task.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

SaveImageTask::SaveImageTask(double _header, const std::string &_cameraIP, const cv::Mat &_image) {
    LOG(INFO) << __FUNCTION__ << " start.";
    time_ = _header;
    cameraIP_ = _cameraIP;
    image_ = _image;
}

SaveImageTask::~SaveImageTask() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

void SaveImageTask::doit() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string picFile("/tmp/" + std::to_string(time_) + "_" + cameraIP_ + ".jpg");

    try {
        cv::imwrite(picFile, image_);
        LOG(INFO) << "Save " << picFile;
    }
    catch(cv::Exception &ex) {
        LOG(ERROR) << ex.what();
    }
}

