#include "save_image_task.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

SaveImageTask::SaveImageTask(double _header, size_t _camera1stIP, const cv::Mat &_image) {
    LOG(INFO) << __FUNCTION__ << " start.";
    time_ = _header;
    camera1stIP_ = _camera1stIP;
    image_ = _image;
}

SaveImageTask::~SaveImageTask() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

void SaveImageTask::doit() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string picFile = "/tmp/" + std::to_string(camera1stIP_) + "_" + std::to_string(time_) + ".jpg";

    try {
        cv::imwrite(picFile, image_);
        LOG(INFO) << "Save " << picFile;
    }
    catch(cv::Exception &ex) {
        LOG(ERROR) << ex.what();
    }
}

