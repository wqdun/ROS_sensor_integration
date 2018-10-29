#include "image_task.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
#include "../../../sc_lib_public_tools/src/tools_no_ros.h"

ImageTask::ImageTask(const std::string &_picName, const cv::Mat &_image) {
    LOG(INFO) << __FUNCTION__ << " start.";
    picName_ = _picName;
    image_ = _image;
}

ImageTask::~ImageTask() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

void ImageTask::doit() {
    LOG(INFO) << __FUNCTION__ << " start.";

    LOG(INFO) << "public_tools::ToolsNoRos::isOK_: " << public_tools::ToolsNoRos::isOK_;

    try {
        cv::imwrite(picName_, image_);
        LOG(INFO) << "Save " << picName_;
    }
    catch(cv::Exception &ex) {
        LOG(ERROR) << ex.what();
    }

    // usleep(200000);
}

