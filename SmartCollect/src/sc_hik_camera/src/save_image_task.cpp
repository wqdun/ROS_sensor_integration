#include "save_image_task.h"

SaveImageTask::SaveImageTask(const cv::Mat &_image, const MV_FRAME_OUT_INFO_EX &_frameInfo, const std::string &_picFile) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    image_ = _image;
    frameInfo_ = _frameInfo;
    picFile_ = _picFile;
}

SaveImageTask::~SaveImageTask() {
    DLOG(INFO) << __FUNCTION__ << " start.";
}

void SaveImageTask::doit() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    cv::imwrite(picFile_, image_);
    DLOG(INFO) << "Save " << picFile_;
}

