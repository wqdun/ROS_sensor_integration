#include "save_image_task.h"

SaveImageTask::SaveImageTask(const cv::Mat &_image, const MV_FRAME_OUT_INFO_EX &_frameInfo, const std::string &_picFile) {
    LOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";
    image_ = _image;
    frameInfo_ = _frameInfo;
    picFile_ = _picFile;
    LOG_EVERY_N(INFO, 100) << __FUNCTION__ << " end.";
}

SaveImageTask::~SaveImageTask() {
    DLOG(INFO) << __FUNCTION__ << " start.";
}

void SaveImageTask::doit() {
    LOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";
    cv::imwrite(picFile_, image_);
    LOG_EVERY_N(INFO, 100) << "Save " << picFile_;
}

