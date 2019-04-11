#include "ptgrey_save_image_task.h"

PtgreySaveImageTask::PtgreySaveImageTask(FlyCapture2::Image *pImage, const std::string &_picName) {
    LOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";
    picFile_ = _picName;
    flyImage_ = *pImage;
}

PtgreySaveImageTask::~PtgreySaveImageTask() {
    DLOG(INFO) << __FUNCTION__ << " start.";
}

void PtgreySaveImageTask::doit() {
    LOG_EVERY_N(INFO, 100) << __FUNCTION__ << " start.";

    flyError_ = flyImage_.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &convertedFlyImage_);
    if(flyError_ != FlyCapture2::PGRERROR_OK) {
        LOG(WARNING) << "Failed to Convert flyImage_ " << picFile_;
        PtgreyCameraManager::LogErrorTrace(flyError_);
        return;
    }

    LOG(INFO) << "I am gonna save " << picFile_;
    flyError_ = convertedFlyImage_.Save(picFile_.c_str(), &picFormatOption_);
    if(flyError_ != FlyCapture2::PGRERROR_OK) {
        LOG(WARNING) << "Failed to save " << picFile_;
        PtgreyCameraManager::LogErrorTrace(flyError_);
        return;
    }
    LOG_EVERY_N(INFO, 100) << "Save " << picFile_;
}




