#ifndef __SAVE_IMAGE_TASK_H__
#define __SAVE_IMAGE_TASK_H__

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>
#include "single_camera.h"

class SaveImageTask {
public:
    SaveImageTask(const cv::Mat &_image, const MV_FRAME_OUT_INFO_EX &_frameInfo, const std::string &_picFile);
    ~SaveImageTask();

    void doit();


private:
    cv::Mat image_;
    MV_FRAME_OUT_INFO_EX frameInfo_;
    std::string picFile_;
};

#endif

