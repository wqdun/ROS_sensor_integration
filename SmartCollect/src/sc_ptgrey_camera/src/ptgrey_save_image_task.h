#ifndef __PTGREY_SAVE_IMAGE_TASK_H__
#define __PTGREY_SAVE_IMAGE_TASK_H__

#include <glog/logging.h>

#include "single_ptgrey_camera.h"
#include "ptgrey_camera_manager.h"

class PtgreySaveImageTask {
public:
    PtgreySaveImageTask(FlyCapture2::Image *pImage, const std::string &_picName);
    ~PtgreySaveImageTask();

    void doit();


private:
    std::string picFile_;
    FlyCapture2::Error flyError_;
    FlyCapture2::Image flyImage_;
    FlyCapture2::Image convertedFlyImage_;
    FlyCapture2::JPEGOption picFormatOption_;
};

#endif

