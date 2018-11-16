#ifndef __SAVE_IMAGE_TASK_H__
#define __SAVE_IMAGE_TASK_H__

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>
#include "single_camera.h"

class SaveImageTask {
public:
    SaveImageTask(SingleCamera *_pSingleCamera, MV_FRAME_OUT_INFO_EX _frameInfo, double _header, unsigned char *_pData);
    ~SaveImageTask();

    void doit();
    void SaveImageUsingTurboJpeg(const std::string &picFile);


private:
    SingleCamera *pSingleCamera_;
    MV_FRAME_OUT_INFO_EX frameInfo_;
    void *_handle;
    double time_;
    std::string cameraIP_;
    size_t frameNum_;
    cv::Mat image_;
    unsigned char *pData_;

};

#endif

