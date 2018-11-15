#ifndef __SAVE_IMAGE_TASK_H__
#define __SAVE_IMAGE_TASK_H__

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>

class SaveImageTask {
public:
    SaveImageTask(double _header, const std::string &_cameraIP, size_t _frameNum, const cv::Mat &_image, unsigned char *_pData);
    ~SaveImageTask();

    void doit();
    void SaveImageUsingTurboJpeg(const std::string &picFile);


private:
    double time_;
    std::string cameraIP_;
    size_t frameNum_;
    cv::Mat image_;
    unsigned char *pData_;
};

#endif

