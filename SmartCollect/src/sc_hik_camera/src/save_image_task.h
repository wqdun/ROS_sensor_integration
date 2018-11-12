#ifndef __SAVE_IMAGE_TASK_H__
#define __SAVE_IMAGE_TASK_H__

#include <opencv2/opencv.hpp>

class SaveImageTask {
public:
    SaveImageTask(double _header, const std::string &_cameraIP, const cv::Mat &_image);
    ~SaveImageTask();
    void doit();


private:
    double time_;
    std::string cameraIP_;
    cv::Mat image_;
};

#endif

