#ifndef __IMAGE_TASK_H__
#define __IMAGE_TASK_H__

#include <opencv2/opencv.hpp>

class ImageTask {
public:
    ImageTask(const std::string &_picName, const cv::Mat &_image);
    ~ImageTask();
    void doit();


private:
    std::string picName_;
    cv::Mat image_;
};

#endif

