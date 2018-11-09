#ifndef __SAVE_IMAGE_TASK_H__
#define __SAVE_IMAGE_TASK_H__

#include <opencv2/opencv.hpp>

class SaveImageTask {
public:
    SaveImageTask(double _header, size_t _camera1stIP, const cv::Mat &_image);
    ~SaveImageTask();
    void doit();


private:
    double time_;
    size_t camera1stIP_;
    cv::Mat image_;
};

#endif

