#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    Mat image(1200 , 1920 , CV_8UC3 , Scalar::all(0) );
    cv::Mat imageResized;
    cout << image.cols << ":" << image.rows;
    cv::resize(image, imageResized, cv::Size(image.cols / 10, image.rows / 10));
    //waitKey(0);
    return 0;
}

// g++ -std=c++11 tmp.cc /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_imgcodecs.so /usr/local/lib/libopencv_imgproc.so
