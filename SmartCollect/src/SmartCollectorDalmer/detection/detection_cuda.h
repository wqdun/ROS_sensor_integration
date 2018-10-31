#ifndef __DETECTION_CUDA__
#define __DETECTION_CUDA__ 
#include <caffe/caffe.hpp>
#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef USE_CUDA
using namespace std; //avoid linking error within opencv gpu module
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudawarping.hpp>
#endif  //USE_CUDA
#endif  // USE_OPENCV

#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

//#ifdef USE_OPENCV
using namespace caffe;  // NOLINT(build/namespaces)

typedef std::chrono::high_resolution_clock Clock;

class Detector 
{
 public:
  Detector(const string& model_file,
           const string& weights_file,
           const string& mean_file,
           const string& mean_value);

  std::vector<vector<float> > Detect(const cv::Mat& img);

 private:
  void SetMean(const string& mean_file, const string& mean_value);

#ifdef USE_CUDA
  void WrapInputLayer(std::vector<cv::cuda::GpuMat>* input_channels);
  
  void Preprocess(const cv::Mat& img, std::vector<cv::cuda::GpuMat>* input_channels);
#else
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);
  
  void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);
#endif

 private:
  std::shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
#ifdef USE_CUDA
  cv::cuda::GpuMat mean_;
#else
  cv::Mat mean_;
#endif
};

//#endif 



#endif 
