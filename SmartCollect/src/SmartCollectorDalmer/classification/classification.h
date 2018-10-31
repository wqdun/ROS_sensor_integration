#ifndef __CLASSIFICATION__
#define __CLASSIFICATION__
#include <caffe/caffe.hpp> 
#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif  // USE_OPENCV

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef USE_OPENCV
using namespace caffe;
using std::string; 

typedef std::pair<string, float> Prediction;

/*-------------------------------------
@function: 
           Classify  分类函数
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           Mat img 
@output: 
           std::vector<vector<float> >
-----------------------------------------*/
class Classifier 
{
 public:
  Classifier(const string& model_file,
             const string& trained_file,
             const string& mean_file,
             const string& label_file);

  std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);

 private:
  void SetMean(const string& mean_file);

  std::vector<float> Predict(const cv::Mat& img);

  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

  void Preprocess(const cv::Mat& img,
                  std::vector<cv::Mat>* input_channels);

 private:
  std::shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
  std::vector<string> labels_;
};

#endif  // USE_OPENCV
#endif 
