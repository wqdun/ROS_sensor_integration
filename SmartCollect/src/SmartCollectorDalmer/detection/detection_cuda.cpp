#include "detection_cuda.h"

/*--------------------------
@function: 
           Detector 构造函数
@author: 
           liuyusen 
@date: 
           2018-01-11
----------------------------*/
Detector::Detector(const string& model_file,
                   const string& weights_file,
                   const string& mean_file,
                   const string& mean_value) 
{
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else
  Caffe::set_mode(Caffe::GPU);
#endif

  /* Load the network. */
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(weights_file);

  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";
  
  /* Load the binaryproto mean file. */
  SetMean(mean_file, mean_value);
}
/*-------------------------------------
@function: 
           Detector 检测函数
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           Mat img 
@output: 
           std::vector<vector<float> >
-----------------------------------------*/

std::vector<vector<float> > Detector::Detect(const cv::Mat& img) 
{
  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = img.channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = img.size();
  input_layer->Reshape(1, num_channels_, input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

#ifdef USE_CUDA
  std::vector<cv::cuda::GpuMat> input_channels_gpu;
  WrapInputLayer(&input_channels_gpu);
  Preprocess(img, &input_channels_gpu);
#else
  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);
  Preprocess(img, &input_channels);
#endif

  net_->Forward();

  /* Copy the output layer to a std::vector */
  Blob<float>* result_blob = net_->output_blobs()[0];
  const float* result = result_blob->cpu_data();
  const int num_det = result_blob->height();
  vector<vector<float> > detections;
  for (int k = 0; k < num_det; ++k) {
    if (result[0] == -1) {
      // Skip invalid detection.
      result += 7;
      continue;
    }
    vector<float> detection(result, result + 7);
    detections.push_back(detection);
    result += 7;
  }
  return detections;
}
/*-------------------------------------
@function: 
           Detector 设置均值变量mean_
		   Load the mean file in binaryproto format
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           const string& mean_file
		   const string& mean_value
@output: 
           void 
-----------------------------------------*/
void Detector::SetMean(const string& mean_file, const string& mean_value) 
{
  Blob<float>* input_layer = net_->input_blobs()[0];
  this->num_channels_ = input_layer->channels();
  this->input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
  cv::Scalar channel_mean;
  if (!mean_file.empty()) {
    CHECK(mean_value.empty()) <<
      "Cannot specify mean_file and mean_value at the same time";
    BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

    /* Convert from BlobProto to Blob<float> */
    Blob<float> mean_blob;
    mean_blob.FromProto(blob_proto);
    CHECK_EQ(mean_blob.channels(), num_channels_)
      << "Number of channels of mean file doesn't match input layer.";

    /* The format of the mean file is planar 32-bit float BGR or grayscale. */
    std::vector<cv::Mat> channels;
    float* data = mean_blob.mutable_cpu_data();
    for (int i = 0; i < num_channels_; ++i) {
      /* Extract an individual channel. */
      cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
      channels.push_back(channel);
      data += mean_blob.height() * mean_blob.width();
    }

    /* Merge the separate channels into a single image. */
    cv::Mat mean;
    cv::merge(channels, mean);

    /* Compute the global mean pixel value and create a mean image
     * filled with this value. */
    channel_mean = cv::mean(mean);
#ifdef USE_CUDA
    mean_ = cv::cuda::GpuMat(input_geometry_, mean.type(), channel_mean);
#else
    mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
#endif
  }
  if (!mean_value.empty()) {
    CHECK(mean_file.empty()) <<
      "Cannot specify mean_file and mean_value at the same time";
    stringstream ss(mean_value);
    vector<float> values;
    string item;
    while (getline(ss, item, ',')) {
      float value = std::atof(item.c_str());
      values.push_back(value);
    }
    CHECK(values.size() == 1 || static_cast<int>(values.size()) == num_channels_) <<
      "Specify either 1 mean_value or as many as channels: " << num_channels_;
#ifdef USE_CUDA
    std::vector<cv::cuda::GpuMat> channels;
    for (int i = 0; i < num_channels_; ++i) 
    {
      /* Extract an individual channel. */
      cv::cuda::GpuMat channel(input_geometry_.height, input_geometry_.width, CV_32FC1, cv::Scalar(values[i]));
      channels.push_back(channel);
    }
    cv::cuda::merge(channels, this->mean_); 
#else
    std::vector<cv::Mat> channels;
    for (int i = 0; i < num_channels_; ++i) 
    {
      /* Extract an individual channel. */
      cv::Mat channel(input_geometry_.height, input_geometry_.width, CV_32FC1, cv::Scalar(values[i]));
      channels.push_back(channel);
    }
    cv::merge(channels, this->mean_);
#endif
  }
}
/*-------------------------------------
@function: 
           Detector 修改input_channels
		   WrapInputLayer
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           std::vector<cv::Mat>* input_channels
@output: 
           void 
-----------------------------------------*/
#ifdef USE_CUDA
void Detector::WrapInputLayer(std::vector<cv::cuda::GpuMat>* input_channels) 
{
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_gpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::cuda::GpuMat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}
/*-------------------------------------
@function: 
           Detector 求取归一化后的input_channels
		   Preprocess 图像预处理
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           const cv::Mat& img
		   std::vector<cv::Mat>* input_channels
@output: 
           void 
-----------------------------------------*/
void Detector::Preprocess(const cv::Mat& img, std::vector<cv::cuda::GpuMat>* input_channels) 
{
  cv::cuda::GpuMat sample;
  /* Convert the input image to the input image format of the network. */
  if (img.channels() == num_channels_) 
  {
    sample.upload(img);
  } 
  else 
  {
    cv::cuda::GpuMat g_img;
    g_img.upload(img);
    if (g_img.channels() == 3 && num_channels_ == 1)
      cv::cuda::cvtColor(g_img, sample, cv::COLOR_BGR2GRAY);
    else if (g_img.channels() == 4 && num_channels_ == 1)
      cv::cuda::cvtColor(g_img, sample, cv::COLOR_BGRA2GRAY);
    else if (g_img.channels() == 4 && num_channels_ == 3)
      cv::cuda::cvtColor(g_img, sample, cv::COLOR_BGRA2BGR);
    else if (g_img.channels() == 1 && num_channels_ == 3)
      cv::cuda::cvtColor(g_img, sample, cv::COLOR_GRAY2BGR);
  }
  
  cv::cuda::GpuMat sample_resized;
  if (sample.size() != input_geometry_)
    cv::cuda::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::cuda::GpuMat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);
  
  //dynamic mean dimensions
  if (this->mean_.size() != img.size()) // operation should only occur on transit of image resolutions
  {
    cv::cuda::GpuMat meanDynamic;
    cv::cuda::resize(this->mean_, meanDynamic, img.size()); 
    this->mean_ = meanDynamic;
  }
  
  cv::cuda::GpuMat sample_normalized;
  cv::cuda::subtract(sample_float, this->mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the Mat
   * objects in input_channels. */
  cv::cuda::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data) == net_->input_blobs()[0]->gpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
#else 
/*-------------------------------------
@function: 
           Detector 修改input_channels
		   WrapInputLayer NOT CUDA 
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           std::vector<cv::Mat>* input_channels
@output: 
           void 
-----------------------------------------*/	
void Detector::WrapInputLayer(std::vector<cv::Mat>* input_channels) 
{
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}	
/*---------------------------------------------------
@function: 
           Detector 求取归一化后的input_channels
		   Preprocess 图像预处理 NO CUDA 
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           const cv::Mat& img
		   std::vector<cv::Mat>* input_channels
@output: 
           void 
---------------------------------------------------*/
void Detector::Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels) 
{
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);
  
  //dynamic mean dimensions
  if (this->mean_.size() != img.size()) // operation should only occur on transit of image resolutions
  {
    cv::Mat meanDynamic;
    cv::resize(this->mean_, meanDynamic, img.size()); 
    this->mean_ = meanDynamic;
  }

  cv::Mat sample_normalized;
  cv::subtract(sample_float, this->mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data) == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
	
#endif 
/*---------------------------------------------------
@function: 
           结果显示
		   Draw On Mat Image 
@author: 
           liuyusen 
@date: 
           2018-01-11
@input:    
           const cv::Mat& img
		   const float & label
		   float & confidence
		   cv::Rect & detection
@output: 
           void 
---------------------------------------------------*/
void Draw(cv::Mat & image, const float & label, const float & confidence, const cv::Rect & detection)
{
  // draw detection
  auto color = cv::Scalar( 255, 0, 0 );
  cv::rectangle( image, detection, color, 2 );

  //put id on image
  std::string idText = "ID: " + std::to_string( label );
  int baseline = 0;
  cv::Size textSize = cv::getTextSize( idText, CV_FONT_HERSHEY_PLAIN, 1.0, 1, &baseline );
  textSize.height += 3;
  textSize.width += 4;
  CvPoint rectID0 = cvPoint( detection.x - 1, detection.y - textSize.height );
  CvPoint rectID1 = cvPoint( detection.x - 1 + textSize.width, detection.y );
  cv::rectangle( image, rectID0, rectID1, color, CV_FILLED );
  cv::putText( image, idText, cv::Point( detection.x + 1, detection.y - 1 ), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar( 255, 255, 255 ) );

  //put confidence on image
  std::string confText = "Score: " + std::to_string( confidence );
  cv::Size confSize = cv::getTextSize( confText, CV_FONT_HERSHEY_PLAIN, 1.0, 1, &baseline );
  confSize.height += 3;
  confSize.width += 4;
  CvPoint rectConf0 = cvPoint( detection.x - 1, detection.y + detection.height );
  CvPoint rectConf1 = cvPoint( detection.x - 1 + confSize.width, detection.y + detection.height + confSize.height );
  cv::rectangle( image, rectConf0, rectConf1, color, CV_FILLED );
  cv::putText( image, confText, cv::Point( detection.x + 1, detection.y + detection.height + confSize.height - 1 ), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar( 255, 255, 255 ) );
}





