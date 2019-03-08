#include "single_ptgrey_camera.h"
#include "ptgrey_camera_manager.h"
#include "ptgrey_save_image_task.h"

PtgreyCameraManager *SinglePtgreyCamera::s_pManager_;
SinglePtgreyCamera::SinglePtgreyCamera(PtgreyCameraManager *pManager):
    mat2Pub_(1200, 1920, CV_8UC3, cv::Scalar::all(0) )
{
    LOG(INFO) << __FUNCTION__ << " start.";
    lastImageTimeStampInSeconds = 0;
    imageTimeStampInSecondsTimes128 = 0;
    s_pManager_ = pManager;

}

void SinglePtgreyCamera::InitCamera(int _index, const std::string &_rawdataDir) {
    LOG(INFO) << __FUNCTION__ << " start.";

    FlyCapture2::BusManager busMgr;
    FlyCapture2::PGRGuid guidCam;

    error_ = busMgr.GetCameraFromIndex(_index, &guidCam);
    if(error_ != FlyCapture2::PGRERROR_OK) {
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(1);
    }

    pCamera_ = new FlyCapture2::GigECamera();
    if(!pCamera_) {
        LOG(ERROR) << "Failed to new GigECamera.";
        exit(2);
    }

    error_ = pCamera_->Connect(&guidCam);
    if(error_ != FlyCapture2::PGRERROR_OK) {
        PtgreyCameraManager::LogErrorTrace(error_);
        delete pCamera_;
        exit(3);
    }

    SetCameraProperties();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pubImage_ = it.advertise("camera/image" + std::to_string(_index), 10);

    SetImagePath();
    return;
}

void SinglePtgreyCamera::SetCameraProperties() {
    LOG(INFO) << __FUNCTION__ << " start.";

    FlyCapture2::Property property;
    property.type = FlyCapture2::SHUTTER;
    error_ = pCamera_->GetProperty(&property);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "Failed to GetProperty.";
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(1);
    }
    LOG(INFO) << "property: " << property.absValue;

    property.autoManualMode = false;
    property.absValue = 1.;
    error_ = pCamera_->SetProperty(&property);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "property.absValue: " << property.absValue;
        LOG(ERROR) << "Failed to SetProperty.";
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(2);
    }

    property.type = FlyCapture2::AUTO_EXPOSURE;
    error_ = pCamera_->GetProperty(&property);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "Failed to GetProperty.";
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(1);
    }
    LOG(INFO) << "property: " << std::boolalpha << property.autoManualMode;

    property.autoManualMode = false;
    error_ = pCamera_->SetProperty(&property);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "property.autoManualMode: " << property.autoManualMode;
        LOG(ERROR) << "Failed to SetProperty.";
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(2);
    }

    property.type = FlyCapture2::GAIN;
    error_ = pCamera_->GetProperty(&property);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "Failed to GetProperty.";
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(1);
    }
    LOG(INFO) << "property: " << std::boolalpha << property.autoManualMode;

    property.autoManualMode = true;
    error_ = pCamera_->SetProperty(&property);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        LOG(ERROR) << "property.autoManualMode: " << property.autoManualMode;
        LOG(ERROR) << "Failed to SetProperty.";
        PtgreyCameraManager::LogErrorTrace(error_);
        exit(2);
    }

    LOG(INFO) << __FUNCTION__ << " end.";
    return;
}

void SinglePtgreyCamera::SetImagePath() {
    LOG(INFO) << __FUNCTION__ << " start.";
    imagePath_ = (s_pManager_->rawDataPath_) + "/Image/";
}

SinglePtgreyCamera::~SinglePtgreyCamera() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

bool SinglePtgreyCamera::Run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    error_ = pCamera_->StartCapture(XferCallBack, this);
    if (error_ != FlyCapture2::PGRERROR_OK) {
        PtgreyCameraManager::LogErrorTrace(error_);
        return false;
    }

    return true;
}

void SinglePtgreyCamera::XferCallBack(FlyCapture2::Image *pImage, const void *_pSinglePtgreyCamera) {
    struct timeval now;
    gettimeofday(&now, NULL);
    const double unixTime = now.tv_sec + now.tv_usec / 1000000.;

    DLOG(INFO) << __FUNCTION__ << " start.";
    SinglePtgreyCamera *pSinglePtgreyCamera = (SinglePtgreyCamera *)_pSinglePtgreyCamera;

    pSinglePtgreyCamera->mat2PubMutex_.lock();
    pSinglePtgreyCamera->inImage_ = *pImage;
    pSinglePtgreyCamera->mat2PubMutex_.unlock();

    const FlyCapture2::TimeStamp _imageTimeStamp = pImage->GetTimeStamp();
    const double imageTimeStamp = _imageTimeStamp.seconds + _imageTimeStamp.microSeconds / 1000000.;
    const std::string cycleTime(std::to_string(_imageTimeStamp.cycleSeconds) + "_" + std::to_string(_imageTimeStamp.cycleCount) );
    const FlyCapture2::ImageMetadata imageMetadata = pImage->GetMetadata();

    unsigned int uiRawTimestamp = imageMetadata.embeddedTimeStamp;

    // from https://www.ptgrey.com/kb/10308; http://www.ptgrey.com/Content/Images/uploaded/KB-Data/PGRFlyCaptureTimestampTest.zip
    const int nSecond      = (uiRawTimestamp >> 25) & 0x7F;   // get rid of cycle_* - keep 7 bits
    const int nCycleCount  = (uiRawTimestamp >> 12) & 0x1FFF; // get rid of offset
    const int nCycleOffset = (uiRawTimestamp >>  0) & 0xFFF;  // get rid of *_count
    double imageTimeStampInSeconds = (double)nSecond + (((double)nCycleCount+((double)nCycleOffset/3072.0))/8000.0);

    if(imageTimeStampInSeconds < pSinglePtgreyCamera->lastImageTimeStampInSeconds) {
        ++(pSinglePtgreyCamera->imageTimeStampInSecondsTimes128);
    }
    pSinglePtgreyCamera->lastImageTimeStampInSeconds = imageTimeStampInSeconds;

    imageTimeStampInSeconds += (pSinglePtgreyCamera->imageTimeStampInSecondsTimes128 * 128);
    const std::string picFileName(pSinglePtgreyCamera->imagePath_ + std::to_string(imageTimeStamp) + "_" + std::to_string(unixTime) + "_" + std::to_string(imageTimeStampInSeconds) + "_" + std::to_string(imageMetadata.embeddedFrameCounter) + "_" + cycleTime + ".jpg");
    DLOG(INFO) << "picFileName: " << picFileName;
    if(s_pManager_->isSaveImg_) {
        PtgreySaveImageTask *pSaveImageTask = new PtgreySaveImageTask(pImage, picFileName);
        s_pManager_->threadPool_.append_task(pSaveImageTask);
    }
    // else not save image

    DLOG(INFO) << __FUNCTION__ << " end.";
    return;
}

void SinglePtgreyCamera::PublishImage() {
    DLOG(INFO) << __FUNCTION__ << " start.";

    LOG_FIRST_N(INFO, 1) << "Convert a 1920*1200 image costs 4 ms in TX2 platform.";
    mat2PubMutex_.lock();
    error_ = inImage_.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &convertedImage_);
    mat2PubMutex_.unlock();
    if(error_ != FlyCapture2::PGRERROR_OK) {
        LOG(WARNING) << "Failed to Convert.";
        PtgreyCameraManager::LogErrorTrace(error_);
        return;
    }

    LOG_FIRST_N(INFO, 1) << "ConvertImage a 1920*1200 image costs 40 ms in TX2 platform.";
    bool isConvert2Mat = ConvertImage(&mat2Pub_, &convertedImage_);
    if (!isConvert2Mat) {
        LOG(ERROR) << "Failed to convertImage.";
        return;
    }

    cv::Mat imageResized;
    cv::resize(mat2Pub_, imageResized, cv::Size(mat2Pub_.cols / 4, mat2Pub_.rows / 4));
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageResized).toImageMsg();
    pubImage_.publish(imgMsg);

    return;
}

// @function:
//           convert the format of the image: FlyCapture2::Image --> Cv::Mat
// @author:
//           liuyusen@navinfo.com
// @date:
//           2018-01-02
// @input:
//           Mat *matImage == output image point which is Mat
//           Image *image  == input  image point which is Image
// @output:
//           bool == tell the result of the convert function
bool SinglePtgreyCamera::ConvertImage(cv::Mat* matImage, FlyCapture2::Image* image)
{
    int rows = matImage->rows;
    int cols = matImage->cols;
    int step = matImage->step;
    uchar* dst_data = (uchar*)matImage->data;

    int height = image->GetRows();
    int width  = image->GetCols();
    int stride = image->GetStride();
    uchar* src_data = (uchar*)image->GetData();

    if(height != rows || width != cols)
    {
        LOG(WARNING) << "height(" << height << ") != rows(" << rows << "); " << "width(" << width << ") != cols(" << cols << ").";
        return false;
    }
    LOG_FIRST_N(INFO, 1) << "height(" << height << ") == rows(" << rows << "); " << "width(" << width << ") == cols(" << cols << "); step(" << step << "); stride(" << stride << ").";

    int cols_3 = 3 * cols;
    for(int i = 0; i < rows; ++i)
    {
        int i_step = i * step;
        int i_stride = i * stride;
        for(int j = 0 ; j < cols_3; j += 3)
        {
            dst_data[i_step + j] = src_data[i_stride + j];
            dst_data[i_step + j + 1] = src_data[i_stride + j + 1];
            dst_data[i_step + j + 2] = src_data[i_stride + j + 2];
        }
    }
    return true;
}



