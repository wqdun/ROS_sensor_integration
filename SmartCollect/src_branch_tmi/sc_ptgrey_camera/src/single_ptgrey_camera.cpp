#include "single_ptgrey_camera.h"
#include "ptgrey_camera_manager.h"
#include "ptgrey_save_image_task.h"

PtgreyCameraManager *SinglePtgreyCamera::s_pManager_;
SinglePtgreyCamera::SinglePtgreyCamera(PtgreyCameraManager *pManager):
    mat2Pub_(1200, 1920, CV_8UC3, cv::Scalar::all(0) )
{
    LOG(INFO) << __FUNCTION__ << " start.";
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

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pubImage_ = it.advertise("camera/image" + std::to_string(_index), 10);

    return;
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
    DLOG(INFO) << __FUNCTION__ << " end.";
    SinglePtgreyCamera *pSinglePtgreyCamera = (SinglePtgreyCamera *)_pSinglePtgreyCamera;

    FlyCapture2::Error flyError = pImage->Convert(FlyCapture2::PIXEL_FORMAT_BGR, &(pSinglePtgreyCamera->convertedImage_) );
    if(flyError != FlyCapture2::PGRERROR_OK) {
        LOG(WARNING) << "Failed to pImage->Convert.";
        PtgreyCameraManager::LogErrorTrace(flyError);
        return;
    }

    pSinglePtgreyCamera->mat2PubMutex_.lock();
    bool isConvert2Mat = ConvertImage(&(pSinglePtgreyCamera->mat2Pub_), &(pSinglePtgreyCamera->convertedImage_) );
    pSinglePtgreyCamera->mat2PubMutex_.unlock();
    if (!isConvert2Mat) {
        LOG(ERROR) << "Failed to convertImage.";
        return;
    }

    static int i = 0;
    ++i;
    const std::string picFileName(std::to_string(i) + ".jpg");
    PtgreySaveImageTask *pSaveImageTask = new PtgreySaveImageTask(pImage, picFileName);
    s_pManager_->threadPool_.append_task(pSaveImageTask);

    DLOG(INFO) << __FUNCTION__ << " end.";
    return;
}

void SinglePtgreyCamera::PublishImage() {
    DLOG(INFO) << __FUNCTION__ << " start.";

    cv::Mat imageResized;
    mat2PubMutex_.lock();
    cv::resize(mat2Pub_, imageResized, cv::Size(mat2Pub_.cols / 10, mat2Pub_.rows / 10));
    mat2PubMutex_.unlock();
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



