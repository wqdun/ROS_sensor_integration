#include "camera.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

Cameras::Cameras(ros::NodeHandle nh, ros::NodeHandle private_nh, const std::string &_imgFormat, const std::string &_rawdataDir):
    cvMatImg_(1200 , 1920 , CV_8UC3 , Scalar::all(0) )
{
    LOG(INFO) << __FUNCTION__ << " start.";

    imgFormat_ = _imgFormat;

    camGainLast_ = -1;
    gpsWeekTimeCorrected_ = -1.;
    isGpsTimeValid_ = false;

    subServer_ = nh.subscribe("sc_monitor", 0, &Cameras::serverCB, this);
    pubCamSpeed_ = nh.advertise<std_msgs::Float64>("cam_speed", 0);

    BusManager busMgr;
    unsigned int cameraNum = 0;
    FlyCapture2::Error error = busMgr.GetNumOfCameras(&cameraNum);
    if(error != PGRERROR_OK) {
        logErrorTrace(error);
        exit(1);
    }
    LOG(INFO) << "Number of cameras detected: " << cameraNum;
    if(cameraNum < 1) {
        LOG(WARNING) << "Insufficient number of cameras: " << cameraNum;
        exit(2);
    }
    pCpgCameras_.clear();
    (void)getCameras(cameraNum, _rawdataDir);
    if( (masterIndex_ = getMasterIndex() ) < 0) {
        LOG(ERROR) << "Failed to getMasterIndex: " << masterIndex_;
        exit(3);
    }

    const double MASTER_CAMERA_FRAME_RATE = 10;
    (void)setCameraFrameRate(MASTER_CAMERA_FRAME_RATE);
    const unsigned int GIGE_PACKET_SIZE = 9000;
    (void)setAllCamerasGigEPacketSize(GIGE_PACKET_SIZE);

    pCommTimer_.reset(new CommTimer(_rawdataDir) );
}

Cameras::~Cameras() {
    LOG(INFO) << __FUNCTION__ << " start.";
    for(auto &pCpgCamera: pCpgCameras_) {
        pCpgCamera->StopCapture();
        pCpgCamera->m_pCamera->Disconnect();
        delete pCpgCamera;
    }

    timeThread_->join();
}

void Cameras::setCameraFrameRate(double frameRate) {
    LOG(INFO) << __FUNCTION__ << " start.";

    FlyCapture2::Error error;
    Property frameRateProp;
    frameRateProp.type = FRAME_RATE;

    GigECamera *pMasterGigECamera = pCpgCameras_[masterIndex_]->m_pCamera;
    error = pMasterGigECamera->GetProperty(&frameRateProp);
    if(error != PGRERROR_OK) {
        LOG(ERROR) << "Error getting camera property.";
        logErrorTrace(error);
        exit(4);
    }

    PropertyInfo frameRatePropInfo;
    frameRatePropInfo.type = FRAME_RATE;
    error = pMasterGigECamera->GetPropertyInfo(&frameRatePropInfo);
    if(error != PGRERROR_OK) {
        LOG(ERROR) << "Error getting camera property information.";
        logErrorTrace(error);
        exit(5);
    }

    if(frameRatePropInfo.absValSupported) {
        LOG(INFO) << "frameRatePropInfo.absValSupported.";
        frameRateProp.absValue = static_cast<float>(frameRate);
        frameRateProp.absControl = true;
    }
    else {
        LOG(WARNING) << "Not frameRatePropInfo.absValSupported.";
        frameRateProp.valueA = static_cast<unsigned int>(frameRate);
        frameRateProp.absControl = false;
    }

    frameRateProp.autoManualMode = false;

    error = pMasterGigECamera->SetProperty(&frameRateProp, false);
    if(error != PGRERROR_OK) {
        LOG(ERROR) << "Error setting camera property.";
        logErrorTrace(error);
        exit(6);
    }

    return;
}

void Cameras::setAllCamerasGigEPacketSize(unsigned int _packetSize) {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(auto &pCpgCamera: pCpgCameras_) {
        GigECamera *pGigECamera = pCpgCamera->m_pCamera;
        (void)setCameraGigEPacketSize(pGigECamera, _packetSize);
    }
    return;
}

void Cameras::setCameraGigEPacketSize(GigECamera *_pGigECamera, unsigned int _packetSize) {
    LOG(INFO) << __FUNCTION__ << " start.";
    FlyCapture2::Error error;

    GigEProperty packetSizeProp;
    packetSizeProp.propType = PACKET_SIZE;
    packetSizeProp.value = _packetSize;
    error = _pGigECamera->SetGigEProperty(&packetSizeProp);
    if(error != PGRERROR_OK) {
        LOG(ERROR) << "Error setting GigE packet size.";
        logErrorTrace(error);
        exit(7);
    }

    packetSizeProp.value = 0;
    error = _pGigECamera->GetGigEProperty(&packetSizeProp);
    if(error != PGRERROR_OK) {
        LOG(ERROR) << "Error getting current packet size.";
        logErrorTrace(error);
        exit(8);
    }
    LOG(INFO) << "Set packet size successfully, current packet size: " << packetSizeProp.value;
    return;
}

void Cameras::getCameras(unsigned int _cameraNum, const std::string &__rawdataDir) {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(int i = 0; i < _cameraNum; ++i) {
        CPGCamera *pCPGCamera = new CPGCamera(i, __rawdataDir);
        if(NULL == pCPGCamera) {
            LOG(ERROR) << "Failed to create CPGCamera.";
            exit(3);
        }
        pCPGCamera->pCamerasDaddy_ = this;

        pCpgCameras_.push_back(pCPGCamera);
    }
}

void Cameras::startAllCapture() {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(auto &pCpgCamera: pCpgCameras_) {
        pCpgCamera->StartCapture();
    }
}

void Cameras::logErrorTrace(FlyCapture2::Error error) {
    LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
}

void Cameras::serverCB(const sc_msgs::MonitorMsg::ConstPtr& pClientMsg) {
    LOG(INFO) << __FUNCTION__ << " start, is_record Camera: " << (int)(pClientMsg->is_record);

    bool isSaveImg = (bool)pClientMsg->is_record;
    for(auto &pCpgCamera: pCpgCameras_) {
        pCpgCamera->isSaveImg_ = isSaveImg;
    }

    int8_t camGain = pClientMsg->cam_gain;
    if(camGain != camGainLast_) {
        LOG(INFO) << "Only set master camera's gain.";

        CPGCamera *pMasterPgCamera = pCpgCameras_[masterIndex_];
        if(!pMasterPgCamera->StopCapture() ) {
            LOG(WARNING) << "Failed to StopCapture.";
            return;
        }
        bool isGainSet = pMasterPgCamera->SetCameragain(camGain);
        LOG(INFO) << "camGainLast_: " << (int)camGainLast_ << " --> camGain: " << (int)camGain;
        if(isGainSet) {
            LOG(INFO) << "camGain changed successfully.";
        }
        else {
            LOG(WARNING) << "Failed to set camGain.";
        }
        pMasterPgCamera->StartCapture();
    }

    camGainLast_ = camGain;
}

void Cameras::pubTopic(int _i) {
    LOG(INFO) << __FUNCTION__ << " start.";

    std_msgs::Float64 msgCamSpeed;
    msgCamSpeed.data = pCpgCameras_[_i]->camFps_;
    pubCamSpeed_.publish(msgCamSpeed);
}

int Cameras::getMasterIndex() {
    LOG(INFO) << __FUNCTION__ << " start.";

    for(int i = 0; i < pCpgCameras_.size(); ++i) {
        if(0 == pCpgCameras_[i]->cameraId_) {
            return i;
        }
    }

    return -1;
}

bool Cameras::flyImage2msg(const FlyCapture2::Image &inFlyImage) {
    LOG(INFO) << __FUNCTION__ << " start.";

    int flyRows = inFlyImage.GetRows();
    int flyCols = inFlyImage.GetCols();
    if(0 == flyRows) {
        LOG(WARNING) << "inFlyImage->GetRows(): 0";
        return false;
    }

    cv::Mat cvMatImg(flyRows, flyCols , CV_8UC3 , Scalar::all(0) );
    bool isConverted = convertImage(&cvMatImg, &inFlyImage);
    LOG(INFO) << "Publish a image, matConvert state: " << isConverted;
    if(!isConverted)
    {
        LOG(WARNING) << "Failed to convertImage.";
        return false;
    }

    cv::Mat cvMatImgResize;
    cv::resize(cvMatImg, cvMatImgResize, cv::Size(1920 / 5, 1200 / 5));
    msgImg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvMatImgResize).toImageMsg();
    if(NULL == msgImg_)
    {
        LOG(WARNING) << "Failed to transfer to sensor_msgs::Image.";
        return false;
    }

    return true;
}

// @function:
//           convert the format of the image
// @author:
//           liuyusen@navinfo.com
// @date:
//           2018-01-02
// @input:
//           Mat *matImage == output image point which is Mat
//           Image *image  == input  image point which is Image
// @output:
//           bool == tell the result of the convert function
bool Cameras::convertImage(cv::Mat* matImage, const Image* image) {
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

void Cameras::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    timeThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CommTimer::getTime, pCommTimer_, this) ) );
    LOG(INFO) << "Start CommTimer thread.";

    (void)startAllCapture();

    const int camNum = pCpgCameras_.size();
    int i = -1;

    ros::Rate rate(5);
    while(ros::ok() ) {
        ++i;
        i %= camNum;
        ros::spinOnce();
        rate.sleep();

        (void)pubTopic(i);
        LOG(INFO) << "gpsWeekTimeCorrected_: " << std::fixed << gpsWeekTimeCorrected_;
    }
}

