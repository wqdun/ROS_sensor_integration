#include "PGCamera.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <FlyCapture2.h>
#include "lock.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

CMutex mymutex;
//time control
extern vector<string> parsed_data;
extern string global_gps ;

//save control
extern int is_save_cam;

//get format
extern int format_int;

//get savepath
extern string pathSave_str;

//set gain
extern float cam_gain;

//string2double
static double string2double(const string& str)
 {
    std::stringstream iss(str);
    double num;
    iss >> num;
    return num;
}

CPGCamera::CPGCamera(int nBufWidth, int nBufHeight, ros::NodeHandle& nh)
{
    m_nBufferWidth = nBufWidth;
    m_nBufferHeight = nBufHeight;

    mnh = nh;
    image_transport::ImageTransport it(mnh);
    pub   = it.advertise("camera/image",1);

    pub_imu_string = mnh.advertise<roscameragpsimg::imu5651>("imu_string", 1);
    pub_cam_speed  = mnh.advertise<std_msgs::Float64>("cam_speed", 1);

    m_pCamera = NULL;

    m_CameraID = 0;
    m_bStartedCapture = false;
}

CPGCamera::~CPGCamera(void)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    delete m_pCamera;
}

bool CPGCamera::InitCamera(int m_CameraID)
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (!IsCameraExist())
    {
        LOG(WARNING) << "Failed to IsCameraExist.";
        return false;
    }

    if (!NewCamera(m_CameraID))
    {
        LOG(WARNING) << "Failed to NewCamera.";
        return false;
    }
    if (!CameraConnect())
    {
        LOG(WARNING) << "Failed to CameraConnect.";
        return false;
    }

    return true;
}

bool CPGCamera::IsCameraExist()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    CameraInfo camInfo[128];
    unsigned int numCamInfo = 0;

    for(size_t i = 0; i < 10; ++i)
    {
        numCamInfo = 128;
        error = BusManager::DiscoverGigECameras(camInfo, &numCamInfo);
        LOG(INFO) << "I discover " << numCamInfo << " camera.";
        if(error != PGRERROR_OK)
        {
            LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
            continue;
        }

        if(numCamInfo >= 1)
        {
            LOG(INFO) << "I discover camera successfully.";
            break;
        }

        error = m_busMgr.RescanBus();
        if(error != PGRERROR_OK)
        {
            LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        }
    }
    if(numCamInfo < 1)
    {
        LOG(ERROR) << "Failed to discover camera, num: " << numCamInfo;
        exit(-1);
    }

    for(size_t i = 0; i < 10; ++i)
    {
        error = m_busMgr.GetNumOfCameras(&m_nCamNum);
        LOG(INFO) << "I GetNumOfCameras: " << m_nCamNum << " cameras.";
        if(error != PGRERROR_OK)
        {
            LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
            continue;
        }

        if(m_nCamNum >= 1)
        {
            LOG(INFO) << "I GetNumOfCameras successfully.";
            break;
        }

        error = m_busMgr.RescanBus();
        if(error != PGRERROR_OK)
        {
            LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        }
    }
    if(m_nCamNum < 1)
    {
        LOG(ERROR) << "Failed to GetNumOfCameras, num: " << m_nCamNum;
        exit(-1);
    }

    return true;
}

bool CPGCamera::NewCamera(int m_CameraID)
{
    LOG(INFO) << __FUNCTION__ << " start.";

    error = m_busMgr.GetCameraFromIndex(m_CameraID, &m_guidCam);
    if (error != PGRERROR_OK)
    {
        LOG(WARNING) << "Failed to GetCameraFromIndex, m_CameraID: " << m_CameraID;
        LOG(WARNING) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        return false;
    }
    m_pCamera = new GigECamera;
    if (!m_pCamera)
    {
        LOG(WARNING) << "Failed to new GigECamera.";
        return false;
    }
    return true;
}

bool CPGCamera::CameraConnect()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        LOG(WARNING) << "m_pCamera is NULL.";
        return false;
    }
    error = m_pCamera->Connect(&m_guidCam);
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        LOG(WARNING) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        return false;
    }

    error = m_pCamera->RestoreFromMemoryChannel(1);
    if ( error != PGRERROR_OK )
    {
        error.PrintErrorTrace();
        LOG(WARNING) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        return false;
    }

    return true;
}

// WriteRegister and RetrieveBuffer is necessary after StartCapture,
// to save image to image(1st param of XferCallBack)
void CPGCamera::Grab()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    GigECamera * m_Cam = (m_pCamera);
    m_triggerModeCam.source = 7;
    if (0 == m_triggerModeCam.source)
    {
        error = m_Cam->StartCapture(XferCallBack , this);
        if ( error != PGRERROR_OK )
        {
            LOG(ERROR) << "startCapture ERROR.";
            cout<< "startCapture ERROR"<<endl;
            return;
        }
    }
    else if (7 == m_triggerModeCam.source)
    {
        if (!m_bStartedCapture)
        {
            error = m_Cam->StartCapture(XferCallBack , this);
            if (error != PGRERROR_OK)
            {
                error = m_Cam->StopCapture();
                return;
            }
            m_bStartedCapture = true;
        }
    }
}

bool CPGCamera::SetCameragain()
{
    LOG(INFO) << __FUNCTION__ << " start.";
    error = ((GigECamera *)m_pCamera)->GetProperty(&pProp );
    if (error != PGRERROR_OK)
    {
        LOG(INFO) << "SetCameragain: GetProperty.";
        error.PrintErrorTrace();
        //return false;
    }
    pProp.type       = GAIN;
    pProp.absControl = true;
    pProp.onePush    = false;
    pProp.onOff      = true;
    pProp.autoManualMode = false;
    pProp.absValue      =  cam_gain;
    error = ((GigECamera *)m_pCamera)->SetProperty(&pProp );
    if (error != PGRERROR_OK)
    {
        LOG(INFO) << "pProp.absValue: " << pProp.absValue;
        LOG(INFO) << "SetCameragain: SetProperty.";
        error.PrintErrorTrace();
    }
    LOG(INFO) << __FUNCTION__ << " end.";
    return true;
}

bool CPGCamera::SetCameraParam()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    LOG(INFO) << "Set camera parameter.";
    error = ((GigECamera *)m_pCamera)->GetGigEImageSettings(&m_GigimageSettings);
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }
    m_GigimageSettings.pixelFormat = PIXEL_FORMAT_MONO8;
    m_GigimageSettings.offsetX = 0;
    m_GigimageSettings.offsetY = 0;
    m_GigimageSettings.width = 2448;
    m_GigimageSettings.height = 2048;

    error = m_pCamera->SetGigEImageSettings(&m_GigimageSettings);
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }

    LOG(INFO) << "Set camera trigger mode: external.";
    error = m_pCamera->GetTriggerMode(&m_triggerModeCam);
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }
    m_triggerModeCam.onOff = true;
    m_triggerModeCam.mode = 1;
    m_triggerModeCam.parameter = 0;
    m_triggerModeCam.source = 0;
    error = m_pCamera->SetTriggerMode(&m_triggerModeCam);
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }
    return true;
}

bool CPGCamera::DisConnectCamera()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }
    error = m_pCamera->Disconnect();
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }
    return true;
}

bool CPGCamera::ReleaseCamera()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    bool bRelease = true;
    bool bConnect = true;

    bConnect = m_pCamera->IsConnected();
    if (bConnect == true)
    {
        StopCapture();
        DisConnectCamera();
    }
    delete m_pCamera;
    m_pCamera = NULL;

    return true;
}

bool CPGCamera::IsCameraConnected()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera)
    {
        return m_pCamera->IsConnected();
    }

    return false;
}

bool CPGCamera::StartCapture()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    error = m_pCamera->StartCapture(XferCallBack, this);
    if (error != PGRERROR_OK)
    {
        LOG(ERROR) << error.GetFilename() << ":" << error.GetLine() << ":" << error.GetDescription();
        error.PrintErrorTrace();
        return false;
    }

    return true;
}

bool CPGCamera::StopCapture()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    if (m_pCamera == NULL)
    {
        return false;
    }

    error = m_pCamera->StopCapture();
    if (error != PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }
    return true;
}

//register callback
void CPGCamera::RegisterDeliverImageCallback(ImageGrabbedCallBack _callback, void *_Owner,void *_Control)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    m_GrabImgCallBack = _callback;
}

//callback
Mat img(1200 , 1920 , CV_8UC3 , Scalar::all(0));
double cam_fps = 0.0;

double curTime_record  = 0;
double preTime_record  = 0;
double fix_rate        = 1.0;
int record_count       = 1;
int time_state         = 0;
int show_state         = 1;

void CPGCamera::XferCallBack(Image* pImages, const void *pCallBackData)
{
    DLOG(INFO) << __FUNCTION__ << " start.";
    ++show_state;
    show_state %= 8;

    ros::Time begin = ros::Time::now();
    double time  = begin.toSec();
    CPGCamera *pThis = (CPGCamera*)pCallBackData;

    FlyCapture2::Image convertedImage;
    FlyCapture2::Error error;

    Image pImage = *(pImages);

    curTime_record = time;

    if(record_count == 1)
    {
        preTime_record = curTime_record;
        record_count = 2;
    }
    else
    {
        double cam_time = curTime_record - preTime_record;
        cam_time = abs(cam_time - 0.4)< 0 ? 1 : cam_time;
        cam_fps = (double)1.0/(double)cam_time;
        preTime_record = curTime_record;
    }

    //time of system
    string seconds_str = std::to_string(time);

    //time of gps
    string gps_str = "--";
    int ret = mymutex.Lock();

    DLOG(INFO) << "trylock result: " << ret;
    if(ret != 0)
    {
        DLOG(WARNING) << "Failed to trylock, result: " << ret;
        return ;
    }

    if(parsed_data.size() >= 17 && global_gps.size() > 2 && global_gps.size() < 20)
    {
        gps_str = global_gps;
        time_state = 0;
    }
    else
    {
        LOG(WARNING) << "GPSweek time error: " << global_gps << "; using system time: " << seconds_str;
        gps_str = seconds_str;
        time_state = 1;
    }

    int retUnlock = mymutex.Unlock();
    DLOG(INFO) << "unlock result: " << retUnlock;

    double gps_str_db = 0.0 ;
    stringstream ss;
    ss << gps_str;
    ss >> gps_str_db;
    gps_str_db = fmod(gps_str_db, 3600 * 24);
    gps_str = std::to_string(gps_str_db);
    gps_str = time_state ? "sys" + gps_str : gps_str;
    DLOG(INFO)<<"time_str: "<<gps_str;

    switch(format_int)
    {
        case 0://jpg
        {
            string gps_str_jpg = pathSave_str + gps_str + ".jpg";
            const char* pathout_gps_jpg_c = gps_str_jpg.c_str();
            if(pathout_gps_jpg_c == NULL)
            {
               DLOG(INFO) << "xcallback: pathout_gps_jgp_c is NULL!";
               break;
            }

            //convert image
            error = pImage.Convert( PIXEL_FORMAT_BGR, &convertedImage);
            if(error != PGRERROR_OK)
            {
                DLOG(INFO) << "xcallback: pImage.Convert error!";
                break ;
            }
            int bits = pImage.GetBitsPerPixel();

            if(is_save_cam && bits == 8)
            {
                FlyCapture2::JPEGOption jpgoption;

                error = convertedImage.Save(pathout_gps_jpg_c, &jpgoption);
                if(error != PGRERROR_OK)
                {
                    LOG(WARNING) << "Save image jpg error!";
                    break;
                }
            }
            if(0 != show_state)
            {
                break;
            }
            bool matConbool = pThis->ConvertImage(&img, &convertedImage);
            Mat matImageDown;
            cv::resize(img, matImageDown, cv::Size(1920 / 5, 1200 / 5));
            LOG(INFO) << "Publish a image, matConvert state: " << matConbool;
            if(matConbool)
            {
                sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matImageDown).toImageMsg();
                if(NULL == msg_img)
                {
                    LOG(WARNING) << "Failed to transfer to sensor_msgs::Image, msg_img is null.";
                    break;
                }
                pThis->pub.publish(msg_img);
            }
            //save image
            if(is_save_cam && bits == 24)
            {
                imwrite(pathout_gps_jpg_c, img);
            }
            break;
        }
        case 1: //raw
        {
            string gps_str_raw  = pathSave_str + gps_str + ".raw";
            const char* pathout_raw    = gps_str_raw.c_str();

            error = pImage.Save(pathout_raw,FlyCapture2::RAW);
            if(error != PGRERROR_OK)
            {
                break;
            }

            break;
        }
        case 2: //jpgcount
        {
            string count_str_jpg = pathSave_str + gps_str +".jpg";//count_str + ".jpg";
            const char* pathout_count_jpg_c    = count_str_jpg.c_str();
            //image convert
            error = pImage.Convert( PIXEL_FORMAT_BGR, &convertedImage);
            if(error != PGRERROR_OK)
            {
                break;
            }
            //image save jpg
            FlyCapture2::JPEGOption jpgoption;
            error = convertedImage.Save(pathout_count_jpg_c,&jpgoption);

            if(error != PGRERROR_OK)
            {
                break;
            }

            break;
        }
        case 3: //png
        {
            string count_str_jpg = pathSave_str +gps_str+ ".png";//count_str + ".png";
            const char* pathout_count_jpg_c    = count_str_jpg.c_str();
            //image convert
            error = pImage.Convert( PIXEL_FORMAT_BGR, &convertedImage);
            if(error != PGRERROR_OK)
            {
                break;
            }
            //image save png
            FlyCapture2::PNGOption pngoption;
            error = convertedImage.Save(pathout_count_jpg_c,&pngoption);

            if(error != PGRERROR_OK)
            {
                break;
            }
            break;
        }
        default:
        {
            DLOG(INFO)<<"the format is not surported!";
            break ;
        }
    }

    pThis->msg_cam_speed.data = cam_fps;
    pThis->pub_cam_speed.publish(pThis->msg_cam_speed);
    LOG(INFO)<<"cam_fps: "<<cam_fps;

    return ;
}

double CPGCamera::TimeStamptoDouble(FlyCapture2::TimeStamp *timeStamp)
{
    return (double)timeStamp->cycleSeconds + (((double)timeStamp->cycleCount+((double)timeStamp->cycleOffset/3072.0))/8000.0);
}

void CPGCamera::ImageRecieve(cv::Mat matImage)
{

}

cv::Mat CPGCamera::GetmatImage(void)
{

}

bool CPGCamera::ConvertImage(cv::Mat* matImage, Image* image)
{
    int rows = matImage->rows;
    int cols = matImage->cols;

    int step = matImage->step;
    uchar* dst_data = (uchar*)matImage->data;

    int height = image->GetRows();
    int width  = image->GetCols();

    int bits  = image->GetBitsPerPixel();
    int stride = image->GetStride();

    uchar* src_data = (uchar*)image->GetData();

    if(height!=rows || width != cols)
    {
        return false;
    }
    for(int i = 0 ; i < rows; i++)
    {
        for(int j = 0 ; j < 3*cols ; j=j+3)
        {
            dst_data[i*step+j]=src_data[i*stride+j];
            dst_data[i*step+j+1]=src_data[i*stride+j+1];
            dst_data[i*step+j+2]=src_data[i*stride+j+2];
        }
    }
    return true;
}
