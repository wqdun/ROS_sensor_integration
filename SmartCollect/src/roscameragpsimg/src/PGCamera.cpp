#include "PGCamera.h"
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<FlyCapture2.h>
#include "lock.h"
CMutex mymutex;
//time control
extern vector<string> parsed_data;
extern string global_gps ;
//string global_gps ;

//save control
extern int is_save_cam;

//get format
extern int format_int;

//get savepath
extern string pathSave_str;

//string2double
static double string2double(const string& str)
 {
    std::stringstream iss(str);
    double num;
    iss >> num;
    return num;
}

CPGCamera::CPGCamera(int nBufWidth, int nBufHeight,ros::NodeHandle& nh)
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
	delete m_pCamera;
}

bool CPGCamera::InitCamera(int m_CameraID)
{
	if (!IsCameraExist())
	{
		return false;
	}

	if (!NewCamera(m_CameraID))
	{
		return false;
	}
	if (!CameraConnect())
	{
		return false;
	}

	return true;
}
bool CPGCamera::IsCameraExist()
{
	error = m_busMgr.GetNumOfCameras(&m_nCamNum);
	if (error != PGRERROR_OK || m_CameraID < 0 || m_CameraID > m_nCamNum)
	{
		error.PrintErrorTrace();
		return false;
	}
	return true;
}

bool CPGCamera::NewCamera(int m_CameraID)
{
	error = m_busMgr.GetCameraFromIndex(m_CameraID, &m_guidCam);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return false;
	}
	m_pCamera = new GigECamera;
	if (!m_pCamera)
	{
		return false;
	}
	return true;

}

bool CPGCamera::CameraConnect()
{
	if (m_pCamera == NULL)
	{
		return false;
	}
	error = m_pCamera->Connect(&m_guidCam);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return false;
	}


	error = m_pCamera->RestoreFromMemoryChannel(1);
	if ( error != PGRERROR_OK )
	{
		error.PrintErrorTrace();
		return false;
	}

	return true;
}
void CPGCamera::Grab()
{
	/******************************************************/
	//修改PG相机软件外触发采集图像问题 调用StartCapture后需WriteRegister并RetrieveBuffer后图像在image中
	//设置回调函数并开始采集
	GigECamera * m_Cam = (m_pCamera);
	m_triggerModeCam.source = 7;
	if (0 == m_triggerModeCam.source)
	{//硬件触发
		error = m_Cam->StartCapture(XferCallBack , this);
		if ( error != PGRERROR_OK )
		{
			cout<< "startCapture ERROR"<<endl;
			return;
		}
	}
	else if (7 == m_triggerModeCam.source)
	{//软件触发
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

bool CPGCamera::SetCameraParam()
{
	if (m_pCamera == NULL)
	{
		return false;
	}

	//写入图像参数
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


	//设置相机触发模式
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
	if (m_pCamera)
	{
		return m_pCamera->IsConnected();
	}

	return false;
}

bool CPGCamera::StartCapture()
{
	if (m_pCamera == NULL)
	{
		return false;
	}

	error = m_pCamera->StartCapture(XferCallBack, this);  //待修改
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return false;
	}


	return true;
}

bool CPGCamera::StopCapture()
{
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

void CPGCamera::XferCallBack(Image* pImages, const void *pCallBackData)
{
    // mymutex.Unlock();
    ros::Time begin = ros::Time::now();
    double    time  = begin.toSec();
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
            cam_time = abs(cam_time - 0.0000001)< 0 ? 1 : cam_time;
            cam_fps      = (double)1.0/(double)cam_time;
            preTime_record = curTime_record;
        }

        //time of system
	    string seconds_str = std::to_string(time);

        //time of gps

        string gps_str = "--";
        int ret = mymutex.Trylock();
        
        if(ret==0&&parsed_data.size() >= 17 && global_gps.size()>2 && global_gps.size()<20)
        {
                gps_str = global_gps;
                time_state = 0;
                mymutex.Unlock();
        }
        else
        {
            DLOG(INFO)<<"time_global_gps_str: "<<global_gps;
            gps_str = seconds_str;
            time_state = 1;
        }
        
       
        double gps_str_db = 0.0 ;
        stringstream ss;
        ss<<gps_str;
        ss>>gps_str_db;
        gps_str_db = fmod(gps_str_db,3600*24);

        gps_str = std::to_string(gps_str_db);

        gps_str = time_state ? "sys" + gps_str : gps_str;
        DLOG(INFO)<<"time_str: "<<gps_str;

        switch(format_int)
        {
            case 0://jpg
               {
                   //save image in jpg format
                   string gps_str_jpg =pathSave_str + gps_str + ".jpg";
                   const char* pathout_gps_jpg_c    = gps_str_jpg.c_str();
                   if(pathout_gps_jpg_c==NULL)
                   {
                      DLOG(INFO)<<"xcallback: pathout_gps_jgp_c is NULL!";
                      break;
                   }
                   //convert image
                   error = pImage.Convert( PIXEL_FORMAT_BGR, &convertedImage);
	               if(error != PGRERROR_OK)
	               {
                       DLOG(INFO)<<"xcallback: pImage.Convert error!";
	                   break ;
	               }

                   bool matConbool =  pThis->ConvertImage(&img,&convertedImage);
                   DLOG(INFO)<<"xcallback  matConvert state: "<<matConbool;
	               if(matConbool)
	               {
	                   sensor_msgs::ImagePtr msg_img=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img).toImageMsg();
                       if(NULL == msg_img)
                       {
                           cout << "[ERROR] msg_img is null.\n";
                       }
                       pThis->pub.publish(msg_img);
	               }
                   //save image
                   if(is_save_cam)
                   {
                       imwrite(pathout_gps_jpg_c,img);
                       //FlyCapture2::JPG2Option jpgoption;
                       //convertedImage.Save(pathout_gps_jpg_c,&jpgoption);
                   }

                   break;
               }
            case 1: //raw
               {
                   //save image in raw format
                   //string count_str    = std::to_string(count);
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
                   //save image in jpg format
                   //string count_str     = std::to_string(count);
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
                   //save image in png format
                   //string count_str     = std::to_string(count);
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
        {
            pThis->msg_cam_speed.data = cam_fps;
            pThis->pub_cam_speed.publish(pThis->msg_cam_speed);
            DLOG(INFO)<<"cam_fps: "<<cam_fps;
        }

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

    FlyCapture2::PixelFormat pixelformat = image->GetPixelFormat();
    DLOG(INFO)<<"pixelFormat: "<<pixelformat;

    int bits  = image->GetBitsPerPixel();
    DLOG(INFO)<<"bits of per pixels: "<<bits;

    int stride = image->GetStride();
    DLOG(INFO)<<"stride of the image: "<< stride;

    uchar* src_data = image->GetData();

    DLOG(INFO)<<"ROWS,COLS,HEIGHT,WIDHT: "<<rows<<" "<<cols<<" "<<height<<" "<<width;
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
