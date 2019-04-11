#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using namespace std;
using namespace cv;

void imageResize(Mat image, Mat* dst, int width, int height, int inter = CV_INTER_AREA )
{

    int w = image.cols;
    int h = image.rows;
    int newW = width;
    int newH = height;
    if(width == 0 && height ==0){
        return;
    }
    if(width == 0){
        float re = h/(float)height;
        newW = (int) w * re;
    } else {
        float re = (float)width / w;
        newH = (int) h * re;;
    }

    cout << newW << "; " << newH << "\n";

    resize(image, *dst, Size(newW, newH),inter);
}

void abs_sobel_thresh(const cv::Mat& src,cv::Mat& dst,const char& orient='x',const int& thresh_min=0,const int& thresh_max=255){
    cv::Mat src_gray,grad;
    cv::Mat abs_gray;
    //转换成为灰度图片
    cv::cvtColor(src,src_gray,cv::COLOR_RGB2GRAY);
    //使用cv::Sobel()计算x方向或y方向的导
    if(orient=='x'){
        cv::Sobel(src_gray,grad,CV_64F,1,0);
        cv::convertScaleAbs(grad,abs_gray);
    }
    if(orient=='y'){
        cv::Sobel(src_gray,grad,CV_64F,0,1);
        cv::convertScaleAbs(grad,abs_gray);
    }
    //二值化
    cv::inRange(abs_gray,thresh_min,thresh_max,dst);
    // cv::threshold(abs_gray,dst,thresh_min,thresh_max,cv::THRESH_BINARY|cv::THRESH_OTSU);
}


void mag_thresh(const cv::Mat& src,cv::Mat& dst,const int& sobel_kernel=3,const int& thresh_min=0,const int& thresh_max=255){
    cv::Mat src_gray,gray_x,gray_y,grad;
    cv::Mat abs_gray_x,abs_gray_y;
    //转换成为灰度图片
    cv::cvtColor(src,src_gray,cv::COLOR_RGB2GRAY);
    //使用cv::Sobel()计算x方向或y方向的导
    cv::Sobel(src_gray,gray_x,CV_64F,1,0,sobel_kernel);
    cv::Sobel(src_gray,gray_y,CV_64F,0,1,sobel_kernel);
    //转换成CV_8U
    cv::convertScaleAbs(gray_x,abs_gray_x);
    cv::convertScaleAbs(gray_y,abs_gray_y);
    //合并x和y方向的梯度
    cv::addWeighted(abs_gray_x,0.5,abs_gray_y,0.5,0,grad);
    //二值化
    cv::inRange(grad,thresh_min,thresh_max,dst);
    // cv::threshold(grad,dst,thresh_min,thresh_max,cv::THRESH_BINARY|cv::THRESH_OTSU);

}

void hls_select(const cv::Mat& src,cv::Mat& dst,const char& channel='s',const int& thresh_min=0,const int& thresh_max=255){
    cv::Mat hls,grad;
    vector<cv::Mat> channels;
    cv::cvtColor(src,hls,cv::COLOR_RGB2HLS);
    //分离通道
    cv::split(hls,channels);
    //选择通道
    switch (channel)
    {
        case 'h':
            grad=channels.at(0);
            break;
        case 'l':
            grad=channels.at(1);
            break;
        case 's':
            grad=channels.at(2);
            break;
        default:
            break;
    }
    //二值化
    cv::inRange(grad,thresh_min,thresh_max,dst);
    // cv::threshold(grad,dst,thresh_min,thresh_max,cv::THRESH_BINARY);
}


/*
@param src 为图像透视变形基础点
@param dst 为图像透视变形目标点
@param M 输出为变形系数矩阵
@param Minv 输出为反变形系数矩阵，还原变形使用
@NOTE 该函数用于计算变形系数矩阵和反变形系数矩阵
*/
void get_M_Minv(const vector<cv::Point2f>& src,const vector<cv::Point2f>& dst,cv::Mat& M,cv::Mat& Minv){
    M=cv::getPerspectiveTransform(src,dst);
    Minv=cv::getPerspectiveTransform(dst,src);
}

/*
@param in_point 为离散坐标点
@param n 为n次多项式
@param mat_k 为返回多项式的k系数，为n*1的矩阵
@NOTE 该函数用于拟合曲线
*/
cv::Mat polyfit(vector<cv::Point>& in_point, int n){
    int size = in_point.size(); //所求未知数个数
    int x_num = n + 1;  //构造矩阵U和Y
    cv::Mat mat_u(size, x_num, CV_64F);
    cv::Mat mat_y(size, 1, CV_64F);
    for (int i = 0; i < mat_u.rows; ++i){
        for (int j = 0; j < mat_u.cols; ++j){
            mat_u.at<double>(i, j) = pow(in_point[i].y, j);//in_point[i].y为以y为递增坐标
        }
    }
    for (int i = 0; i < mat_y.rows; ++i){
        mat_y.at<double>(i, 0) = in_point[i].x;
    }   //矩阵运算，获得系数矩阵K
    cv::Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
    // cout << mat_k << endl;
    return mat_k;
}



/*
@param mat_k 为多项式的系数矩阵
@param src 为需要计算的坐标点
@param n 为计算n次多项式
@NOTE 该函数用于计算拟合过后的曲线点坐标，该函数以y为基础坐标轴，如果要以x为基础坐标轴，则修改y为x
*/
vector<cv::Point> polyval(const cv::Mat& mat_k,const vector<cv::Point>& src,int n){
    vector<cv::Point> ip;
    // cout<<src.back().y<<"kkk"<<src.front().y<<endl;
    for(int i=src.back().y;i<src.front().y;i++){//从y=0开始计算，分别计算出x的值
        cv::Point ipt;
        ipt.x=0;
        ipt.y=i;
        for(int j=0;j<n+1;j++){
            ipt.x+=mat_k.at<double>(j,0)*pow(i,j);//NOTE多项式计算
        }
        ip.push_back(ipt);
    }
    return ip;
}

/*
@param src 为需要计算车道线的变形后且二值化的图像
@param lp 输出为左车道线坐标
@param rp 输出为右车道线坐标
@param rightx_current 输出为左车道线基准x坐标
@param leftx_current 输出为右车道线基准x坐标
@param distance_from_center 输出为车道偏离距离
@NOTE 该函数用于计算车道线坐标点以及偏离距离
*/
void find_line(const cv::Mat& src,vector<cv::Point>& lp,vector<cv::Point>& rp,int& rightx_current,int& leftx_current,double& distance_from_center){
    cv::Mat hist,nonzero,l,r;
    vector<cv::Point> nonzerol,nonzeror,lpoint,rpoint;
    int midpoint;
    cv::Point leftx_base,rightx_base;
    //选择滑窗个数
    int nwindows = 9;
    //设置窗口高度
    int window_height = int(src.rows/nwindows);
    //设置窗口宽度
    int margin=50;
    //设置非零像素坐标最少个数
    int minpix=50;
    //TODO 加入if设置图像连续性，如果leftx_current和rightx_current为零，则认为第一次执行，需要计算该两点，如果已经计算了，则不许再次计算。
    //rowrange图像区域分割
    //将图像处理为一行，以行相加为方法
    cv::reduce(src.rowRange(src.rows/2,src.rows),hist,0,cv::REDUCE_SUM,CV_32S);
    midpoint=int(hist.cols/2);
    //将hist分为左右分别储存，并找出最大值
    //minMaxIdx针对多通道，minMaxLoc针对单通道
    cv::minMaxLoc(hist.colRange(0,midpoint),NULL,NULL,NULL,&leftx_base);
    cv::minMaxLoc(hist.colRange(midpoint,hist.cols),NULL,NULL,NULL,&rightx_base);
    //左右车道线基础点
    leftx_current=leftx_base.x;
    rightx_current=rightx_base.x+midpoint;
    // 提前存入该基础点坐标
    lpoint.push_back(cv::Point(leftx_current,src.rows));
    rpoint.push_back(cv::Point(rightx_current,src.rows));
    for(int i=0;i<nwindows;i++){
        int win_y_low=src.rows-(i+1)*window_height;
        //计算选框x坐标点，并将计算结果限制在图像坐标内
        int win_xleft_low = leftx_current - margin;
        win_xleft_low=win_xleft_low>0?win_xleft_low:0;
        win_xleft_low=win_xleft_low<src.rows?win_xleft_low:src.rows;
        //int win_xleft_high = leftx_current + margin;
        int win_xright_low = rightx_current - margin;
        win_xright_low=win_xright_low>0?win_xright_low:0;
        win_xright_low=win_xright_low<src.rows?win_xright_low:src.rows;
        //int win_xright_high = rightx_current + margin;
        //NOTE要确保参数都大于0，且在src图像范围内，不然会报错
        //NOTE 设置为ROI矩形区域选择
        l=src(cv::Rect(win_xleft_low,win_y_low,2*margin,window_height));
        r=src(cv::Rect(win_xright_low,win_y_low,2*margin,window_height));
        //NOTE 把像素值不为零的像素坐标存入矩阵
        cv::findNonZero(l,nonzerol);
        cv::findNonZero(r,nonzeror);
        //计算每个选框的leftx_current和rightx_current中心点
        if(nonzerol.size()>minpix){
            int leftx=0;
            for(auto& n:nonzerol){
                leftx+=n.x;
            }
            leftx_current=win_xleft_low+leftx/nonzerol.size();
        }
        if(nonzeror.size()>minpix){
            int rightx=0;
            for(auto& n:nonzeror){
                rightx+=n.x;
            }
            rightx_current=win_xright_low+rightx/nonzeror.size();
        }
        //将中心点坐标存入容器
        lpoint.push_back(cv::Point(leftx_current,win_y_low));
        rpoint.push_back(cv::Point(rightx_current,win_y_low));
    }
    //拟合左右车道线坐标
    cv::Mat leftx = polyfit(lpoint,2);
    cv::Mat rightx = polyfit(rpoint,2);
    //计算拟合曲线坐标
    lp=polyval(leftx,lpoint,2);
    rp=polyval(rightx,rpoint,2);
    //计算车道偏离距离
    int lane_width=abs(rpoint.front().x-lpoint.front().x);
    double lane_xm_per_pix=3.7/lane_width;
    double veh_pos=(((rpoint.front().x+lpoint.front().x)*lane_xm_per_pix)/2);
    double cen_pos=((src.cols*lane_xm_per_pix)/2);
    distance_from_center=veh_pos-cen_pos;
    // cout<<"dis"<<distance_from_center<<endl;
    // cout<<lp<<endl;
}

/*
@param src 为原始图像
@param dst 为输出的图像
@param channel 为通道选择
@param thresh_min 为阈值
@param thresh_max 为最大值
@NOTE 该函数用与提取LUV通道
*/
void luv_select(const cv::Mat& src,cv::Mat& dst,const char& channel='l',const int& thresh_min=0,const int& thresh_max=255){
    cv::Mat luv,grad;
    vector<cv::Mat> channels;
    cv::cvtColor(src,luv,cv::COLOR_RGB2Luv);
    cv::split(luv,channels);

    switch (channel)
    {
        case 'l':
            grad=channels.at(0);
            break;
        case 'u':
            grad=channels.at(1);
            break;
        case 'v':
            grad=channels.at(2);
            break;
    }
    cv::inRange(grad,thresh_min,thresh_max,dst);
    // cv::threshold(grad,dst,thresh_min,thresh_max,cv::THRESH_BINARY);
}


/*
@param src 为原始图像
@param lp 输入为左车道线坐标
@param rp 输入为右车道线坐标
@param Minv 输入为反变形矩阵
@param distance_from_center 输入为车道偏离距离
@NOTE 该函数用于绘制车道线和可行使区域
*/
void draw_area(const cv::Mat& src,vector<cv::Point>& lp,vector<cv::Point>& rp,const cv::Mat& Minv,double& distance_from_center){
    vector<cv::Point> rflip,ptr;
    cv::Mat colormask=cv::Mat::zeros(src.rows,src.cols,CV_8UC3);
    cv::Mat dst,midst;
    //绘制车道线
    cv::polylines(colormask,lp,false,cv::Scalar(0,255,0),5);
    cv::polylines(colormask,rp,false,cv::Scalar(0,0,255),5);
    //反转坐标，以便绘制填充区域
    cv::flip(rp,rflip,1);
    //拼接坐标
    cv::hconcat(lp,rflip,ptr);
    //绘制填充区域
    const cv::Point* em[1]={&ptr[0]};
    int nop=(int)ptr.size();
    cv::fillPoly(colormask,em,&nop,1,cv::Scalar(200,200,0));
    //反变形
    cv::warpPerspective(colormask,midst,Minv,src.size(),cv::INTER_LINEAR);
    //将车道线图片和原始图片叠加
    cv::addWeighted(src,1,midst,0.3,0,dst);
    //绘制文字
    cv::putText(dst,"distance bias:"+to_string(distance_from_center)+"m",cv::Point(50,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255),2);
    // cv::imshow("video",dst);

    cv::imwrite("area2.jpg", dst);
    // cv::waitKey(10000);
}

static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();
    // Run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
    t = (double) getTickCount() - t;
    cout << "detection time = " << (t*1000./cv::getTickFrequency()) << " ms" << endl;

    for(size_t i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];

        size_t j;
        // Do not add small detections inside a bigger detection.
        for ( j = 0; j < found.size(); j++ )
            if ( j != i && (r & found[j]) == r )
                break;

        if ( j == found.size() )
            found_filtered.push_back(r);
    }

    for (size_t i = 0; i < found_filtered.size(); i++)
    {
        Rect r = found_filtered[i];

        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rectangle(img, r.tl(), r.br(), cv::Scalar(255, 0, 0), 3);
    }
}


int main( int argc, char** argv ){
  // show help
  if(argc<2){
    cout<<
      " Usage: tracker <video_name>\n"
      " examples:\n"
      " example_tracking_kcf Bolt/img/%04d.jpg\n"
      " example_tracking_kcf faceocc2.webm\n"
      << endl;
    return 0;
  }
  // declares all required variables
  Rect2d roi;
  Mat frame;
  // create a tracker object
  Ptr<Tracker> tracker = TrackerKCF::create();
  // set input video
  std::string video = argv[1];
  VideoCapture cap(video);
  // get bounding box
  cap >> frame;
  roi=selectROI("tracker",frame);
  //quit if ROI was not selected
  if(roi.width==0 || roi.height==0)
    return 0;
  // initialize the tracker
  tracker->init(frame,roi);
  // perform the tracking process
  printf("Start the tracking process, press ESC to quit.\n");
  for ( ;; ){
    // get frame from the video
    cap >> frame;
    // stop the program if no more images
    if(frame.rows==0 || frame.cols==0)
      break;
    // update the tracking result
    tracker->update(frame,roi);
    // draw the tracked object
    rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
    // show image with the tracked object
    imshow("tracker",frame);
    //quit on ESC button
    if(waitKey(1)==27)break;
  }
  return 0;
}



// compile cmd: g++ -std=c++11 mylinedetect.cpp  -o resize  `pkg-config --cflags --libs opencv` && ./resize
