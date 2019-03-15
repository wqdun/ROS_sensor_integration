#include <iostream>
#include <opencv2/opencv.hpp>

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



int main(int argc, char **argv) {
    const std::string _filename(argv[1]);
    const char* filename = _filename.c_str();
    Mat image, dst;
    //image = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    image = imread(filename, CV_LOAD_IMAGE_COLOR);
    assert(!(image.empty()));

    imshow("image", image);
    // cout << "cvWaitKey(0): " << cvWaitKey(2000) << "\n";

    //image.cols 为图像的宽度 image.rows 为图像的高度
    int w = image.cols;
    int h = image.rows;
    std::cout<<"Image size:"<<w <<" * "<<h<<std::endl;
    imageResize(image, &dst, w / 2, 0);

    std::cout<<"new Image size:"<<dst.cols <<" * "<<dst.rows<<std::endl;
    vector<int> compression_params;
    //JPEG，参数为CV_IMWRITE_JPEG_QUALITY，值是从0到100，值越小压缩的越多
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);
    //imshow("dstImage", dst);
    imwrite("dstImage.jpg",dst,compression_params);


    // sobel边缘识别
    cv:Mat _sobel;
    abs_sobel_thresh(image, _sobel, 'x', 55, 200);
    imwrite(_filename + "_sobel.jpg", dst, compression_params);

    // cout << "cvWaitKey(0): " << cvWaitKey(0) << "\n";

    cv::Mat _mag;
    mag_thresh(image, _mag, 3, 45, 150);
    imwrite(_filename + "_mag.jpg", _mag, compression_params);

    cv::Mat _luv;
    luv_select(image, _luv, 'l', 180, 255);
    imwrite(_filename + "_luv.jpg", _luv, compression_params);

    cv::Mat _hls;
    hls_select(image, _hls, 's',160,255);
    imwrite(_filename + "_hls.jpg", _hls, compression_params);

    cv::Mat imgout( (_sobel & _mag & _luv) | (_hls & _luv) );

    // 车道线坐标点
    std::vector<cv::Point> lp, rp;
    // 车道线基坐标点x轴
    int rightx_current, leftx_current;
    double distance_from_center;
    find_line(imgout, lp, rp, rightx_current, leftx_current, distance_from_center);

    // 变形基础点
    vector<cv::Point2f> src={cv::Point2f(203,720),
                            cv::Point2f(585,460),
                            cv::Point2f(695,460),
                            cv::Point2f(1127,720)};
    vector<cv::Point2f> dst_param= {cv::Point2f(320,720),
                            cv::Point2f(320,0),
                            cv::Point2f(960,0),
                            cv::Point2f(960,720)};
    cv::Mat M, Minv;
    get_M_Minv(src, dst_param, M, Minv);

    draw_area(image, lp, rp, Minv, distance_from_center);


    cv::Mat _warp;
    cv::warpPerspective(image, _warp, M, image.size(), cv::INTER_LINEAR);
    imwrite(_filename + "_warp.jpg", _warp, compression_params);

    // sobel边缘识别
    cv::Mat _sobel2;
    abs_sobel_thresh(_warp, _sobel2, 'x', 55, 200);
    imwrite(_filename + "_sobel2.jpg", _sobel2, compression_params);

    // cout << "cvWaitKey(0): " << cvWaitKey(0) << "\n";

    cv::Mat _mag2;
    mag_thresh(_warp, _mag2, 3, 45, 150);
    imwrite(_filename + "_mag2.jpg", _mag2, compression_params);

    cv::Mat _luv2;
    luv_select(_warp, _luv2, 'l', 180, 255);
    imwrite(_filename + "_luv2.jpg", _luv2, compression_params);

    cv::Mat _hls2;
    hls_select(_warp, _hls2, 's',160,255);
    imwrite(_filename + "_hls2.jpg", _hls2, compression_params);

    cv::Mat imgout2( (_sobel2 & _mag2 & _luv2) | (_hls2 & _luv2) );

    find_line(imgout2, lp, rp, rightx_current, leftx_current, distance_from_center);
    draw_area(image, lp, rp, Minv, distance_from_center);


    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_RGB2GRAY);
    cv::imwrite(_filename + "_gray.jpg", imageGray, compression_params);

    cv::Mat imageBinary;
    cv::threshold(imageGray, imageBinary, 160, 255.0, CV_THRESH_BINARY);
    cv::imwrite(_filename + "_binary.jpg", imageBinary, compression_params);

    cv::Mat imageAdaptiveBinary;
    cv::adaptiveThreshold(imageGray, imageAdaptiveBinary, 255.0, ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 11, 3.);
    cv::imwrite(_filename + "_adaptive_binary.jpg", imageAdaptiveBinary, compression_params);

    const int structElementSize = 1;
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1) );
    cv::Mat imageErode;
    cv::erode(imageBinary, imageErode, element);
    cv::imwrite(_filename + "_erode.jpg", imageErode, compression_params);

    cv::Mat imageDilate;
    cv::dilate(imageErode, imageDilate, element);
    cv::imwrite(_filename + "_dilate.jpg", imageDilate, compression_params);

    cv::Mat imageCanny;
    cv::Canny(imageDilate, imageCanny, 50, 120);
    cv::imwrite(_filename + "_Canny.jpg", imageCanny, compression_params);


    cv::Mat imageConvertCanny;
    cv::cvtColor(imageCanny, imageConvertCanny, cv::COLOR_GRAY2BGR);
    cv::imwrite(_filename + "_CvtCanny.jpg", imageConvertCanny, compression_params);

    std::vector<Vec2f> lines;
    cv::HoughLines(imageCanny, lines, 1, CV_PI / 180, 150);
    //画出直线
    for (size_t i = 0; i < lines.size(); ++i) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        //此句代码的OpenCV2版为:
        //line( imageCanny, pt1, pt2, Scalar(55,100,195), 1, CV_AA);
        //此句代码的OpenCV3版为:
        line( imageCanny, pt1, pt2, Scalar(255, 255, 255), 1, LINE_AA);
    }

    cv::imshow("Result:", imageCanny);
    cv::waitKey(0);



    return 0;
}

// compile cmd: g++ -std=c++11 mylinedetect.cpp  -o resize  `pkg-config --cflags --libs opencv` && ./resize
