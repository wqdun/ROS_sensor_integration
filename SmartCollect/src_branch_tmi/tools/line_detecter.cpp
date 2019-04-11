#include<cv.h>
#include<cxcore.h>
#include<highgui.h>
#include"mylinedetect.h"

#include<cstdio>
#include<iostream>
using namespace std;

int main(){
    //声明IplImage指针
    IplImage* pFrame = NULL;
    IplImage* pCutFrame = NULL;
    IplImage* pCutFrImg = NULL;
    //声明CvCapture指针
    CvCapture* pCapture = NULL;
    //声明CvMemStorage和CvSeg指针
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* lines = NULL;
    //生成视频的结构
    VideoWriter writer("result.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(856, 480));
    //当前帧数
    int nFrmNum = 0;
    //裁剪的天空高度
    int CutHeight = 310;
    //窗口命名
    cvNamedWindow("video", 1);
    cvNamedWindow("BWmode", 1);
    //调整窗口初始位置
    cvMoveWindow("video", 300, 0);
    cvMoveWindow("BWmode", 300, 520);
    //不能打开则退出
    if (!(pCapture = cvCaptureFromFile("lane.avi"))){
        fprintf(stderr, "Can not open video file\n");
        return -2;
    }
    //每次读取一桢的视频
    while (pFrame = cvQueryFrame(pCapture)){
        //设置ROI裁剪图像
        cvSetImageROI(pFrame, cvRect(0, CutHeight, pFrame->width, pFrame->height - CutHeight));
        nFrmNum++;
        //第一次要申请内存p
        if (nFrmNum == 1){
            pCutFrame = cvCreateImage(cvSize(pFrame->width, pFrame->height - CutHeight), pFrame->depth, pFrame->nChannels);
            cvCopy(pFrame, pCutFrame, 0);
            pCutFrImg = cvCreateImage(cvSize(pCutFrame->width, pCutFrame->height), IPL_DEPTH_8U, 1);
            //转化成单通道图像再处理
            cvCvtColor(pCutFrame, pCutFrImg, CV_BGR2GRAY);
        }
        else{
            //获得剪切图
            cvCopy(pFrame, pCutFrame, 0);
#if 0       //反透视变换
            //二维坐标下的点，类型为浮点
            CvPoint2D32f srcTri[4], dstTri[4];
            CvMat* warp_mat = cvCreateMat(3, 3, CV_32FC1);
            //计算矩阵反射变换
            srcTri[0].x = 10;
            srcTri[0].y = 20;
            srcTri[1].x = pCutFrame->width - 5;
            srcTri[1].y = 0;
            srcTri[2].x = 0;
            srcTri[2].y = pCutFrame->height - 1;
            srcTri[3].x = pCutFrame->width - 1;
            srcTri[3].y = pCutFrame->height - 1;
            //改变目标图像大小
            dstTri[0].x = 0;
            dstTri[0].y = 0;
            dstTri[1].x = pCutFrImg->width - 1;
            dstTri[1].y = 0;
            dstTri[2].x = 0;
            dstTri[2].y = pCutFrImg->height - 1;
            dstTri[3].x = pCutFrImg->width - 1;
            dstTri[3].y = pCutFrImg->height - 1;
            //获得矩阵
            cvGetPerspectiveTransform(srcTri, dstTri, warp_mat);
            //反透视变换
            cvWarpPerspective(pCutFrame, pCutFrImg, warp_mat);
#endif
            //前景图转换为灰度图
            cvCvtColor(pCutFrame, pCutFrImg, CV_BGR2GRAY);
            //二值化前景图
            cvThreshold(pCutFrImg, pCutFrImg, 80, 255.0, CV_THRESH_BINARY);
            //进行形态学滤波，去掉噪音
            cvErode(pCutFrImg, pCutFrImg, 0, 2);
            cvDilate(pCutFrImg, pCutFrImg, 0, 2);
            //canny变化
            cvCanny(pCutFrImg, pCutFrImg, 50, 120);
            //sobel变化
            //Mat pCutFrMat(pCutFrImg);
            //Sobel(pCutFrMat, pCutFrMat, pCutFrMat.depth(), 1, 1);
            //laplacian变化
            //Laplacian(pCutFrMat, pCutFrMat, pCutFrMat.depth());
#if 1       //0为下面的代码，1为上面的代码
    #pragma region Hough直线检测
            lines = cvHoughLines2(pCutFrImg, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 100, 15, 15);
            printf("Lines number: %d\n", lines->total);
            //画出直线
            for (int i = 0; i<lines->total; i++){
                CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
                double k = ((line[0].y - line[1].y)*1.0 / (line[0].x - line[1].x));
                cout<<"nFrmNum "<<nFrmNum<<" 's k = "<<k<<endl;
                if(!(abs(k)<0.1))//去掉水平直线
                    cvLine(pFrame, line[0], line[1], CV_RGB(255, 0, 0), 6, CV_AA);
            }
    #pragma endregion
#else
    #pragma region mylinedetect
            Mat edge(pCutFrImg);
            vector<struct line> lines = detectLine(edge, 60);
            Mat pFrameMat(pFrame);
            drawLines(pFrameMat, lines);
            namedWindow("mylinedetect", 1);
            imshow("mylinedetect", pFrameMat);
    #pragma endregion
#endif
            //恢复ROI区域
            cvResetImageROI(pFrame);
            //写入视频流
            writer << pFrame;
            //显示图像
            cvShowImage("video", pFrame);
            cvShowImage("BWmode", pCutFrImg);
            //按键事件，空格暂停，其他跳出循环
            int temp = cvWaitKey(2);
            if (temp == 32){
                while (cvWaitKey() == -1);
            }
            else if (temp >= 0){
                break;
            }
        }
    }
    //销毁窗口
    cvDestroyWindow("video");
    cvDestroyWindow("BWmode");
    //释放图像
    cvReleaseImage(&pCutFrImg);
    cvReleaseImage(&pCutFrame);
    cvReleaseCapture(&pCapture);

    return 0;
}