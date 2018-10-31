#ifndef __DATA_OUTPUT__
#define __DATA_OUTPUT__
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#define OUTPUT_PATH 100
#ifdef OUTPUT_PATH

//#define USE_OPENCV 100
#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

class COutputData
{
public:
    COutputData(string outputpath);
    ~COutputData();
public: 
    void saveResultImage(Mat MatImage, string name);
    void saveResultFile(string filevalue, string name);
public: 
    string OutputPath;
    string OutputFile;
    fstream OutStream;
};
#endif 

#endif 

#endif
