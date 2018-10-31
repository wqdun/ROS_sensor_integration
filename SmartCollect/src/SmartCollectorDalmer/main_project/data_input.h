#ifndef __DATA_INPUT__
#define __DATA_INPUT__
#include <iostream>
#include <fstream>
#include <sstream> 
#include <string> 
#include <vector> 

using namespace std;
#define JAPAN 100
#define INPUT_LIST 100
//#define DETECTION 100
#define DETECTIONSSD 100
//#define DETECTIONSSDCPU 100
//#define CLASSIFICATION 100
#define SSDOFVOC   100  
//#define POLEDETECTION  100
//#define MARKDETECTION 100
//#define  MARKLOAD 100
//#define  POLELOAD 100
//#define  LANDLOAD 100

#ifdef INPUT_LIST 
#ifdef CLASSIFICATION
    
#endif

class CInputData
{
public: 
    CInputData(string list);
	~CInputData(void);
public:
    bool   getNameList(void);
    int    getListSize(void);
public:
    vector<string> FileListVec;
    std::string file; 
    std::ifstream input_file;
#ifdef DETECTIONSSD
    std::string model_file_D_SSD;
    std::string weights_file_D_SSD;
    std::string mean_file_D_SSD;
    std::string mean_value_D_SSD;
    std::string file_type_D_SSD;
    std::string out_file_D_SSD;
    double confidence_threshold_D_SSD;
#endif
#ifdef DETECTIONSSDCPU 
    std::string model_file_D_SSDCPU;
    std::string weights_file_D_SSDCPU;
    std::string mean_file_D_SSDCPU;
    std::string mean_value_D_SSDCPU;
    std::string file_type_D_SSDCPU;
    std::string out_file_D_SSDCPU;
    double confidence_threshold_D_SSDCPU;
#endif 
 
public: 
    bool getFileName(const string path,string& name);
};


#endif 

#ifdef INPUT_PATH 

#endif 

#endif 
