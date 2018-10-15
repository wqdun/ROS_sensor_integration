//
//  global_param.h
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/20.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef global_param_h
#define global_param_h
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <queue>
#include <map>
using namespace Eigen;
using namespace std;
using namespace cv;

//true: use loop closure
//false don't use loop closure
#define LOOP_CLOSURE true // false

#define MIN_LOOP_NUM 20
#define LOOP_FREQ 3


//#define DEBUG_MODE false

//#define USE_OPTICAL_PREDICT false
//#define IPHONE_7_PLUS true
//
//#ifdef IPHONE_7_PLUS
//    #define FOCUS_LENGTH_Y ((double)(5.2667856579178056e+02))
//    #define PY ((double)(3.1528065186153088e+02))
//
//    #define FOCUS_LENGTH_X ((double)(5.2660065870168228e+02))
//    #define PX ((double)(2.4348176562199635e+02))
//#endif
//
//#ifdef IPHONE_6S
//    #define FOCUS_LENGTH_Y ((double)(5.4947719555646984e+0))
//    #define PY ((double)(3.2037938153759552e+02))
//
//    #define FOCUS_LENGTH_X ((double)(5.4881326214652529e+02))
//    #define PX ((double)(2.3852032763295512e+02))
//#endif

//    #define FOCUS_LENGTH_Y ((double)(507))
//    #define PY ((double)(247))

//    #define FOCUS_LENGTH_X ((double)(507))
//    #define PX ((double)(324))


#define WINDOW_SIZE 10
#define SIZE_POSE 7
#define SIZE_SPEEDBIAS  9
#define MEDIAN_WINDOW 5

#define SIZE_FEATURE 1

#define NUM_OF_F 1000
#define NUM_OF_CAM 1
#define SOLVER_TIME 0.03
#define FREQ 2

#define GRAVITY ((double)9.805)
//#define ACC_N ((double)1)  //0.02
//#define ACC_W ((double)1)
//#define GYR_N ((double)1)  //0.02
//#define GYR_W ((double)1)
//
//#define ACC_N ((double)0.5)  //0.02
//#define ACC_W ((double)0.002)
//#define GYR_N ((double)0.2)  //0.02
//#define GYR_W ((double)4.0e-5)

////#define GRAVITY ((double)9.81007)
//#define ACC_N ((double)0.0252)  //0.02
//#define ACC_W ((double)0.0017)
//#define GYR_N ((double)0.2)  //0.02
//#define GYR_W ((double)0.00002)



//#define BIAS_ACC_THRESHOLD ((double)0.5)
//#define BIAS_GYR_THRESHOLD ((double)0.1)
#define G_THRESHOLD ((double)1.0)
#define G_NORM ((double)9.805)
#define INIT_KF_THRESHOLD ((double)18)
#define SFM_R_THRESHOLD ((double)180)
//#define MAX_FEATURE_CNT 75
//extrinsic param
#define RIC_y ((double)0.0)
#define RIC_p ((double)0.0)
#define RIC_r ((double)90.0)
#define TIC_X ((double)0.0)
#define TIC_Y ((double)0.0)
#define TIC_Z ((double)0.0)

#define C_PI 3.1415926
#define USE_FIXED_POINTS true
#define TR 0



struct IMU_MSG {
    double header;
    Vector3d acc;
    Vector3d gyr;
    double encoder_v;
};

struct IMUMeasument {
    double a_x;
    double a_y;
    double a_z;
    double w_x;
    double w_y;
    double w_z;
    double encoder_v;
};
//#pragma GCC push_options
//#pragma GCC optimize ("O0")
struct IMG_MSG {
    double header;
    map<int, Matrix<double, 7, 1>> point_clouds;
};

struct IMG_DATA {
    double header;
    cv::Mat *image;
};

class Box
{
public:
    int x0;
    int y0;
    int x1;
    int y1;
    int category;
};

struct IMG_DATA_CACHE {
    double header;
    cv::Mat equ_image;
    cv::Mat *image;
};
struct VINS_DATA_CACHE {
    double header;
    Vector3f P;
    Matrix3f R;
};

typedef IMU_MSG* ImuConstPtr;
typedef IMG_MSG* ImgConstPtr;

class ConfigureReader
{
public:
    ConfigureReader(std::string strConfPath){


    }
};



/* IMU
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 
 */
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12,
    O_ENV = 15
};

#if 1
#define TS(name) int64 t_##name = cv::getTickCount()
#define TE(name) printf("TIMER_" #name ": %.2fms\n", \
1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()))
#else
#define TS(name)
#define TE(name)
#endif

#endif /* global_param_h */

