//
// Created by root on 9/27/17.
//

#include <jni.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <system.h>
#include "global_param.hpp"
#include <string>
#ifdef __cplusplus
extern "C" {
#endif
using namespace cv;
using namespace std;
vinssystem mSystem;

JNIEXPORT jintArray JNICALL Java_com_example_root_ai2_MainActivity_ImageProc(JNIEnv *env, jclass obj, jintArray buf, jint w, jint h, jdouble time){
    jboolean ptfalse = false;
    jint* srcBuf = env->GetIntArrayElements(buf, &ptfalse);
    if(srcBuf == NULL){
        return 0;
    }
    int size=w * h;

    Mat srcImage(h, w, CV_8UC4, (unsigned char*)srcBuf);
    Mat grayImage;
    cvtColor(srcImage, grayImage, COLOR_BGRA2GRAY);
    double ctime=time;
    //////////////////////////////////////////////////////////////feature tracking////////////////////////////////////////////////////////////////
    Mat resImage=mSystem.inputImage(srcImage,ctime);
    //////////////////////////////////////////////////////////////feature tracking////////////////////////////////////////////////////////////////
    //cvtColor(grayImage, srcImage, COLOR_GRAY2BGRA);
    int ressize=resImage.cols*resImage.rows;
    jintArray result = env->NewIntArray(ressize);
    jboolean ptfalse2=false;
    jint* resBuf = env->GetIntArrayElements(result, &ptfalse2);
    Mat resImage2(resImage.rows,resImage.cols,CV_8UC4,(unsigned char*)resBuf);
    resImage.copyTo(resImage2);


    env->SetIntArrayRegion(result, 0, ressize, resBuf);
    env->ReleaseIntArrayElements(buf, srcBuf, 0);


    return result;
}

JNIEXPORT void JNICALL Java_com_example_root_ai2_MainActivity_IMUProc(JNIEnv *env, jclass obj, jdouble header, jdouble wx, jdouble wy, jdouble wz, jdouble ax, jdouble ay, jdouble az){

    double cheader=header;
    double cwx=wx;
    double cwy=wy;
    double cwz=wz;
    double cax=ax;
    double cay=ay;
    double caz=az;

        /*FILE* fp = fopen("/storage/emulated/0/Android/data/com.example.root.mainloop/files/imu1.csv","a");
        fprintf(fp,"%f,%f,%f,%f,%f,%f,%f\n",cheader,cwx,cwy,cwz,cax,cay,caz);
        fclose(fp);*/

    ImuConstPtr imu_msg = new IMU_MSG();

    imu_msg->header = cheader;
    imu_msg->acc(0) = cax;
    imu_msg->acc(1) = cay;
    imu_msg->acc(2) = caz;
    imu_msg->gyr(0) = cwx;
    imu_msg->gyr(1) = cwy;
    imu_msg->gyr(2) = cwz;

    mSystem.inputIMU(imu_msg);
}

JNIEXPORT jintArray JNICALL Java_com_example_root_ai2_MainActivity_VINSInit(JNIEnv *env, jclass obj, jstring vocfile, jstring patternfile, jstring settingfile){
    const char* cvocfile=0;
    cvocfile=env->GetStringUTFChars(vocfile,0);
    string svocfile(cvocfile);
    const char* cpatternfile=0;
    cpatternfile=env->GetStringUTFChars(patternfile,0);
    string spatternfile(cpatternfile);
    const char* csettingfile=0;
    csettingfile=env->GetStringUTFChars(settingfile,0);
    string ssettingfile(csettingfile);
    mSystem.create(svocfile,spatternfile,ssettingfile);
    FILE* fp = fopen("/storage/emulated/0/Android/data/com.example.root.mainloop/files/temporal_calibrate.csv","w");

                        fclose(fp);
    //FILE* fp=fopen("/storage/emulated/0/Android/data/com.example.root.mainloop/timebuf.txt","w");
    //fclose(fp);
}

JNIEXPORT void JNICALL Java_com_example_root_ai2_MainActivity_WriteKeyFrameDataBase(JNIEnv *env, jclass obj, jstring keyfile){
    const char* ckeyfile=0;
    ckeyfile=env->GetStringUTFChars(keyfile,0);
    //mSystem.keyframe_database.WriteKeyFrames(ckeyfile);
    vector<DRAWFRAME_DATA> frames_to_draw = mSystem.keyframe_database.frames_to_draw;
    FILE* fp = fopen(ckeyfile,"w");
    for(int i = 0; i < frames_to_draw.size(); i++)
    {
        Vector3d P_origin = frames_to_draw[i].P_origin;
        Quaterniond Q_origin = frames_to_draw[i].Q_origin.inverse();
        double header = frames_to_draw[i].header;
        fprintf(fp,"%f %f %f %f ",header,P_origin.x(),P_origin.y(),P_origin.z());
        fprintf(fp,"%f %f %f %f\n",Q_origin.x(),Q_origin.y(),Q_origin.z(),Q_origin.w());
    }
    fclose(fp);

    fp = fopen("/storage/emulated/0/Android/data/com.example.root.ai2/files/keyframeh2.txt","w");
    for(int i = 0; i < frames_to_draw.size(); i++)
    {
        Vector3d P_origin = frames_to_draw[i].P_origin;
        Quaterniond Q_origin = frames_to_draw[i].Q_origin;
        double header = frames_to_draw[i].header;
        fprintf(fp,"%f %f %f %f ",header,P_origin.x(),P_origin.y(),P_origin.z());
        fprintf(fp,"%f %f %f %f\n",Q_origin.x(),Q_origin.y(),Q_origin.z(),Q_origin.w());
    }
    fclose(fp);

    fp = fopen("/storage/emulated/0/Android/data/com.example.root.ai2/files/timeh.txt","w");
    for(int i = 0; i < frames_to_draw.size(); i++)
    {
        double header = frames_to_draw[i].header;
        double dt = frames_to_draw[i].dt;
        fprintf(fp,"%f %f\n",header,dt * 1000);
    }
    fclose(fp);
}

JNIEXPORT jfloatArray JNICALL Java_com_example_root_ai2_MainActivity_GetKeyFrames(JNIEnv *env, jclass obj){
/*    list<KeyFrame*>::iterator it;
    int arr_size = mSystem.keyframe_database.keyFrameList.size();
    jfloatArray result = env->NewFloatArray(arr_size*3 + 3);
    float* resBuf = new float [arr_size*3 + 3];
    int i = 0;
    for (it = mSystem.keyframe_database.keyFrameList.begin(); it != mSystem.keyframe_database.keyFrameList.end(); it++)
    {
         Vector3d P;
         Matrix3d R;
         (*it)->getPose(P, R);
         Vector3f P_float;
         P_float = P.cast<float>();
         float X = P_float.x();
         float Y = P_float.y();
         float Z = P_float.z();
         resBuf[3*i]=X;
         resBuf[3*i+1]=Y;
         resBuf[3*i+2]=Z;
         i++;
    }
    env->SetFloatArrayRegion(result,0,arr_size*3,resBuf);

    if(mSystem.mpEstimator->solver_flag == VINS::INITIAL)
    {
        resBuf[3*arr_size] = 0;
        resBuf[3*arr_size+1] = 0;
        resBuf[3*arr_size+2] = 0;
    }
    else
    {
        Vector3d P_real;
        P_real = mSystem.mpEstimator->r_drift * mSystem.mpEstimator->Ps[WINDOW_SIZE-1] + mSystem.mpEstimator->t_drift;
        Vector3f P_real_float;
        P_real_float = P_real.cast<float>();
        float X = P_real_float.x();
        float Y = P_real_float.y();
        float Z = P_real_float.z();
        resBuf[3*arr_size] = X;
        resBuf[3*arr_size+1] = Y;
        resBuf[3*arr_size+2] = Z;
    }
    //env->ReleasefloatArrayElements(result, resBuf, 0);
    return result;*/
    int arr_size = mSystem.keyframe_database.frames_to_draw.size();
    jfloatArray result = env->NewFloatArray(arr_size*3);
    float* resBuf = new float [arr_size*3];
    for(int i = 0;i < arr_size; i++)
    {
        Vector3d P = mSystem.keyframe_database.frames_to_draw[i].P_draw;
        Vector3f P_float;
        P_float = P.cast<float>();
        float X = P_float.x();
        float Y = P_float.y();
        float Z = P_float.z();
        resBuf[3*i]=X;
        resBuf[3*i+1]=Y;
        resBuf[3*i+2]=Z;
    }
    env->SetFloatArrayRegion(result,0,arr_size*3,resBuf);
    return result;
}

JNIEXPORT jfloatArray JNICALL Java_com_example_root_ai2_MainActivity_GetRotation(JNIEnv *env, jclass obj){
    jfloatArray result = env->NewFloatArray(5);
    float* resBuf = new float [5];
    resBuf[0] = mSystem.yaw_now;
    resBuf[1] = mSystem.pitch_now;
    resBuf[2] = mSystem.roll_now;
    if(mSystem.synchronized_flag == true)
    {
        resBuf[3] = 1;
    }
    else
    {
        resBuf[3] = 0;
    }
    resBuf[4] = mSystem.dt_initial + mSystem.mpEstimator->dt2;
    env->SetFloatArrayRegion(result,0,5,resBuf);
    return result;
}

JNIEXPORT jfloatArray JNICALL Java_com_example_root_ai2_MainActivity_GetPoints3d(JNIEnv *env, jclass obj){
    mSystem.m_points3d.lock();
    int arr_size = mSystem.points_3d.size();
    jfloatArray result = env->NewFloatArray(arr_size*3);
    float* resBuf = new float [arr_size*3];
    for(int i = 0;i < arr_size; i++)
    {
        Vector3d P = mSystem.points_3d[i];
        Vector3f P_float;
        P_float = P.cast<float>();
        float X = P_float.x();
        float Y = P_float.y();
        float Z = P_float.z();
        resBuf[3*i]=X;
        resBuf[3*i+1]=Y;
        resBuf[3*i+2]=Z;
    }
    env->SetFloatArrayRegion(result,0,arr_size*3,resBuf);
    mSystem.m_points3d.unlock();
    return result;
}

JNIEXPORT jdouble JNICALL Java_com_example_root_ai2_MainActivity_RMS(JNIEnv *env, jclass obj, jdoubleArray arr_truths){
    jboolean ptfalse = false;
    jdouble* truths = env->GetDoubleArrayElements(arr_truths, &ptfalse);
    if(truths == NULL){
        return -1;
    }
    jint length;
    length = env->GetArrayLength(arr_truths);
    vector<DRAWFRAME_DATA> frames_to_draw = mSystem.keyframe_database.frames_to_draw;
    int j = 0;
    for( j = 0; j < frames_to_draw.size() ; j++ )
    {
        if(frames_to_draw[j].header > truths[0])
        {
            break;
        }
    }
    int startframe = j;
    double* selected_truth = new double[frames_to_draw.size() * 3];
    for( int i = 0; i < length/4 ; i++)
    {
        if(truths[4 * i] < frames_to_draw[j].header)
        {
            continue;
        }
        else
        {
            if(i == 0){return -1;}
            else
            {
                double dt = truths[4 * i] - truths[4 * (i - 1)];
                double dt1 = truths[4 * i] - frames_to_draw[j].header;
                double truthX = dt1/dt * truths[4 * (i - 1) + 1] + (1 - dt1/dt) * truths[4 * i + 1];
                double truthY = dt1/dt * truths[4 * (i - 1) + 2] + (1 - dt1/dt) * truths[4 * i + 2];
                double truthZ = dt1/dt * truths[4 * (i - 1) + 3] + (1 - dt1/dt) * truths[4 * i + 3];
                selected_truth[3 * j] = truthX;
                selected_truth[3 * j + 1] = truthY;
                selected_truth[3 * j + 2] = truthZ;
                j++;
            }
            if(j == frames_to_draw.size())
            {
                break;
            }
        }
    }
    int endframe = j;// actually loop until endframe - 1
    Vector3d mean_truth;
    Vector3d mean_vins;
    mean_truth.setZero();
    mean_vins.setZero();
    for( int i = startframe; i < endframe ; i++)
    {
        Vector3d P_origin = frames_to_draw[i].P_origin;
        mean_vins += P_origin;
    }
    mean_vins = mean_vins / (double)(endframe - startframe);
    for( int i = startframe; i < endframe ; i++)
    {
        frames_to_draw[i].P_origin -= mean_vins;
    }
    for( int i = startframe; i < endframe ; i++)
    {
        Vector3d P_truth = Vector3d(selected_truth[3 * i], selected_truth[3 * i + 1], selected_truth[3 * i + 2]);
        mean_truth += P_truth;
    }
    mean_truth = mean_truth / (double)(endframe - startframe);
    for( int i = startframe; i < endframe ; i++)
    {
        Vector3d P_truth = Vector3d(selected_truth[3 * i], selected_truth[3 * i + 1], selected_truth[3 * i + 2]);
        P_truth -= mean_truth;
        selected_truth[3 * i] = P_truth[0];
        selected_truth[3 * i + 1] = P_truth[1];
        selected_truth[3 * i + 2] = P_truth[2];
    }
    Matrix3d A;
    A.setZero();
    for( int i = startframe; i < endframe ; i++)
    {
        Vector3d P_truth = Vector3d(selected_truth[3 * i], selected_truth[3 * i + 1], selected_truth[3 * i + 2]);
        Vector3d P_origin = frames_to_draw[i].P_origin;
        A = A + P_truth * P_origin.transpose();
    }
    A = A / (double)(endframe - startframe);
    JacobiSVD<Matrix3d> svd(A, ComputeThinU | ComputeThinV);
    Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    double sum_sqerror = 0;
    for( int i = startframe; i < endframe ; i++)
    {
         Vector3d P_truth = Vector3d(selected_truth[3 * i], selected_truth[3 * i + 1], selected_truth[3 * i + 2]);
         Vector3d P_origin = frames_to_draw[i].P_origin;
         Vector3d P_error = P_truth - R * P_origin;
         sum_sqerror += P_error.norm() * P_error.norm();
    }
    double ave_error = sqrt(sum_sqerror / (double)(endframe - startframe));
    env->ReleaseDoubleArrayElements(arr_truths, truths, 0);
    return ave_error;
}


#ifdef __cplusplus
}
#endif