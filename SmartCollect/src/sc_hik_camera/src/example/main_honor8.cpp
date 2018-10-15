#include "system.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "global_param.hpp"
#include <string>

using namespace std;

bool my_compare (const pair<double,vector<Box>> &lhs, const pair<double,vector<Box>> &rhs) {
    return lhs.first < rhs.first;
}

void LoadImages(const string &strCamPath, vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMUData(const string &strIMUPath, vector<IMUMeasument> &vIMUMeasuments,vector<double> &vTimeStamps);

std::vector<std::string> split(const  std::string& s, const std::string& delim);

void IMUProc(IMUMeasument oneIMUMeasurement, double header, vinssystem* mpSystem);

void ReadObjects(const string &strFolderPath,vector<pair<double,vector<Box>>>& boxes);
cv::Mat ImageProc(cv::Mat srcImage, double header, vinssystem* mpSystem, pair<double,vector<Box>> onebox);
int main(int argc, char** argv){




    const char* cvocfile = "/home/nvidia/Documents/VINS-pc/config/briefk10l6.bin";
    const char* cpatternfile = "/home/nvidia/Documents/VINS-pc/config/briefpattern.yml";
    const char* csettingfile = "/home/nvidia/Documents/VINS-pc/config/vehicle-dikuencoder.yaml";
    string svocfile(cvocfile);
    string spatternfile(cpatternfile);
    string ssettingfile(csettingfile);
    vinssystem mSystem(svocfile,spatternfile,ssettingfile);
    cout << "OK" << endl;
    //cv::namedWindow("VINS: Current Frame",CV_WINDOW_NORMAL);

    cv::FileStorage fSettings = cv::FileStorage(ssettingfile, cv::FileStorage::READ);
    double ImageScale;
    fSettings["ImageScale"] >> ImageScale;
    int iUseBoxes;
    fSettings["DetectMovingObjects"] >> iUseBoxes;
    bool bUseBoxes = (iUseBoxes == 1);
    double dt_cam_imu;
    fSettings["dtbig"] >> dt_cam_imu;

    //const char* cdatafolder = "/home/ljx/Documents/evidence8/";
    //const char* cdatafolder = "/home/ljx/Documents/1028_yongfeng/Worknew/scale/";
    //const char* cdatafolder = "/home/ljx/Documents/siweidikunew/Image/scale/";
    //string sdatafolder(cdatafolder);
    string sdatafolder;
    fSettings["datafolder"] >> sdatafolder;
    vector<string> vstrImages;
    vector<double> vTimeStamps;
    LoadImages(sdatafolder,vstrImages,vTimeStamps);


    vector<IMUMeasument> vIMUMeasuments;
    vector<double> vTimestampsIMU;
    LoadIMUData(sdatafolder,vIMUMeasuments,vTimestampsIMU);

    vector<pair<double,vector<Box>>> boxes;
    if(bUseBoxes == true)
    {

        ReadObjects(sdatafolder,boxes);
        cout << "boxes number: " << boxes.size() << endl;

        for(int i = 0; i < boxes.size(); i++)
        {
            for(int j = 0; j < boxes[i].second.size(); j++)
            {
                boxes[i].second[j].x0 = boxes[i].second[j].x0 * ImageScale;
                boxes[i].second[j].y0 = boxes[i].second[j].y0 * ImageScale;
                boxes[i].second[j].x1 = boxes[i].second[j].x1 * ImageScale;
                boxes[i].second[j].y1 = boxes[i].second[j].y1 * ImageScale;
            }
        }
    }


    ////////////////added for siweidiku///////////////////
    for(int i = 0; i < vTimestampsIMU.size(); i++)
    {
        vTimestampsIMU[i] -= dt_cam_imu;
    }
    ///////////////////////////////////////////////////////

    while(vTimestampsIMU.size() && vTimeStamps.size())
    {

        if(vTimestampsIMU[0] < vTimeStamps[0])
        {
            IMUProc(vIMUMeasuments[0],vTimestampsIMU[0],&mSystem);
            vIMUMeasuments.erase(vIMUMeasuments.begin());
            vTimestampsIMU.erase(vTimestampsIMU.begin());
            usleep(1200);
        }
        else
        {
            cv::Mat srcImage = cv::imread(vstrImages[0],CV_LOAD_IMAGE_UNCHANGED);
            if(bUseBoxes == true)
            {
                cv::Mat resImage = ImageProc(srcImage.clone(),vTimeStamps[0],&mSystem,boxes[0]);
                boxes.erase(boxes.begin());
            } else{
                vector<Box> emptybox;
                pair<double,vector<Box>> emptyonebox;
                emptyonebox.first = vTimeStamps[0];
                emptyonebox.second = emptybox;
                cv::Mat resImage = ImageProc(srcImage.clone(),vTimeStamps[0],&mSystem, emptyonebox);
            }

            vstrImages.erase(vstrImages.begin());
            vTimeStamps.erase(vTimeStamps.begin());
            //cv::imshow("VINS: Current Frame",resImage);
            cout << "ImagesLeft" << vTimeStamps.size() << endl;
            usleep(35000);
        }
    }


    return 0;
}

std::vector<std::string> split(const  std::string& s, const std::string& delim) {
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}

void LoadImages(const string &strCamPath, vector<string> &vstrImages, vector<double> &vTimeStamps) {
    string strPathcsv = strCamPath + "data.csv";
//    printf("%s",strPathcsv.c_str());
    ifstream fcsv;
    fcsv.open(strPathcsv.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    //string s0;
    //getline(fcsv,s0);
    while(!fcsv.eof())
    {
        string s;
        getline(fcsv,s);
        vector<string> names= split(s,",");
        if(!s.empty())
        {
            vstrImages.push_back(names[1]);
            vTimeStamps.push_back(atol(names[0].c_str())/1e9);

        }
    }
}

void LoadIMUData(const string &strIMUPath, vector<IMUMeasument> &vIMUMeasuments,vector<double> &vTimeStamps) {
    string strPathcsv = strIMUPath + "imu0.csv";
//    printf("%s",strPathcsv.c_str());

    ifstream fcsv;
    fcsv.open(strPathcsv.c_str());
    vTimeStamps.reserve(5000);
    vIMUMeasuments.reserve(5000);
    //string s0;
    //getline(fcsv,s0);
    while(!fcsv.eof())
    {
        string s;
        getline(fcsv,s);
        vector<string> names = split(s,",");
        if(!s.empty())
        {

            IMUMeasument measument = {strtod(names[4].c_str(),NULL),strtod(names[5].c_str(),NULL),strtod(names[6].c_str(),NULL),
                                      strtod(names[1].c_str(),NULL),strtod(names[2].c_str(),NULL),strtod(names[3].c_str(),NULL),strtod(names[7].c_str(), NULL)};
            vIMUMeasuments.push_back(measument);

            vTimeStamps.push_back(atol(names[0].c_str())/1e9); //+0.01都不行  要毫秒级的同步精度

        }
    }

}

void IMUProc(IMUMeasument oneIMUMeasurement, double header, vinssystem* mpSystem)
{
    ImuConstPtr imu_msg = new IMU_MSG();
    imu_msg->header = header;
    imu_msg->acc(0) = oneIMUMeasurement.a_x;
    imu_msg->acc(1) = oneIMUMeasurement.a_y;
    imu_msg->acc(2) = oneIMUMeasurement.a_z;
    imu_msg->gyr(0) = oneIMUMeasurement.w_x;
    imu_msg->gyr(1) = oneIMUMeasurement.w_y;
    imu_msg->gyr(2) = oneIMUMeasurement.w_z;
    imu_msg->encoder_v = oneIMUMeasurement.encoder_v;
    mpSystem->inputIMU(imu_msg);
}

cv::Mat ImageProc(cv::Mat srcImage, double header, vinssystem* mpSystem,pair<double,vector<Box>> onebox)
{
    cv::Mat resImage = mpSystem->inputImage(srcImage,header,onebox);
    return resImage.clone();
}

void ReadObjects(const string &strFolderPath,vector<pair<double,vector<Box>>>& boxes)
{
    string strBoxPath = strFolderPath + "detectresults.txt";
    ifstream f;
    f.open(strBoxPath.c_str());
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string imgpath;
            int nboxes;
            ss >> imgpath;
            ss >> nboxes;
            string strtimestamp(imgpath.substr(0,imgpath.length()-4));
            //vector<string> tsparts = split(strtimestamp,'.');
            //double timestamp_first = stod(tsparts[0]);
            //double timestamp_second = stod(tsparts[1]);
            double timestampnano = stod(strtimestamp);
            double timestamp = timestampnano * 0.000000001;
            pair<double,vector<Box>> boxoneimg;
            boxoneimg.first = timestamp;
            vector<Box> vboxoneimg;
            for(int i = 0; i < nboxes; i++)
            {
                string sbox;
                getline(f,sbox);
                if(!sbox.empty())
                {
                    stringstream ssbox;
                    ssbox << sbox;
                    Box onebox;
                    ssbox >> onebox.x0;
                    ssbox >> onebox.y0;
                    ssbox >> onebox.x1;
                    ssbox >> onebox.y1;
                    ssbox >> onebox.category;
                    vboxoneimg.push_back(onebox);
                }
            }
            boxoneimg.second = vboxoneimg;
            boxes.push_back(boxoneimg);
        }
    }
    sort(boxes.begin(),boxes.end(),my_compare);

}
