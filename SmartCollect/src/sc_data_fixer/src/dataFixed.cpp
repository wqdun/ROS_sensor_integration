#include "dataFixed.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

dataFixed::dataFixed(ros::NodeHandle node, ros::NodeHandle private_nh, int cinValue)
{
    imageCollectionHz = cinValue;
    totalFileNum = processNum = 0;
}

void dataFixed::initMemberVar() {
    LOG(INFO) << __FUNCTION__ << " start.";

    totalFileNum = processNum = 0;
    beganGPSTime = 0;
    endGPSTime = 0;
    minGPSTime = 100000;
    maxGPSTime = 0;
    minGPSTimeMark = 0;
    belongtoLidarProjectName.clear();
}

void dataFixed::fixProjectsData(const std::string &_projects)
{
    LOG(INFO) << __FUNCTION__ << " start, param: " << _projects;
    (void)initMemberVar();

    std::vector<std::string> projectArr;
    if(!_projects.empty() ) {
        (void)boost::split(projectArr, _projects, boost::is_any_of(",") );
    }

    for(auto &project: projectArr)
    {
        project = "/opt/smartc/record/" + project + "/";
    }

    std::vector<ontimeDataFormat> imuData;
    std::stringstream ss;
    unsigned long fileNum = 0;
    int returnValue;

    const size_t maxLine = 1000;
    char result[maxLine];
    std::vector<int> isProcess(100,0);
    std::string numString;

    for(size_t i = 0; i < projectArr.size(); ++i) {
        LOG(INFO) << "start to count project:" << projectArr[i] << " file Size";
        std::string processPath = projectArr[i] + "/Process";
        int existProcessMark = access(processPath.c_str(), F_OK);
        if( 0 == existProcessMark )
        {
            continue;
        }

        std::string cmd = "ls " + projectArr[i] + "/Rawdata/IMU/*_rt_track.txt";
        const size_t maxLine = 1000;
        char result[maxLine];
        FILE *fpin;
        if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
        {
            LOG(ERROR) << "Failed to " << cmd;
            return;
        }
        size_t rtImuFileNum = 0;
        while(1)
        {
            if(NULL == fgets(result, maxLine, fpin) )
            {
                LOG(INFO) << "Number of rt_track.txt: " << rtImuFileNum;
                break;
            }
            ++rtImuFileNum;
        }
        if(0 != pclose( fpin ) )
        {
            LOG(WARNING) << "Failed to close " << projectArr[i] + "/Rawdata/IMU/*_rt_track.txt";
            continue;
        }
        if(1 != rtImuFileNum)
        {
            LOG(ERROR) << projectArr[i] + "/Rawdata/IMU/ contains " << rtImuFileNum << " rt_track.txt, should be 1.";
            continue;
        }

        std::string lidarPath = projectArr[i] + "/Rawdata/Lidar/";
        std::string lidarCmd = "ls " + lidarPath + "| wc -l";
        std::string imagePath = projectArr[i] + "/Rawdata/Image/";
        std::string imageCmd = "ls " + imagePath + "| wc -l";
        FILE *fp;
        LOG(INFO) << "I am gonna: " << lidarCmd;
        if ( NULL == (fp = popen(lidarCmd.c_str(), "r") ) )
        {
            LOG(ERROR) << "Open " << lidarPath << "File Pipe Failed";
            continue;
        }

        if( NULL == fgets(result, maxLine, fp) )
        {
            LOG(ERROR) << "Read " << lidarPath << "LidarFile size Failed";
            continue;
        }
        LOG(INFO) << "Read " << lidarPath << "; lidar num: " << result;
        if(0 != pclose(fp))
        {
            LOG(ERROR) << "Close "<< lidarPath << "Lidar File Pipe Failed";
            continue;
        }

        numString = result;
        ss << numString;
        ss >> fileNum;
        this->totalFileNum = this->totalFileNum + fileNum;
        LOG(INFO) << "I am gonna: " << imageCmd;
        if ( NULL == (fp = popen(imageCmd.c_str(), "r") ) )
        {
            LOG(ERROR) << "Open " << imagePath << "Image File Pipe Failed";
            continue;
        }

        if( NULL == fgets(result, maxLine, fp) )
        {
            LOG(ERROR) << "Read " << imagePath << "ImageFile size Failed";
            continue;
        }
        LOG(INFO) << "Read " << imagePath << "; image num: " << result;
        if(0 != pclose(fp))
        {
            LOG(ERROR) << "Close " << imagePath << "Image File Pipe Failed.";
            continue;
        }

        numString = result;
        ss.clear();
        ss << numString;
        ss >> fileNum;
        this->totalFileNum = this->totalFileNum + fileNum;
        isProcess[i] = 1;
        LOG(INFO) << "counting Project: " << projectArr[i] << " fileSize is finished";
    }

    LOG(INFO) << "total file number is: " << this->totalFileNum;

    for(size_t i = 0; i < projectArr.size(); ++i)
    {
        imuData.clear();
        if(0 == isProcess[i] )
        {
            LOG(WARNING) << "Project :" << projectArr[i] << "has been processed or read lidar and image files number failed";
            continue;
        }
        LOG(INFO) << "start to fix Project: " << projectArr[i];
        this->markPointGeo2Gauss(projectArr[i]);
        returnValue = this->readOntimeTraceData(projectArr[i], imuData);
        if( 0 != returnValue )
        {
            LOG(ERROR) << "Load IMU ontime Data failed!";
            continue;
        }
        if(0 == imuData.size())
        {
            LOG(ERROR) << "Load IMU ontime Data failed!";
            continue;
        }
        returnValue = this->reNameImageAndMkTraceFile(projectArr[i], imuData);
        if(0 != returnValue)
            continue;
        this->mkLidarTraceFile(projectArr[i], imuData);

        LOG(INFO) << "fixing Project: " << projectArr[i] << " is finished";
    }
}

int dataFixed::mkImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,std::string &reNewPicture,imageTraceDataFormat &oneImageTraceData)
{
    unsigned long imuDataSubMark = 0;
    double hzTime = 0.01;
    std::string constDate = "20";
    constDate += projectName.substr(projectName.size()-6,6);
    if(imageTime < minGPSTime || imageTime > maxGPSTime)
    {
        LOG(WARNING) << imageTime << "can not find Image corresponding GPS Time";
        return -1;
    }
    if(minGPSTimeMark == 0 || imageTime > endGPSTime)
    {
        imuDataSubMark = (unsigned long)((imageTime-beganGPSTime)/hzTime);
    }
    else
    {
        imuDataSubMark = (unsigned long)((imageTime-beganGPSTime)/hzTime) + minGPSTimeMark;
    }
    int beltNumber = (imuData[imuDataSubMark].Longitude+1.5)/3;
    std::string beltString = std::to_string(beltNumber);
    reNewPicture += beltString;
    reNewPicture += projectName;
    int hour = imageTime/3600;
    double leftTime = imageTime-hour*3600;
    int minues = leftTime/60;
    double leftSeconds = leftTime-60*minues;
    int seconds = leftSeconds;
    double mSeconds = leftSeconds-seconds;
    std::string beijingTime;
    std::string dateString;
    hour = hour + 8;
    if(hour > 23)
    {
        hour = hour - 24;
    }
    std::string hourString=std::to_string(hour);
    if(hour < 10)
    {
        reNewPicture += '0';
        beijingTime += '0';
        dateString += '0';

    }


    reNewPicture += hourString;
    beijingTime += hourString;
    dateString += hourString;
    std::string minuesString = std::to_string(minues);
    if(minues < 10)
    {
        reNewPicture += '0';
        beijingTime += '0';
        dateString += '0';
    }


    reNewPicture += minuesString;
    beijingTime += minuesString;
    dateString += minuesString;
    std::string secondString = std::to_string(seconds);
    if(seconds < 10)
    {
        reNewPicture += '0';
        beijingTime += '0';
        dateString += '0';
    }
    reNewPicture += secondString;
    beijingTime += secondString;
    dateString += secondString;
    std::string mSecondString = std::to_string(mSeconds);
    int p = mSecondString.find_last_of(".");
    int r = mSecondString.size()-(p+1);
    if(r < 3)
    {
        reNewPicture += mSecondString.substr(p+1, r);
        beijingTime += mSecondString.substr(p, r);
        dateString += mSecondString.substr(p+1, r);
        for(int m=0; m<3-r; m++)
        {
            reNewPicture += '0';
            beijingTime += '0';
            dateString += '0';
        }
        beijingTime += '0';
    }
    else
    {
        reNewPicture += mSecondString.substr(p+1, 3);
        beijingTime += mSecondString.substr(p, 4);
        dateString += mSecondString.substr(p+1, 3);
    }
    std::string date = constDate;
    date += dateString;
    std::string saveImageName = reNewPicture;
    reNewPicture += ".jpg";
    double tmpLatitude = imuData[imuDataSubMark].Latitude;
    double tmpLongitude = imuData[imuDataSubMark].Longitude;
    double northCoordinate = 0;
    double eastCoordinate = 0;
    GeoToGauss(tmpLongitude, tmpLatitude, 3, &northCoordinate, &eastCoordinate);
    //make image effect ontime trace data
    oneImageTraceData.Pano_name = saveImageName;
    oneImageTraceData.Date = date;
    oneImageTraceData.GpsTime = imageTime;
    oneImageTraceData.BeijingTime = beijingTime;
    oneImageTraceData.Easting = eastCoordinate;
    oneImageTraceData.Northing = northCoordinate;
    oneImageTraceData.H_ell = imuData[imuDataSubMark].Height;
    oneImageTraceData.Latitude = tmpLatitude;
    oneImageTraceData.Longitude = tmpLongitude;
    oneImageTraceData.Roll = imuData[imuDataSubMark].Roll;
    oneImageTraceData.Pitch = imuData[imuDataSubMark].Pitch;
    oneImageTraceData.Heading = imuData[imuDataSubMark].Heading;
    return 0;
}

int dataFixed::mkLostImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,imageTraceDataFormat &oneLostImageTraceData)
{
    std::string constDate = "20";
    constDate += projectName.substr(projectName.size()-6,6);
    std::string saveImageName;
    unsigned long imuDataSubMark = 0;
    double hzTime = 0.01;
     if(imageTime < minGPSTime || imageTime > maxGPSTime)
    {
        LOG( WARNING ) << "can not find LostImage corresponding GPS Time";
        return -1;
    }
    if(minGPSTimeMark == 0 || imageTime > endGPSTime)
    {
        imuDataSubMark=(unsigned long)((imageTime-beganGPSTime) / hzTime);
    }
    else
    {
        imuDataSubMark=(unsigned long)((imageTime-beganGPSTime)/hzTime) + minGPSTimeMark;
    }
    int beltNumber = (imuData[imuDataSubMark].Longitude + 1.5)/3;
    std::string beltString = std::to_string(beltNumber);
    saveImageName += beltString;
    saveImageName += projectName;
    int hour = imageTime/3600;
    double leftTime = imageTime - hour*3600;
    int minues = leftTime / 60;
    double leftSeconds = leftTime - 60*minues;
    int seconds = leftSeconds;
    double mSeconds = leftSeconds - seconds;
    std::string beijingTime;
    std::string dateString;
    hour = hour + 8;
    if(hour > 23)
        hour = hour - 24;
    std::string hourString = std::to_string(hour);
    if(hour < 10)
    {
        saveImageName += '0';
        beijingTime += '0';
        dateString += '0';
    }
    saveImageName += hourString;
    beijingTime += hourString;
    dateString += hourString;
    std::string minuesString = std::to_string(minues);
    if(minues < 10)
    {
        saveImageName += '0';
        beijingTime += '0';
        dateString += '0';
    }
    saveImageName += minuesString;
    beijingTime += minuesString;
    dateString += minuesString;
    std::string secondString = std::to_string(seconds);
    if(seconds < 10)
    {
        saveImageName += '0';
        beijingTime += '0';
        dateString += '0';
    }
    saveImageName += secondString;
    beijingTime += secondString;
    dateString += secondString;
    std::string mSecondString = std::to_string(mSeconds);
    int p = mSecondString.find_last_of(".");
    int r = mSecondString.size() - (p+1);
    if(r<3)
    {
        beijingTime += mSecondString.substr(p,r);
        dateString += mSecondString.substr(p+1,r);
        saveImageName += mSecondString.substr(p+1,r);
        for(int m=0; m < 3-r; m++)
        {
            beijingTime += '0';
            dateString += '0';
            saveImageName += '0';
        }
        beijingTime += '0';
    }
    else
    {
        beijingTime += mSecondString.substr(p,4);
        dateString += mSecondString.substr(p+1,3);
        saveImageName += mSecondString.substr(p+1,3);
    }
    std::string date = constDate;
    date += dateString;
    double tmpLatitude = imuData[imuDataSubMark].Latitude;
    double tmpLongitude = imuData[imuDataSubMark].Longitude;
    double northCoordinate = 0;
    double eastCoordinate = 0;
    GeoToGauss(tmpLongitude, tmpLatitude, 3, &northCoordinate, &eastCoordinate);
    //record image effect ontime trace data
    oneLostImageTraceData.Pano_name = saveImageName;
    oneLostImageTraceData.Date = date;
    oneLostImageTraceData.GpsTime = imageTime;
    oneLostImageTraceData.BeijingTime = beijingTime;
    oneLostImageTraceData.Easting = eastCoordinate;
    oneLostImageTraceData.Northing = northCoordinate;
    oneLostImageTraceData.H_ell = imuData[imuDataSubMark].Height;
    oneLostImageTraceData.Latitude = tmpLatitude;
    oneLostImageTraceData.Longitude = tmpLongitude;
    oneLostImageTraceData.Roll = imuData[imuDataSubMark].Roll;
    oneLostImageTraceData.Pitch = imuData[imuDataSubMark].Pitch;
    oneLostImageTraceData.Heading = imuData[imuDataSubMark].Heading;
    return 0;
}

void dataFixed::saveImageTraceData(std::string &savePath,std::string &projectName,std::vector<imageTraceDataFormat> &imageTraceData,int saveMark){
    std::ofstream imageTraceDataFile;
    std::string imageEffectTracePartName="-RTimgpost.txt";
    int recordBeltNumber=0;
    int beltNumber=0;
    for(unsigned long i=0;i<imageTraceData.size();i++)
    {
        beltNumber=(imageTraceData[i].Longitude+1.5)/3;
        if(i == 0)
        {
            recordBeltNumber=beltNumber;
            std::string beltNumberString=std::to_string(beltNumber);
            std::string tmpImageTracePath=savePath;
            tmpImageTracePath+="/";
            tmpImageTracePath+=beltNumberString;
            tmpImageTracePath+=projectName;
            std::string imageTraceFilePartName;
            if(0 == saveMark)
                imageTraceFilePartName="-RTimgpost.txt";
            else
                imageTraceFilePartName="-RTimglost.txt";
            tmpImageTracePath+=imageTraceFilePartName;
            imageTraceDataFile.open(tmpImageTracePath);
        }
        else
        {
            if(recordBeltNumber != beltNumber)
            {
                imageTraceDataFile.close();
                std::string beltNumberString=std::to_string(beltNumber);
                std::string tmpImageTracePath=savePath;
                tmpImageTracePath+="/";
                tmpImageTracePath+=beltNumberString;
                tmpImageTracePath+=projectName;
                std::string imageTraceFilePartName;
                if(0 == saveMark)
                    imageTraceFilePartName="-RTimgpost.txt";
                else
                    imageTraceFilePartName="-RTimglost.txt";
                tmpImageTracePath+=imageTraceFilePartName;
                imageTraceDataFile.open(tmpImageTracePath);
                recordBeltNumber=beltNumber;
            }
        }
        std::string seqNumString=std::to_string(i);
        imageTraceDataFile << seqNumString << "\t" << imageTraceData[i].Pano_name <<
        "\t" << imageTraceData[i].Date <<"\t"<< std::fixed << std::setprecision(2)
        << imageTraceData[i].GpsTime << "\t" << imageTraceData[i].BeijingTime << "\t"
        <<std::fixed <<std::setprecision(4) << imageTraceData[i].Easting << "\t"
        << std::fixed << std::setprecision(4) << imageTraceData[i].Northing<< "\t"<<
        std::fixed << std::setprecision(3) << imageTraceData[i].H_ell << "\t"
        << std::fixed << std::setprecision(11) << imageTraceData[i].Latitude << "\t" <<
        std::fixed << std::setprecision(11) << imageTraceData[i].Longitude << "\t" <<
        std::fixed << std::setprecision(5) << imageTraceData[i].Roll << "\t"
        << std::fixed << std::setprecision(5) << imageTraceData[i].Pitch << "\t"
        << std::fixed << std::setprecision(5) << imageTraceData[i].Heading << std::endl;
        imageTraceDataFile.unsetf(std::ios::fixed);
    }
    imageTraceDataFile.close();
}

bool dataFixed::markPointGeo2Gauss(std::string &projectPath)
{
    std::vector<std::string> filesPath;
    std::string markFilepath = projectPath + "/Rawdata/Event";
    std::string cmd="ls " + markFilepath + "/event.txt";
    const size_t maxLine = 1000;
    char result[maxLine];
    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
    {
        LOG(ERROR) << markFilepath << ":Creat Markpoint Data Pipe Failed ";
        return false;
    }
    while(1)
    {
        if(NULL == fgets(result, maxLine, fpin) )
        {
            break;
        }
        std::string tmpPath = result;
        if(tmpPath[tmpPath.size()-1] == 10)
            filesPath.push_back(tmpPath.substr(0, tmpPath.size()-1 ) );
        else
            filesPath.push_back(tmpPath);
    }
    if(0 != pclose( fpin ) )
    {
        LOG(ERROR) << markFilepath <<":close Markpoint Pipe Failed";
        return false;
    }
    if(filesPath.size() != 1)
    {
        LOG(ERROR) << markFilepath <<":Load Number of IMU real time Data is not 1";
        return false;
    }

    markFilepath = filesPath[0];
    std::ifstream filePointer(markFilepath.c_str() );
    std::string outPutPath = markFilepath.substr(0, markFilepath.size() - 4) + "2Gauss.txt";
    std::ofstream outStream(outPutPath.c_str());
    if(!filePointer)
    {
        LOG(ERROR) << "Open the file" << markFilepath << "failed";
        return -1;
    }

    char buffer[256];
    std::string lineString;
    std::vector<std::string> lineVector;
    double latitude, longitude , northCoordinate, eastCoordinate;
    while(!filePointer.eof())
    {
        filePointer.getline(buffer,256);
        lineString = buffer;
        boost::split(lineVector, lineString, boost::is_any_of(","));
        if(6 != lineVector.size())
        {
            LOG(WARNING) << "each line of event file should have 6 elements";
            continue;
        }
        std::stringstream ss;
        ss << lineVector[0];
        ss >> longitude;
        ss.clear();
        ss << lineVector[1];
        ss >> latitude;
        ss.clear();
        GeoToGauss(longitude, latitude, 3, &northCoordinate,&eastCoordinate);
        outStream << lineVector[0] << "," << lineVector[1] << "," << lineVector[2] << "," <<std::fixed <<
        eastCoordinate << "," << northCoordinate << "," << lineVector[3] << "," << lineVector[4] << "," <<
        lineVector[5] << std::endl;

    }
    outStream.close();
    filePointer.close();
    return true;
}

int dataFixed::readOntimeTraceData(std::string &projectPath, std::vector<ontimeDataFormat> &traceFileData){
    std::vector<std::string> filesPath;
    std::string traceFilepath = projectPath + "/Rawdata/IMU";
    std::string cmd="ls " + traceFilepath + "/*_rt_track.txt";
    const size_t maxLine = 1000;
    char result[maxLine];
    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
    {
        LOG(ERROR) << traceFilepath << ":Creat IMU real Time Data Pipe Failed ";
        return -1;
    }
    while(1)
    {
        if(NULL == fgets(result, maxLine, fpin) )
        {
            break;
        }
        std::string tmpPath = result;
        if(tmpPath[tmpPath.size()-1] == 10)
            filesPath.push_back(tmpPath.substr(0, tmpPath.size()-1 ) );
        else
            filesPath.push_back(tmpPath);
    }
    if(0 != pclose( fpin ) )
    {
        LOG(ERROR) << traceFilepath <<":close IMU Pipe Failed";
        return -1;
    }
    if(filesPath.size() != 1)
    {
        LOG(ERROR) << traceFilepath <<":Load Number of IMU real time Data is not 1";
        return -1;
    }
    traceFilepath = filesPath[0];
    std::ifstream filePointer(traceFilepath.c_str() );
    if(!filePointer)
    {
        LOG(ERROR) << "Open the file" << traceFilepath << "failed";
        return -1;
    }
    char buffer[256];
    char header[20],status[20];
    int totalDaySecondTime = 24*3600;
    double timeMark=500;
    ontimeDataFormat imuonTimeData;
    unsigned long postNum=0;
    while(!filePointer.eof())
    {
        filePointer.getline(buffer, 256);
        std::string line(buffer);
        std::vector<std::string> parsedLine;
        boost::split(parsedLine, line, boost::is_any_of(",*") );
        if(17 == parsedLine.size())
        {
            std::stringstream ss;
            ss << parsedLine[2];
            double weekSecondTime;
            ss >> weekSecondTime;
            if(weekSecondTime < timeMark)
            {
                continue;
            }
            int modNumber=int(weekSecondTime)/totalDaySecondTime;
            imuonTimeData.GPSWeekTime=weekSecondTime-modNumber*totalDaySecondTime;
            ss.clear();
            ss << parsedLine[3];
            ss >> imuonTimeData.Heading;
            ss.clear();
            ss << parsedLine[4];
            ss >> imuonTimeData.Pitch;
            ss.clear();
            ss << parsedLine[5];
            ss >> imuonTimeData.Roll;
            ss.clear();
            ss << parsedLine[6];
            ss >> imuonTimeData.Latitude;
            ss.clear();
            ss << parsedLine[7];
            ss >> imuonTimeData.Longitude;
            ss.clear();
            ss << parsedLine[8];
            ss >> imuonTimeData.Height;
            ss.clear();
            ss << parsedLine[9];
            ss >> imuonTimeData.Ve;
            ss.clear();
            ss << parsedLine[10];
            ss >> imuonTimeData.Vn;
            ss.clear();
            ss << parsedLine[11];
            ss >> imuonTimeData.Vu;
            ss.clear();
            ss << parsedLine[12];
            ss >> imuonTimeData.Baseline;
            ss.clear();
            ss << parsedLine[13];
            ss >> imuonTimeData.NSV1;
            ss.clear();
            ss << parsedLine[14];
            ss >> imuonTimeData.NSV2;
            if(imuonTimeData.GPSWeekTime<minGPSTime)
            {
                minGPSTime=imuonTimeData.GPSWeekTime;
                minGPSTimeMark=postNum;
            }
            if(imuonTimeData.GPSWeekTime > maxGPSTime)
                maxGPSTime=imuonTimeData.GPSWeekTime;
            traceFileData.push_back(imuonTimeData);
            postNum++;
        }
    }
    filePointer.close();
    if(traceFileData.size()!=0)
    {
        beganGPSTime=traceFileData[0].GPSWeekTime;
        endGPSTime=traceFileData[traceFileData.size()-1].GPSWeekTime;
    }
    return 0;
}


void dataFixed::GeoToGauss(double longitude, double latitude, short beltWidth, double *y, double *x) {
    double t;     //  t=tgB
    double L;     //  中央经线的经度
    double l0;    //  经差
    double longitude2Rad, latitude2Rad;  //  将jd、wd转换成以弧度为单位
    double et2;    //  et2 = (e' ** 2) * (cosB ** 2)
    double N;     //  N = C / sqrt(1 + et2)
    double X;     //  克拉索夫斯基椭球中子午弧长
    double m;     //  m = cosB * PI/180 * l0
    double tsin, tcos, et3;   //  sinB,cosB
    int beltNumber;
    double PI = 3.14159265358979;
    double b_e2 = 0.00669437999013;
    double b_c = 6378137;
    longitude2Rad = longitude * PI / 180.0;
    latitude2Rad = latitude * PI / 180.0;

    if(3==beltWidth)
    {
        beltNumber=(longitude+1.5)/beltWidth;
        L=beltNumber*beltWidth;
    }
    if(6==beltWidth)
    {
        beltNumber=(longitude+6)/beltWidth;
        L=beltNumber*beltWidth-3;
    }

    l0 = longitude - L;       // 计算经差
    tsin = sin(latitude2Rad);        // 计算sinB
    tcos = cos(latitude2Rad);        // 计算cosB
    // 计算克拉索夫斯基椭球中子午弧长
    X = 111132.9558 *latitude - 16038.6496*sin(2 * latitude2Rad) + 16.8607*sin(4 * latitude2Rad) - 0.0220*sin(6 * latitude2Rad);
    et2 = b_e2 * pow(tcos, 2); //  et2 = (e' ** 2) * (cosB ** 2)
    et3 = b_e2 * pow(tsin, 2);
    N = b_c / sqrt(1 - et3);     //  N = C / sqrt(1 + et2)
    t = tan(latitude2Rad);         //  t=tgB
    m = PI / 180 * l0 * tcos;       //  m = cosB * PI/180 * l0
    *x = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);

    *y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
}

int dataFixed::reNameImageAndMkTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData){
    const size_t maxLine = 1000;
    char result[maxLine];
    FILE *fpin;
    std::vector <std::string> pictureName;
    std::string projectName = "-";
    std::string tmpProjectName;
    std::vector<std::string> filterString;
    std::string processPath = projectPath + "/Process";
    mkdir(processPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::string imagePath=projectPath+"/Rawdata/Image/";
    int existImageMark = access(imagePath.c_str(), F_OK);
    if(-1 == existImageMark)
    {
        LOG(ERROR) << "There is no Image File in the " << imagePath;
        return -1;;
    }
    boost::split(filterString , projectPath , boost::is_any_of("/"));
    if(filterString.size() == 0)
    {
        LOG(ERROR) << "ProjectPath:" << projectPath << "is invalid!";
        return -1;
    }
    for(int k = filterString.size()-1 ;k >= 0 ; k--)
    {
        if(filterString[k] != "\0")
        {
            tmpProjectName=filterString[k];
            break;
        }
    }
    filterString.clear();
    boost::split(filterString,tmpProjectName,boost::is_any_of("-"));
    if(filterString.size() == 0 || filterString.size() != 4)
    {
        LOG(ERROR) << "ProjectName:" << tmpProjectName << "is invalid!";
        return -1;
    }
    for(int k = 0; k < filterString.size(); k++)
    {
        projectName += filterString[k];
        belongtoLidarProjectName += filterString[k];
    }
    if( projectName.size() < 6 )
    {
        LOG(ERROR) << "ProjectName: " << tmpProjectName << "is invalid!";
        return -1;
    }
    const std::string cmd="cd " + imagePath + " && ls ";
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
    {
        LOG(ERROR) << projectPath <<":Creat the Image PipeLine Failed";
        return -1;
    }

     while(1)
    {
        if(NULL == fgets(result, maxLine, fpin)) {
            LOG(INFO) << "cmd: " << cmd << " return null.";
            break;
        }
        std::string onePictureName = result;
        DLOG(INFO) << "onePictureName: " << onePictureName;
        if(onePictureName[onePictureName.size()-1] == 10)
            pictureName.push_back(onePictureName.substr(0,onePictureName.size()-1 ) );
        else
            pictureName.push_back(onePictureName);
    }
    if(0 != pclose(fpin) ) {
        LOG(ERROR) << projectPath << "Close ImagePipeLine Failed";
        return -1;
    }
    if(0 == pictureName.size() ) {
        LOG(ERROR) << projectPath <<"Load Picture List Failed";
        return -1;
    }
    std::vector<int> editePictureNameMark(pictureName.size(),0);
    std::string uselessImageMark = "sys";
    std::vector<std::string> newPictureName(pictureName);
    double imageTime;
    //unParsed

    double imageTimeInterval = 1.0/imageCollectionHz*2;
    double setImageTimeInterval = 2.0;
    double imageTimeRecord = 0;
    unsigned long imageMark = 0;
    imageTraceDataFormat oneImageTraceData;
    imageTraceDataFormat oneLostImageTraceData;
    std::vector<imageTraceDataFormat> allImageTraceData;
    std::vector<imageTraceDataFormat> allLostImageTraceData;

    for(unsigned long i=0; i < pictureName.size(); i++)
    {
        std::string readPartImageName = pictureName[i].substr(0,3);
        if(readPartImageName == uselessImageMark)
        {
            editePictureNameMark[i] = 2;
            continue;
        }
        std::vector<std::string> imageNameSplite;
        boost::split(imageNameSplite, pictureName[i], boost::is_any_of(".") );
        if(3 != imageNameSplite.size())
        {
            continue;
        }
        std::stringstream ss;
        int q = pictureName[i].find_last_of('.');
        ss << pictureName[i].substr(0,q-1);
        ss >> imageTime;
        std::string reNewPicture;
        imageMark++;
        mkImageTraceData(imageTime, projectName, imuData, reNewPicture, oneImageTraceData);
        if(reNewPicture.size()==0)
        {
            imageMark--;
            continue;
        }
        newPictureName[i] = reNewPicture;
        allImageTraceData.push_back(oneImageTraceData);
        editePictureNameMark[i] = 1;
        if(imageMark == 1)
            imageTimeRecord = imageTime;
        else
        {
            double imageTimediffer = fabs(imageTime-imageTimeRecord);
            if(imageTimediffer > imageTimeInterval && imageTimediffer < setImageTimeInterval)
            {
                double imageTimeCopy = imageTimeRecord;
                while(imageTimeCopy < imageTime)
                {
                    imageTimeCopy += imageTimeInterval;
                    mkLostImageTraceData(imageTimeCopy, projectName, imuData, oneLostImageTraceData);
                    allLostImageTraceData.push_back(oneLostImageTraceData);
                }
            }
            imageTimeRecord = imageTime;
        }
    }
    LOG(INFO) << "Generating Image ontime Trace Data";
    saveImageTraceData(processPath,projectName,allImageTraceData,0);
    if(0 != allLostImageTraceData.size())
    {
        LOG(INFO) << "Generating Lost Image ontime Trace Data";
        saveImageTraceData(processPath, projectName, allLostImageTraceData, 1);
    }

    //Rename Picture
    LOG(INFO) << "renaming Image";
    LOG(INFO) << "newPictureName.size(): " << newPictureName.size() <<"; pictureName.size(): " << pictureName.size();
    for(unsigned long i = 0; i < editePictureNameMark.size(); i++)
    {
        processNum++;
        if(editePictureNameMark[i] == 1)
        {
            FILE * renameImageFpin;
            const std::string renameImagecmd = "cd "+ imagePath +" && mv " + pictureName[i] + " " + newPictureName[i];
            if(NULL == ( renameImageFpin = popen(renameImagecmd.c_str(),"w" ) ) )
                LOG(ERROR) << projectPath <<":Rename Picture falied";
            pclose(renameImageFpin);
        }
        else
        {
            if(editePictureNameMark[i] == 2)
            {
                FILE * deleImageFpin;
                const std::string deleImagecmd="cd "+ imagePath+" && rm " + pictureName[i];
                if(NULL == ( deleImageFpin = popen(deleImagecmd.c_str(),"w" ) ) )
                    LOG(ERROR) << projectPath << ":delete Picture falied";
                pclose(deleImageFpin);
            }
        }
    }
    return 0;
}

int dataFixed::mkLidarTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData){
    //make lidar effect ontime trace file
    std::string lidarFileTotalpath=projectPath + "/Rawdata/Lidar/";
    int existLidarFileMark=access(lidarFileTotalpath.c_str(),F_OK);
    if(existLidarFileMark == -1)
    {
        LOG(ERROR) << "There is no 'Lidar' directory in the" << lidarFileTotalpath;
        return -1;
    }
    FILE *readLidarFile;
    const size_t maxLine=1000;
    char lidarFilePathName[maxLine];
    std::vector<std::string> totalLidarFilePath;
    totalLidarFilePath.clear();

    const std::string findLidarCmd="ls " + lidarFileTotalpath + "*.dat";
    if(NULL == ( readLidarFile=popen(findLidarCmd.c_str(),"r" ) ) )
    {
        LOG(ERROR) << lidarFileTotalpath << ":Creat the Lidar PipeLine Failed";
        return -1;
    }
    while(1)
    {
        if(NULL == fgets ( lidarFilePathName, maxLine, readLidarFile ) ) {
            LOG(INFO) << "findLidarCmd: " << findLidarCmd << " return null.";
            break;
        }
        std::string oneLidarFilePath=lidarFilePathName;
        LOG(INFO) << "oneLidarFilePath: " << oneLidarFilePath;
        if(oneLidarFilePath[oneLidarFilePath.size()-1] == 10)
            totalLidarFilePath.push_back(oneLidarFilePath.substr(0,oneLidarFilePath.size()-1));
        else
            totalLidarFilePath.push_back(oneLidarFilePath);
    }
    if( totalLidarFilePath.empty() )
    {
        LOG(ERROR) << "Load Lidar file falied or there is no lidar file in the project" << lidarFileTotalpath;
        return -1;
    }
    pktDataFormat pktData;
    std::string projectName=belongtoLidarProjectName;
    std::string saveLidarTracePath=projectPath + "/Process/" +projectName + "-RTlaser.txt";
    std::ofstream lidarTraceFile(saveLidarTracePath);
    std::vector <int> lidarPointGPSTimeMark;
    int firstMark=0;
    unsigned long lidarPointSeq=0;
    double tmpLongitude,tmpLatitude,eastCoordinate,northCoordinate;
    double hzSpeed;
    const size_t pktSize=sizeof(double)+BIT_IN_PACKET_DATA;
    double compareTime=100000;
    double hzTime=0.01;
    int findMark=0;
    unsigned long tmpsubMark=0;
    //define shp file Pointer
    const char *pszDriverName="ESRI Shapefile";
    std::string shapFilePath=projectPath + "/Process/" +projectName;
    const char *pShpName = shapFilePath.c_str();
    GDALDriver *poDriver;
    GDALAllRegister();
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    if( poDriver == NULL )
    {
        LOG(ERROR) << projectPath << ":" << pszDriverName << "driver not available";
        return -1;
    }
    GDALDataset *poDS;
    char cShpName[256];
    sprintf(cShpName, "%s.shp", pShpName);
    poDS = poDriver->Create( cShpName, 0, 0, 0, GDT_Unknown, NULL );
    if( poDS == NULL )
    {
        LOG(ERROR) << projectPath <<":Creation of output shapFile failed";
        return -1;
    }

    OGRLayer *poLayer;
    poLayer = poDS->CreateLayer( pShpName, NULL, wkbMultiLineString, NULL );
    if( poLayer == NULL )
    {
        LOG(ERROR) << projectPath <<":shapLayer creation failed.";
        return -1;
    }
    char szName[33] = "IntelliSense";
    OGRFeature *poFeatureG;
    poFeatureG = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
    poFeatureG->SetField("Name", szName);
    OGRLineString *pLineString = 0;

    double timeRecord=0;
    double timeOfStop=1.0;
    for(int i = 0; i < totalLidarFilePath.size();i++)
    {
        FILE *lidarFilePointer=fopen(totalLidarFilePath[i].c_str(),"rb");
        if(lidarFilePointer == NULL)
        {
            LOG(WARNING) << "Open lidar File" << totalLidarFilePath[i] << "Failed";
            processNum++;
            continue;
        }
        size_t readLength=0;
        fseek(lidarFilePointer,0,SEEK_END);
        size_t oneLidarSize=ftell(lidarFilePointer);
        fseek(lidarFilePointer,0,SEEK_SET);
        unsigned int pktNum=oneLidarSize/pktSize;
        pktDataFormat *P=new pktDataFormat[pktNum];
        readLength=fread(P,pktSize,pktNum,lidarFilePointer);
        if(readLength != pktNum)
        {
            LOG(WARNING) << "read the Lidar :" << totalLidarFilePath[i] << "Failed";
            processNum++;
            continue;
        }
        unsigned int tmpNum=0;
        tmpNum++;
        unsigned long subMarkRecord=0;
        while(tmpNum <= pktNum)
        {
            DLOG(INFO) << "P+tmpNum-1: " << P+tmpNum-1;
            pktData=*(P+tmpNum-1);
            tmpNum++;
            double oneLidarPointTime=pktData.timeStamp;

            if( oneLidarPointTime < minGPSTime||oneLidarPointTime >maxGPSTime)
            {
                LOG(WARNING) << totalLidarFilePath[i] <<"LidarPoint time can not find corresponding GPS Time";
                continue;
            }
            DLOG(INFO) << "the time of lidarPoint is :" << oneLidarPointTime;
            if(minGPSTimeMark==0 || oneLidarPointTime > endGPSTime )
                subMarkRecord=(unsigned long)((oneLidarPointTime-beganGPSTime)/hzTime);
            else
            {
                subMarkRecord=(unsigned long)((oneLidarPointTime-beganGPSTime)/hzTime)+minGPSTimeMark;
            }
            if(lidarPointSeq == 0)
                tmpsubMark=subMarkRecord;
            else
            {
                if( tmpsubMark ==subMarkRecord )
                    continue;
                else
                    tmpsubMark=subMarkRecord;
            }
            if(lidarPointSeq == 0)
            {
                pLineString=new OGRLineString();
                LOG(INFO) << "imuData.size(): " << imuData.size() << "; subMarkRecord: " << subMarkRecord;
                timeRecord = imuData[subMarkRecord].GPSWeekTime;
                pLineString->addPoint(imuData[subMarkRecord].Longitude, imuData[subMarkRecord].Latitude);
            }
            else
            {
                if( fabs(timeRecord-imuData[subMarkRecord].GPSWeekTime) > timeOfStop )
                {
                    poFeatureG->SetGeometry(pLineString);
                    if( poLayer->CreateFeature( poFeatureG ) != OGRERR_NONE )
                    {
                       LOG(ERROR) << totalLidarFilePath[i] <<":Falied to create feature in shapefile";
                       return -1;
                    }
                    pLineString=new OGRLineString();
                }
                pLineString->addPoint(imuData[subMarkRecord].Longitude, imuData[subMarkRecord].Latitude);
                timeRecord=imuData[subMarkRecord].GPSWeekTime;
            }

            //make lidar realTime data effective trace
            lidarPointSeq++;
            tmpLongitude=imuData[subMarkRecord].Longitude;
            tmpLatitude=imuData[subMarkRecord].Latitude;
            GeoToGauss(tmpLongitude,tmpLatitude,3,&northCoordinate,&eastCoordinate);
            hzSpeed=sqrt(pow(imuData[subMarkRecord].Ve,2) + pow(imuData[subMarkRecord].Vn,2));
            std::string pointSeqString=std::to_string(lidarPointSeq);
            lidarTraceFile << pointSeqString<<"\t" << std::fixed << std::setprecision(2) <<
            imuData[subMarkRecord].GPSWeekTime << "\t" <<std::fixed << std::setprecision(4) <<
            northCoordinate << "\t" << std::fixed << std::setprecision(4) << eastCoordinate <<
            "\t" << std::fixed << std::setprecision(3) << imuData[subMarkRecord].Height <<"\t"
            << std::fixed << std::setprecision(11) << tmpLatitude <<"\t" << std::fixed << std::setprecision(11)
            << tmpLongitude <<"\t" << std::fixed << std::setprecision(4) << hzSpeed << "\t" <<
            std::fixed << std::setprecision(5) << imuData[subMarkRecord].Roll <<"\t" << std::fixed << std::setprecision(5)
            << imuData[subMarkRecord].Pitch <<"\t" << std::fixed <<std::setprecision(5) << imuData[subMarkRecord].Heading
            << "\t" << projectName <<"\t" << imuData[subMarkRecord].NSV2 << std::endl;

        }
        fclose(lidarFilePointer);
        processNum++;
    }
    lidarTraceFile.close();

    if(pLineString != 0)
    {
        poFeatureG->SetGeometry(pLineString);
    }

    if( poLayer->CreateFeature( poFeatureG ) != OGRERR_NONE )
    {
        LOG(ERROR) << projectPath << ":Falied to create feature in shapefile";
        return -1;
    }
    OGRFeature::DestroyFeature( poFeatureG );
    GDALClose( poDS );

    LOG(INFO) << "processNum: " << processNum << "; totalFileNum: " << totalFileNum;
}

