#include "dataFixed.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

dataFixed::dataFixed(ros::NodeHandle nh, ros::NodeHandle private_nh, int cinValue)
{
    hzTime_ = 0.1;
    pubProgress_ = nh.advertise<sc_msgs::DataFixerProgress>("sc_data_fixer_progress", 10);
    LOG(INFO) << "Sleep 1 s in case pubProgress_ construct incomplete.";
    sleep(1);
    imageCollectionHz = cinValue;
    totalFileNum = processNum = 0;
}

void dataFixed::initMemberVar() {
    LOG(INFO) << __FUNCTION__ << " start.";

    beganGPSTime = 0;
    endGPSTime = 0;
    minGPSTime = 100000;
    maxGPSTime = 0;
    minGPSTimeMark = 0;
    belongtoLidarProjectName.clear();
}

void dataFixed::removeFile(const std::string &file) {
    LOG(INFO) << __FUNCTION__ << " start, param: " << file;

    if(remove(file.c_str()) != 0) {
        LOG(INFO) << file << " might not exist, but I don't give a shit.";
    }
    return;
}

void dataFixed::touchFile(const std::string &file) {
    LOG(INFO) << __FUNCTION__ << " start, param: " << file;

    std::ofstream out(file.c_str() );
    if(!out) {
        LOG(WARNING) << "Failed to create " << file;
    }

    return;
}




void dataFixed::fixProjectsData(const std::string &_projects)
{
    LOG(INFO) << __FUNCTION__ << " start, param: " << _projects;

    const std::string allFixedIndicator("/tmp/data_fixer_progress_100%");
    (void)removeFile(allFixedIndicator);

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
    unsigned long processImgNum = 0;
    unsigned int remaindr = 0;
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
        std::string panoramasPath = projectArr[i] + "/Rawdata/Image/panoramas/";
        std::string panoramasCmd = "ls " + panoramasPath + "|grep .jpg$" + "| wc -l";
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
        remaindr = fileNum % img2LidarProsVeloRatio;
        if(0 == remaindr)
        {
            processImgNum = fileNum / img2LidarProsVeloRatio;
        }
        else
        {
            processImgNum = fileNum / img2LidarProsVeloRatio + 1;
        }
        this->totalFileNum = this->totalFileNum + processImgNum;

        int ispanoramasPathExist = access(panoramasPath.c_str(), F_OK);
        if(0 != ispanoramasPathExist) {
            LOG(WARNING) << "Failed to access " << panoramasPath;
        }
        else {
            LOG(INFO) << "I am gonna: " << panoramasCmd;
            if ( NULL == (fp = popen(panoramasCmd.c_str(), "r") ) )
            {
                LOG(ERROR) << "Open " << panoramasPath << "Panoramas Image File Pipe Failed";
                continue;
            }
            if( NULL == fgets(result, maxLine, fp) )
            {
                LOG(ERROR) << "Read " << panoramasPath << "Panoramas ImageFile size Failed";
                continue;
            }
            LOG(INFO) << "Read " << panoramasPath << "; Panoramas image num: " << result;
            if(0 != pclose(fp))
            {
                LOG(ERROR) << "Close " << panoramasPath << "Panoramas Image File Pipe Failed.";
                continue;
            }
            numString = result;
            ss.clear();
            ss << numString;
            ss >> fileNum;
            remaindr = fileNum % img2LidarProsVeloRatio;
            if(0 == remaindr)
            {
                processImgNum = fileNum / img2LidarProsVeloRatio;
            }
            else
            {
                processImgNum = fileNum / img2LidarProsVeloRatio + 1;
            }
            this->totalFileNum = this->totalFileNum + processImgNum;
        }
        isProcess[i] = 1;
        LOG(INFO) << "counting Project: " << projectArr[i] << " fileSize is finished";
    }

    LOG(INFO) << "total file number is: " << this->totalFileNum;
    (void)pubProgress(processNum, totalFileNum);

    for(size_t i = 0; i < projectArr.size(); ++i)
    {
        imuData.clear();
        if(0 == isProcess[i] )
        {
            LOG(WARNING) << "Project :" << projectArr[i] << "has been processed or read lidar and image files number failed";
            continue;
        }
        LOG(INFO) << "start to fix Project: " << projectArr[i];
        this->initMemberVar();
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
        LOG(INFO) << "going to invoke mkLidarTraceFile";
        mkLidarTraceFile(projectArr[i], imuData);

        const std::string imageGroupCmd("/opt/smartc/devel/lib/sc_images_group/sc_images_group_node " + projectArr[i]);
        (void)public_tools::PublicTools::PopenWithoutReturn(imageGroupCmd);

        LOG(INFO) << "fixing Project: " << projectArr[i] << " is finished";
    }

    (void)touchFile(allFixedIndicator);
}

int dataFixed::mkImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,std::string &reNewPicture,imageTraceDataFormat &oneImageTraceData)
{
    unsigned long imuDataSubMark = 0;
    std::string constDate = "20";
    constDate += projectName.substr(projectName.size()-6,6);
    if(imageTime < minGPSTime || imageTime > maxGPSTime)
    {
        LOG(WARNING) << imageTime << " can not find Image corresponding GPS Time";
        return -1;
    }
    if(minGPSTimeMark == 0 || imageTime > endGPSTime)
    {
        imuDataSubMark = (unsigned long)((imageTime-beganGPSTime)/hzTime_);
    }
    else
    {
        imuDataSubMark = (unsigned long)((imageTime-minGPSTime)/hzTime_) + minGPSTimeMark;
    }
    if(imuDataSubMark >= imuData.size())
    {
        LOG(ERROR) << "IMU Ontime trace data(232 Serial) is not continuous";
        return -1;
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
    //hour = hour + 8;
    hour = hour + 9;
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
     if(imageTime < minGPSTime || imageTime > maxGPSTime)
    {
        LOG( WARNING ) << "can not find LostImage corresponding GPS Time";
        return -1;
    }
    if(minGPSTimeMark == 0 || imageTime > endGPSTime)
    {
        imuDataSubMark=(unsigned long)((imageTime-beganGPSTime) / hzTime_);
    }
    else
    {
        imuDataSubMark=(unsigned long)((imageTime-minGPSTime)/hzTime_) + minGPSTimeMark;
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
    //hour = hour + 8;
    hour = hour + 9;
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
    LOG(INFO) << __FUNCTION__ << " start.";
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
        imageTraceDataFile << seqNumString << "\t" << imageTraceData[i].Pano_name <<"\t" << imageTraceData[i].addPicName <<
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
    LOG(INFO) << __FUNCTION__ << "start.";
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
        LOG(INFO) << "min time is : " << minGPSTime;
        LOG(INFO) << "max time is : " << maxGPSTime;

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

bool getFileList(std::string &cmd, std::vector<std::string> &fileList)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    FILE *fpin = popen(cmd.c_str(), "r");
    if(NULL == fpin)
    {
        LOG(WARNING) << "creat: " << cmd << " pipe failed";
        return false;
    }
    const size_t maxLine = 1000;
    char result[maxLine];
    while(1)
    {
        if( NULL == fgets(result, maxLine, fpin))
            break;
        std::string tmpString = result;
        if(tmpString[tmpString.size() - 1] == 10)
        {
            fileList.push_back(tmpString.substr(0, tmpString.size() - 1));
        }
        else
            fileList.push_back(tmpString);
    }
    if(0 != pclose(fpin))
    {
        LOG(WARNING) << "close: " << cmd <<" pipe failed";
        return false;
    }
    return true;

}


bool readFPAGPSTime(std::string &projectPath, std::vector<double> &stdGpsTime)
{
    LOG(INFO) << __FUNCTION__ << " start.";
    stdGpsTime.clear();
    std::string recordFile = projectPath + "/Rawdata/IMU/*_timestamp.txt";
    std::string cmd = "ls " + recordFile;
    std::vector<std::string> recordFileList;
    if(!getFileList(cmd, recordFileList))
    {
        LOG(ERROR) << "Got FPGA GPS Record Time File failed.";
        return false;
    }
    if(1 != recordFileList.size())
    {
        LOG(ERROR) << "Got " << recordFileList.size() << " FPGA GPS Record Time File.should be 1.";
        return false;
    }
    std::ifstream fileIn(recordFileList[0].c_str());
    if(!fileIn)
    {
        LOG(ERROR) << "open file:" << recordFileList[0] << " failed.";
        return false;
    }
    char buffer[256];
    std::string lineString;
    std::vector<std::string> lineVector;
    double tmpTime;
    double lastTimeRecord = -1.0;
    std::vector<double> oneSetTime;
    std::vector<std::vector<double>> allSetTime;
    double oneDaySec = -1.0;
    while(!fileIn.eof())
    {
        fileIn.getline(buffer, 256);
        lineString = buffer;
        boost::split(lineVector, lineString, boost::is_any_of(","));
        if(2 > lineVector.size())
        {
            LOG(WARNING) << "the size of file line: " << lineString << " is " << lineVector.size() << ". should be 2.";
            continue;
        }
        std::stringstream ss;
        ss << lineVector[1];
        ss >> tmpTime;
        ss.clear();

        tmpTime = tmpTime + 18.0 ;
        if(tmpTime > 86400.0)
        {
            int tmpQuotient = tmpTime / 86400.0;
            tmpTime = tmpTime - tmpQuotient * 86400.0;
        }

        if(tmpTime < lastTimeRecord && fabs(tmpTime - lastTimeRecord) > 100.0)
        {
            if(10 < oneSetTime.size())
            {
                allSetTime.push_back(oneSetTime);
            }
            oneSetTime.clear();
        }

        lastTimeRecord = tmpTime;
        oneSetTime.push_back(tmpTime);
    }
    fileIn.close();
    allSetTime.push_back(oneSetTime);
    lastTimeRecord = -1.0;

    for(int i = 0; i < allSetTime.size(); i++)
    {
        if(lastTimeRecord > allSetTime[i][0])
        {
            LOG(WARNING) << "across data is found in the FPGA time List.";
            oneDaySec = 86400.0;
        }
        else
        {
            oneDaySec = 0;
        }
        for(int k = 0; k < allSetTime[i].size(); k++)
        {
            stdGpsTime.push_back(allSetTime[i][k] + oneDaySec);
        }
        lastTimeRecord = allSetTime[i][0];
    }
    if(0 == stdGpsTime.size())
    {
        LOG(INFO) << "load FPGA GPS Record Time File failed.";
        return false;
    }
    return true;
}

bool imageTimeMatched(std::vector<double> &FPGAGPSTime, int stdIdx, int tmpIdx, std::vector<double> &sortPicTime, std::vector<double> &sortCameraTime, double &matchedImgTime)
{
    double ImageGPSTime = sortPicTime[stdIdx] + sortCameraTime[tmpIdx] - sortCameraTime[stdIdx];
    //LOG(INFO) << "before matched to FPGA Time, the time is " << ImageGPSTime;
    if(ImageGPSTime < FPGAGPSTime[0] | ImageGPSTime > FPGAGPSTime[FPGAGPSTime.size() - 1])
    {
        LOG(ERROR) << "the pic time :" << ImageGPSTime << " is out of FPGA time range.";
        return false;
    }
    int sMark = 0;
    int eMark = FPGAGPSTime.size() - 1;
    int midMark = -1;
    bool isFound = false;
    while(true)
    {
        midMark = (sMark + eMark)/2;
        if(FPGAGPSTime[midMark] > ImageGPSTime)
        {
            eMark = midMark;
        }
        else
        {
            sMark = midMark;
        }
        int tmpMark = eMark - sMark;
        if(1 == tmpMark)
        {
            //LOG(INFO) << "eMark = " << eMark;
            //LOG(INFO) << "sMark = " << sMark;
            isFound = true;
            ImageGPSTime = FPGAGPSTime[sMark];
            break;
        }
    }
    if(!isFound)
    {
        LOG(ERROR) << "can not find responding FPGAGPSTime in the pic :" << sortPicTime[tmpIdx];
        return false;
    }
    if(ImageGPSTime > 86400.0)
    {
        int tmpQuotient = ImageGPSTime / 86400.0;
        ImageGPSTime = ImageGPSTime - tmpQuotient * 86400.0;
    }
    matchedImgTime = ImageGPSTime;
    //LOG(INFO) << "after matched, the final time is:" << ImageGPSTime;
    return true;
}


int dataFixed::reNameImageAndMkTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData){
    LOG(INFO) << __FUNCTION__ << " start.";
    const size_t maxLine = 1000;
    char result[maxLine];
    FILE *fpin;
    std::vector <std::string> pictureName;
    std::vector<std::string> sortPicName;
    std::string projectName = "-";
    std::string tmpProjectName;
    std::vector<std::string> filterString;
    std::string processPath = projectPath + "/Process";
    mkdir(processPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::string imagePath=projectPath+"/Rawdata/Image/";
    int existImageMark = access(imagePath.c_str(), F_OK);
    std::vector<std::vector<double>> imagTimeSet;
    std::vector<std::vector<double>> cameraTimeSet;
    std::vector<std::vector<int>> imageSetIdx;
    std::vector<double> oneSetImageTime;
    std::vector<double> oneSetCameraTime;
    std::vector<int> oneSetImageIdx;
    std::stringstream ss;
    std::vector<std::string> splitVector;
    double oneImageTime = -1.0;
    double tmpCameraT= -1.0;
    std::vector<double> sortPicTime;
    std::vector<double> sortCameraTime;
    int recordI = -1;
    bool isFoundStdTime = false;
    if(-1 == existImageMark)
    {
        LOG(ERROR) << "There is no Image File in the " << imagePath;
        return -1;
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
    belongtoLidarProjectName.clear();
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

    //get Image File List
    LOG(INFO) << "get Image File List.";
    const std::string cmd="cd " + imagePath + " && ls";
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
        if(onePictureName[onePictureName.size()-1] == 10)
        {
            onePictureName = onePictureName.substr(0,onePictureName.size()-1 );
        }
        boost::split(splitVector, onePictureName, boost::is_any_of("_"));
        if(4 > splitVector.size())
        {
            LOG(WARNING) <<  "picture name: " << onePictureName << " is invalid.";
            continue;
        }
        pictureName.push_back(onePictureName);
        ss << splitVector[0];
        ss >> oneImageTime;
        ss.clear();
        ss << splitVector[2];
        ss >> tmpCameraT;
        ss.clear();
        if(0 != oneSetImageTime.size() && fabs(oneImageTime - oneSetImageTime[oneSetImageTime.size() -1]) > 72000.0)
        {
            LOG(WARNING) << "across day data is Found in the pic List";
            imagTimeSet.push_back(oneSetImageTime);
            oneSetImageTime.clear();
            cameraTimeSet.push_back(oneSetCameraTime);
            oneSetCameraTime.clear();
            imageSetIdx.push_back(oneSetImageIdx);
            oneSetImageIdx.clear();
        }
        oneSetImageTime.push_back(oneImageTime);
        oneSetCameraTime.push_back(tmpCameraT);
        oneSetImageIdx.push_back(pictureName.size()-1);
    }
    imagTimeSet.push_back(oneSetImageTime);
    cameraTimeSet.push_back(oneSetCameraTime);
    imageSetIdx.push_back(oneSetImageIdx);

    if(0 != pclose(fpin) ) {
        LOG(ERROR) << projectPath << "Close ImagePipeLine Failed";
        return -1;
    }
    if(0 == pictureName.size() ) {
        LOG(ERROR) << projectPath <<"Load Picture List Failed";
        return -1;
    }
    if(imagTimeSet.size() > 2 | 0 == imagTimeSet.size())
    {
        LOG(ERROR) << "find " << imagTimeSet.size() << " time segements that means another day is coming. max value should be 2";
        return -1;
    }
    if(2 == imagTimeSet.size())
    {
        for(int i = imagTimeSet.size() - 1; i > -1; i--)
        {
            for(int k = 0; k < imagTimeSet[i].size(); k++)
            {
                sortPicName.push_back(pictureName[imageSetIdx[i][k]]);
                if(0 == i)
                {
                    sortPicTime.push_back(imagTimeSet[i][k] + 86400.0);
                }
                else
                {
                    sortPicTime.push_back(imagTimeSet[i][k]);
                }

                sortCameraTime.push_back(cameraTimeSet[i][k]);
            }
        }
    }
    else
    {
         for(int i = 0; i < imagTimeSet.size(); i++)
        {
            for(int k = 0; k < imagTimeSet[i].size(); k++)
            {
                sortPicName.push_back(pictureName[imageSetIdx[i][k]]);
                sortPicTime.push_back(imagTimeSet[i][k]);
                sortCameraTime.push_back(cameraTimeSet[i][k]);
            }
        }
    }


    for(int i = 0; (i + 9) < sortPicTime.size();)
    {
        recordI = i;
        isFoundStdTime = true;
        for(int k = recordI; k < (recordI+9); k++)
        {
            i++;
            double picTimeDiff = fabs(sortPicTime[k] - sortPicTime[k + 1]);
            double cameraTimeDiff = fabs(sortCameraTime[k] - sortCameraTime[k + 1]);
            if(fabs(picTimeDiff - cameraTimeDiff) > 0.01)
            {
                isFoundStdTime = false;
                break;
            }
        }
        if(isFoundStdTime)
        {
            LOG(WARNING) << "the "<< recordI <<"th pic time:" << sortPicTime[recordI] << " is taken as accurate time.";
            break;
        }

    }
    if(!isFoundStdTime)
    {
        LOG(ERROR) << "can not find accurate time in the pic List.";
        return -1;
    }


    std::vector<int> editePictureNameMark(sortPicName.size(),0);
    std::vector<std::string> newPictureName(sortPicName);
    //unParsed

    double imageTimeInterval = 1.0/imageCollectionHz*2;
    double setImageTimeInterval = 2.0;
    double imageTimeRecord = 0;
    unsigned long imageMark = 0;
    imageTraceDataFormat oneImageTraceData;
    imageTraceDataFormat oneLostImageTraceData;
    std::vector<imageTraceDataFormat> allImageTraceData;
    std::vector<imageTraceDataFormat> allLostImageTraceData;
    std::string addNameString;
    std::string copyString;
    std::vector<double> stdGpsTime;
    double matchedImgTime = -1.0;
    double deltDaySec = 0.0;

    if(!readFPAGPSTime(projectPath, stdGpsTime))
    {
        LOG(ERROR) << "open fille FPGARecord time file failed.";
        return -1;
    }

    for(unsigned long i = 0; i < sortPicName.size(); i++)
    {

        if(!imageTimeMatched(stdGpsTime, recordI, i, sortPicTime, sortCameraTime, matchedImgTime))
        {
            LOG(WARNING) << "find the pic :" << sortPicName[i] << " FPGA time failed.";
            continue;
        }

        std::string reNewPicture;
        imageMark++;
        mkImageTraceData(matchedImgTime, projectName, imuData, reNewPicture, oneImageTraceData);
        if(reNewPicture.size()==0)
        {
            LOG(WARNING) << "can not find the GPSTIME "<< matchedImgTime << " of the pic:" << sortPicName[i];
            imageMark--;
            continue;
        }
        if(i > 0)
        {
            if(reNewPicture == newPictureName[i-1])
            {
                LOG(WARNING) << "the rename of " << sortPicName[i] << " is " << reNewPicture << " that is same to the last pic" << sortPicName[i-1] << "'s name:" << newPictureName[i-1];
                editePictureNameMark[i] = 2;
                continue;
            }
        }
        newPictureName[i] = reNewPicture;
        oneImageTraceData.addPicName = sortPicName[i].substr(0, sortPicName[i].size() - 4);
        allImageTraceData.push_back(oneImageTraceData);
        editePictureNameMark[i] = 1;
        if(imageMark == 1)
            imageTimeRecord = matchedImgTime;
        else
        {
            double imageTimediffer = fabs(matchedImgTime-imageTimeRecord);
            if(imageTimediffer > imageTimeInterval && imageTimediffer < setImageTimeInterval)
            {
                double imageTimeCopy = imageTimeRecord;
                while(imageTimeCopy < matchedImgTime)
                {
                    imageTimeCopy += imageTimeInterval;
                    mkLostImageTraceData(imageTimeCopy, projectName, imuData, oneLostImageTraceData);
                    allLostImageTraceData.push_back(oneLostImageTraceData);
                }
            }
            imageTimeRecord = matchedImgTime;
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
    for(unsigned long i = 0; i < editePictureNameMark.size(); i++)
    {
        if(0 == (i % img2LidarProsVeloRatio))
        {
            LOG(INFO) << "processNum=" << processNum;
            processNum ++;
            (void)pubProgress(processNum, totalFileNum);
        }
        if(editePictureNameMark[i] == 1)
        {
           FILE * renameImageFpin;
            const std::string renameImagecmd = "cd "+ imagePath +" && mv " + sortPicName[i] + " " + newPictureName[i];
            //LOG(INFO) << "renameImagecmd = " << renameImagecmd;
            if(NULL == ( renameImageFpin = popen(renameImagecmd.c_str(),"w" ) ) )
                LOG(ERROR) << projectPath <<":Rename Picture falied";
            if(0 != pclose(renameImageFpin))
            {
                LOG(WARNING) << "close renameImageFpin Pipe failed";
                continue;
            }
        }
        if(editePictureNameMark[i] == 2)
        {
            LOG(WARNING) << "gone to delete the picture:" << sortPicName[i];
            FILE * deleImageFpin;
            const std::string deleImagecmd="cd "+ imagePath+" && rm " + sortPicName[i];
            if(NULL == ( deleImageFpin = popen(deleImagecmd.c_str(),"w" ) ) )
                LOG(ERROR) << projectPath << ":delete Picture falied";
            if(0 != pclose(deleImageFpin))
            {
                LOG(INFO) << "close deleImageFpin Pipe failed";
                continue;
            }
        }

    }
    unsigned long totalImageNum = editePictureNameMark.size();
    return 0;
}

unsigned long dataFixed::findCorrespondImuIndex(double lidarTime) {
    DLOG(INFO) << __FUNCTION__ << " start, lidarTime: " << lidarTime;

    if(lidarTime < minGPSTime || lidarTime > maxGPSTime) {
        LOG(WARNING) << "LidarPoint time can not find corresponding GPS Time.";
        return -1;
    }

    unsigned long subMarkRecord = 0;
    if(minGPSTimeMark==0 || lidarTime > endGPSTime)
        subMarkRecord=(unsigned long)((lidarTime-beganGPSTime)/hzTime_);
    else
    {
        subMarkRecord=(unsigned long)((lidarTime-minGPSTime)/hzTime_)+minGPSTimeMark;
    }

    return subMarkRecord;
}

int dataFixed::mkLidarTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData){
    //make lidar effect ontime trace file
    LOG(INFO) << __FUNCTION__ << " start, projectPath: " << projectPath;

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
    std::string projectName = belongtoLidarProjectName;


    std::vector <int> lidarPointGPSTimeMark;
    int firstMark=0;
    unsigned long lidarPointSeq=0;
    double tmpLongitude,tmpLatitude,eastCoordinate,northCoordinate;
    double hzSpeed;
    const size_t pktSize=sizeof(double)+BIT_IN_PACKET_DATA;
    double compareTime=100000;
    int findMark=0;
    unsigned long tmpsubMark=0;
    unsigned long savePointID = 0;
    double timeDiff = 2.0;

    const std::string pszDriverName("ESRI shapefile");
    GDALAllRegister();
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName.c_str() );
    if(!poDriver) {
        LOG(ERROR) << pszDriverName << " driver not available.";
        exit(1);
    }

    std::string shapFilePath(projectPath + "/Process/" + projectName);
    const std::string shpFile(shapFilePath + ".shp");

    GDALDataset *poDS = poDriver->Create(shpFile.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    if(!poDS) {
        LOG(ERROR) << "Failed to create shpFile: " << shpFile;
        exit(2);
    }

    OGRLayer *poLayer = poDS->CreateLayer(projectName.c_str(), NULL, wkbLineString, NULL);
    if(!poLayer) {
        LOG(ERROR) << "Failed to create poLayer.";
        exit(3);
    }

    OGRFieldDefn oField("Name", OFTString);
    oField.SetWidth(32);
    if(poLayer->CreateField(&oField) != OGRERR_NONE) {
        LOG(ERROR) << "Failed to create field.";
        exit(4);
    }

    OGRFeature *poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn() );
    poFeature->SetField("Name", projectName.c_str() );

    std::string saveLidarTracePath = projectPath + "/Process/" +projectName + "-RTlaser.txt";
    std::ofstream lidarTraceFile(saveLidarTracePath);
    pktDataFormat *pPktDatas;
    for(int i = 0; i < totalLidarFilePath.size(); i++)
    {
        LOG(INFO) << "processing lidar File:" << totalLidarFilePath[i];
        FILE *lidarFilePointer = fopen(totalLidarFilePath[i].c_str(), "rb");
        if(lidarFilePointer == NULL)
        {
            LOG(WARNING) << "Open lidar File" << totalLidarFilePath[i] << "Failed";
            processNum++;
            (void)pubProgress(processNum, totalFileNum);
            continue;
        }
        LOG(INFO) << "open lidar file successfully.";

        size_t readLength=0;
        fseek(lidarFilePointer,0,SEEK_END);
        size_t oneLidarSize=ftell(lidarFilePointer);
        fseek(lidarFilePointer,0,SEEK_SET);
        const unsigned int pktNum=oneLidarSize/pktSize;
        pPktDatas = new pktDataFormat[pktNum];

        LOG(INFO) << pktNum << " = " << oneLidarSize << "/" << pktSize;

        readLength=fread(pPktDatas, pktSize, pktNum, lidarFilePointer);
        fclose(lidarFilePointer);
        if(readLength != pktNum)
        {
            LOG(WARNING) << "read the Lidar :" << totalLidarFilePath[i] << "Failed";
            processNum++;
            (void)pubProgress(processNum, totalFileNum);
            continue;
        }

        OGRLineString line;
        unsigned long imuIndexLast = -1;
        for(size_t k = 0; k < pktNum; ++k) {
            if(k != 0 && k != (pktNum -1))
            {
                if(k % 200 != 0)
                    continue;
            }

            double oneLidarPointTime = (pPktDatas + k)->timeStamp;
            //LOG(INFO) << "lidar time Stamp is " << oneLidarPointTime;
            unsigned long imuIndex = findCorrespondImuIndex(oneLidarPointTime);

            if(imuIndex < 0| imuIndex >= imuData.size()) {
                LOG(WARNING) << "Failed to findCorrespondImuIndex: " << oneLidarPointTime;
                continue;
            }
            if(imuIndexLast == imuIndex) {
                DLOG(WARNING) << "Same findCorrespondImuIndex: " << imuIndex;
                continue;
            }
            imuIndexLast = imuIndex;

            double pktLng = imuData[imuIndex].Longitude;
            double pktLat = imuData[imuIndex].Latitude;
            line.addPoint(pktLng, pktLat);

            if(0 == k) {
                LOG(INFO) << "First index: " << imuIndex << "; lng: " << pktLng << "; lat: " << pktLat;
            }

            tmpLongitude=imuData[imuIndex].Longitude;
            tmpLatitude=imuData[imuIndex].Latitude;
            GeoToGauss(tmpLongitude,tmpLatitude,3,&northCoordinate,&eastCoordinate);
            hzSpeed=sqrt(pow(imuData[imuIndex].Ve,2) + pow(imuData[imuIndex].Vn,2));
            std::string pointSeqString=std::to_string(lidarPointSeq);
            lidarTraceFile << pointSeqString<<"\t" << std::fixed << std::setprecision(2) <<
            imuData[imuIndex].GPSWeekTime << "\t" <<std::fixed << std::setprecision(4) <<
            northCoordinate << "\t" << std::fixed << std::setprecision(4) << eastCoordinate <<
            "\t" << std::fixed << std::setprecision(3) << imuData[imuIndex].Height <<"\t"
            << std::fixed << std::setprecision(11) << tmpLatitude <<"\t" << std::fixed << std::setprecision(11)
            << tmpLongitude <<"\t" << std::fixed << std::setprecision(4) << hzSpeed << "\t" <<
            std::fixed << std::setprecision(5) << imuData[imuIndex].Roll <<"\t" << std::fixed << std::setprecision(5)
            << imuData[imuIndex].Pitch <<"\t" << std::fixed <<std::setprecision(5) << imuData[imuIndex].Heading
            << "\t" << projectName <<"\t" << imuData[imuIndex].NSV2 << std::endl;
        }

        if(NULL == pPktDatas) {
            LOG(WARNING) << "pPktDatas is: " << pPktDatas;
        }
        else {
            delete[] pPktDatas;
        }
        pPktDatas = NULL;

        poFeature->SetGeometry(&line);
        if(poLayer->CreateFeature(poFeature) != OGRERR_NONE)
        {
            LOG(ERROR) << "Failed to create feature in shapefile.";
            exit(5);
        }
        ++processNum;
        (void)pubProgress(processNum, totalFileNum);
    }

    OGRFeature::DestroyFeature(poFeature);
    GDALClose(poDS);

    lidarTraceFile.close();

    LOG(INFO) << "processNum: " << processNum << "; totalFileNum: " << totalFileNum;
}

bool mkImageList(std::string &projectPath, std::vector<std::string> &imageList)
{
    std::string panoramasPath = projectPath + "/Rawdata/Image/panoramas/";
    std::string cmd = "cd " + panoramasPath + " && ls " + " | grep " +  ".jpg$";
    const size_t maxLine = 1000;
    char result[maxLine];
    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
    {
        LOG(ERROR) << "creat panoramas Image list  Pipe failed";
        return false;
    }
    int i =0;
    while(1)
    {
        i++;
        if(NULL == fgets(result, maxLine, fpin))
        {
            break;
        }
        LOG(INFO) << "i is : " << i;
        std::string tmpPath = result;
        LOG(INFO) << "tmpPath: " << tmpPath;
        if(tmpPath[tmpPath.size()-1] == 10)
            imageList.push_back(tmpPath.substr(0, tmpPath.size()-1 ) );
        else
            imageList.push_back(tmpPath);
    }
    if(0 != pclose( fpin ) )
    {
        LOG(ERROR) <<":close panoramas Pipe Failed";
        return false;
    }
    for(int i = 0; i < imageList.size(); i++)
        LOG(INFO) << "imageList： " << imageList[i];
    if(0 == imageList.size())
    {
        return false;
    }
    return true;
}

void isTheSameFrame(std::string first, std::string second, int& flag)
{
    int length = first.length();
    int i = 0;
    while (first.at(i) != '.')
    {
        if (first.at(i) != second.at(i))
        {
            flag = 0;
            return;
        }
        i++;
    }

    flag = 1;
    return;
}

bool dataFixed::panoramasSort(std::string &projectPath)
{
    std::vector<std::string> imgname;
    std::string format = ".jpg";
    std::vector<ImageStruct> image_struct;
    std::string panoramasPath = projectPath + "//Rawdata//Image//panoramas//";
    std::string otherPath = panoramasPath + "other//";
    std::string imglist_txt_path=projectPath+"//Rawdata//Image//panoramas//imglist.txt";
    bool returnValue = mkImageList(projectPath, imgname);
    if(!returnValue)
    {
        LOG(INFO) << "load panoromas Image List failed";
        return false;
    }
    mkdir(otherPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::ofstream ofn(imglist_txt_path);
    FILE *fpin;
    int size = imgname.size();
    int count = 0;
    for (int i = 0; i < size - 1; i++)
    {
        int flag = 0;
        isTheSameFrame(imgname[i], imgname[i + 1], flag);
        if (flag == 1)
        {
            count++;
        }
        if (i == size - 2 && count != 4)//ÒÆ¶¯
        {
            for (int k = size-1; k > i-count; k--)
            {
                std::string cmd = "mv " + panoramasPath + imgname[k] + " " + otherPath;
                if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
                {
                    LOG(INFO) << "create mv unpanoromas picture pipe failed";
                    return false;
                }
                if(0 != pclose(fpin))
                {
                    LOG(INFO) << "close unpanoromas picture pipe failed";
                }
            }
            break;
        }


       if(flag == 0 && count != 4)
        {
            for(int k = i; k >= i - count; k--)
            {
                std::string cmd = "mv " + panoramasPath + imgname[k] + " " + otherPath;
                if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
                {
                    LOG(INFO) << "creat mv unpanoromas picture pipe failed";
                    return false;
                }
                if(0 != pclose(fpin))
                {
                    LOG(INFO) << "close unpanoromas picture pipe failed";
                }
            }
        count=0;
        continue;
        }
        if (count == 4)
        {
            int sdfasdf = 0;
            for (int index = i + 1; index > i - 4; index--)
            {
                ofn << imgname[index] << std::endl;
            }
            ofn << "#" << std::endl;
            count = 0;
            i++;
            if(i == size -2)
            {
                std::string cmd = "mv " + panoramasPath + imgname[i+1] + " " + otherPath;
                if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
                {
                    LOG(INFO) << "creat mv unpanoromas picture pipe failed";
                    return false;
                }
                if(0 != pclose(fpin))
                {
                    LOG(INFO) << "close unpanoromas picture pipe failed";
                }
            }
        }
        if(0 == (i % img2LidarProsVeloRatio))
        {
            processNum ++;
        (void)pubProgress(processNum, totalFileNum);
    }

    }

    ofn.close();
    imgname.clear();
    std::vector<std::string>(imgname).swap(imgname);
    return true;
}

void dataFixed::pubProgress(unsigned long _processNum, unsigned long _totalFileNum) {
    LOG(INFO) << __FUNCTION__ << " start: " << _processNum << "/" << _totalFileNum;

    progressMsg_.processNum = _processNum;
    progressMsg_.totalFileNum = _totalFileNum;
    pubProgress_.publish(progressMsg_);
}

