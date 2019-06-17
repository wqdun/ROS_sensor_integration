#include "dataFixed.h"
std::string belongtoLidarProjectName;
dataFixed::dataFixed(int cinValue){
    imageCollectionHz=cinValue;

}
void dataFixed::mkImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,std::string &reNewPicture,imageTraceDataFormat &oneImageTraceData)
{
    std::string constDate="20";
    constDate+=projectName.substr(projectName.size()-7,6);
    int timeInvalRecord=10.0;
    int imuDataSubMark=0;
    int timeMatchedMark=0;
    for(int k=0;k<imuData.size();k++)
    {
        double timeInval=abs(imageTime-imuData[k].GPSWeekTime);
        if(timeInval < timeInvalRecord)
        {
            timeInvalRecord=timeInval;
            imuDataSubMark=k;
            timeMatchedMark=1;
        }
    }
    if(timeMatchedMark==0)
    {
        std::cout << "can't find image corresponding GPS time" << std::endl;
        exit(1);
    }
    int beltNumber=(imuData[imuDataSubMark].Longitude+1.5)/3;
    std::string beltString=std::to_string(beltNumber);
    reNewPicture+=beltString;
    reNewPicture+=projectName;
    int hour=imageTime/3600;
    double leftTime=imageTime-hour*3600;
    int minues=leftTime/60;
    double leftSeconds=leftTime-60*minues;
    int seconds=leftSeconds;
    double mSeconds=leftSeconds-seconds;
    std::string beijingTime;
    std::string dateString;
    hour=hour+8;
    if(hour>23)
        hour=hour-24;
    std::string hourString=std::to_string(hour);
    if(hour<10)
        reNewPicture+='0';
    reNewPicture+=hourString;
    beijingTime+=hourString;
    dateString+=hourString;
    std::string minuesString=std::to_string(minues);
    if(minues<10)
        reNewPicture+='0';
    reNewPicture+=minuesString;
    beijingTime+=minuesString;
    dateString+=minuesString;
    std::string secondString=std::to_string(seconds);
    if(seconds<10)
        reNewPicture+='0';
    reNewPicture+=secondString;
    beijingTime+=secondString;
    dateString+=secondString;
    std::string mSecondString=std::to_string(mSeconds);
    int p=mSecondString.find_last_of(".");
    int r=mSecondString.size()-(p+1);
    if(r<3)
    {
        reNewPicture+=mSecondString.substr(p+1,r);
        beijingTime+=mSecondString.substr(p,r);
        dateString+=mSecondString.substr(p+1,r);
        for(int m=0;m<3-r;m++)
        {
            reNewPicture+='0';
            beijingTime+='0';
            dateString+='0';
        }
        beijingTime+='0';
    }
    else
    {
        reNewPicture+=mSecondString.substr(p+1,3);
        beijingTime+=mSecondString.substr(p,4);
        dateString+=mSecondString.substr(p+1,3);
    }
    std::string date=constDate;
    date+=dateString;
    std::string saveImageName=reNewPicture;
    reNewPicture+=".jpg";
    double tmpLatitude=imuData[imuDataSubMark].Latitude;
    double tmpLongitude=imuData[imuDataSubMark].Longitude;
    double northCoordinate=0;
    double eastCoordinate=0;
    GeoToGauss(tmpLongitude,tmpLatitude,3,&northCoordinate,&eastCoordinate);
    //make image effect ontime trace data
    oneImageTraceData.Pano_name=saveImageName;
    oneImageTraceData.Date=date;
    oneImageTraceData.GpsTime=imageTime;
    oneImageTraceData.BeijingTime=beijingTime;
    oneImageTraceData.Easting=eastCoordinate;
    oneImageTraceData.Northing=northCoordinate;
    oneImageTraceData.H_ell=imuData[imuDataSubMark].Height;
    oneImageTraceData.Latitude=tmpLatitude;
    oneImageTraceData.Longitude=tmpLongitude;
    oneImageTraceData.Roll=imuData[imuDataSubMark].Roll;
    oneImageTraceData.Pitch=imuData[imuDataSubMark].Pitch;
    oneImageTraceData.Heading=imuData[imuDataSubMark].Heading;

}

void dataFixed::mkLostImageTraceData(double imageTime,std::string &projectName,std::vector <ontimeDataFormat> &imuData,imageTraceDataFormat &oneLostImageTraceData)
{
    std::string constDate="20";
    constDate+=projectName.substr(projectName.size()-7,6);
    std::string saveImageName;
    int timeInvalRecord=10.0;
    int imuDataSubMark=0;
    int timeMatchedMark=0;
    int beltNumberRecord=0;
    for(int k=0;k<imuData.size();k++)
    {
        double timeInval=abs(imageTime-imuData[k].GPSWeekTime);
        if(timeInval < timeInvalRecord)
        {
            timeInvalRecord=timeInval;
            imuDataSubMark=k;
            timeMatchedMark=1;
        }
    }
    if(timeMatchedMark==0)
    {
        std::cout << "can't find image corresponding GPS time" << std::endl;
        exit(1);
    }
    int beltNumber=(imuData[imuDataSubMark].Longitude+1.5)/3;
    std::string beltString=std::to_string(beltNumber);
    saveImageName += beltString;
    saveImageName += projectName;
    int hour=imageTime/3600;
    double leftTime=imageTime-hour*3600;
    int minues=leftTime/60;
    double leftSeconds=leftTime-60*minues;
    int seconds=leftSeconds;
    double mSeconds=leftSeconds-seconds;
    std::string beijingTime;
    std::string dateString;
    hour=hour+8;
    if(hour>23)
        hour=hour-24;
    std::string hourString=std::to_string(hour);
    if(hour<10)
    {
        saveImageName+='0';
        beijingTime+='0';
        dateString+='0';
    }
    saveImageName+=hourString;
    beijingTime+=hourString;
    dateString+=hourString;
    std::string minuesString=std::to_string(minues);
    if(minues<10)
    {
        saveImageName+='0';
        beijingTime+='0';
        dateString+='0';
    }
    saveImageName+=minuesString;
    beijingTime+=minuesString;
    dateString+=minuesString;
    std::string secondString=std::to_string(seconds);
    if(seconds<10)
    {
        saveImageName+='0';
        beijingTime+='0';
        dateString+='0';
    }
    saveImageName+=secondString;
    beijingTime+=secondString;
    dateString+=secondString;
    std::string mSecondString=std::to_string(mSeconds);
    int p=mSecondString.find_last_of(".");
    int r=mSecondString.size()-(p+1);
    if(r<3)
    {
        beijingTime+=mSecondString.substr(p,r);
        dateString+=mSecondString.substr(p+1,r);
        saveImageName+=mSecondString.substr(p+1,r);
        for(int m=0;m<3-r;m++)
        {
            beijingTime+='0';
            dateString+='0';
            saveImageName+='0';
        }
        beijingTime+='0';
    }
    else
    {
        beijingTime+=mSecondString.substr(p,4);
        dateString+=mSecondString.substr(p+1,3);
        saveImageName+=mSecondString.substr(p+1,3);
    }
    std::string date=constDate;
    date+=dateString;
    double tmpLatitude=imuData[imuDataSubMark].Latitude;
    double tmpLongitude=imuData[imuDataSubMark].Longitude;
    double northCoordinate=0;
    double eastCoordinate=0;
    GeoToGauss(tmpLongitude,tmpLatitude,3,&northCoordinate,&eastCoordinate);
    //record image effect ontime trace data
    oneLostImageTraceData.Pano_name=saveImageName;
    oneLostImageTraceData.Date=date;
    oneLostImageTraceData.GpsTime=imageTime;
    oneLostImageTraceData.BeijingTime=beijingTime;
    oneLostImageTraceData.Easting=eastCoordinate;
    oneLostImageTraceData.Northing=northCoordinate;
    oneLostImageTraceData.H_ell=imuData[imuDataSubMark].Height;
    oneLostImageTraceData.Latitude=tmpLatitude;
    oneLostImageTraceData.Longitude=tmpLongitude;
    oneLostImageTraceData.Roll=imuData[imuDataSubMark].Roll;
    oneLostImageTraceData.Pitch=imuData[imuDataSubMark].Pitch;
    oneLostImageTraceData.Heading=imuData[imuDataSubMark].Heading;

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

void dataFixed::readOntimeTraceData(std::string &projectPath,std::vector<ontimeDataFormat> &traceFileData){
    std::string traceFilepath=projectPath+"/Rawdata/IMU/imu.txt";
    int existOntimeTraceMark=access(traceFilepath.c_str(),F_OK);
    if(-1 == existOntimeTraceMark)
    {
        std::cout << "There is no imu.txt in IMU File" << std::endl;
        exit(1);
    }
    std::ifstream filePointer(traceFilepath.c_str() );
	if(!filePointer)
	{
	    std::cout<<"Open the file"<<traceFilepath<<"failed"<<std::endl;
		exit(1);
	}
	char buffer[256];
	char header[20],status[20];
    int totalDaySecondTime=24*3600;
	ontimeDataFormat imuonTimeData;
	while(!filePointer.eof())
	{
	    filePointer.getline(buffer,256);

	    std::string line(buffer);
	    std::vector<std::string> parsedLine;
    	boost::split(parsedLine, line, boost::is_any_of(",*") );
        if(17 == parsedLine.size()) {
            int comparResulto=strcmp(parsedLine[15].c_str(),"0C");
            int comparResultw=strcmp(parsedLine[15].c_str(),"00");
            if(comparResulto!=0&&comparResultw!=0)
            {
                std::stringstream ss;
                ss<<parsedLine[2];
                double weekSecondTime;
                ss>>weekSecondTime;
                int modNumber=int(weekSecondTime)/totalDaySecondTime;
                imuonTimeData.GPSWeekTime=weekSecondTime-modNumber*totalDaySecondTime;
                ss.clear();
                ss<<parsedLine[3];
                ss>>imuonTimeData.Heading;
                ss.clear();
                ss<<parsedLine[4];
                ss>>imuonTimeData.Pitch;
                ss.clear();
                ss<<parsedLine[5];
                ss>>imuonTimeData.Roll;
                ss.clear();
                ss<<parsedLine[6];
                ss>>imuonTimeData.Latitude;
                ss.clear();
                ss<<parsedLine[7];
                ss>>imuonTimeData.Longitude;
                ss.clear();
                ss<<parsedLine[8];
                ss>>imuonTimeData.Height;
                ss.clear();
                ss<<parsedLine[9];
                ss>>imuonTimeData.Ve;
                ss.clear();
                ss<<parsedLine[10];
                ss>>imuonTimeData.Vn;
                ss.clear();
                ss<<parsedLine[11];
                ss>>imuonTimeData.Vu;
                ss.clear();
                ss<<parsedLine[12];
                ss>>imuonTimeData.Baseline;
                ss.clear();
                ss<<parsedLine[13];
                ss>>imuonTimeData.NSV1;
                ss.clear();
                ss<<parsedLine[14];
                ss>>imuonTimeData.NSV2;
                traceFileData.push_back(imuonTimeData);
            }

        }

	}
    filePointer.close();
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

void dataFixed::reNameImageAndMkTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData){
    const size_t maxLine=1000;
    char result[maxLine];
    FILE *fpin;
    std::vector <std::string> pictureName;
    std::string projectName="-";
    std::string tmpProjectName;
    std::vector<std::string> filterString;
    std::string processPath=projectPath+"/Process";
    int existProcessMark=access(processPath.c_str(),F_OK);
    if(0 == existProcessMark)
    {
        std::cout << "Data has been processed!Don't repeat!" << std::endl;
        exit(1);
    }
    mkdir(processPath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::string imagePath=projectPath+"/Rawdata/testImage/";
    int existImageMark=access(imagePath.c_str(),F_OK);
    if(-1 ==existImageMark)
    {
        std::cout << "There is no Image File in the " << imagePath << std::endl;
        exit(1);
    }

    boost::split(filterString , projectPath , boost::is_any_of("/"));
    if(filterString.size()==0)
    {
        std::cout << "ProjectPath is worong" << std::endl;
        exit(1);
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
        std::cout << "ProjectPath is worong" << std::endl;
        exit(1);
    }
    for(int k = 0;k < filterString.size();k++)
    {
        projectName+=filterString[k];
        belongtoLidarProjectName+=filterString[k];
    }

    const std::string cmd="cd " + imagePath + " && ls *.jpg";
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) )
    {
        std::cout << "Creat the Image PipeLine Failed" << std::endl;
        exit(1);
    }

     while(1)
    {
        if(NULL == fgets(result, maxLine, fpin))
            break;

        std::string onePictureName=result;
        if(onePictureName[onePictureName.size()-1]==10)
            pictureName.push_back(onePictureName.substr(0,onePictureName.size()-1));
        else
            pictureName.push_back(onePictureName);
    }
    if(0 != pclose(fpin) ) {
        printf("Close PipeLine Falied");
        exit(1);
    }
    if(0 == pictureName.size() ) {
        printf("Load Picture Name Failed");
        exit(1);
    }
    std::vector<int> editePictureNameMark(pictureName.size(),0);
    std::string uselessImageMark="sys";
    std::vector <std::string> newPictureName;
    double imageTime;
    //unParsed

    double imageTimeInterval=1.0/imageCollectionHz*2;
    double setImageTimeInterval=2.0;
    double imageTimeRecord=0;
    unsigned long imageMark=0;
    imageTraceDataFormat oneImageTraceData;
    imageTraceDataFormat oneLostImageTraceData;
    std::vector<imageTraceDataFormat> allImageTraceData;
    std::vector<imageTraceDataFormat> allLostImageTraceData;
    for(int i=0;i<pictureName.size();i++)
    {
        std::string readPartImageName=pictureName[i].substr(0,3);
        if(readPartImageName==uselessImageMark)
        {
            newPictureName.push_back(pictureName[i]);
            continue;
        }
        std::vector<std::string> imageNameSplite;
        boost::split(imageNameSplite, pictureName[i], boost::is_any_of(".") );
        if(3 != imageNameSplite.size())
            continue;
        std::stringstream ss;
        int q=pictureName[i].find_last_of('.');
        ss<<pictureName[i].substr(0,q-1);
        ss>>imageTime;
        std::string reNewPicture;
        imageMark++;
        mkImageTraceData(imageTime,projectName,imuData,reNewPicture,oneImageTraceData);
        if(reNewPicture.size()==0)
        {
            imageMark--;
            continue;
        }
        newPictureName.push_back(reNewPicture);
        allImageTraceData.push_back(oneImageTraceData);
        editePictureNameMark[i]=1;
        if(imageMark == 1)
            imageTimeRecord=imageTime;
        else
        {
            double imageTimediffer=fabs(imageTime-imageTimeRecord);
            if(imageTimediffer > imageTimeInterval && imageTimediffer < setImageTimeInterval)
            {
                double imageTimeCopy=imageTimeRecord;
                while(imageTimeCopy < imageTime)
                {
                    imageTimeCopy += imageTimeInterval;
                    mkLostImageTraceData(imageTimeCopy,projectName,imuData,oneLostImageTraceData);
                    allLostImageTraceData.push_back(oneLostImageTraceData);
                }
            }
            imageTimeRecord=imageTime;
        }

    }
    std::cout <<"Generating Image ontime Trace Data" << std::endl;
    saveImageTraceData(processPath,projectName,allImageTraceData,0);
    if(0 != allLostImageTraceData.size())
    {
        std::cout <<"Generating Lost Image ontime Trace Data" << std::endl;
        saveImageTraceData(processPath,projectName,allLostImageTraceData,1);
    }

    //Rename Picture
    std::cout << "renaming Image" << std::endl;
    for(int i=0; i < editePictureNameMark.size() ; i++)
    {
        if(editePictureNameMark[i]==1)
        {
            FILE * renameImageFpin;
            const std::string renameImagecmd="cd "+ imagePath+" && mv " + pictureName[i] + " " + newPictureName[i];
            //std::cout << renameImagecmd << std::endl;
            if(NULL == ( renameImageFpin = popen(renameImagecmd.c_str(),"w" ) ) )
                std::cout << "Rename Picture falied"<< std::endl;
            pclose(renameImageFpin);
        }
    }
}

void dataFixed::mkLidarTraceFile(std::string &projectPath,std::vector <ontimeDataFormat> &imuData){
    //make lidar effect ontime trace file
    std::string lidarFileTotalpath=projectPath + "/Rawdata/Lidar/";
    int existLidarFileMark=access(lidarFileTotalpath.c_str(),F_OK);
    if(existLidarFileMark == -1)
    {
        std::cout << "There is no Lidar File in the " << lidarFileTotalpath << std::endl;
        exit(1);
    }
    FILE *readLidarFile;
    const size_t maxLine=1000;
    char lidarFilePathName[maxLine];
    std::vector<std::string> totalLidarFilePath;
    const std::string findLidarCmd="ls " + lidarFileTotalpath + "*.dat";
    if(NULL == ( readLidarFile=popen(findLidarCmd.c_str(),"r" ) ) )
    {
        std::cout << "Creat the Lidar PipeLine Failed" << std::endl;
        exit(1);
    }
    while(1)
    {
        if(NULL == fgets ( lidarFilePathName,maxLine,readLidarFile ) )
            break;
        std::string oneLidarFilePath=lidarFilePathName;
        if(oneLidarFilePath[oneLidarFilePath.size()-1] == 10)
            totalLidarFilePath.push_back(oneLidarFilePath.substr(0,oneLidarFilePath.size()-1));
        else
            totalLidarFilePath.push_back(oneLidarFilePath);
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
    for(int i = 0; i < totalLidarFilePath.size();i++)
    {
        FILE *lidarFilePointer=fopen(totalLidarFilePath[i].c_str(),"rb");
        if(lidarFilePointer == NULL)
        {
            std::cout << "Open lidar File " << totalLidarFilePath[i] <<
             "Failed" << std::endl;
             continue;
        }
        size_t readLength=0;
        while(1)
        {
            if( ( readLength=fread(&pktData,pktSize,1,lidarFilePointer) ) !=1)
                break;
            double oneLidarPointTime=pktData.timeStamp;
            double timeInval=10.0;
            int subMarkRecord=0;
            int findMark=0;
            int repeatMark=0;
            for(int k =0;k < imuData.size();k++)
            {
                double timeInvalRecord=abs(oneLidarPointTime-imuData[k].GPSWeekTime);
                if(timeInvalRecord < timeInval)
                {
                    timeInval=timeInvalRecord;
                    subMarkRecord=k;
                    findMark=1;
                }
            }
            if(findMark == 0)
            {
                std::cout << "can't find Lidar Point corresponding GPS time" << std::endl;
                continue;
            }
            if(firstMark == 0)
            {
                firstMark=1;
                lidarPointGPSTimeMark.push_back(subMarkRecord);
            }
            else
            {
                for(int k=0;k <lidarPointGPSTimeMark.size();k++)
                {
                    if(lidarPointGPSTimeMark[k]==subMarkRecord)
                    {
                        repeatMark=1;
                        break;
                    }
                }
                if(repeatMark==0)
                    lidarPointGPSTimeMark.push_back(subMarkRecord);;
            }
            if(repeatMark==1)
                continue;
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

    }
    lidarTraceFile.close();
}
// int main(){




//     std::vector<ontimeDataFormat> imuData;
// 	std::string filePath="/home/dun/1005-1-008566-180131/Rawdata/IMU/imu.txt";
//     readOntimeTraceData(filePath,imuData);

//     if(0==imuData.size())
//     {
//         std::cout << "Load IMU ontime Data failed!" << std::endl;
//         return 1;
//     }


// 	return 0;
// }

//main
//# func() {




//# }