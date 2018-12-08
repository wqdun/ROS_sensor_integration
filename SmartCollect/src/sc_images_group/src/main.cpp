#include <stdio.h>

#include <stdlib.h>

#include <string>

#include <vector>

#include <fstream>

#include <iostream>

#include <cmath>

#include <iomanip>

//#include <glog/logging.h>
using namespace std;

struct imageTraceDataFormat{
    std::string Pano_name;
    std::string Date;
    double GpsTime;
    std::string BeijingTime;
    double Easting;
    double Northing;
    double H_ell;
    double Latitude;
    double Longitude;
    double Roll;
    double Pitch;
    double Heading;
};

struct PanoGroupElement{
    std::string leftImg;
    std::string middleImg;
    std::string middleImgSrcName;
    std::string rightImg;
    std::string beltNum;
};

struct CameraElements{

    string SrcName;
    string SysName;
    string GPSTimeStamp;
    int Index;
    string InsideTime;
    string IP;
};

void ReadToVector(std::vector<imageTraceDataFormat>& m_imageTraceDataList)
{
    std::string file_path="/media/guowenxian/数据/3AVE/1001-1-50004-180624/Process/35-1001150004180624-RTimgpost.txt";
    std::ifstream ifn(file_path);
    if(!ifn)
    {
	std::cout<<"fail to open RTimgpost.txt"<<std::endl;
	return;
    }

    imageTraceDataFormat m_imageTraceDataFormat;
    while(!ifn.eof())
    {
	std::string temp="";
	getline(ifn,temp);
	ifn>>temp;
	ifn>>m_imageTraceDataFormat.Pano_name;
	ifn>>m_imageTraceDataFormat.Date;

	ifn>>temp;
	m_imageTraceDataFormat.GpsTime=atof(temp.c_str());

	ifn>>m_imageTraceDataFormat.BeijingTime;

	ifn>>temp;
	m_imageTraceDataFormat.Easting=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.Northing=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.H_ell=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.Latitude=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.Longitude=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.Roll=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.Pitch=atof(temp.c_str());
	ifn>>temp;
	m_imageTraceDataFormat.Heading=atof(temp.c_str());

	m_imageTraceDataList.push_back(m_imageTraceDataFormat);

    }


}

bool getformatList(std::string &cmd, std::vector<std::string> &fileList)
{
   //  std::cout<<"cmd"<<cmd<<std::endl;
    FILE *fpin = popen(cmd.c_str(), "r");//"../test/"

    if(NULL == fpin)
    {
	std::cout<<"fail to open file"<<std::endl;
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
        else{

            fileList.push_back(tmpString);
	}
    }
    if(0 != pclose(fpin))
    {
        return false;
    }
    return true;

}

bool getformatCount(std::string &cmd, int& count)
{
    FILE *fpin = popen(cmd.c_str(), "r");//"../test/"
    if(NULL == fpin)
    {
	std::cout<<"fail to open file"<<std::endl;
        return false;
    }
    const size_t maxLine = 1000;
    char result[maxLine];
    while(1)
    {
        if( NULL == fgets(result, maxLine, fpin))
            break;
        count++;
    }
    if(0 != pclose(fpin))
    {
        return false;
    }
    return true;
}
void ParseNameProcess(std::string ImgName,CameraElements &m_CameraElements)
{

    m_CameraElements.SrcName=ImgName;
    std::string temp=ImgName;

    int index=temp.find_first_of("_",0);
    m_CameraElements.GPSTimeStamp=temp.substr(0,index);

    temp=temp.substr(index+1,temp.length()-index);
    index=temp.find_first_of("_",0);
    m_CameraElements.Index=atof(temp.substr(0,index).c_str());

    temp=temp.substr(index+1,temp.length()-index);
    index=temp.find_first_of("_",0);
    m_CameraElements.InsideTime=temp.substr(0,index);

    temp=temp.substr(index+1,temp.length()-index);
    index=temp.find_first_of(".",0);
    m_CameraElements.IP=temp.substr(0,index);
}
bool ParseAssistImgToStructList(std::vector<string> &namelist,std::vector<CameraElements> &AssistImage)
{

   for(int i=0;i<namelist.size();i++)
   {
	CameraElements m_CameraElements;
	//m_CameraElements.SrcName=namelist.at(i);
	ParseNameProcess(namelist.at(i),m_CameraElements);

	AssistImage.push_back(m_CameraElements);
    }

    return true;
}

bool ParseCoreImgToStructList(std::string PostPath,std::vector<CameraElements> &CoreImage)
{
    ifstream ifn(PostPath.c_str());
    if(!ifn)
    {
	std::cout<<"open imgpost.txt error"<<std::endl;
	return false;
    }
    while(!ifn.eof())
    {
	CameraElements m_CameraElements;

	for(int i=0;i<14;i++)
	{
	    string temp;
	    ifn>>temp;
	    if(temp.empty())
	        return true;
	    if(i==1)
		m_CameraElements.SysName=temp+".jpg";//rename name
	    if(i==2)
		m_CameraElements.SrcName=temp+".jpg";//src name
	}

	ParseNameProcess(m_CameraElements.SrcName,m_CameraElements);
	CoreImage.push_back(m_CameraElements);
    }


    return true;
}

bool ComputeInsideClockDiff(std::vector<CameraElements> &leftImgList,
			    CameraElements first_middleImg,
			    std::vector<CameraElements> &rightImgList,
			    double &left_diff,double &right_diff)
{
    double core_GPSTime=atof(first_middleImg.GPSTimeStamp.c_str());
    double core_InsideTime=atof(first_middleImg.InsideTime.c_str());

   CameraElements left_minDiff_frame;
   double mindiff=1000;
   for(int i =0;i<leftImgList.size();i++)
   {
	double current_GPSTime=atof(leftImgList.at(i).GPSTimeStamp.c_str());
	double tempdiff=current_GPSTime-core_GPSTime;
	tempdiff=abs(tempdiff);
	if(tempdiff<=mindiff)
	{
	    mindiff=tempdiff;
	    left_minDiff_frame=leftImgList.at(i);
        }
    }
    double inside_diff_left=atof(left_minDiff_frame.InsideTime.c_str())-core_InsideTime;

cout<<"min name="<<left_minDiff_frame.SrcName<<endl;
cout<<"current_name"<<first_middleImg.SrcName<<endl;
cout<<"gps diff:"<<std::setprecision(13)<<mindiff<<endl;
cout<<"left diff:"<<std::setprecision(13)<<inside_diff_left<<endl;

   CameraElements right_minDiff_frame;
   mindiff=1000;
   for(int i =0;i<rightImgList.size();i++)
   {
	double current_GPSTime=atof(rightImgList.at(i).GPSTimeStamp.c_str());
	double tempdiff=current_GPSTime-core_GPSTime;
	tempdiff=abs(tempdiff);
	if(tempdiff<=mindiff)
	{
	    mindiff=tempdiff;
	    right_minDiff_frame=rightImgList.at(i);
        }
  //      cout<<std::setprecision(13)<<tempdiff<<endl;

    }
    double inside_diff_right=atof(right_minDiff_frame.InsideTime.c_str())-core_InsideTime;

left_diff=inside_diff_left;
right_diff=inside_diff_right;

cout<<"min name="<<right_minDiff_frame.SrcName<<endl;
cout<<"current_name"<<first_middleImg.SrcName<<endl;
cout<<"gps diff:"<<std::setprecision(13)<<mindiff<<endl;
cout<<"right diff:"<<std::setprecision(13)<<inside_diff_left<<endl;

}
bool GetNearestImg(CameraElements &m_CameraElements,std::vector<CameraElements> &ImgList,double insideTime_diff,std::string &outImgName)
{
    double min_diff=1000;

    double middle_insideTime=atof(m_CameraElements.InsideTime.c_str());
    for(int i=0;i<ImgList.size();i++)
    {
	double current_insideTime=atof(ImgList.at(i).InsideTime.c_str());
	double temp_diff=abs(current_insideTime-middle_insideTime-insideTime_diff);
	if(temp_diff<min_diff)
	{
	    min_diff=temp_diff;
	    outImgName=ImgList.at(i).SrcName;
	}

    }
    if(min_diff<0.01)
	return true;
    return false;
}

void GetPanoGroup(std::vector<CameraElements> &leftImgList,
		  std::vector<CameraElements> &middleImgList,
		  std::vector<CameraElements> &rightImgList,
		  std::vector<PanoGroupElement> &panoList,
		   double &left_diff,double &right_diff)
{

    for(int i=3;i<middleImgList.size();i++)
    {
	PanoGroupElement m_PanoGroupElement;
	m_PanoGroupElement.middleImg=middleImgList.at(i).SysName;//待替换成重命名后的名字
	m_PanoGroupElement.middleImgSrcName=middleImgList.at(i).SrcName;//原始命名，匹配率检查时用

	std::string outImgName="";
        bool flag=GetNearestImg(middleImgList.at(i),leftImgList,left_diff,outImgName);
	if(flag==false)
           continue;
	m_PanoGroupElement.leftImg=outImgName;

	flag=GetNearestImg(middleImgList.at(i),rightImgList,right_diff,outImgName);
	if(flag==false)
           continue;
	m_PanoGroupElement.rightImg=outImgName;


	cout<<m_PanoGroupElement.leftImg<<endl;
	cout<<m_PanoGroupElement.middleImg<<endl;
	cout<<m_PanoGroupElement.middleImgSrcName<<endl;
	cout<<m_PanoGroupElement.rightImg<<endl<<endl;

	panoList.push_back(m_PanoGroupElement);

    }

}
void SavePanoGroupList(std::string file_path,std::vector<PanoGroupElement> &PanoGroupList)
{
    ofstream ofn(file_path.c_str());
    for(int i=0;i<PanoGroupList.size();i++)
    {
	ofn<<PanoGroupList.at(i).leftImg<<std::endl;
	ofn<<PanoGroupList.at(i).middleImg<<std::endl;
	ofn<<PanoGroupList.at(i).rightImg<<std::endl;
	ofn<<"#"<<std::endl;

    }

}
bool ImgGroupProcess(std::string &project_path,std::vector<PanoGroupElement> &PanoGroupList)
{
 //   std::string pano_path=project_path+"/media/guowenxian/数据/3AVE/1001-1-50004-180624/Rawdata/Image/panoramas/";
    std::string pano_path=project_path+"/Rawdata/Image/panoramas";
//left
    std::string cmd1="cd "+pano_path+";"+"/bin/ls *_0.jpg";
    std::vector<std::string> leftImagename_List;
    getformatList(cmd1,leftImagename_List);
    std::vector<CameraElements> LeftImage;
    ParseAssistImgToStructList(leftImagename_List,LeftImage);

//right
    std::string cmd2="cd "+pano_path+";"+"/bin/ls *_2.jpg";
    std::vector<std::string> rightImagename_List;
    getformatList(cmd2,rightImagename_List);
    std::vector<CameraElements> RightImage;
    ParseAssistImgToStructList(rightImagename_List,RightImage);

//middle
   std::string cmd3="cd "+project_path+"/Process/;"+"/bin/ls *-RTimgpost.txt";
   vector<std::string> temp;
   getformatList(cmd3,temp);
   if(temp.size()==0)
   {
	std::cout<<"there is no RTimgpost.txt"<<std::endl;
        return 0;
    }


    std::string RTimgPost_Path=project_path+"Process/"+temp.at(0);
    std::vector<CameraElements> MiddleImage;
    ParseCoreImgToStructList(RTimgPost_Path,MiddleImage);

//------------------------------------------------------------------------
    double left_diff=0;
    double right_diff=0;
    ComputeInsideClockDiff(LeftImage,MiddleImage.at(3),RightImage,left_diff,right_diff);

//------------------------------------------------------------------------------------
    GetPanoGroup(LeftImage,MiddleImage,RightImage,PanoGroupList,left_diff,right_diff);
cout<<PanoGroupList.size()<<endl;

    return true;
}
bool isImgLost(std::string srcImgName,std::vector<PanoGroupElement> &m_PanoGroupList)
{

    for(int i=0;i<m_PanoGroupList.size();i++)
    {

	if(srcImgName==m_PanoGroupList.at(i).middleImgSrcName)
	    return true;
    }
    return false;


}
void saveComputeMatchResult(std::string &Matchresult_path,std::vector<string> lostImgList,int totalsize)
{
    ofstream ofn(Matchresult_path.c_str());
    if(!ofn)
    {
	std::cout<<"fail to create MatchResult file"<<std::endl;
	return ;
    }

    ofn<<"lostNum = "<<lostImgList.size()<<std::endl;
    std::cout<<"lostNum = "<<lostImgList.size()<<std::endl;

    ofn<<"totalNum = "<<totalsize<<std::endl;
    std::cout<<"totalNum = "<<totalsize<<std::endl;

    double probability = 100.0*lostImgList.size()/totalsize;
    ofn<<"Pano Img lost probability =  "<<probability<<"%"<<std::endl;
    std::cout<<"Pano Img lost probability =  "<<probability<<"%"<<std::endl;


    ofn<<"lost img list :"<<std::endl;
    std::cout<<"lost img list :"<<std::endl;
    for(int i=0;i<lostImgList.size();i++)
    {
	ofn<<lostImgList.at(i)<<std::endl;
	std::cout<<lostImgList.at(i)<<std::endl;
    }





}
bool ComputeMatching(std::string file_path,std::vector<PanoGroupElement> &m_PanoGroupList,std::vector<std::string> &lostImgList,int &allsize)
{
    ifstream ifn(file_path);
    if(!ifn)
    {
	std::cout<<"fail to open imgsrc list file"<<std::endl;
	return 0;
     }

    vector<string> srcImglist;
    while(!ifn.eof())
    {
	std::string temp;
	ifn>>temp;
        if(temp.empty())
	    break;
	srcImglist.push_back(temp);
    }
    ifn.close();
    allsize=srcImglist.size();

    for(int i=0;i<srcImglist.size();i++)
    {
	std::string srcImgName=srcImglist.at(i);
        bool flag=isImgLost(srcImgName,m_PanoGroupList);
	if(flag==false&&!srcImgName.empty())
        {
	    lostImgList.push_back(srcImgName);
	}
    }
   return true;
}

int main(int argc,char **argv)
{


/*
string cmd="cd /media/guowenxian/数据/0000-1-10000-181206/Rawdata/Image/;ls *.jpg";
vector<string> list;
getformatList(cmd,list);
ofstream ofn("/media/guowenxian/数据/0000-1-10000-181206/Rawdata/Image/RTimgPost.txt");
for(int i=0;i<list.size();i++)
{
ofn<<i<<"    "<<"20181206******"<<to_string(i)<<"    "<<list.at(i).substr(0,list.at(i).length()-4);
ofn<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<"    "<<"****"<<endl;
}
return 0;
*/

    if(argc!=2)
    {
	std::cout<<"input paramers is err,should be ./test project_path"<<std::endl;
	std::cout<<"for example: ./test /media/guowenxian/mosquito/move/1001-1-10005-180602/"<<std::endl;
	return 0;
    }

    std::string project_path=argv[1];
    project_path+="/";

//----------------------------------
    std::vector<imageTraceDataFormat> m_imageTraceDataList;

    ReadToVector(m_imageTraceDataList);

    std::vector<PanoGroupElement> m_PanoGroupList;
    ImgGroupProcess(project_path,m_PanoGroupList);

    std::string panofile_path=project_path+"Rawdata/Image/imglist.txt";
    SavePanoGroupList(panofile_path,m_PanoGroupList);
//统计匹配率
    std::string alllist_path=project_path+"Rawdata/Image/alllist.txt";
    std::vector<std::string> lostPanoImg;
    int allsize=0;
    // ComputeMatching(alllist_path,m_PanoGroupList,lostPanoImg,allsize);

    // std::string lostPano_path=project_path+"Process/PanoLostLog.txt";
    // saveComputeMatchResult(lostPano_path,lostPanoImg,allsize);

    return 0;
}

