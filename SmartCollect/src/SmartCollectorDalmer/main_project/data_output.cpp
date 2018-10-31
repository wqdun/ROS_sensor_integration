#include "data_output.h"

COutputData::COutputData(string outputpath)
{
	OutputPath = outputpath;
	OutputFile = OutputPath;
}

COutputData::~COutputData()
{
	
}

void COutputData::saveResultImage(Mat MatImage, string name)
{
	string out_path        = OutputPath + name;
        const char* out_path_c = out_path.c_str(); 
	imwrite(out_path_c, MatImage);
}

void COutputData::saveResultFile(string filevalue, string name)
{
        string OutputFile_temp   = OutputFile +name + ".txt"; 
        const char* OutputFile_c = OutputFile_temp.c_str();
	OutStream.open(OutputFile_c, std::ios::out|std::ios::app);
	OutStream<<filevalue<<endl;
	OutStream.close();
        cout<<"OutputFile_temp: "<<OutputFile_temp<<endl;
}
