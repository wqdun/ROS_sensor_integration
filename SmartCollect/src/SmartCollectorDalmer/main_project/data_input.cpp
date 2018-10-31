#include "data_input.h"

CInputData::CInputData(string list_str)
{
        const char* list = list_str.c_str();
	input_file.open(list);
	if(!input_file)
	{
	    cout<<"error: data_input -input_file error! "<<endl;
	}
	
	FileListVec.resize(0);

#ifdef DETECTIONSSD
#ifdef SSDOFVOC
        model_file_D_SSD    = "../input/detection_models/ssd/deploy.prototxt";
        weights_file_D_SSD  = "../input/detection_models/ssd/VGG_VOCHADBIGSIGN_SSD_300x300_iter_120000.caffemodel";
#else
        model_file_D_SSD    = "../input/detection_models/ssd_resnet/deploy.prototxt";
        weights_file_D_SSD  = "../input/detection_models/ssd_resnet/VGG_VOC0712Plus_SSD_512x512_iter_83630.caffemodel";
#endif
        mean_file_D_SSD     = " ";          //FLAGS_mean_file;
        mean_value_D_SSD    = "104,117,123";//FLAGS_mean_value;
        file_type_D_SSD     = "image";      //FLAGS_file_type;
        out_file_D_SSD      = " ";          //FLAGS_out_file;
        confidence_threshold_D_SSD  = 0.7;  //FLAGS_confidence_threshold;
#endif

#ifdef  DETECTIONSSDCPU
        model_file_D_SSDCPU    = "../input/detection_models/ssd_resnet_cpu/deploy.prototxt";
        weights_file_D_SSDCPU  = "../input/detection_models/ssd_resnet_cpu/VGG_VOC0712Plus_SSD_512x512_iter_83630.caffemodel";
        mean_file_D_SSDCPU     = " ";          //FLAGS_mean_file;
        mean_value_D_SSDCPU    = "104,117,123";//FLAGS_mean_value;
        file_type_D_SSDCPU     = "image";      //FLAGS_file_type;
        out_file_D_SSDCPU      = " ";          //FLAGS_out_file;
        confidence_threshold_D_SSDCPU  = 0.7;  //FLAGS_confidence_threshold;        
#endif

}

CInputData::~CInputData()
{
	std::vector<string>().swap(FileListVec);
}

bool CInputData::getNameList(void)
{
	while(input_file>>file)
	{
		FileListVec.push_back(file);
	}
	return FileListVec.size()>0 ? true : false;
}

int CInputData::getListSize(void)
{
	return FileListVec.size();
}
bool CInputData::getFileName(const string path,string& name)
{
        if(path==" ")
        {
            cout<< "path is empty!"<<endl;
            return false;
        }
        int pos     = path.find_last_of("/");
        name = path.substr(pos+1,path.size() -4 );
}
