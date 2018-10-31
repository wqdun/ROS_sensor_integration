#include <sstream>
#include "./main_project/data_input.h"
#include "./main_project/data_output.h"
#include "../classification/classification.h"
#include "../detection/detection.h"
#include <boost/algorithm/string.hpp>
#include <vector> 
#include <time.h>
#include "../landload/detLane.h"
//#define DETECTION 100

#define PARAM 100
#ifdef PARAM
DEFINE_string(mean_file, "",
    "The mean file used to subtract from the input image.");
DEFINE_string(mean_value, "104,117,123",
    "If specified, can be one value or can be same as image channels"
    " - would subtract from the corresponding channel). Separated by ','."
    "Either mean_file or mean_value should be provided, not both.");
DEFINE_string(file_type, "image",
    "The file type in the list_file. Currently support image and video.");
DEFINE_string(out_file, "",
    "If provided, store the detection results in the out_file.");
DEFINE_double(confidence_threshold, 0.01,
    "Only store detections with score higher than the threshold.");
#endif

//transpoints
bool transformPoints(cv::Mat matWarp, cv::Rect rectin, cv::Rect& rectout);
//show control
const int FLAGS_display = 0; 

using namespace boost;
 
int main(int argc, char** argv)
{
        ::google::InitGoogleLogging(argv[0]);
        FLAGS_alsologtostderr = 1;
	//[0]数据输入
	string list = "../input/image_input/list.txt"; 
        if(argc==3)
        {
            list = string(argv[1]) + "/list.txt";
        }
	CInputData DataInput(list);
	
	bool bsize = DataInput.getNameList();
	if(!bsize)
	{
		cout<<"error input data is empty!"<<endl;
		return 0;
	}

        cout<<"imagefile list: "<<DataInput.getListSize()<<endl;
#ifdef LANDLOAD
        LaneVerify();
#endif
        
	//[1]模型初始化

#ifdef DETECTIONSSD
        DataInput.mean_file_D_SSD  = FLAGS_mean_file;
        DataInput.mean_value_D_SSD = FLAGS_mean_value;
        Detector detector_SSD(DataInput.model_file_D_SSD, DataInput.weights_file_D_SSD, DataInput.mean_file_D_SSD, DataInput.mean_value_D_SSD);
#endif 

#ifdef DETECTIONSSDCPU
        DataInput.mean_file_D_SSDCPU  = FLAGS_mean_file;
        DataInput.mean_value_D_SSDCPU = FLAGS_mean_value;
        Detector detector_SSDCPU(DataInput.model_file_D_SSDCPU, DataInput.weights_file_D_SSDCPU, DataInput.mean_file_D_SSDCPU, DataInput.mean_value_D_SSDCPU);
#endif
      //[2]开启检测
      string pathout     = "/home/sen/projects/SmartCollector/output/image_debug/";
      string pathoutlist = "/home/sen/projects/SmartCollector/output/text_result/" ;
      if(argc == 3)
      {
          pathout     = string(argv[2]) + "/Detections/image_debug/";
          pathoutlist = string(argv[2]) + "/Detections/text_result/";
      }
      COutputData OutPutData(pathoutlist);
      
      double nanoseconds     = 0;
      int numberOfDetections = 0;

      for(int i = 0 ; i< (int)DataInput.FileListVec.size(); i=i+2) 
      {//process image one by one 
        cout<<"calculate num : \n"<< i <<endl;
	string file = DataInput.FileListVec[i];
        std::cout<<file<<std::endl;
        cv::Mat img = cv::imread(file, -1);
 
        cv::Mat image = img.clone();
                
        if ( img.empty() )
        {
	    std::cout << "Unable to decode image " << file << std::endl;
	    continue;
        }
        namedWindow("imgin",0);
        //imshow("imgin",img);
        //waitKey(10);

        string name = " ";
        bool getnamebool = DataInput.getFileName(file,name);
        if(getnamebool){}
        cout<<"name: "<<name<<endl;
#ifdef DETECTIONSSDCPU 
        LOG(INFO)<<"ST.......";      
        std::vector<vector<float> > detections_SSDCPU = detector_SSDCPU.Detect(img);
        LOG(INFO)<<"ED.......";  
        for (int i = 0; i < static_cast<int>(detections_SSDCPU.size()); ++i) 
	{
            if(static_cast<int>(detections_SSDCPU.size())>50)
            {
                //continue;
            }
            const vector<float>& d = detections_SSDCPU[i];
            
            // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
            CHECK_EQ(d.size(), 7);
	    const float label = d[1];
            const float score = d[2];
	    const int xmin = static_cast<int>(d[3] * img.cols);
	    const int ymin = static_cast<int>(d[4] * img.rows);
	    const int xmax = static_cast<int>(d[5] * img.cols);
	    const int ymax = static_cast<int>(d[6] * img.rows);
            
            if (score >= DataInput.confidence_threshold_D_SSDCPU) 
	    {  
                int xminn = xmin > 0 ? xmin : 0;
                int yminn = ymin > 0 ? ymin : 0;
                int width = xmax-xmin > 0 && xmax-xmin < img.cols ? xmax-xmin : 0;
                int height= ymax-ymin > 0 && ymax-ymin < img.rows ? ymax-ymin : 0;
                if(xminn < 100 && yminn > 1000)
                {
                   continue;
                } 
                if(xminn < 10 || xminn > 1920 || xminn + width > 1920 ){continue;}
                if(yminn < 20 || yminn > 1200 || yminn + height > 1200){continue;}
                if(yminn > 600){continue;} 
                if(height> 400){continue;}              

                cv::Rect rectClassIn(xminn,yminn,width,height);
                cv::rectangle(img, rectClassIn, Scalar(255,0,0), 3, 8, 0);
                
                std::cout << file << " ";
                std::cout << label << " ";
                std::cout << score << " ";
                std::cout << xmin << " ";
                std::cout << ymin << " ";
                std::cout << xmax << " ";
                std::cout << ymax << std::endl;
	        
                string labels = std::to_string(int(label));
                string scores = std::to_string(score);
                string xminns = std::to_string(xminn);
                string yminns = std::to_string(yminn);
                string widths = std::to_string(width);
                string heights= std::to_string(height);
                int label_change = int(label);
                switch(label_change)
                {
                    case 1: 
                    {
                        labels = "5100";
                        break;
                    }
                    case 2: 
                    {
                        labels = "5100";
                        break;
                    }
                }
                
                string filevalue = name + " " + scores + " " + labels +" " + xminns + " " + yminns + " "+widths + " " + heights + "t";
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
	    }
              
        }//detection_ssd result
#endif
#ifdef DETECTIONSSD  
        LOG(INFO)<<"ST.......";       
        std::vector<vector<float> > detections_SSD = detector_SSD.Detect(img);
        LOG(INFO)<<"ED.......";  
        for (int i = 0; i < static_cast<int>(detections_SSD.size()); ++i) 
	{
            if(static_cast<int>(detections_SSD.size())>50)
            {
                //continue;
            }
            const vector<float>& d = detections_SSD[i];
            
            // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
            CHECK_EQ(d.size(), 7);
	    const float label = d[1];
            const float score = d[2];
	    const int xmin = static_cast<int>(d[3] * img.cols);
	    const int ymin = static_cast<int>(d[4] * img.rows);
	    const int xmax = static_cast<int>(d[5] * img.cols);
	    const int ymax = static_cast<int>(d[6] * img.rows);
            
            if (score >= DataInput.confidence_threshold_D_SSD) 
	    {  
                int xminn = xmin > 0 ? xmin : 0;
                int yminn = ymin > 0 ? ymin : 0;
                int width = xmax-xmin > 0 && xmax-xmin < img.cols ? xmax-xmin : 0;
                int height= ymax-ymin > 0 && ymax-ymin < img.rows ? ymax-ymin : 0;
                if(xminn < 100 && yminn > 1000)
                {
                   continue;
                } 
                if(xminn < 10 || xminn > 1920 || xminn + width > 1920 ){continue;}
                if(yminn < 20 || yminn > 1200 || yminn + height > 1200){continue;}
                if(yminn > 600){continue;} 
                if(height> 400){continue;}              

                cv::Rect rectClassIn(xminn,yminn,width,height);
                cv::rectangle(img, rectClassIn, Scalar(255,0,0), 3, 8, 0);
                
                std::cout << file << " ";
                std::cout << label << " ";
                std::cout << score << " ";
                std::cout << xmin << " ";
                std::cout << ymin << " ";
                std::cout << xmax << " ";
                std::cout << ymax << std::endl;
	        
                string labels = std::to_string(int(label));
                string scores = std::to_string(score);
                string xminns = std::to_string(xminn);
                string yminns = std::to_string(yminn);
                string widths = std::to_string(width);
                string heights= std::to_string(height);
                int label_change = int(label);
                switch(label_change)
                {
                    case 1: 
                    {
                        labels = "5100";
                        break;
                    }
                    case 2: 
                    {
                        labels = "5100";
                        break;
                    }
                }
                
                string filevalue = name + " " + scores + " " + labels +" " + xminns + " " + yminns + " "+widths + " " + heights;
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
	    }
              
        }//detection_ssd result
#endif


#ifndef  IMSHOW 
          imshow("imgin",img);       
          string nameout = pathout + name;
          imwrite(nameout,img);
          waitKey(50);
#endif 

    }//image list out
	return 0;
 
}

bool transformPoints(cv::Mat matWarp, cv::Rect rectin, cv::Rect& rectout)
{
    int height = matWarp.rows;
    int width  = matWarp.cols;
    int step   = matWarp.step;
    uchar* data= (uchar*)matWarp.data;
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            int value = data[i*step + j];
            cout<<value<<" ";
        }
        cout<<endl;
    }
    return true;
}

