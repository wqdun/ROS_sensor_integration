#include <sstream>
#include "./main_project/data_input.h"
#include "./main_project/data_output.h"
#include "../classification/classification.h"
#include "../detection/detection.h"
#include "../detection_cpu/detection_cpu.h"
#include <boost/algorithm/string.hpp>
#include <vector> 
#include <time.h>
#include "../landload/detLane.h"
//#define USE_OPENCV 100
#define DETECTION 100
#define CLASSIFICATION 100
#define DETECTIONSSD 100
//#define DETECTIONSSDCPU 100

#ifdef CLASSIFICATION
   #define OTHERHD    100
#else 
   //#define OTHERHD    100
#endif 

#ifdef DETECTION
/*DEFINE_string(mean_file, "",
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
    "Only store detections with score higher than the threshold.");*/
#endif

#ifdef DETECTIONSSD
/*DEFINE_string(mean_file, "",
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
    "Only store detections with score higher than the threshold.");*/
#endif

#ifndef INIT
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
#ifdef DETECTION
        DataInput.mean_file_D  = FLAGS_mean_file;
        DataInput.mean_value_D = FLAGS_mean_value;
        //DataInput.confidence_threshold_D = FLAGS_confidence_threshold;
        Detector detector(DataInput.model_file_D, DataInput.weights_file_D, DataInput.mean_file_D, DataInput.mean_value_D);
#endif 
#ifdef DETECTIONSSD
        DataInput.mean_file_D_SSD  = FLAGS_mean_file;
        DataInput.mean_value_D_SSD = FLAGS_mean_value;
        Detector detector_SSD(DataInput.model_file_D_SSD, DataInput.weights_file_D_SSD, DataInput.mean_file_D_SSD, DataInput.mean_value_D_SSD);
#endif 

#ifdef DETECTIONSSDCPU
        DataInput.mean_file_D_SSDCPU  = FLAGS_mean_file;
        DataInput.mean_value_D_SSDCPU = FLAGS_mean_value;
        DetectorCPU detector_SSDCPU(DataInput.model_file_D_SSDCPU, DataInput.weights_file_D_SSDCPU, DataInput.mean_file_D_SSDCPU, DataInput.mean_value_D_SSDCPU);
#endif 

#ifdef MARKDETECTION
        DataInput.mean_file_D_MARK  = FLAGS_mean_file;
        DataInput.mean_value_D_MARK = FLAGS_mean_value;
        Detector detector_MARK(DataInput.model_file_D_MARK, DataInput.weights_file_D_MARK, DataInput.mean_file_D_MARK, DataInput.mean_value_D_MARK);        
#endif
#ifdef POLEDETECTION
        DataInput.mean_file_D_POLE  = FLAGS_mean_file;
        DataInput.mean_value_D_POLE = FLAGS_mean_value;
        Detector detector_POLE(DataInput.model_file_D_POLE, DataInput.weights_file_D_POLE, DataInput.mean_file_D_POLE, DataInput.mean_value_D_POLE);        
#endif  
      
#ifdef CLASSIFICATION 
        Classifier classifier(DataInput.model_file_C, DataInput.weights_file_C, DataInput.mean_file_C, DataInput.label_file_C);
#endif 

      cv::Point2f src_vertices[4] = { Point2f(671, 735) ,Point2f(1163, 725) ,Point2f(118, 1200) ,Point2f(1722, 1200) };
      cv::Point2f dst_vertices[4] = { Point2f(800,700),Point2f(1200,700),Point2f(800,1200),Point2f(1200,1200) };
      Mat warpMatrix = getPerspectiveTransform(src_vertices, dst_vertices);
      //[2]开启检测
      string pathout     = "/home/sen/projects/SmartCollectorJP/output/image_debug/";
      string pathoutlist = "/home/sen/projects/SmartCollectorJP/output/text_result/" ;
      
      COutputData OutPutData(pathoutlist);
      
      double nanoseconds     = 0;
      int numberOfDetections = 0;

      for(int i = 0 ; i< (int)DataInput.FileListVec.size(); i=i+1) 
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
        
#ifdef DETECTION 
        clock_t time_st = clock();
        std::vector<vector<float> > detections = detector.Detect(img);
        clock_t time_ed = clock();

        nanoseconds = (time_ed - time_st);///CLOCKS_PER_SEC;
        cout<<"detection time cost: "<<nanoseconds<<" s"<<endl;
        ++numberOfDetections;
        
        /* Print the detection results. */
        int detection_count = 0;
        for (int i = 0; i < static_cast<int>(detections.size()); ++i) 
	{
            const vector<float>& d = detections[i];
           
            // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
            CHECK_EQ(d.size(), 7);
	    const float label = d[1];
            const float score = d[2];
	    const int xmin = static_cast<int>(d[3] * img.cols);
	    const int ymin = static_cast<int>(d[4] * img.rows);
	    const int xmax = static_cast<int>(d[5] * img.cols);
	    const int ymax = static_cast<int>(d[6] * img.rows);
            
            if (score >= DataInput.confidence_threshold_D) 
	    {  

                int xminn = xmin > 0 ? xmin : 0;
                int yminn = ymin > 0 ? ymin : 0;
                int width = xmax-xmin > 0 && xmax-xmin < img.cols ? xmax-xmin : 0;
                int height= ymax-ymin > 0 && ymax-ymin < img.rows ? ymax-ymin : 0;
                if(xminn < 100 && yminn > 800)
                {
                   continue;
                }  
                if(xminn < 10 || xminn > 1920 || xminn + width > 1920 ){continue;}
                if(yminn < 20 || yminn > 1200 || yminn + height > 1200){continue;}              

                cv::Rect rectClassIn(xminn,yminn,width,height);
                cv::rectangle(img, rectClassIn, Scalar(255,0,0), 3, 8, 0);
                

                string name_local = " ";
                stringstream ss;
                ss<<numberOfDetections;
                ss>>name_local;
                name_local = pathout + name_local + ".jpg";
                const char* temp_path = name_local.c_str();
                cout<<"temp_path: "<<temp_path<<endl;

                if(xminn+width > img.cols || yminn+height > img.rows)
                {
                    continue;
                }
                if(xminn < 10 || xminn > 1920 || xminn + width > 1920 ){continue;}
                if(yminn < 20 || yminn > 1200 || yminn + height > 1200){continue;}
#ifdef CLASSIFICATION
                cv::Mat  matClassIn(height, width, CV_8UC3, cv::Scalar::all(0));
                cout<<"xminn: "<< xminn<<endl;
                cout<<"yminn: "<< yminn<<endl;
                cout<<"width: "<< width<<endl;
                cout<<"height: "<< height<<endl;

		img(rectClassIn).copyTo(matClassIn);
                
#endif
                string labels = std::to_string(label);
                string scores = std::to_string(score);
                string xminns = std::to_string(xminn);
                string yminns = std::to_string(yminn);
                string widths = std::to_string(width);
                string heights= std::to_string(height);
                string filevalue = name + " " + scores + " " +labels +" " + xminns + " " + yminns + " "+widths + " " + heights;
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
#ifdef CLASSIFICATION

		std::vector<Prediction> predictions = classifier.Classify(matClassIn);
				
		Prediction p;
		for (size_t i = 0; i < 1; ++i) //predictions.size(); ++i)
		{
                    p = predictions[i];
#ifndef DEBUG 
                    std::cout << std::fixed << std::setprecision(4) << p.second << " - \""<< p.first << "\"" << std::endl;
#endif 
                }
#endif //CLASSIFICATION
                std::cout << file << " ";
                std::cout << label << " ";
                std::cout << score << " ";
                std::cout << xmin << " ";
                std::cout << ymin << " ";
                std::cout << xmax << " ";
                std::cout << ymax << std::endl;
#ifdef CLASSIFICATION
		std::cout << std::fixed << std::setprecision(4) << p.second << " - \""<< p.first << "\"" << std::endl;

		stringstream ss_C;
                string number_str_C =" ";
                ss_C<<p.first;
                ss_C>>number_str_C;

                stringstream ss_confidence_C;
                float number_C = -0.1;

                ss_confidence_C<<p.second;
                ss_confidence_C>>number_C;
                cout<<"number_C: "<<number_C<<endl;
                
                string path_show = " ";
#endif
#ifdef OTHERHD 
                if(number_str_C != " "&& number_C- 0.7 > 0.0001 && (number_str_C== "0" || number_str_C =="1" || number_str_C == "2" ||number_str_C == "3" || number_str_C == "4" || number_str_C =="5" || number_str_C == "6"|| number_str_C == "7" || number_str_C == "8"
|| number_str_C == "9"|| number_str_C == "10"))
                {
                    string labels_target;
                    if(number_str_C== "0"){labels_target="31";}
                    if(number_str_C== "1"){labels_target="43";}
                    if(number_str_C== "2"){labels_target="48";}
                    if(number_str_C== "3"){labels_target="62";}
                    if(number_str_C== "4"){labels_target="63";}
                    if(number_str_C== "5"){labels_target="79";}
                    if(number_str_C== "6"){labels_target="81";}
                    if(number_str_C== "7"){labels_target="86";}
                    if(number_str_C== "8"){labels_target="87";}
                    if(number_str_C== "9"){labels_target="88";}
                    if(number_str_C== "10"){labels_target="90";}

                    string filevalue = name + " " + scores + " " + labels_target +" " + xminns + " " + yminns + " "+widths + " " + heights;
                    OutPutData.saveResultFile(filevalue,name);
                    path_show = DataInput.standard_image_path_C + number_str_C + ".jpg";
                    cv::Mat mat_standard   = imread(path_show);
                    cv::Mat mat_standard_resize;
                    cv::resize(mat_standard,mat_standard_resize,cv::Size(100,100));
                    cv::Rect rect_standard(1620-detection_count,800,mat_standard_resize.cols,mat_standard_resize.rows);
                    detection_count = detection_count + 120;
                    mat_standard_resize.copyTo(img(rect_standard));
                } 
                 
#endif //OTHERHD
	      }//if confidence result
              
	  }//detection result
#endif //DETECTION 

#ifdef DETECTIONSSDCPU      
        std::vector<vector<float> > detections_SSDCPU = detector_SSDCPU.Detect(img);
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
                cv::rectangle(img, rectClassIn, Scalar(0,255,0), 3, 8, 0);
                
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
                        labels = "9001";
                        break;
                    }
                    case 2: 
                    {
                        labels = "9002";
                        break;
                    }
                    case 3: 
                    {
                        labels = "9003";
                        break;
                    }
                    case 4: 
                    {
                        labels = "9004";
                        break;
                    }
                    case 5: 
                    {
                        labels = "9005";
                        break;
                    }
                    case 6: 
                    {
                        labels = "9101";
                        break;
                    }
                }
                
                string filevalue = name + " " + scores + " " + labels +" " + xminns + " " + yminns + " "+widths + " " + heights;
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
	    }
              
        }//detection_ssd result
#endif


#ifdef DETECTIONSSD       
        std::vector<vector<float> > detections_SSD = detector_SSD.Detect(img);
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
                cv::rectangle(img, rectClassIn, Scalar(0,0,255), 3, 8, 0);
                
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
                        labels = "9001";
                        break;
                    }
                    case 2: 
                    {
                        labels = "9002";
                        break;
                    }
                    case 3: 
                    {
                        labels = "9003";
                        break;
                    }
                    case 4: 
                    {
                        labels = "9004";
                        break;
                    }
                    case 5: 
                    {
                        labels = "9005";
                        break;
                    }
                    case 6: 
                    {
                        labels = "9101";
                        break;
                    }
                }
                
                string filevalue = name + " " + scores + " " + labels +" " + xminns + " " + yminns + " "+widths + " " + heights;
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
	    }
              
        }//detection_ssd result
#endif


#ifdef POLEDETECTION       
        std::vector<vector<float> > detections_POLE = detector_POLE.Detect(img);
        for (int i = 0; i < static_cast<int>(detections_POLE.size()); ++i) 
	{
            if(static_cast<int>(detections_POLE.size())>50)
            {
                //continue;
            }
            const vector<float>& d = detections_POLE[i];
            
            // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
            CHECK_EQ(d.size(), 7);
	    const float label = d[1];
            const float score = d[2];
	    const int xmin = static_cast<int>(d[3] * img.cols);
	    const int ymin = static_cast<int>(d[4] * img.rows);
	    const int xmax = static_cast<int>(d[5] * img.cols);
	    const int ymax = static_cast<int>(d[6] * img.rows);
            
            if (score >= DataInput.confidence_threshold_D_POLE) 
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
                //if(xminn < 950){continue;}            

                cv::Rect rectClassIn(xminn,yminn,width,height);
                cv::rectangle(img, rectClassIn, Scalar(0,255,0), 3, 8, 0);
                
                std::cout << file << " ";
                std::cout << label << " ";
                std::cout << score << " ";
                std::cout << xmin << " ";
                std::cout << ymin << " ";
                std::cout << xmax << " ";
                std::cout << ymax << std::endl;
	        
                string labels = std::to_string(label);
                string scores = std::to_string(score);
                string xminns = std::to_string(xminn);
                string yminns = std::to_string(yminn);
                string widths = std::to_string(width);
                string heights= std::to_string(height);
                string filevalue = name + " " + scores + " " + labels +" " + xminns + " " + yminns + " "+widths + " " + heights;
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
	    }
              
        }//detection_POLE result
#endif
       //imshow("imgin",img);
       //waitKey(1000);
#ifdef MARKDETECTION 
        cv::Mat rotated;
	warpPerspective(img, rotated, warpMatrix, rotated.size(), INTER_LINEAR, BORDER_CONSTANT); 
     	
        std::vector<vector<float> > detections_MARK = detector_MARK.Detect(rotated);
        for (int i = 0; i < static_cast<int>(detections_MARK.size()); ++i) 
	{
            if(static_cast<int>(detections_MARK.size())>50)
            {
                //continue;
            }
            const vector<float>& d = detections_MARK[i];
            
            // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
            CHECK_EQ(d.size(), 7);
	    const float label = d[1];
            const float score = d[2];
	
	    const int xmin = static_cast<int>(d[3] * img.cols);
	    const int ymin = static_cast<int>(d[4] * img.rows);
	    const int xmax = static_cast<int>(d[5] * img.cols);
	    const int ymax = static_cast<int>(d[6] * img.rows);
            
            if (score >= DataInput.confidence_threshold_D_MARK) 
	    {  
                int xminn = xmin > 0 ? xmin : 0;
                int yminn = ymin > 0 ? ymin : 0;
                int width = xmax-xmin > 0 && xmax-xmin < img.cols ? xmax-xmin : 0;
                int height= ymax-ymin > 0 && ymax-ymin < img.rows ? ymax-ymin : 0;          
                cv::Rect rectClassIn(xminn,yminn,width,height);
                cv::rectangle(rotated, rectClassIn, Scalar(0,0,255), 3, 8, 0);
                                
                vector<Point2f> points,points_trans;
                points.push_back(Point2f(xminn,yminn));
                points.push_back(Point2f(xminn+width,yminn));
                points.push_back(Point2f(xminn,yminn+height));
                points.push_back(Point2f(xminn+width,yminn+height));

                cv::perspectiveTransform(points,points_trans, warpMatrix.inv());
                int xminn_trans = 100000;
                int yminn_trans = 100000;
                int xmaxn_trans = 0;
                int ymaxn_trans = 0;
                for(int i = 0 ;i < (int)points_trans.size(); i++)
                {
                    cout<<points_trans<<endl;
                    xminn_trans = points_trans[i].x < xminn_trans? points_trans[i].x : xminn_trans;
                    yminn_trans = points_trans[i].y < yminn_trans? points_trans[i].y : yminn_trans;
                    xmaxn_trans = points_trans[i].x > xmaxn_trans? points_trans[i].x : xmaxn_trans;
                    ymaxn_trans = points_trans[i].y > ymaxn_trans? points_trans[i].y : ymaxn_trans;
                }

                cv::rectangle(img, Point2f(xminn_trans,yminn_trans),Point2f(xmaxn_trans,ymaxn_trans), Scalar(0,0,255), 3, 8, 0);
                
                std::cout << file << " ";
                std::cout << label << " ";
                std::cout << score << " ";
                std::cout << xmin << " ";
                std::cout << ymin << " ";
                std::cout << xmax << " ";
                std::cout << ymax << std::endl;
	        
                string labels = std::to_string(label);
                string scores = std::to_string(score);
                string xminns = std::to_string(xminn);
                string yminns = std::to_string(yminn);
                string widths = std::to_string(width);
                string heights= std::to_string(height);
                string filevalue = name + " " + scores + " " + labels +" " + xminns + " " + yminns + " "+widths + " " + heights;
                OutPutData.saveResultFile(filevalue,name);
                //return 0;
	    }
              
        }//detection_mark result
        cv::Mat reMap;
	warpPerspective(rotated, reMap, warpMatrix.inv(), rotated.size(), INTER_LINEAR, BORDER_CONSTANT);
#endif 
#ifdef MARKDETECTION
          cv::Rect rect_left(0,0,1920,1200);
          cv::Rect rect_right(1920,0,1920,1200);
          cv::Mat image_result(img.rows,2*img.cols,CV_8UC3,Scalar::all(0));

          img.copyTo(image_result(rect_left));
          rotated.copyTo(image_result(rect_right));
          imshow("imgin",image_result);
          string nameout = pathout + name;
          imwrite(nameout,img);
          waitKey(1);
#else

#ifdef MARKLOAD
          string lane_path = "/home/sen/Downloads/LaneFile/";
          string lane_name = lane_path + name + ".txt";
          cout<<"lane_name: "<<lane_name<<endl;
          fstream file_lane;
          const char* lane_name_c =  lane_name.c_str();
          file_lane.open(lane_name_c ,ios::in);
          if(!file_lane)
          {
              cout<<"file lane_name open error!"<<endl;
              //continue;
          }
          else
          {
              cout<<"file lane_name open successful!"<<endl;
              
              string line_str;
              vector<string> fields;
              while(file_lane>>line_str)
              {
                  cout<<"line_str: "<<line_str<<endl;
                  split(fields,line_str,is_any_of(","));
                  cout<<fields[0]<<endl;
                  int xminn = stoi(fields[2]);
                  int yminn = stoi(fields[3]);
                  int width = stoi(fields[4]);
                  int height= stoi(fields[5]);
                  
                  vector<Point2f> points,points_trans;
                points.push_back(Point2f(xminn,yminn));
                points.push_back(Point2f(xminn+width,yminn));
                points.push_back(Point2f(xminn,yminn+height));
                points.push_back(Point2f(xminn+width,yminn+height));

                cv::perspectiveTransform(points,points_trans, warpMatrix.inv());
                int xminn_trans = 100000;
                int yminn_trans = 100000;
                int xmaxn_trans = 0;
                int ymaxn_trans = 0;
                for(int i = 0 ;i < (int)points_trans.size(); i++)
                {
                    cout<<points_trans<<endl;
                    xminn_trans = points_trans[i].x < xminn_trans? points_trans[i].x : xminn_trans;
                    yminn_trans = points_trans[i].y < yminn_trans? points_trans[i].y : yminn_trans;
                    xmaxn_trans = points_trans[i].x > xmaxn_trans? points_trans[i].x : xmaxn_trans;
                    ymaxn_trans = points_trans[i].y > ymaxn_trans? points_trans[i].y : ymaxn_trans;
                }

                cv::rectangle(img, Point2f(xminn_trans,yminn_trans),Point2f(xmaxn_trans,ymaxn_trans), Scalar(0,0,255), 3, 8, 0);
              }
              file_lane.close();
          }
#endif 
#ifdef  POLELOAD
          string pole_path = "/home/sen/Downloads/poles/";
          string pole_name = pole_path + name + ".txt";
          cout<<"pole_name: "<<pole_name<<endl;
          fstream file_pole;
          const char* pole_name_c =  pole_name.c_str();
          file_pole.open(pole_name_c ,ios::in);
          if(!file_pole)
          {
              cout<<"file lane_name open error!"<<endl;
              //continue;
          }
          else
          {
              cout<<"file lane_name open successful!"<<endl;
              
              string line_str;
              vector<string> fields;
              while(file_pole>>line_str)
              {
                  cout<<"line_str: "<<line_str<<endl;
                  split(fields,line_str,is_any_of(","));
                  cout<<fields[0]<<endl;
                  int xminn = stoi(fields[2]);
                  int yminn = stoi(fields[3]);
                  int width = stoi(fields[4]) - xminn;
                  int height= stoi(fields[5]) - yminn;
                  if(width > 200||height >600){break;}

                  cv::Rect rectClassIn(xminn,yminn,width,height);
                  cv::rectangle(img, rectClassIn, Scalar(0,255,100), 3, 12, 0);
              }
              file_pole.close();
          }
#endif

#ifdef  LANDLOAD
         vector<string> fields_name;
         split(fields_name,name,is_any_of("."));
         string imgName = fields_name[0] + "." + fields_name[1];
         cout<<"name landload: "<< imgName<<endl;

         DrawImage(img, imgName);
#endif
#ifndef  IMSHOW 
          imshow("imgin",img);       
          string nameout = pathout + name;
          imwrite(nameout,img);
          waitKey(600);
#endif 
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

