
struct GeoLane2D
{
	int type;
	std::vector<cv::Point2f> lane;
	GeoLane2D() {};
};

map<string, vector<GeoLane2D>> mapLane;

void DrawImage(cv::Mat& img, string imgName)
{
        map<string,vector<GeoLane2D>>::iterator it; 
        it = mapLane.find(imgName);
        
        cout<<"mapLane.size: "<<mapLane.size()<<endl;
        for (int i = 0; i < (int)it->second.size(); i++)
        {
            for(int j = 0; j < (int)it->second[i].lane.size() - 1; j++)
            {
	        line(img, it->second[i].lane[j], it->second[i].lane[j + 1], Scalar(0, 0, 255),3,10);
            }
	}
	      
}

void LaneVerify()
{
	fstream file("/home/sen/projects/SmartCollector/landload/detLane.txt");
	char buf[2048] = { 0 };
	

	while (file.getline(buf,2048))
	{
		string strBuf = buf;
		vector<string> vecItem;
		boost::split(vecItem, strBuf, boost::is_any_of(";"));
		if(vecItem.empty())
			continue;
		string imgName = vecItem[0];
		GeoLane2D geoLane;
		for (int i = 1; i < (int)vecItem.size() - 2; i+=2) {
			float x = boost::lexical_cast<float>(vecItem[i]);
			float y = boost::lexical_cast<float>(vecItem[i + 1]);
			geoLane.lane.push_back(Point2f(x, y));
		}


		auto iter = mapLane.find(imgName);
		if (iter != mapLane.end()) {
			iter->second.push_back(geoLane);
		}
		else {
			vector<GeoLane2D> vecLane;
			vecLane.push_back(geoLane);
			mapLane.insert(std::pair<string, vector<GeoLane2D>>(imgName, vecLane));
		}


	}
       
        
	/*for (auto it : mapLane)
	{
		string imgName = it.first;
		imgName = "E:\\道路视频\\G7\\" + imgName;
		imgName += ".jpg";
		cv::Mat img = imread(imgName);
		if (img.empty())
			continue;

		for (auto iter : it.second) 
		{
			for (int i = 0; i < (int)iter.lane.size()-1; i++) {
				line(img, iter.lane[i], iter.lane[i + 1], Scalar(255, 0, 0));
			}
		}

		imshow("img", img);
		waitKey(1);

	}*/
}
