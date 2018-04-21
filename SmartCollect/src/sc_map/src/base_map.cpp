#include "base_map.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

BaseMap::BaseMap(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    LOG(INFO) << __FUNCTION__ << " start.";

    pubBaseMap_ = nh.advertise<sc_msgs::Lines2D>("sc_base_map", 0);
    pubPlanMap_ = nh.advertise<sc_msgs::Lines2D>("sc_plan_map", 0);

    const std::string exePath(public_tools::PublicTools::safeReadlink("/proc/self/exe") );
    LOG(INFO) << "Node path: " << exePath;
    const std::string smartcPath(exePath.substr(0, exePath.find("/devel/") ) );
    LOG(INFO) << "Get SmartCollector path: " << smartcPath;

    const std::string baseMapPath(smartcPath + "/data/BaseMap/");
    std::vector<std::string> baseMapFiles;
    (void)public_tools::PublicTools::getFilesInDir(baseMapPath, ".kml", baseMapFiles);
    if(1 != baseMapFiles.size() ) {
        LOG(WARNING) << "Got " << baseMapFiles.size() << " baseMapFiles in " << baseMapPath << ", should be 1.";
    }
    else {
        (void)getLines(baseMapFiles[0], baseMapLines_);
    }

    const std::string planMapPath(smartcPath + "/data/PlanLayer/");
    std::vector<std::string> planMapFiles;
    (void)public_tools::PublicTools::getFilesInDir(planMapPath, ".kml", planMapFiles);
    if(1 != planMapFiles.size() ) {
        LOG(WARNING) << "Got " << planMapFiles.size() << " planMapFiles in " << planMapPath << ", should be 1.";
    }
    else {
        (void)getLines(planMapFiles[0], planMapLines_);
    }
}

void BaseMap::getLines(const std::string &_shpFile, sc_msgs::Lines2D &_lines) {
    LOG(INFO) << __FUNCTION__ << " start, reading " << _shpFile;

    GDALAllRegister();
    GDALDataset *poDS = (GDALDataset*)GDALOpenEx(_shpFile.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(!poDS) {
        LOG(ERROR) << "Failed to open " << _shpFile;
        exit(1);
    }

    const int layerNum = poDS->GetLayerCount();
    if(1 != layerNum) {
        LOG(ERROR) << "Got " << layerNum << " layer in " << _shpFile << "; should be 1.";
        GDALClose(poDS);
        exit(1);
    }
    OGRLayer *poLayer = poDS->GetLayer(0);
    if(!poLayer) {
        LOG(ERROR) << "poLayer is NULL.";
        GDALClose(poDS);
        exit(1);
    }
    LOG(INFO) << "Layer name: " << poLayer->GetName();

    GIntBig featureCnt = 0;
    OGRFeature *poFeature;
    poLayer->ResetReading();
    _lines.lines2D.clear();
    sc_msgs::Line2D baseMapLine;
    sc_msgs::Point2D lngLat;
    while(NULL != (poFeature = poLayer->GetNextFeature() ) ) {
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if(!poGeometry) {
            LOG(ERROR) << "poGeometry is NULL.";
            OGRFeature::DestroyFeature(poFeature);
            continue;
        }

        if(wkbLineString != wkbFlatten(poGeometry->getGeometryType() ) ) {
            LOG(ERROR) << "GeometryType should be wkbLineString(2), but it's " << poGeometry->getGeometryType();
            OGRFeature::DestroyFeature(poFeature);
            continue;
        }

        OGRSimpleCurve *poSimpleCurve = (OGRSimpleCurve *)poGeometry;
        LOG(INFO) << "Feature[" << featureCnt << "] dimension: " << poSimpleCurve->getDimension() << ", contains " << poSimpleCurve->getNumPoints() << " points.";

        baseMapLine.line2D.clear();
        for(int j = 0; j < poSimpleCurve->getNumPoints(); ++j) {
            lngLat.x = poSimpleCurve->getX(j);
            lngLat.y = poSimpleCurve->getY(j);
            DLOG(INFO) << std::fixed << "longitude: " << lngLat.x << "; latitude: " << lngLat.y;
            baseMapLine.line2D.push_back(lngLat);
        }
        _lines.lines2D.push_back(baseMapLine);
        ++featureCnt;
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(poDS);

    LOG(INFO) << "Got " << featureCnt << " lines; _lines.lines2D.size(): " << _lines.lines2D.size();
}

BaseMap::~BaseMap() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye.";
}

void BaseMap::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    ros::Rate rate(0.5);
    while(ros::ok() ) {
        ros::spinOnce();
        rate.sleep();

        pubBaseMap_.publish(baseMapLines_);
        pubPlanMap_.publish(planMapLines_);
    }

    LOG(INFO) << __FUNCTION__ << " end.";
}
