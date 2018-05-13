#include "base_map.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

BaseMap::BaseMap(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    LOG(INFO) << __FUNCTION__ << " start.";

    pTracker_.reset(new Track() );
    isRecord_ = false;
    subMonitor_ = nh.subscribe("sc_monitor", 0, &BaseMap::monitorCB, this);

    pubBaseMap_ = nh.advertise<sc_msgs::Lines2D>("sc_base_map", 0);
    pubPlanMap_ = nh.advertise<sc_msgs::Lines2D>("sc_plan_map", 0);

    pubUnrecordedTrack_ = nh.advertise<sc_msgs::Lines2D>("sc_unrecorded_track", 0);
    pubRecordedTrack_ = nh.advertise<sc_msgs::Lines2D>("sc_recorded_track", 0);

    const std::string exePath(public_tools::PublicTools::safeReadlink("/proc/self/exe") );
    LOG(INFO) << "Node path: " << exePath;
    const std::string smartcPath(exePath.substr(0, exePath.find("/devel/") ) );
    LOG(INFO) << "Get SmartCollector path: " << smartcPath;

    const std::string baseMapPath(smartcPath + "/data/BaseMap/");
    std::vector<std::string> baseMapFiles;
    (void)public_tools::PublicTools::getFilesInDir(baseMapPath, ".shp", baseMapFiles);
    if(1 != baseMapFiles.size() ) {
        LOG(WARNING) << "Got " << baseMapFiles.size() << " baseMapFiles in " << baseMapPath << ", should be 1.";
    }
    else {
        (void)getLines(baseMapFiles[0], baseMapLines_);
    }

    const std::string planMapPath(smartcPath + "/data/PlanLayer/");
    std::vector<std::string> planMapFiles;
    (void)public_tools::PublicTools::getFilesInDir(planMapPath, ".shp", planMapFiles);
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

void BaseMap::monitorCB(const sc_msgs::MonitorMsg::ConstPtr& pMonitorMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, is_record: " << (int)(pMonitorMsg->is_record) << "; lat: " << pMonitorMsg->lat_lon_hei.x;

    isRecord_ = pMonitorMsg->is_record;

    if(!public_tools::PublicTools::isInChina(pMonitorMsg->lat_lon_hei.x, pMonitorMsg->lat_lon_hei.y) ) {
        LOG_EVERY_N(INFO, 100) << "I am not china.";
        gpsLonLat_.x = pMonitorMsg->lat_lon_hei.y;
        gpsLonLat_.y = pMonitorMsg->lat_lon_hei.x;
        return;
    }
    LOG_EVERY_N(INFO, 100) << "I am china, gonna translate coordination (" << pMonitorMsg->lat_lon_hei.y << ", " << pMonitorMsg->lat_lon_hei.x << ") to Mars.";
    if(0 != coordtrans("wgs84", "gcj02", pMonitorMsg->lat_lon_hei.y, pMonitorMsg->lat_lon_hei.x, gpsLonLat_.x, gpsLonLat_.y) ) {
        LOG(ERROR) << "Failed translate coordination (" << pMonitorMsg->lat_lon_hei.y << ", " << pMonitorMsg->lat_lon_hei.x << ") to Mars.";
        exit(1);
    }
}

void BaseMap::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    ros::Rate rate(0.5);
    while(ros::ok() ) {
        ros::spinOnce();
        rate.sleep();

        pubBaseMap_.publish(baseMapLines_);
        pubPlanMap_.publish(planMapLines_);

        pTracker_->run(isRecord_, gpsLonLat_);
        pubUnrecordedTrack_.publish(pTracker_->unrecordedLines_);
        pubRecordedTrack_.publish(pTracker_->recordedLines_);
    }

    LOG(INFO) << __FUNCTION__ << " end.";
}
