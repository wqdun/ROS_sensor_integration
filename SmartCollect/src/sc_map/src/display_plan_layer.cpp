#include "display_plan_layer.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

PlanLayerDisplayer::PlanLayerDisplayer(const std::string &_planLayerPath) {
    planLayerScaleRatio_ = 0.01;
    pubPlanLayer_ = nh.advertise<visualization_msgs::MarkerArray>("show_plan_layer", 0);

    std::vector<std::string> planLayerFiles;
    planLayerFiles.clear();
    (void)public_tools::PublicTools::getFilesInDir(_planLayerPath, ".TAB", planLayerFiles);
    if(1 == planLayerFiles.size() ) {
        const std::string _planLayerFile(planLayerFiles[0]);
        // fill planLayerLines_
        (void)getPlanLines(_planLayerFile);
    }
    else {
        LOG(WARNING) << "Got " << planLayerFiles.size() << " planLayerFiles, should be 1.";
    }
}

void PlanLayerDisplayer::getPlanLines(const std::string &planLayerFile) {
    LOG(INFO) << __FUNCTION__ << " start, reading " << planLayerFile;

    GDALAllRegister();
    GDALDataset *poDS = (GDALDataset*)GDALOpenEx(planLayerFile.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(!poDS) {
        LOG(ERROR) << "Failed to open " << planLayerFile;
        exit(1);
    }

    const int layerNum = poDS->GetLayerCount();
    if(1 != layerNum) {
        LOG(ERROR) << "Got " << layerNum << " layer in " << planLayerFile << "; should be 1.";
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
    planLayerLines_.clear();
    public_tools::geoPoints_t planLayerLine;

    OGRFeature *poFeature;
    poLayer->ResetReading();
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

        planLayerLine.clear();
        geometry_msgs::Point gaussXY;
        for(int j = 0; j < poSimpleCurve->getNumPoints(); ++j) {
            const double lng = poSimpleCurve->getX(j);
            const double lat = poSimpleCurve->getY(j);
            double gaussX;
            double gaussY;
            (void)public_tools::PublicTools::GeoToGauss(lng * 3600, lat * 3600, 39, 3, &gaussY, &gaussX, 117);
            DLOG(INFO) << std::fixed << "longitude: " << poSimpleCurve->getX(j) << "; latitude: " << poSimpleCurve->getY(j);
            gaussXY.x = gaussY;
            gaussXY.y = gaussX;
            gaussXY.z = 0;
            planLayerLine.push_back(gaussXY);
        }
        planLayerLines_.push_back(planLayerLine);
        ++featureCnt;
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(poDS);
    LOG(INFO) << "Got " << featureCnt << " lines; planLayerLines_.size(): " << planLayerLines_.size();
    planLayerLines2Show_.markers.reserve(featureCnt);
}

void PlanLayerDisplayer::initMarker(visualization_msgs::Marker &marker, const size_t id) {
    marker.points.clear();
    marker.header.frame_id = "/velodyne";
    marker.ns = "plan_layer";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width; POINTS markers use x and y scale for width/height respectively
    marker.scale.x = 1.5 * planLayerScaleRatio_;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    // set .a = 0 to hide display
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(1.0);
}


void PlanLayerDisplayer::displayPlanLayer(const public_tools::geoPoint_t &encryptedGauss) {
    LOG_EVERY_N(INFO, 50) << __FUNCTION__ << " start, to show planLayerLines_.size(): " << planLayerLines_.size();

    planLayerLines2Show_.markers.clear();
    visualization_msgs::Marker offsetLine;
    for(size_t markerId = 0; markerId < planLayerLines_.size(); ++markerId) {
        (void)initMarker(offsetLine, markerId);
        (void)public_tools::PublicTools::transform_coordinate(planLayerLines_[markerId], encryptedGauss, offsetLine.points, planLayerScaleRatio_);
        planLayerLines2Show_.markers.push_back(offsetLine);
    }

    pubPlanLayer_.publish(planLayerLines2Show_);
}
