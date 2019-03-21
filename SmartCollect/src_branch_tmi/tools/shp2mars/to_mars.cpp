#include "to_mars.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

ToMars::ToMars() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

ToMars::~ToMars() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

void ToMars::getLines(const std::string &_shpFile) {
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

    lines_.clear();
    std::vector<Point2D> line;

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

        line.clear();
        for(int j = 0; j < poSimpleCurve->getNumPoints(); ++j) {
            Point2D lngLat(poSimpleCurve->getX(j), poSimpleCurve->getY(j) );
            DLOG(INFO) << std::fixed << "longitude: " << lngLat.x << "; latitude: " << lngLat.y;
            line.push_back(lngLat);
        }
        lines_.push_back(line);
        ++featureCnt;
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(poDS);

    LOG(INFO) << "Got " << featureCnt << " lines; _lines.size(): " << lines_.size();
}


void ToMars::writeShp() {
    LOG(INFO) << __FUNCTION__ << " start, reading " << _shpFile;




}


