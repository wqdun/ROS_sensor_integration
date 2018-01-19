#include <iostream>
#include <sstream>
#include <cmath>
#include <bitset>
#include <string>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <pthread.h>
#include <unistd.h>
// GDAL library
#include <gdal.h>
#include "gdal_alg.h"
#include "cpl_conv.h"
#include "cpl_port.h"
#include "cpl_multiproc.h"
#include "ogr_srs_api.h"
// #include "proj_api.h"
#include "ogrsf_frmts.h"

using namespace std;

int main(int argc, char **argv) {
    GDALAllRegister();
    GDALDataset *poDS;
    const string fileName("PlanLayer/2501-1-104-180108.TAB");
    poDS = (GDALDataset*)GDALOpenEx(fileName.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(!poDS) {
        cout << "Open failed.\n";
        exit(1);
    }

    cout << poDS->GetLayerCount() << endl;

    for(int i = 0; i < poDS->GetLayerCount(); ++i) {
        OGRLayer *poLayer;
        poLayer = poDS->GetLayer(i);
        cout << poLayer->GetName() << endl;
    }

    OGRLayer *poLayer;
    poLayer = poDS->GetLayer(0);
    cout << poLayer->GetName() << endl;

    // poLayer = poDS->GetLayerByName("25011104180108175655-laser");
    if(!poLayer) {
        cout << "Got none." << endl;
        exit(1);
    }
    cout << "Gotcha you." << endl;

    poLayer->ResetReading();
    GIntBig featureCnt = poLayer->GetFeatureCount();
    cout << featureCnt << endl;

    for(GIntBig i = 0; i < featureCnt; ++i) {
        OGRFeature *poFeature = poLayer->GetFeature(i);
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if(!poGeometry) {
            cout << "poGeometry is NULL." << endl;
            OGRFeature::DestroyFeature(poFeature);
            continue;
        }

        if(wkbLineString != wkbFlatten(poGeometry->getGeometryType() ) ) {
            cout << "poGeometry->getGeometryType(): " << poGeometry->getGeometryType() << endl;
            continue;
        }
        OGRSimpleCurve *poSimpleCurve = (OGRSimpleCurve *)poGeometry;
        cout << "poSimpleCurve->getNumPoints(): " << poSimpleCurve->getNumPoints() << endl;
        // cout << "poSimpleCurve->getDimension(): " << poSimpleCurve->getDimension() << endl;
        for(int i = 0; i < poSimpleCurve->getNumPoints(); ++i) {
            cout << fixed << i << ": " << poSimpleCurve->getX(i) << ": " << poSimpleCurve->getY(i) << ": " << poSimpleCurve->getZ(i) << endl;


    }



    exit(1);

    while( (poFeature = poLayer->GetNextFeature() ) ) {
        ++featureCnt;
        cout << "featureCnt: " << featureCnt << endl;
        // cout << poFeature->GetNameRef() << endl;
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if(!poGeometry) {
            cout << "poGeometry is NULL." << endl;
        }
        cout << poFeature->GetDefnRef()->GetFieldCount() << endl;

        for(int i = 0; i < poFeature->GetDefnRef()->GetFieldCount(); ++i) {
            cout << poFeature->GetDefnRef()->GetFieldDefn(i)->GetNameRef() << endl;
        }

        // cout << poFeature->GetGeomFieldCount() << endl;
        // cout << "poGeometry->getGeometryType(): " << poGeometry->getGeometryType() << endl;

        if(poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType() ) == wkbLineString) {
            OGRSimpleCurve *poPoint = (OGRSimpleCurve *) poGeometry;
            cout << "poPoint->getNumPoints(): " << poPoint->getNumPoints() << endl;
            cout << "poPoint->getDimension(): " << poPoint->getDimension() << endl;
            for(int i = 0; i < poPoint->getNumPoints(); ++i) {
                cout << fixed << i << ": " << poPoint->getX(i) << ": " << poPoint->getY(i) << ": " << poPoint->getZ(i) << endl;
            }

            // printf( "%.3f,%3.f\n", poPoint->getX(), poPoint->getY() );
        }
        else {
            printf("no point geometry\n");
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose( poDS );
    cout << "I got " << featureCnt << " features.\n";

    exit(1);

    const char *pszDriverName = "ESRI Shapefile";

    GDALDriver *poDriver;
    GDALAllRegister();
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    if( poDriver == NULL ) {
        printf( "%s driver not available.\n", pszDriverName );
        exit( 1 );
    }

    // GDALDataset *poDS;
    poDS = poDriver->Create("point_out.shp", 0, 0, 0, GDT_Unknown, NULL );
    if( poDS == NULL ) {
        printf( "Creation of output file failed.\n" );
        exit( 1 );
    }

    // OGRLayer *poLayer;
    poLayer = poDS->CreateLayer("point_out", NULL, wkbPoint, NULL );
    if( poLayer == NULL ) {
        printf( "Layer creation failed.\n" );
        exit( 1 );
    }

    OGRFieldDefn oField("Name", OFTString );
    oField.SetWidth(32);
    if( poLayer->CreateField( &oField) != OGRERR_NONE ) {
        printf( "Creating Name field failed.\n" );
        exit( 1 );
    }

    double x, y;
    char szName[33];
    while( !feof(stdin) && fscanf( stdin, "%lf,%lf,%32s", &x, &y, szName ) == 3 ) {
        OGRFeature *poFeature;
        poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
        poFeature->SetField("Name", szName );

        OGRPoint pt;
        pt.setX( x );
        pt.setY( y );
        cout << x << y << szName << endl;
        poFeature->SetGeometry( &pt );

        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE ) {
            printf( "Failed to create feature in shapefile.\n" );
            exit( 1 );
        }
        OGRFeature::DestroyFeature( poFeature );
    }



    GDALClose( poDS );


    cout << "Bye..." << endl;
}




// OGRGeometry *poGeometry;


// int nGeomFieldCount;


// nGeomFieldCount = poFeature->GetGeomFieldCount();

// for(int iGeomField = 0; iGeomField < nGeomFieldCount; iGeomField++ ) {

// poGeometry = poFeature->GetGeomFieldRef(iGeomField);

// if( poGeometry != NULL

// &&
// wkbFlatten(poGeometry->getGeometryType()) ==

// wkbPoint )

// {

// OGRPoint *poPoint = (OGRPoint
//  *) poGeometry;


// printf( "%.3f,%3.f\n", poPoint->getX(),
//  poPoint->getY() );

// }

// else

// {

// printf( "no point geometry\n" );

// }





        // OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
        // for(int iField = 0; iField < poFDefn->GetFieldCount(); ++iField) {
        //     OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn(iField);
        //     if(poFieldDefn->GetType() == OFTInteger) {
        //         printf("%d,", poFeature->GetFieldAsInteger(iField) );
        //     }
        //     else
        //     if(poFieldDefn->GetType() == OFTInteger64) {
        //         printf(CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64(iField) );
        //     }
        //     else
        //     if(poFieldDefn->GetType() == OFTReal) {
        //         printf("%.3f,", poFeature->GetFieldAsDouble(iField) );
        //     }
        //     else
        //     if(poFieldDefn->GetType() == OFTString) {
        //         printf("%s,", poFeature->GetFieldAsString(iField) );
        //     }
        //     else {
        //         printf("%s,", poFeature->GetFieldAsString(iField) );
        //     }
        // }



#if 0 // using getfeatureCount may fail it returns: !!
E0116 15:02:47.204110  8068 display_plan_layer.cpp:53] Feature[0]: poFeature is NULL.
E0116 15:02:47.207525  8068 display_plan_layer.cpp:53] Feature[35]: poFeature is NULL.
E0116 15:02:47.207577  8068 display_plan_layer.cpp:53] Feature[37]: poFeature is NULL.
E0116 15:02:47.231138  8068 display_plan_layer.cpp:53] Feature[284]: poFeature is NULL.
E0116 15:02:47.238708  8068 display_plan_layer.cpp:53] Feature[400]: poFeature is NULL.
E0116 15:02:47.238728  8068 display_plan_layer.cpp:53] Feature[401]: poFeature is NULL.
E0116 15:02:47.238963  8068 display_plan_layer.cpp:53] Feature[404]: poFeature is NULL.
E0116 15:02:47.238982  8068 display_plan_layer.cpp:53] Feature[405]: poFeature is NULL.
E0116 15:02:47.238993  8068 display_plan_layer.cpp:53] Feature[406]: poFeature is NULL.
E0116 15:02:47.239003  8068 display_plan_layer.cpp:53] Feature[407]: poFeature is NULL.
E0116 15:02:47.239014  8068 display_plan_layer.cpp:53] Feature[408]: poFeature is NULL.
E0116 15:02:47.239035  8068 display_plan_layer.cpp:53] Feature[409]: poFeature is NULL.


void PlanLayerDisplayer::getPlanLines(const std::string &planLayerFile) {
    LOG(INFO) << __FUNCTION__ << " start, reading " << planLayerFile;
    // open file and get elements
    GDALAllRegister();
    GDALDataset *poDS = (GDALDataset*)GDALOpenEx(planLayerFile.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(!poDS) {
        LOG(ERROR) << "Failed to open " << planLayerFile;
        exit(1);
    }

    const int layerNum = poDS->GetLayerCount();
    if(1 != layerNum) {
        LOG(ERROR) << "Got " << layerNum << " layer in " << planLayerFile << "; should be 1.";
        exit(1);
    }
    OGRLayer *poLayer = poDS->GetLayer(0);
    if(!poLayer) {
        LOG(ERROR) << "poLayer is NULL.";
        exit(1);
    }
    LOG(INFO) << "Layer name: " << poLayer->GetName();

    const GIntBig featureNum = poLayer->GetFeatureCount();
    planLayerLines_.clear();
    planLayerLines_.reserve(featureNum);
    planLayerLines2Show_.markers.reserve(featureNum);
    public_tools::geoPoints_t planLayerLine;

    for(GIntBig i = 1; i < featureNum + 100; ++i) {
        OGRFeature *poFeature = poLayer->GetFeature(i);
        if(!poFeature) {
            LOG(ERROR) << "Feature[" << i << "]: poFeature is NULL.";
            continue;
        }
        OGRGeometry *poGeometry = poFeature->GetGeometryRef();
        if(!poGeometry) {
            LOG(ERROR) << "poGeometry is NULL.";
            OGRFeature::DestroyFeature(poFeature);
            continue;
        }

        if(wkbLineString != wkbFlatten(poGeometry->getGeometryType() ) ) {
            LOG(ERROR) << "GeometryType should be wkbLineString(2), but it's " << poGeometry->getGeometryType();
            continue;
        }

        OGRSimpleCurve *poSimpleCurve = (OGRSimpleCurve *)poGeometry;
        DLOG(INFO) << "Feature[" << i << "] dimension: " << poSimpleCurve->getDimension() << ", contains " << poSimpleCurve->getNumPoints() << " points.";

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

        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(poDS);

}
#endif
