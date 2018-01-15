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
    poDS = (GDALDataset*)GDALOpenEx("PlanLayer/2501-1-104-180108.TAB", GDAL_OF_VECTOR, NULL, NULL, NULL);
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

    OGRFeature *poFeature;
    poLayer->ResetReading();
    int featureCnt = 0;
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
