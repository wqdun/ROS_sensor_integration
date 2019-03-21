#include "test.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

using namespace std;

Test::Test() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string pszDriverName("ESRI shapefile");
    GDALAllRegister();
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName.c_str() );
    if(!poDriver) {
        printf( "%s driver not available.\n", pszDriverName.c_str() );
        exit(1);
    }

    const std::string shpFile("point_out.shp");
    poDS_ = poDriver->Create(shpFile.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    if(!poDS_) {
        printf("Creation of output file failed.\n" );
        exit(2);
    }

    poLayer_ = poDS_->CreateLayer("point_out", NULL, wkbLineString, NULL);
    if(!poLayer_) {
        printf( "Layer creation failed.\n" );
        exit(3);
    }

    OGRFieldDefn oField("Name", OFTString);
    oField.SetWidth(32);
    if(poLayer_->CreateField(&oField) != OGRERR_NONE) {
        printf( "Creating Name field failed.\n" );
        exit(4);
    }
}

Test::~Test() {
    LOG(INFO) << __FUNCTION__ << " start.";
    GDALClose(poDS_);
}

void Test::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    double x = 116;
    double y = 40;
    OGRFeature *poFeature = OGRFeature::CreateFeature(poLayer_->GetLayerDefn() );
    const std::string szName("hello");
    poFeature->SetField("Name", szName.c_str() );

    for(int i = 0; i < 3; ++i) {
        OGRLineString line;
        for(int i = 0; i < 3; ++i) {
            x += 0.0001;
            y -= 0.0001;
            // sleep(5);
            printf("Add one.\n");
            line.addPoint(x, y);
        }

        poFeature->SetGeometry(&line);

        cout << poFeature << "\n";
        if(poLayer_->CreateFeature(poFeature) != OGRERR_NONE) {
            printf("Failed to create feature in shapefile.\n");
            exit(5);
        }
        printf("Add one line.\n");
    }

    OGRFeature::DestroyFeature(poFeature);
}

