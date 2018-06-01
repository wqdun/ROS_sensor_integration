#include "ogrsf_frmts.h"

int main() {
    const std::string pszDriverName("ESRI shapefile");

    GDALAllRegister();
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName.c_str() );
    if(!poDriver) {
        printf( "%s driver not available.\n", pszDriverName.c_str() );
        exit( 1 );
    }


    const std::string shpFile("point_out.shp");
    GDALDataset *poDS = poDriver->Create(shpFile.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    if(!poDS) {
        printf("Creation of output file failed.\n" );
        exit(2);
    }

    OGRLayer *poLayer = poDS->CreateLayer("point_out", NULL, wkbPoint, NULL);
    if(!poLayer) {
        printf( "Layer creation failed.\n" );
        exit(3);
    }

    OGRFieldDefn oField("Name", OFTString);
    oField.SetWidth(32);
    if(poLayer->CreateField(&oField) != OGRERR_NONE) {
        printf( "Creating Name field failed.\n" );
        exit(4);
    }

    double x = 116;
    double y = 40;
    const std::string szName("hello");
    for(int i = 0; i < 3; ++i) {
        OGRFeature *poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn() );
        OGRPoint pt;
        x += 0.0001;
        y -= 0.0001;
        pt.setX(x);
        pt.setY(y);
        poFeature->SetGeometry(&pt);
        poFeature->SetField("Name", szName.c_str() );

        if(poLayer->CreateFeature(poFeature) != OGRERR_NONE) {
            printf( "Failed to create feature in shapefile.\n" );
            exit(5);
        }
        OGRFeature::DestroyFeature(poFeature);
    }

    GDALClose(poDS);
    return 0;
}
