#include "mif_read.h"

MifReader::MifReader(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    private_nh.param("mif_path", mMifPath, mMifPath);
    LOG(INFO) << "Mif data is:" << mMifPath;
}

void MifReader::run() {
    vector<string> mifFiles;
    (void)getFiles(mMifPath, mifFiles);
    LOG(INFO) << "Got " << mifFiles.size() << " mif/mid files.";
    for(auto &mifFile: mifFiles) {
        readFile(mifFile);
    }

}

void MifReader::getFiles(const string &basePath, vector<string> &files) {
    DIR *dir;
    dirent *ptr;

    // if dir == NULL
    if( !(dir = opendir(basePath.c_str())) ) {
        LOG(ERROR) << "Failed to open " << basePath;
        exit(-1);
    }

    // while ptr != NULL
    while(ptr = readdir(dir)) {
        // ignore . and ..
        if( !(strcmp(ptr->d_name, ".")) ||
            !(strcmp(ptr->d_name, "..")) ) {
            continue;
        }

        // regular file
        if(8 == ptr->d_type) {
            // only deal ROAD_LANE_MARKING_GEO mif & mid
            const string fileName(ptr->d_name);
            if(string::npos != fileName.find("ROAD_LANE_MARKING_GEO")) {
                files.push_back(basePath + "/" + fileName);
            }
            // else do nothing
            // else {}
        }
        // directory
        else
        if(4 == ptr->d_type) {
            const string subDir(basePath + "/" + ptr->d_name);
            getFiles(subDir, files);
        }
        // else {
        //     // ignore links(10) & others
        // }
    }

    closedir(dir);
    return;
}

void MifReader::readFile(const string &file) {
    DLOG(INFO) << __FUNCTION__ << " start with " << file;



}


 // mifReader::getLonLat() {

 // }




