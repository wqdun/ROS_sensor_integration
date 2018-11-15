#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

#include "save_image_task.h"

SaveImageTask::SaveImageTask(double _header, const std::string &_cameraIP, size_t _frameNum, const cv::Mat &_image, unsigned char *_pData) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    time_ = _header;
    cameraIP_ = _cameraIP;
    frameNum_ =_frameNum;
    image_ = _image;
    pData_ = _pData;
}

SaveImageTask::~SaveImageTask() {
    DLOG(INFO) << __FUNCTION__ << " start.";
}

void SaveImageTask::doit() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string picFile("/tmp/" + std::to_string(time_) + "_" + cameraIP_ + "_" + std::to_string(frameNum_) + ".jpg");

    std::vector<int> jpgQuality{40};
    try {
        // cv::imwrite(picFile, image_, jpgQuality);
        // SaveImageUsingTurboJpeg(picFile);
        LOG(INFO) << "Save " << picFile;
    }
    catch(cv::Exception &ex) {
        LOG(ERROR) << ex.what();
    }
}

void SaveImageTask::SaveImageUsingTurboJpeg(const std::string &picFile) {
    // Output image format is JPEG; Compress the uncompressed image
    tjhandle tjInstance = tjInitCompress();
    if(NULL == tjInstance) {
        LOG(ERROR) << "initializing compressor.";
        exit(-1);
    }
    unsigned char *jpegBuf = NULL;
    unsigned long jpegSize = 0;
    const int outQual = 80;
    const int pixelFormat = TJPF_RGB;
    const int outSubsamp = TJSAMP_444;
    const int flags = 0;
    if(tjCompress2(tjInstance, pData_, 4096, 0, 2160, pixelFormat, &jpegBuf, &jpegSize, outSubsamp, outQual, flags) < 0) {
        LOG(ERROR) << "compressing image.";
        exit(-1);
    }
    if(pData_) {
        free(pData_);
        pData_ = NULL;
    }
    tjDestroy(tjInstance);
    tjInstance = NULL;

    FILE *jpegFile = fopen(picFile.c_str(), "wb");
    if(!jpegFile) {
        LOG(ERROR) << "Opening output file.";
        exit(-1);
    }
    if(fwrite(jpegBuf, jpegSize, 1, jpegFile) < 1) {
        LOG(ERROR) << "Writing output file.";
        exit(-1);
    }
    fclose(jpegFile);
    jpegFile = NULL;
    tjFree(jpegBuf);
    jpegBuf = NULL;
}

