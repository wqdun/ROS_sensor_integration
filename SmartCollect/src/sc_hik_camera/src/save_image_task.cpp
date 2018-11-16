#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

#include "save_image_task.h"

SaveImageTask::SaveImageTask(SingleCamera *_pSingleCamera, MV_FRAME_OUT_INFO_EX _frameInfo, double _header, unsigned char *_pData) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    time_ = _header;
    pData_ = _pData;
    pSingleCamera_ = _pSingleCamera;
    frameInfo_ = _frameInfo;
}

SaveImageTask::~SaveImageTask() {
    DLOG(INFO) << __FUNCTION__ << " start.";
}

void SaveImageTask::doit() {
    LOG(INFO) << __FUNCTION__ << " start.";

    const std::string picFile("/tmp/" + std::to_string(time_) + "_" + pSingleCamera_->GetCameraIP() + "_" + std::to_string(frameInfo_.nFrameNum) + ".jpg");

    int err = MV_OK;
    const std::string cameraIP(pSingleCamera_->GetCameraIP());
    void *handle = pSingleCamera_->GetHandle();
    LOG(INFO) << __FUNCTION__ << " start.";
    unsigned char *pDataForRGB = (unsigned char*)malloc(frameInfo_.nWidth * frameInfo_.nHeight * 4 + 2048);
    assert(pDataForRGB);
    MV_CC_PIXEL_CONVERT_PARAM convertParam = {0};
    convertParam.nWidth = frameInfo_.nWidth;
    convertParam.nHeight = frameInfo_.nHeight;
    convertParam.pSrcData = pData_;
    convertParam.nSrcDataLen = frameInfo_.nFrameLen;
    convertParam.enSrcPixelType = frameInfo_.enPixelType;
    convertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convertParam.pDstBuffer = pDataForRGB;
    convertParam.nDstBufferSize = frameInfo_.nWidth * frameInfo_.nHeight * 4 + 2048;
    err = MV_CC_ConvertPixelType(handle, &convertParam);
    if(pData_) {
        free(pData_);
        pData_ = NULL;
    }
    if(MV_OK != err) {
        LOG(ERROR) << "Failed to MV_CC_ConvertPixelType; err: " << err;
        return;
    }

    cv::Mat matBGR(frameInfo_.nHeight, frameInfo_.nWidth, CV_8UC3);
    memcpy(matBGR.data, pDataForRGB, frameInfo_.nWidth * frameInfo_.nHeight * 3);
    if(pDataForRGB) {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }

    cv::imwrite(picFile, matBGR);
    LOG(INFO) << "Save " << picFile;


    LOG(INFO) << __FUNCTION__ << " end.";

}

