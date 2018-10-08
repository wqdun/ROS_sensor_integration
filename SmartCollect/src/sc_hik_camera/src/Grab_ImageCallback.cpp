#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "MvCameraControl.h"
#include "turbojpeg.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
// #define __USE_HIK_API_SAVING_JPG__

void *handle = NULL;

// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n" , "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        // 打印当前相机ip和用户自定义名字
        // print current ip and user defined name
        printf("%s %x\n" , "nCurrentIp:" , pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                   //当前IP
        printf("%s %s\n\n" , "chUserDefinedName:" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);     //用户定义名
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

MV_CC_PIXEL_CONVERT_PARAM stConvertParam_ = { 0 };
void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }
    LOG(INFO) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: " << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight;
    int nRet = MV_OK;

    // convert pixel format
    unsigned char *pDataForRGB = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    if(!pDataForRGB) {
        LOG(ERROR) << "Failed to allocate memory for pDataForRGB.";
        exit(-1);
    }
    stConvertParam_.nWidth = pFrameInfo->nWidth;
    stConvertParam_.nHeight = pFrameInfo->nHeight;
    stConvertParam_.pSrcData = pData;
    stConvertParam_.nSrcDataLen = pFrameInfo->nFrameLen;
    stConvertParam_.enSrcPixelType = pFrameInfo->enPixelType;
    stConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam_.pDstBuffer = pDataForRGB;
    stConvertParam_.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight *  4 + 2048;

    nRet = MV_CC_ConvertPixelType(handle, &stConvertParam_);
    if(MV_OK != nRet) {
        LOG(ERROR) << "MV_CC_ConvertPixelType fail! nRet: " << nRet;
        return;
    }
    LOG(INFO) << "ConvertPixelType " << pFrameInfo->enPixelType << " --> " << PixelType_Gvsp_RGB8_Packed;

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
    if(tjCompress2(tjInstance, pDataForRGB, pFrameInfo->nWidth, 0, pFrameInfo->nHeight, pixelFormat, &jpegBuf, &jpegSize, outSubsamp, outQual, flags) < 0) {
        LOG(ERROR) << "compressing image.";
        exit(-1);
    }
    if(pDataForRGB) {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }
    tjDestroy(tjInstance);
    tjInstance = NULL;

    std::string imageName(std::to_string(pFrameInfo->nFrameNum) + ".jpg");
    FILE *jpegFile = fopen(imageName.c_str(), "wb");
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

#ifdef __USE_HIK_API_SAVING_JPG__
    unsigned char *pDataForSaveImage = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    if (!pDataForSaveImage) {
        LOG(ERROR) << "Failed to allocate memory for pDataForSaveImage.";
        exit(-1);
    }
    MV_SAVE_IMAGE_PARAM_EX stSaveParam;
    memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));

    stSaveParam.enImageType = MV_Image_Jpeg;
    stSaveParam.enPixelType = pFrameInfo->enPixelType;
    stSaveParam.nBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048;
    stSaveParam.nWidth      = pFrameInfo->nWidth;
    stSaveParam.nHeight     = pFrameInfo->nHeight;
    stSaveParam.pData       = pData;
    stSaveParam.nDataLen    = pFrameInfo->nFrameLen;
    stSaveParam.pImageBuffer = pDataForSaveImage;
    stSaveParam.nJpgQuality = 80;

    LOG(INFO) << stSaveParam.nDataLen;
    nRet = MV_CC_SaveImageEx(&stSaveParam);
    if(MV_OK != nRet) {
        LOG(ERROR) << "Failed in MV_CC_SaveImage, nRet: " << nRet;
        exit(-1);
    }

    imageName = std::to_string(pFrameInfo->nFrameNum) + "_hik.jpg";
    jpegFile = fopen(imageName.c_str(), "wb");
    if(!jpegFile) {
        LOG(ERROR) << "Opening output file.";
        exit(-1);
    }
    fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, jpegFile);
    free(pDataForSaveImage);
    pDataForSaveImage = NULL;
    fclose(jpegFile);
#endif

    LOG(INFO) << "Save " << imageName;
    return;
}

int main()
{
    int nRet = MV_OK;

    // void* handle = NULL;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return -1;
    }
    unsigned int nIndex = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else
    {
        printf("Find No Devices!\n");
        return -1;
    }

    scanf("%d", &nIndex);

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return -1;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return -1;
    }

    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return -1;
    }

    // 注册抓图回调
    // register image callback
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
        return -1;
    }

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }

    PressEnterToExit();

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        return -1;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        return -1;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return -1;
    }

    printf("exit\n");

    return 0;
}
