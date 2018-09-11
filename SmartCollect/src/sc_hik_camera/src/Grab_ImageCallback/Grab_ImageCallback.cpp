#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

#include "MvCameraControl.h"
#include "libjpeg-turbo/turbojpeg.h"

void* handle = NULL;


// 等待用户输入enter键来结束取流或结束程序
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

void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    timespec time_sys_end, time_sys_start;
    clock_gettime(CLOCK_REALTIME, &time_sys_start);
    LOG(INFO);
    int nRet = MV_OK;
    static int i = 0;
    if(!pFrameInfo) {
        printf("pFrameInfo is NULL\n");
        return;
    }
    printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);


    unsigned char *pDataForSaveImage = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    if (NULL == pDataForSaveImage)
    {
        exit;
    }
    LOG(INFO);
    MV_SAVE_IMAGE_PARAM_EX stSaveParam;
    LOG(INFO);
    memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
    // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
    // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
    // Top to bottom are：
    stSaveParam.enImageType = MV_Image_Jpeg;
    stSaveParam.enPixelType = pFrameInfo->enPixelType;
    LOG(INFO) << "pFrameInfo->enPixelType: " << pFrameInfo->enPixelType;
    stSaveParam.nBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048;
    stSaveParam.nWidth      = pFrameInfo->nWidth;
    stSaveParam.nHeight     = pFrameInfo->nHeight;
    stSaveParam.pData       = pData;
    LOG(INFO) << (void *)pData;
    LOG(INFO) << pFrameInfo;
    LOG(INFO) << (unsigned char *)pData - (unsigned char *)pFrameInfo;
    stSaveParam.nDataLen    = pFrameInfo->nFrameLen;
    LOG(INFO) << stSaveParam.nDataLen;

    LOG(INFO);

    unsigned char *pDataForRGB = (unsigned char*)malloc(pFrameInfo->nWidth * pFrameInfo->nHeight * 4 + 2048);
    if (NULL == pDataForRGB)
    {
        exit(-1);
    }
        LOG(INFO);

    // 像素格式转换
    // convert pixel format
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
    // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
    // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
    // destination pixel format, output data buffer, provided output buffer size
    stConvertParam.nWidth = pFrameInfo->nWidth;
    stConvertParam.nHeight = pFrameInfo->nHeight;
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = pFrameInfo->nFrameLen;
    stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;
    LOG(INFO) << stConvertParam.enSrcPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer = pDataForRGB;
    stConvertParam.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight *  4 + 2048;
    if(NULL == handle) {
        std::cout << "NULL handler...\n";
        exit(-1);
    }
        LOG(INFO);

    nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
    if (MV_OK != nRet)
    {
        printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
        return;
    }

    // FILE* fp = fopen("AfterConvert_RGB.raw", "wb");
    // if (NULL == fp)
    // {
    //     printf("fopen failed\n");
    //     exit(-1);
    // }
    // fwrite(pDataForRGB, 1, stConvertParam.nDstLen, fp);
    // fclose(fp);
    printf("convert succeed\n");
    LOG(INFO);

    stSaveParam.pImageBuffer = pDataForSaveImage;
    stSaveParam.nJpgQuality = 80;
    LOG(INFO);

    /* Output image format is JPEG.  Compress the uncompressed image. */
    unsigned char *jpegBuf = NULL;  /* Dynamically allocate the JPEG buffer */
    unsigned long jpegSize = 0;
    const int outQual = 80;
    tjhandle tjInstance = tjInitCompress();
    if(NULL == tjInstance) {
      std::cout << ("initializing compressor\n");
      exit(-1);
    }
    LOG(INFO);
    unsigned char *imgBuf = pDataForRGB;
    int width = pFrameInfo->nWidth;
    int height = pFrameInfo->nHeight;
    const int pixelFormat = TJPF_RGB;
    const int outSubsamp = TJSAMP_444;
    const int flags = 0;
    LOG(INFO);
    if(tjCompress2(tjInstance, imgBuf, width, 0, height, pixelFormat, &jpegBuf, &jpegSize, outSubsamp, outQual, flags) < 0) {
      std::cout << ("compressing image\n");
      exit(-1);
    }
    LOG(INFO);
    tjDestroy(tjInstance);
    LOG(INFO);
    tjInstance = NULL;

    const std::string imageName(std::to_string(pFrameInfo->nFrameNum) + ".jpg");
    FILE *jpegFile = fopen(imageName.c_str(), "wb");
    if(NULL == jpegFile) {
        std::cout << ("opening output file\n");
        exit(-1);
    }
    if(fwrite(jpegBuf, jpegSize, 1, jpegFile) < 1) {
        std::cout << ("writing output file\n");
        exit(-1);
    }
    tjDestroy(tjInstance);  tjInstance = NULL;
    fclose(jpegFile);  jpegFile = NULL;
    tjFree(jpegBuf);  jpegBuf = NULL;

    FILE* fp = fopen(imageName.c_str(), "wb");
    if (NULL == fp)
    {
        printf("fopen failed\n");
        exit;
    }

    LOG(INFO);

    nRet = MV_CC_SaveImageEx(&stSaveParam);
    LOG(INFO);

    if(MV_OK != nRet)
    {
        printf("failed in MV_CC_SaveImage, nRet[%x]\n", nRet);
        exit(1);
    }

    const std::string image2Name(std::to_string(pFrameInfo->nFrameNum) + ".png");
    fp = fopen(image2Name.c_str(), "wb");
    if (NULL == fp)
    {
        printf("fopen failed\n");
        exit;
    }
    fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, fp);
    free(pDataForSaveImage);
    pDataForSaveImage = NULL;
    LOG(INFO);
    fclose(fp);
    LOG(INFO);
    printf("save image succeed.\n");

    clock_gettime(CLOCK_REALTIME, &time_sys_end);
    double time_diff = time_sys_end.tv_sec + time_sys_end.tv_nsec / 1000000000.0 - (time_sys_start.tv_sec + time_sys_start.tv_nsec / 1000000000.0);
    printf("Cost time: %6f\n", time_diff);
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
