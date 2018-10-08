#include <unistd.h>
#include <stdlib.h>

#include "MvCameraControl.h"


#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>
// #define __USE_HIK_API_SAVING_JPG__

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
        LOG(ERROR) << "The Pointer of pstMVDevInfo is NULL!";
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        LOG(INFO) << "Device Model Name: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName;
        LOG(INFO) << "CurrentIp: " << nIp1 << nIp2 << nIp3<< nIp4;
        LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        LOG(INFO) << "Device Model Name: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName;
        LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
    }
    else
    {
        LOG(ERROR) << "Not support.";
    }

    return true;
}


void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if(!pFrameInfo) {
        LOG(ERROR) << "pFrameInfo is NULL.";
        return;
    }
    LOG(INFO) << "GetOneFrame[" << pFrameInfo->nFrameNum << "]: " << pFrameInfo->nWidth << " * " << pFrameInfo->nHeight;
    int nRet = MV_OK;

}

int main()
{
    int nRet = MV_OK;

    void* handle = NULL;
    do
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_EnumDevices fail! nRet: " << nRet;
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                LOG(INFO) << "device: " << i;
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
            LOG(WARNING) << "Find No Devices!";
            break;
        }

        printf("Please Intput camera index:");
        unsigned int nIndex = 0;
        scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            LOG(ERROR) << "Intput error!";
            break;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_CreateHandle fail! nRet: " << nRet;
            break;
        }

        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_OpenDevice fail! nRet: " << nRet;
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    LOG(ERROR) << "Warning: Set Packet Size fail nRet: " << nRet;
                }
            }
            else
            {
                LOG(ERROR) << "Warning: Get Packet Size fail nRet: " << nPacketSize;
            }
        }

        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_SetTriggerMode fail! nRet: " << nRet;
            break;
        }

        // 注册抓图回调
        // register image callback
        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_RegisterImageCallBackEx fail! nRet: " << nRet;
            break;
        }

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_StartGrabbing fail! nRet: " << nRet;
            break;
        }

        PressEnterToExit();

        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_StopGrabbing fail! nRet: " << nRet;
            break;
        }

        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_CloseDevice fail! nRet: " << nRet;
            break;
        }

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            LOG(ERROR) << "MV_CC_DestroyHandle fail! nRet: " << nRet;
            break;
        }
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    LOG(INFO) << "exiting...";

    return 0;
}
