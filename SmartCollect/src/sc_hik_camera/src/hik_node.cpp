#include "TlFactory.h"
#include "CameraParams.h"
using namespace MvCamCtrl;

void main()
{
    int nRet = -1;

    // 获取工厂单件
    CTlFactory& tlFactory = CTlFactory::GetInstance();

    // 调用实例里面的接口
    unsigned int nSupportedTls = tlFactory.EnumerateTls();
    if (MV_GIGE_DEVICE == (nSupportedTls & MV_GIGE_DEVICE))
    {
        //支持GigE Vision传输协议，后续对该协议设备进行操作
        // 枚举子网内所有GigE设备，返回设备信息列表
        MV_CC_DEVICE_INFO_LIST stDevList;
        memset(&stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        nRet = tlFactory.EnumDevices(MV_GIGE_DEVICE, stDevList);
        if (MV_OK != nRet)
        {
            printf("error: EnumDevices [%x]\n", nRet);
            return;
        }

        int i = 0;
        if (stDevList.nDeviceNum == 0)
        {
            printf("no camera found\n");
            Sleep(100000000);
        }
        else
        {
            for (int n = 0; n < stDevList.nDeviceNum; n++)
            {
                printf("%d . %s\n", n, stDevList.pDeviceInfo[n]->SpecialInfo.stGigEInfo.chModelName);
            }
            printf("select one：");
            scanf("%d", &i);
            if (i < 0 || i >= stDevList.nDeviceNum)
            {
                printf("error : input\n");
                Sleep(10000000);
            }
        }

        // 选择查找到的第一台在线设备判断是否可以访问
        if(!tlFactory.IsDeviceAccessible(*(stDevList.pDeviceInfo[i])))
        {
            printf("error: IsDeviceAccessible\n");
            break;
        }

        // 选择查找到的第一台在线设备创建设备实例
        int i=0;
        IMvDevice* MyDevice = tlFactory.CreateDevice(*(stDevList.pDeviceInfo[i]));

        if (NULL == MyDevice)
        {
            printf("error: CreateDevice\n");
            break;
        }

        //...设备其他操作处理

        // 从工厂中销毁设备
        nRet = tlFactory.DestroyDevice(MyDevice);
        if (MV_OK != nRet)
        {
            printf("error: DestroyDevice fail [%x]\n", nRet);
        }
    }
}

