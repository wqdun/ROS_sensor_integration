#ifndef __HIK_CAMERA_MANAGER_H__
#define __HIK_CAMERA_MANAGER_H__

#include <vector>
#include <assert.h>
#include "include/MvCameraControl.h"
#include "../../sc_lib_public_tools/src/thread_pool.h"
#include "save_image_task.h"
#include "single_camera.h"

class HikCameraManager {
public:
    HikCameraManager();
    ~HikCameraManager();
    void Run();

    threadpool<SaveImageTask> threadPool_;


private:
    std::vector<boost::shared_ptr<SingleCamera>> pSingleCameras_;
    MV_CC_DEVICE_INFO_LIST deviceList_;

    void DoClean();
    void PressEnterToExit();
};

#endif

