#ifndef __PTGREY_CAMERA_MANAGER_H__
#define __PTGREY_CAMERA_MANAGER_H__

#include <sstream>
#include <glog/logging.h>

#include "FlyCapture2.h"
#include "../../sc_lib_public_tools/src/thread_pool.h"

class PtgreySaveImageTask;
class SinglePtgreyCamera;
class PtgreyCameraManager {
public:
    static void LogErrorTrace(FlyCapture2::Error error);
    PtgreyCameraManager(const std::string &_rawPath);
    ~PtgreyCameraManager();
    void Run();

    threadpool<PtgreySaveImageTask> threadPool_;
    std::string rawDataPath_;
    bool isSaveImg_;


private:
    void LogBuildInfo();
    void GetCameras(unsigned int _cameraNum, const std::string &__rawdataDir);
    void RunAllCameras();
    void PressEnterToExit();

    std::vector<SinglePtgreyCamera *> pSinglePtgreyCameras_;
};

#endif

