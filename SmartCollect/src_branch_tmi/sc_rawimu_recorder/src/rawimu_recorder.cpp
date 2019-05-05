#include "rawimu_recorder.h"

RawimuRecorder::RawimuRecorder(const std::string &_serialName, const std::string &_imuPath) {
    LOG(INFO) << __FUNCTION__ << " start.";
    serialName_ = _serialName;
    imuPath_ = _imuPath;
}

RawimuRecorder::~RawimuRecorder() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

int RawimuRecorder::Run(int baudrate) {
    LOG(INFO) << __FUNCTION__ << " start.";

    fd_ = open(serialName_.c_str(), O_RDWR);
    if(fd_ < 0) {
        LOG(ERROR) << "Failed to open " << serialName_ << ", try change permission...";
        return -1;
    }

    if(public_tools::ToolsNoRos::SetSerialOption(fd_, baudrate, 8, 'N', 1) < 0) {
        LOG(ERROR) << "Failed to setup " << ttyname(fd_);
        return -1;
    }

    std::string imuFileNamePrefix("");
    (void)public_tools::PublicTools::generateFileName(imuPath_, imuFileNamePrefix);
    const std::string rawInsFile(imuPath_ + imuFileNamePrefix + "_integrate_imu.dat");
    (void)Record2File(rawInsFile);

    close(fd_);
    return 0;
}

void RawimuRecorder::Record2File(const std::string &rawInsFile) {
    LOG(INFO) << __FUNCTION__ << " start.";
    FILE *pOutFile;
    if(!(pOutFile = fopen(rawInsFile.c_str(), "wb") ) ) {
        LOG(ERROR) << "Failed to create: " << rawInsFile;
        return;
    }
    LOG(INFO) << "Create "<< rawInsFile << " successfully.";

    const size_t BUFFER_SIZE = 1000;
    unsigned char buf[BUFFER_SIZE];
    while(ros::ok() ) {
        bzero(buf, BUFFER_SIZE);
        int nread = read(fd_, buf, BUFFER_SIZE);
        LOG_EVERY_N(INFO, 50) << "nread: " << nread;
        if(nread <= 0) {
            continue;
        }

#ifndef NDEBUG
        for(size_t i = 0; i < nread; ++i) {
            LOG(INFO) << std::hex << (int)buf[i];
        }
#endif

        (void)fwrite(buf, nread, 1, pOutFile);
    }

    fclose(pOutFile);
    return;
}
