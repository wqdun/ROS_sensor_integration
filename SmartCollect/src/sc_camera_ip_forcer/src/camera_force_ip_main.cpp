#include "camera_force_ip.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int lock_file(int fd) {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_start = 0;
    fl.l_whence = SEEK_SET;
    fl.l_len = 0;
    return (fcntl(fd, F_SETLK, &fl));
}

int main(int argc, char **argv) {
    FLAGS_log_dir = "/opt/smartc/log/";
    google::InitGoogleLogging(argv[0]);

    char buf[16];
    const std::string PID_FILE("/var/run/sc_camera_ip_forcer_node.pid");
    int fd = open(PID_FILE.c_str(), O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        perror("open");
        LOG(ERROR) << "Failed to open " << PID_FILE;
        return -1;
    }
    if (lock_file(fd) < 0) {
        if (errno == EACCES || errno == EAGAIN) {
            close(fd);
            LOG(WARNING) << "Already running.";
            return -1;
        }
        LOG(WARNING) << "Failed to lock " << PID_FILE << "; error: " << strerror(errno);
    }

    ftruncate(fd, 0);
    sprintf(buf, "%ld", (long)getpid());
    write(fd, buf, strlen(buf) + 1);

    CameraForceIp cameraForceIper;

    const size_t TRY_TIMES = 3;
    const int CAMERA_NUM = 6;
    int cameraNum = -1;
    for(size_t i = 0; i < TRY_TIMES; ++i) {
        if(CAMERA_NUM == (cameraNum = cameraForceIper.getCamerasInfo() ) ) {
            LOG(INFO) << "i: " << i << "; got " << cameraNum << " cameras successfully.";
            return cameraNum;
        }

        cameraNum = cameraForceIper.doForceIp();
        if(cameraNum != CAMERA_NUM) {
            LOG(WARNING) << "i: " << i << "; got " << cameraNum << " cameras.";
            continue;
        }

        LOG(INFO) << "i: " << i << "; got " << cameraNum << " cameras successfully.";
        return cameraNum;
    }

    LOG(ERROR) << "Got only " << cameraNum << " cameras.";
    return cameraNum;
}

