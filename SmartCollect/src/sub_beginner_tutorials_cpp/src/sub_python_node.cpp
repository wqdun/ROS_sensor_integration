#include "sub_python.h"

int main(int argc, char **argv) {
    // server node glog path:
    // FLAGS_log_dir = "/opt/smartc/log";
    // google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "sub_python_node");

    PythonSubscriber pythonSubscriber;
    pythonSubscriber.run();

    return 0;
}