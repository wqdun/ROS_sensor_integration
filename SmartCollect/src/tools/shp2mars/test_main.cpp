#include "test.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    // FLAGS_log_dir = "/opt/smartc/log";
    // google::InitGoogleLogging(argv[0]);

    Test testor;
    testor.run();
    return 0;
}