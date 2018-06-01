#include "to_mars.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    ToMars toMarsor();
    baseMapper.convert();

    return 0;
}