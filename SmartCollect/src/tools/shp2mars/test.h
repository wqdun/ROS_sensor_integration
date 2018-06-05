#ifndef ___TEST_H__
#define ___TEST_H__

#include "ogrsf_frmts.h"

class Test {
public:
    Test();
    ~Test();
    void run();


private:
    GDALDataset *poDS_;
    OGRLayer *poLayer_;
};

#endif

