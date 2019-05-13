#include <glog/logging.h>

#include "base_serial.h"

class SerialA1: public BaseSerial {
public:
    SerialA1();

    void Run();
};

