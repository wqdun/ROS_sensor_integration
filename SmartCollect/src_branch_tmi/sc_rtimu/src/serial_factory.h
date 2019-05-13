#include <string>

#include "base_serial.h"

class SerialFactory {
public:
    BaseSerial *CreateSerial(const std::string &serialType);
};