#ifndef __SERIAL_FACTORY_H__
#define __SERIAL_FACTORY_H__

#include <string>

class BaseSerial;
class SerialFactory {
public:
    BaseSerial *CreateSerial(const std::string &serialType);
};


#endif
