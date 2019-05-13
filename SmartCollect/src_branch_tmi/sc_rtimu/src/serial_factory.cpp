#include "serial_factory.h"
#include "base_serial.h"
#include "serial_a1.h"

BaseSerial *SerialFactory::CreateSerial(const std::string &serialType) {
    if("a1" == serialType) {
        // BaseSerial *pSerial = ;
        return new SerialA1();
    }

    // if("5651" == serialType) {
    //     return new Serial5651();
    // }

    return NULL;
}