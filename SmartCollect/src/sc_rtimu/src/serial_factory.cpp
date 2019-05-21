#include "serial_factory.h"
#include "serial_a1.h"
#include "serial_5651.h"

BaseSerial *SerialFactory::CreateSerial(const std::string &serialType) {
    if ("a1" == serialType) {
        return new SerialA1();
    }

    if ("5651" == serialType) {
        return new Serial5651();
    }

    LOG(ERROR) << "Unsupported type: " << serialType;
    exit(1);
}