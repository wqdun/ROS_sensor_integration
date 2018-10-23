#ifndef __CAN_PARSER_H__
#define __CAN_PARSER_H__

#include <stdlib.h>
#include "controlcan.h"

class CanParser {
public:
    CanParser();
    ~CanParser();
    void Run();
    void Receiver();
    void StopDevice();

    double decimalResult_;
    bool isCanParserRunning_;


private:
    VCI_BOARD_INFO vciInfo_;
};


#endif
