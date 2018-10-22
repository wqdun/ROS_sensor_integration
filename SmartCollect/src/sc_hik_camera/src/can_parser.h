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


private:
    double decimalResult_;
    VCI_BOARD_INFO vciInfo_;
    int isRunning;
};


#endif
