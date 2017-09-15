#ifndef _GET_SERIAL_H
#define _GET_SERIAL_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <error.h>

extern std::string frame2pub;

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
int open_serial(void);
int read_serial(int fd);

#endif
